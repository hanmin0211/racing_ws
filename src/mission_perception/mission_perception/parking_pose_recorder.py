#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parking_pose_recorder.py — 주차 완료 자세 하나만 찍는다 (x, y, yaw).

teleop 으로 진입 궤적을 통째로 기록하는 대신, **차를 주차칸 안에 손으로 정확히
넣어두고 그 자세만** 찍는다. 진입 궤적은 tools/make_parking_path.py 가
기하학적으로 만든다(최소회전반경을 지키므로 실현 가능성이 설계상 보장된다).

  ros2 run mission_perception parking_pose_recorder --ros-args -p slot:=1
  → ~/parking_pose_1.yaml
  다음: python3 tools/make_parking_path.py --slot 1

★ 왜 이 방식이 나은가
  teleop 으로 좁은 칸에 후진 진입하는 건 사람 운전 실력에 달렸고, 급하게 꺾인
  궤적이 기록되면 재생할 때 못 따라간다(최소회전반경 2.42m). 차를 손으로 놓으면
  최종 자세는 정확하고, 경로는 계산이 보장한다.

★ 헤딩 캘리브가 반드시 끝나 있어야 한다
  stop_point_recorder 와 달리 이건 **yaw 를 쓴다.** 캘리브 전 yaw 는 IMU 원시값이라
  방향 의미가 없고, 그대로 찍으면 주차 방향이 통째로 틀린다.
  IMU yaw 는 세션마다 리셋되므로 매 세션 캘리브가 필요하다.

★ 차를 놓는 자세 그대로 찍힌다
  주차칸 안에, 최종적으로 서 있길 원하는 위치·방향 그대로 놓고 엔터.
  (기록되는 건 GPS 안테나 위치다 — 재생 때도 안테나가 이 점에 오도록 선다)
"""

import math
import os
import statistics
import sys
import threading

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


class ParkingPoseRecorder(Node):

  def __init__(self):
    super().__init__('parking_pose_recorder')
    self.declare_parameter('slot', 1)
    self.declare_parameter('output', '')
    self.declare_parameter('odom_topic', '/odometry/filtered')
    # 정지 상태에서 여러 샘플을 평균낸다 — GPS 지터를 줄인다.
    self.declare_parameter('samples', 40)
    self.declare_parameter('require_heading_calib', True)

    self.slot = int(self.get_parameter('slot').value)
    self.output = (str(self.get_parameter('output').value)
                   or os.path.expanduser(f'~/parking_pose_{self.slot}.yaml'))
    self.n_samples = int(self.get_parameter('samples').value)
    self.require_calib = bool(
        self.get_parameter('require_heading_calib').value)
    self.heading_ready = not self.require_calib

    self.buf = []          # 최근 (x, y, yaw)
    self.lock = threading.Lock()

    latched = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Float64, '/heading/yaw_offset',
                             self.heading_cb, latched)
    self.create_subscription(Odometry,
                             str(self.get_parameter('odom_topic').value),
                             self.odom_cb, 10)
    self.create_timer(1.0, self.status)

    self.get_logger().info(
        f'주차 자세 기록 — 자리 {self.slot} → {self.output}')
    if not self.heading_ready:
      self.get_logger().error(
          '⛔ 헤딩 캘리브 대기 — 10m 직진 캘리브를 먼저 끝낼 것. '
          '캘리브 전 yaw 로 찍으면 주차 방향이 틀린다.')

    threading.Thread(target=self.key_loop, daemon=True).start()

  def heading_cb(self, msg):
    if not self.heading_ready:
      self.heading_ready = True
      self.get_logger().info(
          f'✅ 헤딩 캘리브 확인 ({math.degrees(float(msg.data)):.1f}°)')

  def odom_cb(self, msg: Odometry):
    p = msg.pose.pose
    with self.lock:
      self.buf.append((p.position.x, p.position.y,
                       yaw_from_quat(p.orientation)))
      if len(self.buf) > self.n_samples:
        self.buf.pop(0)

  def status(self):
    with self.lock:
      n, b = len(self.buf), list(self.buf)
    if not self.heading_ready:
      self.get_logger().error('⛔ 헤딩 캘리브 전 — 아직 찍을 수 없다')
      return
    if n == 0:
      self.get_logger().warn('측위 미수신 — /odometry/filtered 확인')
      return
    x, y, yaw = b[-1]
    # 정지해 있는지: 최근 샘플의 위치 분산
    spread = max(math.hypot(p[0] - x, p[1] - y) for p in b) if n > 1 else 0.0
    self.get_logger().info(
        f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):+.1f}°  '
        f'흔들림 {spread*100:.0f}cm  샘플 {n}/{self.n_samples}  — 엔터로 기록')

  def key_loop(self):
    for _ in sys.stdin:
      if not self.heading_ready:
        print('  ⛔ 헤딩 캘리브 전 — 기록 안 함')
        continue
      with self.lock:
        b = list(self.buf)
      if len(b) < 5:
        print('  ⚠ 샘플 부족 — 측위가 오는지 확인')
        continue
      x = statistics.fmean(p[0] for p in b)
      y = statistics.fmean(p[1] for p in b)
      # 각도는 산술평균하면 안 된다(±180° 경계). 벡터로 평균낸다.
      cs = statistics.fmean(math.cos(p[2]) for p in b)
      sn = statistics.fmean(math.sin(p[2]) for p in b)
      yaw = math.atan2(sn, cs)
      spread = max(math.hypot(p[0] - x, p[1] - y) for p in b)
      if spread > 0.10:
        print(f'  ⚠ 차가 흔들리고 있다({spread*100:.0f}cm) — '
              f'멈춘 뒤 다시 찍는 걸 권한다. 일단 기록은 한다.')
      self.save(x, y, yaw, len(b), spread)

  def save(self, x, y, yaw, n, spread):
    from waypoint_follower.site_origin import load_site_origin
    epsg, ox, oy, site = load_site_origin(self.get_logger())
    data = {
        'origin': {'x': ox, 'y': oy, 'epsg': epsg, 'site': site},
        'slot': self.slot,
        'pose': {'x': round(float(x), 3), 'y': round(float(y), 3),
                 'yaw_deg': round(math.degrees(yaw), 2)},
        'quality': {'samples': n, 'spread_m': round(spread, 3)},
    }
    with open(self.output, 'w', encoding='utf-8') as f:
      yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f'  ✅ 자리 {self.slot} 기록: x={x:.2f} y={y:.2f} '
          f'yaw={math.degrees(yaw):+.1f}°  → {self.output}', flush=True)
    print(f'     다음: python3 tools/make_parking_path.py --slot {self.slot}',
          flush=True)


def main(args=None):
  rclpy.init(args=args)
  node = ParkingPoseRecorder()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

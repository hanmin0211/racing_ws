#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stop_point_recorder.py — 대회장에서 정지 지점을 찍는 도구

차를 정지선에 세워두고 **엔터만 누르면** 현재 위치가 기록된다. 다 찍고
Ctrl-C 하면 traffic_light_bridge 에 그대로 붙여넣을 수 있는 문자열이 나온다.

    ros2 run mission_perception stop_point_recorder

★ 헤딩 캘리브가 필요 없다
  기록에는 위치(/fix)만 쓴다. 10m 직진 캘리브는 주행할 때 필요한 것이지
  좌표를 찍는 데는 필요 없다. 대회장에서 준비 시간을 아끼는 지점이다.

★ 원점을 반드시 맞출 것
  origin_x/y 는 direct_localization·waypoint_recorder 와 **같은 값**이어야
  한다. 다르면 좌표계가 어긋나 정지 지점이 엉뚱한 데로 간다.
  기본값은 기존 파이프라인과 동일(399848.522 / 4092209.171, EPSG:32652).

★ 안테나 위치로 기록된다 (중요)
  기록되는 건 GPS 안테나의 위치다. 그리고 주행 중엔 안테나가 이 점에서
  0.3m 안에 들어오면 정지한다(longitudinal_controller 의 하드정지 임계값).
  따라서 **"차를 세우고 싶은 자세 그대로 세운 뒤 찍는 것"** 이 가장 정확하다.
  정지선 앞에 범퍼를 맞춰 세우고 찍으면, 주행 때도 같은 자세로 선다.

파라미터:
  fix_topic   : GPS 토픽 (기본 /fix)
  utm_epsg    : 기본 32652 (UTM 52N)
  origin_x/y  : 로컬 원점 — 다른 노드와 반드시 동일하게
  output_file : 저장 경로 (기본 ~/stop_points.yaml)
"""

import os
import sys
import threading

import rclpy
import yaml
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from waypoint_follower.site_origin import declare_and_get


class StopPointRecorder(Node):

  def __init__(self):
    super().__init__('stop_point_recorder')

    self.declare_parameter('fix_topic', '/fix')
    # 원점은 config/site_origin.yaml 이 정본 — waypoint_recorder 와 반드시 동일.
    epsg, self.origin_x, self.origin_y = declare_and_get(self)
    self.declare_parameter('output_file',
                           os.path.expanduser('~/stop_points.yaml'))

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.out = str(g('output_file'))
    self.tf = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)

    self.fix = None          # (lat, lon, x, y, h_acc)
    self.points = []
    self.lock = threading.Lock()

    self.create_subscription(NavSatFix, str(g('fix_topic')), self.fix_cb,
                             qos_profile_sensor_data)
    self.create_timer(1.0, self.status)

    self.get_logger().info(
        f'정지지점 기록 시작 (원점 {self.origin_x:.0f},{self.origin_y:.0f}). '
        f'차를 세우고 **엔터**를 누르면 기록된다. 끝나면 Ctrl-C.')

    threading.Thread(target=self.key_loop, daemon=True).start()

  def fix_cb(self, msg: NavSatFix):
    if msg.latitude != msg.latitude or abs(msg.latitude) < 1e-9:
      return                                    # NaN 또는 미수신
    ux, uy = self.tf.transform(msg.longitude, msg.latitude)
    # covariance[0] 은 동서방향 분산[m^2] — 대략적인 수평 정확도로 쓴다.
    # ★ position_covariance 는 numpy 배열이라 `if msg.position_covariance` 로
    # 검사하면 ValueError(ambiguous truth value)가 난다. 길이로 판정할 것.
    cov = msg.position_covariance
    h_acc = float(cov[0]) ** 0.5 if len(cov) > 0 and cov[0] > 0 else -1.0
    with self.lock:
      self.fix = (msg.latitude, msg.longitude,
                  ux - self.origin_x, uy - self.origin_y, h_acc)

  def status(self):
    with self.lock:
      f = self.fix
    if f is None:
      self.get_logger().warn('GPS 미수신 — /fix 가 오는지 확인할 것')
      return
    acc = f'{f[4]*100:.1f}cm' if f[4] >= 0 else '?'
    self.get_logger().info(
        f'현재 x={f[2]:.2f} y={f[3]:.2f}  (정확도 {acc})  '
        f'기록됨 {len(self.points)}개  — 엔터로 기록')

  def key_loop(self):
    for _ in sys.stdin:                          # 엔터마다 한 번
      with self.lock:
        f = self.fix
      if f is None:
        print('  ⚠ GPS 미수신 — 기록 안 됨')
        continue
      if f[4] >= 0 and f[4] > 0.10:
        # RTK Fixed 면 보통 몇 cm 다. 10cm 를 넘으면 Float/단독측위 의심.
        print(f'  ⚠ 정확도 {f[4]*100:.0f}cm — RTK Fixed 가 아닐 수 있다. '
              f'그래도 기록은 한다.')
      with self.lock:
        self.points.append((f[2], f[3]))
      print(f'  ✅ {len(self.points)}번 기록: x={f[2]:.2f} y={f[3]:.2f} '
            f'(lat {f[0]:.7f}, lon {f[1]:.7f})')

  def save(self):
    if not self.points:
      print('\n기록된 점이 없다.')
      return
    flat = []
    for (x, y) in self.points:
      flat += [round(x, 3), round(y, 3)]
    with open(self.out, 'w') as fp:
      yaml.safe_dump({'origin_x': self.origin_x, 'origin_y': self.origin_y,
                      'stop_points': [{'x': round(x, 3), 'y': round(y, 3)}
                                      for (x, y) in self.points]},
                     fp, allow_unicode=True, sort_keys=False)
    arg = '[' + ', '.join(f'{v}' for v in flat) + ']'
    print(f'\n저장: {self.out}  ({len(self.points)}개)')
    print('\n아래를 그대로 쓰면 된다:\n')
    print(f'  ros2 run mission_perception traffic_light_bridge \\')
    print(f'      --ros-args -p "stop_points:={arg}"\n')


def main(args=None):
  rclpy.init(args=args)
  node = StopPointRecorder()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  except rclpy.executors.ExternalShutdownException:
    # SIGTERM(kill, timeout 등)으로 종료된 경우. 현장에서 트레이스백이
    # 쏟아지면 기록이 날아간 줄 알기 쉬우므로 조용히 저장만 하고 끝낸다.
    pass
  finally:
    node.save()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

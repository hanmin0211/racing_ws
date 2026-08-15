#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tracking_monitor_node.py — 경로 추종 품질을 숫자로 본다.

"경로를 잘 못 따라가는 것 같다"를 **횡방향 오차(cross-track error)** 로 정량화한다.
튜닝은 눈대중으로 하면 뭘 바꿔서 좋아졌는지 알 수 없다.

발행:
  /cross_track_error (Float64) 경로 중심선까지의 부호 있는 거리[m]
                               (+ = 경로 왼쪽으로 벗어남, − = 오른쪽)
로그:
  1초마다 현재 오차, 주행 구간 평균/최대, 속도, 조향각을 출력하고
  Ctrl-C 시 전체 통계를 요약한다.

사용:
  ros2 run waypoint_follower tracking_monitor
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


class TrackingMonitor(Node):

  def __init__(self):
    super().__init__('tracking_monitor')
    self.wps = np.empty((0, 2))
    self.errs = []
    self.speeds = []
    self.steers = []
    self.cur_speed = 0.0
    self.cur_steer = 0.0

    self.pub = self.create_publisher(Float64, '/cross_track_error', 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Path, '/global_path', self.path_cb, qos)
    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
    self.create_subscription(Float64, '/current_speed', self.spd_cb, 10)
    self.create_subscription(Float64, '/steering_cmd', self.str_cb, 10)
    self.create_timer(1.0, self.report)
    self.get_logger().info('경로 추종 모니터 시작 — 횡방향 오차를 1초마다 표시')

  def path_cb(self, msg):
    self.wps = np.array([[p.pose.position.x, p.pose.position.y]
                         for p in msg.poses]) if msg.poses else np.empty((0, 2))

  def spd_cb(self, m):
    self.cur_speed = abs(float(m.data))

  def str_cb(self, m):
    self.cur_steer = float(m.data)

  def odom_cb(self, msg):
    if len(self.wps) < 2:
      return
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    d = np.hypot(self.wps[:, 0] - x, self.wps[:, 1] - y)
    i = int(np.argmin(d))
    # 부호: 경로 진행방향 기준 왼쪽(+)/오른쪽(−)
    j = min(i + 1, len(self.wps) - 1)
    pdir = math.atan2(self.wps[j, 1] - self.wps[i, 1],
                      self.wps[j, 0] - self.wps[i, 0])
    dx, dy = self.wps[i, 0] - x, self.wps[i, 1] - y
    lateral = -(-dx * math.sin(pdir) + dy * math.cos(pdir))
    self.errs.append(abs(lateral))
    self.speeds.append(self.cur_speed)
    self.steers.append(abs(self.cur_steer))
    self.pub.publish(Float64(data=float(lateral)))
    self.last = (lateral, i, yaw, pdir)

  def report(self):
    if not self.errs:
      self.get_logger().info('대기중 — /global_path 와 /odometry/filtered 필요')
      return
    lat, i, yaw, pdir = self.last
    n = min(len(self.errs), 30)
    recent = self.errs[-n:]
    self.get_logger().info(
        f'횡오차 {lat:+.2f}m (최근평균 {sum(recent)/len(recent):.2f}) | '
        f'속도 {self.cur_speed:.2f}m/s | 조향 {self.cur_steer:+.1f}° | '
        f'경로 {i}번')

  def summary(self):
    print('\n' + '=' * 58)
    if not self.errs:
      print('데이터 없음')
      print('=' * 58)
      return
    e = np.array(self.errs)
    s = np.array(self.speeds)
    st = np.array(self.steers)
    print('경로 추종 결과')
    print('-' * 58)
    print(f'  샘플 {len(e)}개')
    print(f'  횡방향 오차  평균 {e.mean():.2f} m | 최대 {e.max():.2f} m | '
          f'95% {np.percentile(e, 95):.2f} m')
    print(f'  속도         평균 {s.mean():.2f} m/s | 최대 {s.max():.2f} m/s')
    print(f'  조향각(절대) 평균 {st.mean():.1f}° | 최대 {st.max():.1f}°')
    print('-' * 58)
    if e.mean() < 0.3:
      print('  ✅ 추종 양호 (평균 오차 30cm 이내)')
    elif e.mean() < 0.7:
      print('  ⚠ 추종 보통 — lookahead/속도 튜닝 여지 있음')
    else:
      print('  ❌ 추종 불량 — 속도가 너무 빠르거나 lookahead 부적절')
    if st.max() > 17:
      print('  ⚠ 조향이 한계(18°)에 닿음 — 코너가 차량 능력보다 급하거나')
      print('    진입 속도가 높아 경로를 벗어난 뒤 크게 복귀 중일 수 있다')
    print('=' * 58)


def main(args=None):
  rclpy.init(args=args)
  n = TrackingMonitor()
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.summary()
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

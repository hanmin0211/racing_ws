#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lap_timer.py — 자율주행 랩 시간·거리·평균속도 자동 측정.

'모드: AUTO 진입' 부터 '완주 신호' 까지를 잰다. 매번 터미널에서 타임스탬프를
빼지 않아도 된다. 별도 터미널에서 켜두면 랩이 끝날 때 요약을 찍는다.

  python3 tools/lap_timer.py

측정:
  · 랩 시간      : 실제 주행 시작 → 완주
  · 주행 거리    : /odometry/filtered 위치 적분
  · 평균/최고 속도

기준:
  AUTO 시작 = /cmd_vel 의 linear.x 가 0 → 양수로 바뀌는 순간
  완주       = /goal_reached == True
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class LapTimer(Node):

  def __init__(self):
    super().__init__('lap_timer')
    self.t_start = None
    self.dist = 0.0
    self.last_xy = None
    self.v_max = 0.0
    self.v_sum = 0.0
    self.v_n = 0
    self.done = False

    self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
    self.create_subscription(Bool, '/goal_reached', self.goal_cb, 10)
    self.get_logger().info('랩 타이머 대기 — 차가 출발하면 자동으로 잰다.')

  def cmd_cb(self, msg):
    if self.t_start is None and msg.linear.x > 0.05:
      self.t_start = time.time()
      self.get_logger().info('▶ 랩 시작 (출발 감지)')

  def odom_cb(self, msg):
    if self.t_start is None or self.done:
      return
    xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    if self.last_xy is not None:
      d = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
      if d > 0.005:
        self.dist += d
    self.last_xy = xy
    v = abs(msg.twist.twist.linear.x)
    self.v_max = max(self.v_max, v)
    self.v_sum += v
    self.v_n += 1

  def goal_cb(self, msg):
    if bool(msg.data) and self.t_start and not self.done:
      self.done = True
      dt = time.time() - self.t_start
      avg = self.v_sum / self.v_n if self.v_n else 0.0
      m, s = divmod(int(dt), 60)
      print('\n' + '=' * 56)
      print('🏁 랩 완주')
      print('-' * 56)
      print(f'  시간      : {m}분 {s}초  ({dt:.0f}s)')
      print(f'  주행거리  : {self.dist:.1f} m')
      print(f'  평균속도  : {avg:.2f} m/s')
      print(f'  최고속도  : {self.v_max:.2f} m/s')
      print('=' * 56)
      print('  (다음 랩을 재려면 이 타이머를 다시 실행)')


def main():
  rclpy.init()
  n = LapTimer()
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

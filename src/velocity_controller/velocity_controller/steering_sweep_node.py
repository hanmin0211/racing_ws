#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
steering_sweep_node.py — 조향 저속 스무스니스 벤치 평가용 삼각파 스윕.

속도는 0으로 두고 조향각만 천천히 연속 왕복(삼각파)시킨다. 자율주행의 연속
조향 입력을 흉내내므로 저속 추종/스티션을 제대로 본다.
/cmd_vel(Twist): linear.x=0, angular.z=조향각[도] → serial_bridge → Arduino.

파라미터:
  amplitude   : 진폭[도] (기본 15, 실측 최대타각 20° 안쪽)
  half_period : 편도(0→+amp) 시간[s] (기본 6). 클수록 느린 스윕=스티션 검사 가혹.

실행:
  ros2 run velocity_controller steering_sweep
  ros2 run velocity_controller steering_sweep --ros-args -p amplitude:=18.0 -p half_period:=10.0
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SteeringSweep(Node):

  def __init__(self):
    super().__init__('steering_sweep')
    self.declare_parameter('amplitude', 15.0)
    self.declare_parameter('half_period', 6.0)
    self.amp = float(self.get_parameter('amplitude').value)
    self.half = max(0.5, float(self.get_parameter('half_period').value))
    self.rate_dps = self.amp / self.half     # 삼각파 기울기[도/초]

    self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.t0 = time.time()
    self.create_timer(0.05, self.tick)       # 20Hz
    self.get_logger().info(
        f'조향 스윕 시작: ±{self.amp:.0f}°, 편도 {self.half:.1f}s '
        f'(={self.rate_dps:.1f}°/s). 속도=0. Ctrl-C로 중앙 복귀.')

  def tick(self):
    t = (time.time() - self.t0) % (4.0 * self.half)
    if t < self.half:
      ang = self.rate_dps * t
    elif t < 3.0 * self.half:
      ang = self.amp - self.rate_dps * (t - self.half)
    else:
      ang = -self.amp + self.rate_dps * (t - 3.0 * self.half)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = float(ang)
    self.pub.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = SteeringSweep()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.pub.publish(Twist())
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

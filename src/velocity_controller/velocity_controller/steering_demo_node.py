#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
steering_demo_node.py — 실제 운전대 돌리듯 조향을 한 편의 시퀀스로 시연.

속도는 0으로 고정하고 조향각만 부드럽게(smoothstep 이징) 움직인다:
  중앙 → 왼쪽 코너 → 중앙 → 오른쪽 코너 → 중앙
      → 천천히 풀락 좌↔우(lock-to-lock) → 빠른 좌우 보정(위글) → 중앙 정지

키프레임 사이를 smoothstep으로 보간해 급격함 없이 '핸들 돌리는' 느낌을 낸다.
/cmd_vel(Twist): linear.x=0, angular.z=조향각[도] → serial_bridge → Arduino.

파라미터:
  amplitude : 데모 진폭[도] (기본 18, 실측 최대타각 20° 안쪽)
  loop      : 시퀀스 반복 여부 (기본 False, 1회 재생 후 중앙정지·종료)

실행:
  ros2 run velocity_controller steering_demo
  ros2 run velocity_controller steering_demo --ros-args -p amplitude:=15.0 -p loop:=true
  (serial_bridge 가 켜져 있어야 실제 모터로 나감. 조향축 공중 권장.)
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def smoothstep(a, b, t):
  t = 0.0 if t < 0 else (1.0 if t > 1 else t)
  s = t * t * (3.0 - 2.0 * t)
  return a + (b - a) * s


def build_keyframes(amp):
  """운전대 시나리오 키프레임 [(시각[s], 목표각[°]), ...]."""
  w = 0.43 * amp   # 위글 진폭(≈amp의 절반 조금 못미치게)
  return [
      (0.0,   0.0),
      (1.5,   0.0),    # 잠깐 중앙
      (3.5,   amp),    # 왼쪽 코너
      (5.5,   0.0),    # 복귀
      (7.5,  -amp),    # 오른쪽 코너
      (9.5,   0.0),    # 복귀
      (13.5,  amp),    # 천천히 왼쪽 풀락
      (17.5, -amp),    # 천천히 오른쪽 풀락 (lock-to-lock)
      (20.0,  0.0),    # 중앙
      (20.8,  w),      # 빠른 좌우 보정(위글)
      (21.6, -w),
      (22.4,  w),
      (23.2, -w),
      (24.0,  0.0),    # 중앙 정지
      (26.0,  0.0),
  ]


class SteeringDemo(Node):

  def __init__(self):
    super().__init__('steering_demo')
    self.declare_parameter('amplitude', 18.0)
    self.declare_parameter('loop', False)
    amp = float(self.get_parameter('amplitude').value)
    self.loop = bool(self.get_parameter('loop').value)

    self.kf = build_keyframes(amp)
    self.duration = self.kf[-1][0]
    self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.t0 = time.time()
    self.create_timer(0.05, self.tick)   # 20Hz
    self.get_logger().info(
        f'조향 데모 시작 ({self.duration:.0f}s, ±{amp:.0f}°, loop={self.loop}): '
        f'좌우 코너 → 풀락 → 위글 → 중앙. 속도=0.')

  def angle_at(self, elapsed):
    kf = self.kf
    if elapsed <= kf[0][0]:
      return kf[0][1]
    if elapsed >= kf[-1][0]:
      return kf[-1][1]
    for i in range(1, len(kf)):
      t0, a0 = kf[i - 1]
      t1, a1 = kf[i]
      if elapsed <= t1:
        return smoothstep(a0, a1, (elapsed - t0) / (t1 - t0))
    return kf[-1][1]

  def tick(self):
    el = time.time() - self.t0
    if self.loop:
      el = el % self.duration
    ang = self.angle_at(el)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = float(ang)
    self.pub.publish(msg)
    if not self.loop and (time.time() - self.t0) >= self.duration:
      self.get_logger().info('데모 완료 → 중앙 정지.')
      self.pub.publish(Twist())
      raise KeyboardInterrupt


def main(args=None):
  rclpy.init(args=args)
  node = SteeringDemo()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.pub.publish(Twist())   # 중앙 복귀
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

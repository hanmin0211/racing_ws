#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_cmd_mux_node.py — 종방향(속도) + 횡방향(조향)을 합쳐 최종 /cmd_vel 생성.

자율주행 명령과 수동 teleop 명령을 한 곳에서 조정(arbitration)하고, 마지막
안전 클램프를 건 뒤 딱 하나의 /cmd_vel 만 내보낸다. 이렇게 단일 출구를 두면
여러 노드가 /cmd_vel 을 동시에 쏴서 서로 덮어쓰는 사고를 막을 수 있다.

우선순위:
  1) E-stop(/e_stop = true)      → 무조건 정지 (조향은 직진)
  2) teleop(/teleop/cmd_vel)     → 최근 수신 중이면 자율 명령을 덮어씀(사람 우선)
  3) 자율(/target_speed + /steering_cmd)
  4) 입력 끊김(워치독)           → 정지

/cmd_vel 규약 (스택 전체 공통):
  linear.x  = 목표 속도 [m/s]   (음수 = 후진)
  angular.z = 조향각   [도]     (좌 +, 우 −)   ※ rad/s 아님!

입력: /target_speed(Float64), /steering_cmd(Float64,도),
      /teleop/cmd_vel(Twist), /e_stop(Bool)
출력: /cmd_vel(Twist)
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class VehicleCmdMux(Node):

  def __init__(self):
    super().__init__('vehicle_cmd_mux')

    # 최종 안전 클램프 — 스택 어디서 뭐가 오든 여기서 마지막으로 잘린다.
    # 조향 18°: 물리한계 20°지만 포텐셔미터 포화(ADC 0) 회피 마진.
    self.declare_parameter('max_steer_deg', 18.0)
    self.declare_parameter('max_speed', 1.0)      # 구동 벤치검증 후 상향
    self.declare_parameter('min_speed', -0.6)     # 후진 한계
    self.declare_parameter('input_timeout', 0.5)  # 자율 입력 끊김 판정
    self.declare_parameter('teleop_timeout', 0.5)  # 이 시간 지나면 teleop 해제
    self.declare_parameter('rate', 20.0)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.max_steer = float(g('max_steer_deg'))
    self.max_speed = float(g('max_speed'))
    self.min_speed = float(g('min_speed'))
    self.input_timeout = float(g('input_timeout'))
    self.teleop_timeout = float(g('teleop_timeout'))
    rate = float(g('rate'))

    self.target_speed = 0.0
    self.steer_deg = 0.0
    self.speed_time = None
    self.steer_time = None
    self.teleop = None
    self.teleop_time = None
    self.estop = False
    self.last_mode = None

    self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.create_subscription(Float64, '/target_speed', self.speed_cb, 10)
    self.create_subscription(Float64, '/steering_cmd', self.steer_cb, 10)
    self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_cb, 10)
    self.create_subscription(Bool, '/e_stop', self.estop_cb, 10)
    self.create_timer(1.0 / rate, self.tick)

    self.get_logger().info(
        f'명령 먹스 시작: 조향±{self.max_steer}° 속도 {self.min_speed}~'
        f'{self.max_speed}m/s (E-stop > teleop > 자율)')

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def speed_cb(self, msg):
    self.target_speed = float(msg.data)
    self.speed_time = self.now()

  def steer_cb(self, msg):
    self.steer_deg = float(msg.data)
    self.steer_time = self.now()

  def teleop_cb(self, msg):
    self.teleop = msg
    self.teleop_time = self.now()

  def estop_cb(self, msg):
    if bool(msg.data) != self.estop:
      self.get_logger().warn(f'E-STOP {"작동" if msg.data else "해제"}')
    self.estop = bool(msg.data)

  def fresh(self, t, timeout):
    return t is not None and (self.now() - t) <= timeout

  def tick(self):
    v, s, mode = 0.0, 0.0, 'STOP'

    if self.estop:
      v, s, mode = 0.0, 0.0, 'E-STOP'
    elif self.fresh(self.teleop_time, self.teleop_timeout) and self.teleop:
      # 사람이 잡으면 사람이 우선 (teleop은 이미 도 단위로 발행)
      v, s, mode = self.teleop.linear.x, self.teleop.angular.z, 'TELEOP'
    elif self.fresh(self.speed_time, self.input_timeout) and \
            self.fresh(self.steer_time, self.input_timeout):
      v, s, mode = self.target_speed, self.steer_deg, 'AUTO'
    else:
      # 자율 입력 중 하나라도 끊기면 정지(조향은 유지하지 않고 직진으로)
      mode = 'STOP(입력끊김)'

    # 최종 클램프
    v = max(self.min_speed, min(self.max_speed, float(v)))
    s = max(-self.max_steer, min(self.max_steer, float(s)))

    if mode != self.last_mode:
      self.get_logger().info(f'모드: {mode}')
      self.last_mode = mode

    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = s
    self.pub.publish(cmd)


def main(args=None):
  rclpy.init(args=args)
  node = VehicleCmdMux()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node.pub.publish(Twist())
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

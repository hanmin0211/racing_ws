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

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
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
    # ★ 헤딩 캘리브 완료 전에는 자율(AUTO)을 막는다.
    # direct_localization 은 yaw_offset=0(=IMU 원시 yaw, 방향 의미 없음)으로도
    # odom 을 발행한다. 그 위에서 로컬경로·조향이 계산되므로, 캘리브 전에
    # control:=true 로 띄우면 **차가 엉뚱한 방향으로 스스로 출발한다.**
    # teleop 과 E-stop 은 막지 않는다 — 캘리브 10m 직진을 사람이 몰아야 하므로.
    self.declare_parameter('require_heading_calib', True)

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
    # ★ 라이다 회피 조향 override (2026-08-19). AUTO 중 라이다가 유효한 회피각을
    # 주면(NaN 아님) GPS 경로 조향 대신 그 각으로 장애물을 피한다. 속도는 종방향이
    # /obstacle_distance 로 이미 안전하게 낮춘다. NaN 이면 평소대로 GPS 조향.
    self.avoid_steer = float('nan')
    self.avoid_time = None
    # 라이다 무신호 0.3s 면 회피각을 버리고 GPS 조향으로 복귀(옛 각을 물고 있지 않게).
    self.avoid_timeout = 0.3
    self.teleop = None
    self.teleop_time = None
    self.estop = False
    self.last_mode = None
    self.require_calib = bool(g('require_heading_calib'))
    self.heading_ready = not self.require_calib

    self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.create_subscription(Float64, '/target_speed', self.speed_cb, 10)
    self.create_subscription(Float64, '/steering_cmd', self.steer_cb, 10)
    self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_cb, 10)
    self.create_subscription(Bool, '/e_stop', self.estop_cb, 10)
    self.create_subscription(Float64, '/lidar/avoid_steer', self.avoid_cb, 10)
    # heading_init 은 계산 후 종료하므로 latched(TRANSIENT_LOCAL)로 발행한다.
    # 늦게 뜬 먹스도 과거 값을 받아야 하므로 같은 QoS 로 구독한다.
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Float64, '/heading/yaw_offset',
                             self.heading_cb, latched)
    self.create_timer(1.0 / rate, self.tick)

    self.get_logger().info(
        f'명령 먹스 시작: 조향±{self.max_steer}° 속도 {self.min_speed}~'
        f'{self.max_speed}m/s (E-stop > teleop > 자율)')
    if self.require_calib:
      self.get_logger().warn(
          '헤딩 캘리브 대기 — /heading/yaw_offset 을 받기 전에는 자율(AUTO)을 '
          '거부한다. teleop 으로 10m 직진해 캘리브를 끝낼 것.')

  def heading_cb(self, msg):
    if not self.heading_ready:
      self.heading_ready = True
      self.get_logger().info(
          f'헤딩 캘리브 완료 ({math.degrees(float(msg.data)):.1f}°) — 자율 허용')

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

  def avoid_cb(self, msg):
    self.avoid_steer = float(msg.data)   # NaN = 회피 없음
    self.avoid_time = self.now()

  def fresh(self, t, timeout):
    return t is not None and (self.now() - t) <= timeout

  def tick(self):
    v, s, mode = 0.0, 0.0, 'STOP'

    if self.estop:
      v, s, mode = 0.0, 0.0, 'E-STOP'
    elif self.fresh(self.teleop_time, self.teleop_timeout) and self.teleop:
      # 사람이 잡으면 사람이 우선 (teleop은 이미 도 단위로 발행)
      v, s, mode = self.teleop.linear.x, self.teleop.angular.z, 'TELEOP'
    elif not self.heading_ready:
      # 캘리브 전 자율 거부. 이유를 명시해야 현장에서 '왜 안 가지'로 헤매지 않는다.
      mode = 'STOP(헤딩 캘리브 전)'
    elif self.fresh(self.speed_time, self.input_timeout) and \
            self.fresh(self.steer_time, self.input_timeout):
      v, s, mode = self.target_speed, self.steer_deg, 'AUTO'
      # 라이다 회피: 유효한 회피각(NaN 아님)이 신선하면 GPS 조향을 덮어쓴다.
      # 속도(v)는 종방향이 /obstacle_distance 로 이미 낮췄으므로 그대로 둔다.
      if self.fresh(self.avoid_time, self.avoid_timeout) and \
              not math.isnan(self.avoid_steer):
        s, mode = self.avoid_steer, 'AVOID(라이다)'
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

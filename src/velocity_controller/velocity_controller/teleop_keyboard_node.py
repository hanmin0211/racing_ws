#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_keyboard_node.py — 실차 테스트용 키보드 수동 제어.

★ /cmd_vel 을 직접 쏘지 않고 **/teleop/cmd_vel** 로 발행한다.
   vehicle_cmd_mux 가 이걸 받아 자율 명령보다 우선 적용한다(사람 우선). 여러
   노드가 /cmd_vel 을 동시에 덮어쓰는 사고를 막기 위한 구조.

★ 단위 규약: angular.z = **조향각[도]** (rad/s 아님). 스택 전체가 도 단위다.

조작:
  W / S : 전진 / 후진   ※ **누르고 있는 동안만** 이동(데드맨). 떼면 즉시 0
  A / D : 조향 좌 / 우  ※ 각도는 유지(래치). 스텝 2°
  SPACE : 정지 + 조향 중앙
  E     : 비상정지 토글 (/e_stop) — 걸면 먹스가 모든 명령 차단
  Q / Z : 목표 속도 증감
  Ctrl+C: 종료

속도는 데드맨(홀드)이고 조향은 래치인 이유: 속도는 손을 떼면 멈추는 게 안전하고,
조향은 각도를 유지해야 코너를 잡을 수 있기 때문.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

STEER_STEP = 2.0     # A/D 한 번에 바뀌는 조향각[도]
SPEED_STEP = 0.1     # Q/Z 속도 증감[m/s]
KEY_HOLD_SEC = 0.25  # 이 시간 안에 W/S 재입력 없으면 뗀 것으로 간주(데드맨)

USAGE = """
=================================================
  HENES T870 키보드 수동 제어 (teleop)
=================================================
  W / S : 전진 / 후진   (누르고 있는 동안만 — 데드맨)
  A / D : 조향 좌 / 우  (각도 유지, 스텝 2°)
  SPACE : 정지 + 조향 중앙
  E     : 비상정지 토글
  Q / Z : 목표 속도 증감
  Ctrl+C: 종료
=================================================
"""


class TeleopKeyboard(Node):

  def __init__(self):
    super().__init__('teleop_keyboard')
    # 기본값은 스택 전체(먹스/펌웨어)와 동일한 보수적 한계
    self.declare_parameter('max_steer_deg', 18.0)
    self.declare_parameter('max_speed', 1.0)
    self.declare_parameter('min_speed', -0.6)
    self.declare_parameter('speed_setpoint', 0.5)

    self.max_steer = float(self.get_parameter('max_steer_deg').value)
    self.max_speed = float(self.get_parameter('max_speed').value)
    self.min_speed = float(self.get_parameter('min_speed').value)
    self.setpoint = float(self.get_parameter('speed_setpoint').value)

    self.steer = 0.0
    self.speed = 0.0
    self.estop = False
    self.last_drive_key = None   # 마지막 W/S 입력 시각(데드맨 판정)

    self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
    self.estop_pub = self.create_publisher(Bool, '/e_stop', 10)
    # 20Hz 로 계속 발행 — 먹스/펌웨어 워치독(0.5s)보다 훨씬 자주.
    self.create_timer(0.05, self.publish_cmd)

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def publish_cmd(self):
    # 데드맨: W/S 를 계속 누르고 있지 않으면 속도 0
    if self.last_drive_key is None or \
            (self.now() - self.last_drive_key) > KEY_HOLD_SEC:
      self.speed = 0.0
    cmd = Twist()
    cmd.linear.x = float(self.speed)
    cmd.angular.z = float(self.steer)
    self.pub.publish(cmd)
    self.estop_pub.publish(Bool(data=self.estop))

  def status(self):
    print(f'\r속도 {self.speed:+.2f} m/s (설정 {self.setpoint:.2f}) | '
          f'조향 {self.steer:+.1f}° | '
          f'{"★E-STOP★" if self.estop else "        "}   ', end='', flush=True)


def get_key(settings):
  tty.setraw(sys.stdin.fileno())
  r, _, _ = select.select([sys.stdin], [], [], 0.05)
  key = sys.stdin.read(1) if r else ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


def main(args=None):
  settings = termios.tcgetattr(sys.stdin)
  rclpy.init(args=args)
  node = TeleopKeyboard()
  print(USAGE)
  try:
    while rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.0)
      key = get_key(settings)
      if not key:
        continue
      k = key.lower()
      if k == 'w':
        node.speed = min(node.max_speed, node.setpoint)
        node.last_drive_key = node.now()
      elif k == 's':
        node.speed = max(node.min_speed, -node.setpoint)
        node.last_drive_key = node.now()
      elif k == 'a':
        node.steer = min(node.max_steer, node.steer + STEER_STEP)
      elif k == 'd':
        node.steer = max(-node.max_steer, node.steer - STEER_STEP)
      elif k == ' ':
        node.speed, node.steer = 0.0, 0.0
        node.last_drive_key = None
      elif k == 'e':
        node.estop = not node.estop
      elif k == 'q':
        node.setpoint = min(node.max_speed, node.setpoint + SPEED_STEP)
      elif k == 'z':
        node.setpoint = max(0.0, node.setpoint - SPEED_STEP)
      elif key == '\x03':
        break
      node.status()
  except Exception as e:  # noqa: BLE001
    node.get_logger().error(f'오류: {e}')
  finally:
    node.pub.publish(Twist())            # 정지 명령
    node.estop_pub.publish(Bool(data=False))
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print('\nteleop 종료.')


if __name__ == '__main__':
  main()

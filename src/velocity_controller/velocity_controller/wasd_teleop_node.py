#!/usr/bin/env python3
"""
wasd_teleop_node.py

역할: 터미널에서 WASD 키 입력을 받아 /cmd_vel (geometry_msgs/Twist) 토픽으로
     발행하는 조작 노드.

키 매핑
-------
    W : 전진 속도 +0.1 m/s
    S : 후진 속도 (또는 감속) -0.1 m/s
    A : 좌회전 (조향각 -5도)
    D : 우회전 (조향각 +5도)
    X 또는 스페이스바 : 즉시 정지 (속도 0, 조향 0)
    Q : 프로그램 종료

동작 방식
---------
한 번 누를 때마다 값이 누적/변경되고, 그 값을 계속 20Hz로 반복 발행합니다.
(예: W를 세 번 누르면 0.1 → 0.2 → 0.3 m/s로 계속 증가)

주의: 이 노드는 serial_bridge_node와 마찬가지로, 이 창(터미널)에 포커스가
있어야 키 입력이 인식됩니다. 다른 창을 클릭한 상태로 키를 누르면 반응 안 함.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---------- 설정값 (필요하면 여기 숫자만 바꾸면 됨) ----------
SPEED_STEP = 0.1      # W/S 한 번에 바뀌는 속도량 (m/s)
STEER_STEP = 5.0       # A/D 한 번에 바뀌는 조향각량 (degree)
MAX_SPEED = 1.0        # 최대 속도 제한 (m/s) - 안전을 위해 낮게 시작
MAX_STEER = 18.0       # 실사용 최대 조향각 (degree). 물리한계 20°지만 포텐셔미터
                       # 포화(ADC 0)를 피해 18°로 제한 — serial_bridge/pure_pursuit와 동일

INSTRUCTIONS = """
=====================================
   WASD 조작 모드
=====================================
   W : 전진 속도 +{step}
   S : 후진/감속 -{step}
   A : 좌회전
   D : 우회전
   X 또는 Space : 즉시 정지
   Q : 종료
=====================================
""".format(step=SPEED_STEP)


def get_key(settings):
    """터미널에서 키 하나를 눌렀을 때 바로 읽어오는 함수.
    (Enter 안 눌러도 즉시 인식되도록 raw 모드로 전환해서 읽음)
    """
    tty.setraw(sys.stdin.fileno())
    # 0.1초 안에 입력이 없으면 빈 문자열 반환 (프로그램이 멈춰있지 않게)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class WasdTeleopNode(Node):

    def __init__(self):
        super().__init__('wasd_teleop_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.speed = 0.0   # 현재 목표 속도 (m/s)
        self.steer = 0.0   # 현재 목표 조향각 (degree)

        # 20Hz로 현재 값을 계속 반복 발행
        # (serial_bridge_node의 워치독이 0.5초마다 체크하니, 그보다 훨씬
        #  자주 보내야 "명령이 끊겼다"고 오인해서 자동 정지되지 않음)
        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info(INSTRUCTIONS)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.steer
        self.pub.publish(msg)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = WasdTeleopNode()

    try:
        while rclpy.ok():
            # ROS2 타이머(발행)와 콜백들이 계속 처리되도록 한 번 돌려줌
            rclpy.spin_once(node, timeout_sec=0.0)

            key = get_key(settings)

            if key == 'w':
                node.speed = min(node.speed + SPEED_STEP, MAX_SPEED)
            elif key == 's':
                node.speed = max(node.speed - SPEED_STEP, -MAX_SPEED)
            elif key == 'a':
                node.steer = max(node.steer - STEER_STEP, -MAX_STEER)
            elif key == 'd':
                node.steer = min(node.steer + STEER_STEP, MAX_STEER)
            elif key in ('x', ' '):
                node.speed = 0.0
                node.steer = 0.0
            elif key == 'q':
                break
            elif key == '\x03':  # Ctrl+C
                break

            if key:
                node.get_logger().info(
                    f'speed={node.speed:.2f} m/s, steer={node.steer:.1f} deg')

    except Exception as e:
        node.get_logger().error(f'에러 발생: {e}')

    finally:
        # 종료 직전 반드시 정지 명령 발행 (안전)
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()

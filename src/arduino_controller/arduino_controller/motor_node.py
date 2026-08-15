#!/usr/bin/env python3
"""
motor_node_v2.py
ROS2 Humble — 헤네스 PID 모터 제어 노드 v2

구독: /cmd_vel (geometry_msgs/Twist)
  linear.x  → 속도 목표 [km/h]  (양수=전진, 음수=후진)
  angular.z → 조향 목표각 [deg] (-30=좌, +30=우)

시리얼 명령:
  VEL,<km/h>              → 속도
  STEER,<deg>             → 조향
  STOP                    → 정지
  PID,STEER,<Kp>,<Ki>,<Kd> → 조향 PID 튜닝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

# ── 스케일 상수 ──────────────────────────────────────────────────────
# teleop_twist_keyboard 기본값: max linear 0.5, max angular 1.0
# 이걸 km/h, deg 로 변환
MAX_KMH   = 10.0   # linear.x 최대값(0.5) → 10 km/h
MAX_DEG   = 30.0   # angular.z 최대값(1.0) → 30 deg

VEL_SCALE   = MAX_KMH / 0.5
STEER_SCALE = MAX_DEG / 1.0


class HenesMotorV2(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('port',        '/dev/ttyACM0')
        self.declare_parameter('baudrate',    115200)
        self.declare_parameter('cmd_timeout', 0.5)

        port    = self.get_parameter('port').value
        baud    = self.get_parameter('baudrate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # ── 상태 변수 ─────────────────────────────────────────────────
        self.vel_kmh  = 0.0
        self.steer_deg = 0.0
        self.last_cmd_time = time.monotonic()

        # ── 시리얼 연결 ───────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'아두이노 연결 완료! ({port})')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 연결 실패: {e}')
            raise

        # ── 시리얼 수신 스레드 (아두이노 디버그 출력) ────────────────
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()

        # ── ROS2 구독 ─────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        # ── 타임아웃 감시 (10Hz) ─────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('HenesMotorV2 시작')
        self.get_logger().info(
            f'  linear.x × {VEL_SCALE:.1f} = km/h  |  '
            f'angular.z × {STEER_SCALE:.1f} = deg')

    # ── /cmd_vel 콜백 ─────────────────────────────────────────────────
    def _cmd_cb(self, msg: Twist):
        self.vel_kmh   = round(msg.linear.x  * VEL_SCALE,  2)
        self.steer_deg = round(msg.angular.z * STEER_SCALE, 2)
        self.vel_kmh   = max(-MAX_KMH, min(MAX_KMH,  self.vel_kmh))
        self.steer_deg = max(-MAX_DEG, min(MAX_DEG, self.steer_deg))
        self.last_cmd_time = time.monotonic()

        self._send(f'VEL,{self.vel_kmh}')
        self._send(f'STEER,{self.steer_deg}')

        self.get_logger().info(
            f'CMD → {self.vel_kmh} km/h  |  {self.steer_deg} deg')

    # ── 타임아웃 감시 ─────────────────────────────────────────────────
    def _watchdog(self):
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            if self.vel_kmh != 0.0 or self.steer_deg != 0.0:
                self.get_logger().warn('타임아웃 → STOP')
                self.vel_kmh   = 0.0
                self.steer_deg = 0.0
                self._send('STOP')

    # ── 시리얼 수신 (아두이노 디버그 출력 표시) ─────────────────────
    def _reader(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'[Arduino] {line}')
            except Exception:
                pass
            time.sleep(0.01)

    # ── 시리얼 송신 ──────────────────────────────────────────────────
    def _send(self, cmd: str):
        try:
            self.ser.write((cmd + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().error(f'송신 오류: {e}')

    # ── 종료 ─────────────────────────────────────────────────────────
    def destroy_node(self):
        self._send('STOP')
        time.sleep(0.1)
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HenesMotorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

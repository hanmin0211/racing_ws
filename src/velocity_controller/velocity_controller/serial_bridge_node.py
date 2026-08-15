#!/usr/bin/env python3
"""
serial_bridge_node.py

역할: PC(ROS2)와 Arduino Mega(POWERPACK.ino) 사이의 "통역사" 노드.

    [ROS2 쪽]                              [Arduino 쪽]
    /cmd_vel (Twist)  ──이 노드가 변환──→  "VEL:0.30,STEER:10\n"  (시리얼)
    /vehicle_status   ←─이 노드가 변환──   "STATUS_10ms: ENC1=... VEL=..." (시리얼)

주요 기능
---------
1. /cmd_vel 토픽 구독 (linear.x = 목표속도 m/s, angular.z = 조향각 deg로 재사용)
2. Arduino가 이해하는 "VEL:x,STEER:y\n" 문자열로 변환해서 0.05초(20Hz)마다 전송
3. Arduino가 보내는 상태 메시지를 읽어서 /vehicle_status 토픽으로 재발행
4. 워치독: 일정 시간(기본 0.5초) 동안 /cmd_vel이 안 들어오면 자동으로 정지 명령 전송
   (키보드 텔레옵이 죽거나 연결이 끊겨도 차가 계속 달리는 사고 방지)
"""

import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Int32, String

import serial
from serial.tools import list_ports


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        # ---------- 파라미터: 실행할 때 값 바꿀 수 있게 (포트, baud, 워치독시간) ----------
        # port='auto'면 /dev/arduino → 없으면 Arduino Mega(2341:0042) 자동감지.
        # 특정 포트를 직접 주면(예: /dev/ttyACM0) 그걸 그대로 씀.
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('watchdog_timeout', 0.5)  # 초 단위
        # 조향각 클램프[도] = 실사용 한계. 물리 한계는 20°(펌웨어 캘리브 기준값)지만
        # 그 지점에서 포텐셔미터가 ADC 0으로 포화해 피드백을 잃고 엔드스톱 컷이 걸리므로
        # 18°로 제한한다. (예전 하드코딩 30°는 실제보다 훨씬 커서 위험했음)
        self.declare_parameter('max_steer_deg', 18.0)

        port = self._resolve_port(
            self.get_parameter('port').get_parameter_value().string_value)
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)

        # ---------- 시리얼 포트 열기 ----------
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)

            time.sleep(2)

            self.ser.reset_input_buffer()

            self.get_logger().info(f'시리얼 포트 연결 성공: {port} @ {baud}bps')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 포트 연결 실패: {e}')
            raise

        # ---------- 현재 목표값 저장 변수 ----------
        self.target_vel = 0.0       # m/s
        self.target_steer = 0.0     # degree (-30~30), 소수 유지(펌웨어가 float 파싱)

        # ---------- 구독자: /cmd_vel ----------
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        # 개루프 PWM 요청(FF 식별 전용). 값이 오면 VEL 대신 PWM: 명령을 보낸다.
        # 0을 받거나 openloop_timeout 동안 소식이 없으면 폐루프(VEL)로 자동 복귀.
        self.openloop_pwm = None
        self.openloop_time = None
        self.declare_parameter('openloop_timeout', 0.5)
        self.openloop_timeout = float(
            self.get_parameter('openloop_timeout').value)
        self.create_subscription(Int32, '/drive_pwm_cmd',
                                 self.drive_pwm_callback, 10)

        # ---------- 발행자: Arduino 상태를 ROS2 토픽으로 재발행 ----------
        self.status_pub = self.create_publisher(String, '/vehicle_status', 10)
        # 구조화 텔레메트리 — 다른 노드(종방향 제어 등)가 실제로 쓸 수 있는 형태.
        # 문자열 재발행만으로는 아무도 못 쓰기 때문에 필드별로 풀어서 낸다.
        self.speed_pub = self.create_publisher(Float64, '/current_speed', 10)
        self.steer_ang_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.steer_adc_pub = self.create_publisher(Int32, '/steering_adc', 10)
        # 조향 추종 오차(명령각 − 실제각). 주행 중 조향이 명령을 못 따라가는지 감시.
        self.steer_err_pub = self.create_publisher(Float64, '/steering_error', 10)
        self.obstacle_pub = self.create_publisher(Float64, '/obstacle_distance', 10)
        self.stall_pub = self.create_publisher(Bool, '/vehicle_stall', 10)
        # 엔코더 원시 카운트 — 엔코더 스케일(counts_per_revolution) 검증에 필수.
        # RTK 이동거리와 비교해 1카운트당 실제 거리를 역산한다.
        self.enc_pub = self.create_publisher(Int32, '/encoder_count', 10)
        # 개루프 식별용: 현재 인가 중인 구동 PWM
        self.drive_pwm_pub = self.create_publisher(Int32, '/drive_pwm', 10)
        # 소나 유효 최대거리[m]. NewPing은 미검출 시 0을 주므로 그대로 쓰면
        # '장애물 0m'로 오인해 급정지한다 → 미검출은 이 값(=없음)으로 변환.
        self.declare_parameter('sonar_max_range', 2.0)
        self.sonar_max = float(self.get_parameter('sonar_max_range').value)

        # ---------- 마지막으로 /cmd_vel 을 받은 시각 (워치독 판단용) ----------
        self.last_cmd_time = self.get_clock().now()

        # ---------- 타이머 1: 20Hz로 Arduino에 명령 전송 ----------
        # Arduino 자체 루프는 100Hz(10ms)로 돌지만, 목표값 전송은 그렇게 자주
        # 안 보내도 됨. 20Hz(50ms)면 반응성과 시리얼 부하 사이 적당한 절충점.
        self.send_timer = self.create_timer(0.05, self.send_command)

        # ---------- 타이머 2: 10Hz로 워치독(안전장치) 체크 ----------
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        # ---------- 시리얼 읽기는 별도 스레드에서 처리 ----------
        # 시리얼 읽기(ser.readline())는 데이터 올 때까지 잠깐 멈춰있는(blocking)
        # 작업이라, ROS2 메인 스레드에서 그대로 하면 다른 콜백들이 밀림.
        # 그래서 읽기 전용 스레드를 따로 하나 만들어서 돌림.
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        # Arduino가 보내는 "STATUS_10ms: ENC1=... VEL=... ..." 형식을 뽑아내는 정규식
        self.status_pattern = re.compile(
            r'STATUS_10ms:\s*ENC1=(-?\d+)\s*VEL=(-?\d+\.\d+)\s*TARGET=(-?\d+\.\d+)\s*'
            # SLOPE 와 SONAR1 사이에 MODE= 등 필드가 추가돼도 깨지지 않게 .*? 사용
            r'PWM=(-?\d+)\s*SLOPE=(\w+).*?SONAR1=(-?\d+\.\d+)\s*SONAR2=(-?\d+\.\d+)\s*SONAR3=(-?\d+\.\d+)'
        )

    # ------------------------------------------------------------------
    def _resolve_port(self, port):
        """포트 결정: 'auto'면 /dev/arduino 우선, 없으면 Arduino Mega(2341:0042)를
        스캔해 찾는다. 특정 포트를 지정하면 그대로 사용(udev 없이도 동작)."""
        if port and port != 'auto':
            return port
        if os.path.exists('/dev/arduino'):
            return '/dev/arduino'
        for p in list_ports.comports():
            if p.vid == 0x2341 and p.pid == 0x0042:   # Arduino Mega 2560
                self.get_logger().info(f'Arduino Mega 자동감지: {p.device}')
                return p.device
        self.get_logger().warn(
            'Arduino Mega(2341:0042) 자동감지 실패 → /dev/arduino 로 시도')
        return '/dev/arduino'

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        """/cmd_vel 토픽이 들어올 때마다 자동 호출됨."""
        self.target_vel = msg.linear.x
        # angular.z 값을 "조향각(도)"으로 재사용. 원래 Twist는 각속도(rad/s)
        # 용도지만, 여기선 편의상 그냥 각도 값으로 씀. -30~30도로 제한.
        # int 절삭하면 1° 해상도 손실(펌웨어 데드밴드와 충돌) → 소수 유지.
        m = self.max_steer_deg
        self.target_steer = max(-m, min(m, float(msg.angular.z)))
        self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def drive_pwm_callback(self, msg: Int32):
        """개루프 PWM 요청. FF 식별(ff_sweep) 전용."""
        self.openloop_pwm = int(msg.data)
        self.openloop_time = self.get_clock().now()
        self.last_cmd_time = self.openloop_time

    # ------------------------------------------------------------------
    def send_command(self):
        """0.05초(20Hz)마다 실행. 현재 목표값을 Arduino로 전송."""
        # 개루프 요청이 살아있으면 PWM 명령, 아니면 평소대로 VEL 명령
        ol = False
        if self.openloop_pwm is not None and self.openloop_time is not None:
            age = (self.get_clock().now() - self.openloop_time).nanoseconds / 1e9
            if age <= self.openloop_timeout and self.openloop_pwm != 0:
                ol = True
            elif age > self.openloop_timeout:
                self.openloop_pwm = None      # 만료 → 폐루프 복귀
        if ol:
            cmd = f'PWM:{self.openloop_pwm},STEER:{self.target_steer:.1f}\n'
        else:
            cmd = f'VEL:{self.target_vel:.2f},STEER:{self.target_steer:.1f}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 전송 실패: {e}')

    # ------------------------------------------------------------------
    def watchdog_check(self):
        """일정 시간 /cmd_vel이 안 오면 강제 정지 (안전장치)."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout:
            if self.target_vel != 0.0 or self.target_steer != 0.0:
                self.get_logger().warn(
                    f'{self.watchdog_timeout}초간 /cmd_vel 수신 없음 → 안전 정지')
            self.target_vel = 0.0
            self.target_steer = 0.0

    # ------------------------------------------------------------------
# ------------------------------------------------------------------
    def read_serial_loop(self):
        """별도 스레드에서 계속 Arduino로부터 오는 줄을 읽어서 처리."""
        self.get_logger().info("Serial read thread started")

        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    # 20Hz 텔레메트리라 INFO로 찍으면 콘솔 홍수 → DEBUG로. 필요 시
                    # `--ros-args --log-level debug` 로 확인.
                    self.get_logger().debug(f"RAW: {repr(line)}")

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                continue

            if not line:
                continue

            msg = String()
            msg.data = line
            self.status_pub.publish(msg)

            self.parse_telemetry(line)

    # ------------------------------------------------------------------
    def parse_telemetry(self, line):
        """펌웨어 텔레메트리 문자열 → 구조화 토픽.

        STATUS_10ms: ENC1=.. VEL=.. TARGET=.. PWM=.. SLOPE=.. SONAR1/2/3=..
        STEER: ADC=.. TGT=.. PWM=.. ANG=.. VCC=.. VMIN=..
        STALL: drive=.. steer=..
        """
        try:
            if line.startswith('STATUS_10ms'):
                m = self.status_pattern.search(line)
                if m:
                    enc1, vel, _, pwm, _, s1, s2, s3 = m.groups()
                    self.speed_pub.publish(Float64(data=float(vel)))
                    self.enc_pub.publish(Int32(data=int(enc1)))
                    self.drive_pwm_pub.publish(Int32(data=int(pwm)))
                    # 미검출(0.00)은 '장애물 없음'이므로 최대거리로 치환한 뒤 최솟값.
                    ds = []
                    for s in (s1, s2, s3):
                        d = float(s)
                        ds.append(self.sonar_max if d <= 0.01 else d)
                    self.obstacle_pub.publish(Float64(data=min(ds)))
            elif line.startswith('STEER:'):
                adc = int(line.split('ADC=')[1].split()[0])
                self.steer_adc_pub.publish(Int32(data=adc))
                # ANGACT = ADC로 환산한 '실제' 조향각. ANG은 명령각이라 그대로 쓰면
                # 명령을 되돌려받는 셈이라 추종 검증이 안 된다 → 실제각을 발행한다.
                if 'ANGACT=' in line:
                    act = float(line.split('ANGACT=')[1].split()[0])
                    self.steer_ang_pub.publish(Float64(data=act))
                    cmd = float(line.split(' ANG=')[1].split()[0])
                    self.steer_err_pub.publish(Float64(data=cmd - act))
            elif line.startswith('STALL:'):
                drive = line.split('drive=')[1].split()[0].strip()
                steer = line.split('steer=')[1].split()[0].strip()
                stalled = (drive not in ('0',)) or (steer not in ('0',))
                self.stall_pub.publish(Bool(data=stalled))
                if stalled:
                    self.get_logger().warn(
                        f'펌웨어 스톨 감지: {line}', throttle_duration_sec=2.0)
        except (IndexError, ValueError):
            pass   # 전송 중 잘린 줄은 무시

    # ------------------------------------------------------------------
    def destroy_node(self):
        """노드가 꺼질 때(Ctrl+C 등) 안전하게 정지 명령 보내고 시리얼 닫기."""
        try:
            self.ser.write(b'VEL:0,STEER:0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

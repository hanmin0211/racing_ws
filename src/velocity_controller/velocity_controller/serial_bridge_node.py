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

import math
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
        # ★ 스톨 해제 판정 시간 (2026-09-09)
        #   펌웨어는 **스톨일 때만** "STALL: drive=.. steer=.." 줄을 낸다.
        #   해제됐다는 신호는 따로 안 준다. 그래서 예전 코드는 한 번 true 가
        #   되면 /vehicle_stall 이 영영 true 로 남았다 — 실제로는 풀렸는데도
        #   스톨로 보여서 로그·미션 판단이 오판한다.
        #   텔레메트리가 20Hz(50ms)이므로, 이 시간 동안 STALL 줄이 한 번도
        #   안 오면 해제된 것으로 본다.
        self.declare_parameter('stall_clear_timeout', 0.5)
        # 조향각 클램프[도] = 실사용 한계. 물리 한계는 20°(펌웨어 캘리브 기준값)지만
        # 그 지점에서 포텐셔미터가 ADC 0으로 포화해 피드백을 잃고 엔드스톱 컷이 걸리므로
        # 18°로 제한한다. (예전 하드코딩 30°는 실제보다 훨씬 커서 위험했음)
        self.declare_parameter('max_steer_deg', 18.0)

        self._port_param = self.get_parameter('port').get_parameter_value().string_value
        port = self._resolve_port(self._port_param)
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self._baud = baud
        self._reconnects = 0
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)

        # ---------- 시리얼 포트 열기 ----------
        try:
            # ★ exclusive=True → TIOCEXCL. 다른 프로세스가 이 포트를 여는 것을
            # **커널이 막는다**. 두 가지를 동시에 해결한다:
            #  1) ModemManager 가 /dev/ttyACM* 를 모뎀인지 프로브하려고 여는 것.
            #     여는 순간 DTR 로 아두이노가 리셋되고, 리셋되면 장치가 다시
            #     나타나 또 프로브당해 무한 리셋 루프가 된다(2026-08-18 현장:
            #     연결 0.32초 뒤 끊김이 정확히 반복).
            #  2) serial_bridge 를 실수로 두 개 띄웠을 때의 상호 리셋.
            # 근본 해결은 udev 의 ID_MM_DEVICE_IGNORE 지만 그건 sudo 가 필요하다.
            self.ser = self._open_no_reset(port, baud)
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

        # ★ ROS 쪽 FF (2026-09-12 학교 실측) — 플래싱 없이 구동 상수를 고친다.
        #
        #   펌웨어의 개루프 FF 는  PWM = 80 + 95·v  인데 지면 실측과 크게 다르다:
        #     PWM 55 → 0.46 m/s · 65 → 1.11 · 75 → 1.20   (tools/ff_identify.py)
        #     적합:  PWM = 22.6·v + 44.2
        #   옛 상수로는 명령 0.15 m/s 에 PWM 94 가 걸려 차가 **2 m/s 로 폭주**했고,
        #   헤딩 캘리브가 8회 연속 실패했다(측위 점프로 오인).
        #
        #   ff_mode:='ros' 면 VEL: 대신 여기서 변환한 PWM: 을 보낸다. 펌웨어를
        #   다시 굽지 않고 현장에서 상수를 고칠 수 있다.
        #   ⚠ 엔코더를 살려 속도 폐루프(NO_ENCODER 0)를 복구하면 이건 꺼야 한다
        #     — 그때는 펌웨어가 실제 속도를 보고 제어하는 쪽이 옳다.
        self.declare_parameter('ff_mode', 'firmware')   # 'firmware' | 'ros'
        # ★★ 2026-09-12 2차 수정 — 캘리브 10m 주행 로그로 재적합.
        #   ff_identify 의 'PWM 55 → 0.46 m/s' 는 틀린 측정이었다. 그 런은 12초에
        #   3.6m 밖에 못 갔는데 정상상태를 0.46 으로 뽑았다(앞뒤가 안 맞는다).
        #   반면 캘리브 10m 직진은 1m 구간 9개가 전부 1.00±0.02 m/s 로 일관됐다:
        #     명령 0.5 m/s → PWM 56 → **실측 1.00 m/s** (정확히 2배 빨랐다)
        #   이 점과 calib_trace 의 PWM 94 → 1.98 m/s 로 다시 그으면
        #     PWM = 38.8·v + 17.2
        #   ⚠ 캘리브 진행 로그('직진 중... N.N/10m')는 그 자체가 속도계다.
        #     현장에서 노면이 바뀌면 그 로그로 상수를 다시 뽑을 것.
        self.declare_parameter('ff_static', 17.2)
        self.declare_parameter('ff_gain', 38.8)
        self.declare_parameter('ff_deadband', 0.05)     # 이 이하 명령은 PWM 0
        # ★ 크리프 하한 — 탈락 방지용.
        #   이 차는 개루프로 0.45 m/s 아래를 못 낸다(PWM 45 → 0.00 m/s).
        #   그런데 종방향 제어는 커브에서 v = v_max/(1+6·|κ|) 로 줄인다.
        #   T자 급커브(κ≈0.5)면 명령이 0.25 m/s → PWM 50 → **바퀴가 안 돈다**.
        #   그대로 서 버리면 '1분 이상 정지' 로 탈락이다.
        #   그래서 '멈추라(<deadband)' 가 아닌 한 최소 ff_min_pwm 은 인가해
        #   느리더라도 계속 굴러가게 한다. 실제 속도는 약 0.46 m/s 가 된다.
        #   하한 50: 정지마찰을 넘기는 최소값. 이 선은 '구르는 중' 을 맞춘 것이라
        #   출발 순간에는 부족할 수 있어 하한을 따로 둔다(PWM 45 는 안 굴렀다).
        self.declare_parameter('ff_min_pwm', 50.0)
        # ★ 정지마찰 하한 (2026-09-13) — cbf75ee 가 "따로 둔다" 고 써 놓고
        #   **실제로는 만들지 않은** 것이다. 그래서 유일한 하한이 55→50 으로
        #   내려간 채 남았고, 9/13 학교에서 차가 출발을 못 했다.
        #     PWM 45 → 0.00 m/s (안 구름)
        #     PWM 50 → 9/13 실측 **안 구름** (좌표 1cm 도 안 변하고 VMIN 2938mV)
        #     PWM 55 → 덜컹(스틱슬립)
        #     PWM 56 → 1.00 m/s ✓   (9/12 캘리브가 이 값으로 굴렀다)
        #     PWM 60 → 9/13 실측 1.07 m/s ✓
        #   즉 **정지마찰 문턱이 PWM 55~56 사이**다. 하한 50 은 그 아래다.
        #
        #   '구르는 중' 의 하한(ff_min_pwm)과 '출발' 의 하한은 다른 값이어야
        #   한다. 잠긴 모터는 역기전력이 없어 전류를 최대로 빨고(9/13 VMIN
        #   2938mV), 그 상태로 버티면 전압이 더 무너진다. **빨리 굴리는 것이
        #   전원에도 이롭다.**
        #
        #   엔코더가 없어서 '실제로 움직였는가' 를 알 수 없다. 그래서
        #   시간으로 끊는다 — 정지(PWM 0)에서 출발할 때만 breakaway_ms 동안
        #   높은 PWM 을 인가하고 그 뒤 평소 하한으로 내린다.
        #   ⚠ 시간은 **펌웨어 램프를 감안해서** 잡아야 한다. 펌웨어의
        #     OPENLOOP_RATE 가 증가 방향으로 3/10ms 제한을 걸어서, 0 → 60 에
        #     실측 **390ms** 가 걸린다(9/13 breakaway_check). 600ms 로 두면
        #     실제로 60 이 나가는 건 260ms 뿐이고, 앞의 390ms 동안 모터는
        #     문턱(55~56) 아래에서 **잠긴 채 전류만 빤다.** 전원을 지키려고
        #     만든 램프가 오히려 스톨을 길게 만드는 셈이다.
        #     1200ms 면 램프를 빼고도 810ms 가 남는다.
        #     (내리는 방향은 램프가 없어 60 → 50 은 즉시다)
        self.declare_parameter('ff_breakaway_pwm', 60.0)
        self.declare_parameter('ff_breakaway_ms', 1200.0)
        # ★ 능동 제동 (2026-09-16) — **기본 OFF(0)**. 인자로만 켜진다.
        #
        #   왜 필요한가: `PWM 0` 은 '동력 끊기' 지 '멈추기' 가 아니다. 그 뒤는
        #   관성이다. 9/15 실측 — 돌발 더미 앞에서
        #       장애물 1.77m v=1.42 → 0.30m v=0.85 → 제동거리 1.96m / 2.5초
        #   더미 30cm 앞에서도 0.85m/s 로 굴러가고 있었다. 조금만 달라도 박는다.
        #   실측 가속능력(상위10%) 0.87m/s² · 관성감속 0.56m/s² 기준,
        #   역토크를 걸면 1.15m/s 에서 제동거리 1.18m → 0.4~0.55m 로 준다.
        #
        #   ⚠ 엔코더가 없어 '멈췄다' 를 직접 못 본다. 그래서 안전장치 셋:
        #     ① ff_brake_ms 시간 상한 — 측정이 없어도 이건 항상 건다
        #     ② 측정속도(/odometry/filtered)가 ff_brake_min_v 밑이면 즉시 해제
        #     ③ ff_brake_trigger_v 이상으로 달리다 선 경우에만 건다
        #        (주차처럼 느린 기동에서 덜컹거리지 않게)
        #   ⚠ 펌웨어가 방향전환을 0 경유로 막고 역토크를 3/10ms 로 올린다.
        #     즉 제동이 200~400ms 에 걸쳐 붙는다. 그만큼 효과가 깎인다.
        self.declare_parameter('ff_brake_pwm', 0.0)
        # ★ 상한은 '제동력' 이 아니라 **뒤로 밀리지 않게 하는 안전장치**다.
        #   돌발정지는 최대한 빨리 서는 게 목적이므로 이 값이 제동을 중간에
        #   끊으면 안 된다. 1.15m/s 에서 실효감속 1.4m/s² 면 정지까지 0.82초다.
        #   800ms 는 그걸 덮는다. 실제로 멈추면 아래 측정속도 연동이 먼저 끊는다.
        self.declare_parameter('ff_brake_ms', 800.0)
        self.declare_parameter('ff_brake_min_v', 0.15)
        self.declare_parameter('ff_brake_trigger_v', 0.40)
        # 제동 지속시간을 **속도에 맞춰** 잡는다. 느린 정지에 400ms 를 통째로
        # 걸면 이미 선 차에 역토크가 계속 걸려 뒤로 밀린다.
        #   t = min(ff_brake_ms, |직전속도| / ff_brake_decel)
        # 기본 1.4 m/s² 는 실측 가속능력(0.87)+관성감속(0.56) 에서 왔다.
        #   ⚠ **보수적으로(낮게) 잡는다.** 이 값이 실제보다 높으면 차가 서기
        #     전에 제동을 끊는다(급정지 실패). 낮으면 더 오래 걸되, 멈추는
        #     순간 측정속도 연동이 끊는다. 급정지에서는 후자가 안전하다.
        self.declare_parameter('ff_brake_decel', 1.0)
        # ★ 2026-09-17 — **내리막 속도 거버너** (기본 0 = 꺼짐)
        #
        #   왜: 개루프에는 '달리는 중' 감속 권한이 없다. 제동(ff_brake_pwm)은
        #   **정지 명령에만** 붙는다. 그래서 내리막에서 중력이 이기면 명령을
        #   낮춰도 계속 빨라진다. 실측 — 오늘 시험장(18~20%)에서 관성만으로도
        #   8m 에 4.1 m/s 가 된다. 굴절코스 첫 코너(R=6.4m) 상한은 3.0 이다.
        #
        #   무엇을: 측정속도가 명령보다 gov_deadband 넘게 빠르면, 초과분에
        #   비례해 **역 PWM** 을 낸다. 명령 자체를 따라가는 게 아니라
        #   '과속만 깎는' 보조 제어다.
        #
        #   ⚠ 전진 중 역 PWM 은 **플러깅**이라 역기전력이 인가전압에 더해져
        #     전류가 기동보다도 크다. 그래서 gov_pwm 으로 상한을 반드시 건다.
        #     법정 내리막(6.5~9%)은 실측 구름저항 0.733 덕에 역 PWM 50 이면
        #     충분하다(제동력 1.01 + 저항 0.733 > 중력 0.88).
        #
        #   ⚠ 이건 **개루프용 임시 수단**이다. NO_ENCODER 0 으로 펌웨어 속도
        #     PID 를 복구하면 그쪽이 같은 일을 더 잘한다(연속 제어).
        self.declare_parameter('gov_pwm', 0.0)        # 역 PWM 상한. 0 = 꺼짐
        self.declare_parameter('gov_deadband', 0.30)  # 이만큼 넘어야 개입 [m/s]
        self.declare_parameter('gov_gain', 80.0)      # 초과 1 m/s 당 역 PWM
        self.ff_mode = str(self.get_parameter('ff_mode').value).lower()
        self.ff_static = float(self.get_parameter('ff_static').value)
        self.ff_gain = float(self.get_parameter('ff_gain').value)
        self.ff_deadband = float(self.get_parameter('ff_deadband').value)
        self.ff_min_pwm = float(self.get_parameter('ff_min_pwm').value)
        self.ff_breakaway_pwm = float(
            self.get_parameter('ff_breakaway_pwm').value)
        self.ff_breakaway_s = float(
            self.get_parameter('ff_breakaway_ms').value) / 1000.0
        self._moving_since = None      # 정지→출발 전환 시각 (None = 정지 중)
        self.ff_brake_pwm = float(self.get_parameter('ff_brake_pwm').value)
        self.ff_brake_s = float(self.get_parameter('ff_brake_ms').value) / 1000.0
        self.ff_brake_min_v = float(
            self.get_parameter('ff_brake_min_v').value)
        self.ff_brake_trigger_v = float(
            self.get_parameter('ff_brake_trigger_v').value)
        self.ff_brake_decel = max(
            0.1, float(self.get_parameter('ff_brake_decel').value))
        self.gov_pwm = float(self.get_parameter('gov_pwm').value)
        self.gov_deadband = float(self.get_parameter('gov_deadband').value)
        self.gov_gain = float(self.get_parameter('gov_gain').value)
        self._brake_until = None       # 제동 종료 예정 시각 (None = 제동 안 함)
        self._brake_sign = 0           # 직전 진행 방향 (+1 전진 / -1 후진)
        self._last_cmd_v = 0.0         # 직전 주기의 명령 속도
        self._meas_v = None            # 측정 속도 (없으면 None → 시간상한만)
        self._meas_v_t = 0.0
        if self.ff_brake_pwm > 0.0 or self.gov_pwm > 0.0:
            from nav_msgs.msg import Odometry  # noqa: PLC0415
            self.create_subscription(Odometry, '/odometry/filtered',
                                     self._odom_cb, 10)
            if self.gov_pwm > 0.0:
                self.get_logger().warn(
                    f'★ 내리막 속도 거버너 켜짐: 측정속도가 명령보다 '
                    f'{self.gov_deadband:.2f}m/s 넘게 빠르면 역 PWM 을 낸다 '
                    f'(초과 1m/s 당 {self.gov_gain:.0f}, 상한 {self.gov_pwm:.0f}). '
                    f'⚠ 플러깅이라 전류가 크다 — 상한을 함부로 올리지 말 것.')
        if self.ff_brake_pwm > 0.0:
            self.get_logger().warn(
                f'★ 능동 제동 켜짐: 역 PWM {self.ff_brake_pwm:.0f} 을 최대 '
                f'{self.ff_brake_s * 1000:.0f}ms. '
                f'측정속도 {self.ff_brake_min_v:.2f}m/s 밑이면 즉시 해제. '
                f'{self.ff_brake_trigger_v:.2f}m/s 이상에서 선 경우에만 건다.')
        if self.ff_mode == 'ros':
            self.get_logger().warn(
                f'★ ROS 쪽 FF 사용: PWM = {self.ff_gain:.1f}·v + '
                f'{self.ff_static:.1f} (펌웨어 FF 우회). '
                '엔코더 복구 후에는 ff_mode:=firmware 로 되돌릴 것.')

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
        # 공급전압 진단 (mV). vcc=현재, vmin=직전 구간 최솟값(순간 강하 포착)
        self.vcc_pub = self.create_publisher(Int32, '/vcc_mv', 10)
        self.vmin_pub = self.create_publisher(Int32, '/vcc_min_mv', 10)
        # 엔코더 원시 카운트 — 엔코더 스케일(counts_per_revolution) 검증에 필수.
        # RTK 이동거리와 비교해 1카운트당 실제 거리를 역산한다.
        self.enc_pub = self.create_publisher(Int32, '/encoder_count', 10)
        # 개루프 식별용: 현재 인가 중인 구동 PWM
        self.drive_pwm_pub = self.create_publisher(Int32, '/drive_pwm', 10)
        # 소나 유효 최대거리[m]. NewPing은 미검출 시 0을 주므로 그대로 쓰면
        # '장애물 0m'로 오인해 급정지한다 → 미검출은 이 값(=없음)으로 변환.
        self.declare_parameter('sonar_max_range', 2.0)
        self.sonar_max = float(self.get_parameter('sonar_max_range').value)
        # ★ 2026-08-19: 소나는 장애물 회피 센서가 아니다(라이다가 담당). 소나의
        # /obstacle_distance 발행은 기본 OFF — 켜면 라이다와 충돌하고, 과거엔
        # trigger>range 불일치로 빈 트랙을 상시 감속시켰다(랩 10분 버그). 소나로만
        # 근접정지를 쓰고 싶을 때만 publish_sonar_obstacle:=true.
        self.publish_sonar_obstacle = bool(
            self.declare_parameter('publish_sonar_obstacle', False).value)

        # ---------- 마지막으로 /cmd_vel 을 받은 시각 (워치독 판단용) ----------
        self.last_cmd_time = self.get_clock().now()

        # ---------- 타이머 1: 20Hz로 Arduino에 명령 전송 ----------
        # Arduino 자체 루프는 100Hz(10ms)로 돌지만, 목표값 전송은 그렇게 자주
        # 안 보내도 됨. 20Hz(50ms)면 반응성과 시리얼 부하 사이 적당한 절충점.
        self.stall_clear_timeout = float(
            self.get_parameter('stall_clear_timeout').value)
        self.stall_state = False      # 마지막으로 발행한 값
        self.last_stall_msg = 0.0     # STALL 줄을 마지막으로 본 시각
        self.stall_timer = self.create_timer(0.1, self.stall_check)

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
    def _open_no_reset(self, port, baud):
        """DTR 토글 없이 포트를 연다 → 아두이노가 리셋되지 않는다.

        ★ 이게 어제부터의 리셋 루프의 진짜 해법이다.
        기본 pyserial 은 포트를 열 때 DTR 을 토글하고, 아두이노는 그걸 '리셋'으로
        받는다. 커널 로그(2026-08-18)에 전기적 에러(-71/-110)는 전혀 없고 깨끗한
        USB disconnect→재열거만 3.3→4.3→6.3초(재연결 백오프) 간격으로 찍혔다.
        즉 물리 문제가 아니라, 재연결이 열 때마다 DTR 로 스스로 리셋을 만들어
        그 리셋을 보고 또 재연결하는 자기유발 루프였다.

        해법: 포트를 열기 전에 termios 로 HUPCL(닫을 때 DTR 내림)을 끄고,
        연 직후 DTR/RTS 를 유지한다. 표준 방식이며 sudo 불필요.
        """
        import termios
        # 먼저 파일 디스크립터만 열어 HUPCL 을 끈다(열 때/닫을 때 리셋 방지)
        try:
            import os
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[2] &= ~termios.HUPCL          # c_cflag 에서 HUPCL 제거
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            os.close(fd)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'HUPCL 해제 실패(무시하고 진행): {e}')
        # dsrdtr=False + 열고 나서 DTR 을 능동적으로 유지
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.1
        # ★ exclusive 는 끈다. 아두이노와 u-blox 가 같은 USB 허브에 물리면
        # TIOCEXCL 이 허브 레벨 충돌을 악화시켜 ublox 드라이버가 EBUSY(-16)로
        # 프로브 실패 → segfault → 허브 전체 재열거를 유발했다(2026-08-18 dmesg).
        # DTR 억제(HUPCL 해제)만으로 리셋은 막히고, 중복 노드는 ros_cleanup 로 막는다.
        ser.exclusive = False
        ser.dsrdtr = False
        ser.rtscts = False
        ser.open()
        try:
            ser.dtr = True       # 리셋 없이 통신 유지
            ser.rts = True
        except Exception:  # noqa: BLE001
            pass
        return ser

    def _resolve_port(self, port):
        """포트 결정: 'auto'면 /dev/arduino 우선, 없으면 Arduino Mega(2341:0042)를
        스캔해 찾는다. 특정 포트를 지정하면 그대로 사용(udev 없이도 동작)."""
        if port and port != 'auto':
            return port
        if os.path.exists('/dev/arduino'):
            return '/dev/arduino'
        # ★ VID/PID 스캔을 몇 번 재시도한다.
        # 아두이노가 막 재열거된 직후엔 comports() 목록에 아직 안 떠서 한 번에
        # 못 찾고 죽는 일이 있었다(2026-08-18: /dev/arduino 없음으로 폴백 →
        # 그 링크도 없어 SerialException). 2341:0042 는 확실히 존재하므로
        # 잠깐 기다렸다 다시 스캔하면 잡힌다.
        for attempt in range(5):
            for p in list_ports.comports():
                if p.vid == 0x2341 and p.pid == 0x0042:   # Arduino Mega 2560
                    self.get_logger().info(f'Arduino Mega 자동감지: {p.device}')
                    return p.device
            time.sleep(1.0)
            self.get_logger().warn(
                f'Arduino 자동감지 재시도 {attempt + 1}/5...')
        # 마지막 폴백: /dev/ttyACM* 중 아무거나(u-blox 도 ACM 이므로 위험하지만
        # 스캔이 5회 실패했다면 최후의 수단). 없으면 예외로 죽는 게 낫다.
        import glob
        acms = sorted(glob.glob('/dev/ttyACM*'))
        if acms:
            self.get_logger().error(
                f'VID/PID 스캔 5회 실패 — {acms[-1]} 로 시도(마지막 수단). '
                '틀리면 포트를 -p port:=/dev/ttyACMx 로 직접 지정할 것.')
            return acms[-1]
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
    def _odom_cb(self, msg):
        """측정 속도 — 제동을 **언제 멈출지** 판단하는 유일한 근거다.

        ⚠ 이 토픽은 IMU 가 끊기면 같이 멈춘다(9/15 에 32번 났다). 그때는
          self._meas_v 가 낡은 값으로 남으므로 **시간 상한만 믿는다**.
          아래 _brake_pwm() 에서 수신 시각을 같이 본다.
        """
        self._meas_v = float(msg.twist.twist.linear.x)
        self._meas_v_t = self.get_clock().now().nanoseconds * 1e-9

    # 제동시간 기준속도에 쓰는 측정속도의 조건.
    #   신선도 0.5s — 해제조건 ② 와 같은 값. IMU 가 끊기면 낡은 값이 남는다.
    #   타당성 3.0m/s — local_pure_pursuit 의 max_plausible_speed 와 같은 값.
    #     /odometry/filtered 의 twist 는 EKF 프레임 지연으로 튄다(실측 61.9m/s).
    BRAKE_MEAS_FRESH_S = 0.5
    BRAKE_V_PLAUSIBLE = 3.0

    def _governor_pwm(self, cmd_v, now):
        """내리막 과속을 깎는 역 PWM. 개입 안 하면 None.

        측정속도가 명령보다 gov_deadband 넘게 빠를 때만 작동하고, 초과분에
        비례해 역방향 PWM 을 낸다. 명령을 추종하는 게 아니라 **과속만 깎는다**.

        안전:
          · gov_pwm 으로 상한 (플러깅 전류 제한)
          · 측정이 0.5s 넘게 낡으면 개입 안 함 (IMU 끊기면 옛 값이 남는다)
          · 3.0m/s 초과 측정은 무시 (odom twist 가 61.9m/s 까지 튄 실측)
          · 명령과 측정의 **부호가 같을 때만** — 후진 중 전진명령 같은
            상황에서 엉뚱한 방향으로 밀지 않는다
        """
        if self.gov_pwm <= 0.0 or abs(cmd_v) < self.ff_deadband:
            return None
        if self._meas_v is None or (now - self._meas_v_t) >= self.BRAKE_MEAS_FRESH_S:
            return None
        mv = self._meas_v
        if abs(mv) > self.BRAKE_V_PLAUSIBLE:
            return None
        if (mv > 0) != (cmd_v > 0):          # 부호가 다르면 개입 안 한다
            return None
        excess = abs(mv) - abs(cmd_v) - self.gov_deadband
        if excess <= 0.0:
            return None
        mag = min(self.gov_pwm, self.gov_gain * excess)
        self.get_logger().info(
            f'거버너 — 측정 {abs(mv):.2f} > 명령 {abs(cmd_v):.2f} '
            f'(+{excess + self.gov_deadband:.2f}) → 역 PWM {mag:.0f}',
            throttle_duration_sec=1.0)
        return -math.copysign(mag, cmd_v)

    def _brake_pwm(self, cmd_v, now):
        """능동 제동 상태기계. 제동 중이면 인가할 PWM, 아니면 None.

        진입:  직전에 ff_brake_trigger_v 이상으로 달리다가
               이번 주기에 '멈추라'(|cmd_v| < deadband) 가 온 순간
        해제:  ① 시간 상한 초과  ② 측정속도가 ff_brake_min_v 밑
               ③ 다시 가라는 명령이 옴
        """
        if self.ff_brake_pwm <= 0.0:
            return None

        stop_cmd = abs(cmd_v) < self.ff_deadband
        was_fast = abs(self._last_cmd_v) >= self.ff_brake_trigger_v

        # ① 진입 판정
        if self._brake_until is None:
            if stop_cmd and was_fast:
                self._brake_sign = 1 if self._last_cmd_v > 0 else -1
                # ★ 2026-09-17 — 제동시간의 기준속도를 **측정속도로 올린다**.
                #   명령속도는 실제보다 낮다. ff_min_pwm 55 하한이 낮은 명령을
                #   전부 덮어쓰기 때문이다(drive_0152 실측: 명령 0.70 / 실제 1.12).
                #   명령으로 재면 dur = 0.70s 인데 실제로 필요한 건 1.12s 분이다.
                #
                #   ⚠ 평지에서는 이 수정이 거의 무의미하다 — 해제조건 ②(측정속도
                #     0.15 미만)가 먼저 끊기 때문이다. drive_0152 를 위치미분으로
                #     보면 차는 t+0.75s 에 이미 섰고 제동은 0.70s 에 끝났다.
                #     (odom twist 로 보면 0.90s 인데 그건 **약 0.2s 지연**이다.
                #      인계문서의 '마지막 400ms 는 관성' 은 그 지연 아티팩트다.)
                #
                #   그럼 왜 고치나 — **내리막** 때문이다. 내리막에서는 실제속도가
                #   명령보다 훨씬 높고 감속도 느려서, 명령으로 잰 dur 이 차가 아직
                #   구르는 중에 제동을 끊는다(②는 아직 안 걸린 상태). 용인에
                #   내리막이 있다. 돌발정지가 거기서 걸리면 이게 차이를 만든다.
                #
                #   안전: **올리기만 하고 내리지 않는다.** 측정속도가 명령보다
                #   낮게 잘못 나와도 제동이 짧아지지 않는다(급정지 실패 방향).
                #   스파이크는 무시한다(odom twist 가 61.9m/s 까지 튄 실측이 있다).
                #   그래도 최종값은 ff_brake_ms 상한과 해제조건 ②가 잡는다.
                v_ref, src = abs(self._last_cmd_v), '명령'
                if (self._meas_v is not None
                        and (now - self._meas_v_t) < self.BRAKE_MEAS_FRESH_S):
                    mv = abs(self._meas_v)
                    if self.BRAKE_V_PLAUSIBLE >= mv > v_ref:
                        v_ref, src = mv, '측정'
                dur = min(self.ff_brake_s, v_ref / self.ff_brake_decel)
                self._brake_until = now + dur
                self.get_logger().info(
                    f'제동 — 역 PWM {self.ff_brake_pwm:.0f} 을 '
                    f'{dur * 1000:.0f}ms ({src}속도 {v_ref:.2f}m/s, '
                    f'직전명령 {self._last_cmd_v:+.2f}m/s, '
                    f'상한 {self.ff_brake_s * 1000:.0f}ms)')
            else:
                return None

        # ③ 다시 가라는 명령이면 즉시 해제
        if not stop_cmd:
            self._brake_until = None
            return None

        # ① 시간 상한
        if now >= self._brake_until:
            self._brake_until = None
            return None

        # ② 측정속도 — **신선할 때만** 믿는다(IMU 끊기면 낡은 값이 남는다)
        if self._meas_v is not None and (now - self._meas_v_t) < 0.5:
            if abs(self._meas_v) < self.ff_brake_min_v:
                self._brake_until = None
                self.get_logger().info(
                    f'제동 해제 — 측정속도 {self._meas_v:+.2f}m/s '
                    f'({self.ff_brake_min_v:.2f} 미만)')
                return None

        return -self._brake_sign * self.ff_brake_pwm

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
        elif self.ff_mode == 'ros':
            # ROS 쪽 FF — 실측 상수로 직접 PWM 을 만든다(위 주석 참고).
            v = float(self.target_vel)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            brake = self._brake_pwm(v, now_s)
            if abs(v) < self.ff_deadband:
                pwm = 0 if brake is None else int(round(brake))
                self._moving_since = None      # 섰다 — 다음 출발은 다시 breakaway
            else:
                gov = self._governor_pwm(v, now_s)
                if gov is not None:
                    # ★ 거버너가 개입하면 **하한(ff_min_pwm)을 건너뛴다.**
                    #   안 그러면 '과속이니 멈춰' 와 '최소한 이만큼은 밀어' 가
                    #   싸워서 내리막에서 계속 가속한다.
                    self._last_cmd_v = v
                    cmd = f'PWM:{int(round(gov))},STEER:{self.target_steer:.1f}\n'
                    try:
                        self.ser.write(cmd.encode('utf-8'))
                    except Exception as e:  # noqa: BLE001
                        self.get_logger().error(f'시리얼 전송 실패: {e}',
                                                throttle_duration_sec=2.0)
                    return
                mag = self.ff_static + self.ff_gain * abs(v)
                mag = max(mag, self.ff_min_pwm)   # 크리프 하한(위 주석 참고)
                # 정지마찰 구간: 정지에서 막 출발했으면 잠깐 더 세게 민다.
                now = self.get_clock().now().nanoseconds * 1e-9
                if self._moving_since is None:
                    self._moving_since = now
                    self.get_logger().info(
                        f'출발 — 정지마찰 하한 {self.ff_breakaway_pwm:.0f} 을 '
                        f'{self.ff_breakaway_s * 1000:.0f}ms 인가한다',
                        throttle_duration_sec=2.0)
                if now - self._moving_since < self.ff_breakaway_s:
                    mag = max(mag, self.ff_breakaway_pwm)
                pwm = int(round(math.copysign(min(mag, 255.0), v)))
            # ⚠ **양쪽 분기 뒤에서** 갱신해야 한다. 정지 분기에서 안 갱신하면
            #   _last_cmd_v 가 옛 주행속도를 계속 들고 있어, 제동이 끝난 뒤에도
            #   was_fast 가 참이라 **매 주기 재진입**한다(서 있는 차가 계속
            #   역토크를 받는다). 여기서 갱신하면 정지 다음 주기에 0 이 되어
            #   다시 달리기 전까지는 트리거가 안 걸린다.
            self._last_cmd_v = v
            cmd = f'PWM:{pwm},STEER:{self.target_steer:.1f}\n'
        else:
            cmd = f'VEL:{self.target_vel:.2f},STEER:{self.target_steer:.1f}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
        except Exception as e:  # noqa: BLE001
            # 읽기 스레드가 _reconnect 로 복구하는 중일 수 있다. 여기서 같이
            # 재연결을 시도하면 두 스레드가 포트를 두고 싸우므로 로그만 남긴다.
            # 재연결이 끝나면 다음 주기부터 자동으로 다시 나간다.
            self.get_logger().error(f'시리얼 전송 실패: {e}',
                                    throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def stall_check(self):
        """STALL 줄이 stall_clear_timeout 동안 안 오면 해제로 본다.

        펌웨어가 해제 신호를 안 주기 때문에 '안 오는 것' 으로 판단할 수밖에 없다.
        텔레메트리가 20Hz 라 0.5s 면 10주기 — 충분히 여유 있다.
        """
        if not self.stall_state:
            return
        if time.time() - self.last_stall_msg > self.stall_clear_timeout:
            self.stall_state = False
            self.stall_pub.publish(Bool(data=False))
            self.get_logger().info(
                f'스톨 해제 ({self.stall_clear_timeout:.1f}s 간 STALL 없음)')

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
    def _reconnect(self, err):
        """시리얼이 끊겼을 때 포트를 다시 찾아 연결한다.

        아두이노가 리셋되면 USB 장치번호와 /dev/ttyACM* 번호가 함께 바뀌므로
        원래 경로를 다시 열어봐야 소용없다. _resolve_port 로 재탐색한다.
        """
        self._reconnects += 1
        # ★ 백오프. 포트를 여는 것 자체가 DTR 로 아두이노를 리셋시키므로,
        # 재연결을 빠르게 반복하면 **우리가 리셋 루프를 유지하게 된다.**
        # 2026-08-17 현장에서 3.3초 주기로 29회 반복됐다(연결 → 0.32초 뒤 끊김).
        # 연속 실패가 쌓이면 간격을 늘려 보드가 스스로 안정될 시간을 준다.
        now = time.time()
        if now - getattr(self, '_last_reconnect_t', 0.0) < 10.0:
            self._fail_streak = getattr(self, '_fail_streak', 0) + 1
        else:
            self._fail_streak = 0
        self._last_reconnect_t = now
        wait = min(1.0 * (2 ** min(self._fail_streak, 4)), 16.0)
        self.get_logger().error(
            f'시리얼 끊김({err}) — 재연결 시도 #{self._reconnects} '
            f'(연속 실패 {self._fail_streak}회, {wait:.0f}초 대기)',
            throttle_duration_sec=2.0)
        if self._fail_streak >= 3:
            self.get_logger().error(
                '⚠ 재연결이 반복된다. 아두이노가 부팅 직후 리셋되는 상황일 수 '
                '있다(조향이 중앙에서 크게 벗어나 있으면 부팅 시 조향모터가 '
                '전류를 끌어 브라운아웃). 앞바퀴를 중앙으로 맞추고 전원을 확인할 것.',
                throttle_duration_sec=15.0)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(wait)
        try:
            port = self._resolve_port(self._port_param)
            ser = self._open_no_reset(port, self._baud)
            time.sleep(2.0)          # 아두이노 부트로더 대기
            ser.reset_input_buffer()
            self.ser = ser
            self.get_logger().warn(
                f'✅ 시리얼 재연결 성공: {port} (총 {self._reconnects}회)')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f'재연결 실패 ({e}) — 1초 뒤 재시도', throttle_duration_sec=5.0)
            time.sleep(1.0)

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
                # ★ 아두이노가 USB 재열거되면 열어둔 포트가 죽고 여기로 떨어진다.
                # 예전엔 continue 만 해서 초당 수천 줄 에러를 뿌리며 영영 복구되지
                # 않았다(2026-08-17 현장: Device 012→015→016→017 로 4회 재열거,
                # 포트도 ttyACM0↔ttyACM1 로 바뀜). 주행 중 한 번 나면 랩이 끝난다.
                # → 포트를 다시 탐색해서 재연결한다. 재연결까지 차는 펌웨어
                #   워치독(0.5s)으로 이미 정지 상태이므로 안전하다.
                self._reconnect(e)
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
                    # 미검출(0.00) = '장애물 없음'.
                    # ★ 2026-08-19 버그 수정 (랩 10분의 주범):
                    # 이걸 sonar_max(2.0m)로 치환하면 longitudinal 의
                    # obstacle_trigger(4.0m)보다 작아, 빈 트랙인데도 '2m 앞 장애물'로
                    # 오인돼 장애물 감속이 상시 걸렸다. v_obs = 0.8*(2.0-0.8)/(4.0-0.8)
                    # = 0.3 m/s(1.08km/h)로 전 구간이 묶여 190m 를 10분에 기었다.
                    # '없음'은 트리거보다 확실히 큰 CLEAR 로 발행해야 감속이 안 걸린다.
                    # 실제 반향(d>0.01)은 그대로 써서 진짜 장애물 감속은 유지한다.
                    if self.publish_sonar_obstacle:
                        CLEAR = 999.0
                        ds = []
                        for s in (s1, s2, s3):
                            d = float(s)
                            ds.append(CLEAR if d <= 0.01 else d)
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
                # ★ 공급전압. 펌웨어는 계속 보내고 있었는데 파싱하지 않아
                # 토픽으로 나오지 않았다. 주행 중 전원이 꺼지는 원인을
                # (배터리 sag / 브라운아웃) 판별하려면 이 값이 필요하다.
                # VMIN 은 직전 텔레메트리 구간의 **최솟값**이라 순간 강하를 잡는다.
                # ⚠ 절대값은 모터 PWM 노이즈에 오염될 수 있다(밴드갭 ADC).
                #   'VMIN 이 갑자기 낮아졌다'는 추세로 보고, 리셋 발생 여부와
                #   함께 판단할 것.
                if 'VCC=' in line:
                    self.vcc_pub.publish(
                        Int32(data=int(line.split('VCC=')[1].split()[0])))
                if 'VMIN=' in line:
                    vmin = int(line.split('VMIN=')[1].split()[0])
                    self.vmin_pub.publish(Int32(data=vmin))
                    if 0 < vmin < 4300:
                        self.get_logger().warn(
                            f'⚠ 공급전압 강하: VMIN={vmin}mV — 배터리/전원 확인',
                            throttle_duration_sec=3.0)
            elif line.startswith('STALL:'):
                drive = line.split('drive=')[1].split()[0].strip()
                steer = line.split('steer=')[1].split()[0].strip()
                stalled = (drive not in ('0',)) or (steer not in ('0',))
                if stalled:
                    self.last_stall_msg = time.time()
                    if not self.stall_state:
                        self.stall_state = True
                        self.stall_pub.publish(Bool(data=True))
                    self.get_logger().warn(
                        f'펌웨어 스톨 감지: {line}', throttle_duration_sec=2.0)
                else:
                    # drive=0 steer=0 을 명시적으로 준 경우 — 즉시 해제
                    self.last_stall_msg = 0.0
                    if self.stall_state:
                        self.stall_state = False
                        self.stall_pub.publish(Bool(data=False))
                        self.get_logger().info('스톨 해제')
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
        # Ctrl-C 시 시그널 핸들러가 이미 컨텍스트를 닫아둔 경우가 있다.
        # 그때 다시 부르면 RCLError 가 나면서 **정상 종료가 크래시처럼 보인다**
        # (현장에서 오진하기 쉽다). 이미 닫혔으면 조용히 넘어간다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

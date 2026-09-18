#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
control.launch.py — 종/횡방향 제어 체인 (실제 모터까지 나감).

  local_pure_pursuit      : /local_path       → /steering_cmd (조향각[도])
  longitudinal_controller : /curvature 등     → /target_speed (m/s)
  vehicle_cmd_mux         : 위 둘 + teleop/E-stop → /cmd_vel  (단일 출구)
  serial_bridge           : /cmd_vel          → "VEL:x,STEER:y" → Arduino

★ 역할 분담: ROS는 '목표 속도·조향각'까지만 책임지고, PWM 변환과 스톨가드는
  펌웨어가 담당한다. ROS가 raw PWM을 쏘면 펌웨어 안전가드가 무력화된다.

⚠ 이 런치는 **모터를 실제로 구동**한다. bringup에는 control:=true 로만 붙는다.

  ros2 launch pure_pursuit_pkg control.launch.py
  ros2 launch pure_pursuit_pkg control.launch.py max_speed:=0.6   # 더 보수적으로

전제: bringup(로컬라이제이션+경로)이 이미 돌아 /local_path·/curvature 발행 중.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  pp_share = get_package_share_directory('pure_pursuit_pkg')
  default_params = os.path.join(pp_share, 'config', 'pure_pursuit_params.yaml')

  pp_params = LaunchConfiguration('pp_params')
  arduino_port = LaunchConfiguration('arduino_port')
  teleop = LaunchConfiguration('teleop')
  # ★ ROS 쪽 FF (2026-09-12) — 펌웨어를 다시 굽지 않고 구동 상수를 고친다.
  #   펌웨어 FF(80 + 95·v)는 지면 실측과 4배 어긋나 있다. 0.15 m/s 명령이
  #   PWM 94 로 나가 차가 2 m/s 로 달렸고, 그래서 헤딩 캘리브가 계속 깨졌다.
  #   실측(tools/ff_identify.py, 학교 아스팔트): PWM = 38.8·v + 17.2
  #   펌웨어를 올바른 상수로 다시 구웠다면 ff_mode:=firmware 로 되돌릴 것.
  ff_mode = LaunchConfiguration('ff_mode')

  # ★ 타입 강제 (2026-09-09)
  #   런치 인자는 문자열이고 launch_ros 가 YAML 로 타입을 추론한다. 그래서
  #   `max_steer_deg:=15` 는 INTEGER 로 들어가는데 노드는 declare_parameter(
  #   'max_steer_deg', 18.0) 으로 DOUBLE 을 기대해 InvalidParameterTypeException
  #   으로 즉사한다(vehicle_cmd_mux/serial_bridge 가 실제로 이렇게 죽었다).
  #   ParameterValue(value_type=float) 로 감싸면 `15` 도 `15.0` 도 통과한다.
  max_speed = ParameterValue(LaunchConfiguration('max_speed'), value_type=float)
  max_steer = ParameterValue(LaunchConfiguration('max_steer_deg'),
                             value_type=float)
  # ★ 코너 감속 세기 (2026-09-10 추가).
  #   longitudinal_controller 가 init 에서 값을 캐시하므로 `ros2 param set` 으로는
  #   못 바꾼다. 재빌드 없이 현장에서 조정하려면 런치 인자여야 한다.
  # 정수로 주면 노드가 즉사하므로 반드시 float 로 강제한다(런치 인자 함정).
  ff_static = ParameterValue(LaunchConfiguration('ff_static'), value_type=float)
  ff_gain = ParameterValue(LaunchConfiguration('ff_gain'), value_type=float)
  curv_gain = ParameterValue(LaunchConfiguration('curvature_gain'),
                             value_type=float)

  # teleop_keyboard 는 키 입력을 받아야 하므로 자체 터미널이 필요하다(xterm).
  # xterm 이 없으면 런치가 조용히 실패한다 — 현장에서 '왜 키가 안 먹지'로
  # 헤매게 되므로 여기서 미리 알린다. 없을 땐 별도 터미널에서 직접 실행할 것:
  #   ros2 run velocity_controller teleop_keyboard
  teleop_prefix = 'xterm -e' if shutil.which('xterm') else ''
  if not teleop_prefix:
    print('[control] ⚠ xterm 이 없다. teleop:=true 는 키 입력을 못 받는다.\n'
          '[control]   별도 터미널에서 실행할 것: '
          'ros2 run velocity_controller teleop_keyboard')

  return LaunchDescription([
      DeclareLaunchArgument('pp_params', default_value=default_params),
      DeclareLaunchArgument('arduino_port', default_value='auto'),
      # 구동모터 벤치검증 전이라 보수적 기본값. 검증 후 상향할 것.
      # ⚠ max_speed 는 **실제로 도달 가능한 속도**여야 한다.
      #   곡률 감속식이 v = max_speed/(1+gain·|κ|) 라서, max_speed 를 하드웨어
      #   상한보다 높게 주면 분자만 커져 **코너 목표속도만 올라간다**(직진은
      #   어차피 펌웨어가 자른다). 즉 코너 감속이 조용히 무력화된다.
      #   실측(2026-09-10 시뮬): max_speed 2.8 + PWM160(실제 0.84) 이면
      #   코스 전 구간을 0.84 로 통과한다 = gain 이 없는 것과 같다.
      #   도달속도 = (MAX_DRIVE_PWM − 80) / 95   (펌웨어 개루프 FF 실측값)
      #     PWM 160 → 0.84 · PWM 200 → 1.26 · PWM 255 → 1.84
      DeclareLaunchArgument('max_speed', default_value='1.0'),
      DeclareLaunchArgument('max_steer_deg', default_value='18.0'),
      # 코너 감속 세기. 낮출수록 빠르고 경로에서 더 벌어진다.
      #   판단표: python3 tools/lap_budget.py --waypoints <코스파일>
      DeclareLaunchArgument('curvature_gain', default_value='6.0'),
      # teleop:=true 면 키보드 수동 제어 노드도 함께 띄운다(먹스에서 사람 우선).
      DeclareLaunchArgument('teleop', default_value='false'),
      # 기본값을 'ros' 로 둔다 — 현재 보드에 구워져 있는 펌웨어 FF 가 틀렸고,
      # 그대로 두면 차가 명령의 4배 속도로 달린다(위 주석 참고).
      DeclareLaunchArgument('ff_mode', default_value='ros',
                            choices=['ros', 'firmware']),
      DeclareLaunchArgument('ff_static', default_value='17.2'),
      DeclareLaunchArgument('ff_gain', default_value='38.8'),
      # ★ 크리프 하한 (2026-09-13 노출). serial_bridge 가 init 에서 캐시하므로
      #   런치 인자가 아니면 현장에서 못 바꾼다.
      #   ⚠ 이 값이 곧 **차의 최저 속도**다. PWM = 38.8·v + 17.2 를 뒤집으면
      #     50 → 0.85 m/s 다. max_speed 0.7 을 줘도 PWM 44 가 50 으로 올라가
      #     **max_speed 가 통째로 무력화된다.** 커브 감속(v/(1+6|κ|))도 같이
      #     무력화된다 — 어느 명령이든 PWM 은 50 이다.
      #   ⚠ 더 나쁜 것: 장애물 감속 램프(4.0m→0.8m)도 무력화된다. 차는
      #     하한 속도 그대로 다가가다 0.8m 에서 동력만 끊는다. 브레이크가
      #     없으므로 그 뒤는 관성이다(0.85 m/s → 1.35m).
      #   그럼에도 하한이 필요한 이유는 커브에서 바퀴가 멈추면 '1분 정지 =
      #   탈락' 이기 때문이다. 낮추려면 **정지마찰을 넘는 최소값**을 실측할 것.
      #   ※ PWM 50 의 실제 속도는 아직 실측이 없다. 45→0.00, 56→1.00 사이의
      #     스틱슬립 구간이라 직선 적합(0.85)을 그대로 믿으면 안 된다.
      #     캘리브 10m 주행이 그 자체로 PWM 50 속도계다(명령 0.5 → PWM 36.6
      #     → 하한 50). 진행 로그의 소요시간으로 환산할 것.
      DeclareLaunchArgument('ff_min_pwm', default_value='50.0'),
      # ★ 정지마찰 하한 — '구르는 중' 하한(ff_min_pwm)과 다른 값이어야 한다.
      #   9/13 실측: PWM 50 은 출발을 못 하고(모터 잠김, VMIN 2938mV),
      #   56 이면 구르고, 60 이면 1.07 m/s. 문턱이 55~56 사이다.
      DeclareLaunchArgument('ff_breakaway_pwm', default_value='60.0'),
      DeclareLaunchArgument('ff_breakaway_ms', default_value='1200.0'),
      # ★ 능동 제동 — **기본 0 = 꺼짐**. 돌발정지에서 관성거리를 줄인다.
      #   `PWM 0` 은 동력을 끊을 뿐이라 그 뒤는 관성이다(9/15 실측 제동거리
      #   1.96m, 더미 0.30m 앞에서 v=0.85m/s). 역토크를 걸면 1.15m/s 에서
      #   1.18m → 0.4~0.55m 로 준다(펌웨어 램프 200~400ms 감안).
      #   ⚠ 엔코더가 없어 정지를 직접 못 본다. 측정속도(/odometry/filtered)와
      #     시간상한이 안전장치다. 검증 전에는 켜지 말 것.
      DeclareLaunchArgument('ff_brake_pwm', default_value='0.0'),
      DeclareLaunchArgument('ff_brake_ms', default_value='800.0'),
      # ★ 장애물 정지거리 (2026-09-13 노출). 브레이크가 없어서 이 값은
      #   '멈출 거리' 가 아니라 '동력을 끊을 거리' 다. 실제 정지점은
      #   여기서 관성거리(1.384·v^1.506)만큼 더 간다.
      DeclareLaunchArgument('obstacle_stop_dist', default_value='0.8'),

      # ── 경사로 구간 속도 (기본 안 씀) ──
      # 시퀀서가 이 토픽으로 arm 을 내는 동안 기준속도가 바뀐다.
      # 켜려면 config/mission_plan.yaml 의 ramp_up/ramp_down 을
      # enabled:true 로 하고 s 를 실측해 넣은 뒤, 여기에 토픽·속도를 준다.
      #   ros2 launch ... ramp_up_arm_topic:=/ramp/up_arm v_ramp_up:=1.6 \
      #                   ramp_down_arm_topic:=/ramp/down_arm v_ramp_down:=1.11
      DeclareLaunchArgument('ramp_up_arm_topic', default_value=''),
      DeclareLaunchArgument('ramp_down_arm_topic', default_value=''),
      # ⚠ 정수로 주면 INTEGER 로 들어가 노드가 즉사한다 — 반드시 소수점.
      DeclareLaunchArgument('v_ramp_up', default_value='0.0'),
      DeclareLaunchArgument('v_ramp_down', default_value='0.0'),

      # ── 경사로: 거버너 · IMU 경사 보상 (serial_bridge) ─────────────
      # ★ 구간 속도(v_ramp_up/down)만으로는 경사로가 안 된다.
      #   구간 속도는 '어디서 얼마의 속도를 낼지' 만 정한다. 그 속도를 내려면
      #   PWM 이 얼마여야 하는지는 개루프 FF 가 모른다 — 평지에서 식별한
      #   식이라 중력이 안 들어 있다. 그래서 셋이 같이 필요하다:
      #       구간 속도  어디서 얼마로        (여기, longitudinal)
      #       grade_ff   그러려면 PWM 얼마    (아래, serial_bridge · IMU 피치)
      #       거버너     그래도 빨라지면 깎음 (아래, serial_bridge · 엔코더)
      #
      #   근거: tools/ramp_profile.py --sweep 6,8,10,12.5,15,19.3
      #     grade_ff 꺼짐 → 12.5% 에서 등반 0.37 m/s, 구동 15% 약화면 못 넘음
      #     grade_ff 켬   → 19.3% 까지 등반 1.24 m/s 이상
      #
      # ⚠ grade_ff 는 **부호를 현장에서 확정한 뒤에만** 켠다. 반대면
      #   내리막에서 가속한다(시뮬에서 오르막 −0.19 m/s = 뒤로 밀림).
      #       python3 tools/imu_grade.py --level                 (평지 영점)
      #       python3 tools/imu_grade.py --measure --expect 12.5 (경사 부호)
      #   절차 전체는 RAMP_RUNBOOK.md.
      #
      # 기본값은 노드 기본값과 같다 = 전부 꺼짐. 안 주면 예전과 똑같이 돈다.
      DeclareLaunchArgument('gov_pwm', default_value='0.0'),
      DeclareLaunchArgument('gov_deadband', default_value='0.10'),
      DeclareLaunchArgument('gov_gain', default_value='300.0'),
      DeclareLaunchArgument('gov_lead_s', default_value='0.30'),
      # 거버너 자세 게이트 — sinθ 기준 내리막 경사. 0 = 판정 안 함(예전 동작).
      # 0.03 이면 3% 보다 급한 내리막에서만 거버너가 일한다.
      DeclareLaunchArgument('gov_min_grade', default_value='0.0'),
      DeclareLaunchArgument('grade_ff_gain', default_value='0.0'),
      DeclareLaunchArgument('grade_ff_max', default_value='70.0'),
      DeclareLaunchArgument('imu_topic', default_value='handsfree/imu'),

      # ★ 2026-09-16 — 회피 override 유지시간 / 이탈 상한 (먹스).
      #   근거와 실측은 vehicle_cmd_mux_node.py 의 avoid_hold_s 주석 참고.
      #   셋 다 기본 0 = 꺼짐. drive_0337 은 이 기능들 없이 완주했다 —
      #   검증된 동작을 기본값으로 바꾸지 않는다.
      #   ⚠ 이탈 상한(avoid_max_*)은 **유지시간과 같이** 쓸 것. 유지시간만
      #     늘리면 회피각을 더 오래 물고 있어 이탈이 커진다.
      #   권장 시험값:  avoid_hold_s:=0.25
      #                 avoid_max_lateral_m:=0.85  avoid_max_heading_deg:=22.0
      DeclareLaunchArgument('avoid_hold_s', default_value='0.0'),
      DeclareLaunchArgument('avoid_max_lateral_m', default_value='0.0'),
      DeclareLaunchArgument('avoid_max_heading_deg', default_value='0.0'),

      # 횡방향: 조향각만 발행 (속도는 종방향이 소유)
      Node(
          package='pure_pursuit_pkg',
          executable='local_pure_pursuit_node',
          name='local_pure_pursuit',
          output='screen',
          parameters=[pp_params, {'standalone': False}],
      ),

      # 종방향: 미션/곡률/장애물 종합 → 목표속도
      Node(
          package='velocity_controller',
          executable='longitudinal_controller',
          name='longitudinal_controller',
          output='screen',
          parameters=[{
              'v_max': max_speed,
              'curvature_gain': curv_gain,
              'obstacle_stop_dist': ParameterValue(
                  LaunchConfiguration('obstacle_stop_dist'),
                  value_type=float),
              # ── 경사로 구간 속도 ──
              # 기본은 빈 토픽 + 0.0 = **안 씀**. 시퀀서가 arm 을 내는
              # 구간에서 기준속도만 바뀌고, 곡률·정지선·장애물 감속은
              # 그대로 걸린다(tools/test_ramp_section.py 가 못 박는다).
              # ⚠ 내리막은 명령만 낮춰서는 안 선다 — 중력이 이긴다.
              #   serial_bridge 의 gov_pwm(거버너)·grade_ff 를 같이 켤 것.
              'ramp_up_arm_topic': ParameterValue(
                  LaunchConfiguration('ramp_up_arm_topic'), value_type=str),
              'ramp_down_arm_topic': ParameterValue(
                  LaunchConfiguration('ramp_down_arm_topic'), value_type=str),
              'v_ramp_up': ParameterValue(
                  LaunchConfiguration('v_ramp_up'), value_type=float),
              'v_ramp_down': ParameterValue(
                  LaunchConfiguration('v_ramp_down'), value_type=float)}],
      ),

      # 명령 먹스: 단일 /cmd_vel 출구 + 최종 안전 클램프
      Node(
          package='velocity_controller',
          executable='vehicle_cmd_mux',
          name='vehicle_cmd_mux',
          output='screen',
          # ⚠ 전부 ParameterValue(float) 로 감싼다. `avoid_hold_s:=0` 처럼
          #   정수로 주면 INTEGER 로 들어가 노드가 즉사한다(런치 인자 함정).
          parameters=[{
              'max_speed': max_speed,
              'max_steer_deg': max_steer,
              'avoid_hold_s': ParameterValue(
                  LaunchConfiguration('avoid_hold_s'), value_type=float),
              'avoid_max_lateral_m': ParameterValue(
                  LaunchConfiguration('avoid_max_lateral_m'),
                  value_type=float),
              'avoid_max_heading_deg': ParameterValue(
                  LaunchConfiguration('avoid_max_heading_deg'),
                  value_type=float),
          }],
      ),

      # 시리얼 브리지: /cmd_vel → Arduino, 텔레메트리 → 구조화 토픽
      Node(
          package='velocity_controller',
          executable='serial_bridge',
          name='serial_bridge_node',
          output='screen',
          parameters=[{
              'port': arduino_port,
              'baud': 57600,
              'watchdog_timeout': 0.5,
              'max_steer_deg': max_steer,
              'ff_mode': ff_mode,
              'ff_static': ff_static,
              'ff_gain': ff_gain,
              'ff_min_pwm': ParameterValue(
                  LaunchConfiguration('ff_min_pwm'), value_type=float),
              'ff_breakaway_pwm': ParameterValue(
                  LaunchConfiguration('ff_breakaway_pwm'), value_type=float),
              'ff_breakaway_ms': ParameterValue(
                  LaunchConfiguration('ff_breakaway_ms'), value_type=float),
              'ff_brake_pwm': ParameterValue(
                  LaunchConfiguration('ff_brake_pwm'), value_type=float),
              'ff_brake_ms': ParameterValue(
                  LaunchConfiguration('ff_brake_ms'), value_type=float),
              # ── 경사로 (2026-09-18 노출) ───────────────────────────
              # ★ 왜 여기 추가했나
              #   거버너(gov_*)와 경사 보상(grade_ff_*)은 지금까지
              #   tools/teleop_drive.launch.py 에만 있었다. 그래서 **자율
              #   주행으로는 켤 방법이 아예 없었다.** 경사로를 웨이포인트로
              #   달리려면 이 둘이 필요한데(tools/ramp_profile.py: 12.5%
              #   에서 grade_ff 없이는 구동이 15% 만 약해져도 못 넘는다),
              #   런치에 인자가 없으니 teleop 으로만 시험할 수 있었다.
              #
              # ⚠ 기본값은 **노드 기본값과 같다**(gov_pwm 0 = 꺼짐,
              #   grade_ff_gain 0 = 꺼짐). 인자를 안 주면 이 블록이
              #   추가되기 전과 **완전히 같은 동작**이다.
              #
              # ⚠ 전부 float 강제 — `gov_pwm:=50` 은 INTEGER 로 들어가
              #   노드가 즉사한다(런치 인자 함정).
              'gov_pwm': ParameterValue(
                  LaunchConfiguration('gov_pwm'), value_type=float),
              'gov_deadband': ParameterValue(
                  LaunchConfiguration('gov_deadband'), value_type=float),
              'gov_gain': ParameterValue(
                  LaunchConfiguration('gov_gain'), value_type=float),
              'gov_lead_s': ParameterValue(
                  LaunchConfiguration('gov_lead_s'), value_type=float),
              'gov_min_grade': ParameterValue(
                  LaunchConfiguration('gov_min_grade'), value_type=float),
              'grade_ff_gain': ParameterValue(
                  LaunchConfiguration('grade_ff_gain'), value_type=float),
              'grade_ff_max': ParameterValue(
                  LaunchConfiguration('grade_ff_max'), value_type=float),
              'imu_topic': ParameterValue(
                  LaunchConfiguration('imu_topic'), value_type=str),
          }],
      ),

      # (선택) 키보드 수동 제어 — 먹스에서 자율보다 우선
      Node(
          package='velocity_controller',
          executable='teleop_keyboard',
          name='teleop_keyboard',
          output='screen',
          prefix=teleop_prefix,
          parameters=[{'max_speed': max_speed, 'max_steer_deg': max_steer}],
          condition=IfCondition(teleop),
      ),
  ])

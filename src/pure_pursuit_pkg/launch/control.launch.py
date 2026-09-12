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
  #   실측(tools/ff_identify.py, 학교 아스팔트): PWM = 22.6·v + 44.2
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
      DeclareLaunchArgument('ff_static', default_value='44.2'),
      DeclareLaunchArgument('ff_gain', default_value='22.6'),

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
          parameters=[{'v_max': max_speed, 'curvature_gain': curv_gain}],
      ),

      # 명령 먹스: 단일 /cmd_vel 출구 + 최종 안전 클램프
      Node(
          package='velocity_controller',
          executable='vehicle_cmd_mux',
          name='vehicle_cmd_mux',
          output='screen',
          parameters=[{'max_speed': max_speed, 'max_steer_deg': max_steer}],
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""teleop_drive.launch.py — 수동 조종 최소 구성 (차 위치잡기/테스트용).

serial_bridge + vehicle_cmd_mux 만 띄운다. GPS/측위/자율 스택 없음.
키보드 노드는 **별도 터미널에서 직접** 실행한다(포커스된 터미널이 BT 키보드 입력을 받음):

  터미널 A (노트북에 두는 쪽):
    cd ~/racing_ws && source install/setup.bash
    ros2 launch tools/teleop_drive.launch.py            # port 자동
    # 포트 지정: ros2 launch tools/teleop_drive.launch.py arduino_port:=/dev/ttyACM0

  터미널 B (이 창을 포커스한 채 BT 키보드로 조종):
    cd ~/racing_ws && source install/setup.bash
    ros2 run velocity_controller teleop_keyboard

조작: W/S 전후(누르는 동안), A/D 조향(래치 2°), SPACE 정지, E 비상정지, Q/Z 속도, Ctrl+C 종료.
teleop 은 mux 에서 자율보다 우선이고, 헤딩 캘리브 게이트에도 안 막힌다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  arduino_port = LaunchConfiguration('arduino_port')
  return LaunchDescription([
      DeclareLaunchArgument('arduino_port', default_value='auto'),
      # ★ 2026-09-17 — 경사 시험용으로 인자를 열었다. 예전엔 전부 하드코딩이라
      #   **teleop 에서는 제동이 아예 안 걸렸다.**
      #
      #   ff_mode 를 안 넘기면 노드 기본값 'firmware' 가 된다. 그 경로는
      #   VEL:{v} 만 보내고, 능동 제동(ff_brake_pwm)은 'ros' 경로에만 있다.
      #   즉 내리막에서 W 를 놓으면 PWM 0 = **관성**이다. 경사 3.8% 를 넘으면
      #   놓아도 계속 빨라진다(g·sinθ > 관성감속 0.37 m/s²).
      #
      #   내리막 시험은 반드시 ff_mode:=ros + ff_brake_pwm:=50 으로 할 것.
      #   놓는 순간 역 PWM 이 걸려 감속이 0.37 → 0.98 m/s² 가 된다
      #   (4m/s 에서 정지거리 21.6m → 8.2m).
      # ★ 2026-09-17 — 기본값을 **firmware** 로. 엔코더를 살려 펌웨어 속도
      #   PID(NO_ENCODER 0)를 쓰면 VEL: 을 보내야 한다. 'ros' 는 PWM: 을 보내
      #   PID 를 우회하므로 경사에서 다시 개루프가 된다.
      #   ⚠ 'firmware' 에서는 ff_brake_pwm(ROS 쪽 능동 제동)이 동작하지 않는다.
      #     대신 PID 가 과속에 역 PWM 을 낸다 — 그쪽이 옳다.
      # ⚠ 기본값을 잠시 'firmware' 로 바꿨다가 **되돌렸다**(2026-09-17).
      #   펌웨어를 아직 안 구웠는데(NO_ENCODER 1) 기본값만 바꿔 두면,
      #   인자 없이 띄웠을 때 VEL: 이 나가고 ROS 쪽 ff_min_pwm 이 무시된다.
      #   실제로 그래서 PWM 이 42~45 밖에 안 나가 차가 안 움직였다.
      #   **NO_ENCODER 0 으로 굽고 검증한 뒤에** 'firmware' 로 바꿀 것.
      DeclareLaunchArgument('ff_mode', default_value='ros',
                            description="'ros'=개루프 PWM(현재 펌웨어) · "
                                        "'firmware'=속도 PID(NO_ENCODER 0 이후)"),
      DeclareLaunchArgument('ff_brake_pwm', default_value='0.0',
                            description='능동 제동 역 PWM. 내리막 시험은 50 권장'),
      DeclareLaunchArgument('ff_min_pwm', default_value='50.0'),
      # 정지→출발 시 정지마찰을 뚫는 값. 평지 실측: PWM 50 안 구름 / 55 덜컹 /
      # 60 → 1.07 m/s. **오르막 출발은 더 필요할 수 있다.**
      DeclareLaunchArgument('ff_breakaway_pwm', default_value='60.0'),
      DeclareLaunchArgument('ff_breakaway_ms', default_value='1200.0'),
      DeclareLaunchArgument('max_speed', default_value='0.6',
                            description='먹스 속도 상한. 오르막 시험은 올려야 할 수 있다'),

      # 단일 /cmd_vel 출구 + 안전 클램프 (positioning 이라 보수적 상한)
      Node(
          package='velocity_controller',
          executable='vehicle_cmd_mux',
          name='vehicle_cmd_mux',
          output='screen',
          parameters=[{
              'max_speed': ParameterValue(LaunchConfiguration('max_speed'),
                                          value_type=float),
              'max_steer_deg': 18.0,
          }],
      ),

      # /cmd_vel → Arduino
      Node(
          package='velocity_controller',
          executable='serial_bridge',
          name='serial_bridge_node',
          output='screen',
          parameters=[{
              'port': arduino_port,
              'baud': 57600,
              'watchdog_timeout': 0.5,
              'max_steer_deg': 18.0,
              'ff_mode': LaunchConfiguration('ff_mode'),
              # ⚠ 정수로 주면 노드가 즉사한다 — value_type=float 필수
              'ff_brake_pwm': ParameterValue(
                  LaunchConfiguration('ff_brake_pwm'), value_type=float),
              'ff_min_pwm': ParameterValue(
                  LaunchConfiguration('ff_min_pwm'), value_type=float),
              'ff_breakaway_pwm': ParameterValue(
                  LaunchConfiguration('ff_breakaway_pwm'), value_type=float),
              'ff_breakaway_ms': ParameterValue(
                  LaunchConfiguration('ff_breakaway_ms'), value_type=float),
          }],
      ),
  ])

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


def generate_launch_description():
  arduino_port = LaunchConfiguration('arduino_port')
  return LaunchDescription([
      DeclareLaunchArgument('arduino_port', default_value='auto'),

      # 단일 /cmd_vel 출구 + 안전 클램프 (positioning 이라 보수적 상한)
      Node(
          package='velocity_controller',
          executable='vehicle_cmd_mux',
          name='vehicle_cmd_mux',
          output='screen',
          parameters=[{'max_speed': 0.6, 'max_steer_deg': 18.0}],
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
          }],
      ),
  ])

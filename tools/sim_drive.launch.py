#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_drive.launch.py — 하드웨어 없이 맵을 주행하는 순수 시뮬.

★ hil.launch.py 와 무엇이 다른가
  hil.launch.py 는 serial_bridge 를 띄워 **아두이노로 실제 명령이 나간다**.
  차가 땅에 있으면 그대로 굴러간다. 이 파일은 serial_bridge 를 빼서
  **코드만** 돌린다 — 경로·제어기·미션 로직을 안전하게 먼저 검증하는 용도다.

  순서:
    1) sim_drive.launch.py  코드만 (여기)            ← 하드웨어 무관, 안전
    2) hil.launch.py        + 아두이노 (바퀴 들고)   ← 구동·조향 하드웨어까지
    3) 실차

  가상차량(hil_vehicle.py)이 /cmd_vel 을 자전거모델로 적분해
  /odometry/filtered 를 내고, 그 위에서 로컬경로·pure_pursuit·종방향이 돌아
  다시 /cmd_vel 을 만든다 → 폐루프.

  ⚠ 이 시뮬은 **조향 하드웨어를 검증하지 못한다.** 맵 추종이 완벽해 보여도
    실제 조향각이 틀릴 수 있다(명령각을 그대로 적분하기 때문). 조향은
    실차에서 ADC 로 확인해야 한다.

사용:
  ros2 launch tools/sim_drive.launch.py \
      waypoints:=/home/han/racing_ws/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml
  ros2 launch tools/sim_drive.launch.py rviz:=true      # RViz 같이 띄우기
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WS = '/home/han/racing_ws'
DEFAULT_WP = f'{WS}/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml'


def generate_launch_description():
  wp = LaunchConfiguration('waypoints')
  max_speed = LaunchConfiguration('max_speed')
  use_rviz = LaunchConfiguration('rviz')
  start_idx = LaunchConfiguration('start_idx')

  # 시뮬 전용 뷰 — 위에서 내려다보고 코스 중심에 맞춰 둔다.
  # 기본 rviz 설정은 카메라 초점이 (-14.9, 95) 라 용인 코스(y -62~26)가 화면 밖이다.
  rviz_cfg = f'{WS}/tools/sim_drive.rviz'

  return LaunchDescription([
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WP),
      DeclareLaunchArgument('max_speed', default_value='1.0'),
      DeclareLaunchArgument('rviz', default_value='true'),
      DeclareLaunchArgument('start_idx', default_value='0'),

      # 전역 경로 (원점 검증 포함)
      Node(package='waypoint_follower', executable='global_path_publisher',
           name='global_path_publisher', output='screen',
           parameters=[{'path_file': ParameterValue(wp, value_type=str)}]),

      # 로컬 슬라이딩 윈도우 → /local_path, /curvature, /goal_reached
      Node(package='waypoint_follower', executable='local_sliding_window_node',
           name='local_sliding_window_node', output='screen'),

      # 횡방향
      Node(package='pure_pursuit_pkg', executable='local_pure_pursuit_node',
           name='local_pure_pursuit', output='screen'),

      # 종방향 (곡률 감속)
      Node(package='velocity_controller', executable='longitudinal_controller',
           name='longitudinal_controller', output='screen',
           parameters=[{'v_max': ParameterValue(max_speed, value_type=float)}]),

      # 먹스 — 시뮬이라 헤딩 캘리브 게이트를 끈다
      Node(package='velocity_controller', executable='vehicle_cmd_mux',
           name='vehicle_cmd_mux', output='screen',
           parameters=[{
               'require_heading_calib': False,
               'max_speed': ParameterValue(max_speed, value_type=float)}]),

      # 가상 차량 (자전거 모델) — serial_bridge 없음.
      # hil_vehicle.py 는 패키지 실행파일이 아니라 스크립트라 ExecuteProcess 로 띄운다.
      ExecuteProcess(
          cmd=['python3', f'{WS}/tools/hil_vehicle.py',
               '--wp', wp, '--start-idx', start_idx],
          output='screen'),

      Node(package='rviz2', executable='rviz2', name='rviz2',
           arguments=['-d', rviz_cfg], output='log',
           condition=IfCondition(use_rviz)),
  ])

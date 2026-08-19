#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hil.launch.py — HIL(하드웨어 인 더 루프) 맵 주행 시험.

차를 공중에 띄운 채, GPS·IMU·트랙 없이 **맵을 실제 치수대로 주행**하는지 본다.
가상 차량(tools/hil_vehicle.py)이 /cmd_vel 을 받아 자전거 모델로 움직여
/odometry/filtered 를 내면, 그 위에서 경로·조향·속도 제어가 돌아 다시 /cmd_vel 을
만든다(폐루프). serial_bridge 가 같은 /cmd_vel 을 아두이노로 보내므로 공중의 바퀴가
맵대로 돌고 조향이 맵의 커브대로 꺾인다.

  ros2 launch gps_localization hil.launch.py max_speed:=1.0
  python3 tools/hil_vehicle.py        # 다른 터미널 — 폐루프를 닫는다

★ 무부하(공중)라 전원 강하/브라운아웃은 재현되지 않는다. 이 시험은 '맵에 맞는
  조향·속도·경로추종' 제어 로직과 하드웨어 구동을 보는 것이다.
★ 먹스의 헤딩 캘리브 게이트는 hil_vehicle 이 /heading/yaw_offset 을 발행해 풀린다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_WAYPOINTS = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
                     'waypoints_recorded_resampled_0.5.yaml')


def generate_launch_description():
  pp_share = get_package_share_directory('pure_pursuit_pkg')

  waypoints = LaunchConfiguration('waypoints')
  max_speed = LaunchConfiguration('max_speed')
  arduino_port = LaunchConfiguration('arduino_port')

  return LaunchDescription([
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WAYPOINTS),
      # 공중 시험이라 실제 도달속도가 아니라 '맵대로 조향/감속' 확인이 목적.
      DeclareLaunchArgument('max_speed', default_value='1.0'),
      DeclareLaunchArgument('arduino_port', default_value='auto'),

      # 맵(전역경로)
      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
          parameters=[{'path_file': waypoints}],
      ),
      # 로컬 경로(전방 윈도우 + 곡률)
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
      ),
      # 제어 체인: pure_pursuit + longitudinal + mux + serial_bridge(→아두이노)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(pp_share, 'launch', 'control.launch.py')),
          launch_arguments={
              'arduino_port': arduino_port,
              'max_speed': max_speed,
          }.items(),
      ),
  ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_tracking.launch.py — 폐루프 경로추종 시뮬레이션 (RViz).

차량·GPS·아두이노 없이, 지금까지 만든 경로 파이프라인이 실제로 도는 걸 본다.

  global_path_publisher ──/global_path──▶ local_sliding_window_node
                                                │
                                          /local_path (차량기준 3차곡선)
                                                ▼
                                          sim_vehicle (pure pursuit + 자전거모델)
                                                │
                                       /odometry/filtered, map→base_link TF
                                                └──▶ 다시 local_sliding_window

sim_rviz.launch.py 와의 차이: 저쪽은 가상 차량을 경로 위에 강제로 올려놓아
추종 오차가 항상 0이다. 이 런치는 차가 스스로 조향해서 따라가므로 오차가 보인다.

RViz 표시:
  초록 = 기록한 전역 경로 / 빨강 = 지금 만들어진 로컬 경로(앞 10m)
  파랑 = 차가 실제로 지나온 궤적 / 화살표 = 차량 자세

사용:
  ros2 launch waypoint_follower sim_tracking.launch.py
  ros2 launch waypoint_follower sim_tracking.launch.py speed:=0.35
  ros2 launch waypoint_follower sim_tracking.launch.py init_offset:=2.0 init_heading_err:=30.0
  ros2 launch waypoint_follower sim_tracking.launch.py k_ld:=0.4 max_lookahead:=3.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
              'waypoints_recorded_resampled_0.5.yaml')


def generate_launch_description():
  share = get_package_share_directory('waypoint_follower')
  rviz_config = os.path.join(share, 'rviz', 'sim_tracking.rviz')

  lc = LaunchConfiguration
  args = [
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WP),
      DeclareLaunchArgument('speed', default_value='1.0'),
      DeclareLaunchArgument('min_speed', default_value='0.4'),
      DeclareLaunchArgument('k_ld', default_value='0.6'),
      DeclareLaunchArgument('min_lookahead', default_value='1.0'),
      DeclareLaunchArgument('max_lookahead', default_value='4.0'),
      DeclareLaunchArgument('max_steering_deg', default_value='18.0'),
      # 출발 시 의도적 오차 — 복귀 성능을 눈으로 확인할 때 쓴다
      DeclareLaunchArgument('init_offset', default_value='0.0'),
      DeclareLaunchArgument('init_heading_err', default_value='0.0'),
      DeclareLaunchArgument('rviz', default_value='true'),
  ]

  return LaunchDescription(args + [
      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
          parameters=[{'path_file': lc('waypoints')}],
      ),
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
      ),
      Node(
          package='waypoint_follower',
          executable='sim_vehicle',
          name='sim_vehicle',
          output='screen',
          parameters=[{
              'path_file': lc('waypoints'),
              'speed': lc('speed'),
              'min_speed': lc('min_speed'),
              'k_ld': lc('k_ld'),
              'min_lookahead': lc('min_lookahead'),
              'max_lookahead': lc('max_lookahead'),
              'max_steering_deg': lc('max_steering_deg'),
              'init_offset': lc('init_offset'),
              'init_heading_err': lc('init_heading_err'),
          }],
      ),
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config],
          output='screen',
          condition=IfCondition(lc('rviz')),
      ),
  ])

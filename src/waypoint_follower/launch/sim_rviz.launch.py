#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_rviz.launch.py
==================
실제 센서/GPS+IMU 융합 없이 시뮬레이션으로 로컬 경로 추종을 RViz에서
확인하기 위한 통합 런치.

  ros2 launch waypoint_follower sim_rviz.launch.py

실행되는 노드:
  - sim_odom_publisher     : 전역 경로를 따라 가상 차량을 움직이며
                             /odometry/filtered + map->base_link TF 발행
  - global_path_publisher  : 웨이포인트 YAML -> /global_path
  - local_sliding_window_node : 3차 곡선 피팅 -> /local_path (앞 10m)
  - rviz2                  : /global_path(초록), /local_path(빨강),
                             차량 TF/Odometry 시각화

속도를 바꾸려면:  ros2 launch ... sim_rviz.launch.py speed:=5.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('waypoint_follower')
  rviz_config = os.path.join(pkg_share, 'rviz', 'waypoint_follower.rviz')

  speed = LaunchConfiguration('speed')

  return LaunchDescription([
      DeclareLaunchArgument('speed', default_value='3.0'),

      Node(
          package='waypoint_follower',
          executable='sim_odom_publisher',
          name='sim_odom_publisher',
          output='screen',
          parameters=[{'speed': speed}],
      ),
      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
      ),
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
          parameters=[{
              'odom_topic': '/odometry/filtered',
              'n_back': 5,
              'n_forward': 20,
              'poly_order': 3,
              'lookahead_distance': 10.0,
              'point_spacing': 0.5,
          }],
      ),
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config],
          output='screen',
      ),
  ])

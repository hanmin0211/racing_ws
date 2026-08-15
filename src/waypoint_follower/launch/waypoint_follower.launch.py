#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_follower.launch.py
===========================
전역 경로 발행 + 로컬 슬라이딩 윈도우(3차 곡선 피팅/미래점 생성)를 함께 띄운다.

  ros2 launch waypoint_follower waypoint_follower.launch.py

전제: GPS+IMU 융합(robot_localization_config/gps_imu_fusion.launch.py)이
먼저(또는 같이) 실행되어 /odometry/filtered 가 살아있어야 한다.
rosbag 재생으로 테스트할 때는 use_sim_time:=true 를 붙인다.

  ros2 launch waypoint_follower waypoint_follower.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  odom_topic = LaunchConfiguration('odom_topic')

  return LaunchDescription([
      DeclareLaunchArgument('use_sim_time', default_value='false'),
      DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),

      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
      ),
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
          parameters=[{
              'use_sim_time': use_sim_time,
              'odom_topic': odom_topic,
              'n_back': 5,
              'n_forward': 20,
              'poly_order': 3,
              'lookahead_distance': 10.0,
              'point_spacing': 0.5,
          }],
      ),
  ])

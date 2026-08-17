#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localization_direct.launch.py
=============================
간단·견고 로컬라이제이션 (robot_localization/navsat/EKF 대체).

  ros2 launch gps_localization localization_direct.launch.py

실행:
  1. gps_heading_init   : handsfree/imu를 10m 직진 GPS-course로 정렬 →
                          /imu/corrected 발행 (캘리브 전 통과, 완료 후 정렬)
  2. direct_localization: /fix(RTK) + /imu/corrected → /odometry/filtered
                          (map 프레임, 웨이포인트와 동일 UTM 원점) + map->base_link TF

전제: RTK GPS(ngii_rtk.launch.py)와 IMU(handsfree)가 먼저 떠 있어야 함.
현장 순서: 다 켜고 → 10m 직진(헤딩) → /odometry/filtered가 실제 위치+정렬된 yaw로
나옴 → local_sliding_window_node가 이걸 받아 /local_path 생성.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  calib_distance = LaunchConfiguration('calib_distance')

  return LaunchDescription([
      DeclareLaunchArgument('calib_distance', default_value='10.0'),

      Node(
          package='gps_heading_init',
          executable='heading_init_node',
          name='gps_heading_init',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': 'handsfree/imu',
              'calib_distance': calib_distance,
          }],
      ),
      Node(
          package='gps_localization',
          executable='direct_localization_node',
          name='direct_localization',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': '/imu/corrected',
              'output_topic': '/odometry/filtered',
              # 원점은 config/site_origin.yaml 이 정본 (노드가 직접 읽음).
              'rate': 30.0,
              'publish_tf': True,
          }],
      ),
  ])

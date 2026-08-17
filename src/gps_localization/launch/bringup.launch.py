#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup.launch.py
=================
전체 스택을 한 번에 실행하는 마스터 런치.

  ros2 launch gps_localization bringup.launch.py

실행 구성:
  1. RTK GPS      : ngii_rtk.launch.py (ublox_dgnss + nav_sat_fix + NTRIP)
  2. IMU          : handsfree A9 (/dev/imu)
  3. heading_init : 10m 직진 후 yaw_offset 계산 → 발행 후 자동 종료
  4. direct_localization : /fix + IMU + yaw_offset → /odometry/filtered
  5. global_path_publisher : 웨이포인트 → /global_path
  6. local_sliding_window_node : 3차곡선 → /local_path (앞 10m)
  7. rviz2        : 시각화 (rviz:=false 로 끌 수 있음)

인자:
  rviz         : RViz 실행 여부 (기본 true)
  waypoints    : 웨이포인트 파일 경로
  calib_distance : 헤딩 캘리브 직진 거리[m] (기본 10)
  imu_port     : IMU 시리얼 포트 (기본 /dev/imu)

비밀번호: ngii_rtk가 env NGII_PW(기본 'ngii')를 사용.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_WAYPOINTS = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
                     'waypoints_recorded_resampled_0.5.yaml')


def generate_launch_description():
  ngii_share = get_package_share_directory('ngii_ntrip')
  wpf_share = get_package_share_directory('waypoint_follower')
  pp_share = get_package_share_directory('pure_pursuit_pkg')
  rviz_config = os.path.join(wpf_share, 'rviz', 'waypoint_follower.rviz')

  rviz = LaunchConfiguration('rviz')
  waypoints = LaunchConfiguration('waypoints')
  calib_distance = LaunchConfiguration('calib_distance')
  imu_port = LaunchConfiguration('imu_port')
  invert_imu_yaw = LaunchConfiguration('invert_imu_yaw')
  control = LaunchConfiguration('control')
  arduino_port = LaunchConfiguration('arduino_port')
  auto_calib = LaunchConfiguration('auto_calib')
  auto_calib_speed = LaunchConfiguration('auto_calib_speed')

  return LaunchDescription([
      DeclareLaunchArgument('rviz', default_value='true'),
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WAYPOINTS),
      DeclareLaunchArgument('calib_distance', default_value='10.0'),
      DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
      # 헤딩 반전 진단: 회전 시 화살표가 반대로 돌면 invert_imu_yaw:=true 로 실행.
      # heading_init·direct_localization 둘 다에 같은 값으로 전달됨.
      DeclareLaunchArgument('invert_imu_yaw', default_value='false'),
      # ⚠ control:=true 면 제어 체인(local_pure_pursuit+serial_bridge)까지 켜져
      # 실제 모터가 구동된다. 기본 false(안전). 포트는 auto(자동감지).
      DeclareLaunchArgument('control', default_value='false'),
      DeclareLaunchArgument('arduino_port', default_value='auto'),
      # ⚠ auto_calib:=true 면 헤딩 캘리브 10m를 차가 스스로 직진해서 한다.
      # 헤딩을 모르는 개루프 전진이므로 앞을 비우고 E-stop을 쥔 채로 쓸 것.
      # control:=true 여야 실제로 움직인다(serial_bridge가 있어야 명령이 나감).
      DeclareLaunchArgument('auto_calib', default_value='false'),
      DeclareLaunchArgument('auto_calib_speed', default_value='0.3'),

      # 1. RTK GPS (ublox_dgnss + nav_sat_fix + NTRIP)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(ngii_share, 'launch', 'ngii_rtk.launch.py'))),

      # 2. IMU
      Node(
          package='handsfree_ros2_imu',
          executable='hfi_a9_ros2',
          name='imu',
          output='screen',
          parameters=[{'port': imu_port, 'gra_normalization': True}],
      ),

      # 3. 헤딩 초기화 (10m 직진 후 자동 종료)
      Node(
          package='gps_heading_init',
          executable='heading_init_node',
          name='gps_heading_init',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': 'handsfree/imu',
              'calib_distance': calib_distance,
              'invert_imu_yaw': invert_imu_yaw,
              'auto_drive': auto_calib,
              'auto_speed': auto_calib_speed,
          }],
      ),

      # 4. 직접 로컬라이제이션
      Node(
          package='gps_localization',
          executable='direct_localization_node',
          name='direct_localization',
          output='screen',
          parameters=[{
              'fix_topic': '/fix',
              'imu_topic': 'handsfree/imu',
              'output_topic': '/odometry/filtered',
              # 원점은 지정하지 않는다 — config/site_origin.yaml 이 정본이고
              # 노드가 직접 읽는다. 여기서 덮어쓰면 waypoint_recorder 와
              # 어긋날 수 있다(어긋나면 전역경로가 통째로 평행이동한다).
              'rate': 30.0,
              'publish_tf': True,
              'invert_imu_yaw': invert_imu_yaw,
          }],
      ),

      # 5. 전역경로
      Node(
          package='waypoint_follower',
          executable='global_path_publisher',
          name='global_path_publisher',
          output='screen',
          parameters=[{'path_file': waypoints}],
      ),

      # 6. 로컬 슬라이딩 윈도우
      Node(
          package='waypoint_follower',
          executable='local_sliding_window_node',
          name='local_sliding_window_node',
          output='screen',
      ),

      # 7. 제어 체인 (control:=true 일 때만 — 실제 모터 구동)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(pp_share, 'launch', 'control.launch.py')),
          condition=IfCondition(control),
          launch_arguments={'arduino_port': arduino_port}.items(),
      ),

      # 8. RViz (rviz:=false 로 끌 수 있음)
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config],
          output='screen',
          condition=IfCondition(rviz),
      ),
  ])

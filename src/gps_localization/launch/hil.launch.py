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
from launch.conditions import IfCondition
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
  max_steer = LaunchConfiguration('max_steer_deg')
  curv_gain = LaunchConfiguration('curvature_gain')
  lidar = LaunchConfiguration('lidar')

  return LaunchDescription([
      DeclareLaunchArgument('waypoints', default_value=DEFAULT_WAYPOINTS),
      # 공중 시험이라 실제 도달속도가 아니라 '맵대로 조향/감속' 확인이 목적.
      DeclareLaunchArgument('max_speed', default_value='1.0'),
      DeclareLaunchArgument('arduino_port', default_value='auto'),
      # ★ 조향 상한 (2026-09-09 추가)
      #   control.launch.py 에는 있었는데 여기서 안 넘겨줘서 HIL 에서는 늘 18°
      #   고정이었다. steer_sweep 실측 결과 ±18° 는 기구 끝단이라 양방향 모두
      #   스톨이 걸린다(ADC 62~863 도달 후 컷). 용인 코스 최대 필요타각은
      #   10.5° 이므로 12~15° 로 제한하면 끝단을 안 건드린다.
      DeclareLaunchArgument('max_steer_deg', default_value='18.0'),
      # ★ 코너 감속 세기 (2026-09-10 추가). 8분 예산의 가장 큰 소프트웨어 레버라
      #   HIL 에서 조합을 시험할 수 있어야 한다.
      #   판단표: python3 tools/lap_budget.py --waypoints <코스파일>
      DeclareLaunchArgument('curvature_gain', default_value='6.0'),
      # ★ 라이다 회피를 HIL 에 얹는다 (2026-09-11 추가).
      #   GPS·웨이포인트·실주행 없이 **회피 체인 전체**를 검증할 수 있다:
      #     실제 라이다 스캔 → cluster_plot_node → /lidar/avoid_steer
      #       → vehicle_cmd_mux(AUTO 중 조향 override) → /cmd_vel
      #       → 가상 차량이 RViz 에서 실제로 피해 간다
      #   차를 안 움직이고도 "라이다가 보고 실제로 조향을 트는가" 를 눈으로 본다.
      #   ⚠ 별도로 드라이버가 /scan 을 쏘고 있어야 한다:
      #     ros2 launch sllidar_ros2 sllidar_a1_launch.py       #         serial_port:=/dev/ldlidar serial_baudrate:=256000
      #   ⚠ 마운트 방향을 먼저 확인할 것(fg_yaw_offset_deg 기본 180).
      #     tools/lidar_monitor.py 로 차 정면 물체가 ~0° 로 보이는지 본다.
      #     뒤집혀 있으면 **반대로 피한다** = 이탈.
      DeclareLaunchArgument('lidar', default_value='false'),
      # ★ 라이다가 앞뒤 두 대다. lidar_dual.launch.py 가 /scan_front 로 낸다.
      #   한 대만(sllidar_a1_launch.py) 띄웠으면 scan_topic:=/scan 으로 줄 것.
      #   어긋나면 조용히 라이다가 없는 것처럼 동작한다(노드가 경고는 낸다).
      DeclareLaunchArgument('scan_topic', default_value='/scan_front'),

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
      # 라이다 장애물 회피 (lidar:=true 일 때만)
      #   require_arm_for_steer 는 false 로 둔다 — HIL 시험에는 시퀀서가 없고,
      #   arm 을 아무도 안 주면 조향 회피가 영영 안 켜져 시험이 성립하지 않는다.
      #   (실차 bringup 에서는 sequencer 인자에 묶여 구간 밖에서 안 켜진다)
      Node(
          package='lidar_clustering',
          executable='cluster_plot_node',
          name='lidar_clustering',
          output='screen',
          condition=IfCondition(lidar),
          parameters=[{'scan_topic': LaunchConfiguration('scan_topic'),
                       'enable_plot': False,
                       'require_arm_for_steer': False}],
      ),
      # 제어 체인: pure_pursuit + longitudinal + mux + serial_bridge(→아두이노)
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(pp_share, 'launch', 'control.launch.py')),
          launch_arguments={
              'arduino_port': arduino_port,
              'max_speed': max_speed,
              'max_steer_deg': max_steer,
              'curvature_gain': curv_gain,
          }.items(),
      ),
  ])

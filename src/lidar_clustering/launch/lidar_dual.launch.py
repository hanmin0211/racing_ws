#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_dual.launch.py — 앞뒤 라이다 두 대를 **분리된 토픽**으로 띄운다.

★ 왜 따로 띄워야 하나
  이 차에는 RPLidar A1 이 앞뒤로 두 대 달려 있고 **둘 다 CP2102** 다.
  sllidar_a1_launch.py 를 두 번 띄우면 기본 토픽이 같아서 **둘 다 /scan 에
  발행**한다. 그러면 전방 회피가 **뒤쪽 물체에 반응**해 조향을 튼다 = 이탈 = 탈락.
  그래서 토픽·프레임·노드이름을 전부 분리한다.

    앞 → /scan_front  (frame laser_front)   ← 회피·전방정지가 쓰는 것
    뒤 → /scan_rear   (frame laser_rear)    ← 지금은 아무도 안 쓴다(기록용)

★ 장치 이름은 시리얼 고정 심볼릭 링크를 쓴다
  IMU 도 같은 CP210x 라 /dev/ttyUSB* 번호가 **꽂는 순서에 따라 뒤바뀐다**
  (2026-09-12 실제로 0↔2 스왑을 확인했다). 번호로 지정하면 현장에서
  "어제 되던 게 오늘 안 된다" 가 난다. /etc/udev/rules.d/99-ldlidar.rules 로
  아래 두 링크를 만들어 둘 것:
      /dev/ldlidar_front  ← 시리얼 f072916a7f58c6418f97ba0c944c359f
      /dev/ldlidar_rear   ← 시리얼 3f70bf129c502d4ebbceb3e38373d74b

  ※ 전방 개체(f072916a)는 2026-09-12 에 마운트 방향을 실측 확정했다:
    정면 물체가 원본각 180° 로 보인다 → fg_yaw_offset_deg 180 (기본값).
    **개체를 바꾸면 이 값을 다시 재야 한다** (tools/lidar_orient.py --moving 8).

사용:
  ros2 launch lidar_clustering lidar_dual.launch.py
  ros2 launch lidar_clustering lidar_dual.launch.py rear:=false   # 앞만
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _lidar(name, port, frame, topic, baud, cond=None):
  return Node(
      package='sllidar_ros2',
      executable='sllidar_node',
      name=name,
      output='screen',
      condition=cond,
      parameters=[{
          'channel_type': 'serial',
          'serial_port': port,
          'serial_baudrate': baud,
          'frame_id': frame,
          'inverted': False,
          'angle_compensate': True,
      }],
      # ★ 이 remap 이 핵심이다. 없으면 둘 다 /scan 으로 나간다.
      remappings=[('scan', topic)],
  )


def generate_launch_description():
  front_port = LaunchConfiguration('front_port')
  rear_port = LaunchConfiguration('rear_port')
  baud = LaunchConfiguration('serial_baudrate')
  rear = LaunchConfiguration('rear')

  return LaunchDescription([
      DeclareLaunchArgument('front_port', default_value='/dev/ldlidar_front'),
      DeclareLaunchArgument('rear_port', default_value='/dev/ldlidar_rear'),
      DeclareLaunchArgument('serial_baudrate', default_value='256000'),
      # 뒤쪽은 아직 쓰는 데가 없다. 기본으로 띄우되 필요 없으면 false.
      DeclareLaunchArgument('rear', default_value='true'),

      _lidar('sllidar_front', front_port, 'laser_front', '/scan_front', baud),
      _lidar('sllidar_rear', rear_port, 'laser_rear', '/scan_rear', baud,
             cond=IfCondition(rear)),
  ])

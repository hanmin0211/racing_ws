#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
steering_demo.launch.py — serial_bridge + steering_demo 를 한 번에.

  ros2 launch velocity_controller steering_demo.launch.py
  ros2 launch velocity_controller steering_demo.launch.py amplitude:=15.0 loop:=true
  ros2 launch velocity_controller steering_demo.launch.py mode:=sweep half_period:=10.0

⚠ 실제 조향 모터가 움직인다. 조향축 공중(벤치) 권장. 속도는 0(구동 안 함).

인자:
  mode        : demo(운전대 시퀀스) | sweep(삼각파)   (기본 demo)
  amplitude   : 데모 진폭[도] (기본 18, 실측 최대 20°)
  loop        : demo 반복 여부                          (기본 false)
  half_period : sweep 편도 시간[s]                      (기본 6)
  arduino_port: 시리얼 포트(auto=자동감지)             (기본 auto)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  mode = LaunchConfiguration('mode')
  amplitude = LaunchConfiguration('amplitude')
  loop = LaunchConfiguration('loop')
  half_period = LaunchConfiguration('half_period')
  arduino_port = LaunchConfiguration('arduino_port')

  return LaunchDescription([
      DeclareLaunchArgument('mode', default_value='demo'),
      DeclareLaunchArgument('amplitude', default_value='18.0'),
      DeclareLaunchArgument('loop', default_value='false'),
      DeclareLaunchArgument('half_period', default_value='6.0'),
      DeclareLaunchArgument('arduino_port', default_value='auto'),

      Node(
          package='velocity_controller',
          executable='serial_bridge',
          name='serial_bridge_node',
          output='screen',
          parameters=[{'port': arduino_port, 'baud': 57600,
                       'watchdog_timeout': 0.5}],
      ),

      # mode:=demo → 운전대 시퀀스
      Node(
          package='velocity_controller',
          executable='steering_demo',
          name='steering_demo',
          output='screen',
          parameters=[{'amplitude': amplitude, 'loop': loop}],
          condition=LaunchConfigurationEquals('mode', 'demo'),
      ),

      # mode:=sweep → 삼각파 스윕
      Node(
          package='velocity_controller',
          executable='steering_sweep',
          name='steering_sweep',
          output='screen',
          parameters=[{'amplitude': amplitude, 'half_period': half_period}],
          condition=LaunchConfigurationEquals('mode', 'sweep'),
      ),
  ])

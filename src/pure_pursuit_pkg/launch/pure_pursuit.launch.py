#!/usr/bin/env python3
"""
pure_pursuit.launch.py
========================
gps_local_realtime_node + pure_pursuit_node 를 동시에 실행하는 launch 파일.

실행:
    ros2 launch pure_pursuit_pkg pure_pursuit.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_pkg')
    config_file = os.path.join(pkg_share, 'config', 'pure_pursuit_params.yaml')

    gps_node = Node(
        package='pure_pursuit_pkg',
        executable='gps_local_realtime_node',
        name='gps_local_realtime_node',
        output='screen',
        parameters=[config_file],
    )

    pp_node = Node(
        package='pure_pursuit_pkg',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([gps_node, pp_node])

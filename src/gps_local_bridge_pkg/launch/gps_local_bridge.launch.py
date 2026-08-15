import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gps_local_bridge_pkg')
    default_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    return LaunchDescription([
        Node(
            package='gps_local_bridge_pkg',
            executable='gps_local_bridge',
            name='gps_local_bridge',
            output='screen',
            parameters=[default_params],
        )
    ])

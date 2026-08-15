import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_transform_pkg')
    default_params = os.path.join(pkg_share, 'config', 'vehicle_transform_params.yaml')

    return LaunchDescription([
        Node(
            package='vehicle_transform_pkg',
            executable='vehicle_transform',
            name='vehicle_transform',
            output='screen',
            parameters=[default_params],
        )
    ])

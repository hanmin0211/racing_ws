import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'config',
        'ekf_gps.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config_path],
            remappings=[
                ('imu/data', '/handsfree/imu/corrected'),
                ('gps/fix', '/fix'),
                ('odometry/filtered', '/odometry/global'),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ]
        ),
    ])

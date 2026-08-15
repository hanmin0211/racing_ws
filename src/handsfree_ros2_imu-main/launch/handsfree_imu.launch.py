import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_node = Node(
        package='handsfree_ros2_imu',
        executable='hfi_a9_ros2',  
        name='imu',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'gra_normalization': True  
        }]
    )

    return LaunchDescription([imu_node])

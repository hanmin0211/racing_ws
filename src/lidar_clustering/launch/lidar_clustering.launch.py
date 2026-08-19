from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_clustering',
            executable='cluster_plot_node',
            name='cluster_plot_node',
            output='screen',
        ),
    ])

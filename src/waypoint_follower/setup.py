import glob

from setuptools import find_packages, setup

package_name = 'waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_sliding_window_node = waypoint_follower.local_sliding_window_node:main',
            'global_path_publisher = waypoint_follower.global_path_publisher:main',
            'sim_odom_publisher = waypoint_follower.sim_odom_publisher:main',
            'waypoint_recorder = waypoint_follower.waypoint_recorder:main',
            'tracking_monitor = waypoint_follower.tracking_monitor_node:main',
            'resample_waypoints = waypoint_follower.waypoint_resample:main',

        ],
    },
)

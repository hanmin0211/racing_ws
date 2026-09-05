import glob

from setuptools import find_packages, setup

package_name = 'mission_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='Mission perception - camera input and detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = mission_perception.camera_node:main',
            'traffic_light_bridge = mission_perception.traffic_light_bridge:main',
            'stop_point_recorder = mission_perception.stop_point_recorder:main',
            'stop_detector = mission_perception.stop_detector:main',
            'parking_recorder = mission_perception.parking_recorder:main',
            'parking_pose_recorder = mission_perception.parking_pose_recorder:main',
            'parking_node = mission_perception.parking_node:main',
            'parking_direct = mission_perception.parking_direct:main',
            'crosswalk_stop_node = mission_perception.crosswalk_stop_node:main',
            'mission_sequencer = mission_perception.mission_sequencer:main',
            'sudden_stop_node = mission_perception.sudden_stop_node:main',
        ],
    },
)

from setuptools import setup
from glob import glob
import os 

package_name = 'handsfree_ros2_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='a2053231@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'hfi_a9_ros2 = handsfree_ros2_imu.hfi_a9_ros2:main',
            'get_imu_rclpy = handsfree_ros2_imu.get_imu_rclpy:main',
        ],
    },
)

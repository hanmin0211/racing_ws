import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'velocity_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'serial_bridge = velocity_controller.serial_bridge_node:main',
        'wasd_teleop = velocity_controller.wasd_teleop_node:main',
        'steering_demo = velocity_controller.steering_demo_node:main',
        'steering_sweep = velocity_controller.steering_sweep_node:main',
        'longitudinal_controller = velocity_controller.longitudinal_controller_node:main',
        'vehicle_cmd_mux = velocity_controller.vehicle_cmd_mux_node:main',
        'teleop_keyboard = velocity_controller.teleop_keyboard_node:main',
        'encoder_calib = velocity_controller.encoder_calib_node:main',
        'ff_sweep = velocity_controller.ff_sweep_node:main',
        ],
    },
)

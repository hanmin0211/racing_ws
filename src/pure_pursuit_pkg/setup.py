import os
from glob import glob
from setuptools import setup

package_name = 'pure_pursuit_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='you@example.com',
    description='GPS/헤딩 기반 로컬라이제이션 + Pure Pursuit 경로추종 제어기',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_local_realtime_node = pure_pursuit_pkg.gps_local_realtime_node:main',
            'pure_pursuit_node = pure_pursuit_pkg.pure_pursuit_node:main',
            'local_pure_pursuit_node = pure_pursuit_pkg.local_pure_pursuit_node:main',
            'local_sliding_window_node = pure_pursuit_pkg.local_sliding_window:main',
            'dummy_odom = pure_pursuit_pkg.dummy_odom:main',
            'dummy_odom_track = pure_pursuit_pkg.dummy_odom_track:main',
        ],
    },
)

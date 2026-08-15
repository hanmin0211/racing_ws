import os
from glob import glob
from setuptools import setup

package_name = 'gps_local_bridge_pkg'

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
    install_requires=['setuptools', 'pyproj', 'pyyaml'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description=(
        'GPS(WGS84) 웨이포인트/실시간 위치를 로컬 평면좌표로 변환하여 '
        'pure_pursuit_controller가 필요로 하는 /current_position, /local_waypoints, '
        '/local_waypoint_path, /corrected_heading 토픽을 발행하는 브릿지 노드'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_local_bridge = gps_local_bridge_pkg.gps_local_bridge:main',
        ],
    },
)

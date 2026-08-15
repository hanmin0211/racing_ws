import os
from glob import glob
from setuptools import setup

package_name = 'vehicle_transform_pkg'

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
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description=(
        'ENU-local 웨이포인트를 차량좌표계로 변환하고 3차 다항식으로 피팅하는 노드'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_transform = vehicle_transform_pkg.vehicle_transform:main',
        ],
    },
)

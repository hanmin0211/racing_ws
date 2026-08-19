from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'lidar_clustering'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='2D LiDAR clustering, Hungarian tracking and Follow-the-Gap planning',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cluster_plot_node = lidar_clustering.cluster_plot_node:main',
        ],
    },
)

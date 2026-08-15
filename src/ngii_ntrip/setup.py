import glob

from setuptools import find_packages, setup

package_name = 'ngii_ntrip'

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
    description='NGII VRS NTRIP client (GGA 지원) + F9P RTK launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vrs_ntrip_client = ngii_ntrip.vrs_ntrip_client:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'waypoint_save_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='GPS 웨이포인트를 답사하며 YAML로 저장하는 노드',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_save = waypoint_save_pkg.waypoint_save:main',
        ],
    },
)

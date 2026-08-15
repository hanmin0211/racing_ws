#!/usr/bin/env python3
"""
gps_imu_fusion.launch.py
===========================
ekf_node + navsat_transform_node 를 동시에 실행해서
실제 IMU(handsfree/imu) + GPS(/fix) 센서 퓨전 결과를
/odometry/filtered 로 발행한다.

사전 조건: handsfree_ros2_imu 노드와 ublox GPS 드라이버가 먼저(또는 같이)
실행되어 /fix, /navheading, handsfree/imu 토픽이 살아있어야 한다.

실행:
    ros2 launch robot_localization_config gps_imu_fusion.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization_config')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    # ---- 정적 TF: base_link <-> 각 센서 프레임 ----
    # robot_localization은 센서 메시지의 frame_id(gps, imu_link)를 base_link
    # 기준으로 변환해서 사용하므로, 이 변환관계가 TF 트리에 반드시 있어야 한다.
    # 지금은 장착 위치를 실측하지 않아 오프셋을 0(차량 중심과 동일)으로 두었다.
    # 나중에 실제 GPS 안테나/IMU 장착 위치를 줄자로 재서
    # x y z yaw pitch roll 값을 정확히 넣으면 정밀도가 올라간다.
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    static_tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
    )

    # ---- IMU 헤딩 초기화 ----
    # handsfree/imu 원본을 받아 yaw를 GPS-course(10m 직진)로 정렬한 뒤
    # /imu/corrected 로 재발행한다. EKF/navsat_transform이 이 토픽을 쓴다.
    # (캘리브 전에는 원본 통과, 10m 직진 완료 시 정렬값 적용)
    heading_init_node = Node(
        package='gps_heading_init',
        executable='heading_init_node',
        name='gps_heading_init',
        output='screen',
        parameters=[{
            'fix_topic': '/fix',
            'imu_topic': 'handsfree/imu',
            'calib_distance': 10.0,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_config],
        remappings=[
            # 헤딩 정렬된 IMU 사용 (gps_heading_init → /imu/corrected)
            ('imu', '/imu/corrected'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([static_tf_imu, static_tf_gps, heading_init_node,
                              ekf_node, navsat_node])

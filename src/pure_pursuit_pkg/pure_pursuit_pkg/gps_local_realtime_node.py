#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_local_realtime_node.py
============================
실시간으로 들어오는 GPS(/fix, sensor_msgs/NavSatFix)와 헤딩
(/navheading, sensor_msgs/Imu의 orientation 필드 - u-blox 듀얼안테나
GPS가 제공하는 헤딩)을 받아, 오프라인에서 waypoint 경로를 만들 때
사용한 것과 동일한 원점(origin_lat, origin_lon) 기준 로컬좌표계로
변환하여 nav_msgs/Odometry 로 발행한다.

[왜 원점을 파라미터로 고정하는가]
gps_to_local_fit.py 로 waypoint 경로 파일을 만들 때 사용한 원점과
반드시 동일한 원점을 써야, 실시간 차량 위치와 저장된 경로가 같은
좌표계 위에서 올바르게 비교된다. 원점이 다르면 차량이 경로에서
크게 벗어난 것처럼 계산되어 위험하다.

[헤딩 처리]
/navheading 토픽은 sensor_msgs/Imu 메시지 형식으로 GPS 듀얼안테나
헤딩을 담아 발행되고 있음을 실제 확인했다 (orientation 필드에
쿼터니언으로 담겨 있음, frame_id: gps). 이 쿼터니언에서 yaw만
추출해서 사용한다.

발행 토픽: /vehicle_local_pose (nav_msgs/Odometry)
  - pose.pose.position.x, y : 로컬좌표 (m)
  - pose.pose.orientation   : 헤딩(yaw)을 쿼터니언으로
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

from pure_pursuit_pkg.core_logic import GpsLocalConverter


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsLocalRealtimeNode(Node):
    def __init__(self):
        super().__init__('gps_local_realtime_node')

        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('heading_topic', 'handsfree/imu')  # 실제 물리 IMU 사용
        self.declare_parameter('output_topic', '/vehicle_local_pose')

        origin_lat = self.get_parameter('origin_lat').value
        origin_lon = self.get_parameter('origin_lon').value

        if origin_lat == 0.0 and origin_lon == 0.0:
            self.get_logger().error(
                '[오류] origin_lat / origin_lon 파라미터가 설정되지 않았습니다. '
                'waypoint 경로 생성 시 사용한 원점과 반드시 동일한 값을 '
                '--ros-args -p origin_lat:=.. -p origin_lon:=.. 로 지정하세요.'
            )
            raise RuntimeError('origin_lat/origin_lon 파라미터 필요')

        self.converter = GpsLocalConverter(origin_lat, origin_lon)
        self.get_logger().info(
            f'[초기화] 원점=({origin_lat:.8f}, {origin_lon:.8f}), '
            f'UTM Zone {self.converter.zone}N (EPSG:{self.converter.epsg})'
        )

        self.latest_yaw = 0.0
        self.have_heading = False

        gps_topic = self.get_parameter('gps_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self.gps_callback, 10)
        self.heading_sub = self.create_subscription(
            Imu, heading_topic, self.heading_callback, 10)
        self.pub = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f'[구독] GPS: {gps_topic}, 헤딩: {heading_topic}  ->  발행: {output_topic}'
        )

    def heading_callback(self, msg: Imu):
        q = msg.orientation
        self.latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_heading = True

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn('[경고] GPS fix 상태 불량 (status < 0), 무시함',
                                    throttle_duration_sec=2.0)
            return

        x, y = self.converter.to_local(msg.latitude, msg.longitude)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.latest_yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.pub.publish(odom)

        if not self.have_heading:
            self.get_logger().warn(
                '[경고] 아직 헤딩(/navheading) 데이터를 받지 못해 yaw=0으로 발행 중',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GpsLocalRealtimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

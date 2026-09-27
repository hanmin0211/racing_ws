#!/usr/bin/env python3
"""
waypoint_save.py
------------------
GPS(/fix)를 구독하고, 서비스 호출(/save_waypoint)이 올 때마다
현재 위치를 웨이포인트로 기록해서 YAML 파일로 저장한다.
gps_local_bridge_pkg가 기대하는 형식과 정확히 맞춤:
  waypoints:
    - latitude: ...
      longitude: ...
    - latitude: ...
      longitude: ...

사용법:
  1. 이 노드를 실행한다.
  2. 차량을 트랙을 따라 몰면서, 웨이포인트로 찍고 싶은 지점마다:
       ros2 service call /save_waypoint std_srvs/srv/Trigger {}
  3. 트랙 전체를 다 찍었으면 Ctrl+C로 종료 (그 시점까지 YAML로 저장됨)
"""
import os
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger


class WaypointSave(Node):
    def __init__(self):
        super().__init__('waypoint_save')

        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('output_file', '/home/han/waypoints/all_waypoint.yaml')
        self.declare_parameter('min_fix_status', 0)

        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.min_fix_status = self.get_parameter('min_fix_status').get_parameter_value().integer_value

        self.latest_fix = None
        self.waypoints = []  # [{'latitude':, 'longitude':}, ...]

        self.sub = self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
        self.save_srv = self.create_service(Trigger, 'save_waypoint', self.save_waypoint_callback)

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        self.get_logger().info(f'웨이포인트 저장 노드 시작됨. GPS 토픽 구독: {gps_topic}')
        self.get_logger().info('현재 위치를 저장하려면: ros2 service call /save_waypoint std_srvs/srv/Trigger {}')
        self.get_logger().info(f'저장 파일 경로: {self.output_file}')

    def gps_callback(self, msg: NavSatFix):
        self.latest_fix = msg

    def save_waypoint_callback(self, request, response):
        if self.latest_fix is None:
            response.success = False
            response.message = 'GPS 데이터를 아직 못 받았습니다.'
            self.get_logger().warn(response.message)
            return response

        if self.latest_fix.status.status < self.min_fix_status:
            response.success = False
            response.message = f'GPS fix 상태가 좋지 않습니다 (status={self.latest_fix.status.status}).'
            self.get_logger().warn(response.message)
            return response

        lat = self.latest_fix.latitude
        lon = self.latest_fix.longitude

        self.waypoints.append({'latitude': lat, 'longitude': lon})

        idx = len(self.waypoints) - 1
        response.success = True
        response.message = f'웨이포인트 #{idx} 저장됨: lat={lat:.7f}, lon={lon:.7f}'
        self.get_logger().info(response.message)
        self._flush_to_file()
        return response

    def _flush_to_file(self):
        try:
            with open(self.output_file, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            self.get_logger().error(f'파일 저장 실패: {e}')

    def destroy_node(self):
        self._flush_to_file()
        self.get_logger().info(f'총 {len(self.waypoints)}개 웨이포인트를 {self.output_file}에 저장했습니다.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSave()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

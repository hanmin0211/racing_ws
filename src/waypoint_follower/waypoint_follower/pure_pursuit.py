#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
from pyproj import Transformer

from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker

class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.get_logger().info("===== [구형 복구본] Pure Pursuit 노드 시동 =====")

        # -------------------------
        # Waypoint 파일 (6140개 보간 맵 직결)
        # -------------------------
        self.yaml_file = "/home/han/gps_converter/waypoints_local_resampled_0.3.yaml"

        # Origin 세팅
        self.origin_x = 399848.522
        self.origin_y = 4092209.171

        # WGS84 -> UTM52N 변환기 세팅
        self.transformer = Transformer.from_crs(
            "EPSG:4326",
            "EPSG:32652",
            always_xy=True
        )

        self.path = []
        self.load_waypoints()

        self.current_lat = None
        self.current_lon = None

        # 실제 유블럭스 GPS 구독
        self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)

        # Rviz2 조향점 마커 발행자 (QoS: Best Effort로 시각화 씹힘 원천 차단)
        self.marker_pub = self.create_publisher(Marker, "/pure_pursuit_marker", 10)

        self.get_logger().info("📡 실제 유블럭스 정밀 GPS 신호 대기 중...")

    def load_waypoints(self):
        try:
            with open(self.yaml_file, "r") as f:
                data = yaml.safe_load(f)
            self.path = data["waypoints"]
            self.get_logger().info(f"📂 성공적으로 {len(self.path)}개의 정밀 웨이포인트를 로드했습니다.")
        except Exception as e:
            self.get_logger().error(f"❌ YAML 로드 실패: {str(e)}")

    def find_closest_waypoint(self, x, y):
        min_dist = float("inf")
        closest_idx = 0

        for i, wp in enumerate(self.path):
            dx = wp["x"] - x
            dy = wp["y"] - y
            dist = dx * dx + dy * dy

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def publish_marker(self, x, y, marker_id, r, g, b):
        """ [수술 완료] 들여쓰기 공백 8칸 칼 정렬 마감 """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pure_pursuit"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.5  # 가시성을 위해 마커 크기 소폭 확대
        marker.scale.y = 1.5
        marker.scale.z = 1.5

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        easting, northing = self.transformer.transform(self.current_lon, self.current_lat)

        local_x = easting - self.origin_x
        local_y = northing - self.origin_y

        closest_idx = self.find_closest_waypoint(local_x, local_y)
        closest_wp = self.path[closest_idx]

        self.get_logger().info(
            f"📍 내차 위치: ({local_x:.3f}, {local_y:.3f}) | 최단 점 번호: {closest_idx}",
            throttle_duration_sec=0.5
        )

        # 차량 위치 마커 발행 (초록색 원)
                # 차량 위치 (초록)
        self.publish_marker(
            local_x,
            local_y,
            0,
            0.0,
            1.0,
            0.0
        )

        # 최근접 웨이포인트 (빨강)
        self.publish_marker(
            closest_wp["x"],
            closest_wp["y"],
            1,
            1.0,
            0.0,
            0.0
        )

        
def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


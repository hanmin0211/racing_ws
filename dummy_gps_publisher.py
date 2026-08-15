#!/usr/bin/env python3
"""
dummy_gps_publisher
====================

GPS 하드웨어 없이 vehicle_transform_node 파이프라인을 검증하기 위한 노드.

동작 방식
--------
1. 저장된 웨이포인트 yaml(x, y, origin 정보 포함)을 로드한다.
2. x, y를 원점(origin_lat/origin_lon) 기준으로 다시 위경도로 역변환한다.
3. 인덱스를 일정 간격으로 증가시키며, 마치 차량이 그 경로를 실제로
   주행하는 것처럼 /fix (NavSatFix) 를 발행한다.
4. 동시에 현재 점 -> 다음 점 방향으로 /heading (Float32, degree) 을 발행한다.

주의
----
- 이건 어디까지나 "좌표변환 -> 피팅 -> 20개 점 샘플링" 로직이 정상 동작하는지
  확인하기 위한 디버깅용 노드다. 실제 대회에서는 절대 쓰지 않는다.
- vehicle_transform_node 와 반드시 같은 origin_lat/origin_lon, 같은
  yaml_path 를 쓰도록 맞춰야 의미 있는 테스트가 된다.
"""

import math
import os

import numpy as np
import rclpy
import yaml
from pyproj import Transformer
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32


class DummyGPSPublisher(Node):
    def __init__(self) -> None:
        super().__init__("dummy_gps_publisher")

        self.declare_parameter("utm_epsg", "epsg:32652")
        self.declare_parameter("origin_lat", 36.970661166666666)
        self.declare_parameter("origin_lon", 127.87485216666667)
        self.declare_parameter(
            "yaml_path", os.path.expanduser("~/gps_converter/waypoints_local_resampled_0.3.yaml")
        )
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("step", 1)  # 매 publish마다 몇 인덱스씩 전진할지
        self.declare_parameter("noise_std_m", 0.0)  # 위치 노이즈 (테스트용, 기본 0)

        utm_epsg = self.get_parameter("utm_epsg").value
        self.origin_lat = self.get_parameter("origin_lat").value
        self.origin_lon = self.get_parameter("origin_lon").value
        yaml_path = self.get_parameter("yaml_path").value
        rate = self.get_parameter("publish_rate_hz").value
        self.step = self.get_parameter("step").value
        self.noise_std_m = self.get_parameter("noise_std_m").value

        self.inv_transformer = Transformer.from_crs("epsg:32652" if not utm_epsg else utm_epsg, "epsg:4326", always_xy=True)
        fwd = Transformer.from_crs("epsg:4326", utm_epsg, always_xy=True)
        self.origin_utm = fwd.transform(self.origin_lon, self.origin_lat)

        self.waypoints = self._load_waypoints(yaml_path)
        self.idx = 0

        self.pub_fix = self.create_publisher(NavSatFix, "/fix", 10)
        self.pub_heading = self.create_publisher(Float32, "/heading", 10)

        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f"dummy_gps_publisher 시작. 웨이포인트 {len(self.waypoints)}개, "
            f"{rate}Hz로 /fix, /heading 발행 (가짜 주행 시뮬레이션)"
        )

    def _load_waypoints(self, path: str) -> np.ndarray:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        pts = [[float(wp["x"]), float(wp["y"])] for wp in data["waypoints"]]
        return np.array(pts, dtype=float)

    def timer_callback(self) -> None:
        if len(self.waypoints) == 0:
            return

        n = len(self.waypoints)
        cur = self.waypoints[self.idx % n]
        nxt = self.waypoints[(self.idx + 1) % n]

        x = cur[0] + self.origin_utm[0]
        y = cur[1] + self.origin_utm[1]

        if self.noise_std_m > 0:
            x += np.random.normal(0, self.noise_std_m)
            y += np.random.normal(0, self.noise_std_m)

        lon, lat = self.inv_transformer.transform(x, y)

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "gps"
        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = float(lat)
        fix_msg.longitude = float(lon)
        fix_msg.altitude = 217.0
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        fix_msg.position_covariance = [0.02] * 9
        self.pub_fix.publish(fix_msg)

        heading_deg = math.degrees(math.atan2(nxt[1] - cur[1], nxt[0] - cur[0]))
        heading_msg = Float32()
        heading_msg.data = heading_deg
        self.pub_heading.publish(heading_msg)

        self.idx = (self.idx + self.step) % n


def main(args=None):
    rclpy.init(args=args)
    node = DummyGPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
vehicle_transform_node
=======================

GPS 담당자 산출물 (자율주행 경진대회용)

역할
----
1. 실시간 GPS(/fix) + 헤딩(/heading)을 받아 차량의 현재 위치/자세를 추정한다.
2. 사전에 로드된 전역 웨이포인트 중, 차량과 가장 가까운 지점을 찾고
   그 지점부터 앞으로 forward_index_count 개만 잘라 사용한다.
   -> 트랙이 자기 자신과 가까이 지나가는 구간(출발/도착점, 헤어핀)에서
      엉뚱한 구간이 섞여 들어가는 것을 방지하기 위함.
3. 잘라낸 구간을 차량 기준 로컬 좌표계(진행방향 = x축)로 변환한다.
4. min_forward_x ~ lookahead_window_m 범위의 점들만 남겨 최소자승법으로
   poly_degree차 다항식에 피팅한다.
5. 피팅된 곡선 위에서 curve_sample_count(기본 20)개의 점을 균일 간격으로
   샘플링하여 제어팀에 발행한다. (이것이 "앞의 20개 점" 산출물)

주의
----
- 이 노드는 시뮬레이션 코드가 아니다. 실제 /fix, /heading 값을 그대로 사용한다.
- 웨이포인트 파일(yaml)과 좌표계를 반드시 맞춰야 한다. 웨이포인트가
  특정 원점(origin_lat/origin_lon) 기준 상대좌표로 저장되어 있다면,
  이 노드도 반드시 같은 원점을 사용해야 한다. (그렇지 않으면 차량 위치와
  웨이포인트가 서로 다른 좌표계에 놓여 완전히 어긋난다.)
"""

import math
import os
from bisect import bisect_left
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster


@dataclass
class PathFitResult:
    """한 사이클의 경로 피팅 결과를 담는 컨테이너."""

    coeffs: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    rmse: float = 999.0
    valid: bool = False
    raw_local_x: List[float] = field(default_factory=list)
    raw_local_y: List[float] = field(default_factory=list)
    sampled_x: List[float] = field(default_factory=list)
    sampled_y: List[float] = field(default_factory=list)


class VehicleTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_transform")

        self._declare_parameters()
        self._load_parameters()

        self.transformer = Transformer.from_crs(
            "epsg:4326", self.utm_epsg, always_xy=True
        )

        # ---- 상태 변수 ----
        self.origin_utm: Optional[Tuple[float, float]] = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.prev_heading: Optional[float] = None
        self.last_position: Optional[Tuple[float, float]] = None
        self.heading_source_active = False  # /heading 토픽이 실제로 들어오는지

        # ---- 웨이포인트 로드 ----
        self.global_waypoints = np.empty((0, 2), dtype=float)
        self._load_waypoints_from_yaml(self.yaml_path)

        # ---- 구독자 ----
        # NavSatFix는 센서 데이터이므로 sensor QoS(Best Effort) 사용을 권장하지만,
        # ublox_gps 드라이버가 RELIABLE로 publish하는 경우가 많아 기본 QoS로 둔다.
        self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.create_subscription(Float32, "/heading", self.heading_callback, 10)

        # ---- 발행자 ----
        self.pub_coeffs = self.create_publisher(Float32MultiArray, "/path_polynomial_coeffs", 10)
        self.pub_rmse = self.create_publisher(Float32, "/path_fit_rmse", 10)
        self.pub_local_path = self.create_publisher(Path, "/local_path", 10)          # 20개 샘플 (제어팀 산출물)
        self.pub_raw_local_path = self.create_publisher(Path, "/local_path_raw", 10)  # 디버깅용 원본 로컬점
        self.pub_global_path = self.create_publisher(Path, "/vehicle_frame_path", 10)  # 시각화용 전역 경로
        self.pub_waypoints_local = self.create_publisher(
            Float32MultiArray, "/waypoints_local", 10
        )  # 제어팀이 바로 쓰기 편한 flat float 배열: [x0,y0,x1,y1,...]

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- 주기 발행 타이머 ----
        # GPS 콜백에만 의존하지 않고 publish_rate_hz로 일정하게 발행해야
        # 제어팀 쪽에서 주기가 불규칙해지는 문제를 피할 수 있다.
        timer_period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"vehicle_transform_node 시작. "
            f"웨이포인트 {len(self.global_waypoints)}개 로드, "
            f"forward_index_count={self.forward_index_count}, "
            f"lookahead_window_m={self.lookahead_window_m}, "
            f"curve_sample_count={self.curve_sample_count}"
        )

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # config/vehicle_transform_params.yaml 과 이름을 반드시 일치시킨다.
        self.declare_parameter("lookahead_window_m", 8.0)
        self.declare_parameter("min_forward_x", -1.0)
        self.declare_parameter("min_points_for_fit", 4)
        self.declare_parameter("poly_degree", 3)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("curve_sample_count", 20)
        self.declare_parameter("forward_index_count", 20)

        # 좌표계 관련 (yaml 파일 자체에 origin 정보가 없을 때의 fallback)
        self.declare_parameter("utm_epsg", "epsg:32652")
        self.declare_parameter("origin_lat", 0.0)
        self.declare_parameter("origin_lon", 0.0)
        self.declare_parameter(
            "yaml_path", os.path.expanduser("~/gps_converter/waypoints_local_resampled_0.3.yaml")
        )

        # 피팅 결과 이상치 방어용
        self.declare_parameter("max_valid_coeff_abs", 15.0)
        self.declare_parameter("heading_timeout_sec", 1.0)

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self.lookahead_window_m = gp("lookahead_window_m").value
        self.min_forward_x = gp("min_forward_x").value
        self.min_points_for_fit = gp("min_points_for_fit").value
        self.poly_degree = gp("poly_degree").value
        self.publish_rate_hz = gp("publish_rate_hz").value
        self.curve_sample_count = gp("curve_sample_count").value
        self.forward_index_count = gp("forward_index_count").value

        self.utm_epsg = gp("utm_epsg").value
        self.param_origin_lat = gp("origin_lat").value
        self.param_origin_lon = gp("origin_lon").value
        self.yaml_path = gp("yaml_path").value

        self.max_valid_coeff_abs = gp("max_valid_coeff_abs").value
        self.heading_timeout_sec = gp("heading_timeout_sec").value

    # ------------------------------------------------------------------
    # 웨이포인트 로드
    # ------------------------------------------------------------------
    def _load_waypoints_from_yaml(self, path: str) -> None:
        if not os.path.exists(path):
            self.get_logger().error(f"웨이포인트 파일을 찾을 수 없습니다: {path}")
            return

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not data or "waypoints" not in data or data["waypoints"] is None:
            self.get_logger().error(f"'waypoints' 키가 없거나 비어있습니다: {path}")
            return

        pts = [[float(wp["x"]), float(wp["y"])] for wp in data["waypoints"]]
        self.global_waypoints = np.array(pts, dtype=float)

        # 파일 안에 원점 정보(utm_info)가 있으면 그걸 최우선으로 사용한다.
        # (차량 위치와 웨이포인트가 같은 좌표계를 쓰도록 보장하는 핵심 로직)
        utm_info = data.get("utm_info")
        if utm_info and "origin_lat" in utm_info and "origin_lon" in utm_info:
            origin_lat = float(utm_info["origin_lat"])
            origin_lon = float(utm_info["origin_lon"])
            ox, oy = self.transformer.transform(origin_lon, origin_lat)
            self.origin_utm = (ox, oy)
            self.get_logger().info(
                f"웨이포인트 파일의 원점을 사용합니다: "
                f"lat={origin_lat}, lon={origin_lon}"
            )
        elif self.param_origin_lat != 0.0 or self.param_origin_lon != 0.0:
            ox, oy = self.transformer.transform(self.param_origin_lon, self.param_origin_lat)
            self.origin_utm = (ox, oy)
            self.get_logger().info("파라미터로 지정된 origin_lat/origin_lon을 사용합니다.")
        else:
            self.origin_utm = None
            self.get_logger().warn(
                "원점 정보가 없습니다. 첫 GPS fix를 원점으로 사용합니다. "
                "웨이포인트 파일의 좌표계와 실제로 일치하는지 반드시 확인하세요."
            )

        self.get_logger().info(
            f"웨이포인트 {len(self.global_waypoints)}개 로드 완료: {path}"
        )

    # ------------------------------------------------------------------
    # 콜백: GPS
    # ------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn("GPS 위경도 값이 NaN 입니다.", throttle_duration_sec=5.0)
            return

        # NavSatFix.status.status: -1 = no fix. 값이 있어도 신뢰할 수 없다.
        if msg.status.status < 0:
            self.get_logger().warn(
                "GPS fix 없음 (status < 0). 위치 갱신을 건너뜁니다.",
                throttle_duration_sec=5.0,
            )

        x_utm, y_utm = self.transformer.transform(msg.longitude, msg.latitude)

        if self.origin_utm is None:
            # 원점이 아직 없으면 이번 fix를 원점으로 고정한다 (최초 1회만).
            self.origin_utm = (x_utm, y_utm)
            self.get_logger().warn(
                f"첫 GPS fix를 원점으로 고정했습니다: "
                f"lat={msg.latitude}, lon={msg.longitude}"
            )

        prev_position = self.last_position
        self.current_x = x_utm - self.origin_utm[0]
        self.current_y = y_utm - self.origin_utm[1]
        self.last_position = (self.current_x, self.current_y)

        # /heading 토픽이 아직 한 번도 들어온 적 없다면(=heading_source_active False),
        # 최소한의 fallback으로 GPS 위치 변화량에서 heading을 역산한다.
        if not self.heading_source_active and prev_position is not None:
            dx = self.current_x - prev_position[0]
            dy = self.current_y - prev_position[1]
            if math.hypot(dx, dy) > 0.05:  # 노이즈로 인한 튐 방지 (5cm 미만 이동은 무시)
                self.current_heading = math.atan2(dy, dx)

    # ------------------------------------------------------------------
    # 콜백: 헤딩 (실제 센서값. GPS 콜백에서 절대 덮어쓰지 않는다)
    # ------------------------------------------------------------------
    def heading_callback(self, msg: Float32) -> None:
        self.heading_source_active = True
        raw_heading = math.radians(msg.data)

        if self.prev_heading is None:
            self.prev_heading = raw_heading
            self.current_heading = raw_heading
            return

        # -pi ~ pi 랩어라운드 보정
        diff = raw_heading - self.prev_heading
        if diff > math.pi:
            raw_heading -= 2 * math.pi
        elif diff < -math.pi:
            raw_heading += 2 * math.pi

        alpha = 0.2  # 저역통과 필터 계수 (필요시 파라미터화 가능)
        filtered = alpha * raw_heading + (1.0 - alpha) * self.prev_heading
        self.current_heading = (filtered + 2 * math.pi) % (2 * math.pi)
        self.prev_heading = self.current_heading

    # ------------------------------------------------------------------
    # 주기 실행: 경로 피팅 + 발행
    # ------------------------------------------------------------------
    def timer_callback(self) -> None:
        if len(self.global_waypoints) == 0:
            return

        result = self._compute_path_fit()
        self._publish_result(result)
        self._publish_tf()
        self._publish_global_path()

    # ------------------------------------------------------------------
    # 핵심 로직: 가장 가까운 인덱스 탐색 -> 앞쪽 N개 슬라이싱 -> 로컬 변환 -> 피팅 -> 샘플링
    # ------------------------------------------------------------------
    def _compute_path_fit(self) -> PathFitResult:
        result = PathFitResult()

        nearest_idx = self._find_nearest_index(self.current_x, self.current_y)

        n = len(self.global_waypoints)
        count = min(self.forward_index_count, n)
        # 루프 트랙 대응: 인덱스를 wrap-around 시켜서 앞쪽 count개만 연속으로 뽑는다.
        indices = [(nearest_idx + i) % n for i in range(count)]
        window_pts = self.global_waypoints[indices]

        cos_h = math.cos(self.current_heading)
        sin_h = math.sin(self.current_heading)

        dx = window_pts[:, 0] - self.current_x
        dy = window_pts[:, 1] - self.current_y
        rx = dx * cos_h + dy * sin_h
        ry = -dx * sin_h + dy * cos_h

        mask = (rx >= self.min_forward_x) & (rx <= self.lookahead_window_m)
        local_x = rx[mask]
        local_y = ry[mask]

        result.raw_local_x = local_x.tolist()
        result.raw_local_y = local_y.tolist()

        if len(local_x) < self.min_points_for_fit:
            nearest_pt = self.global_waypoints[nearest_idx]
            dist_to_nearest = math.hypot(
                nearest_pt[0] - self.current_x, nearest_pt[1] - self.current_y
            )
            self.get_logger().warn(
                f"피팅에 필요한 최소 점 개수 미달 ({len(local_x)} < {self.min_points_for_fit}). "
                f"가장 가까운 웨이포인트까지 거리={dist_to_nearest:.2f}m, "
                f"현재위치(x={self.current_x:.2f}, y={self.current_y:.2f}), "
                f"heading={math.degrees(self.current_heading):.1f}deg",
                throttle_duration_sec=2.0,
            )
            return result

        degree = self.poly_degree if len(local_x) >= self.poly_degree + 1 else 1

        try:
            # x가 단조증가하지 않으면 polyfit이 왜곡될 수 있으므로 정렬한다.
            order = np.argsort(local_x)
            sx = local_x[order]
            sy = local_y[order]

            raw_coeffs = np.polyfit(sx, sy, degree)  # 내림차순 [a, b, c, d]
            y_fit = np.polyval(raw_coeffs, sx)
            rmse = float(np.sqrt(np.mean((sy - y_fit) ** 2)))

            coeffs_ascending = list(raw_coeffs[::-1])
            while len(coeffs_ascending) < 4:
                coeffs_ascending.append(0.0)

            if np.any(np.isnan(coeffs_ascending)) or np.any(np.isinf(coeffs_ascending)):
                raise ValueError("피팅 계수에 NaN/Inf 발생")

            if np.any(np.abs(coeffs_ascending) > self.max_valid_coeff_abs):
                self.get_logger().warn(
                    f"피팅 계수가 임계값({self.max_valid_coeff_abs})을 초과했습니다. "
                    f"coeffs={coeffs_ascending}"
                )
                result.rmse = 999.0
                return result

            result.coeffs = coeffs_ascending[:4]
            result.rmse = rmse
            result.valid = True

            # ---- 앞쪽 curve_sample_count개 점 샘플링 (최종 산출물) ----
            x_start = max(self.min_forward_x, float(sx.min()))
            x_end = min(self.lookahead_window_m, float(sx.max()))
            if x_end <= x_start:
                x_end = x_start + 1e-3

            sample_x = np.linspace(x_start, x_end, self.curve_sample_count)
            poly = np.poly1d(raw_coeffs)
            sample_y = poly(sample_x)

            result.sampled_x = sample_x.tolist()
            result.sampled_y = sample_y.tolist()

        except (np.linalg.LinAlgError, ValueError) as e:
            self.get_logger().warn(f"경로 피팅 실패: {e}")
            result.rmse = 999.0
            result.valid = False

        return result

    def _find_nearest_index(self, x: float, y: float) -> int:
        """전체 웨이포인트 중 현재 위치에 가장 가까운 인덱스를 반환한다."""
        diffs = self.global_waypoints - np.array([x, y])
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        return int(np.argmin(dists_sq))

    # ------------------------------------------------------------------
    # 발행
    # ------------------------------------------------------------------
    def _publish_result(self, result: PathFitResult) -> None:
        coeffs_msg = Float32MultiArray()
        coeffs_msg.data = [float(c) for c in result.coeffs]
        self.pub_coeffs.publish(coeffs_msg)

        rmse_msg = Float32()
        rmse_msg.data = result.rmse
        self.pub_rmse.publish(rmse_msg)

        # 20개 샘플 -> 제어팀 최종 산출물 (Path + flat float array 둘 다 제공)
        stamp = self.get_clock().now().to_msg()

        sampled_path = Path()
        sampled_path.header.stamp = stamp
        sampled_path.header.frame_id = "base_link"
        for x, y in zip(result.sampled_x, result.sampled_y):
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            sampled_path.poses.append(pose)
        self.pub_local_path.publish(sampled_path)

        flat_msg = Float32MultiArray()
        flat = []
        for x, y in zip(result.sampled_x, result.sampled_y):
            flat.extend([float(x), float(y)])
        flat_msg.data = flat
        self.pub_waypoints_local.publish(flat_msg)

        # 디버깅용 원본 로컬 점(필터링만 된, 피팅 전 점들)
        raw_path = Path()
        raw_path.header.stamp = stamp
        raw_path.header.frame_id = "base_link"
        for x, y in zip(result.raw_local_x, result.raw_local_y):
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            raw_path.poses.append(pose)
        self.pub_raw_local_path.publish(raw_path)

    def _publish_global_path(self) -> None:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y in self.global_waypoints:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)
        self.pub_global_path.publish(path_msg)

    def _publish_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(self.current_heading / 2.0))
        t.transform.rotation.w = float(math.cos(self.current_heading / 2.0))

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

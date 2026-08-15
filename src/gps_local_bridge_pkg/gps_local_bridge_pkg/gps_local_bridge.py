#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_local_bridge

역할:
  기존 rtk_gps_package(waypoint_save.py)가 답사 시 저장해둔 GPS 웨이포인트(YAML, WGS84)와
  실시간 GPS(기본 /fix, 파라미터로 변경 가능), IMU(기본 handsfree/imu, 파라미터로 변경 가능) 데이터를 받아서,
  pure_pursuit_controller가 실제로 구독하는 아래 3개 토픽을 만들어 발행한다.

    /current_position      (geometry_msgs/Point)   실시간 로컬 x,y 위치
    /local_waypoints        (geometry_msgs/Point)   저장된 웨이포인트를 로컬 좌표로 순차 발행
    /local_waypoint_path     (nav_msgs/Path)          위 웨이포인트 전체를 한 번에 Path로도 발행
    /corrected_heading       (std_msgs/Float32)       IMU 원본 yaw + 오프셋 보정값

좌표계:
  UTM(Zone 52N, WGS84)으로 변환 후, "원점(origin)"을 기준으로 한 로컬 평면좌표(m)를 사용한다.
  origin은 기본적으로 저장된 웨이포인트의 첫 번째 점을 사용하며,
  origin_mode 파라미터를 'static'으로 바꾸면 origin_latitude/origin_longitude를 직접 지정할 수 있다.
"""

import os
import math
import threading
from collections import deque

import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Path

try:
    import pyproj
except ImportError as e:
    raise ImportError(
        "pyproj가 필요합니다. 설치: pip3 install pyproj --break-system-packages"
    ) from e

try:
    from tf_transformations import euler_from_quaternion
except ImportError as e:
    raise ImportError(
        "tf_transformations가 필요합니다. 설치: sudo apt install ros-${ROS_DISTRO}-tf-transformations"
    ) from e


def normalize_angle(angle_rad: float) -> float:
    """각도를 -pi ~ pi 범위로 정규화"""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class GpsLocalBridge(Node):
    def __init__(self):
        super().__init__('gps_local_bridge')

        # ────────────────────── 파라미터 선언 ──────────────────────
        self.declare_parameter('waypoint_file', '/home/aicar/waypoints/all_waypoint.yaml')
        self.declare_parameter('utm_zone', 52)
        self.declare_parameter('utm_hemisphere_north', True)

        # origin 설정: 'first_waypoint'(기본) 또는 'static'
        self.declare_parameter('origin_mode', 'first_waypoint')
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)

        # IMU-GPS 오프셋 (imu_gps_auto_calibrator.py에서 구한 값을 여기에 입력)
        self.declare_parameter('yaw_offset_deg', 0.0)

        # 웨이포인트 발행 관련
        self.declare_parameter('waypoint_publish_interval', 0.05)   # 초당 웨이포인트 발행 간격
        self.declare_parameter('startup_delay_sec', 3.0)             # 시작 전 대기(구독자 준비시간 확보)

        # GPS fix 품질 필터링
        self.declare_parameter('min_fix_status', 0)  # NavSatStatus.STATUS_FIX 이상만 사용

        # ────────────────────── 파라미터 값 로드 ──────────────────────
        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value
        self.utm_north = self.get_parameter('utm_hemisphere_north').get_parameter_value().bool_value
        self.origin_mode = self.get_parameter('origin_mode').get_parameter_value().string_value
        self.yaw_offset_rad = math.radians(
            self.get_parameter('yaw_offset_deg').get_parameter_value().double_value
        )
        self.waypoint_publish_interval = self.get_parameter(
            'waypoint_publish_interval').get_parameter_value().double_value
        self.startup_delay_sec = self.get_parameter('startup_delay_sec').get_parameter_value().double_value
        self.min_fix_status = self.get_parameter('min_fix_status').get_parameter_value().integer_value

        # ────────────────────── UTM 변환기 준비 ──────────────────────
        self.utm_proj = pyproj.Proj(
            proj='utm', zone=self.utm_zone, ellps='WGS84', datum='WGS84',
            south=not self.utm_north
        )
        self.wgs84_proj = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        self.transformer = pyproj.Transformer.from_proj(self.wgs84_proj, self.utm_proj, always_xy=True)

        # ────────────────────── 상태 변수 ──────────────────────
        self.origin_utm = None            # (x, y) - 로컬좌표계 원점
        self.origin_ready = False
        self.local_waypoints = []         # [{'x':, 'y':, 'yaw':}, ...] origin 기준 로컬좌표
        self.waypoint_send_queue = deque()
        self.current_gps_fix_count = 0
        self.current_imu_count = 0
        self._lock = threading.Lock()

        # ────────────────────── QoS 프로필 ──────────────────────
        # 센서 원본(GPS/IMU) 구독용 - 업스트림 드라이버가 대개 BEST_EFFORT로 발행
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # pure_pursuit_controller가 구독하는 realtime 토픽과 QoS를 반드시 맞춰야
        # RELIABLE 구독자가 데이터를 받을 수 있음 (QoS 미스매치 시 연결 자체가 안 됨)
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=50
        )

        # 웨이포인트 파일 로드 & origin 계산 이전에 토픽 이름도 파라미터화
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('imu_topic', 'handsfree/imu')
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # ────────────────────── 구독자 ──────────────────────
        self.gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self.gps_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, sensor_qos)
        self.request_sub = self.create_subscription(
            Bool, '/request_waypoints', self.request_waypoints_callback, 10)

        # ────────────────────── 발행자 ──────────────────────
        self.current_position_pub = self.create_publisher(Point, '/current_position', realtime_qos)
        self.local_waypoint_pub = self.create_publisher(Point, '/local_waypoints', realtime_qos)
        self.local_waypoint_path_pub = self.create_publisher(Path, '/local_waypoint_path', realtime_qos)
        self.corrected_heading_pub = self.create_publisher(Float32, '/corrected_heading', realtime_qos)

        # 파라미터 실시간 변경 콜백 (특히 yaw_offset_deg 현장 미세조정용)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ────────────────────── 웨이포인트 파일 로드 & origin 계산 ──────────────────────
        self._load_waypoints_and_setup_origin()

        # ────────────────────── 웨이포인트 발행 타이머 (지연 시작) ──────────────────────
        self._waypoint_send_timer = None
        self._startup_timer_handle = self.create_timer(
            self.startup_delay_sec, self._start_waypoint_broadcast_once
        )

        self.get_logger().info("🌉 GPS Local Bridge 시작됨")
        self.get_logger().info(f"   → 웨이포인트 파일: {self.waypoint_file}")
        self.get_logger().info(f"   → UTM Zone: {self.utm_zone}{'N' if self.utm_north else 'S'}")
        self.get_logger().info(f"   → Origin 모드: {self.origin_mode}")
        self.get_logger().info(f"   → Yaw offset: {math.degrees(self.yaw_offset_rad):.2f}°")
        self.get_logger().info(f"   → {self.startup_delay_sec:.1f}초 후 웨이포인트 발행 시작")

    # ============================================================
    # 초기화: 웨이포인트 로드 + 좌표 변환 + origin 설정
    # ============================================================
    def _load_waypoints_and_setup_origin(self):
        raw_waypoints = self._read_waypoint_file(self.waypoint_file)

        # ---- origin 결정 ----
        if self.origin_mode == 'static':
            origin_lat = self.get_parameter('origin_latitude').get_parameter_value().double_value
            origin_lon = self.get_parameter('origin_longitude').get_parameter_value().double_value
            if origin_lat == 0.0 and origin_lon == 0.0:
                self.get_logger().error(
                    "origin_mode='static'인데 origin_latitude/origin_longitude가 설정되지 않았습니다. "
                    "첫 GPS fix를 origin으로 임시 사용합니다."
                )
                self.origin_ready = False
            else:
                self.origin_utm = self._latlon_to_utm(origin_lat, origin_lon)
                self.origin_ready = True
                self.get_logger().info(f"📍 정적 Origin 설정: lat={origin_lat}, lon={origin_lon}")
        elif raw_waypoints:
            first = raw_waypoints[0]
            try:
                self.origin_utm = self._latlon_to_utm(first['latitude'], first['longitude'])
                self.origin_ready = True
                self.get_logger().info(
                    f"📍 첫 번째 웨이포인트를 Origin으로 설정: "
                    f"lat={first['latitude']:.6f}, lon={first['longitude']:.6f}"
                )
            except (KeyError, TypeError) as e:
                self.get_logger().error(f"첫 웨이포인트에서 origin 계산 실패: {e}")
                self.origin_ready = False
        else:
            self.get_logger().warn(
                "웨이포인트 파일이 비어있거나 없습니다. 첫 GPS fix를 origin으로 임시 사용합니다."
            )
            self.origin_ready = False

        # ---- origin이 준비된 경우에만 웨이포인트를 로컬좌표로 변환 ----
        if self.origin_ready and raw_waypoints:
            self.local_waypoints = self._convert_waypoints_to_local(raw_waypoints)
            self.get_logger().info(f"✅ 총 {len(self.local_waypoints)}개 웨이포인트를 로컬좌표로 변환 완료")
        elif not raw_waypoints:
            self.local_waypoints = []

    def _read_waypoint_file(self, filepath):
        """YAML 웨이포인트 파일 읽기. 실패해도 노드가 죽지 않고 빈 리스트 반환"""
        if not filepath or not os.path.exists(filepath):
            self.get_logger().error(f"❌ 웨이포인트 파일을 찾을 수 없습니다: {filepath}")
            return []

        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            self.get_logger().error(f"❌ YAML 파싱 오류: {e}")
            return []
        except OSError as e:
            self.get_logger().error(f"❌ 파일 읽기 오류: {e}")
            return []

        if not data or 'waypoints' not in data:
            self.get_logger().error("❌ YAML 파일에 'waypoints' 키가 없습니다.")
            return []

        waypoints = data['waypoints']
        valid_waypoints = []
        for i, wp in enumerate(waypoints):
            if 'latitude' in wp and 'longitude' in wp:
                valid_waypoints.append(wp)
            else:
                self.get_logger().warn(f"⚠️ waypoint #{i}에 latitude/longitude가 없어 건너뜁니다: {wp}")

        if not valid_waypoints:
            self.get_logger().error("❌ 유효한 waypoint가 하나도 없습니다.")

        return valid_waypoints

    def _latlon_to_utm(self, lat, lon):
        """위경도 -> UTM (x, y). 실패 시 예외를 그대로 올림(호출부에서 처리)"""
        x, y = self.transformer.transform(lon, lat)
        return (x, y)

    def _convert_waypoints_to_local(self, raw_waypoints):
        """원본 GPS 웨이포인트 리스트를 origin 기준 로컬 x,y로 변환하고,
        연속된 두 점 사이의 방향으로 yaw를 계산해 채운다."""
        local_wps = []
        utm_points = []

        for i, wp in enumerate(raw_waypoints):
            try:
                utm_x, utm_y = self._latlon_to_utm(wp['latitude'], wp['longitude'])
                utm_points.append((utm_x, utm_y))
            except Exception as e:
                self.get_logger().warn(f"⚠️ waypoint #{i} UTM 변환 실패, 건너뜁니다: {e}")
                utm_points.append(None)

        n = len(utm_points)
        for i in range(n):
            if utm_points[i] is None:
                continue

            local_x = utm_points[i][0] - self.origin_utm[0]
            local_y = utm_points[i][1] - self.origin_utm[1]

            # yaw: 다음 유효한 점을 향한 방향. 마지막 점은 이전 점 방향을 그대로 사용
            yaw = 0.0
            next_valid = None
            for j in range(i + 1, n):
                if utm_points[j] is not None:
                    next_valid = utm_points[j]
                    break

            if next_valid is not None:
                dx = next_valid[0] - utm_points[i][0]
                dy = next_valid[1] - utm_points[i][1]
                yaw = math.atan2(dy, dx)
            elif local_wps:
                yaw = local_wps[-1]['yaw']

            local_wps.append({'x': local_x, 'y': local_y, 'yaw': yaw})

        return local_wps

    # ============================================================
    # 웨이포인트 실시간 발행 (지연 시작 → 큐에서 하나씩 발행)
    # ============================================================
    def _start_waypoint_broadcast_once(self):
        """startup_delay_sec 후 딱 한 번 호출되어 웨이포인트 발행을 시작.
        이 콜백은 반복 타이머로 등록되므로, 최초 실행 직후 스스로를 취소해서 1회성으로 만든다."""
        self._startup_timer_handle.cancel()
        self._broadcast_all_waypoints()

    def _broadcast_all_waypoints(self):
        if not self.local_waypoints:
            self.get_logger().warn("⚠️ 발행할 웨이포인트가 없습니다. (파일 로드 실패 또는 origin 미설정)")
            return

        with self._lock:
            self.waypoint_send_queue = deque(self.local_waypoints)

        # 이미 실행 중인 발행 타이머가 있으면 정리 후 재생성 (재요청 대응)
        if self._waypoint_send_timer is not None:
            self._waypoint_send_timer.cancel()

        self._waypoint_send_timer = self.create_timer(
            self.waypoint_publish_interval, self._publish_next_waypoint_from_queue
        )
        self.get_logger().info(f"📡 웨이포인트 발행 시작: 총 {len(self.local_waypoints)}개")

    def _publish_next_waypoint_from_queue(self):
        with self._lock:
            if not self.waypoint_send_queue:
                # 큐가 비면 타이머 정지 + Path 메시지도 한 번에 발행
                if self._waypoint_send_timer is not None:
                    self._waypoint_send_timer.cancel()
                    self._waypoint_send_timer = None
                self._publish_full_path()
                self.get_logger().info("✅ 모든 웨이포인트 발행 완료")
                return
            wp = self.waypoint_send_queue.popleft()

        msg = Point()
        msg.x = float(wp['x'])
        msg.y = float(wp['y'])
        msg.z = float(wp['yaw'])   # pure_pursuit_node의 local_waypoint_callback은 msg.z를 yaw로 사용
        self.local_waypoint_pub.publish(msg)

    def _publish_full_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for wp in self.local_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.local_waypoint_path_pub.publish(path_msg)

    def request_waypoints_callback(self, msg: Bool):
        """/request_waypoints 에 True를 보내면 웨이포인트 전체를 다시 발행 (컨트롤러 재시작 대응)"""
        if msg.data:
            self.get_logger().info("🔄 웨이포인트 재발행 요청 수신")
            self._broadcast_all_waypoints()

    # ============================================================
    # 실시간 GPS → current_position
    # ============================================================
    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < self.min_fix_status:
            return

        try:
            utm_x, utm_y = self._latlon_to_utm(msg.latitude, msg.longitude)
        except Exception as e:
            self.get_logger().warn(f"⚠️ GPS→UTM 변환 오류: {e}", throttle_duration_sec=5.0)
            return

        # origin이 아직 없으면(웨이포인트 파일이 없거나 static origin 미설정) 첫 fix를 origin으로 임시 채택
        if not self.origin_ready:
            self.origin_utm = (utm_x, utm_y)
            self.origin_ready = True
            self.get_logger().warn(
                f"📍 웨이포인트/정적 origin이 없어 첫 GPS fix를 임시 origin으로 사용합니다: "
                f"lat={msg.latitude:.6f}, lon={msg.longitude:.6f}"
            )

        local_x = utm_x - self.origin_utm[0]
        local_y = utm_y - self.origin_utm[1]

        point_msg = Point()
        point_msg.x = local_x
        point_msg.y = local_y
        point_msg.z = 0.0
        self.current_position_pub.publish(point_msg)

        self.current_gps_fix_count += 1
        if self.current_gps_fix_count == 1:
            self.get_logger().info(f"📍 첫 GPS 위치 발행: local=({local_x:.2f}, {local_y:.2f})")

    # ============================================================
    # 실시간 IMU → corrected_heading
    # ============================================================
    def imu_callback(self, msg: Imu):
        try:
            quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            euler = euler_from_quaternion(quat)
            raw_yaw = euler[2]
        except Exception as e:
            self.get_logger().warn(f"⚠️ IMU 쿼터니언 변환 오류: {e}", throttle_duration_sec=5.0)
            return

        corrected = normalize_angle(raw_yaw + self.yaw_offset_rad)

        heading_msg = Float32()
        heading_msg.data = corrected
        self.corrected_heading_pub.publish(heading_msg)

        self.current_imu_count += 1
        if self.current_imu_count == 1:
            self.get_logger().info(
                f"🧭 첫 heading 발행: raw={math.degrees(raw_yaw):.1f}°, "
                f"offset={math.degrees(self.yaw_offset_rad):.1f}°, "
                f"corrected={math.degrees(corrected):.1f}°"
            )

    # ============================================================
    # 파라미터 실시간 변경 (현장에서 yaw_offset_deg 미세조정 등)
    # ============================================================
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'yaw_offset_deg':
                self.yaw_offset_rad = math.radians(param.value)
                self.get_logger().info(f"🧭 Yaw offset 변경됨: {param.value:.2f}°")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = GpsLocalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 GPS Local Bridge 정지됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

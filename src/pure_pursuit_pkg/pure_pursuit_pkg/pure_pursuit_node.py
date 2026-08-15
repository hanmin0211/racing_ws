#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_node.py
======================
검증된 core_logic.py 를 사용해 실제 Pure Pursuit 제어를 수행하는 노드.

처리 순서 (매 /vehicle_local_pose 콜백마다):
  1. 정적 waypoint 경로(YAML, 0.3m 간격 재샘플링본 권장) 로드 - 최초 1회
  2. 현재 차량 위치에서 가장 가까운 waypoint 탐색 (closest waypoint)
  3. 속도 기반 적응형 lookahead distance 계산
  4. lookahead point 계산 (원-직선 교차 보간, 검증된 정밀 방식)
  5. 차량 기준 로컬 윈도우 추출 + 3차 최소자승법 피팅 (곡률 추정 -> 코너 감속)
  6. Pure Pursuit 조향각 계산: delta = atan2(2*L*sin(alpha), Ld)
  7. AckermannDriveStamped 로 조향각+속도 발행 (/drive)
  8. RViz Marker 발행 (/pure_pursuit/markers): 현재위치, closest, lookahead, 로컬윈도우

[안전 관련 주의사항 - 실차 적용 전 필독]
- 이 코드는 시뮬레이션 환경에서 알고리즘(조향각 계산)만 수치적으로
  검증했고, 실제 ROS2 환경에서 빌드/실행 테스트는 하지 못했습니다.
  반드시 아래 순서로 안전하게 검증한 뒤 트랙에 투입하세요:
    1) 바퀴를 지면에서 띄운 상태로 빌드 및 노드 실행 -> 에러 없이 도는지 확인
    2) RViz에서 현재위치/closest/lookahead 마커가 실제 위치와 맞는지 육안 확인
    3) 저속(도보 속도 이하)으로 직선 구간 먼저 테스트
    4) 문제 없으면 곡선 구간, 이후 목표 속도까지 단계적으로 증가
"""

import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from pure_pursuit_pkg.core_logic import (
    find_closest_index, find_lookahead_point, compute_steering_angle,
    adaptive_lookahead, compute_cumulative_distance, select_local_window,
    transform_to_vehicle_frame, fit_cubic_least_squares,
    estimate_curvature_from_cubic,
)

# quaternion_to_yaw 는 gps_local_realtime_node 에 있으므로 여기서 재정의
def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_path(path_file):
    with open(path_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    waypoints = data['waypoints'] if isinstance(data, dict) else data
    xs = [wp['x'] for wp in waypoints]
    ys = [wp['y'] for wp in waypoints]
    return np.column_stack([xs, ys])


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ---- 파라미터 ----
        self.declare_parameter('path_file', '')
        self.declare_parameter('wheelbase', 0.88)          # HENES T870 실측값
        self.declare_parameter('max_steering_angle_deg', 30.0)  # HENES T870 실측 최대 타각(30~35도)의 안전마진 하한
        self.declare_parameter('min_valid_speed', 0.15)     # m/s, 이보다 느리면 헤딩 불신 -> 안전정지
        self.declare_parameter('target_speed', 1.0)          # m/s, 초기 튜닝은 저속 권장
        self.declare_parameter('use_adaptive_lookahead', True)
        self.declare_parameter('fixed_lookahead', 1.5)       # use_adaptive_lookahead=False 일 때 사용
        self.declare_parameter('k_ld', 0.6)
        self.declare_parameter('min_lookahead', 1.0)
        self.declare_parameter('max_lookahead', 4.0)
        self.declare_parameter('window_behind', 3.0)
        self.declare_parameter('window_ahead', 8.0)
        self.declare_parameter('curvature_speed_gain', 2.0)  # 코너 감속 강도
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('pose_topic', '/vehicle_local_pose')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('goal_tolerance', 0.5)

        path_file = self.get_parameter('path_file').value
        if not path_file or not os.path.isfile(path_file):
            self.get_logger().error(
                f"[오류] path_file 파라미터가 유효하지 않습니다: '{path_file}'. "
                "--ros-args -p path_file:=/home/han/gps_converter/waypoints_local_resampled_0.3.yaml "
                "처럼 지정하세요."
            )
            raise RuntimeError('path_file 파라미터 필요')

        self.path_xy = load_path(path_file)
        self.cum_dist = compute_cumulative_distance(self.path_xy)
        self.get_logger().info(
            f"[로드] '{path_file}' 에서 waypoint {len(self.path_xy)}개 로드 "
            f"(총 길이 {self.cum_dist[-1]:.2f} m)"
        )

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = math.radians(
            self.get_parameter('max_steering_angle_deg').value)
        self.last_closest_idx = 0

        pose_topic = self.get_parameter('pose_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        self.pose_sub = self.create_subscription(
            Odometry, pose_topic, self.pose_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/pure_pursuit/markers', 10)

        self.get_logger().info(
            f'[구독] {pose_topic}  ->  [발행] {drive_topic}, /pure_pursuit/markers'
        )

    # --------------------------------------------------------------
    def pose_callback(self, msg: Odometry):
        vx = msg.pose.pose.position.x
        vy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vehicle_xy = (vx, vy)

        # 0) 최소속도 게이트: GPS는 정지 상태에서 헤딩을 신뢰할 수 없고,
        #    IMU는 정지 상태에서도 자이로 드리프트가 계속 누적된다. 실제로
        #    정지 테스트에서 헤딩이 계속 틀어져 조향각이 36→40도로 커지는
        #    현상을 확인했다. 융합된 속도(twist)가 임계값 미만이면 헤딩을
        #    신뢰하지 않고 안전하게 정지 명령만 내보낸다.
        speed_now = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        min_valid_speed = self.get_parameter('min_valid_speed').value
        if speed_now < min_valid_speed:
            self.publish_drive(0.0, 0.0)
            self.get_logger().warn(
                f'[안전정지] 속도 {speed_now:.3f} m/s < 최소기준 {min_valid_speed} m/s '
                f'- 헤딩 신뢰 불가로 판단, 조향 계산 건너뛰고 정지 명령 발행',
                throttle_duration_sec=2.0)
            return

        # 1) 가장 가까운 waypoint (탐색 범위를 이전 인덱스 근처로 제한해 효율화 가능하나,
        #    초기 버전은 안전하게 전체 탐색)
        closest_idx, dists = find_closest_index(self.path_xy, vehicle_xy)
        self.last_closest_idx = closest_idx

        # 목표(경로 끝) 도달 판정
        goal_tol = self.get_parameter('goal_tolerance').value
        dist_to_goal = math.hypot(self.path_xy[-1][0] - vx, self.path_xy[-1][1] - vy)
        if closest_idx >= len(self.path_xy) - 2 and dist_to_goal < goal_tol:
            self.publish_drive(0.0, 0.0)
            self.get_logger().info('[종료] 경로 끝 도달, 정지', throttle_duration_sec=2.0)
            return

        # 2) lookahead distance 결정
        target_speed = self.get_parameter('target_speed').value
        if self.get_parameter('use_adaptive_lookahead').value:
            Ld = adaptive_lookahead(
                target_speed,
                k_ld=self.get_parameter('k_ld').value,
                min_ld=self.get_parameter('min_lookahead').value,
                max_ld=self.get_parameter('max_lookahead').value,
            )
        else:
            Ld = self.get_parameter('fixed_lookahead').value

        # 3) lookahead point
        lookahead_xy, lookahead_idx = find_lookahead_point(
            self.path_xy, vehicle_xy, closest_idx, Ld)

        # 4) 로컬 윈도우 + 3차 피팅 (곡률 기반 감속용)
        window = select_local_window(
            self.path_xy, self.cum_dist, closest_idx,
            behind=self.get_parameter('window_behind').value,
            ahead=self.get_parameter('window_ahead').value,
        )
        curvature = 0.0
        if len(window) >= 4:
            x_v, y_v = transform_to_vehicle_frame(window, vx, vy, heading)
            order = np.argsort(x_v)
            fit = fit_cubic_least_squares(x_v[order], y_v[order])
            curvature = abs(estimate_curvature_from_cubic(fit, x_eval=0.0))

        # 5) 조향각 계산
        delta, alpha = compute_steering_angle(
            vehicle_xy, heading, lookahead_xy, self.wheelbase, Ld)

        # 5-1) 안전 제한: HENES T870 실측 최대 타각(약 30~35도)을 넘지 않도록 클램프
        if abs(delta) > self.max_steering_angle:
            self.get_logger().warn(
                f'[안전제한] 계산된 조향각 {math.degrees(delta):.1f}deg 가 '
                f'최대치 {math.degrees(self.max_steering_angle):.1f}deg 를 초과해 클램프함',
                throttle_duration_sec=1.0)
            delta = math.copysign(self.max_steering_angle, delta)

        # 6) 곡률 기반 감속 (커브가 급할수록 감속, 최소속도 보장)
        gain = self.get_parameter('curvature_speed_gain').value
        min_speed = self.get_parameter('min_speed').value
        speed_cmd = max(min_speed, target_speed / (1.0 + gain * curvature))

        self.publish_drive(delta, speed_cmd)
        self.publish_markers(vehicle_xy, self.path_xy[closest_idx], lookahead_xy, window)

        self.get_logger().debug(
            f'closest={closest_idx} Ld={Ld:.2f} delta={math.degrees(delta):.2f}deg '
            f'curvature={curvature:.3f} speed={speed_cmd:.2f}'
        )

    # --------------------------------------------------------------
    def publish_drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    # --------------------------------------------------------------
    def publish_markers(self, vehicle_xy, closest_xy, lookahead_xy, window):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def make_point_marker(mid, x, y, r, g, b, scale=0.3):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'pure_pursuit'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            return m

        arr.markers.append(make_point_marker(0, vehicle_xy[0], vehicle_xy[1], 0.0, 0.4, 1.0, 0.35))   # 파랑: 현재 위치
        arr.markers.append(make_point_marker(1, closest_xy[0], closest_xy[1], 0.0, 1.0, 0.0, 0.3))    # 초록: closest
        arr.markers.append(make_point_marker(2, lookahead_xy[0], lookahead_xy[1], 1.0, 0.0, 0.0, 0.3))  # 빨강: lookahead

        # 로컬 윈도우를 선(line strip)으로 표시
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = now
        line.ns = 'pure_pursuit'
        line.id = 3
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 1.0, 0.0, 1.0  # 노랑
        line.pose.orientation.w = 1.0
        for p in window:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
            line.points.append(pt)
        arr.markers.append(line)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_sliding_window_node.py
============================
자율주행 로컬 경로 생성 노드.

전역 경로(/global_path)와 차량 위치(/odometry/filtered, GPS+IMU 융합 결과)를
받아서, 차량 기준(base_link) 로컬 경로(/local_path)를 만들어 발행한다.
로컬 경로가 전역 경로를 잘 추종하도록 다음 순서로 동작한다.

  1. 차량에서 가장 가까운 전역 웨이포인트를 찾는다.
  2. 그 점을 기준으로 '과거점(뒤쪽 n_back개) + 전방점(n_forward개)'을
     슬라이딩 윈도우로 잘라낸다. 과거점을 함께 넣어야 피팅이 안정적이고,
     그 위에서 미래점을 예측·생성할 수 있다.
  3. 잘라낸 점들을 차량 기준 좌표계로 변환한다. (차량 정면 = +x축)
  4. 최소자승법으로 3차 다항식 y = f(x) 을 피팅한다.
  5. 피팅된 곡선을 '호길이(거리) 단위'로 샘플링하여 10m 앞까지 미래점을
     생성한다. 윈도우가 10m에 못 미치면 다항식을 그대로 외삽(연장)하여
     미래점을 예측한다.
  6. 생성한 미래점들을 nav_msgs/Path 로 /local_path 에 발행한다.

파라미터 (실행 시 --ros-args -p 이름:=값 으로 조정 가능):
  odom_topic         : 차량 위치 토픽              (기본 /odometry/filtered)
  n_back             : 윈도우에 포함할 과거점 개수  (기본 5)
  n_forward          : 윈도우에 포함할 전방점 개수  (기본 20)
  poly_order         : 다항식 차수                  (기본 3)
  lookahead_distance : 미래점을 생성할 최대 거리[m] (기본 10.0)
  point_spacing      : 미래점 간 호길이 간격[m]     (기본 0.5)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, Float64MultiArray


class LocalSlidingWindowNode(Node):

  def __init__(self):
    super().__init__('local_sliding_window_node')

    # ---- 파라미터 선언 ----
    # (use_sim_time은 rclpy가 자동 선언하므로 여기서 다시 선언하지 않는다.
    #  rosbag 재생 시에는 --ros-args -p use_sim_time:=true 를 붙일 것.)
    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('n_back', 5)
    self.declare_parameter('n_forward', 20)
    self.declare_parameter('poly_order', 3)
    self.declare_parameter('lookahead_distance', 10.0)
    self.declare_parameter('point_spacing', 0.5)

    odom_topic = self.get_parameter('odom_topic').value
    self.n_back = int(self.get_parameter('n_back').value)
    self.n_forward = int(self.get_parameter('n_forward').value)
    self.poly_order = int(self.get_parameter('poly_order').value)
    self.lookahead_distance = float(
        self.get_parameter('lookahead_distance').value)
    self.point_spacing = float(self.get_parameter('point_spacing').value)

    # ---- 퍼블리셔 ----
    self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
    # rqt_plot 등으로 3차 다항식 계수(a,b,c,d) 변화를 확인하기 위한 퍼블리셔
    self.poly_coeffs_pub = self.create_publisher(
        Float64MultiArray, '/poly_coeffs', 10)
    # 차량 위치(x=0)에서의 부호 있는 곡률 κ (좌회전 +, 우회전 −). pure_pursuit의
    # 피드포워드 조향(atan(L·κ)) + 곡률 기반 속도조절에 사용.
    self.curvature_pub = self.create_publisher(Float64, '/curvature', 10)

    # ---- 서브스크라이버 ----
    # 전역 경로는 한 번만 발행되는 경우가 많으므로 TRANSIENT_LOCAL QoS로
    # 늦게 뜬 노드도 마지막 메시지를 받을 수 있도록 한다.
    global_path_qos = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.global_path_sub = self.create_subscription(
        Path, '/global_path', self.global_path_callback, global_path_qos)
    self.odom_sub = self.create_subscription(
        Odometry, odom_topic, self.odom_callback, 10)

    # ---- 상태 변수 ----
    self.global_waypoints = np.empty((0, 2))  # [[x, y], ...]
    self.car_x = 0.0
    self.car_y = 0.0
    self.car_yaw = 0.0
    self.is_path_received = False
    self.is_odom_received = False

    self.get_logger().info(
        f'local_sliding_window_node 시작 (odom_topic={odom_topic}, '
        f'n_back={self.n_back}, n_forward={self.n_forward}, '
        f'lookahead={self.lookahead_distance}m, spacing={self.point_spacing}m)')

  def global_path_callback(self, msg):
    pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
    self.global_waypoints = np.array(pts) if pts else np.empty((0, 2))
    self.is_path_received = True

  def odom_callback(self, msg):
    self.car_x = msg.pose.pose.position.x
    self.car_y = msg.pose.pose.position.y

    # 쿼터니언 -> Yaw
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    self.car_yaw = np.arctan2(siny_cosp, cosy_cosp)

    self.is_odom_received = True
    self.process_sliding_window()

  def get_closest_index(self):
    if len(self.global_waypoints) == 0:
      return 0
    d = np.hypot(self.global_waypoints[:, 0] - self.car_x,
                 self.global_waypoints[:, 1] - self.car_y)
    return int(np.argmin(d))

  def process_sliding_window(self):
    if not self.is_path_received or not self.is_odom_received:
      return
    total = len(self.global_waypoints)
    if total < 2:
      return

    closest_idx = self.get_closest_index()

    # 1. 과거점 + 전방점 슬라이딩 윈도우 (폐루프 트랙 전제, 순환 인덱싱)
    win = min(self.n_back + self.n_forward + 1, total)
    indices = [(closest_idx - self.n_back + i) % total for i in range(win)]
    window_global = self.global_waypoints[indices]

    # 2. 전역 좌표계 -> 차량 기준(base_link) 좌표계. 차량 정면이 +x축.
    dx = window_global[:, 0] - self.car_x
    dy = window_global[:, 1] - self.car_y
    cos_y, sin_y = np.cos(self.car_yaw), np.sin(self.car_yaw)
    x_local = dx * cos_y + dy * sin_y
    y_local = -dx * sin_y + dy * cos_y

    # y=f(x) 3차 피팅은 x가 단조증가해야 왜곡이 없다. 급커브로 뒤집히는
    # 구간을 막기 위해 x 기준으로 정렬한 뒤 중복 x를 제거한다.
    order = np.argsort(x_local)
    x_sorted = x_local[order]
    y_sorted = y_local[order]
    keep = np.concatenate(([True], np.diff(x_sorted) > 1e-6))
    x_fit = x_sorted[keep]
    y_fit = y_sorted[keep]
    if len(x_fit) <= self.poly_order:
      self.get_logger().warn('피팅에 필요한 점이 부족합니다.')
      return

    # 3. 최소자승법 3차 다항식 피팅
    try:
      coeffs = np.polyfit(x_fit, y_fit, self.poly_order)
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(f'다항식 피팅 실패: {e}')
      return
    poly = np.poly1d(coeffs)
    dpoly = np.polyder(poly)   # f'(x): 호길이 계산에 사용
    ddpoly = np.polyder(poly, 2)  # f''(x): 곡률 계산에 사용

    # 계수 발행 (rqt_plot에서 /poly_coeffs/data[0..3])
    cmsg = Float64MultiArray()
    cmsg.data = [float(c) for c in coeffs]
    self.poly_coeffs_pub.publish(cmsg)

    # 차량 위치(x=0)에서의 부호 있는 곡률: κ = f''(0) / (1 + f'(0)^2)^1.5
    fp0 = float(dpoly(0.0))
    fpp0 = float(ddpoly(0.0))
    curvature = fpp0 / (1.0 + fp0 * fp0) ** 1.5
    self.curvature_pub.publish(Float64(data=curvature))

    # 4. 피팅 곡선을 호길이(거리) 단위로 샘플링하여 미래점 생성.
    #    차량은 base_link 원점(0,0)에 있으므로 x=0에서 전방으로 진행하며
    #    ds = sqrt(1 + f'(x)^2) dx 로 누적 거리를 적분한다. 윈도우가 닿는
    #    범위를 넘어가면 다항식을 그대로 외삽하여 미래점을 예측한다.
    future_pts = self.sample_by_arclength(poly, dpoly)

    # 5. /local_path 발행
    path_msg = Path()
    path_msg.header.stamp = self.get_clock().now().to_msg()
    path_msg.header.frame_id = 'base_link'
    poses = []
    for fx, fy in future_pts:
      pose = PoseStamped()
      pose.header = path_msg.header
      pose.pose.position.x = float(fx)
      pose.pose.position.y = float(fy)
      pose.pose.position.z = 0.0
      pose.pose.orientation.w = 1.0
      poses.append(pose)
    path_msg.poses = poses
    self.local_path_pub.publish(path_msg)

  def sample_by_arclength(self, poly, dpoly):
    """x=0에서 전방으로 진행하며 호길이 point_spacing 간격으로 점을 뽑아
    lookahead_distance(예: 10m)까지 미래점 리스트 [(x, y), ...] 를 만든다.

    누적 호길이 s(x) = ∫sqrt(1+f'(x)^2)dx 를 촘촘한 격자로 적분한 뒤,
    목표 호길이(0, spacing, 2*spacing, ..., lookahead)에 해당하는 x를
    보간(interp)으로 역산한다. 이렇게 하면 간격이 정확히 균일하고
    마지막 점이 정확히 lookahead_distance에 오며, 곡선일수록 x보다
    호길이가 길어지는 것도 자연스럽게 반영된다."""
    dx = 0.02                      # 적분 격자 간격(작을수록 정확)
    max_x = 200.0                  # 안전용 x 상한
    xs = np.arange(0.0, max_x, dx)
    ds = np.sqrt(1.0 + dpoly(xs) ** 2) * dx
    s = np.concatenate(([0.0], np.cumsum(ds)[:-1]))  # 각 xs에서의 누적 호길이

    n = int(round(self.lookahead_distance / self.point_spacing))
    targets = np.linspace(0.0, self.lookahead_distance, n + 1)
    # 피팅 곡선을 외삽해도 닿지 못하는 거리는 잘라낸다(수치 안전장치).
    targets = targets[targets <= s[-1]]
    x_at = np.interp(targets, s, xs)
    return [(float(x), float(poly(x))) for x in x_at]


def main(args=None):
  rclpy.init(args=args)
  node = LocalSlidingWindowNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

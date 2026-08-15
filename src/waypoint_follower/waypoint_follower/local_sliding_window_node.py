#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_sliding_window_node.py
============================
자율주행 로컬 경로 생성 노드.

전역 경로(/global_path)와 차량 위치(/odometry/filtered)를 받아 차량 기준
(base_link) 로컬 경로(/local_path)를 만들어 발행한다.

★ 계산 로직은 local_path_core.py 에 분리돼 있다. 오프라인 하네스
  (tools/local_path_harness.py)가 **같은 코드**를 import 해서 검증하므로,
  "하네스는 통과했는데 실차는 다르다"는 괴리가 생기지 않는다.

2026-08-15 개정 (오프라인 하네스로 검증):
  · y=f(x) 3차 피팅 → **호길이 매개변수 피팅 x(s), y(s)**
    급코너에서 경로가 차량 뒤로 말리면 x가 단조롭지 않아 y=f(x)로는 표현이
    불가능했다(실측: 179지점 중 39지점=22%에서 전제 붕괴).
  · 순환 인덱싱(`% total`) → **열린/닫힌 경로 자동 판정**
    기록 경로는 시작-끝이 3.82m 벌어진 열린 경로인데 순환시켜서 이음매마다
    경로가 3.82m 튀었다(실측 25회). 개정 후 0회.
  · 곡률: 차량 위치(s=0) → **전방 preview 구간 최대 |κ|**
    예전엔 코너에 진입한 뒤에야 곡률이 커져 선제 감속이 불가능했다.
    개정 후 코너 7m 전부터 곡률이 올라간다.
  · 완주 판정(/goal_reached) 추가 — 열린 경로 끝에서 정지할 수 있게.

발행:
  /local_path    (Path, base_link)  전방 로컬 경로
  /curvature     (Float64)          전방 preview 최대 곡률(부호 포함)
  /poly_coeffs   (Float64MultiArray) 진단용 [x(s)계수..., y(s)계수...]
  /goal_reached  (Bool)             열린 경로 완주 여부
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, Float64MultiArray

from waypoint_follower.local_path_core import (LocalPathParams,
                                               build_local_path,
                                               is_closed_path)


class LocalSlidingWindowNode(Node):

  def __init__(self):
    super().__init__('local_sliding_window_node')

    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('n_back', 5)
    self.declare_parameter('n_forward', 20)
    self.declare_parameter('poly_order', 3)
    self.declare_parameter('lookahead_distance', 10.0)
    self.declare_parameter('point_spacing', 0.5)
    # 곡률을 앞쪽 몇 m 구간에서 볼지 (선제 감속 여유). 속도×수 초 정도로 잡는다.
    self.declare_parameter('curvature_preview', 4.0)
    # 'auto' | 'true' | 'false' — 경로가 닫힌 루프인지. auto면 시작-끝 간격으로 판정.
    self.declare_parameter('closed_path', 'auto')
    self.declare_parameter('goal_tolerance', 1.0)

    odom_topic = self.get_parameter('odom_topic').value
    cp = str(self.get_parameter('closed_path').value).lower()
    closed_param = 'auto' if cp == 'auto' else (cp in ('true', '1', 'yes'))

    self.params = LocalPathParams(
        n_back=int(self.get_parameter('n_back').value),
        n_forward=int(self.get_parameter('n_forward').value),
        poly_order=int(self.get_parameter('poly_order').value),
        lookahead_distance=float(
            self.get_parameter('lookahead_distance').value),
        point_spacing=float(self.get_parameter('point_spacing').value),
        curvature_preview=float(
            self.get_parameter('curvature_preview').value),
        closed_path=closed_param,
        goal_tolerance=float(self.get_parameter('goal_tolerance').value),
    )

    self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
    self.poly_coeffs_pub = self.create_publisher(
        Float64MultiArray, '/poly_coeffs', 10)
    self.curvature_pub = self.create_publisher(Float64, '/curvature', 10)
    self.goal_pub = self.create_publisher(Bool, '/goal_reached', 10)

    global_path_qos = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Path, '/global_path', self.global_path_callback,
                             global_path_qos)
    self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    self.global_waypoints = np.empty((0, 2))
    self.closed = None
    self.prev_idx = None          # 최근접 인덱스 국소 탐색용(진행 추적)
    self.is_path_received = False
    self.goal_latched = False

    self.get_logger().info(
        f'local_sliding_window_node 시작 (odom={odom_topic}, '
        f'window={self.params.n_back}+{self.params.n_forward}, '
        f'lookahead={self.params.lookahead_distance}m, '
        f'곡률 preview={self.params.curvature_preview}m)')

  def global_path_callback(self, msg):
    pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
    self.global_waypoints = np.array(pts) if pts else np.empty((0, 2))
    self.prev_idx = None
    self.goal_latched = False
    if len(self.global_waypoints) >= 2:
      if self.params.closed_path == 'auto':
        self.closed = is_closed_path(self.global_waypoints,
                                     self.params.closed_gap_thresh)
      else:
        self.closed = bool(self.params.closed_path)
      gap = float(np.hypot(
          self.global_waypoints[0, 0] - self.global_waypoints[-1, 0],
          self.global_waypoints[0, 1] - self.global_waypoints[-1, 1]))
      self.get_logger().info(
          f'전역 경로 수신: {len(self.global_waypoints)}점, '
          f'시작-끝 {gap:.2f}m → {"닫힌 루프" if self.closed else "열린 경로"}')
    self.is_path_received = True

  def odom_callback(self, msg):
    if not self.is_path_received or len(self.global_waypoints) < 2:
      return

    cx = msg.pose.pose.position.x
    cy = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    r = build_local_path(self.global_waypoints, cx, cy, float(yaw),
                         self.params, prev_idx=self.prev_idx,
                         closed=self.closed)
    self.prev_idx = r['closest']

    if r['goal_reached']:
      if not self.goal_latched:
        self.get_logger().info(
            f'★ 경로 완주 (남은 거리 {r["remaining_dist"]:.2f}m) → 정지 신호 발행')
        self.goal_latched = True
      self.goal_pub.publish(Bool(data=True))
      # 완주 시 /local_path 를 발행하지 않는다 → 하류(pure_pursuit)가
      # 경로 타임아웃으로 안전 정지한다.
      return

    self.goal_pub.publish(Bool(data=False))

    if not r['ok'] or r['points'] is None:
      self.get_logger().warn('로컬 경로 생성 실패 — 발행 보류',
                             throttle_duration_sec=2.0)
      return

    self.curvature_pub.publish(Float64(data=float(r['curvature'])))

    if r['coeffs'] is not None:
      cmsg = Float64MultiArray()
      cmsg.data = [float(v) for v in r['coeffs'][0]] + \
                  [float(v) for v in r['coeffs'][1]]
      self.poly_coeffs_pub.publish(cmsg)

    path_msg = Path()
    path_msg.header.stamp = self.get_clock().now().to_msg()
    path_msg.header.frame_id = 'base_link'
    poses = []
    for fx, fy in r['points']:
      pose = PoseStamped()
      pose.header = path_msg.header
      pose.pose.position.x = float(fx)
      pose.pose.position.y = float(fy)
      pose.pose.position.z = 0.0
      pose.pose.orientation.w = 1.0
      poses.append(pose)
    path_msg.poses = poses
    self.local_path_pub.publish(path_msg)


def main(args=None):
  rclpy.init(args=args)
  node = LocalSlidingWindowNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

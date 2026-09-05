#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
global_path_publisher.py
=========================
사전에 기록된 웨이포인트 YAML 파일(waypoints_local_resampled_0.3.yaml)을
읽어서 nav_msgs/Path 메시지로 변환한 뒤 /global_path 토픽으로 발행한다.

local_sliding_window_node(waypoint_follower 패키지 버전)가 /global_path를
구독해서 슬라이딩 윈도우 + 3차 곡선 피팅을 수행하므로, 이 노드를 먼저
띄워 전역 경로를 공급해줘야 한다.

/global_path 구독 쪽이 TRANSIENT_LOCAL QoS로 맞춰져 있으므로, 이 노드도
동일하게 TRANSIENT_LOCAL로 발행하여 구독 노드가 나중에 떠도 마지막
메시지를 받을 수 있도록 한다. 다만 QoS 설정이 어긋나는 경우를 대비해
1초 주기로도 재발행한다.
"""

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from waypoint_follower.site_origin import reconcile_origin

# 사용하는 웨이포인트 파일 경로 (환경에 맞게 수정 가능)
# 현재 사용 중인 웨이포인트. (예전 기본값 waypoints_local_resampled_0.3.yaml 은
# 옛 파이프라인 잔재라, 단독 실행 시 엉뚱한 경로가 로드되는 원인이었다.)
DEFAULT_PATH_FILE = (
    '/home/han/racing_ws/src/pure_pursuit_pkg/config/'
    'waypoints_recorded_resampled_0.5.yaml'
)


class GlobalPathPublisher(Node):

  def __init__(self):
    super().__init__('global_path_publisher')

    self.declare_parameter('path_file', DEFAULT_PATH_FILE)
    self.declare_parameter('frame_id', 'map')
    # 원점 검증 강도: strict(기본, 다른 장소면 발행 거부) / warn / off
    self.declare_parameter('origin_check', 'strict')

    path_file = self.get_parameter('path_file').value
    self.frame_id = self.get_parameter('frame_id').value
    self.origin_check = str(self.get_parameter('origin_check').value)

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.pub = self.create_publisher(Path, '/global_path', qos)

    self.path_file = path_file
    self.path_msg = self.load_global_path(path_file)

    if self.path_msg is not None:
      # 최초 1회 즉시 발행
      self.publish_path()
      # 이후 늦게 뜨는 구독자를 위해 1초마다 재발행
      self.timer = self.create_timer(1.0, self.publish_path)
    else:
      # ★ 경로를 못 실었으면 조용히 있지 말고 계속 외친다.
      #   bringup 은 노드를 10개쯤 띄우므로 한 번만 찍힌 에러는 묻힌다.
      #   하류는 '경로 끊김' 만 말해서 원인을 상류로 되짚게 만든다(§8.1).
      #   여기서 5초마다 반복해 그 추적을 없앤다.
      self.timer = self.create_timer(5.0, self.nag)

  def nag(self):
    self.get_logger().error(
        f'❌ 전역 경로 없음 — /global_path 를 발행하지 않는다. '
        f'({self.path_file}) 하류의 "경로 끊김" 은 이것이 원인이다.')

  def load_global_path(self, path_file):
    """웨이포인트 YAML 로드 + **원점 검증**.

    ★ 왜 검증하나
      로컬좌표는 (UTM − 원점)이라 원점을 모르면 해석할 수 없다. 예전엔 이
      파일에 원점이 안 적혀 있어, 장소가 바뀌어 site_origin.yaml 을 갱신하면
      옛 파일이 **조용히** 150km 어긋난 경로로 읽혔다(2026-08-17 실사고 계열).
      정지지점 쪽(bringup.load_stop_points)엔 이미 이 검사가 있었는데
      정작 주행경로엔 없었다. 여기서 막는다.

    스탬프가 있으면 현재 원점으로 환산하고, 1km 이상 차이 나면(=다른 장소)
    strict 모드에서 **발행을 거부**한다. 경로가 없으면 차는 안 움직인다 —
    엉뚱한 경로를 쫓는 것보다 안전하다.
    """
    try:
      with open(path_file, 'r') as f:
        data = yaml.safe_load(f)
      raw_points = data.get('waypoints', data.get('poses', list(data.values())[0]))

      pts = []
      for pt in raw_points:
        if isinstance(pt, dict):
          pts.append([float(pt['x']), float(pt['y'])])
        else:
          pts.append([float(pt[0]), float(pt[1])])

      waypoints = np.array(pts)
      if len(waypoints) == 0:
        self.get_logger().error(f'전역 경로가 비어 있다: {path_file}')
        return None

      if self.origin_check != 'off':
        dx, dy, note = reconcile_origin(data, path_file, self.get_logger())
        if note in ('shifted', 'match', 'no-stamp'):
          if dx or dy:
            waypoints = waypoints + np.array([dx, dy])
        fatal = (note == 'epsg-mismatch'
                 or (note == 'shifted' and (dx * dx + dy * dy) ** 0.5 > 1000.0))
        if fatal and self.origin_check == 'strict':
          self.get_logger().error(
              '❌ 이 웨이포인트는 다른 장소에서 기록된 것이다 — 전역 경로를 '
              '발행하지 않는다. 이 장소에서 다시 기록하거나, 확인 후 '
              'origin_check:=warn 으로 강제할 것.')
          return None

      x0, x1 = float(waypoints[:, 0].min()), float(waypoints[:, 0].max())
      y0, y1 = float(waypoints[:, 1].min()), float(waypoints[:, 1].max())
      seg = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
      self.get_logger().info(
          f'전역 경로 로드 완료: {path_file} (총 {len(waypoints)}개 점, '
          f'{float(seg.sum()):.1f}m)')
      # 범위를 같이 찍는다 — 원점이 틀리면 여기서 바로 눈에 띈다.
      self.get_logger().info(
          f'  로컬 범위 x[{x0:.1f}, {x1:.1f}] y[{y0:.1f}, {y1:.1f}] '
          f'(정상이면 0~수백 m)')
      return self.build_path_msg(waypoints)
    except Exception as e:
      self.get_logger().error(f'전역 경로 로드 실패: {e}')
      return None

  def build_path_msg(self, waypoints):
    path_msg = Path()
    path_msg.header.stamp = self.get_clock().now().to_msg()
    path_msg.header.frame_id = self.frame_id

    poses_list = []
    for x, y in waypoints:
      pose = PoseStamped()
      pose.header = path_msg.header
      pose.pose.position.x = float(x)
      pose.pose.position.y = float(y)
      pose.pose.position.z = 0.0
      pose.pose.orientation.w = 1.0
      poses_list.append(pose)

    path_msg.poses = poses_list
    return path_msg

  def publish_path(self):
    if self.path_msg is None:
      return
    self.path_msg.header.stamp = self.get_clock().now().to_msg()
    self.pub.publish(self.path_msg)


def main(args=None):
  # ExternalShutdownException 은 Ctrl-C/SIGTERM 시 rclpy 가 내는 정상 종료
  # 신호다. 안 잡으면 종료할 때마다 traceback 이 찍혀 진짜 에러를 가린다.
  rclpy.init(args=args)
  node = GlobalPathPublisher()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

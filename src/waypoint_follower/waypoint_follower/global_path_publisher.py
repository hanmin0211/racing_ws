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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

# 사용하는 웨이포인트 파일 경로 (환경에 맞게 수정 가능)
DEFAULT_PATH_FILE = (
    '/home/han/racing_ws/src/pure_pursuit_pkg/config/'
    'waypoints_local_resampled_0.3.yaml'
)


class GlobalPathPublisher(Node):

  def __init__(self):
    super().__init__('global_path_publisher')

    self.declare_parameter('path_file', DEFAULT_PATH_FILE)
    self.declare_parameter('frame_id', 'map')

    path_file = self.get_parameter('path_file').value
    self.frame_id = self.get_parameter('frame_id').value

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.pub = self.create_publisher(Path, '/global_path', qos)

    self.path_msg = self.load_global_path(path_file)

    if self.path_msg is not None:
      # 최초 1회 즉시 발행
      self.publish_path()
      # 이후 늦게 뜨는 구독자를 위해 1초마다 재발행
      self.timer = self.create_timer(1.0, self.publish_path)

  def load_global_path(self, path_file):
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
      self.get_logger().info(
          f'전역 경로 로드 완료: {path_file} (총 {len(waypoints)}개 점)'
      )
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
  rclpy.init(args=args)
  node = GlobalPathPublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dummy_odom_track.py
=====================
로컬라이제이션(GPS/IMU 등)이 아직 준비되지 않은 상태에서, 사전에
기록된 웨이포인트 YAML 파일(waypoints_local_resampled_0.3.yaml)의
트랙을 따라 차량이 실제로 주행하는 것처럼 가상 오도메트리를
/odometry/filtered 로 발행하는 노드.

기존 dummy_odom.py는 원점에서 x축을 따라 무한정 직진만 해서, 폐루프
트랙 검증에는 맞지 않았다. 이 노드는 웨이포인트 배열을 순서대로
따라가며, 이동 방향으로 heading(yaw)도 함께 계산해서 발행한다.
"""

import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# 사용하는 웨이포인트 파일 경로 (환경에 맞게 수정 가능)
DEFAULT_PATH_FILE = (
    '/home/han/racing_ws/src/pure_pursuit_pkg/config/'
    'waypoints_local_resampled_0.3.yaml'
)


def yaw_to_quaternion(yaw):
  return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DummyOdomTrack(Node):

  def __init__(self):
    super().__init__('dummy_odom_track')

    self.declare_parameter('path_file', DEFAULT_PATH_FILE)
    self.declare_parameter('speed_mps', 3.0)  # 가상 주행 속도 (m/s)
    self.declare_parameter('publish_rate_hz', 10.0)

    path_file = self.get_parameter('path_file').value
    self.speed_mps = self.get_parameter('speed_mps').value
    rate_hz = self.get_parameter('publish_rate_hz').value

    self.waypoints = self.load_waypoints(path_file)
    if self.waypoints is None or len(self.waypoints) < 2:
      self.get_logger().error('웨이포인트를 불러오지 못해 노드를 시작할 수 없습니다.')
      raise RuntimeError('웨이포인트 로드 실패')

    # 웨이포인트 간 누적 거리(arc length) 미리 계산
    diffs = np.diff(self.waypoints, axis=0)
    seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    self.cumdist = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    self.total_length = self.cumdist[-1]

    self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.tf_broadcaster = TransformBroadcaster(self)
    self.traveled = 0.0
    self.dt = 1.0 / rate_hz
    self.timer = self.create_timer(self.dt, self.timer_callback)

    self.get_logger().info(
        f'가상 트랙 추종 오도메트리 시작: 총 {len(self.waypoints)}개 점, '
        f'트랙 길이 {self.total_length:.1f}m, 속도 {self.speed_mps}m/s'
    )

  def load_waypoints(self, path_file):
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
      return np.array(pts)
    except Exception as e:
      self.get_logger().error(f'웨이포인트 로드 실패: {e}')
      return None

  def get_pose_at_distance(self, dist):
    """트랙 시작점부터 dist(m)만큼 이동했을 때의 (x, y, yaw)를 보간해서 반환.
    폐루프 트랙이므로 total_length를 넘어가면 처음으로 순환."""
    dist = dist % self.total_length
    idx = int(np.searchsorted(self.cumdist, dist, side='right') - 1)
    idx = max(0, min(idx, len(self.waypoints) - 2))

    seg_start_dist = self.cumdist[idx]
    seg_len = self.cumdist[idx + 1] - seg_start_dist
    ratio = 0.0 if seg_len == 0 else (dist - seg_start_dist) / seg_len

    p0 = self.waypoints[idx]
    p1 = self.waypoints[idx + 1]
    x = p0[0] + (p1[0] - p0[0]) * ratio
    y = p0[1] + (p1[1] - p0[1]) * ratio
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return x, y, yaw

  def timer_callback(self):
    x, y, yaw = self.get_pose_at_distance(self.traveled)

    msg = Odometry()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    self.pub.publish(msg)

    # RViz에서 base_link 프레임(local_path)을 map 기준으로 올바르게
    # 그리려면 map -> base_link TF가 반드시 필요하다.
    tf_msg = TransformStamped()
    tf_msg.header.stamp = msg.header.stamp
    tf_msg.header.frame_id = 'map'
    tf_msg.child_frame_id = 'base_link'
    tf_msg.transform.translation.x = float(x)
    tf_msg.transform.translation.y = float(y)
    tf_msg.transform.translation.z = 0.0
    tf_msg.transform.rotation.x = qx
    tf_msg.transform.rotation.y = qy
    tf_msg.transform.rotation.z = qz
    tf_msg.transform.rotation.w = qw
    self.tf_broadcaster.sendTransform(tf_msg)

    self.traveled += self.speed_mps * self.dt


def main(args=None):
  rclpy.init(args=args)
  node = DummyOdomTrack()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_odom_publisher.py
=====================
시뮬레이션용 차량 위치 발행 노드. 실제 GPS+IMU 융합(/odometry/filtered)이
없어도 RViz로 로컬 경로 추종을 확인할 수 있도록, 전역 경로(웨이포인트)를
따라 일정 속도로 '가상의 차량'을 움직이며 다음을 발행한다.

  - nav_msgs/Odometry : /odometry/filtered (map 프레임, base_link 자세)
  - TF                : map -> base_link  (RViz가 base_link 프레임의
                        /local_path 를 전역 경로 위에 겹쳐 그리기 위해 필요)

차량은 웨이포인트를 호길이 기준으로 보간하며 진행하고, 진행 방향(접선)을
yaw로 사용한다. 폐루프 트랙이면 끝에서 처음으로 순환한다.

파라미터:
  path_file : 웨이포인트 YAML 경로
  speed     : 진행 속도[m/s]           (기본 3.0)
  rate      : 발행 주기[Hz]            (기본 20.0)
  loop      : 끝점 도달 시 순환 여부    (기본 True)
"""

import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

DEFAULT_PATH_FILE = (
    '/home/han/racing_ws/src/pure_pursuit_pkg/config/'
    'waypoints_local_resampled_0.3.yaml'
)


def yaw_to_quat(yaw):
  return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class SimOdomPublisher(Node):

  def __init__(self):
    super().__init__('sim_odom_publisher')

    self.declare_parameter('path_file', DEFAULT_PATH_FILE)
    self.declare_parameter('speed', 3.0)
    self.declare_parameter('rate', 20.0)
    self.declare_parameter('loop', True)

    path_file = self.get_parameter('path_file').value
    self.speed = float(self.get_parameter('speed').value)
    rate = float(self.get_parameter('rate').value)
    self.loop = bool(self.get_parameter('loop').value)

    self.wp = self.load_waypoints(path_file)
    # 웨이포인트 누적 호길이(closed loop 가정: 마지막->처음 구간도 포함)
    seg = np.linalg.norm(np.diff(self.wp, axis=0, append=self.wp[:1]), axis=1)
    self.cum = np.concatenate(([0.0], np.cumsum(seg)))
    self.total_len = float(self.cum[-1])
    self.s = 0.0  # 현재 진행 호길이

    self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.tf_broadcaster = TransformBroadcaster(self)
    self.dt = 1.0 / rate
    self.timer = self.create_timer(self.dt, self.on_timer)

    self.get_logger().info(
        f'sim_odom_publisher 시작: {len(self.wp)}점, 트랙길이 '
        f'{self.total_len:.1f}m, 속도 {self.speed}m/s, {rate}Hz')

  def load_waypoints(self, path_file):
    data = yaml.safe_load(open(path_file, 'r'))
    raw = data.get('waypoints', data.get('poses', list(data.values())[0]))
    pts = []
    for p in raw:
      if isinstance(p, dict):
        pts.append([float(p['x']), float(p['y'])])
      else:
        pts.append([float(p[0]), float(p[1])])
    return np.array(pts)

  def pose_at(self, s):
    """호길이 s에서의 (x, y, yaw)를 웨이포인트 보간으로 구한다."""
    s = s % self.total_len
    i = int(np.searchsorted(self.cum, s) - 1)
    i = max(0, min(i, len(self.wp) - 1))
    seg_len = self.cum[i + 1] - self.cum[i]
    t = 0.0 if seg_len < 1e-9 else (s - self.cum[i]) / seg_len
    p0 = self.wp[i]
    p1 = self.wp[(i + 1) % len(self.wp)]
    x, y = p0 + t * (p1 - p0)
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return float(x), float(y), yaw

  def on_timer(self):
    x, y, yaw = self.pose_at(self.s)
    now = self.get_clock().now().to_msg()

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = yaw_to_quat(yaw)
    odom.twist.twist.linear.x = self.speed
    self.pub.publish(odom)

    tf = TransformStamped()
    tf.header.stamp = now
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.rotation = yaw_to_quat(yaw)
    self.tf_broadcaster.sendTransform(tf)

    # 다음 스텝 전진 (loop=False면 끝에서 정지)
    nxt = self.s + self.speed * self.dt
    if not self.loop and nxt >= self.total_len:
      self.s = self.total_len
    else:
      self.s = nxt


def main(args=None):
  rclpy.init(args=args)
  node = SimOdomPublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

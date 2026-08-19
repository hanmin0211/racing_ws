#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hil_vehicle.py — HIL(하드웨어 인 더 루프) 가상 차량.

차를 공중에 띄운 상태로, 실내에서 **맵을 실제 치수대로 주행**하는 걸 확인한다.

  실제 스택이 만든 /cmd_vel(목표속도+조향각)을 받아 자전거 모델로 위치를 적분해
  /odometry/filtered 로 낸다. 그 위에서 local_path·pure_pursuit·longitudinal 이
  돌아 다시 /cmd_vel 을 만든다 → **폐루프**. serial_bridge 가 같은 /cmd_vel 을
  아두이노로 보내므로, 공중의 바퀴가 맵대로 돌고 조향이 맵의 커브대로 꺾인다.

  GPS·IMU·트랙 없이 제어체인+하드웨어 구동을 통째로 검증한다.
  (단, 무부하라 전원 강하/브라운아웃은 재현 안 됨 — 그건 실차 전원 테스트 몫.)

사용:
  ros2 launch gps_localization hil.launch.py max_speed:=1.0   # 제어체인+맵
  python3 tools/hil_vehicle.py                                # 이 노드(폐루프)

옵션:
  --speed-scale 1.0   실제 아두이노 없이 순수 시뮬만 볼 때 cmd 속도 그대로 사용.
  --start-idx 0       맵의 몇 번째 웨이포인트에서 출발할지.
"""

import argparse
import math

import rclpy
import yaml
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

DEFAULT_WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
              'waypoints_recorded_resampled_0.5.yaml')


def yaw_to_quat(yaw):
  return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class HILVehicle(Node):

  def __init__(self, args):
    super().__init__('hil_vehicle')
    self.L = 0.785
    self.dt = 1.0 / 50.0
    self.wps = self._load(args.wp)
    i0 = min(args.start_idx, len(self.wps) - 2)
    self.x, self.y = self.wps[i0]
    nxt = self.wps[i0 + 1]
    self.yaw = math.atan2(nxt[1] - self.y, nxt[0] - self.x)
    self.v = 0.0
    self.steer = 0.0        # rad
    self.cmd_time = None
    self.trail = []

    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # 먹스가 헤딩 캘리브를 기다리지 않게 오프셋을 미리 발행(HIL 은 odom 직접 제공).
    self.off_pub = self.create_publisher(Float64, '/heading/yaw_offset', latched)
    self.off_pub.publish(Float64(data=0.0))
    self.trail_pub = self.create_publisher(Path, '/sim_trail', latched)
    self.tfb = TransformBroadcaster(self)

    self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
    self.create_timer(self.dt, self.tick)
    self.create_timer(2.0, self.report)
    self.get_logger().info(
        f'HIL 가상차량 시작: 출발 ({self.x:.1f},{self.y:.1f}) '
        f'yaw {math.degrees(self.yaw):.0f}°. /cmd_vel 대기중 — '
        '제어체인(hil.launch.py)과 아두이노를 함께 켤 것.')

  def _load(self, path):
    d = yaml.safe_load(open(path))
    return [(float(p['x']), float(p['y'])) for p in d['waypoints']]

  def cmd_cb(self, msg):
    self.v = float(msg.linear.x)                 # m/s
    self.steer = math.radians(float(msg.angular.z))  # angular.z = 조향각[도]
    self.cmd_time = self.get_clock().now().nanoseconds * 1e-9

  def tick(self):
    # 자전거 모델 적분 (cmd_vel 이 최근에 왔을 때만 움직임)
    now = self.get_clock().now().nanoseconds * 1e-9
    moving = self.cmd_time is not None and (now - self.cmd_time) < 0.5
    if moving:
      self.x += self.v * math.cos(self.yaw) * self.dt
      self.y += self.v * math.sin(self.yaw) * self.dt
      self.yaw += (self.v / self.L) * math.tan(self.steer) * self.dt
      self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    tstamp = self.get_clock().now().to_msg()
    od = Odometry()
    od.header.stamp = tstamp
    od.header.frame_id = 'map'
    od.child_frame_id = 'base_link'
    od.pose.pose.position.x = self.x
    od.pose.pose.position.y = self.y
    od.pose.pose.orientation = yaw_to_quat(self.yaw)
    od.twist.twist.linear.x = self.v if moving else 0.0
    self.odom_pub.publish(od)

    tf = TransformStamped()
    tf.header.stamp = tstamp
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = self.x
    tf.transform.translation.y = self.y
    tf.transform.rotation = yaw_to_quat(self.yaw)
    self.tfb.sendTransform(tf)

    if moving and (not self.trail or
                   math.hypot(self.x - self.trail[-1][0],
                              self.y - self.trail[-1][1]) > 0.2):
      self.trail.append((self.x, self.y))
      self._pub_trail(tstamp)

  def _pub_trail(self, tstamp):
    p = Path()
    p.header.stamp = tstamp
    p.header.frame_id = 'map'
    for (x, y) in self.trail[-2000:]:
      ps = PoseStamped()
      ps.header = p.header
      ps.pose.position.x = x
      ps.pose.position.y = y
      ps.pose.orientation.w = 1.0
      p.poses.append(ps)
    self.trail_pub.publish(p)

  def report(self):
    self.get_logger().info(
        f'HIL pose: x={self.x:.1f} y={self.y:.1f} '
        f'yaw={math.degrees(self.yaw):.0f}° v={self.v:.2f}m/s '
        f'steer={math.degrees(self.steer):+.0f}°',
        throttle_duration_sec=1.9)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--wp', default=DEFAULT_WP)
  ap.add_argument('--start-idx', type=int, default=0)
  args = ap.parse_args()
  rclpy.init()
  n = HILVehicle(args)
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

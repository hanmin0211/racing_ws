#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
direct_localization_node.py
===========================
간단·견고한 직접 로컬라이제이션 (robot_localization/navsat 우회).

RTK GPS가 cm급으로 안정적이면 굳이 EKF 융합을 안 거쳐도 된다. 이 노드는:
  - /fix(NavSatFix, 위경도)를 UTM 52N으로 변환하고 웨이포인트와 동일한 원점을
    빼서 맵 프레임 로컬좌표(x, y)로 만든다. (RTK Fixed면 이 x,y가 cm급)
  - /imu/corrected(헤딩 초기화로 GPS 정렬된 IMU)에서 yaw를 가져온다.
  - 둘을 합쳐 nav_msgs/Odometry(/odometry/filtered, map 프레임)로 발행하고
    map->base_link TF도 broadcast 한다.

출력 프레임/좌표계가 웨이포인트(global_path)와 동일하므로 local_sliding_window_node
가 이 /odometry/filtered 를 그대로 받아 로컬경로를 만든다.

파라미터:
  fix_topic, imu_topic : 입력 (기본 /fix, /imu/corrected)
  output_topic         : 출력 (기본 /odometry/filtered)
  origin_x, origin_y   : UTM52N 원점 (웨이포인트와 동일: 399848.522 / 4092209.171)
  utm_epsg             : 32652 (UTM 52N)
  rate                 : 발행 주기[Hz] (기본 30)
  publish_tf           : map->base_link TF 발행 여부 (기본 True)
"""

import math

import rclpy
import tf_transformations as tft
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


class DirectLocalizationNode(Node):

  def __init__(self):
    super().__init__('direct_localization')

    self.declare_parameter('fix_topic', '/fix')
    # 원본 IMU를 받고, /heading/yaw_offset(heading_init이 계산)를 직접 적용한다.
    self.declare_parameter('imu_topic', 'handsfree/imu')
    self.declare_parameter('output_topic', '/odometry/filtered')
    self.declare_parameter('origin_x', 399848.522)
    self.declare_parameter('origin_y', 4092209.171)
    self.declare_parameter('utm_epsg', 32652)
    self.declare_parameter('rate', 30.0)
    self.declare_parameter('publish_tf', True)
    self.declare_parameter('frame_id', 'map')
    self.declare_parameter('child_frame_id', 'base_link')
    # 헤딩 반전 진단용: A9 yaw가 시계방향(NED)이면 회전방향이 반대로 나온다.
    # 현장서 회전 시 화살표가 반대로 돌면 true로(런치/CLI에서) 뒤집는다.
    # heading_init도 같은 값으로 맞춰야 yaw_offset이 일관됨. 기본 false=무변화.
    self.declare_parameter('invert_imu_yaw', False)

    fix_topic = self.get_parameter('fix_topic').value
    imu_topic = self.get_parameter('imu_topic').value
    output_topic = self.get_parameter('output_topic').value
    self.origin_x = float(self.get_parameter('origin_x').value)
    self.origin_y = float(self.get_parameter('origin_y').value)
    epsg = int(self.get_parameter('utm_epsg').value)
    rate = float(self.get_parameter('rate').value)
    self.publish_tf = bool(self.get_parameter('publish_tf').value)
    self.frame_id = self.get_parameter('frame_id').value
    self.child_frame_id = self.get_parameter('child_frame_id').value
    self.invert_imu_yaw = bool(self.get_parameter('invert_imu_yaw').value)

    self.tf = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}',
                                   always_xy=True)

    self.x = None
    self.y = None
    self.yaw = 0.0
    self.yaw_offset = 0.0        # heading_init이 /heading/yaw_offset로 넣어줌
    self.have_fix = False
    self.have_imu = False
    self.have_offset = False
    # 속도 추정용
    self.prev_x = None
    self.prev_y = None
    self.prev_t = None
    self.vx_body = 0.0

    self.pub = self.create_publisher(Odometry, output_topic, 10)
    self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

    self.create_subscription(NavSatFix, fix_topic, self.fix_cb,
                             qos_profile_sensor_data)
    self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
    # 헤딩 오프셋은 래치(TRANSIENT_LOCAL)로 받는다 — heading_init이 종료돼도 OK.
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Float64, '/heading/yaw_offset', self.offset_cb,
                             latched)
    self.create_timer(1.0 / rate, self.publish_odom)
    self.create_timer(3.0, self.status)

    self.get_logger().info(
        f'직접 로컬라이제이션 시작: {fix_topic}+{imu_topic} → {output_topic} '
        f'(map 프레임, 원점 {self.origin_x:.0f},{self.origin_y:.0f}, {rate:.0f}Hz)')

  def fix_cb(self, msg: NavSatFix):
    if math.isnan(msg.latitude) or abs(msg.latitude) < 1e-9:
      return
    ux, uy = self.tf.transform(msg.longitude, msg.latitude)
    x = ux - self.origin_x
    y = uy - self.origin_y
    t = self.get_clock().now().nanoseconds * 1e-9
    # 진행방향 속도 추정 (위치 변화를 차량 전방으로 투영)
    if self.prev_x is not None and self.prev_t is not None:
      dt = t - self.prev_t
      if dt > 1e-3:
        vx = (x - self.prev_x) / dt
        vy = (y - self.prev_y) / dt
        # 차량 전방(yaw) 성분
        self.vx_body = vx * math.cos(self.yaw) + vy * math.sin(self.yaw)
    self.prev_x, self.prev_y, self.prev_t = x, y, t
    self.x, self.y = x, y
    self.have_fix = True

  def imu_cb(self, msg: Imu):
    raw = yaw_from_quat(msg.orientation)
    if self.invert_imu_yaw:
      raw = -raw
    y = raw + self.yaw_offset
    self.yaw = math.atan2(math.sin(y), math.cos(y))   # 정규화
    self.have_imu = True

  def offset_cb(self, msg: Float64):
    self.yaw_offset = float(msg.data)
    self.have_offset = True
    self.get_logger().info(
        f'헤딩 오프셋 수신: {math.degrees(self.yaw_offset):.1f}° — 헤딩 정렬 적용됨.')

  def publish_odom(self):
    if not self.have_fix:
      return
    now = self.get_clock().now().to_msg()
    q = tft.quaternion_from_euler(0.0, 0.0, self.yaw)

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = self.frame_id
    odom.child_frame_id = self.child_frame_id
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    odom.twist.twist.linear.x = self.vx_body
    self.pub.publish(odom)

    if self.tf_broadcaster is not None:
      tf = TransformStamped()
      tf.header.stamp = now
      tf.header.frame_id = self.frame_id
      tf.child_frame_id = self.child_frame_id
      tf.transform.translation.x = self.x
      tf.transform.translation.y = self.y
      tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
      self.tf_broadcaster.sendTransform(tf)

  def status(self):
    if self.have_fix and self.have_imu:
      cal = '정렬됨' if self.have_offset else '미정렬(10m직진 필요)'
      self.get_logger().info(
          f'pose: x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}° '
          f'v={self.vx_body:.2f}m/s [헤딩 {cal}]')
    else:
      self.get_logger().warn(
          f'대기중 — fix:{"O" if self.have_fix else "X"} '
          f'imu:{"O" if self.have_imu else "X"}')


def main(args=None):
  rclpy.init(args=args)
  node = DirectLocalizationNode()
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

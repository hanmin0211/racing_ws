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
from waypoint_follower.site_origin import declare_and_get

from gps_localization.heading_tracker import HeadingTracker


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
    # ★ 원점은 config/site_origin.yaml 이 정본이다.
    # 이 값이 waypoint_recorder 와 1mm라도 다르면 전역경로가 통째로 평행이동해
    # 차가 엉뚱한 곳을 따라간다. 그래서 같은 로더를 공유한다.
    epsg, self.origin_x, self.origin_y = declare_and_get(self)
    self.declare_parameter('rate', 30.0)
    self.declare_parameter('publish_tf', True)
    self.declare_parameter('frame_id', 'map')
    self.declare_parameter('child_frame_id', 'base_link')
    # 헤딩 반전 진단용: A9 yaw가 시계방향(NED)이면 회전방향이 반대로 나온다.
    # 현장서 회전 시 화살표가 반대로 돌면 true로(런치/CLI에서) 뒤집는다.
    # heading_init도 같은 값으로 맞춰야 yaw_offset이 일관됨. 기본 false=무변화.
    self.declare_parameter('invert_imu_yaw', False)
    # ★ GPS 끊김 감지. 이 시간 이상 /fix가 안 오면 odom 발행을 멈춘다.
    # 예전엔 fix가 끊겨도 마지막 위치를 30Hz로 계속 발행해서, 차는 움직이는데
    # 위치는 고정된 채 로컬 경로가 계속 생성됐다(하류 타임아웃도 안 걸림).
    # 발행을 멈추면 local_sliding_window/pure_pursuit가 타임아웃으로 안전 정지한다.
    self.declare_parameter('fix_timeout', 1.0)
    # ★ IMU 끊김 감지. GPS와 같은 이유로 필수다.
    # 2026-08-15 현장에서 A9가 주행 중 송신을 멈췄는데(노드 프로세스는 살아있음)
    # yaw가 마지막 값(+34.1°)에 얼어붙은 채로 계속 발행됐다. 차량은 175° 넘게
    # 회전했는데 헤딩은 그대로라 course와의 오차가 180°까지 벌어졌다.
    # 이 상태로 자율주행하면 경로를 완전히 이탈한다 → 끊기면 발행을 멈춘다.
    self.declare_parameter('imu_timeout', 0.5)
    # ★ 헤딩 연속 보정 (2026-10-12) — 기본 false = 예전 동작(캘리브 1회 오프셋 고정).
    #   저가 IMU yaw 가 떠내려간다(10/12 −21°/분 → 랩 끝 −28°, 차가 경로 오른쪽
    #   0.5~1.5 m 로 달림). 직진·주행·RTK FIXED 표본에서 GPS 진행방향으로 오프셋을
    #   tau 초 시정수로 계속 당긴다. 원리·거르는 조건은 heading_tracker.py.
    #   bag 재생(tools/heading_track_replay.py): 직선 헤딩 오차 RMS 15.0° → 2.2°.
    #   ⚠ false 여도 추정은 계속 돌려 /heading/yaw_offset_live 로 낸다(기록·비교용).
    #     적용만 안 한다.
    self.declare_parameter('heading_track', False)
    self.declare_parameter('heading_track_tau', 3.0)

    fix_topic = self.get_parameter('fix_topic').value
    imu_topic = self.get_parameter('imu_topic').value
    output_topic = self.get_parameter('output_topic').value
    rate = float(self.get_parameter('rate').value)
    self.publish_tf = bool(self.get_parameter('publish_tf').value)
    self.frame_id = self.get_parameter('frame_id').value
    self.child_frame_id = self.get_parameter('child_frame_id').value
    self.invert_imu_yaw = bool(self.get_parameter('invert_imu_yaw').value)
    self.fix_timeout = float(self.get_parameter('fix_timeout').value)
    self.last_fix_time = None
    self.fix_lost = False
    self.imu_timeout = float(self.get_parameter('imu_timeout').value)
    self.heading_track = bool(self.get_parameter('heading_track').value)
    self.tracker = HeadingTracker(
        tau=float(self.get_parameter('heading_track_tau').value))
    self._trk_off0 = None          # 캘리브 오프셋 (보정량을 보고하려고)
    self.last_imu_time = None
    self.imu_lost = False

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
    self.live_pub = self.create_publisher(Float64, '/heading/yaw_offset_live', 10)
    self.create_timer(1.0 / rate, self.publish_odom)
    self.create_timer(3.0, self.status)

    self.get_logger().info(
        f'직접 로컬라이제이션 시작: {fix_topic}+{imu_topic} → {output_topic} '
        f'(map 프레임, 원점 {self.origin_x:.0f},{self.origin_y:.0f}, {rate:.0f}Hz)')
    if self.heading_track:
      self.get_logger().warn(
          f'★ 헤딩 연속 보정 켜짐 — 직진·주행 중 GPS 진행방향으로 IMU 오프셋을 '
          f'계속 고친다 (tau {self.tracker.tau:.1f}s)')

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
    self.last_fix_time = t
    # 헤딩 연속 보정 — 캘리브 전에는 tracker.offset 이 None 이라 아무것도 안 한다
    sigma = None
    if msg.position_covariance_type != 0:
      sigma = math.sqrt(max(0.0, msg.position_covariance[0]
                            + msg.position_covariance[4]))
    if self.tracker.add_fix(t, x, y, sigma) is not None:
      self.live_pub.publish(Float64(data=float(self.tracker.offset)))
      if self.heading_track:
        self.yaw_offset = self.tracker.offset
    if self.fix_lost:
      self.get_logger().info('GPS 복구 — 위치 발행 재개')
      self.fix_lost = False

  def imu_cb(self, msg: Imu):
    self.last_imu_time = self.get_clock().now().nanoseconds * 1e-9
    if self.imu_lost:
      self.get_logger().info('IMU 복구 — 위치 발행 재개')
      self.imu_lost = False
    raw = yaw_from_quat(msg.orientation)
    if self.invert_imu_yaw:
      raw = -raw
    self.tracker.add_imu(self.last_imu_time, raw)
    y = raw + self.yaw_offset
    self.yaw = math.atan2(math.sin(y), math.cos(y))   # 정규화
    self.have_imu = True

  def offset_cb(self, msg: Float64):
    self.yaw_offset = float(msg.data)
    self.have_offset = True
    self.tracker.set_offset(self.yaw_offset)       # 재캘리브면 추정도 다시 시작
    self._trk_off0 = self.yaw_offset
    self.get_logger().info(
        f'헤딩 오프셋 수신: {math.degrees(self.yaw_offset):.1f}° — 헤딩 정렬 적용됨.')

  def publish_odom(self):
    if not self.have_fix:
      return
    # ★ GPS 끊김: 오래된 위치를 계속 내보내면 차는 움직이는데 위치는 고정되어
    # 하류가 엉뚱한 경로를 따라간다. 발행을 멈춰 하류가 타임아웃 정지하게 한다.
    t_now = self.get_clock().now().nanoseconds * 1e-9
    if self.last_fix_time is not None and \
            (t_now - self.last_fix_time) > self.fix_timeout:
      if not self.fix_lost:
        self.get_logger().error(
            f'GPS 끊김 ({t_now - self.last_fix_time:.1f}s) → 위치 발행 중단 '
            f'(하류가 타임아웃으로 정지함)')
        self.fix_lost = True
      return
    # IMU 끊김: yaw가 얼어붙은 채 발행하면 회전을 전혀 반영하지 못한다.
    if self.have_imu and self.last_imu_time is not None and \
            (t_now - self.last_imu_time) > self.imu_timeout:
      if not self.imu_lost:
        self.get_logger().error(
            f'IMU 끊김 ({t_now - self.last_imu_time:.1f}s) → 위치 발행 중단. '
            f'헤딩이 고정되어 경로를 이탈하므로 정지시킨다. '
            f'IMU USB 재연결 후 heading_init 재실행 필요.')
        self.imu_lost = True
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
      if self.have_offset and self.tracker.offset is not None:
        d = math.degrees(math.atan2(math.sin(self.tracker.offset - self._trk_off0),
                                    math.cos(self.tracker.offset - self._trk_off0)))
        c = self.tracker.counts
        cal += (f' · 연속보정 {"적용" if self.heading_track else "추정만"} {d:+.1f}° '
                f'(갱신 {c["ok"]} · 커브 {c["turn"]} · 느림 {c["slow"]} · RTK {c["rtk"]} '
                f'· 이상 {c["outlier"]})')
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
heading_init_node.py
====================
10m 직진 GPS-course 헤딩(yaw) 초기화 — 1회성(one-shot) 노드.

A9 IMU의 yaw는 자력계 오차로 맵 좌표계에 정렬돼 있지 않다. 차량이 직진할 때
GPS 이동방향(course)을 '진짜 헤딩'으로 삼아, 그 순간 IMU yaw와의 차이
(yaw_offset)를 계산한다.

  1. 첫 유효 GPS를 시작점으로 기록
  2. 직진 → 시작점에서 calib_distance(기본 10m) 도달 시:
        course     = atan2(Δnorth, Δeast)
        yaw_offset = normalize(course - imu_yaw)
  3. /heading/yaw_offset 을 래치(TRANSIENT_LOCAL)로 발행한 뒤 노드 자동 종료.

오프셋을 실제로 IMU에 적용하는 건 direct_localization_node가 한다. 그래서 이
노드는 값만 계산하고 빠져도 로컬라이제이션이 계속 돌아간다. 재캘리브가
필요하면 이 노드를 다시 실행하면 된다(이미 떠 있는 구독자가 새 값을 받음).

파라미터:
  fix_topic, imu_topic : 입력 (기본 /fix, handsfree/imu)
  calib_distance       : 직진 거리[m] (기본 10.0)
  restart_settle_sec   : 직진성 검증 실패 후 재시작 전 정지 대기[s] (기본 3.0)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_sensor_data)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64

M_PER_DEG = 111320.0


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


def normalize_angle(a):
  return math.atan2(math.sin(a), math.cos(a))


class HeadingInitNode(Node):

  def __init__(self):
    super().__init__('gps_heading_init')

    self.declare_parameter('fix_topic', '/fix')
    self.declare_parameter('imu_topic', 'handsfree/imu')
    self.declare_parameter('calib_distance', 10.0)
    # direct_localization의 invert_imu_yaw와 반드시 같은 값이어야 yaw_offset이 일관됨.
    self.declare_parameter('invert_imu_yaw', False)
    # 캘리브 중 허용할 최대 진행방향 편차[도]. 이보다 휘면 직진이 아니라고 보고 거부.
    self.declare_parameter('max_deviation_deg', 20.0)
    # 실패 후 재시작: 차량이 이 시간만큼 멈춰 있어야 새 시작점을 잡는다.
    self.declare_parameter('restart_settle_sec', 3.0)
    self.declare_parameter('restart_settle_radius', 0.5)   # 이보다 움직이면 '이동 중'
    self.declare_parameter('restart_settle_timeout', 20.0)  # 정지 감지 실패 시 탈출

    fix_topic = self.get_parameter('fix_topic').value
    imu_topic = self.get_parameter('imu_topic').value
    self.calib_distance = float(self.get_parameter('calib_distance').value)
    self.invert_imu_yaw = bool(self.get_parameter('invert_imu_yaw').value)
    self.max_deviation = math.radians(
        float(self.get_parameter('max_deviation_deg').value))
    self.settle_sec = float(self.get_parameter('restart_settle_sec').value)
    self.settle_radius = float(self.get_parameter('restart_settle_radius').value)
    self.settle_timeout = float(self.get_parameter('restart_settle_timeout').value)

    # 실패 후 재무장 대기 상태 (최초 1회차에는 적용하지 않는다 — 런치 직후엔
    # 차가 서 있는 게 정상이고, 괜히 3초를 더 기다리게 만들 이유가 없다)
    self.rearming = False
    self._settle_lat = None
    self._settle_lon = None
    self._settle_t = 0.0
    self._rearm_t0 = 0.0
    self._last_settle_log = 0.0

    self.lat0 = None
    self.lon0 = None
    self.cos_lat0 = 1.0
    self.imu_yaw = None
    self.done_time = None
    self.prev_e = None
    self.prev_n = None
    self.max_dev = 0.0

    # 오프셋은 래치(TRANSIENT_LOCAL)로 발행 — 이 노드가 종료해도 이미 구독 중인
    # direct_localization이 값을 받도록. (같은 이유로 course도 래치)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.off_pub = self.create_publisher(Float64, '/heading/yaw_offset', latched)
    self.course_pub = self.create_publisher(Float64, '/heading/gps_course',
                                            latched)

    self.create_subscription(NavSatFix, fix_topic, self.fix_cb,
                             qos_profile_sensor_data)
    self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
    self.create_timer(0.3, self.shutdown_check)

    self.get_logger().info(
        f'헤딩 초기화(1회성) 시작: {self.calib_distance:.0f}m 직진하면 '
        f'yaw_offset 계산 후 자동 종료 (fix={fix_topic}, imu={imu_topic})')

  def imu_cb(self, msg: Imu):
    y = yaw_from_quat(msg.orientation)
    self.imu_yaw = -y if self.invert_imu_yaw else y

  def _settled(self, msg: NavSatFix) -> bool:
    """실패 후 재시작: 차량이 실제로 멈출 때까지 시작점 기록을 미룬다.

    실패 직후 곧바로 시작점을 잡으면, 운전자가 차를 되돌리는 그 동작이 다음
    시도의 앞구간으로 기록돼 또 '직진 아님'으로 거부된다. 현장에서 실제로
    이것 때문에 2회 연속 실패했다(1차 실패 110ms 뒤에 2차 시작점이 잡힘).
    """
    t = self.get_clock().now().nanoseconds * 1e-9
    if self._settle_lat is None:
      self._settle_lat, self._settle_lon = msg.latitude, msg.longitude
      self._settle_t = self._rearm_t0 = t
      return False

    cos_lat = math.cos(math.radians(self._settle_lat))
    de = (msg.longitude - self._settle_lon) * M_PER_DEG * cos_lat
    dn = (msg.latitude - self._settle_lat) * M_PER_DEG
    if math.hypot(de, dn) > self.settle_radius:
      # 아직 움직이는 중 — 기준점을 현재로 옮기고 정지 타이머를 리셋
      self._settle_lat, self._settle_lon = msg.latitude, msg.longitude
      self._settle_t = t

    still = t - self._settle_t
    if still >= self.settle_sec:
      return True

    # RTK가 나빠 위치가 계속 튀면 영원히 '정지'로 안 잡힌다. 그 경우 노드가
    # 조용히 멎어버리는 게 원래 버그보다 나쁘므로 탈출구를 둔다.
    if t - self._rearm_t0 >= self.settle_timeout:
      self.get_logger().warn(
          f'정지 감지 실패 ({self.settle_timeout:.0f}s 경과 — GPS 튐 가능). '
          f'그대로 시작점을 잡는다. 차량이 멈춰 있는지 눈으로 확인할 것.')
      return True

    if t - self._last_settle_log >= 2.0:
      self._last_settle_log = t
      self.get_logger().info(
          f'재시작 대기: 차량을 세우고 기다리세요 '
          f'({still:.0f}/{self.settle_sec:.0f}s)')
    return False

  def fix_cb(self, msg: NavSatFix):
    if self.done_time is not None:
      return
    if math.isnan(msg.latitude) or abs(msg.latitude) < 1e-9:
      return
    if self.lat0 is None:
      if self.rearming and not self._settled(msg):
        return
      self.rearming = False
      self._settle_lat = None
      self.lat0, self.lon0 = msg.latitude, msg.longitude
      self.cos_lat0 = math.cos(math.radians(self.lat0))
      self.prev_e = self.prev_n = None
      self.max_dev = 0.0
      self.get_logger().info(
          f'시작점 기록: ({self.lat0:.7f}, {self.lon0:.7f}). 직진 시작하세요.')
      return

    east = (msg.longitude - self.lon0) * M_PER_DEG * self.cos_lat0
    north = (msg.latitude - self.lat0) * M_PER_DEG
    dist = math.hypot(east, north)

    # ★ 직진성 검증: 시작점→현재점의 '직선 방향'을 헤딩으로 쓰기 때문에,
    # 캘리브 중 곡선으로 가거나 후진하면 그 직선이 실제 진행방향과 달라져
    # yaw_offset이 통째로 틀어진다(전 구간 경로 이탈로 이어짐).
    # 최근 구간의 진행방향과 전체 직선방향이 크게 다르면 캘리브를 거부한다.
    if self.prev_e is not None:
      seg_e, seg_n = east - self.prev_e, north - self.prev_n
      if math.hypot(seg_e, seg_n) > 0.3:      # 유의미하게 움직였을 때만 평가
        seg_course = math.atan2(seg_n, seg_e)
        chord_course = math.atan2(north, east)
        dev = abs(normalize_angle(seg_course - chord_course))
        self.max_dev = max(getattr(self, 'max_dev', 0.0), dev)
        self.prev_e, self.prev_n = east, north
    else:
      self.prev_e, self.prev_n = east, north

    if dist >= self.calib_distance:
      if self.imu_yaw is None:
        self.get_logger().warn('IMU yaw 미수신 — 헤딩 계산 보류.')
        return
      max_dev = getattr(self, 'max_dev', 0.0)
      if max_dev > self.max_deviation:
        self.get_logger().error(
            f'❌ 직진이 아닙니다 (최대 편차 {math.degrees(max_dev):.0f}° > '
            f'{math.degrees(self.max_deviation):.0f}°). 헤딩 캘리브 무효 — '
            f'차량을 되돌려 **곧게** 다시 {self.calib_distance:.0f}m 직진하세요.')
        # 처음부터 다시. 단, 차를 되돌리는 동안은 시작점을 잡지 않는다(_settled).
        self.lat0 = None
        self.prev_e = self.prev_n = None
        self.max_dev = 0.0
        self._last_log_m = -1
        self.rearming = True
        self._settle_lat = None
        self._last_settle_log = 0.0
        return
      course = math.atan2(north, east)
      yaw_offset = normalize_angle(course - self.imu_yaw)
      self.get_logger().info(
          f'✅ 헤딩 초기화 완료: {dist:.1f}m 직진. '
          f'GPS course={math.degrees(course):.1f}°, '
          f'IMU yaw={math.degrees(self.imu_yaw):.1f}°, '
          f'→ yaw_offset={math.degrees(yaw_offset):.1f}°')
      # 래치 발행 (구독자에게 전파될 시간을 준 뒤 자동 종료)
      self.off_pub.publish(Float64(data=float(yaw_offset)))
      self.course_pub.publish(Float64(data=float(course)))
      self.done_time = self.get_clock().now().nanoseconds * 1e-9
    else:
      if int(dist) != getattr(self, '_last_log_m', -1):
        self._last_log_m = int(dist)
        self.get_logger().info(f'직진 중... {dist:.1f}/{self.calib_distance:.0f}m')

  def shutdown_check(self):
    # 오프셋 발행 후 1초 지나면 자동 종료 (래치 전파 시간 확보)
    if self.done_time is not None:
      if self.get_clock().now().nanoseconds * 1e-9 - self.done_time > 1.0:
        self.get_logger().info('yaw_offset 발행 완료 → 노드 자동 종료.')
        rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)
  node = HeadingInitNode()
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

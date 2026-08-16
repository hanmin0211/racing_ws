#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""straight_drive_node.py — 차량이 스스로 지정 거리만큼 직진하고 정지.

헤딩 캘리브(10m), 엔코더 캘리브(15~20m), FF 식별 구간 확보(30m) 등에서
사람이 밀거나 teleop 키를 잡고 있을 필요 없이 차가 직접 달린다.

★ 거리는 **RTK 위치로 잰다**(시간×속도가 아니라). 지금 차량은 명령보다 느리게
  나가는 상태(0.25 명령 → 실측 0.157m/s)라 시간 기준으로 하면 거리가 크게 어긋난다.

★ 직진 유지: 출발 시점의 헤딩을 기억하고 그 방향을 유지하도록 조향을 소폭 보정한다.
  보정각은 max_correction(기본 5°)으로 제한 — 폭주 방지.

안전:
  · 목표 거리 도달 → 감속 정지
  · timeout 초과 → 정지 (스톨·미끄러짐 대비)
  · /odometry/filtered 끊기면 즉시 정지
  · Ctrl-C → 정지 명령 발행 후 종료 (펌웨어 워치독도 0.5초 내 정지)

사용 (serial_bridge 실행 중이어야 함):
  ros2 run velocity_controller straight_drive                      # 기본 30m, 0.4m/s
  ros2 run velocity_controller straight_drive --ros-args -p distance:=10.0
  ros2 run velocity_controller straight_drive --ros-args -p distance:=30.0 -p speed:=0.5

  # bringup control:=true 로 먹스가 떠 있으면 teleop 경로로 보내 E-stop 우선권을 살린다
  ros2 run velocity_controller straight_drive --ros-args -p cmd_topic:=/teleop/cmd_vel
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def norm(a):
  return math.atan2(math.sin(a), math.cos(a))


class StraightDrive(Node):

  def __init__(self):
    super().__init__('straight_drive')
    self.declare_parameter('distance', 30.0)        # 목표 이동거리 [m]
    self.declare_parameter('speed', 0.4)            # 목표 속도 [m/s]
    self.declare_parameter('cmd_topic', '/cmd_vel')
    self.declare_parameter('heading_gain', 1.5)     # 도/도 — 헤딩 오차→조향각
    self.declare_parameter('max_correction', 5.0)   # 보정 조향각 상한 [도]
    self.declare_parameter('accel_time', 1.5)       # 출발 램프 [s]
    self.declare_parameter('timeout_margin', 3.0)   # 예상시간 대비 여유 배수

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.target = float(g('distance'))
    self.speed = float(g('speed'))
    self.gain = float(g('heading_gain'))
    self.max_corr = float(g('max_correction'))
    self.accel_time = float(g('accel_time'))
    self.timeout = self.target / max(0.05, self.speed) * float(g('timeout_margin')) + 15.0

    self.pub = self.create_publisher(Twist, str(g('cmd_topic')), 10)
    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)

    self.x0 = self.y0 = self.yaw0 = None
    self.x = self.y = self.yaw = None
    self.last_odom = None
    self.dist = 0.0
    self.t0 = None
    self.done = False
    self.dt = 0.05
    self.create_timer(self.dt, self.tick)
    self.create_timer(1.0, self.report)

    self.get_logger().info(
        f'직진 주행 준비: 목표 {self.target:.1f}m, 속도 {self.speed:.2f}m/s, '
        f'헤딩유지 ±{self.max_corr:.0f}°, 타임아웃 {self.timeout:.0f}s → {g("cmd_topic")}')
    self.get_logger().warn('⚠ 차량이 스스로 출발한다. 앞을 비우고 Ctrl-C 준비.')

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def odom_cb(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
    self.last_odom = self.now()
    if self.x0 is None:
      self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw
      self.t0 = self.now()
      self.get_logger().info(
          f'출발점 ({self.x0:.2f}, {self.y0:.2f}), 유지할 헤딩 '
          f'{math.degrees(self.yaw0):.1f}° — 주행 시작')
    else:
      self.dist = math.hypot(self.x - self.x0, self.y - self.y0)

  def stop(self, reason):
    if not self.done:
      self.get_logger().info(f'정지: {reason} (이동 {self.dist:.2f}m)')
      self.done = True
    self.pub.publish(Twist())

  def tick(self):
    if self.done:
      self.pub.publish(Twist())
      return
    if self.x0 is None:
      self.get_logger().info('대기중 — /odometry/filtered 필요 (헤딩 정렬 후 실행)',
                             throttle_duration_sec=3.0)
      return
    # 안전: odom 끊기면 정지
    if self.last_odom is None or (self.now() - self.last_odom) > 0.5:
      self.stop('위치 정보 끊김')
      return
    el = self.now() - self.t0
    if el > self.timeout:
      self.stop(f'타임아웃 {self.timeout:.0f}s')
      return
    if self.dist >= self.target:
      self.stop(f'목표 {self.target:.1f}m 도달')
      return

    # 출발 램프 (급가속 방지)
    v = self.speed * min(1.0, el / max(0.1, self.accel_time))
    # 도착 전 감속
    remain = self.target - self.dist
    if remain < 1.0:
      v *= max(0.3, remain / 1.0)

    # 헤딩 유지 보정 (출발 헤딩 대비 오차를 조향으로 되돌림)
    err = math.degrees(norm(self.yaw0 - self.yaw))
    corr = max(-self.max_corr, min(self.max_corr, self.gain * err))

    cmd = Twist()
    cmd.linear.x = float(v)
    cmd.angular.z = float(corr)
    self.pub.publish(cmd)

  def report(self):
    if self.x0 is None or self.done:
      return
    err = math.degrees(norm(self.yaw0 - self.yaw)) if self.yaw is not None else 0.0
    self.get_logger().info(
        f'{self.dist:5.2f} / {self.target:.1f} m  |  헤딩오차 {err:+5.1f}°')


def main(args=None):
  rclpy.init(args=args)
  n = StraightDrive()
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      for _ in range(5):
        n.pub.publish(Twist())
    except Exception:  # noqa: BLE001
      pass
    print(f'\n최종 이동거리: {n.dist:.2f} m')
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

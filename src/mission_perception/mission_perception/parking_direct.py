#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parking_direct.py — 필승용 단순 후진주차.

기존 parking_node 는 전진→후진→정차 궤적 전체를 정확히 재현한다.
장점: 오차 14cm. 단점: 시작점 자세가 맞아야 하고 실패 지점이 많다.

이 노드는 대회 규정("뒷바퀴 걸침만 인정")에 맞춰 극단적으로 단순화:
  - 어디에 있든 GPS 목표점을 향해 저속 후진.
  - 조향은 후진 pursuit (rear-axle 기준 목표점 좌표).
  - GPS 안테나가 목표점 goal_tol 이내면 정지.
  - 뒷바퀴는 안테나 뒤에 있으므로, 안테나가 목표에 도달하면 뒷바퀴는 이미 자리 안.

파라미터:
  slot          : 자리 번호. parking_<slot>.yaml 마지막 점을 목표로 로드.
  target_x/y    : 직접 지정. slot 로드보다 우선.
  speed         : 후진 속도 크기 [m/s] (기본 0.2)
  goal_tol      : 도달 판정 [m] (기본 0.6 — 뒷걸침 규정에 여유)
  timeout       : 자동 중단 [s] (기본 40)
  steer_sign    : 조향 부호. 실차에서 반대로 가면 -1.0 (기본 +1.0)
  auto_start    : 즉시 시작
  trigger_on_goal_reached : /goal_reached True 를 트리거로 (기본 True)

트리거: /parking/start (Bool) 또는 /goal_reached (Bool)
출력  : /teleop/cmd_vel (mux 우선순위로 자율을 덮어씀), /parking/state (String)
"""

import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ParkingDirect(Node):

    def __init__(self):
        super().__init__('parking_direct')
        self.declare_parameter('slot', 1)
        self.declare_parameter('parking_dir', os.path.expanduser('~'))
        self.declare_parameter('target_x', 9999.0)
        self.declare_parameter('target_y', 9999.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('goal_tol', 0.6)
        self.declare_parameter('timeout', 40.0)
        self.declare_parameter('wheelbase', 0.785)
        self.declare_parameter('max_steer_deg', 18.0)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('trigger_on_goal_reached', True)
        self.declare_parameter('rate', 20.0)

        g = lambda k: self.get_parameter(k).value
        self.slot = int(g('slot'))
        self.parking_dir = str(g('parking_dir'))
        self.speed = abs(float(g('speed')))
        self.goal_tol = float(g('goal_tol'))
        self.timeout = float(g('timeout'))
        self.L = float(g('wheelbase'))
        self.max_steer = float(g('max_steer_deg'))
        self.steer_sign = float(g('steer_sign'))
        rate = float(g('rate'))

        tx, ty = float(g('target_x')), float(g('target_y'))
        if tx < 9000 and ty < 9000:
            self.target = (tx, ty)
            self.get_logger().info(f'목표점 파라미터 지정: {self.target}')
        else:
            self.target = self.load_target(self.slot)

        self.state = 'WAIT'
        self.pose = None
        self.t_start = None
        self.d_initial = None
        self.d_best = None
        self.stall_count = 0

        self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/parking/state', 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.odom_cb, 10)
        self.create_subscription(Bool, '/parking/start', self.start_cb, 10)
        self.create_subscription(Int32, '/parking/select', self.select_cb, 10)
        if bool(g('trigger_on_goal_reached')):
            self.create_subscription(Bool, '/goal_reached', self.start_cb, 10)
            self.get_logger().info('완주 신호(/goal_reached)도 트리거로 사용.')

        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'★ 직접 후진주차 노드: 자리 {self.slot}, '
            f'목표 ({self.target[0]:.2f},{self.target[1]:.2f}), '
            f'속도 -{self.speed} m/s, 도달 {self.goal_tol} m, '
            f'steer_sign {self.steer_sign:+.1f}')
        if bool(g('auto_start')):
            self.begin()
        else:
            self.get_logger().info(
                '대기 — /parking/start 또는 /goal_reached True 로 시작.')

    def load_target(self, slot):
        path = os.path.join(self.parking_dir, f'parking_{slot}.yaml')
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
            last = d['points'][-1]
            t = (float(last['x']), float(last['y']))
            self.get_logger().info(f'{path} 로드: 목표 {t}')
            return t
        except Exception as e:
            self.get_logger().error(f'{path} 로드 실패: {e} — 목표 없음')
            return (9999.0, 9999.0)

    def select_cb(self, msg):
        new = int(msg.data)
        if new != self.slot:
            self.slot = new
            self.target = self.load_target(self.slot)

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     yaw_from_quat(p.orientation))

    def start_cb(self, msg):
        if not bool(msg.data):
            return
        if self.state == 'DRIVE':
            self.get_logger().warn('트리거 무시 — 이미 DRIVE.')
            return
        self.get_logger().info('★ 직접 주차 트리거 수신')
        self.begin()

    def begin(self):
        if self.pose is None:
            self.get_logger().error('측위 미수신 — 시작 못함.')
            return
        if self.target[0] > 9000:
            self.get_logger().error('목표점 없음 — 시작 못함.')
            return
        x, y, _ = self.pose
        d0 = math.hypot(self.target[0] - x, self.target[1] - y)
        self.d_initial = d0
        self.d_best = d0
        self.t_start = time.time()
        self.stall_count = 0
        self.state = 'DRIVE'
        self.get_logger().info(
            f'▶ 시작: 현재 ({x:.2f},{y:.2f}) → 목표 {self.target}, 거리 {d0:.2f} m')

    def tick(self):
        self.state_pub.publish(String(data=self.state))
        if self.state == 'WAIT':
            return
        if self.state in ('DONE', 'ABORT'):
            self.pub.publish(Twist())
            return

        if self.pose is None:
            self.pub.publish(Twist())
            return

        if time.time() - self.t_start > self.timeout:
            self.get_logger().error(
                f'타임아웃 {self.timeout} s — ABORT (초기 {self.d_initial:.2f}m → '
                f'최근접 {self.d_best:.2f}m)')
            self.state = 'ABORT'
            self.pub.publish(Twist())
            return

        x, y, yaw = self.pose
        tx, ty = self.target
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < self.d_best:
            self.d_best = d

        if d < self.goal_tol:
            self.get_logger().info(
                f'🎯 목표 도달 — 오차 {d:.2f} m (규정 뒷걸침 OK). 완료.')
            self.state = 'DONE'
            self.pub.publish(Twist())
            return

        # 발산 감지: 최근접 대비 크게 멀어지면 조향 부호가 반대일 확률 큼
        if self.d_best < self.d_initial and d > self.d_best + 1.5:
            self.get_logger().error(
                f'★ 발산 감지 (최근접 {self.d_best:.2f} → 현재 {d:.2f} m) — ABORT. '
                'steer_sign 을 반대로 두고 재시도하라.')
            self.state = 'ABORT'
            self.pub.publish(Twist())
            return

        # 목표점을 rear-axle 좌표계로
        c, s = math.cos(yaw), math.sin(yaw)
        x_r = dx * c + dy * s      # +면 앞, −면 뒤
        y_r = -dx * s + dy * c     # +면 왼쪽, −면 오른쪽

        # 후진 pursuit
        ld = max(d, 0.5)
        kappa = 2.0 * y_r / (ld * ld)
        delta = math.degrees(math.atan(self.L * kappa)) * self.steer_sign
        delta = max(-self.max_steer, min(self.max_steer, delta))

        # 후진 속도. 가까워지면 감속.
        v = -self.speed
        if d < 1.5:
            v = -max(0.10, self.speed * (d / 1.5))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = delta
        self.pub.publish(cmd)

        # 0.5초마다 진단 로그
        now_ms = int(time.time() * 2)
        if now_ms != getattr(self, '_last_log', -1):
            self._last_log = now_ms
            side = '(목표 뒤)' if x_r < 0 else '(목표 앞!)'
            self.get_logger().info(
                f'DRIVE d={d:.2f}m  y_r={y_r:+.2f}  δ={delta:+.1f}°  '
                f'v={v:+.2f}  {side}')


def main():
    rclpy.init()
    node = ParkingDirect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""encoder_calib_node.py — 엔코더 스케일(counts_per_revolution) 검증.

차량을 직진시키면서 **RTK가 잰 실제 이동거리**와 **엔코더가 센 카운트**를 비교해
1카운트당 실제 거리를 역산한다. 지금 펌웨어의 counts_per_revolution=-290,
wheel_radius=0.13 은 한 번도 검증된 적이 없고, 이게 틀리면 그 위의 모든 속도
제어(FF·PID·목표속도)가 통째로 어긋난다. 그래서 구동 튜닝의 **첫 단계**다.

  RTK 거리   = √(Δx² + Δy²)          (/odometry/filtered, 1.4cm 정확도)
  엔코더 거리 = Δcounts / CPR × 원주   (현재 설정 기준)
  보정계수   = RTK 거리 / 엔코더 거리
  → 새 CPR  = 현재 CPR × 보정계수 ... 를 자동 출력

※ 타이어가 눌리면 유효 반경이 줄어 줄자 실측과 다르다. RTK 비교라야 실제값이 나온다.

사용:
  ros2 run velocity_controller encoder_calib
  (그 상태에서 차량을 **직진**으로 15~20m 주행 — teleop이든 수동으로 밀든 무관)
  Ctrl-C 하면 결과 요약 출력. 진행 중에도 3초마다 중간 결과를 보여준다.

파라미터:
  min_distance : 이 거리 이상 이동해야 결과를 신뢰 (기본 10.0 m)
  straight_tol : 직진 판정 허용 편차 [m] (기본 1.0). 곡선 주행이면 경고.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32

# 펌웨어 현재 설정 (henes_firmware.ino 와 일치시킬 것)
FW_COUNTS_PER_REV = 290.0
FW_WHEEL_RADIUS = 0.13
FW_CIRCUMFERENCE = 2.0 * math.pi * FW_WHEEL_RADIUS


class EncoderCalib(Node):

  def __init__(self):
    super().__init__('encoder_calib')
    self.declare_parameter('min_distance', 10.0)
    self.declare_parameter('straight_tol', 1.0)
    self.min_dist = float(self.get_parameter('min_distance').value)
    self.straight_tol = float(self.get_parameter('straight_tol').value)

    self.x0 = self.y0 = None
    self.x = self.y = None
    self.enc0 = None
    self.enc = None
    self.path_len = 0.0          # 실제 주행 궤적 길이(누적)
    self.prev_x = self.prev_y = None

    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
    self.create_subscription(Int32, '/encoder_count', self.enc_cb, 10)
    self.create_timer(3.0, self.report)

    self.get_logger().info(
        '엔코더 스케일 캘리브 시작 — 차량을 **직진**으로 '
        f'{self.min_dist:.0f}m 이상 주행시키세요. 3초마다 중간 결과 표시.')

  def odom_cb(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    if self.x0 is None:
      self.x0, self.y0 = self.x, self.y
      self.prev_x, self.prev_y = self.x, self.y
      return
    d = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
    if d > 0.01:                 # 지터 무시
      self.path_len += d
      self.prev_x, self.prev_y = self.x, self.y

  def enc_cb(self, msg):
    self.enc = int(msg.data)
    if self.enc0 is None:
      self.enc0 = self.enc

  def result(self):
    if self.x0 is None or self.enc0 is None or self.x is None:
      return None
    straight = math.hypot(self.x - self.x0, self.y - self.y0)
    d_enc = abs(self.enc - self.enc0)
    if d_enc == 0:
      return None
    enc_dist = d_enc / FW_COUNTS_PER_REV * FW_CIRCUMFERENCE
    return {
        'straight': straight,
        'path': self.path_len,
        'counts': d_enc,
        'enc_dist': enc_dist,
        'factor': straight / enc_dist if enc_dist > 1e-6 else 0.0,
        'curve_dev': self.path_len - straight,
    }

  def report(self):
    # '데이터 없음'과 '아직 안 움직임'을 구분해서 알려준다.
    # (예전엔 둘 다 '수신 필요'로 떠서 현장에서 원인 판단이 어려웠다)
    missing = []
    if self.x is None:
      missing.append('/odometry/filtered')
    if self.enc is None:
      missing.append('/encoder_count')
    if missing:
      self.get_logger().info(f'대기중 — 수신 없음: {", ".join(missing)}')
      return
    r = self.result()
    if r is None:
      d = 0 if self.enc0 is None else abs(self.enc - self.enc0)
      self.get_logger().info(
          f'토픽 정상 수신 중 — 차량을 직진시키세요 (엔코더 변화 {d}counts)')
      return
    self.get_logger().info(
        f'RTK {r["straight"]:.2f}m | 엔코더 {r["enc_dist"]:.2f}m '
        f'({r["counts"]}counts) | 보정계수 {r["factor"]:.4f}')

  def summary(self):
    r = self.result()
    print('\n' + '=' * 60)
    if r is None:
      print('데이터 부족 — 결과 없음')
      print('=' * 60)
      return
    print('엔코더 스케일 캘리브 결과')
    print('-' * 60)
    print(f'  RTK 직선거리   : {r["straight"]:.3f} m')
    print(f'  실제 궤적길이  : {r["path"]:.3f} m  (직선 대비 +{r["curve_dev"]:.2f}m)')
    print(f'  엔코더 카운트  : {r["counts"]}')
    print(f'  엔코더 환산거리: {r["enc_dist"]:.3f} m  (현재 CPR={FW_COUNTS_PER_REV:.0f})')
    print('-' * 60)
    if r['straight'] < self.min_dist:
      print(f'  ⚠ 이동거리가 {self.min_dist:.0f}m 미만 — 신뢰도 낮음. 더 길게 재주행 권장')
    if r['curve_dev'] > self.straight_tol:
      print(f'  ⚠ 궤적이 직선보다 {r["curve_dev"]:.2f}m 길다 — 곡선 주행으로 보임.')
      print('    직진이 아니면 RTK 직선거리가 실제 주행거리보다 짧아 계수가 왜곡된다.')
    print(f'  ★ 보정계수 = {r["factor"]:.4f}')
    print()
    print('  적용 방법 — henes_firmware.ino 에서 둘 중 하나:')
    new_cpr = FW_COUNTS_PER_REV / r['factor'] if r['factor'] > 1e-6 else 0
    new_rad = FW_WHEEL_RADIUS * r['factor']
    print(f'    counts_per_revolution : -290  →  {-new_cpr:.1f}')
    print(f'    (또는) wheel_radius   : 0.13  →  {new_rad:.4f}')
    print('    ※ 둘 중 하나만 고칠 것. 물리적으로는 타이어 눌림 때문이므로')
    print('      wheel_radius 쪽을 고치는 편이 의미가 분명하다.')
    print('=' * 60)


def main(args=None):
  rclpy.init(args=args)
  node = EncoderCalib()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.summary()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

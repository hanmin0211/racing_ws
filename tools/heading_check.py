#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heading_check.py — IMU yaw 부호가 맞는지 주행으로 검증.

헤딩 초기화 직후엔 yaw = GPS course 로 맞춰지므로 그 시점 비교는 순환논리다.
**yaw 부호가 뒤집혔다면 차량이 회전할 때 yaw가 반대로 돌아** 오차가 벌어진다.
그래서 실제로 곡선 주행을 하면서, 위치 변화로 계산한 진짜 진행방향(course)과
로컬라이제이션이 보고하는 yaw 를 비교한다.

  판정:
    평균 오차 < 20°   → 정상
    평균 오차 ~180°   → 헤딩 180° 뒤집힘
    회전할수록 오차 증가 → yaw 부호 반전 (invert_imu_yaw 필요)

사용: 이 스크립트를 켠 상태로 **곡선을 포함해 20~30m 주행** 후 Ctrl-C
  python3 tools/heading_check.py
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def norm(a):
  return math.atan2(math.sin(a), math.cos(a))


class HeadingCheck(Node):

  def __init__(self):
    super().__init__('heading_check')
    self.prev = None
    self.samples = []          # (course_deg, yaw_deg, diff_deg)
    self.create_subscription(Odometry, '/odometry/filtered', self.cb, 20)
    self.create_timer(3.0, self.progress)
    print('곡선을 포함해 20~30m 주행하세요. Ctrl-C 로 결과 출력.', flush=True)

  def cb(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    if self.prev is None:
      self.prev = (x, y)
      return
    dx, dy = x - self.prev[0], y - self.prev[1]
    d = math.hypot(dx, dy)
    if d < 0.25:               # 지터 제거: 충분히 움직였을 때만
      return
    course = math.atan2(dy, dx)
    diff = math.degrees(norm(course - yaw))
    self.samples.append((math.degrees(course), math.degrees(yaw), diff))
    self.prev = (x, y)

  def progress(self):
    if self.samples:
      last = self.samples[-1]
      print(f'  샘플 {len(self.samples)}개 | course {last[0]:+7.1f}° '
            f'yaw {last[1]:+7.1f}° 차이 {last[2]:+6.1f}°', flush=True)
    else:
      print('  대기중 — 차량을 움직이세요', flush=True)

  def summary(self):
    print('\n' + '=' * 58)
    if len(self.samples) < 8:
      print(f'샘플 부족({len(self.samples)}) — 더 주행해야 판정 가능')
      print('=' * 58)
      return
    diffs = [s[2] for s in self.samples]
    absd = [abs(d) for d in diffs]
    mean = sum(absd) / len(absd)
    # 180도 부근인지 확인 (부호 무시)
    near180 = sum(1 for d in absd if d > 150) / len(absd)
    # 회전 구간(course 변화가 큰 구간)에서 오차가 커지는지
    turns = []
    for i in range(1, len(self.samples)):
      dc = abs(norm(math.radians(self.samples[i][0] - self.samples[i - 1][0])))
      if math.degrees(dc) > 8:      # 회전 중
        turns.append(absd[i])
    print(f'샘플 {len(self.samples)}개')
    print(f'  평균 |course − yaw| = {mean:.1f}°')
    print(f'  최대 = {max(absd):.1f}°,  180° 근처 비율 = {near180*100:.0f}%')
    if turns:
      print(f'  회전 구간 평균 오차 = {sum(turns)/len(turns):.1f}° '
            f'({len(turns)}샘플)')
    print('-' * 58)
    if near180 > 0.6:
      print('  ❌ 헤딩이 180° 뒤집혀 있음 → invert_imu_yaw:=true 로 재확인')
    elif mean > 35:
      print('  ❌ 오차가 큼 → yaw 부호 반전 의심. invert_imu_yaw:=true 시도')
      if turns and sum(turns) / len(turns) > mean:
        print('     (회전 구간에서 오차가 더 큼 = 부호 반전의 전형적 증상)')
    elif mean > 20:
      print('  ⚠ 오차가 다소 큼 — 저속 GPS 노이즈일 수 있음. 재측정 권장')
    else:
      print('  ✅ 헤딩 정상 (yaw 가 실제 진행방향을 따라감)')
    print('=' * 58)


def main():
  rclpy.init()
  n = HeadingCheck()
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.summary()
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

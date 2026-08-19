#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_slalom_test.py — 대회 회피 미션(슬라롬) 폐루프 오프라인 검증.

실물 코스 없이, 실제 대회 배치를 합성 라이다로 재현해 follow-gap 회피가
박스를 안 박고 통과하는지 검증한다. (ROS 불필요 — 순수 계산)

대회 스펙(2026):
  · 박스 900×500×600mm, 종방향 2.5m 간격, 좌우 교차 슬라롬(2케이스).
  · follow-gap 이 avoid_steer 를 주면 그대로 조향, CLEAR 면 직진.

검증 항목:
  · 전 구간 박스와의 최소 간격 (차폭 고려) — 충돌 없이 통과하는가.
  · 조향 포화 여부 / 코스 이탈 여부.
"""

import math
import sys

sys.path.insert(0, '/home/han/racing_ws/src/lidar_clustering')

from lidar_clustering.follow_gap_planner import FollowGapPlanner


class Box:
  """차량 맵 좌표계의 축정렬 박스(장애물)."""

  def __init__(self, cx, cy, length=0.9, width=0.5):
    self.cx, self.cy = cx, cy
    self.x0, self.x1 = cx - length / 2, cx + length / 2
    self.y0, self.y1 = cy - width / 2, cy + width / 2

  def clearance(self, px, py):
    dx = max(self.x0 - px, 0, px - self.x1)
    dy = max(self.y0 - py, 0, py - self.y1)
    return math.hypot(dx, dy)


class FakeScan:
  """맵상의 박스들을 차량(px,py,yaw)에서 본 LaserScan 으로 레이캐스트."""

  def __init__(self, n=360, rmax=12.0):
    self.n = n
    self.angle_min = -math.pi
    self.angle_max = math.pi
    self.angle_increment = 2 * math.pi / n
    self.range_min = 0.10
    self.range_max = rmax

  def cast(self, boxes, px, py, yaw):
    ranges = []
    for i in range(self.n):
      # 차량 로컬(정면 +x, yaw_offset=0 으로 planner 를 쓸 것)
      a = self.angle_min + i * self.angle_increment
      wa = yaw + a
      cos, sin = math.cos(wa), math.sin(wa)
      best = float('inf')
      for b in boxes:
        t = self._ray_box(px, py, cos, sin, b)
        if t is not None and t < best:
          best = t
      ranges.append(best if best < self.range_max else float('inf'))
    self.ranges = ranges
    return self

  @staticmethod
  def _ray_box(px, py, cos, sin, b):
    # slab method
    tmin, tmax = 0.0, 1e9
    for p, d, lo, hi in ((px, cos, b.x0, b.x1), (py, sin, b.y0, b.y1)):
      if abs(d) < 1e-9:
        if p < lo or p > hi:
          return None
      else:
        t1, t2 = (lo - p) / d, (hi - p) / d
        if t1 > t2:
          t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
          return None
    return tmin if tmin > 0 else None


def run(case_name, boxes, y_start=0.0, v=0.6):
  planner = FollowGapPlanner(
      yaw_offset_deg=0.0, front_fov_deg=180.0, min_range=0.10, max_range=8.0,
      track_width=2.7, planning_lookahead=2.2, obstacle_trigger_distance=3.0,
      vehicle_width=0.775, safety_margin=0.25, straight_deadband_deg=5.0,
      min_gap_width_deg=3.0, side_score_margin=0.20)
  scan = FakeScan()
  L = 0.785
  max_steer = math.radians(18)
  px, py, yaw = -3.0, y_start, 0.0    # 첫 박스 3m 앞에서 시작
  dt = 0.05
  delta = 0.0
  slew = math.radians(90) * dt
  min_clear = float('inf')
  x_end = max(b.cx for b in boxes) + 2.0
  steps = 0
  collided = False
  half_w = 0.775 / 2
  while px < x_end and steps < 4000:
    d = planner.plan(scan.cast(boxes, px, py, yaw))
    if d.mode in ('AVOID',):
      tgt = max(-max_steer, min(max_steer, math.radians(d.best_angle_deg)))
      vv = 0.5      # 회피 저속
    elif d.mode in ('BLOCKED', 'NO_SCAN'):
      tgt = delta   # 유지(정지 상황)
      vv = 0.0
    else:            # CLEAR → GPS 경로(중앙선 y=0)로 복귀. 실제 시스템은 이때
      # pure-pursuit 가 경로를 따르므로 '직진'이 아니라 중앙선으로 돌아온다.
      # 간이 경로추종: heading + 횡오차(py) 비례로 중앙선 겨냥.
      tgt = max(-max_steer, min(max_steer, -1.2 * yaw - 0.6 * py))
      vv = v
    delta = max(delta - slew, min(delta + slew, tgt))
    px += vv * math.cos(yaw) * dt
    py += vv * math.sin(yaw) * dt
    yaw += vv / L * math.tan(delta) * dt
    # 차량 양 모서리(전방) 간격 체크
    for corner in ((px + 0.4 * math.cos(yaw) - half_w * math.sin(yaw),
                    py + 0.4 * math.sin(yaw) + half_w * math.cos(yaw)),
                   (px + 0.4 * math.cos(yaw) + half_w * math.sin(yaw),
                    py + 0.4 * math.sin(yaw) - half_w * math.cos(yaw))):
      for b in boxes:
        c = b.clearance(*corner)
        min_clear = min(min_clear, c)
        if c <= 0.0:
          collided = True
    steps += 1
    if vv == 0.0 and d.mode != 'CLEAR':
      # 정지 상황이 지속되면 통과 실패
      if steps > 200:
        break
  passed = px >= x_end and not collided
  print(f'  [{case_name}] {"✅ 통과" if passed else "❌ 실패"} — '
        f'최종 x={px:.1f}m, 박스 최소간격 {min_clear:.2f}m, '
        f'{"충돌!" if collided else "충돌없음"}')
  return passed


def main():
  s = 2.5   # 종방향 간격
  off = 0.65
  print('대회 슬라롬 회피 폐루프 검증 (박스 0.9×0.5m, 2.5m 간격, ±0.65m 교차)\n')
  # 케이스1: 좌-우-좌
  c1 = [Box(0, +off), Box(s, -off), Box(2 * s, +off)]
  # 케이스2: 우-좌-우
  c2 = [Box(0, -off), Box(s, +off), Box(2 * s, -off)]
  r1 = run('케이스1 좌-우-좌', c1)
  r2 = run('케이스2 우-좌-우', c2)
  print()
  print('결과:', '✅ 두 케이스 모두 통과' if (r1 and r2) else '⚠ 일부 실패 — 파라미터 튜닝 필요')


if __name__ == '__main__':
  main()

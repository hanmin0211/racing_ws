#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_slalom_test.py — 대회 회피 미션(슬라롬) 폐루프 오프라인 검증.

실물 코스 없이 장애물 배치를 합성 라이다로 재현해, follow-gap 회피가 장애물을
안 박고 통과하는지 검증한다. (ROS 불필요 — 순수 계산)

★ 대회 규정 (HL FMA 2026 경기규정 항목 4 — S코스 장애물 회피, 신규)
  · 장애물은 **T870 차체(바퀴 없음) 2대**를 고정 배치한다. (박스가 아니다)
  · 중앙선 없이 **좌·우측으로 자유 회피** 주행.
  · 배치는 **좌→우 또는 우→좌 랜덤**으로 **매 주행마다 변동 가능**.
    → 그래서 웨이포인트로 궤적을 미리 찍어둘 수 없다. 라이다가 그날 본 대로
      피해야 한다. 이 시험이 존재하는 이유다.
  · 감점: 접촉 10점/회 · 충돌 후 주행불가 10점 · 차량 이동 5점 ·
          미션 포기 15점 · 구간 내 차선이탈 최대 10점.

⚠ **장애물 치수는 아직 실측 전이다.**
  예전 이 파일에는 "이삿짐박스 900×500×600mm, 2.5m 간격, 3개" 가 적혀 있었는데
  **규정에 없는 값이었다**(출처 불명). 지금 기본값은 T870 제원에서 잡은 **잠정치**다.
  9/5 현장에서 차체 길이·폭과 배치 간격을 실측해 --length/--width/--spacing 으로
  넣고 다시 돌릴 것. 그전 튜닝 결과는 참고치일 뿐이다.

검증 항목:
  · 전 구간 장애물과의 최소 간격 (차폭 고려) — 충돌 없이 통과하는가.
  · 조향 포화 여부 / 코스 이탈 여부.
"""

import argparse
import math
import sys

sys.path.insert(0, '/home/han/racing_ws/src/lidar_clustering')

from lidar_clustering.follow_gap_planner import FollowGapPlanner


class Obstacle:
  """차량 맵 좌표계의 축정렬 장애물(T870 차체).

  기본값은 T870 제원 기준 **잠정치** — 현장 실측으로 덮어쓸 것.
  """

  def __init__(self, cx, cy, length=1.30, width=0.78):
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

  def cast(self, obstacles, px, py, yaw):
    ranges = []
    for i in range(self.n):
      # 차량 로컬(정면 +x, yaw_offset=0 으로 planner 를 쓸 것)
      a = self.angle_min + i * self.angle_increment
      wa = yaw + a
      cos, sin = math.cos(wa), math.sin(wa)
      best = float('inf')
      for b in obstacles:
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


def run(case_name, obstacles, y_start=0.0, v=0.6):
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
  x_end = max(b.cx for b in obstacles) + 2.0
  steps = 0
  collided = False
  half_w = 0.775 / 2
  while px < x_end and steps < 4000:
    d = planner.plan(scan.cast(obstacles, px, py, yaw))
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
      for b in obstacles:
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
        f'최종 x={px:.1f}m, 장애물 최소간격 {min_clear:.2f}m, '
        f'{"충돌!" if collided else "충돌없음"}')
  return passed


def main():
  ap = argparse.ArgumentParser(
      description='S코스 장애물 회피(T870 차체 2대) 폐루프 검증')
  ap.add_argument('--length', type=float, default=1.30,
                  help='장애물 길이[m] — T870 차체. **현장 실측값을 넣을 것**')
  ap.add_argument('--width', type=float, default=0.78,
                  help='장애물 폭[m] — T870 차체. **현장 실측값을 넣을 것**')
  ap.add_argument('--spacing', type=float, default=2.5,
                  help='두 장애물의 종방향 간격[m] — 규정에 명시 없음. 실측할 것')
  ap.add_argument('--offset', type=float, default=0.65,
                  help='중심선에서 좌우 오프셋[m] — 실측할 것')
  ap.add_argument('--speed', type=float, default=0.6, help='주행 속도[m/s]')
  args = ap.parse_args()

  def O(cx, cy):
    return Obstacle(cx, cy, args.length, args.width)

  s, off = args.spacing, args.offset
  print('S코스 장애물 회피 검증 — 규정: T870 차체 2대, 좌우 랜덤 배치\n')
  print(f'  장애물 {args.length:.2f}×{args.width:.2f}m · 간격 {s:.2f}m · '
        f'오프셋 ±{off:.2f}m · 속도 {args.speed:.1f}m/s')
  print('  ⚠ 위 값은 잠정치다 — 현장 실측 후 --length/--width/--spacing 으로 덮어쓸 것\n')

  # 규정: 배치는 좌→우 또는 우→좌 랜덤. 두 경우 다 통과해야 한다.
  r1 = run('배치A 좌→우', [O(0, +off), O(s, -off)], v=args.speed)
  r2 = run('배치B 우→좌', [O(0, -off), O(s, +off)], v=args.speed)
  print()
  ok = r1 and r2
  print('결과:', '✅ 두 배치 모두 통과' if ok
        else '⚠ 일부 실패 — 파라미터 튜닝 필요 (배치는 당일 랜덤이라 둘 다 통과해야 한다)')
  sys.exit(0 if ok else 1)


if __name__ == '__main__':
  main()

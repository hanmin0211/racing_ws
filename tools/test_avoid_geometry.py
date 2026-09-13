#!/usr/bin/env python3
"""test_avoid_geometry.py — 회피가 '충분한 각'을 요구할 수 있는지 지킨다.

왜 이 테스트가 있나
  2026-09-13 학교 주행: 회피는 하는데 조향각이 모자라 장애물을 스쳤다.
  원인은 조향 클램프가 아니라 **플래너가 큰 각을 요구할 수 없는 기하**였다.
  갭을 겨냥할 수 있는 각도 창은
      atan(max_center_y / planning_lookahead),
      max_center_y = track_width/2 − 차폭/2 − safety_margin
  이고 옛 값(track_width 2.7 / lookahead 2.2)에서는 ±17.9° 뿐이었다.
  정면 장애물이 안전버블로 막는 각은 2.0m 에서 ±25.7° 라 창보다 넓어서,
  남는 갭이 없어 AVOID 가 아니라 **BLOCKED(정지)** 로 떨어졌다.

  track_width 를 넓히면 창은 커지지만 옆 벽·연석을 장애물로 오인한다.
  lookahead 를 줄이면 **탐지 폭은 그대로 두고 조준 창만** 넓어진다 — 그래서
  track_width 3.0 / lookahead 1.5 를 택했다.

  이 파일은 그 두 성질을 고정한다:
    ① 정면 장애물에 2.0m 까지 AVOID 가 유지되고 각이 15° 이상 나온다
    ② 옆 1.6m 벽만 있을 때는 AVOID 를 내지 않는다(연석 오탐 금지)

  python3 tools/test_avoid_geometry.py
"""
import math
import sys

import numpy as np

sys.path.insert(0, '/home/han/racing_ws/src/lidar_clustering')
from lidar_clustering.follow_gap_planner import FollowGapPlanner  # noqa: E402

# bringup.launch.py 가 넘기는 값과 같아야 한다.
TRACK_WIDTH = 3.0
PLANNING_LOOKAHEAD = 1.5
OBSTACLE_TRIGGER = 4.0
MAX_STEER_DEG = 18.0      # vehicle_cmd_mux 의 클램프


class _Scan:
  pass


def make_scan(obstacles, free=6.0, n=1440):
  """obstacles = [(전방 x[m], 좌측 y[m], 폭[m])] → 360° LaserScan 흉내."""
  s = _Scan()
  s.angle_min = -math.pi
  s.angle_increment = 2 * math.pi / n
  s.range_min, s.range_max = 0.05, 16.0
  r = np.full(n, float(free))
  for (x, y, w) in obstacles:
    for dy in np.linspace(-w / 2.0, w / 2.0, 60):
      d = math.hypot(x, y + dy)
      raw = math.atan2(y + dy, x) + math.pi   # yaw_offset 180° 보정
      raw = (raw + math.pi) % (2 * math.pi) - math.pi
      i = int(round((raw - s.angle_min) / s.angle_increment)) % n
      r[i] = min(r[i], d)
  s.ranges = r.tolist()
  return s


def planner():
  return FollowGapPlanner(
      yaw_offset_deg=180.0, front_fov_deg=180.0, min_range=0.30,
      max_range=8.0, track_width=TRACK_WIDTH,
      planning_lookahead=PLANNING_LOOKAHEAD,
      obstacle_trigger_distance=OBSTACLE_TRIGGER,
      vehicle_width=0.775, safety_margin=0.25,
      straight_deadband_deg=5.0, min_gap_width_deg=3.0,
      side_score_margin=0.20, aim='path', aim_margin_deg=2.0)


def main():
  pl = planner()
  fails = []

  window = math.degrees(math.atan(pl.max_center_y / PLANNING_LOOKAHEAD))
  print(f'조준 창 ±{window:.1f}°  (max_center_y {pl.max_center_y:.3f}m / '
        f'lookahead {PLANNING_LOOKAHEAD}m)')
  if window < 25.0:
    fails.append(f'조준 창이 {window:.1f}° 로 좁다 — 25° 이상이어야 '
                 f'2m 정면 장애물에서 갭이 남는다')

  print('\n정면 장애물(폭 0.5m) — AVOID 가 유지되고 각이 충분한가')
  for d in (3.5, 3.0, 2.5, 2.0):
    dec = pl.plan(make_scan([(d, 0.0, 0.5)]), target_deg=0.0)
    clipped = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, dec.best_angle_deg))
    ok = dec.mode == 'AVOID' and abs(dec.best_angle_deg) >= 15.0
    print(f'   {d:.1f}m  {dec.mode:>8}  요구 {dec.best_angle_deg:+6.1f}° '
          f'→ 클램프 {clipped:+6.1f}°  {"OK" if ok else "✗"}')
    if not ok:
      fails.append(f'{d:.1f}m 정면: mode={dec.mode} '
                   f'각={dec.best_angle_deg:+.1f}° (AVOID·15° 이상이어야 함)')

  print('\n좌우 1.6m 벽만 있을 때 — 연석을 장애물로 오인하면 안 된다')
  walls = ([(x, 1.6, 0.1) for x in np.arange(1.0, 5.0, 0.15)]
           + [(x, -1.6, 0.1) for x in np.arange(1.0, 5.0, 0.15)])
  dec = pl.plan(make_scan(walls), target_deg=0.0)
  ok = dec.mode != 'AVOID'
  print(f'   벽 양쪽  {dec.mode:>8}  {"OK" if ok else "✗ 오탐"}')
  if not ok:
    fails.append(f'옆 벽에 {dec.mode} — track_width 가 너무 넓다(연석 오탐)')

  print()
  if fails:
    print('❌ 실패')
    for f in fails:
      print(f'   · {f}')
    return 1
  print('✅ 통과 — 회피 기하가 충분한 조향각을 낼 수 있다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

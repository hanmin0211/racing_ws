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
TRACK_WIDTH = 2.4      # 규정 도로폭 2.7 − 연석 제외 여유
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
  if window < 18.0:
    fails.append(f'조준 창이 {window:.1f}° 로 좁다 — 조향 클램프 '
                 f'{MAX_STEER_DEG:.0f}° 를 채울 수 없다')

  # ★ 정중앙 장애물은 BLOCKED 가 **올바른 답**이다.
  #   통로 2.4m 에 폭 0.5m 가 정중앙이면 양옆 0.95m 인데, 지나가려면
  #   차폭 0.775 + 안전여유 0.25×2 = 1.275m 가 필요하다. 물리적으로 못 간다.
  #   억지로 통과시키면 스친다 — 접촉 10점이다. 서서 감속하는 게 맞고,
  #   교착은 cluster_plot_node 의 blocked_escape_s(8초)가 푼다.
  #   규정 배치(아래)는 장애물이 중앙선에 걸쳐 있어 이 경우가 아니다.

  # 규정 도로폭 2700mm → 연석은 ±1.35m 에 있다.
  # 여기서 AVOID 가 뜨면 커브에서 경로조향을 버려 밖으로 밀린다 = 이탈 = 탈락.
  print('\n규정 연석(±1.35m)만 보일 때 — 장애물로 오인하면 커브에서 이탈한다')
  walls = ([(x, 1.35, 0.1) for x in np.arange(0.8, 5.0, 0.1)]
           + [(x, -1.35, 0.1) for x in np.arange(0.8, 5.0, 0.1)])
  for target in (0.0, 15.0, -15.0):
    dec = pl.plan(make_scan(walls), target_deg=target)
    ok = dec.mode != 'AVOID'
    print(f'   연석만 · 경로 {target:+5.1f}°  {dec.mode:>8}  '
          f'{"OK" if ok else "✗ 오탐"}')
    if not ok:
      fails.append(f'연석에 {dec.mode}(경로 {target:+.0f}°) — '
                   f'track_width 가 너무 넓다')

  print('\n규정 S코스 장애물 — 연석과 함께 있어도 회피해야 한다')
  print('   (장애물 중심 +0.45m, 폭 0.5m = 중앙선 넘어 200mm 돌출)')
  for d in (3.0, 2.5, 2.0):
    dec = pl.plan(make_scan(walls + [(d, 0.45, 0.5)]), target_deg=0.0)
    ok = dec.mode == 'AVOID'
    print(f'   {d:.1f}m  {dec.mode:>8}  {dec.best_angle_deg:+6.1f}°  '
          f'{"OK" if ok else "✗"}')
    if not ok:
      fails.append(f'연석+장애물 {d:.1f}m: {dec.mode} — 회피해야 한다')

  print('\n콘/의자 두 개가 만든 문 — 바깥으로 돌지 말고 사이로 지나야 한다')
  print('   (차폭 0.775 + 안전여유 0.25×2 = 안쪽 틈 1.275m 이상이어야 물리적으로 가능)')
  for spacing, want_pass in ((1.8, False), (2.0, True), (2.5, True)):
    dec = pl.plan(make_scan([(3.0, +spacing / 2, 0.45),
                             (3.0, -spacing / 2, 0.45)]), target_deg=0.0)
    passed = dec.mode == 'AVOID' and abs(dec.best_angle_deg) < 5.0
    ok = passed == want_pass
    verdict = '사이로 통과' if passed else f'{dec.mode}(우회/정지)'
    print(f'   중심간격 {spacing:.1f}m (안쪽 {spacing - 0.45:.2f}m)  '
          f'{verdict:<16} {"OK" if ok else "✗"}')
    if not ok:
      fails.append(
          f'문 간격 {spacing:.1f}m: {verdict} — '
          f'{"통과해야" if want_pass else "막혀야"} 한다')

  # ────────────────────────────────────────────────────────────────
  # ★ 2026-09-16 — "차가 문 사이 대신 바깥으로 돈다" 를 조사하며 확인한 불변식.
  #   처음엔 갭 선택 규칙(prefer_path_gap 의 '품었는가' 판정)이 원인인 줄 알고
  #   고쳤는데, 시험해 보니 **옛 코드도 문을 고른다.** 구조를 따져보니 이유가
  #   있었다 — 트랙 창이 ±1.2m 인데 의자 바깥 끝이 0.92~1.33m 이고 안전반경이
  #   0.6375m 라, **문 바깥에 갭이 생길 여지가 아예 없다.**
  #   즉 플래너는 '바깥으로 돌' 수가 없다. 관측된 현상은 다른 원인이다
  #   (의자가 트랙 창 밖이라 장애물로 안 잡히는 쪽이 유력하다).
  #   그 불변식을 여기에 못 박아 둔다 — 나중에 track_width 를 올리면 깨진다.
  print('\n★ 문 바깥으로는 돌 수 없어야 한다 (구조적 불변식)')
  for target in (0.0, 8.0, 16.0):
    dec = pl.plan(make_scan([(3.0, +1.0, 0.45), (3.0, -1.0, 0.45)]),
                  target_deg=target)
    thru = dec.mode == 'AVOID' and abs(dec.best_angle_deg) <= 6.5
    print(f'   경로 목표 {target:+5.1f}°  →  {dec.mode:<8} '
          f'조준 {dec.best_angle_deg:+6.1f}°  '
          f'{"문 통과" if thru else "바깥으로 돌았다"}  {"OK" if thru else "✗"}')
    if not thru:
      fails.append(f'문 통과(목표 {target:+.0f}°): 조준 '
                   f'{dec.best_angle_deg:+.1f}° — 바깥으로 돌았다')

  print('\n★ 트랙 창 밖의 물체는 장애물로 안 잡힌다 (의자 배치 기준)')
  for off in (1.0, 1.3, 1.6):
    dec = pl.plan(make_scan([(3.0, +off, 0.45), (3.0, -off, 0.45)]),
                  target_deg=0.0)
    seen = dec.mode != 'CLEAR'
    print(f'   경로에서 ±{off:.1f}m  →  {dec.mode:<8} '
          f'{"장애물로 잡힘" if seen else "**안 보임**(창 ±1.2m 밖)"}')
    if off <= 1.0 and not seen:
      fails.append(f'±{off:.1f}m 의자를 못 본다 — 창 안인데 놓쳤다')

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

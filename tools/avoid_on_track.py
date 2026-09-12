#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""avoid_on_track.py — **실제 트랙 위에** 장애물을 놓고 라이다 회피를 폐루프로 본다.

★ 왜 이게 필요한가
  `tools/lidar_slalom_test.py` 는 직선 위에서 회피만 본다. 그런데 실제로는
  경로추종(pure pursuit) 과 회피(follow-gap) 가 **조향을 서로 뺏으며** 돌아간다.
  먹스가 AVOID 동안 GPS 조향을 통째로 회피각으로 갈아끼우기 때문이다.
  그래서 "이 트랙 이 지점에 장애물 N 개를 놓으면 피하는가" 는 둘을 같이
  돌려봐야만 답이 나온다. 이 하네스가 그걸 한다.

  섞어 쓰는 것:
    · 경로/추종  : waypoint_follower.local_path_core + local_pure_pursuit 조향식
                   (tools/tracking_sim.py 와 같은 계산)
    · 종방향     : longitudinal_controller_node 의 감속식 그대로
                   (곡률 감속 + 전방 장애물 비례 감속/정지 + 가감속 프로파일)
    · 회피       : lidar_clustering.follow_gap_planner (실제 플래너)
    · 합성 라이다: tools/lidar_slalom_test.FakeScan (박스 레이캐스트)
    · 막힘 탈출  : cluster_plot_node 의 blocked_escape_s 와 같은 동작

사용:
  # 가장 긴 직선에 3개를 좌우 번갈아 자동 배치
  python3 tools/avoid_on_track.py \
      --waypoints config/chungju_school/wp_school_track_0.5.yaml --auto 3

  # 위치를 직접 지정 (s[m]:횡오프셋[m], + 는 왼쪽)
  python3 tools/avoid_on_track.py --waypoints <파일> \
      --obstacles 20:+0.6 24:-0.6 28:+0.6

  # 현장 실측 치수로
  python3 tools/avoid_on_track.py --waypoints <파일> --auto 3 \
      --length 1.30 --width 0.78 --spacing 4.0 --offset 0.65

  --plot out.png  으로 궤적 그림을 남긴다.

⚠ 합성 라이다는 **장애물만** 본다. 실제로는 연석·풀·사람도 같이 들어온다.
  그래서 여기서 통과해도 현장에서 더 어려울 수 있다. 반대로 여기서 실패하면
  현장에서도 실패한다 — **거르는 용도**로 쓸 것.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, 'tools'))
sys.path.insert(0, os.path.join(WS, 'src', 'waypoint_follower'))
sys.path.insert(0, os.path.join(WS, 'src', 'lidar_clustering'))

from lidar_clustering.follow_gap_planner import FollowGapPlanner  # noqa: E402
from lidar_slalom_test import FakeScan, Obstacle                  # noqa: E402
from waypoint_follower.local_path_core import (                   # noqa: E402
    LocalPathParams, build_local_path, is_closed_path)


def load_waypoints(path):
  d = yaml.safe_load(open(path, encoding='utf-8'))
  raw = d.get('waypoints', d.get('poses'))
  return np.array([[float(p['x']), float(p['y'])] for p in raw])


def arclength(w):
  return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(w, axis=0).T))])


def point_at_s(w, s_arr, s):
  """진행거리 s 위치의 (좌표, 진행방향 단위벡터, 좌측 법선)."""
  i = int(np.searchsorted(s_arr, s))
  i = max(1, min(i, len(w) - 1))
  t = ((s - s_arr[i - 1]) / max(s_arr[i] - s_arr[i - 1], 1e-9))
  p = w[i - 1] + t * (w[i] - w[i - 1])
  d = w[i] - w[i - 1]
  d = d / max(np.hypot(*d), 1e-9)
  n = np.array([-d[1], d[0]])        # 좌측(+)
  return p, d, n


def cross_track(w, x, y):
  a, b = w[:-1], w[1:]
  ab = b - a
  ap = np.array([x, y]) - a
  den = np.einsum('ij,ij->i', ab, ab)
  t = np.clip(np.einsum('ij,ij->i', ap, ab) / np.maximum(den, 1e-12), 0.0, 1.0)
  proj = a + t[:, None] * ab
  return float(np.min(np.hypot(proj[:, 0] - x, proj[:, 1] - y)))


def find_lookahead(pts, ld):
  """local_pure_pursuit_node.find_lookahead 와 동일."""
  if len(pts) < 2:
    return None
  s = 0.0
  for i in range(1, len(pts)):
    seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    if s + seg >= ld:
      t = (ld - s) / seg if seg > 1e-6 else 0.0
      return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
              pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))
    s += seg
  return pts[-1]


def longest_straight(w, s_arr, max_turn_deg=25.0):
  """헤딩 누적 변화가 작은 최장 구간의 (s_start, s_end)."""
  def hd(i):
    d = w[i + 1] - w[i]
    return math.atan2(d[1], d[0])
  best = (0.0, 0.0, 0.0)
  for a in range(len(w) - 3):
    tot, b = 0.0, a + 1
    while b < len(w) - 2:
      dh = hd(b) - hd(b - 1)
      dh = abs(math.degrees(math.atan2(math.sin(dh), math.cos(dh))))
      if tot + dh > max_turn_deg:
        break
      tot += dh
      b += 1
    if s_arr[b] - s_arr[a] > best[0]:
      best = (s_arr[b] - s_arr[a], s_arr[a], s_arr[b])
  return best[1], best[2]


def build_obstacles(w, s_arr, specs, length, width):
  obs = []
  for s, off in specs:
    p, _, n = point_at_s(w, s_arr, s)
    c = p + n * off
    obs.append(Obstacle(float(c[0]), float(c[1]), length, width))
  return obs


def simulate(w, obstacles, a):
  s_arr = arclength(w)
  p = LocalPathParams(n_back=5, n_forward=20, poly_order=3,
                      lookahead_distance=10.0, point_spacing=0.5,
                      curvature_preview=4.0, closed_path='auto',
                      goal_tolerance=1.0)
  closed = is_closed_path(w, p.closed_gap_thresh)
  planner = FollowGapPlanner(
      yaw_offset_deg=0.0, front_fov_deg=180.0, min_range=0.10, max_range=8.0,
      track_width=a.track_width, planning_lookahead=a.planning_lookahead,
      obstacle_trigger_distance=3.0, vehicle_width=a.vehicle_width,
      safety_margin=a.safety_margin, straight_deadband_deg=5.0,
      min_gap_width_deg=3.0, side_score_margin=0.20,
      aim=a.aim, aim_margin_deg=a.aim_margin_deg)
  scan = FakeScan()

  x, y = w[0]
  yaw = math.atan2(w[1][1] - w[0][1], w[1][0] - w[0][0])
  dt = 1.0 / a.rate
  L = a.wheelbase
  max_steer = math.radians(a.max_steer_deg)
  max_step = math.radians(a.max_steer_rate_deg) / a.rate
  half_w = a.vehicle_width / 2.0

  delta = 0.0
  v = 0.0
  prev_idx = None
  stuck = 0.0
  escaping = False
  log = {'x': [], 'y': [], 'cte': [], 's': [], 'v': [], 'delta': [],
         'mode': [], 'obs': []}
  events = []
  min_clear = float('inf')
  collided = False
  goal = False
  reason = None

  for step in range(int(a.max_time * a.rate)):
    out = build_local_path(w, x, y, yaw, p, prev_idx=prev_idx, closed=closed)
    prev_idx = out['closest']
    if out.get('goal_reached'):
      goal = True
      reason = '완주'
      break
    if not out['ok'] or out['points'] is None or len(out['points']) < 2:
      reason = f'로컬 경로 생성 실패 (step {step})'
      break

    # ---- 라이다 ----
    d = planner.plan(scan.cast(obstacles, x, y, yaw))
    mode = d.mode
    # 고착 두 종류 (cluster_plot_node 와 같은 판정)
    #   BLOCKED/NO_SCAN     — 갭이 없다 / 스캔이 없다
    #   AVOID + 전방 근접   — 비켜가려는데 종방향이 차를 세워 못 나간다
    is_stuck = (mode in ('BLOCKED', 'NO_SCAN')
                or (mode == 'AVOID'
                    and float(d.front_distance) <= a.stuck_distance))
    if is_stuck:
      stuck += dt
    else:
      stuck = 0.0
      if escaping:
        events.append((step / a.rate, '막힘 해소 — 정상 복귀'))
      escaping = False
    if a.escape_s > 0.0 and stuck >= a.escape_s and not escaping:
      escaping = True
      events.append((step / a.rate, f'막힘 탈출 발동 ({stuck:.1f}s 막힘)'))
    # cluster_plot_node._publish_decision 규약
    def bearing_to_steer(deg):
      if a.steer_mode != 'pursuit':
        return float(deg)
      al = math.radians(float(deg))
      return math.degrees(math.atan(2.0 * L * math.sin(al)
                                    / max(a.planning_lookahead, 0.1)))

    if mode == 'CLEAR':
      obs_d, avoid = a.clear_distance, float('nan')
    elif mode == 'AVOID':
      obs_d, avoid = float(d.front_distance), bearing_to_steer(d.best_angle_deg)
    else:                                    # BLOCKED / NO_SCAN
      obs_d, avoid = float(d.front_distance), float('nan')
    if escaping:
      # 속도만 푼다. AVOID 고착이면 **조향은 유지**해 하던 회피를 마저 시킨다.
      obs_d = a.clear_distance
      avoid = (bearing_to_steer(d.best_angle_deg) if mode == 'AVOID'
               else float('nan'))

    # ---- 횡방향: pure pursuit, AVOID 면 먹스가 통째로 갈아끼운다 ----
    ld = min(max(a.k_ld * v + a.min_ld, a.min_ld), a.max_ld)
    la = find_lookahead(out['points'], ld)
    if la is None:
      reason = 'lookahead 없음'
      break
    x_ld, y_ld = la
    ld_dist = math.hypot(x_ld, y_ld)
    if x_ld <= 0.05 or ld_dist < 0.1:
      reason = f'lookahead 가 차량 뒤 (step {step})'
      break
    d_path = math.atan(L * (2.0 * y_ld / (ld_dist * ld_dist)))
    # ★ 먹스 규약: AVOID 면 GPS 조향을 **통째로** 회피각으로 갈아끼운다.
    #   (vehicle_cmd_mux_node.py: s, mode = self.avoid_steer, 'AVOID(라이다)')
    d_target = math.radians(avoid) if not math.isnan(avoid) else d_path
    d_target = max(-max_steer, min(max_steer, d_target))
    delta = max(delta - max_step, min(delta + max_step, d_target))

    # ---- 종방향: longitudinal_controller_node.decide_target 과 같은 식 ----
    kappa = abs(float(out['curvature']))
    v_t = a.v_max / (1.0 + a.curv_gain * kappa)
    v_t = min(a.v_max, max(v_t, a.v_min))
    if obs_d < a.obs_trigger:
      span = max(1e-3, a.obs_trigger - a.obs_stop)
      v_t = min(v_t, a.v_slow * max(0.0, obs_d - a.obs_stop) / span)
      if obs_d <= a.obs_stop:
        v_t = 0.0
    v_t = max(v_t, a.v_min) if v_t > 0.05 else 0.0
    dv = v_t - v
    v += min(dv, a.max_accel * dt) if dv > 0 else -min(-dv, a.max_decel * dt)

    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(delta) * dt

    for corner in ((x + 0.4 * math.cos(yaw) - half_w * math.sin(yaw),
                    y + 0.4 * math.sin(yaw) + half_w * math.cos(yaw)),
                   (x + 0.4 * math.cos(yaw) + half_w * math.sin(yaw),
                    y + 0.4 * math.sin(yaw) - half_w * math.cos(yaw))):
      for b in obstacles:
        c = b.clearance(*corner)
        min_clear = min(min_clear, c)
        if c <= 0.0:
          collided = True

    log['x'].append(x); log['y'].append(y)
    log['cte'].append(cross_track(w, x, y))
    log['s'].append(s_arr[out['closest']])
    log['v'].append(v); log['delta'].append(math.degrees(delta))
    log['mode'].append(mode + ('(ESC)' if escaping else ''))
    log['obs'].append(obs_d)
    if stuck > a.escape_s + 20.0 or (a.escape_s <= 0 and stuck > 20.0):
      reason = f'{mode} 고착 {stuck:.0f}s — 빠져나오지 못했다'
      break
  else:
    reason = f'시간초과 ({a.max_time}s)'

  return {k: (np.array(v) if k != 'mode' else v) for k, v in log.items()} | {
      'goal': goal, 'reason': reason, 'collided': collided,
      'min_clear': min_clear, 'events': events, 'path_len': float(s_arr[-1])}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--waypoints', required=True)
  ap.add_argument('--obstacles', nargs='*', default=None,
                  metavar='S:OFF', help='s[m]:횡오프셋[m] (+ 왼쪽). 예: 20:+0.6')
  ap.add_argument('--auto', type=int, default=0,
                  help='가장 긴 직선에 이 개수를 좌우 번갈아 자동 배치')
  ap.add_argument('--spacing', type=float, default=4.0,
                  help='자동 배치 종방향 간격[m] — **현장 실측할 것**')
  ap.add_argument('--offset', type=float, default=0.65,
                  help='자동 배치 횡 오프셋[m] — **현장 실측할 것**')
  ap.add_argument('--length', type=float, default=1.30, help='장애물 길이[m]')
  ap.add_argument('--width', type=float, default=0.78, help='장애물 폭[m]')
  # 제어 파라미터 — 실제 설정과 같게
  ap.add_argument('--v-max', type=float, default=0.8)
  ap.add_argument('--v-min', type=float, default=0.25)
  ap.add_argument('--v-slow', type=float, default=0.8)
  ap.add_argument('--curv-gain', type=float, default=3.0)
  ap.add_argument('--k-ld', type=float, default=0.8)
  ap.add_argument('--min-ld', type=float, default=1.6)
  ap.add_argument('--max-ld', type=float, default=4.0)
  ap.add_argument('--wheelbase', type=float, default=0.785)
  ap.add_argument('--max-steer-deg', type=float, default=15.0)
  ap.add_argument('--max-steer-rate-deg', type=float, default=45.0)
  ap.add_argument('--max-accel', type=float, default=1.0)
  ap.add_argument('--max-decel', type=float, default=1.8)
  ap.add_argument('--obs-trigger', type=float, default=4.0)
  ap.add_argument('--obs-stop', type=float, default=0.8)
  ap.add_argument('--clear-distance', type=float, default=999.0)
  # 플래너
  ap.add_argument('--track-width', type=float, default=2.7)
  ap.add_argument('--planning-lookahead', type=float, default=2.2)
  ap.add_argument('--vehicle-width', type=float, default=0.775)
  ap.add_argument('--safety-margin', type=float, default=0.25)
  ap.add_argument('--escape-s', type=float, default=8.0)
  ap.add_argument('--aim', choices=('center', 'nearest'), default='center',
                  help="갭 안 겨냥점. center=갭 중앙(원형), "
                       "nearest=직진에 가장 가까운 각(필요한 만큼만 비켜감)")
  ap.add_argument('--aim-margin-deg', type=float, default=2.0)
  ap.add_argument('--steer-mode', choices=('bearing', 'pursuit'),
                  default='bearing',
                  help="회피각 처리. bearing=갭 방위각을 조향각으로 그대로(현행), "
                       "pursuit=퓨어퍼슛 환산 δ=atan(2·L·sinα/Ld)")
  ap.add_argument('--stuck-distance', type=float, default=1.2,
                  help='AVOID 인데 전방이 이 거리 안이면 고착으로 본다. '
                       '차가 실제로 서는 거리는 obs_stop(0.8)이 아니라 1.0m 다 '
                       '(v_obs<=0.05 이면 0 으로 떨어지는 클램프 때문)')
  ap.add_argument('--rate', type=float, default=20.0)
  ap.add_argument('--max-time', type=float, default=400.0)
  ap.add_argument('--plot', default=None)
  a = ap.parse_args()

  w = load_waypoints(a.waypoints)
  s_arr = arclength(w)

  specs = []
  if a.obstacles:
    for t in a.obstacles:
      ss, off = t.split(':')
      specs.append((float(ss), float(off)))
  elif a.auto > 0:
    s0, s1 = longest_straight(w, s_arr)
    mid = (s0 + s1) / 2.0
    start = mid - a.spacing * (a.auto - 1) / 2.0
    for i in range(a.auto):
      specs.append((start + i * a.spacing,
                    a.offset if i % 2 == 0 else -a.offset))
    print(f'자동 배치: 가장 긴 직선 s {s0:.1f}~{s1:.1f}m ({s1 - s0:.1f}m) 가운데에 '
          f'{a.auto}개, 간격 {a.spacing:.1f}m, 오프셋 ±{a.offset:.2f}m')
  else:
    print('--obstacles 또는 --auto 를 줄 것', file=sys.stderr)
    return 2

  obstacles = build_obstacles(w, s_arr, specs, a.length, a.width)

  print('=' * 74)
  print(f'트랙 위 라이다 회피 — {os.path.basename(a.waypoints)} '
        f'({len(w)}점 {s_arr[-1]:.1f}m)')
  print(f'  장애물 {len(specs)}개 · {a.length:.2f}×{a.width:.2f}m '
        f'· v_max {a.v_max} · 타각상한 {a.max_steer_deg}° '
        f'· 막힘탈출 {a.escape_s:.0f}s')
  for (ss, off) in specs:
    print(f'    s={ss:6.1f}m  횡 {off:+.2f}m ({"좌" if off > 0 else "우"})')
  print('-' * 74)

  r = simulate(w, obstacles, a)
  if len(r['cte']) == 0:
    print(f'  샘플 없음 — {r["reason"]}')
    return 1

  cte = r['cte']
  lane_lim = a.track_width / 2.0 - a.vehicle_width / 2.0
  n_av = sum(1 for m in r['mode'] if m.startswith('AVOID'))
  n_bl = sum(1 for m in r['mode'] if m.startswith(('BLOCKED', 'NO_SCAN')))
  n_esc = sum(1 for m in r['mode'] if '(ESC)' in m)

  print(f'  종료           : {r["reason"]}  '
        f'({len(cte) / a.rate:.0f}s, {r["s"].max():.1f}m 진행)')
  print(f'  충돌           : {"❌ 있음" if r["collided"] else "✅ 없음"}  '
        f'(장애물 최소간격 {r["min_clear"]:.2f}m)')
  print(f'  경로 이탈      : 평균 {cte.mean():.3f}m  최대 {cte.max():.3f}m  '
        f'(차선 허용 {lane_lim:.2f}m)')
  over = int((cte > lane_lim).sum())
  print(f'    차선 초과    : {over}샘플 ({over / len(cte) * 100:.1f}%)'
        + ('  ❌' if over else '  ✅'))
  print(f'  라이다 모드    : AVOID {n_av} · BLOCKED {n_bl} · 탈출중 {n_esc}')
  print(f'  조향           : 최대 {np.abs(r["delta"]).max():.1f}° '
        f'(상한 {a.max_steer_deg}°)')
  if r['events']:
    print('  이벤트         :')
    for t, e in r['events'][:8]:
      print(f'    [{t:6.1f}s] {e}')
  print('-' * 74)
  ok = (r['goal'] and not r['collided'] and over == 0)
  if ok:
    print('  ✅ 통과 — 완주 · 충돌 없음 · 차선 안에서 회피')
  else:
    if not r['goal']:
      print(f'  ❌ 완주 실패 — {r["reason"]}')
    if r['collided']:
      print('  ❌ 장애물 충돌 (규정: 접촉 1회당 10점)')
    if over:
      print(f'  ❌ 차선 이탈 최대 {cte.max():.2f}m > 허용 {lane_lim:.2f}m '
            '(1회당 1점·최대 10점, 연석이면 탈락)')
  print('=' * 74)

  if a.plot:
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import font_manager as fm
    for f in fm.findSystemFonts():
      if 'NotoSansCJK' in f:
        fm.fontManager.addfont(f)
        matplotlib.rcParams['font.family'] = \
            fm.FontProperties(fname=f).get_name()
        break
    matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(w[:, 0], w[:, 1], '-', color='0.6', lw=2, label='기준 경로')
    for b in obstacles:
      ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0,
                             color='tab:red', alpha=0.55))
    sc = ax.scatter(r['x'], r['y'], c=cte, cmap='YlOrRd', s=10,
                    vmin=0, vmax=max(0.3, float(cte.max())))
    plt.colorbar(sc, ax=ax, label='경로 이탈 [m]')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend()
    ax.set_title(f'트랙 위 회피 — 장애물 {len(specs)}개\n'
                 f'{"통과" if ok else "실패"} · 최소간격 {r["min_clear"]:.2f}m '
                 f'· 최대이탈 {cte.max():.2f}m')
    plt.tight_layout(); plt.savefig(a.plot, dpi=110)
    print(f'  그림: {a.plot}')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

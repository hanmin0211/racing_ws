#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tracking_sim.py — 경로 추종 폐루프 시뮬레이션.

sim_odom_publisher 는 가상 차량을 **경로 위에 강제로 올려놓고** 움직인다.
그래서 로컬 경로가 잘 그려지는지는 보이지만 '차가 경로를 따라가는가'는
전혀 검증되지 않는다(차량 위치가 언제나 정답이므로 오차가 0이다).

이 하네스는 실제 제어 체인과 같은 계산을 폐루프로 돌린다:

    자전거 모델 위치/헤딩
      → local_path_core.build_local_path()   (노드가 쓰는 그 코드 그대로 import)
      → pure pursuit 조향식 (local_pure_pursuit_node 와 동일)
      → 조향 슬루레이트 제한 + 타각 포화
      → 다시 자전거 모델

즉 차가 경로를 벗어나면 그 벗어난 위치에서 다음 로컬 경로가 만들어진다.
이래야 횡방향 오차(cross-track error)가 실제로 나온다.

측정 항목:
  · 횡방향 오차 (평균/RMS/최대) — 경로를 얼마나 잘 붙어 가는가
  · 조향 포화 시간 비율 — 타각 한계에 걸려 못 도는 구간이 있는가
  · 완주 여부 / 완주 거리

사용:
  python3 tools/tracking_sim.py
  python3 tools/tracking_sim.py --speed 0.35 --k-ld 0.4 --max-ld 3.0
  python3 tools/tracking_sim.py --plot /tmp/track.png
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'waypoint_follower'))
from waypoint_follower.local_path_core import (  # noqa: E402
    LocalPathParams, build_local_path, is_closed_path)

DEFAULT_WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
              'waypoints_recorded_resampled_0.5.yaml')


def load_waypoints(path):
  with open(path) as f:
    data = yaml.safe_load(f)
  raw = data.get('waypoints', data.get('poses'))
  return np.array([[float(p['x']), float(p['y'])] for p in raw])


def find_lookahead(pts, ld_target):
  """local_pure_pursuit_node.find_lookahead 와 동일한 계산."""
  if len(pts) < 2:
    return None
  s = 0.0
  for i in range(1, len(pts)):
    seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    if s + seg >= ld_target:
      t = (ld_target - s) / seg if seg > 1e-6 else 0.0
      return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
              pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))
    s += seg
  return pts[-1]


def cross_track(wps, x, y):
  """경로(꺾은선)까지의 최단 수직거리."""
  a = wps[:-1]
  b = wps[1:]
  ab = b - a
  ap = np.array([x, y]) - a
  denom = np.einsum('ij,ij->i', ab, ab)
  t = np.clip(np.einsum('ij,ij->i', ap, ab) / np.maximum(denom, 1e-12), 0.0, 1.0)
  proj = a + t[:, None] * ab
  return float(np.min(np.hypot(proj[:, 0] - x, proj[:, 1] - y)))


def simulate(wps, args):
  p = LocalPathParams(n_back=args.n_back, n_forward=args.n_forward,
                      poly_order=3, lookahead_distance=10.0,
                      point_spacing=0.5, curvature_preview=4.0,
                      closed_path='auto', goal_tolerance=1.0)
  closed = is_closed_path(wps, p.closed_gap_thresh)

  # 초기 자세: 경로 시작점, 경로 진행방향. 원하면 의도적으로 어긋나게 시작한다.
  x, y = wps[0]
  yaw0 = math.atan2(wps[1][1] - wps[0][1], wps[1][0] - wps[0][0])
  yaw = yaw0 + math.radians(args.init_heading_err)
  x += -math.sin(yaw0) * args.init_offset
  y += math.cos(yaw0) * args.init_offset

  dt = 1.0 / args.rate
  L = args.wheelbase
  max_steer = math.radians(args.max_steer_deg)
  max_step = math.radians(args.max_steer_rate_deg) / args.rate

  delta = 0.0
  prev_idx = None
  log = {'x': [], 'y': [], 'cte': [], 'delta': [], 'v': [], 's': [],
         'kappa': []}
  travelled = 0.0
  goal = False
  stop_reason = None

  for step in range(int(args.max_time * args.rate)):
    out = build_local_path(wps, x, y, yaw, p, prev_idx=prev_idx, closed=closed)
    prev_idx = out['closest']

    if out.get('goal_reached'):
      goal = True
      stop_reason = '완주 (goal_reached)'
      break
    if not out['ok'] or out['points'] is None or len(out['points']) < 2:
      stop_reason = f'로컬 경로 생성 실패 (step {step})'
      break

    pts = out['points']
    kappa_path = abs(float(out['curvature']))

    # 속도: longitudinal_controller 와 같은 곡률 감속식
    v = args.speed / (1.0 + args.curv_gain * kappa_path)
    v = max(args.min_speed, min(args.speed, v))

    # pure pursuit
    ld = min(max(args.k_ld * v + args.min_ld, args.min_ld), args.max_ld)
    la = find_lookahead(pts, ld)
    if la is None:
      stop_reason = 'lookahead 없음'
      break
    x_ld, y_ld = la
    ld_dist = math.hypot(x_ld, y_ld)
    if x_ld <= 0.05 or ld_dist < 0.1:
      stop_reason = f'lookahead가 차량 뒤 (step {step}) — 헤딩 이상'
      break

    kappa_cmd = 2.0 * y_ld / (ld_dist * ld_dist)
    d_target = max(-max_steer, min(max_steer, math.atan(L * kappa_cmd)))
    # 조향 슬루레이트 제한
    delta = max(delta - max_step, min(delta + max_step, d_target))

    # ★ 실제 바퀴 각도 = 명령각 + 캘리브 오차.
    # 조향 중심(STEER_CENTER)이 틀어져 있거나 COUNTS_PER_DEG 가 어긋나면
    # 명령한 각도와 실제 각도가 다르다. 실전 트랙은 최대 16.3°를 요구해
    # 한계 18°까지 여유가 1.7°뿐이므로 이 오차가 완주를 좌우한다.
    real = delta + math.radians(args.steer_bias)
    real *= args.steer_scale
    real = max(-max_steer, min(max_steer, real))

    # 자전거 모델 전진
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(real) * dt
    travelled += v * dt

    log['x'].append(x); log['y'].append(y)
    log['cte'].append(cross_track(wps, x, y))
    log['delta'].append(math.degrees(real))
    log['v'].append(v)
    log['s'].append(travelled)
    log['kappa'].append(kappa_path)
  else:
    stop_reason = f'시간초과 ({args.max_time}s)'

  return {k: np.array(v) for k, v in log.items()} | {
      'goal': goal, 'reason': stop_reason, 'travelled': travelled,
      'closed': closed, 'max_steer_deg': args.max_steer_deg}


def report(wps, r, args):
  cte = r['cte']
  path_len = float(np.sum(np.hypot(np.diff(wps[:, 0]), np.diff(wps[:, 1]))))
  print('=' * 68)
  print('경로 추종 폐루프 시뮬레이션 결과')
  print('-' * 68)
  print(f'  경로            : {len(wps)}점 / {path_len:.1f}m '
        f'({"닫힌 루프" if r["closed"] else "열린 경로"})')
  print(f'  설정            : 목표속도 {args.speed} m/s, k_ld {args.k_ld}, '
        f'lookahead {args.min_ld}~{args.max_ld}m, 최대타각 {args.max_steer_deg}°')
  print(f'  초기 오차       : 횡 {args.init_offset:+.2f}m, '
        f'헤딩 {args.init_heading_err:+.1f}°')
  print('-' * 68)
  print(f'  종료            : {r["reason"]}')
  print(f'  주행거리        : {r["travelled"]:.1f}m / {path_len:.1f}m '
        f'({r["travelled"] / path_len * 100:.0f}%)')
  if len(cte) == 0:
    print('  (샘플 없음)')
    print('=' * 68)
    return False
  print('-' * 68)
  print(f'  횡방향 오차     : 평균 {cte.mean():.3f}m  RMS '
        f'{math.sqrt((cte ** 2).mean()):.3f}m  최대 {cte.max():.3f}m')
  # 초기 수렴 구간(앞 5m)을 뺀 정상상태 오차
  ss = cte[r['s'] > 5.0]
  if len(ss):
    print(f'  정상상태 오차   : 평균 {ss.mean():.3f}m  RMS '
          f'{math.sqrt((ss ** 2).mean()):.3f}m  최대 {ss.max():.3f}m')
  sat = np.abs(r['delta']) >= args.max_steer_deg - 0.1
  print(f'  조향 포화       : {sat.sum()}/{len(sat)} 스텝 '
        f'({sat.mean() * 100:.1f}%)  최대타각 {np.abs(r["delta"]).max():.1f}°')
  print('-' * 68)
  worst = float(ss.max()) if len(ss) else float(cte.max())
  ok = r['goal'] and worst < 0.5 and sat.mean() < 0.05
  if ok:
    print('  ✅ 추종 양호 — 완주, 정상상태 최대오차 50cm 미만, 조향 포화 거의 없음')
  else:
    if not r['goal']:
      print('  ❌ 완주 실패')
    if worst >= 0.5:
      i = int(np.argmax(cte * (r['s'] > 5.0)))
      print(f'  ⚠ 최대오차 {worst:.2f}m — 경로 {r["s"][i]:.0f}m 지점 '
            f'({r["x"][i]:.1f}, {r["y"][i]:.1f})')
    if sat.mean() >= 0.05:
      print(f'  ⚠ 조향 포화 {sat.mean() * 100:.0f}% — 타각이 모자란 코너가 있다')
  print('=' * 68)
  return ok


def plot(wps, r, path):
  import matplotlib
  matplotlib.use('Agg')
  from matplotlib import font_manager as fm
  import matplotlib.pyplot as plt
  for f in fm.findSystemFonts():
    if 'NotoSansCJK' in f:
      fm.fontManager.addfont(f)
      matplotlib.rcParams['font.family'] = fm.FontProperties(fname=f).get_name()
      break
  matplotlib.rcParams['axes.unicode_minus'] = False

  fig = plt.figure(figsize=(15, 9))
  ax = fig.add_subplot(1, 2, 1)
  ax.plot(wps[:, 0], wps[:, 1], '-', color='0.7', lw=3, label='기록 경로')
  sc = ax.scatter(r['x'], r['y'], c=r['cte'], cmap='inferno_r', s=6,
                  vmin=0, vmax=max(0.3, float(r['cte'].max())), zorder=3)
  plt.colorbar(sc, ax=ax, label='횡방향 오차 [m]')
  ax.plot(wps[0, 0], wps[0, 1], 'o', ms=13, color='#00b050', label='시작')
  ax.plot(wps[-1, 0], wps[-1, 1], 's', ms=11, color='#d00000', label='정지지점')
  ax.set_aspect('equal'); ax.grid(alpha=0.3, ls=':')
  ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
  ax.set_title('주행 궤적 (색 = 횡방향 오차)')
  ax.legend(fontsize=9)

  ax2 = fig.add_subplot(2, 2, 2)
  ax2.plot(r['s'], r['cte'], lw=1.2, color='#c00')
  ax2.axhline(0.5, ls='--', color='0.5', lw=1)
  ax2.text(0.5, 0.52, '허용 50cm', fontsize=8, color='0.4')
  ax2.set_ylabel('횡방향 오차 [m]'); ax2.grid(alpha=0.3, ls=':')
  ax2.set_title('경로 진행거리별 추종 오차')

  ax3 = fig.add_subplot(2, 2, 4)
  ax3.plot(r['s'], r['delta'], lw=1.2, color='#06c', label='조향각')
  m = r['max_steer_deg']
  ax3.axhline(m, ls='--', color='r', lw=1); ax3.axhline(-m, ls='--', color='r', lw=1)
  ax3.plot(r['s'], r['v'] * 20, lw=1.0, color='#0a0', alpha=0.7,
           label='속도 ×20 [m/s]')
  ax3.set_xlabel('진행거리 [m]'); ax3.set_ylabel('조향각 [°]')
  ax3.grid(alpha=0.3, ls=':'); ax3.legend(fontsize=8)
  ax3.set_title(f'조향각 (빨간선 = 한계 ±{m:.0f}°) / 속도')

  plt.tight_layout(); plt.savefig(path, dpi=115)
  print(f'  그래프 저장: {path}')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('waypoints', nargs='?', default=DEFAULT_WP)
  ap.add_argument('--speed', type=float, default=1.0)
  ap.add_argument('--min-speed', type=float, default=0.4)
  ap.add_argument('--curv-gain', type=float, default=2.0)
  ap.add_argument('--k-ld', type=float, default=0.6)
  ap.add_argument('--min-ld', type=float, default=1.0)
  ap.add_argument('--max-ld', type=float, default=4.0)
  ap.add_argument('--wheelbase', type=float, default=0.785)
  ap.add_argument('--max-steer-deg', type=float, default=18.0)
  ap.add_argument('--max-steer-rate-deg', type=float, default=90.0)
  ap.add_argument('--n-back', type=int, default=5)
  ap.add_argument('--n-forward', type=int, default=20)
  ap.add_argument('--rate', type=float, default=20.0)
  ap.add_argument('--max-time', type=float, default=900.0)
  ap.add_argument('--init-offset', type=float, default=0.0,
                  help='출발 시 횡방향으로 어긋난 거리[m] — 수렴성 확인용')
  ap.add_argument('--steer-bias', type=float, default=0.0,
                  help='조향 캘리브 오차[도] — 중심이 틀어진 경우')
  ap.add_argument('--steer-scale', type=float, default=1.0,
                  help='조향 스케일 오차 — COUNTS_PER_DEG 어긋남 (1.0=정확)')
  ap.add_argument('--init-heading-err', type=float, default=0.0,
                  help='출발 시 헤딩 오차[도]')
  ap.add_argument('--plot', default=None)
  args = ap.parse_args()

  wps = load_waypoints(args.waypoints)
  r = simulate(wps, args)
  ok = report(wps, r, args)
  if args.plot and len(r['cte']):
    plot(wps, r, args.plot)
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

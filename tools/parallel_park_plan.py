#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parallel_park_plan.py — 평행주차 **끊어 넣기** 궤적을 만든다.

★ 왜 새로 만드나
  규정(항목 8) 평행주차 공간은 차량규격의 **1.5배 = 2.10m** 인데, 우리 차는
  최소회전반경이 전장보다 크다(R 2.42m / 전장 1.40m = 1.7배).
  `tools/parking_feasibility.py` 로 확인한 결과 **단일 S자(2원호)로는 약 4.0m**
  가 필요하다 — 규정 공간의 두 배다. 한 번에는 물리적으로 못 들어간다.

  대신 사람이 좁은 자리에 넣듯 **짧게 여러 번 왕복**하면 한 사이클마다
  d²/R 만큼 횡으로 민다(d = 칸 안 종방향 여유). 2.10m 면 사이클당 0.203m,
  약 5사이클이면 들어간다는 계산이 나온다. 이 도구가 그 궤적을 실제로 만든다.

  현재 `parking_planner.plan_parking` 은 '후진 원호 1개 + 직선'(cusp 2개)
  구조라 이런 다중 왕복을 **구조적으로 못 만든다.**

★ 방식 — 탐욕 다중 스트로크
  매 스트로크마다 (전진/후진) × (좌타/우타/직진) 조합을 모두 시뮬해서,
  **차체가 닿기 직전까지** 간 뒤 목표 자세에 가장 가까워지는 것을 고른다.
  일반 플래너(하이브리드 A*)보다 단순하지만, 평행주차는 기동 구조가 정해져
  있어서 이걸로 충분하다. 그리고 실제 실행과 모양이 같다 —
  **각 스트로크는 '닿기 직전까지 후진/전진' 이라 후방 라이다로 끊을 수 있다.**

★ 부호 (2026-09-12 확인)
  자전거 모델 dθ = (v/L)·tanδ, 좌조향 δ>0.
  칸이 −y(오른쪽)일 때 **후진+우타** 가 칸으로 파고든다
  (v<0, tanδ<0 → dθ>0: 코가 좌, 꼬리가 우=칸 쪽).

★ 좌표
  칸      x ∈ [0, S], y ∈ [−D, 0]
  앞차/콘 x < 0,  뒤차/콘 x > S,  연석 y < −D,  차선 y > 0
  차는 차선에서 **앞차 옆**에 서서 시작한다(실제 주차와 같은 자세).

사용:
  python3 tools/parallel_park_plan.py                    # 규정 2.10m
  python3 tools/parallel_park_plan.py --slot-len 2.5 --plot /tmp/pp.png
  python3 tools/parallel_park_plan.py --self-test        # 4.0m 회귀검사
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from parking_feasibility import body_corners, seg_rect_clear, wrapdeg  # noqa


class Scene:
  def __init__(self, slot_len, slot_depth, veh, lane_w=3.0):
    self.S, self.D = slot_len, slot_depth
    self.l_r, self.l_f, self.half_w = veh
    self.L_v = self.l_r + self.l_f
    big = 30.0
    self.rects = [
        (-big, -slot_depth, 0.0, 0.0),        # 앞차/콘
        (slot_len, -slot_depth, big, 0.0),    # 뒤차/콘
        (-big, -big, big, -slot_depth),       # 연석
        (-big, lane_w, big, big),             # 반대편 차선 경계
    ]

  def corners(self, p):
    return body_corners(p[0], p[1], p[2], self.l_r, self.l_f, self.half_w)

  def clear(self, p):
    return seg_rect_clear(self.corners(p), self.rects)

  def inside(self, p, tol=1e-3):
    c = self.corners(p)
    return (min(q[0] for q in c) >= -tol and max(q[0] for q in c) <= self.S + tol
            and min(q[1] for q in c) >= -self.D - tol
            and max(q[1] for q in c) <= tol)

  def goal(self):
    gx = self.S / 2.0 - (self.L_v / 2.0 - self.l_r)
    return (gx, -self.D / 2.0, 0.0)


def step(p, v, delta, L, ds):
  x, y, th = p
  return (x + v * ds * math.cos(th),
          y + v * ds * math.sin(th),
          th + v * ds * math.tan(delta) / L)


def stroke(scene, p, v, delta, L, ds=0.005, margin=0.03, max_len=6.0):
  """차체가 닿기 직전까지 간다. (끝자세, 이동거리, 궤적)"""
  pts = [p]
  moved = 0.0
  cur = p
  while moved < max_len:
    nxt = step(cur, v, delta, L, ds)
    if scene.clear(nxt) < margin:
      break
    cur = nxt
    moved += ds
    pts.append(cur)
  return cur, moved, pts


def cost(scene, p):
  """목표 자세까지의 비용."""
  g = scene.goal()
  return math.hypot(p[0] - g[0], p[1] - g[1]) + 0.03 * abs(wrapdeg(p[2] - g[2]))


def plan(scene, L, max_strokes=16, steer_deg=18.0, ds=0.02, margin=0.03,
         beam=40, fracs=(0.34, 0.67, 1.0)):
  """스트로크 열을 **빔 탐색**으로 찾는다.

  ★ 왜 탐욕이 아니라 탐색인가 (2026-09-12)
    평행주차의 첫 수(후진+우타)는 차를 목표에서 **멀어지게** 만든다. 코가 차선
    쪽으로 나가기 때문이다. 그래서 '매 스트로크마다 비용이 줄어야 한다' 는
    탐욕 규칙으로는 첫 수부터 막힌다 — 단일 S자로 되는 4.0m 조차 못 풀었다.
    빔 탐색은 일시적으로 나빠지는 수를 허용하므로 이 구조를 넘는다.

  각 스트로크는 (전진/후진) × (좌타/직진/우타) 6가지이고, **닿기 직전까지**
  간 길이의 1/3, 2/3, 전부 세 지점에서 끊을 수 있다. 실제 실행과 같은 모양이라
  (닿기 직전까지 가서 기어 바꾸기) 후방 라이다로 스트로크를 끊을 수 있다.
  """
  d = math.radians(steer_deg)
  actions = [(v, dd) for v in (-1.0, +1.0) for dd in (-d, 0.0, +d)]
  start = (-0.2, scene.half_w + 0.25, 0.0)
  if scene.clear(start) < margin:
    return None, [], '시작 자세가 이미 막혔다'

  def key(p):
    return (round(p[0] / 0.05), round(p[1] / 0.05),
            round(math.degrees(p[2]) / 5.0))

  frontier = [(cost(scene, start), start, [], [start])]
  seen = {key(start)}
  best = None
  for depth in range(max_strokes):
    nxt = []
    for _, p, strokes, traj in frontier:
      for v, delta in actions:
        q, moved, pts = stroke(scene, p, v, delta, L, ds, margin)
        if moved < ds * 3:
          continue
        for fr in fracs:
          n = max(2, int(len(pts) * fr))
          qq = pts[n - 1]
          k = key(qq)
          if k in seen:
            continue
          seen.add(k)
          st2 = strokes + [dict(dir='후진' if v < 0 else '전진',
                                steer=round(math.degrees(delta)),
                                length=round(moved * fr, 3))]
          tr2 = traj + pts[1:n]
          if scene.inside(qq) and abs(wrapdeg(qq[2])) < 8.0:
            return st2, tr2, None
          c = cost(scene, qq)
          nxt.append((c, qq, st2, tr2))
          if best is None or c < best[0]:
            best = (c, qq, st2, tr2)
    if not nxt:
      break
    nxt.sort(key=lambda t: t[0])
    frontier = nxt[:beam]
  if best:
    return None, best[3], (f'{max_strokes}스트로크 안에 못 넣음 '
                           f'(최선 비용 {best[0]:.3f})')
  return None, [start], '첫 스트로크부터 막혔다'


def report(scene, strokes, traj, err, veh, args):
  print('=' * 72)
  print(f'평행주차 끊어 넣기 — 칸 {scene.S:.2f}m × {scene.D:.2f}m '
        f'(전장 {scene.L_v:.2f}m 의 {scene.S / scene.L_v:.2f}배)')
  print('=' * 72)
  if err:
    print(f'  ❌ 실패 — {err}')
    if traj:
      p = traj[-1]
      print(f'     최종 자세 x={p[0]:+.2f} y={p[1]:+.2f} '
            f'θ={math.degrees(p[2]):+.1f}°  여유 {scene.clear(p):.3f}m')
    print('=' * 72)
    return False
  fwd = sum(s['length'] for s in strokes if s['dir'] == '전진')
  rev = sum(s['length'] for s in strokes if s['dir'] == '후진')
  cusps = sum(1 for i in range(1, len(strokes))
              if strokes[i]['dir'] != strokes[i - 1]['dir'])
  t = (fwd + rev) / args.speed + cusps * args.cusp_dwell
  p = traj[-1]
  print(f'  ✅ 들어간다 — 스트로크 {len(strokes)}개 · 기어전환 {cusps}회')
  print(f'     주행 전진 {fwd:.2f}m / 후진 {rev:.2f}m · '
        f'예상 {t:.0f}초 ({args.speed} m/s, 전환 {args.cusp_dwell}s)')
  print(f'     최종 자세 x={p[0]:+.3f} y={p[1]:+.3f} '
        f'θ={math.degrees(p[2]):+.2f}° · 최소여유 '
        f'{min(scene.clear(q) for q in traj):.3f}m')
  print('-' * 72)
  for i, s in enumerate(strokes, 1):
    print(f'    {i:2d}. {s["dir"]} {s["steer"]:+3d}° {s["length"]:5.2f}m')
  print('=' * 72)
  return True


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--veh-len', type=float, default=1.40)
  ap.add_argument('--veh-width', type=float, default=0.775)
  ap.add_argument('--wheelbase', type=float, default=0.785)
  ap.add_argument('--max-steer-deg', type=float, default=18.0)
  ap.add_argument('--slot-len', type=float, default=None)
  ap.add_argument('--slot-depth', type=float, default=1.10)
  ap.add_argument('--margin', type=float, default=0.03)
  ap.add_argument('--speed', type=float, default=0.25)
  ap.add_argument('--cusp-dwell', type=float, default=1.0)
  ap.add_argument('--max-strokes', type=int, default=16)
  ap.add_argument('--beam', type=int, default=40)
  ap.add_argument('--plot', default=None)
  ap.add_argument('--self-test', action='store_true',
                  help='4.0m(단일 S자로 되는 칸)에서 반드시 성공해야 한다')
  a = ap.parse_args()

  over = a.veh_len - a.wheelbase
  l_r = over / 2.0
  veh = (l_r, a.veh_len - l_r, a.veh_width / 2.0)

  if a.self_test:
    ok_all = True
    for S in (4.0, 3.0, 2.6, 2.3, 2.10):
      sc = Scene(S, a.slot_depth, veh)
      st, tr, err = plan(sc, a.wheelbase, a.max_strokes, a.max_steer_deg,
                         margin=a.margin, beam=a.beam)
      mark = '✅' if st else '❌'
      n = len(st) if st else 0
      print(f'  {mark} 칸 {S:.2f}m ({S / a.veh_len:.2f}배) — '
            + (f'스트로크 {n}개' if st else f'{err}'))
      if S >= 4.0 and not st:
        ok_all = False
    print()
    print('  회귀 기준: 4.0m 는 단일 S자로 되는 칸이라 **반드시** 성공해야 한다.')
    return 0 if ok_all else 1

  S = a.slot_len if a.slot_len else a.veh_len * 1.5
  sc = Scene(S, a.slot_depth, veh)
  st, tr, err = plan(sc, a.wheelbase, a.max_strokes, a.max_steer_deg,
                     margin=a.margin, beam=a.beam)
  ok = report(sc, st, tr, err, veh, a)

  if a.plot and tr:
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
    from matplotlib.patches import Polygon, Rectangle
    fig, ax = plt.subplots(figsize=(11, 6))
    for (x0, y0, x1, y1) in sc.rects[:3]:
      ax.add_patch(Rectangle((max(x0, -3), max(y0, -3)),
                             min(x1, S + 3) - max(x0, -3),
                             min(y1, 3) - max(y0, -3),
                             color='0.75'))
    ax.add_patch(Rectangle((0, -sc.D), S, sc.D, fill=False,
                           ec='tab:blue', lw=2, ls='--', label='주차칸'))
    for q in tr[::max(1, len(tr) // 60)]:
      ax.add_patch(Polygon(sc.corners(q), fill=False, ec='tab:orange',
                           alpha=0.35, lw=0.8))
    ax.add_patch(Polygon(sc.corners(tr[-1]), fill=False, ec='tab:red', lw=2))
    ax.plot([q[0] for q in tr], [q[1] for q in tr], '-', color='tab:green',
            lw=1.2, label='뒷축 궤적')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    ax.set_title(f'평행주차 {S:.2f}m — '
                 + (f'스트로크 {len(st)}개' if st else '실패'))
    plt.tight_layout(); plt.savefig(a.plot, dpi=110)
    print(f'  그림: {a.plot}')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

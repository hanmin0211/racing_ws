#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""course_map.py — 코스를 **미션을 둘 수 있는 곳 기준으로** 한 장에 그린다.

★ 왜 (2026-09-17)
  용인 계획은 현장에서 짜야 하는데, 어디가 직선이고 어디가 급커브인지 숫자표로는
  안 잡힌다. 잘못 잡으면 증상이 '조용한 탈락' 이다 — 커브에서 회피가 arm 되면
  먹스가 경로조향을 통째로 버려 그 자체가 이탈이다(실제로 벽에 박았다).

  위: 지도. 경로를 **회피 arm 가능 / 경계 / 금지** 로 칠한다.
  아래: 진행거리 s 에 따른 곡률반경 R. 문턱선과 쓸 수 있는 구간을 띠로.

  --plan 을 주면 계획의 미션 구간을 겹쳐 그린다 — 구간이 급커브에 걸쳤는지
  눈으로 바로 보인다.

사용:
  python3 tools/course_map.py config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml
  python3 tools/course_map.py <wp.yaml> --plan config/mission_plan.yaml
  python3 tools/course_map.py <wp.yaml> --out /tmp/course.png

판정 문턱은 tools/mission_plan_check.py 와 **같은 값**을 쓴다(거기서 가져온다).
"""

import argparse
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                       # noqa: E402
import numpy as np                                    # noqa: E402
import yaml                                           # noqa: E402
from matplotlib.collections import LineCollection     # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_plan_check import (ARM_R_FAIL, ARM_R_WARN,   # noqa: E402
                                curvature_radius)

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHEELBASE, MAX_STEER_DEG = 0.785, 18.0
# 물리 한계: R = 축거 / tan(최대타각). 이보다 급하면 제어가 완벽해도 못 돈다.
R_PHYSICAL = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))
R_TIGHT = R_PHYSICAL * 1.45          # preflight.py 의 '빠듯한 커브' 기준

# 색 — 검증된 값이다. `scripts/validate_palette.js` 와 같은 검사를 돌려
#   초록/빨강 조합이 **색각 이상에서 구분 불가**(deutan ΔE 4.1 < 6)임을 확인하고
#   파랑/호박/빨강으로 바꿨다(최소 ΔE 19.8). 색만으로 뜻을 나르지 않는다 —
#   범례에 글자를 달고, 아래 패널에서 위치로도 갈린다.
C_OK, C_EDGE, C_BAD = '#2a78d6', '#eda100', '#d03b3b'
C_INK, C_MUTED, C_GRID = '#0b0b0b', '#52514e', '#d8d7d2'
C_SURF = '#fcfcfb'
C_MISSION = '#4a3aa7'


def radii_adjacent(P):
  """연속 **3점**(기선 1.0m) 외접원 반경. preflight.py · smooth_path.py 와 같은 정의.

  ⚠ 이 파일의 다른 곡률(`curvature_radius`, i±2 · 기선 2.0m)과 **다른 수치가
    나온다**. 둘은 다른 질문의 답이다:
      · 인접 3점 (여기)  — "차가 이 점을 물리적으로 돌 수 있나"
                          급한 곳을 안 깎으므로 **보수적**이다.
      · i±2 (mission_plan_check) — "경로조향을 2m 버리면 얼마나 벗어나나"
                          이탈식 d²/(2R) 의 d=2m 과 기선이 맞는다.
    용인 코스 실측: 인접 3점 2.71m vs i±2 2.96m. 섞어 쓰면 안 된다.
  """
  out = np.full(len(P), 999.0)
  for i in range(1, len(P) - 1):
    a = np.hypot(*(P[i] - P[i - 1]))
    b = np.hypot(*(P[i + 1] - P[i]))
    c = np.hypot(*(P[i + 1] - P[i - 1]))
    t = (a + b + c) / 2.0
    ar2 = t * (t - a) * (t - b) * (t - c)
    if ar2 > 1e-12:
      out[i] = (a * b * c) / (4.0 * math.sqrt(ar2))
  return out


def classify(R):
  """0=arm 가능 · 1=경계 · 2=금지."""
  return np.where(R >= ARM_R_WARN, 0, np.where(R >= ARM_R_FAIL, 1, 2))


def runs_of(mask, s, min_len=0.0):
  out, i = [], 0
  n = len(mask)
  while i < n:
    if mask[i]:
      j = i
      while j + 1 < n and mask[j + 1]:
        j += 1
      if s[j] - s[i] >= min_len:
        out.append((i, j))
      i = j + 1
    else:
      i += 1
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('wp')
  ap.add_argument('--plan', default=None, help='미션 계획 — 구간을 겹쳐 그린다')
  ap.add_argument('--mark', action='append', default=[], metavar='이름=s0:s1',
                  help='구간에 이름표를 붙인다. 예: --mark "S자=176:250"')
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  wp = a.wp if os.path.isabs(a.wp) else os.path.join(WS, a.wp)
  P = np.array([[p['x'], p['y']] for p in yaml.safe_load(open(wp))['waypoints']])
  s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
  R = curvature_radius(P)
  cls = classify(R)
  L = s[-1]

  ok_mask = cls == 0
  usable = sum(s[j] - s[i] for i, j in runs_of(ok_mask, s, 6.8))
  sharp = runs_of(cls == 2, s, 1.0)
  # 물리 한계 판정은 **인접 3점**(보수적)으로 한다 — preflight 와 같은 수치가
  # 나와야 두 도구가 서로 다른 말을 하지 않는다.
  Ra = radii_adjacent(P)
  i_min = int(np.argmin(Ra[1:-1])) + 1
  need = math.degrees(math.atan(WHEELBASE / Ra[i_min]))

  missions = []
  if a.plan:
    pp = a.plan if os.path.isabs(a.plan) else os.path.join(WS, a.plan)
    d = yaml.safe_load(open(pp)) or {}
    for m in (d.get('missions') or []):
      t = m.get('trigger') or {}
      if t.get('s_enter') is None or t.get('s_exit') is None:
        continue
      if float(t['s_exit']) <= float(t['s_enter']):
        continue
      missions.append((m.get('name', '?'), float(t['s_enter']),
                       float(t['s_exit']), bool(m.get('enabled', True))))

  fig = plt.figure(figsize=(14, 11), facecolor=C_SURF)
  gs = fig.add_gridspec(2, 1, height_ratios=[2.1, 1], hspace=0.22,
                        left=0.07, right=0.97, top=0.90, bottom=0.115)

  # ── 위: 지도 ─────────────────────────────────────────────────────────
  ax = fig.add_subplot(gs[0], facecolor=C_SURF)
  seg = np.stack([P[:-1], P[1:]], axis=1)
  cols = [(C_OK, C_EDGE, C_BAD)[c] for c in cls[:-1]]
  ax.add_collection(LineCollection(seg, colors=cols, linewidths=3.4,
                                   capstyle='round', zorder=2))
  ax.plot(P[0, 0], P[0, 1], 'o', ms=11, mfc=C_SURF, mec=C_INK, mew=2, zorder=5)
  ax.annotate('출발 s0', P[0], textcoords='offset points', xytext=(10, 10),
              fontsize=11, color=C_INK, weight='bold')
  ax.plot(P[-1, 0], P[-1, 1], 's', ms=10, mfc=C_SURF, mec=C_INK, mew=2, zorder=5)
  ax.annotate(f'종점 s{L:.0f}', P[-1], textcoords='offset points',
              xytext=(10, -16), fontsize=11, color=C_INK, weight='bold')
  # 가장 급한 커브 — 여기가 경로 전체의 한계다
  ax.plot(*P[i_min], 'o', ms=15, mfc='none', mec=C_BAD, mew=2.4, zorder=6)
  ax.annotate(f'최소 R={Ra[i_min]:.2f}m  (s{s[i_min]:.0f}, 인접3점)\n'
              f'필요타각 {need:.1f}° / 상한 18°',
              P[i_min], textcoords='offset points', xytext=(16, 12),
              fontsize=11, color=C_BAD, weight='bold',
              bbox=dict(boxstyle='round,pad=0.4', fc=C_SURF, ec=C_BAD, lw=1.2))
  # 50m 마다 s 눈금 — 현장에서 위치를 짚을 수 있어야 한다
  for mark in range(0, int(L) + 1, 50):
    k = int(np.argmin(np.abs(s - mark)))
    ax.plot(*P[k], '|', ms=9, color=C_MUTED, mew=1.6, zorder=4)
    ax.annotate(f'{mark}', P[k], textcoords='offset points', xytext=(4, -13),
                fontsize=8.5, color=C_MUTED)
  if missions:
    for name, e, x, en in missions:
      k0, k1 = int(np.argmin(np.abs(s - e))), int(np.argmin(np.abs(s - x)))
      ax.plot(P[k0:k1 + 1, 0], P[k0:k1 + 1, 1], lw=9, alpha=0.30,
              color=C_MISSION, solid_capstyle='round', zorder=1)
      mid = (k0 + k1) // 2
      ax.annotate(f'{name}\ns{e:.0f}~{x:.0f}' + ('' if en else ' (비활성)'),
                  P[mid], textcoords='offset points', xytext=(0, 20),
                  ha='center', fontsize=10, color=C_MISSION, weight='bold')
  # 사용자가 지정한 구간 이름표 — 현장 코스 구조를 지도 위에서 확인한다
  MARKC = ['#4a3aa7', '#008300', '#d03b3b', '#0b0b0b', '#eb6834']
  marks = []
  for n, spec in enumerate(a.mark):
    nm, rng = spec.split('=', 1)
    e, x = (float(z) for z in rng.split(':'))
    k0, k1 = int(np.argmin(np.abs(s - e))), int(np.argmin(np.abs(s - x)))
    col = MARKC[n % len(MARKC)]
    ax.plot(P[k0:k1 + 1, 0], P[k0:k1 + 1, 1], lw=11, alpha=0.26, color=col,
            solid_capstyle='round', zorder=1)
    mid = (k0 + k1) // 2
    ax.annotate(f'{nm}\ns{e:.0f}~{x:.0f}', P[mid], textcoords='offset points',
                xytext=(0, 24), ha='center', fontsize=11, color=col,
                weight='bold', zorder=7,
                bbox=dict(boxstyle='round,pad=0.3', fc=C_SURF, ec=col, lw=1.1,
                          alpha=0.95))
    marks.append((nm, e, x, col))

  ax.set_aspect('equal')
  ax.set_xlabel('x [m]', color=C_MUTED, fontsize=10)
  ax.set_ylabel('y [m]', color=C_MUTED, fontsize=10)
  ax.grid(True, color=C_GRID, lw=0.6, alpha=0.7)
  ax.set_axisbelow(True)
  for sp in ax.spines.values():
    sp.set_color(C_GRID)
  ax.tick_params(colors=C_MUTED, labelsize=9)
  ax.autoscale_view()

  handles = [
      plt.Line2D([], [], color=C_OK, lw=3.4,
                 label=f'회피 arm 가능   R ≥ {ARM_R_WARN:.0f}m'),
      plt.Line2D([], [], color=C_EDGE, lw=3.4,
                 label=f'경계   {ARM_R_FAIL:.0f} ≤ R < {ARM_R_WARN:.0f}m'),
      plt.Line2D([], [], color=C_BAD, lw=3.4,
                 label=f'미션 금지   R < {ARM_R_FAIL:.0f}m'),
  ]
  if missions:
    handles.append(plt.Line2D([], [], color=C_MISSION, lw=9, alpha=0.30,
                              label='계획된 미션 구간'))
  ax.legend(handles=handles, loc='upper left', fontsize=10, framealpha=0.95,
            facecolor=C_SURF, edgecolor=C_GRID)

  # ── 아래: s 에 따른 곡률반경 ──────────────────────────────────────────
  bx = fig.add_subplot(gs[1], facecolor=C_SURF)
  for i, j in runs_of(ok_mask, s, 6.8):
    bx.axvspan(s[i], s[j], color=C_OK, alpha=0.13, lw=0, zorder=0)
  Rp = np.clip(R, None, 60)
  bx.plot(s, Rp, lw=1.6, color=C_MUTED, zorder=3)
  for name, lvl, col, ls in (
      (f'arm 권장 {ARM_R_WARN:.0f}m', ARM_R_WARN, C_OK, '-'),
      (f'미션 금지 {ARM_R_FAIL:.0f}m', ARM_R_FAIL, C_EDGE, '-'),
      (f'빠듯 {R_TIGHT:.2f}m', R_TIGHT, C_MUTED, ':'),
      (f'물리 한계 {R_PHYSICAL:.2f}m (타각 18°)', R_PHYSICAL, C_BAD, '--')):
    bx.axhline(lvl, color=col, lw=1.4, ls=ls, zorder=2)
    bx.annotate(name, (L * 0.004, lvl), fontsize=9, color=col, ha='left',
                va='center', weight='bold', zorder=6,
                bbox=dict(boxstyle='round,pad=0.22', fc=C_SURF, ec='none',
                          alpha=0.92))
  for i, j in sharp:
    bx.plot([s[i], s[j]], [R[i:j + 1].min()] * 2, lw=3.2, color=C_BAD,
            solid_capstyle='round', zorder=4)
  bx.plot(s[i_min], Ra[i_min], 'o', ms=8, mfc=C_SURF, mec=C_BAD, mew=2, zorder=5)
  if missions:
    for name, e, x, en in missions:
      bx.axvspan(e, x, color=C_MISSION, alpha=0.18, lw=0, zorder=1)
  for nm, e, x, col in marks:
    bx.axvspan(e, x, color=col, alpha=0.16, lw=0, zorder=1)
    bx.annotate(nm, ((e + x) / 2, 66), fontsize=9.5, color=col, ha='center',
                weight='bold', zorder=6)
  bx.set_yscale('log')
  bx.set_ylim(1.9, 78)
  bx.set_xlim(0, L)
  bx.set_yticks([2, 3, 5, 8, 11, 15, 20, 30, 60])
  bx.set_yticklabels(['2', '3', '5', '8', '11', '15', '20', '30', '60+'])
  bx.set_xlabel('진행거리 s [m]', color=C_MUTED, fontsize=10)
  bx.set_ylabel('곡률반경 R [m]  (로그)', color=C_MUTED, fontsize=10)
  bx.grid(True, color=C_GRID, lw=0.6, alpha=0.7)
  bx.set_axisbelow(True)
  for sp in bx.spines.values():
    sp.set_color(C_GRID)
  bx.tick_params(colors=C_MUTED, labelsize=9)
  fig.text(0.07, 0.030,
           f'파란 띠 = 회피 arm 가능 구간 — R ≥ {ARM_R_WARN:.0f}m 이 6.8m 이상 '
           f'이어지는 곳 (6.8 = 감지 트리거 4.0 + BLOCKED 반경 2.8).  '
           f'굵은 빨간 선분 = 급커브 {len(sharp)}곳의 최저점.',
           fontsize=9.5, color=C_MUTED)
  fig.text(0.07, 0.008,
           '곡선 R 은 i±2 (기선 2.0m) — 이탈식 d²/(2R) 의 d=2m 과 맞춘 정의. '
           '빨간 동그라미(최소 R)만 인접 3점(기선 1.0m, 보수적)이라 '
           'preflight.py 와 같은 값이 나온다. 두 수치를 섞지 말 것.',
           fontsize=9, color=C_MUTED)

  # ── 제목 ─────────────────────────────────────────────────────────────
  fig.text(0.07, 0.965, os.path.basename(wp), fontsize=17, weight='bold',
           color=C_INK)
  fig.text(0.07, 0.932,
           f'{L:.0f} m · {len(P)}점     '
           f'회피 arm 가능 {usable:.0f}m ({100 * usable / L:.0f}%)     '
           f'급커브(R<{ARM_R_FAIL:.0f}m) {len(sharp)}곳     '
           f'최소 R={Ra[i_min]:.2f}m → 필요타각 {need:.1f}° / 상한 {MAX_STEER_DEG:.0f}°',
           fontsize=11.5, color=C_MUTED)

  out = a.out or os.path.join(
      '/tmp', os.path.basename(wp).replace('.yaml', '_map.png'))
  fig.savefig(out, dpi=125, facecolor=C_SURF)
  print(f'저장: {out}')
  print(f'  {L:.1f}m · arm 가능 {usable:.0f}m ({100 * usable / L:.0f}%) · '
        f'급커브 {len(sharp)}곳 · 최소 R={Ra[i_min]:.2f}m 인접3점 '
        f'(필요타각 {need:.1f}°)')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_plot.py — drive_record.py 가 쌓은 CSV 를 한 장의 지도로 그린다.

★ 무엇을 보려는 것인가 (2026-09-14)
  "회피는 하는데 의자 사이로 안 간다" 를 눈으로 확인하기 위해서다.
  계획 경로 · 실제 궤적 · 라이다가 본 것(지도 좌표) · 회피가 켜진 구간을
  한 장에 겹쳐 놓으면, 차가 문 사이로 갔는지 바깥으로 돌았는지가 보인다.

  라이다 점을 지도 좌표로 쌓는 것이 핵심이다. 차량 기준으로 두면 같은
  물체가 프레임마다 다른 자리에 찍혀 아무것도 안 보인다.

사용:
  python3 tools/drive_plot.py /tmp/drive_0130.csv
  python3 tools/drive_plot.py /tmp/drive_0130.csv --wp config/.../wp.yaml
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

DEF_WP = '/home/han/racing_ws/config/chungju_school/wp_school_track_0.5.yaml'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--wp', default=DEF_WP)
  ap.add_argument('--out', default=None)
  ap.add_argument('--zoom', nargs=4, type=float, metavar=('X0', 'X1', 'Y0', 'Y1'),
                  help='관심 영역만 확대')
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  if not rows:
    print('빈 CSV'); return 1
  X = np.array([float(r['x']) for r in rows])
  Y = np.array([float(r['y']) for r in rows])
  MODE = [r['mode'] for r in rows]
  V = np.array([float(r['v'] or 0) for r in rows])

  sp = os.path.splitext(a.csv)[0] + '_scan.csv'
  SX = SY = None
  if os.path.exists(sp):
    s = list(csv.DictReader(open(sp)))
    SX = np.array([float(r['x']) for r in s])
    SY = np.array([float(r['y']) for r in s])

  wp = yaml.safe_load(open(a.wp))
  wp = wp['waypoints'] if isinstance(wp, dict) and 'waypoints' in wp else wp
  WX = [p['x'] for p in wp]; WY = [p['y'] for p in wp]

  fig, ax = plt.subplots(figsize=(12, 9))
  if SX is not None and len(SX):
    ax.plot(SX, SY, '.', ms=1.2, color='#b9c2cb', alpha=.5, zorder=1,
            label=f'라이다가 본 것 ({len(SX)}점, 지도좌표)')
  ax.plot(WX, WY, '-', color='#8e99a4', lw=2.5, zorder=2, label='계획 경로')

  # 실제 궤적 — 회피 중인 구간만 색을 바꾼다
  av = np.array([m.startswith('AVOID') for m in MODE])
  ax.plot(X, Y, '-', color='#2a6fb0', lw=2.6, zorder=3, label='실제 궤적')
  if av.any():
    ax.plot(X[av], Y[av], '.', ms=7, color='#d9372b', zorder=4,
            label=f'회피(AVOID) 중 {int(av.sum())}샘플')

  ax.plot(X[0], Y[0], '^', ms=14, color='#2e9e4f', mec='white', mew=2, zorder=6)
  ax.plot(X[-1], Y[-1], '*', ms=20, color='#111', mec='white', mew=1.5, zorder=6)

  # 정지한 지점
  st = (np.abs(V) < 0.05)
  if st.any():
    ax.plot(X[st], Y[st], 'x', ms=6, color='#e8833a', zorder=5,
            label='정지(v<0.05)')

  ax.set_aspect('equal'); ax.grid(alpha=.25, ls=':')
  ax.set_xlabel('로컬 x [m]'); ax.set_ylabel('로컬 y [m]')
  ax.set_title(f'주행 기록 — {os.path.basename(a.csv)}', fontsize=13,
               fontweight='bold')
  ax.legend(loc='best', fontsize=9, framealpha=.95)
  if a.zoom:
    ax.set_xlim(a.zoom[0], a.zoom[1]); ax.set_ylim(a.zoom[2], a.zoom[3])
  out = a.out or (os.path.splitext(a.csv)[0] + '.png')
  plt.tight_layout(); plt.savefig(out, dpi=130)
  print(f'저장: {out}')

  # 숫자 요약
  d = float(np.sum(np.hypot(np.diff(X), np.diff(Y))))
  print(f'  주행거리 {d:.1f}m · 샘플 {len(X)} · 회피 {int(av.sum())}샘플')
  return 0


if __name__ == '__main__':
  sys.exit(main())

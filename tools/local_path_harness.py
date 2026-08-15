#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""local_path_harness.py — 로컬 경로 생성 로직 오프라인 검증 하네스.

ROS도 차량도 없이, 기록된 웨이포인트 위를 차가 주행한다고 가정하고
**노드가 실제로 쓰는 코드(local_path_core)를 그대로 import** 해서 품질을 검사한다.
야외 세션을 쓰지 않고 회귀를 잡을 수 있다.

  python3 tools/local_path_harness.py [웨이포인트.yaml] [--legacy]

  --legacy : 개정 전 방식(y=f(x) + 순환 인덱싱)으로 검사해 개선 전후를 비교
"""

import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'src', 'waypoint_follower'))
from waypoint_follower.local_path_core import (  # noqa: E402
    LocalPathParams, build_local_path, is_closed_path)

DEFAULT_WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
              'waypoints_recorded_resampled_0.5.yaml')


def load_waypoints(path):
  with open(path, 'r') as f:
    data = yaml.safe_load(f)
  raw = data.get('waypoints', data.get('poses', list(data.values())[0]))
  pts = []
  for p in raw:
    if isinstance(p, dict):
      pts.append([float(p['x']), float(p['y'])])
    else:
      pts.append([float(p[0]), float(p[1])])
  return np.array(pts)


def legacy_build(wps, cx, cy, cyaw, p):
  """개정 전 방식(y=f(x) + 순환 인덱싱) — 비교용."""
  total = len(wps)
  d = np.hypot(wps[:, 0] - cx, wps[:, 1] - cy)
  ci = int(np.argmin(d))
  win = min(p.n_back + p.n_forward + 1, total)
  idx = [(ci - p.n_back + i) % total for i in range(win)]
  w = wps[idx]
  seg = np.hypot(np.diff(w[:, 0]), np.diff(w[:, 1]))
  jump = float(seg.max()) if len(seg) else 0.0
  dx, dy = w[:, 0] - cx, w[:, 1] - cy
  c, s = math.cos(cyaw), math.sin(cyaw)
  xl, yl = dx * c + dy * s, -dx * s + dy * c
  mono = bool(np.all(np.diff(xl) > 0))
  order = np.argsort(xl)
  xs, ys = xl[order], yl[order]
  keep = np.concatenate(([True], np.diff(xs) > 1e-6))
  xf, yf = xs[keep], ys[keep]
  if len(xf) <= p.poly_order:
    return None, 0.0, ci, jump, mono
  poly = np.poly1d(np.polyfit(xf, yf, p.poly_order))
  fp0 = float(np.polyder(poly)(0.0))
  fpp0 = float(np.polyder(poly, 2)(0.0))
  curv = fpp0 / (1.0 + fp0 * fp0) ** 1.5
  gx = np.arange(0.0, 200.0, 0.02)
  ds = np.sqrt(1.0 + np.polyder(poly)(gx) ** 2) * 0.02
  sa = np.concatenate(([0.0], np.cumsum(ds)[:-1]))
  n = int(round(p.lookahead_distance / p.point_spacing))
  tg = np.linspace(0.0, p.lookahead_distance, n + 1)
  tg = tg[tg <= sa[-1]]
  xa = np.interp(tg, sa, gx)
  return ([(float(x), float(poly(x))) for x in xa], curv, ci, jump, mono)


def true_curvature(wps, i):
  n = len(wps)
  if i <= 0 or i >= n - 1:
    return 0.0
  (x1, y1), (x2, y2), (x3, y3) = wps[i - 1], wps[i], wps[i + 1]
  cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
  a = math.hypot(x2 - x1, y2 - y1)
  b = math.hypot(x3 - x2, y3 - y2)
  c = math.hypot(x3 - x1, y3 - y1)
  area = abs(cross) / 2.0
  if area < 1e-9:
    return 0.0
  return (1.0 / (a * b * c / (4 * area))) * (1.0 if cross > 0 else -1.0)


def main():
  args = [a for a in sys.argv[1:] if not a.startswith('--')]
  legacy = '--legacy' in sys.argv
  wp_file = args[0] if args else DEFAULT_WP

  wps = load_waypoints(wp_file)
  n = len(wps)
  gap = math.hypot(wps[0, 0] - wps[-1, 0], wps[0, 1] - wps[-1, 1])
  closed = is_closed_path(wps)
  p = LocalPathParams()

  print('웨이포인트 %d점 | 시작-끝 %.2fm | 판정: %s | 모드: %s'
        % (n, gap, '닫힌 루프' if closed else '열린 경로',
           '개정 전(legacy)' if legacy else '개정 후'))
  print('-' * 62)

  prev_idx = None
  prev_pts = None
  jumps, breaks, seam, discont, fails, goals = [], [], [], [], [], []
  curv_err = []

  for i in range(n):
    cx, cy = wps[i]
    nxt = wps[(i + 1) % n]
    cyaw = math.atan2(nxt[1] - cy, nxt[0] - cx)

    if legacy:
      pts, curv, ci, jump, mono = legacy_build(wps, cx, cy, cyaw, p)
      ok = pts is not None
    else:
      r = build_local_path(wps, cx, cy, cyaw, p, prev_idx=prev_idx)
      pts, curv, ci, ok = r['points'], r['curvature'], r['closest'], r['ok']
      if r['goal_reached']:
        goals.append(i)
        prev_idx = ci
        continue
      w = wps[window_idx(wps, r, p, closed)]
      segs = np.hypot(np.diff(w[:, 0]), np.diff(w[:, 1]))
      jump = float(segs.max()) if len(segs) else 0.0
      dxx, dyy = w[:, 0] - cx, w[:, 1] - cy
      cc, ss = math.cos(cyaw), math.sin(cyaw)
      mono = bool(np.all(np.diff(dxx * cc + dyy * ss) > 0))

    if prev_idx is not None:
      d = (ci - prev_idx) % n
      if 5 < d < n - 5:
        jumps.append((i, prev_idx, ci))
    prev_idx = ci

    if not mono:
      breaks.append(i)
    if jump > 2.0:
      seam.append((i, jump))
    if not ok:
      fails.append(i)

    if pts is not None:
      a = np.array(pts)
      if prev_pts is not None and len(prev_pts) == len(a):
        delta = float(np.max(np.hypot(a[:, 0] - prev_pts[:, 0],
                                      a[:, 1] - prev_pts[:, 1])))
        if delta > 3.0:
          discont.append((i, delta))
      prev_pts = a

    curv_err.append(abs(curv - true_curvature(wps, i)))

  print('[1] 최근접 인덱스 점프      : %s'
        % ('★ %d회' % len(jumps) if jumps else '없음'))
  print('[2] 피팅 전제 붕괴(x 비단조): %s'
        % ('★ %d/%d (%.0f%%)' % (len(breaks), n, 100.0 * len(breaks) / n)
           if breaks else '없음 (호길이 방식은 영향 없음)'))
  print('[3] 윈도우 이음매 점프      : %s'
        % ('★ %d회 (최대 %.2fm)' % (len(seam), max(v for _, v in seam))
           if seam else '없음'))
  print('[4] 로컬경로 프레임간 급변  : %s'
        % ('★ %d회' % len(discont) if discont else '없음'))
  print('[5] 경로 생성 실패          : %s'
        % ('★ %d회' % len(fails) if fails else '없음'))
  print('[5b] 완주 판정(정상 동작)   : %s'
        % ('%d지점 (경로 끝 %.1fm 구간)' % (len(goals), len(goals) * 0.5)
           if goals else '없음'))
  ce = np.array(curv_err)
  print('[6] 곡률 오차               : 평균 %.3f / 최대 %.3f 1/m'
        % (ce.mean(), ce.max()))
  print('-' * 62)


def window_idx(wps, r, p, closed):
  """검사용: 코어가 사용한 윈도우 인덱스를 재구성."""
  from waypoint_follower.local_path_core import window_indices
  return window_indices(len(wps), r['closest'], p, closed)


if __name__ == '__main__':
  main()

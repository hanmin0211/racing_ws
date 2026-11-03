#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_dataset.py — 기존 주행 로그로 '카메라 없는' 예비 학습 데이터를 만든다.

★ 왜
  VLA 에 측위를 넣으면 좋아지는지 보려면 카메라 데이터가 필요한데 아직 없다.
  대신 2026-09-17 교내 루프 주행 7회에 라이다(지도좌표 점)·위치·조향 명령이
  같은 시각으로 남아 있다. 이걸로 '라이다만' vs '라이다+측위' 를 먼저 비교한다.

★ 샘플 = 라이다 스캔 한 장 (약 4 Hz)
  label   steer = cmd_steer_deg (|값| ≤ 18° — 0020 에 −711° 같은 깨진 행이 있다)
          v     = cmd_v. 주행 구간(처음 출발 ~ 마지막 주행) 안의 정지 순간도 넣는다.
          조향 실험은 train.py 가 v > 0.1 만 골라 예전과 같은 샘플을 쓴다.
  blocked 장애물 때문에 멈춘 순간(mode BLOCKED) 표시 — 속도 실험 분석용
  hist    시간 맥락 — 과거 0.25 s 간격 8칸(2 s):
          hist_f{k}  k×0.25 s 전 스캔의 앞쪽(±20°) 최소 거리 (지금 스캔은 lidar 열에 있다)
          hist_v{k}  k×0.25 s 전 측정 속도 v (k=0 은 지금). 명령(cmd_v)이 아니라 측정값 —
                     직전 명령을 넣으면 모델이 베끼기만 해도 점수가 난다.
  ⚠ 2026-09-17 정지는 장애물·돌발정지 시험 때문이다. 정밀지도 정지선에서 5 m 안에서
    멈춘 적이 없다(실측) — 이 데이터로는 지도 정지선이 속도에 주는 효과를 못 본다.
  lidar   차량 기준 36 구간(10°씩) 최소거리, 빈 구간은 6 m (기록기가 6 m 에서 잘랐다)
  loc     경로 대비 횡오프셋·방향오차 + 1~5 m 앞 경로점(차량 기준) 10개 = 12개
  loc_nX  같은 것을 위치에 σ=X m 잡음을 넣은 자세로 계산 (측위 품질 실험용).
          샘플마다 독립인 백색 잡음이다.
  loc_gX  RTK FLOAT 처럼 '한쪽으로 몇 초씩 밀리는' 오차. 1차 Gauss-Markov
          (시간상수 TAU 초, 정상상태 σ=X m). 0.5 m 는 학습에 안 쓰는 시험 전용.
  두 잡음 모두 위치만 흔든다(방향은 그대로). 라이다는 실제 자세로 되돌리므로 무관하다.
  map     정밀지도 특징 8개 — 측위 자세에서 계산하므로 map_nX / map_gX 잡음판도 같이 만든다.
          좌·우 차선(B2 501/502/503/505/506/515)까지, 좌·우 연석·가드레일(C3)까지,
          앞 정지선(B2 530)·횡단보도(B3 5321)까지 거리, 차로 중심선(A2) 대비 오프셋·방향차.
          지도 폴더는 $HDMAP_DIR. 지도와 RTK 정합은 아직 현장 미검증(지도가 맞다고 가정).

★ 경로 투영은 직전 s 근처만 찾는다. 루프가 나갔다 돌아오는 구조라 전역
  최근접을 쓰면 반대편 차로로 튄다.

사용 (표준 라이브러리만 — 학습 PC 에서도 그대로 돈다):
  python3 research/vla_pilot/make_dataset.py            # → research/vla_pilot/dataset.csv
"""

import bisect
import csv
import glob
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(WS, 'tools'))
from hdmap_common import cumlen, load_layer, load_waypoints, map_dir, seg_cross  # noqa: E402

PATH = os.path.join(WS, 'config/chungju_school/wp_school_track_raw.yaml')
RUNS = sorted(f for f in glob.glob(os.path.join(WS, 'data/2026-09-17/drive_*.csv'))
              if not f.endswith('_scan.csv'))
NBIN, RMAX = 36, 6.0
PREVIEW = (1.0, 2.0, 3.0, 4.0, 5.0)
NOISE = (0.1, 0.3, 1.0)
GM = (0.1, 0.3, 0.5, 1.0)
TAU = 10.0
STEER_MAX = 18.0
LANE_KINDS = {'501', '502', '503', '505', '506', '515'}
NMAP = 8
HIST = [0.25 * k for k in range(1, 9)]
FRONT = slice(16, 20)   # lidar 구간 16~19 = 차량 기준 −20°~+20°


def wrap(a):
  return (a + math.pi) % (2 * math.pi) - math.pi


class Track:
  """경로 위 진행거리 s 를 이어서 추적한다."""

  def __init__(self, pts):
    self.p = pts
    self.s = cumlen(pts)
    self.L = self.s[-1]
    self.closed = math.dist(pts[0], pts[-1]) < 2.0

  def proj(self, q, s_prev=None, win=6.0):
    """→ (s, 부호있는 횡오프셋[+좌], 경로 방향). s_prev 가 있으면 ±win 안에서만."""
    best = None
    for i in range(len(self.p) - 1):
      if s_prev is not None:
        d = abs(self.s[i] - s_prev)
        if self.closed:
          d = min(d, self.L - d)
        if d > win:
          continue
      (ax, ay), (bx, by) = self.p[i], self.p[i + 1]
      vx, vy = bx - ax, by - ay
      L2 = vx * vx + vy * vy
      if L2 < 1e-12:
        continue
      t = max(0.0, min(1.0, ((q[0] - ax) * vx + (q[1] - ay) * vy) / L2))
      dd = math.hypot(q[0] - ax - vx * t, q[1] - ay - vy * t)
      if best is None or dd < best[0]:
        side = vx * (q[1] - ay) - vy * (q[0] - ax)
        best = (dd, self.s[i] + math.sqrt(L2) * t, math.copysign(dd, side),
                math.atan2(vy, vx))
    if s_prev is not None:
      # 창 안 결과가 전역보다 1 m 넘게 나쁘면 다시 잡는다 — 이 경로는 끝이 앞부분과
      # 겹치는 열린 경로라, 끝을 지나 앞으로 넘어가면 창에 갇힌다(실측 off 22 m).
      g = self.proj(q)
      if best is None or abs(best[2]) > abs(g[1]) + 1.0:
        return g
    return best[1], best[2], best[3]

  def at(self, s):
    if self.closed:
      s %= self.L
    s = max(0.0, min(self.L, s))
    for i in range(len(self.s) - 1):
      if self.s[i] <= s <= self.s[i + 1]:
        t = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        return (self.p[i][0] + (self.p[i + 1][0] - self.p[i][0]) * t,
                self.p[i][1] + (self.p[i + 1][1] - self.p[i][1]) * t)
    return self.p[-1]


def loc_feats(tr, x, y, yaw, s_prev):
  s, off, hdg = tr.proj((x, y), s_prev)
  out = [off, math.degrees(wrap(yaw - hdg))]
  c, si = math.cos(yaw), math.sin(yaw)
  for d in PREVIEW:
    px, py = tr.at(s + d)
    dx, dy = px - x, py - y
    out += [c * dx + si * dy, -si * dx + c * dy]
  return s, out


class HdMap:
  """경로 주변(±30 m) 정밀지도 선분만 들고 있다가 차량 자세에서 특징을 낸다. 좌표는 로컬."""

  def __init__(self, d, ox, oy, box):
    def segs(layer, keep, close=False):
      out = []
      for r, parts in load_layer(d, layer):
        if not keep(r):
          continue
        for part in parts:
          p = [(x - ox, y - oy) for x, y in part]
          if close and p[0] != p[-1]:
            p.append(p[0])
          # 꼭짓점이 아니라 경계 상자로 거른다 — 긴 직선은 양 끝이 영역 밖이어도 영역을 지난다
          xs, ys = [x for x, _ in p], [y for _, y in p]
          if min(xs) < box[1] and max(xs) > box[0] and min(ys) < box[3] and max(ys) > box[2]:
            out += list(zip(p, p[1:]))
      return out
    self.lane = segs('B2_SURFACELINEMARK', lambda r: r['Kind'] in LANE_KINDS)
    self.stop = segs('B2_SURFACELINEMARK', lambda r: r['Kind'] == '530')
    self.curb = segs('C3_VEHICLEPROTECTIONSAFETY', lambda r: True)
    self.xwalk = segs('B3_SURFACEMARK', lambda r: r['Kind'] == '5321', close=True)
    self.link = segs('A2_LINK', lambda r: True)
    assert self.lane and self.link, '경로 주변에 정밀지도 차선·링크가 없다 — 원점·지도 폴더 확인'

  @staticmethod
  def ray(x, y, ang, L, segs):
    """(x,y) 에서 ang 방향 L m 광선이 처음 만나는 선분까지 거리. 없으면 L.
    shortcut: 앞 정지선·횡단보도도 직선 광선이라 커브 안쪽 것은 놓친다, 필요하면 경로 따라 탐색으로."""
    q = (x + L * math.cos(ang), y + L * math.sin(ang))
    ts = [t for a, b in segs if (t := seg_cross((x, y), q, a, b)) is not None]
    return min(ts) * L if ts else L

  def feats(self, x, y, yaw):
    l90, r90 = yaw + math.pi / 2, yaw - math.pi / 2
    out = [self.ray(x, y, l90, 6.0, self.lane), self.ray(x, y, r90, 6.0, self.lane),
           self.ray(x, y, l90, 8.0, self.curb), self.ray(x, y, r90, 8.0, self.curb),
           self.ray(x, y, yaw, 20.0, self.stop), self.ray(x, y, yaw, 20.0, self.xwalk)]
    # 진행 방향과 90° 안쪽인 차로 중심선 중 가장 가까운 것 (반대 차로를 집지 않게)
    best = None
    for (ax, ay), (bx, by) in self.link:
      vx, vy = bx - ax, by - ay
      L2 = vx * vx + vy * vy
      hd = wrap(math.atan2(vy, vx) - yaw)
      if L2 < 1e-12 or abs(hd) > math.pi / 2:
        continue
      t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
      dd = math.hypot(x - ax - vx * t, y - ay - vy * t)
      if best is None or dd < best[0]:
        best = (dd, math.copysign(dd, vx * (y - ay) - vy * (x - ax)), math.degrees(hd))
    return out + ([best[1], best[2]] if best else [6.0, 0.0])


def lidar_bins(pts, x, y, yaw):
  b = [RMAX] * NBIN
  c, si = math.cos(yaw), math.sin(yaw)
  for px, py in pts:
    dx, dy = px - x, py - y
    vx, vy = c * dx + si * dy, -si * dx + c * dy
    r = math.hypot(vx, vy)
    k = int((math.degrees(math.atan2(vy, vx)) + 180.0) // (360.0 / NBIN)) % NBIN
    b[k] = min(b[k], r)
  return b


def main():
  pts, org = load_waypoints(PATH)
  tr = Track(pts)
  xs, ys = [p[0] for p in pts], [p[1] for p in pts]
  hd = HdMap(map_dir(None), org[0], org[1],
             (min(xs) - 30, max(xs) + 30, min(ys) - 30, max(ys) + 30))
  print(f'정밀지도: 차선 {len(hd.lane)} · 정지선 {len(hd.stop)} · 연석 {len(hd.curb)} · '
        f'횡단보도 {len(hd.xwalk)} · 링크 {len(hd.link)} 선분')
  rng = random.Random(0)
  head = (['run', 't', 'steer', 'v', 'avoid_active', 'blocked']
          + [f'lidar{i}' for i in range(NBIN)]
          + [f'loc{i}' for i in range(2 + 2 * len(PREVIEW))]
          + [f'loc_n{n}_{i}' for n in NOISE for i in range(2 + 2 * len(PREVIEW))]
          + [f'loc_g{n}_{i}' for n in GM for i in range(2 + 2 * len(PREVIEW))]
          + [f'map{i}' for i in range(NMAP)]
          + [f'map_n{n}_{i}' for n in NOISE for i in range(NMAP)]
          + [f'map_g{n}_{i}' for n in GM for i in range(NMAP)]
          + [f'hist_f{k}' for k in range(1, len(HIST) + 1)]
          + [f'hist_v{k}' for k in range(len(HIST) + 1)])
  out_path = os.path.join(HERE, 'dataset.csv')
  n_total = 0
  with open(out_path, 'w', newline='', encoding='utf-8') as fo:
    w = csv.writer(fo)
    w.writerow(head)
    for run in RUNS:
      rows = list(csv.DictReader(open(run, encoding='utf-8')))
      scans = {}
      for t, x, y in list(csv.reader(open(run.replace('.csv', '_scan.csv'), encoding='utf-8')))[1:]:
        scans.setdefault(t, []).append((float(x), float(y)))
      # 20 Hz 전 행에서 s 를 이어 추적 → 스캔 시각에서만 샘플을 낸다
      moving = [i for i, r in enumerate(rows) if float(r['cmd_v'] or 0) > 0.1]
      first, last = (moving[0], moving[-1]) if moving else (1, 0)
      s_prev, s_noisy, by_t = None, {}, {}
      gm = {n: [rng.gauss(0, n), rng.gauss(0, n)] for n in GM}  # 정상상태에서 시작
      t_prev = None
      for i, r in enumerate(rows):
        x, y, yaw = float(r['x']), float(r['y']), math.radians(float(r['yaw_deg']))
        t = float(r['t'])
        a = math.exp(-max(0.0, min(0.5, t - t_prev)) / TAU) if t_prev is not None else 1.0
        t_prev = t
        s_prev, loc = loc_feats(tr, x, y, yaw, s_prev)
        noisy, poses = [], [(x, y)]   # poses: 깨끗 → 백색 σ들 → FLOAT형 σ들 (열 순서와 같다)
        for n in NOISE:
          q = (x + rng.gauss(0, n), y + rng.gauss(0, n))
          s_noisy['n', n], f = loc_feats(tr, q[0], q[1], yaw, s_noisy.get(('n', n)))
          noisy += f
          poses.append(q)
        for n in GM:
          e = gm[n]
          e[0] = a * e[0] + math.sqrt(1 - a * a) * rng.gauss(0, n)
          e[1] = a * e[1] + math.sqrt(1 - a * a) * rng.gauss(0, n)
          s_noisy['g', n], f = loc_feats(tr, x + e[0], y + e[1], yaw, s_noisy.get(('g', n)))
          noisy += f
          poses.append((x + e[0], y + e[1]))
        by_t[r['t']] = (r, loc, noisy, poses, first <= i <= last)
      # 스캔 t 와 같은 문자열이 없으면 가장 가까운 행 (실측 차이 ≤ 0.02 s)
      near_key = lambda t: t if t in by_t else min(by_t, key=lambda k: abs(float(k) - float(t)))
      bins = {}
      for t, sp in scans.items():   # 과거 칸을 위해 모든 스캔의 구간값을 먼저 만든다
        r = by_t[near_key(t)][0]
        bins[t] = lidar_bins(sp, float(r['x']), float(r['y']), math.radians(float(r['yaw_deg'])))
      scan_t = sorted(bins, key=float)
      scan_tf = [float(t) for t in scan_t]
      row_t = [float(r['t']) for r in rows]
      row_v = [float(r['v'] or 0) for r in rows]
      v_at = lambda t: row_v[max(0, bisect.bisect_right(row_t, t) - 1)]

      def f_at(t):
        i = bisect.bisect_right(scan_tf, t) - 1
        return min(bins[scan_t[i]][FRONT]) if i >= 0 else RMAX

      keep = 0
      for t in scans:
        r, loc, noisy, poses, driving = by_t[near_key(t)]
        if not r['cmd_steer_deg'] or not r['cmd_v'] or not driving:
          continue
        steer, v = float(r['cmd_steer_deg']), float(r['cmd_v'])
        if abs(steer) > STEER_MAX:
          continue
        mode = r['mode']
        avoid = int(mode.startswith('AVOID') and '조향OFF' not in mode)
        tf = float(t)
        w.writerow([os.path.basename(run), t, steer, v, avoid, int(mode.startswith('BLOCKED'))]
                   + [round(b, 3) for b in bins[t]]
                   + [round(f, 4) for f in loc + noisy]
                   + [round(f, 3) for q in poses
                      for f in hd.feats(q[0], q[1], math.radians(float(r['yaw_deg'])))]
                   + [round(f_at(tf - h - 1e-6), 3) for h in HIST]
                   + [round(v_at(tf - h), 3) for h in [0.0] + HIST])
        keep += 1
      n_total += keep
      print(f'{os.path.basename(run)}: 스캔 {len(scans)} → 샘플 {keep}')
  print(f'총 {n_total} 샘플 → {out_path}')


if __name__ == '__main__':
  main()

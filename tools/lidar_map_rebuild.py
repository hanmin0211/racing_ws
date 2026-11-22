#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_map_rebuild.py — bag 의 원본 스캔으로 라이다 지도를 **다시** 만든다.

★ 왜 (2026-10-12)
  drive_record 의 drive_scan.csv 는 '차 위치 + 차 헤딩' 으로 그 자리에서 지도
  좌표로 바꿔 저장한다. 그날 IMU yaw 가 분당 −21° 씩 떠내려가서(heading_offset_plot)
  벽이 부채꼴로 번져 쓸 수 없었다.
  bag 에는 원본(/scan_front · /handsfree/imu · /odometry/filtered)이 있으니
  헤딩을 **GPS 진행방향으로 시간에 따라 보정**한 뒤 다시 투영한다.

방법
  ① 헤딩 보정  c(t) = GPS 진행방향 − IMU 원시 yaw  를 직진·주행 구간에서 뽑아
     8초 이동중앙값으로 매끄럽게 → yaw(t) = IMU(t) + c(t).
     (IMU 는 짧은 시간 회전은 정확하고 천천히 떠내려간다 → 느린 성분만 GPS 로 잡는다)
  ② 라이다 장착각·장착위치는 모른다 → 지도가 **가장 선명해지는**(점유 셀 수 최소)
     값을 격자 탐색으로 고른다.
  ③ 점유 격자(0.10 m)에 쌓고, **서로 다른 스캔 N 개 이상**에서 잡힌 셀만 남긴다.
     한두 번 스친 잡음·움직이는 물체가 빠지고 벽만 남는다.

⚠ 한계: 위치는 GPS 안테나 기준이다. 라이다–안테나 거리는 ② 에서 추정한 값이다.

사용:
  python3 tools/lidar_map_rebuild.py data/bag_1012_2106 \\
      --wp config/chungju_school/wp_heading_1007b_v2.yaml
"""
import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CELL = 0.10


def wrap(a):
  return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_q(q):
  return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path, topics):
  r = rosbag2_py.SequentialReader()
  r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
         rosbag2_py.ConverterOptions('cdr', 'cdr'))
  types = {t.name: t.type for t in r.get_all_topics_and_types()}
  cls = {t: get_message(types[t]) for t in topics if t in types}
  out = {t: [] for t in topics}
  while r.has_next():
    topic, data, ts = r.read_next()
    if topic in cls:
      out[topic].append((ts * 1e-9, deserialize_message(data, cls[topic])))
  return out


def heading_correction(odo_t, odo_xy, imu_t, imu_yaw, win_med=8.0):
  """GPS 진행방향 − IMU yaw 를 시간 함수 c(t) 로. 직진·주행 표본만 쓴다."""
  # 같은 위치 반복 제거 (위치 30 Hz · GPS 10 Hz)
  keep = np.r_[True, np.any(np.diff(odo_xy, axis=0) != 0, axis=1)]
  t, xy = odo_t[keep], odo_xy[keep]
  # 중심차분 (±0.3 s) — 지연 없는 진행방향
  ts, cs = [], []
  for i in range(len(t)):
    a = np.searchsorted(t, t[i] - 0.3); b = np.searchsorted(t, t[i] + 0.3) - 1
    if a < 0 or b >= len(t) or b <= a:
      continue
    d = xy[b] - xy[a]; dt = t[b] - t[a]
    if dt <= 0 or np.hypot(*d) / dt < 0.5:          # 서 있거나 너무 느리면 방향을 못 믿는다
      continue
    ts.append(t[i]); cs.append(math.atan2(d[1], d[0]))
  ts, cs = np.array(ts), np.array(cs)
  iy = np.unwrap(imu_yaw)
  raw = np.interp(ts, imu_t, iy)
  c = wrap(cs - raw)
  # 커브에서는 현 방향(chord)이 접선과 어긋난다 → IMU 회전율이 큰 표본은 뺀다
  rate = np.abs(np.gradient(np.interp(ts, imu_t, iy), ts))
  ok = rate < math.radians(15)
  ts, c = ts[ok], np.unwrap(c[ok])
  # 고립 표본(차를 손으로 옮길 때 등) 제거 — 앞뒤 5초에 10개 미만이면 버린다
  dense = np.array([((ts > x - 5) & (ts < x + 5)).sum() >= 10 for x in ts])
  ts, c = ts[dense], c[dense]
  # 이동중앙값으로 느린 성분만
  cm = np.array([np.median(c[(ts > x - win_med / 2) & (ts < x + win_med / 2)]) for x in ts])
  return ts, c, cm


def project(scans, pose_t, pose_xy, yaw_fn, mount_deg, lever, rmax=12.0, rmin=0.4):
  P = []
  for ts, m in scans:
    if ts < pose_t[0] or ts > pose_t[-1]:
      continue
    x = np.interp(ts, pose_t, pose_xy[:, 0]); y = np.interp(ts, pose_t, pose_xy[:, 1])
    yaw = yaw_fn(ts)
    r = np.asarray(m.ranges, float)
    a = m.angle_min + m.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    lx = x + lever * math.cos(yaw); ly = y + lever * math.sin(yaw)
    th = yaw + a[ok] + math.radians(mount_deg)
    P.append(np.c_[np.full(ok.sum(), ts), lx + r[ok] * np.cos(th), ly + r[ok] * np.sin(th)])
  return np.vstack(P) if P else np.zeros((0, 3))


def grid(P, min_scans):
  """서로 다른 스캔 min_scans 개 이상에서 잡힌 셀. (셀중심 xy, 히트수)"""
  ij = np.floor(P[:, 1:] / CELL).astype(np.int64)
  key = ij[:, 0] * 1_000_003 + ij[:, 1]
  uniq = np.unique(np.c_[key, np.round(P[:, 0] * 100).astype(np.int64)], axis=0)  # 셀×스캔
  k, n = np.unique(uniq[:, 0], return_counts=True)
  sel = n >= min_scans
  kk = k[sel]
  i = np.floor_divide(kk + 500_001 * 1_000_003, 1_000_003) - 500_001
  j = kk - i * 1_000_003
  return np.c_[(i + .5) * CELL, (j + .5) * CELL], n[sel]


def sharpness(P):
  """점유 셀 수 — 같은 점을 덜 번지게 찍을수록 적다."""
  ij = np.floor(P[:, 1:] / CELL).astype(np.int64)
  return len(np.unique(ij[:, 0] * 1_000_003 + ij[:, 1]))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('bag')
  ap.add_argument('--wp', default=None)
  ap.add_argument('--min-scans', type=int, default=6)
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  print('bag 읽는 중...')
  B = read_bag(a.bag, ['/scan_front', '/handsfree/imu', '/odometry/filtered', '/heading/yaw_offset'])
  odo_t = np.array([t for t, _ in B['/odometry/filtered']])
  odo_xy = np.array([(m.pose.pose.position.x, m.pose.pose.position.y) for _, m in B['/odometry/filtered']])
  odo_yaw = np.array([yaw_q(m.pose.pose.orientation) for _, m in B['/odometry/filtered']])
  imu_t = np.array([t for t, _ in B['/handsfree/imu']])
  imu_yaw = np.array([yaw_q(m.orientation) for _, m in B['/handsfree/imu']])
  scans = B['/scan_front']
  print(f'  스캔 {len(scans)} · IMU {len(imu_t)} · 위치 {len(odo_t)}')

  # 주행 구간만 (움직인 위치가 있는 시간)
  keep = np.r_[True, np.any(np.diff(odo_xy, axis=0) != 0, axis=1)]
  pt, pxy = odo_t[keep], odo_xy[keep]

  ts, c, cm = heading_correction(odo_t, odo_xy, imu_t, imu_yaw)
  iy = np.unwrap(imu_yaw)
  def yaw_corr(t):
    return float(np.interp(t, imu_t, iy) + np.interp(t, ts, cm))
  def yaw_odom(t):
    return float(np.interp(t, odo_t, np.unwrap(odo_yaw)))
  print(f'  헤딩 보정량 c(t): {math.degrees(cm[0]):+.1f}° → {math.degrees(cm[-1]):+.1f}° '
        f'(표본 {len(ts)}, 잔차 표준편차 {math.degrees(np.std(wrap(c - cm))):.1f}°)')

  # 주행 중 스캔만 (정지 중 스캔은 같은 자리에 쌓여 히트수만 부풀린다)
  moving_t = ts
  mv = [(t, m) for t, m in scans if np.min(np.abs(moving_t - t)) < 0.3]
  print(f'  주행 중 스캔 {len(mv)}')

  # 장착각·장착위치 탐색 (선명도 최대)
  best = None
  sub = mv[::4]
  for mount in np.arange(170, 191, 2.0):
    for lever in (-0.5, 0.0, 0.5, 1.0):
      s = sharpness(project(sub, pt, pxy, yaw_corr, mount, lever))
      if best is None or s < best[0]:
        best = (s, mount, lever)
  for mount in np.arange(best[1] - 1.5, best[1] + 1.6, 0.5):
    for lever in np.arange(best[2] - 0.4, best[2] + 0.41, 0.2):
      s = sharpness(project(sub, pt, pxy, yaw_corr, mount, lever))
      if s < best[0]:
        best = (s, mount, lever)
  _, mount, lever = best
  print(f'  선명도 최대: 장착각 {mount:.1f}° · 라이다가 안테나보다 앞 {lever:+.1f} m')

  Pc = project(mv, pt, pxy, yaw_corr, mount, lever)
  Po = project(mv, pt, pxy, yaw_odom, 180.0, 0.0)        # 기존 방식(기록기와 같다)
  Gc, nc = grid(Pc, a.min_scans)
  Go, no = grid(Po, a.min_scans)
  print(f'  점유 셀: 기존 {sharpness(Po):,} → 보정 {sharpness(Pc):,}  '
        f'({100 * (1 - sharpness(Pc) / sharpness(Po)):.0f}% 덜 번짐)')
  print(f'  {a.min_scans}회 이상 잡힌 셀: 기존 {len(Go):,} · 보정 {len(Gc):,}')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  wp = None
  if a.wp:
    import yaml
    wp = np.array([(p['x'], p['y']) for p in yaml.safe_load(open(a.wp))['waypoints']])
  fig = plt.figure(figsize=(19, 13))
  gs = fig.add_gridspec(2, 2, height_ratios=[1, .38], hspace=.25)
  lim = None
  for k, (G, n, title) in enumerate(((Go, no, '기존 (기록 당시 IMU 헤딩 그대로)'),
                                     (Gc, nc, f'보정 (GPS 진행방향으로 헤딩 보정 · {a.min_scans}회 이상 잡힌 곳만)'))):
    ax = fig.add_subplot(gs[0, k])
    ax.scatter(G[:, 0], G[:, 1], c=np.minimum(n, 40), cmap='Greys', vmin=0, vmax=40, s=1.2, marker='s')
    if wp is not None:
      ax.plot(wp[:, 0], wp[:, 1], color='#dc2626', lw=1.6, label='경로')
    ax.plot(pxy[:, 0], pxy[:, 1], color='#2563eb', lw=.8, alpha=.7, label='실주행 (GPS)')
    if lim is None:
      cx, cy = pxy[:, 0], pxy[:, 1]
      lim = (cx.min() - 8, cx.max() + 8, cy.min() - 8, cy.max() + 8)
    ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
    ax.set_aspect('equal'); ax.grid(alpha=.25); ax.legend(loc='lower left', fontsize=10)
    ax.set_title(title, fontsize=13, weight='bold')
  ax = fig.add_subplot(gs[1, :])
  ax.plot(ts - ts[0], np.degrees(wrap(c)), '.', color='#94a3b8', ms=2, label='GPS 진행방향 − IMU yaw (표본)')
  ax.plot(ts - ts[0], np.degrees(wrap(cm)), color='#7c3aed', lw=2, label='보정량 c(t) (8초 이동중앙값)')
  ax.set_xlabel('t [s]'); ax.set_ylabel('[°]'); ax.grid(alpha=.3); ax.legend(fontsize=10)
  ax.set_title('IMU 헤딩 보정량 — 기울기가 곧 IMU 드리프트', fontsize=12, weight='bold')
  name = os.path.basename(os.path.normpath(a.bag))
  fig.suptitle(f'라이다 지도 재구성 — {name}', fontsize=16, weight='bold')
  out = a.out or f'/home/han/racing_ws/logs/lidar_map_{name}.png'
  fig.savefig(out, dpi=110, bbox_inches='tight'); print(f'✅ {out}')
  np.savetxt(os.path.splitext(out)[0] + '_cells.csv', np.c_[Gc, nc], delimiter=',',
             header='x,y,hits', comments='', fmt=['%.2f', '%.2f', '%d'])
  return 0


if __name__ == '__main__':
  sys.exit(main())

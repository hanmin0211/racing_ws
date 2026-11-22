#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_timeseries.py — bag 의 라이다를 **시계열**로 푼다 (스캔마다 한 행).

★ 왜 (2026-10-12)
  지도 좌표 점구름은 헤딩 오차에 번지고, '언제 어디서 벽에 가까웠나' 를 읽기 어렵다.
  차 기준 거리는 헤딩과 무관하다 — 스캔마다 앞·왼쪽·오른쪽 최근접 거리를 뽑아
  시간축에 놓으면 벽 접근·회피 개입·충돌 위험을 바로 본다.

  섹터(차 기준, 장착각 보정 후):  앞 ±15° · 왼쪽 60~120° · 오른쪽 −120~−60°
  0.4 m 미만은 차체로 보고 버린다.

출력
  logs/lidar_ts_<bag>.csv  : t_auto, x, y, yaw_corr_deg, front_m, left_m, right_m, n_pts
  logs/lidar_ts_<bag>.png  : 시계열 그래프 (+ drive.csv 가 있으면 라이다 회피 개입 표시)

사용:
  python3 tools/lidar_timeseries.py data/bag_1012_2106 \\
      --drive data/2026-10-12/pid_run_2106/drive.csv
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_map_rebuild as L  # noqa: E402

RMIN = 0.4
SECT = {'front': (-15, 15), 'left': (60, 120), 'right': (-120, -60)}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('bag')
  ap.add_argument('--drive', default=None, help='같은 주행의 drive.csv (회피 개입 표시용)')
  ap.add_argument('--mount', type=float, default=179.0, help='장착각 (lidar_map_rebuild 추정값)')
  ap.add_argument('--name', default=None, help='그림 제목에 쓸 주행 이름')
  ap.add_argument('--end-rel', type=float, default=None, help='캘리브 후 이 초까지만 (충돌·고장 이후 버림)')
  ap.add_argument('--pretty', action='store_true', help='한 장짜리 그림 (_pretty.png) — 속도·헤딩 그림과 같은 형식')
  a = ap.parse_args()

  B = L.read_bag(a.bag, ['/scan_front', '/handsfree/imu', '/odometry/filtered', '/heading/yaw_offset'])
  ot = np.array([t for t, _ in B['/odometry/filtered']])
  oxy = np.array([(m.pose.pose.position.x, m.pose.pose.position.y) for _, m in B['/odometry/filtered']])
  it = np.array([t for t, _ in B['/handsfree/imu']])
  iy = np.unwrap(np.array([L.yaw_q(m.orientation) for _, m in B['/handsfree/imu']]))
  ts, c, cm = L.heading_correction(ot, oxy, it, iy)
  t_cal = B['/heading/yaw_offset'][0][0] if B['/heading/yaw_offset'] else ts[0]

  rows = []
  for t, m in B['/scan_front']:
    if t < ts[0] - 1 or t > ts[-1] + 1:
      continue
    r = np.asarray(m.ranges, float)
    ang = np.degrees(L.wrap(m.angle_min + m.angle_increment * np.arange(len(r)) + math.radians(a.mount)))
    ok = np.isfinite(r) & (r > RMIN) & (r < m.range_max)
    d = {}
    for k, (lo, hi) in SECT.items():
      s = ok & (ang >= lo) & (ang <= hi)
      d[k] = float(r[s].min()) if s.any() else float('nan')
    rows.append((t - t_cal, float(np.interp(t, ot, oxy[:, 0])), float(np.interp(t, ot, oxy[:, 1])),
                 math.degrees(L.wrap(np.interp(t, it, iy) + np.interp(t, ts, cm))),
                 d['front'], d['left'], d['right'], int(ok.sum())))
  R = np.array(rows)
  if a.end_rel is not None:
    R = R[R[:, 0] <= a.end_rel]
  name = os.path.basename(os.path.normpath(a.bag))
  out_csv = f'/home/han/racing_ws/logs/lidar_ts_{name}.csv'
  with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t_s', 'x_m', 'y_m', 'heading_deg', 'front_m', 'left_m', 'right_m', 'side_min_m', 'n_pts'])
    for row in R:
      side = np.nanmin(row[5:7]) if not np.all(np.isnan(row[5:7])) else float('nan')
      w.writerow([f'{row[0]:.2f}', f'{row[1]:.3f}', f'{row[2]:.3f}', f'{row[3]:.1f}',
                  *[('' if math.isnan(v) else f'{v:.2f}') for v in (row[4], row[5], row[6], side)],
                  int(row[7])])
  print(f'✅ {out_csv}  ({len(R)} 스캔)')

  # 회피 개입 구간 (drive.csv 의 mode) — yaw_offset 이 처음 찍힌 행을 캘리브 시각으로 맞춘다
  avoid = []
  if a.drive and os.path.exists(a.drive):
    dr = list(csv.DictReader(open(a.drive)))
    t0 = next(float(x['t']) for x in dr if x['yaw_offset_deg'])
    on = None
    for x in dr:
      tt = float(x['t']) - t0
      act = x['mode'].startswith('AVOID') and x['yaw_offset_deg'] and float(x['cmd_v'] or 0) > 0.05
      if act and on is None: on = tt
      if not act and on is not None:
        if tt - on > 0.2: avoid.append((on, tt))
        on = None

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  if a.pretty:
    tt = R[:, 0]; mv = tt >= 0
    def mn(i):
      v = R[mv, i]; v = v[~np.isnan(v)]
      j = np.nanargmin(np.where(mv, R[:, i], np.nan))
      return v.min(), 100 * (v < 1.0).mean(), tt[j]
    (lf, lp, lt), (rf, rp, rt), (ff_, fp, ft) = mn(5), mn(6), mn(4)
    fig, ax0 = plt.subplots(figsize=(14, 4.8))
    for s_, e_ in avoid:
      ax0.axvspan(s_, e_, color='#fde68a', alpha=.45)
    if avoid:
      ax0.plot([], [], color='#fde68a', lw=8, label='라이다 회피 개입')
    def med(v, n=5):                       # 0.5 s 이동중앙값 — 한 스캔짜리 튐만 걷어낸다
      return np.array([np.nanmedian(v[max(0, i - n // 2):i + n // 2 + 1]) if
                       np.any(~np.isnan(v[max(0, i - n // 2):i + n // 2 + 1])) else np.nan for i in range(len(v))])
    ax0.plot(tt, med(R[:, 5]), color='#2563eb', lw=1.8, label='왼쪽 (루프 안쪽)')
    ax0.plot(tt, med(R[:, 6]), color='#dc2626', lw=1.8, label='오른쪽 (루프 바깥쪽)')
    ax0.axhline(1.0, color='#f59e0b', ls='--', lw=1.3, label='1 m')
    ax0.set_ylim(0, 5); ax0.set_xlim(tt[0], tt[-1]); ax0.grid(alpha=.3)
    ax0.set_xlabel('캘리브 완료 후 시간 [s]', fontsize=11); ax0.set_ylabel('옆 물체까지 거리 [m]', fontsize=11)
    ax0.legend(fontsize=10.5, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=4, frameon=False)
    ax0.set_title(f'{a.name or name}\n최근접  왼쪽 {lf:.2f} m · 오른쪽 {rf:.2f} m · 앞 {ff_:.2f} m   '
                  f'(1 m 미만 — 왼쪽 {lp:.1f}% · 오른쪽 {rp:.1f}%)', fontsize=13, weight='bold')
    out = os.path.splitext(out_csv)[0] + '_pretty.png'
    fig.savefig(out, dpi=130, bbox_inches='tight'); print(f'✅ {out}')
    print(f'  요약: 왼쪽 최소 {lf:.2f}m(t={lt:.1f}) 1m미만 {lp:.1f}% · 오른쪽 {rf:.2f}m(t={rt:.1f}) {rp:.1f}% · '
          f'앞 {ff_:.2f}m(t={ft:.1f}) {fp:.1f}% · 회피 {len(avoid)}회 · 스캔 {len(R)}')
    return 0
  fig, ax = plt.subplots(4, 1, figsize=(16, 14), sharex=True, gridspec_kw=dict(height_ratios=[1, 1, 1, .8]))
  t = R[:, 0]
  for k, (i, col, lab) in enumerate(((5, '#2563eb', '왼쪽 (60~120°)'), (6, '#dc2626', '오른쪽 (−120~−60°)'),
                                      (4, '#111827', '앞 (±15°)'))):
    ax[k].plot(t, R[:, i], color=col, lw=1)
    ax[k].axhline(1.0, color='#f59e0b', ls='--', lw=1)
    ax[k].set_ylabel('최근접 거리 [m]'); ax[k].grid(alpha=.3); ax[k].set_ylim(0, 8.5)
    ax[k].set_title(f'{"①②③"[k]} {lab} 최근접 물체', fontsize=12, weight='bold')
    for s, e in avoid:
      ax[k].axvspan(s, e, color='#fde68a', alpha=.6)
  ax[0].plot([], [], color='#fde68a', lw=8, label='라이다 회피 개입 (벽을 장애물로 봄)')
  ax[0].plot([], [], color='#f59e0b', ls='--', label='1 m')
  ax[0].legend(fontsize=9.5, loc='upper right')
  ax[3].plot(ts - t_cal, np.degrees(L.wrap(cm)), color='#7c3aed', lw=2)
  ax[3].set_ylabel('[°]'); ax[3].grid(alpha=.3); ax[3].set_xlabel('캘리브 완료 후 시간 [s]')
  ax[3].set_title('④ IMU 헤딩 보정량 (GPS 진행방향 − IMU) — 기울기 = IMU 드리프트', fontsize=12, weight='bold')
  fig.suptitle(f'라이다 시계열 — {name}', fontsize=15, weight='bold')
  out = os.path.splitext(out_csv)[0] + '.png'
  fig.savefig(out, dpi=110, bbox_inches='tight'); print(f'✅ {out}')

  for k, i in (('왼쪽', 5), ('오른쪽', 6), ('앞', 4)):
    v = R[t > 0, i]; v = v[~np.isnan(v)]
    if len(v):
      j = np.nanargmin(np.where(t > 0, R[:, i], np.nan))
      print(f'  {k:3s} 최근접 {v.min():.2f} m (t={t[j]:.1f}s, ({R[j,1]:.1f},{R[j,2]:.1f})) · 1 m 미만 {100*(v<1).mean():.1f}%')
  return 0


if __name__ == '__main__':
  sys.exit(main())

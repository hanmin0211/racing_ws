#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heading_track_replay.py — 헤딩 연속 보정(HeadingTracker)을 bag 으로 미리 돌려 본다.

direct_localization 이 쓰는 **같은 클래스**에 bag 의 IMU(원시 yaw)와 위치를 시간순으로
넣고, 캘리브 오프셋 하나로 고정했을 때(지금까지)와 비교한다.

평가 기준: 직진·주행 구간에서 '중심차분 GPS 진행방향 − 보정된 헤딩'.
  ⚠ 같은 GPS 로 고치고 같은 GPS 로 채점하므로 '독립 검증' 은 아니다. 다만 고정 오프셋이
    랩 끝에 −28° 까지 벌어지던 것이 어디까지 붙는지는 그대로 보인다.

사용:
  python3 tools/heading_track_replay.py data/bag_1012_2106
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'gps_localization'))
import lidar_map_rebuild as L                          # noqa: E402  (bag 읽기 재사용)
from gps_localization.heading_tracker import HeadingTracker, wrap  # noqa: E402


def run(events, off0, t_cal, **kw):
  """events: 시간순 [(t, 'imu', raw) | (t, 'fix', x, y)]. 오프셋 시계열을 돌려준다."""
  tr = HeadingTracker(**kw)
  out_t, out_o = [], []
  for e in events:
    if e[1] == 'imu':
      tr.add_imu(e[0], e[2])
    else:
      if tr.offset is None and e[0] >= t_cal:
        tr.set_offset(off0)
      tr.add_fix(e[0], e[2], e[3], 0.01)
      if tr.offset is not None:
        out_t.append(e[0]); out_o.append(tr.offset)
  return np.array(out_t), np.unwrap(np.array(out_o)), tr.counts


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('bag')
  ap.add_argument('--tau', type=float, default=3.0)
  a = ap.parse_args()

  B = L.read_bag(a.bag, ['/handsfree/imu', '/odometry/filtered', '/heading/yaw_offset'])
  it = np.array([t for t, _ in B['/handsfree/imu']])
  iraw = np.array([L.yaw_q(m.orientation) for _, m in B['/handsfree/imu']])
  ot = np.array([t for t, _ in B['/odometry/filtered']])
  oxy = np.array([(m.pose.pose.position.x, m.pose.pose.position.y) for _, m in B['/odometry/filtered']])
  keep = np.r_[True, np.any(np.diff(oxy, axis=0) != 0, axis=1)]   # 30 Hz 중 실제 GPS 갱신만
  ft, fxy = ot[keep], oxy[keep]
  t_cal, off0 = B['/heading/yaw_offset'][0][0], B['/heading/yaw_offset'][0][1].data

  ev = [(t, 'imu', r) for t, r in zip(it, iraw)] + [(t, 'fix', x, y) for t, (x, y) in zip(ft, fxy)]
  ev.sort(key=lambda e: e[0])

  # 채점용 기준: 중심차분 GPS 진행방향 (±0.3 s), 직진·주행 표본만
  iy = np.unwrap(iraw)
  gt, gc = [], []
  for i in range(len(ft)):
    j0 = np.searchsorted(ft, ft[i] - 0.3); j1 = np.searchsorted(ft, ft[i] + 0.3) - 1
    if j0 < 0 or j1 >= len(ft) or j1 <= j0 or ft[i] < t_cal:
      continue
    d = fxy[j1] - fxy[j0]; dt = ft[j1] - ft[j0]
    if dt <= 0 or np.hypot(*d) / dt < 0.5:
      continue
    if abs(np.interp(ft[j1], it, iy) - np.interp(ft[j0], it, iy)) / dt > math.radians(15):
      continue
    gt.append(ft[i]); gc.append(math.atan2(d[1], d[0]))
  gt, gc = np.array(gt), np.array(gc)
  raw_g = np.interp(gt, it, iy)

  def score(off_t, off_v):
    o = np.full(len(gt), off0) if off_t is None else np.interp(gt, off_t, off_v)
    e = np.degrees(np.array([wrap(v) for v in gc - (raw_g + o)]))
    slope = np.polyfit(gt - gt[0], e, 1)[0] * 60
    return e, dict(mean=e.mean(), rms=math.sqrt((e ** 2).mean()), p95=np.percentile(np.abs(e), 95),
                   slope=slope)

  e_fix, s_fix = score(None, None)
  print(f'캘리브 오프셋 {math.degrees(off0):.1f}° · 채점 표본 {len(gt)} (직진·주행)\n')
  print(f'{"":22s} {"평균":>7s} {"RMS":>6s} {"95%":>6s} {"기울기":>9s}')
  print(f'{"고정 (지금까지)":22s} {s_fix["mean"]:+6.1f}° {s_fix["rms"]:5.1f}° {s_fix["p95"]:5.1f}° {s_fix["slope"]:+6.1f}°/분')
  res = {}
  for tau in (1.0, 3.0, 5.0, 10.0):
    tt, oo, cnt = run(ev, off0, t_cal, tau=tau)
    e, s = score(tt, oo)
    res[tau] = (tt, oo, e, s, cnt)
    print(f'{"연속 보정 tau " + str(tau) + "s":22s} {s["mean"]:+6.1f}° {s["rms"]:5.1f}° {s["p95"]:5.1f}° {s["slope"]:+6.1f}°/분')
  cnt = res[a.tau][4]
  tot = sum(cnt.values())
  print('\n표본 처리 (tau %.0fs): ' % a.tau + ' · '.join(f'{k} {100 * v / tot:.0f}%' for k, v in cnt.items()))

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig, ax = plt.subplots(2, 1, figsize=(15, 10), sharex=True)
  tt, oo, e, s, _ = res[a.tau]
  x = gt - t_cal
  ax[0].plot(x, e_fix, '.', ms=3, color='#94a3b8',
             label=f'고정 오프셋 (지금까지)  평균 {s_fix["mean"]:+.1f}° · RMS {s_fix["rms"]:.1f}°')
  ax[0].plot(x, e, '.', ms=3, color='#2563eb',
             label=f'연속 보정 tau {a.tau:.0f}s  평균 {s["mean"]:+.1f}° · RMS {s["rms"]:.1f}°')
  ax[0].axhline(0, color='#333', lw=.8); ax[0].axhspan(-2, 2, color='#16a34a', alpha=.12)
  ax[0].set_ylabel('GPS 진행방향 − 헤딩 [°]'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=10)
  ax[0].set_title('① 헤딩 오차 (직진·주행 표본)', fontsize=13, weight='bold')
  ax[1].axhline(math.degrees(off0), color='#94a3b8', lw=2, ls='--', label='고정 오프셋 (캘리브 1회)')
  ax[1].plot(tt - t_cal, np.degrees(oo), color='#2563eb', lw=2, label='연속 보정 오프셋')
  ax[1].set_ylabel('헤딩 오프셋 [°]'); ax[1].set_xlabel('캘리브 완료 후 시간 [s]')
  ax[1].grid(alpha=.3); ax[1].legend(fontsize=10)
  ax[1].set_title('② 오프셋이 IMU 드리프트를 따라간다', fontsize=13, weight='bold')
  name = os.path.basename(os.path.normpath(a.bag))
  fig.suptitle(f'헤딩 연속 보정 재생 — {name}', fontsize=15, weight='bold')
  out = f'/home/han/racing_ws/logs/heading_track_{name}.png'
  fig.savefig(out, dpi=110, bbox_inches='tight'); print(f'\n✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

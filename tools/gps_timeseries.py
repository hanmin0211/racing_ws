#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gps_timeseries.py — GPS 진행방향 기반 헤딩 보정을 **시계열**로 푼다 (논문 GPS 파트).

한 주행(drive.csv)을 GPS 갱신(10 Hz)마다 한 행으로:
  t_s            캘리브 완료 기준 시간 [s]
  x_m, y_m       RTK 위치 (지도 좌표)
  rtk_sigma_cm   수평 σ (10/12 부터 기록. 이전 주행은 빈칸)
  gps_speed      위치 중심차분 속도 [m/s] (±0.25 s, GPS 점프 이어붙임)
  gps_course_deg GPS 진행방향 (중심차분 ±0.3 s) — 느리면 빈칸
  imu_raw_deg    IMU 원시 yaw (오프셋 전)
  offset_fixed   캘리브 1회 오프셋 [°]
  offset_track   연속 보정 오프셋 [°] — 그 주행에서 실제로 적용됐으면 기록값,
                 아니면 HeadingTracker(같은 코드)로 재생한 값
  err_fixed_deg  GPS 진행방향 − (IMU + 고정 오프셋)
  err_track_deg  GPS 진행방향 − (IMU + 연속 보정 오프셋)
  usable         직진·주행 표본(채점 대상)이면 1

사용:
  python3 tools/gps_timeseries.py data/2026-10-12/pid_run_2106/drive.csv
  python3 tools/gps_timeseries.py <csv> --end 143 --name '10/12 PI+헤딩보정'
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'gps_localization'))
from gps_localization.heading_tracker import HeadingTracker  # noqa: E402


def col(rows, k):
  o = []
  for r in rows:
    try:
      o.append(float(r.get(k)))
    except (TypeError, ValueError):
      o.append(np.nan)
  return np.array(o)


def wrapd(a):
  return (np.asarray(a) + 180.0) % 360.0 - 180.0


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--end', type=float, default=None, help='이 시각(t) 이후 버림 (충돌·고장)')
  ap.add_argument('--name', default=None)
  ap.add_argument('--tau', type=float, default=3.0)
  ap.add_argument('--simple', action='store_true',
                  help='GPS 진행방향 vs 차가 쓴 IMU 헤딩 한 장만 (_heading.png)')
  ap.add_argument('--note', action='append', default=[], help='t0,t1,설명 — 음영 (캘리브 기준 시간)')
  ap.add_argument('--error', action='store_true',
                  help='헤딩 오차 한 장만 — 1회 캘리브 vs 연속 보정 (_herr.png)')
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  t, X, Y = col(rows, 't'), col(rows, 'x'), col(rows, 'y')
  yaw, yo, cv = col(rows, 'yaw_deg'), col(rows, 'yaw_offset_deg'), col(rows, 'cmd_v')
  sig, live, raw_col = col(rows, 'rtk_sigma_cm'), col(rows, 'yaw_offset_live_deg'), col(rows, 'imu_yaw_raw_deg')
  i_cal = int(np.argmax(~np.isnan(yo)))
  t_cal, off0 = t[i_cal], yo[i_cal]
  # IMU 원시 yaw — 기록돼 있으면 그대로, 아니면 '차가 쓴 헤딩 − 적용 오프셋' 으로 되돌린다
  # 연속 보정이 '실제로 적용' 됐나 — 차가 쓴 헤딩 = IMU 원시 + 실시간 오프셋 이면 적용된 것.
  #   세 열이 서로 다른 토픽이라 순간적으로 어긋난다 → 최댓값이 아니라 중앙값으로 본다.
  applied_live = False
  if not np.all(np.isnan(live)) and not np.all(np.isnan(raw_col)):
    dd = np.abs(wrapd(yaw - raw_col - live)); dd = dd[~np.isnan(dd)]
    applied_live = len(dd) > 0 and np.median(dd) < 0.5
  if not np.all(np.isnan(raw_col)):
    raw = raw_col
  else:
    raw = wrapd(yaw - np.where(np.isnan(yo), 0.0, yo))

  # GPS 갱신(같은 위치 반복 제외)만 — 점프 이어붙임
  ch = np.r_[True, (np.diff(X) != 0) | (np.diff(Y) != 0)] & ~np.isnan(X)
  idx = np.where(ch)[0]
  ut, ux, uy = t[idx].copy(), X[idx].copy(), Y[idx].copy()
  off = np.zeros(2); last_v = np.zeros(2); nj = 0
  for k in range(1, len(ut)):
    dt = ut[k] - ut[k - 1]
    d = np.array([X[idx[k]] - X[idx[k - 1]], Y[idx[k]] - Y[idx[k - 1]]])
    if dt > 0 and np.hypot(*d) / dt > 3.0:
      off += d - last_v * dt; nj += 1
    elif dt > 0:
      last_v = d / dt
    ux[k] = X[idx[k]] - off[0]; uy[k] = Y[idx[k]] - off[1]

  # 연속 보정 — 실제 적용 기록이 있으면 그걸, 없으면 같은 코드로 재생
  if applied_live:
    trk = np.interp(ut, t[~np.isnan(live)], np.unwrap(np.radians(live[~np.isnan(live)])))
    trk = np.degrees(trk); src = '실제 적용 (기록값)'
  else:
    tr = HeadingTracker(tau=a.tau)
    trk = np.full(len(ut), np.nan); j = 0
    for i in range(len(t)):
      tr.add_imu(t[i], math.radians(raw[i]))
      if j < len(idx) and idx[j] == i:
        if tr.offset is None and i >= i_cal:
          tr.set_offset(math.radians(off0))
        s_ = None if np.isnan(sig[i]) else sig[i] / 100
        tr.add_fix(t[i], ux[j], uy[j], s_)
        if tr.offset is not None:
          trk[j] = math.degrees(tr.offset)
        j += 1
    src = f'재생 (HeadingTracker, tau {a.tau:.0f}s)'

  # 속도·진행방향 (중심차분)
  n = len(ut)
  spd = np.full(n, np.nan); crs = np.full(n, np.nan); use = np.zeros(n, bool)
  rawu = np.unwrap(np.radians(raw[idx]))
  for k in range(n):
    a0 = np.searchsorted(ut, ut[k] - 0.25); b0 = np.searchsorted(ut, ut[k] + 0.25, 'right') - 1
    if b0 > a0 and ut[b0] > ut[a0]:
      spd[k] = math.hypot(ux[b0] - ux[a0], uy[b0] - uy[a0]) / (ut[b0] - ut[a0])
    a1 = np.searchsorted(ut, ut[k] - 0.3); b1 = np.searchsorted(ut, ut[k] + 0.3, 'right') - 1
    if b1 > a1 and ut[b1] > ut[a1]:
      d = np.array([ux[b1] - ux[a1], uy[b1] - uy[a1]]); dt = ut[b1] - ut[a1]
      if np.hypot(*d) / dt >= 0.5:
        crs[k] = math.degrees(math.atan2(d[1], d[0]))
        rate = abs(rawu[b1] - rawu[a1]) / dt
        use[k] = rate < math.radians(15)
  auto = (~np.isnan(yo[idx])) & (cv[idx] > 0.05)
  if a.end is not None:
    auto &= ut <= a.end
  ef = wrapd(crs - (raw[idx] + off0))
  et = wrapd(crs - (raw[idx] + trk))
  use &= auto & ~np.isnan(crs)

  name = a.name or os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  base = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  out_csv = f'/home/han/racing_ws/logs/gps_ts_{base}.csv'
  keep = auto | (ut >= t_cal - 15)                      # 캘리브 직진도 보이게 15초 앞부터
  if a.end is not None:
    keep &= ut <= a.end
  with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t_s', 'x_m', 'y_m', 'rtk_sigma_cm', 'gps_speed', 'gps_course_deg', 'imu_raw_deg',
                'offset_fixed_deg', 'offset_track_deg', 'err_fixed_deg', 'err_track_deg', 'usable'])
    fm = lambda v, p='.2f': '' if (v is None or np.isnan(v)) else format(v, p)  # noqa: E731
    for k in np.where(keep)[0]:
      i = idx[k]
      w.writerow([f'{ut[k] - t_cal:.2f}', f'{ux[k]:.3f}', f'{uy[k]:.3f}', fm(sig[i], '.1f'), fm(spd[k], '.3f'),
                  fm(crs[k]), fm(raw[i]), f'{off0:.2f}', fm(wrapd(trk[k]) if not np.isnan(trk[k]) else np.nan),
                  fm(ef[k] if not np.isnan(crs[k]) else np.nan), fm(et[k] if not np.isnan(crs[k]) else np.nan),
                  int(use[k])])
  E_f, E_t = ef[use], et[use]
  st = lambda e: (e.mean(), math.sqrt((e ** 2).mean()))  # noqa: E731
  mf, rf = st(E_f); mt, rt = st(E_t)
  print(f'{name}: 캘리브 오프셋 {off0:.1f}° · 채점 표본 {use.sum()} · 점프 {nj}회')
  print(f'  고정       평균 {mf:+5.2f}°  RMS {rf:5.2f}°')
  print(f'  연속 보정  평균 {mt:+5.2f}°  RMS {rt:5.2f}°   ← {src}')
  print(f'✅ {out_csv}')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  x = ut - t_cal; k = keep
  if a.error:
    u = use & k
    fig, ax0 = plt.subplots(figsize=(14, 4.8))
    ax0.axhspan(-2, 2, color='#16a34a', alpha=.12, label='±2°')
    ax0.axhline(0, color='#333', lw=.8)
    lab_t = '연속 보정 (실제 적용)' if applied_live else f'연속 보정 (기록 재생, tau {a.tau:.0f}s)'
    ax0.plot(x[u], ef[u], '.', ms=4, color='#94a3b8', label=f'1회 캘리브   평균 {mf:+.1f}° · RMS {rf:.1f}°')
    ax0.plot(x[u], et[u], '.', ms=4, color='#2563eb', label=f'{lab_t}   평균 {mt:+.1f}° · RMS {rt:.1f}°')
    ax0.set_xlim(min(0.0, x[u][0]) - 1, x[u][-1] + 1); ax0.grid(alpha=.3)
    lo = min(-5, np.nanpercentile(ef[u], 1) - 2); hi = max(5, np.nanpercentile(ef[u], 99) + 2)
    ax0.set_ylim(lo, hi)
    ax0.set_xlabel('캘리브 완료 후 시간 [s]', fontsize=11)
    ax0.set_ylabel('헤딩 오차 [°]\n(GPS 진행방향 − 헤딩)', fontsize=11)
    ax0.legend(fontsize=10.5, loc='lower left')
    ax0.set_title(f'{name}\n헤딩 오차 RMS  1회 캘리브 {rf:.1f}° → 연속 보정 {rt:.1f}°  (직진·주행 표본 {u.sum()}개)',
                  fontsize=13, weight='bold')
    out = os.path.splitext(out_csv)[0] + '_herr.png'
    fig.savefig(out, dpi=130, bbox_inches='tight'); print(f'✅ {out}')
    return 0
  if a.simple:
    # 차가 실제로 쓴 헤딩 = IMU + (그 주행에서 적용된 오프셋). 연속값으로 펼쳐서 그린다.
    used = raw[idx] + (trk if applied_live else off0)
    hu = np.degrees(np.unwrap(np.radians(used)))
    # GPS 진행방향을 같은 바퀴 수로 맞춘다 (±360° 차이만 제거)
    cu = crs + 360.0 * np.round((hu - crs) / 360.0)
    E = ef if not applied_live else et
    eu = E[use & k]
    fig, ax0 = plt.subplots(figsize=(14, 4.8))
    for nt in a.note:
      t0_, t1_, lab = nt.split(',', 2)
      ax0.axvspan(float(t0_), float(t1_), color='#fecaca', alpha=.4)
      ax0.text(float(t0_) + .3, 0.97, lab, fontsize=10, color='#b91c1c', va='top', transform=ax0.get_xaxis_transform())
    ax0.plot(x[k], cu[k], '.', ms=3.5, color='#dc2626', label='GPS 진행방향 (기준)')
    ax0.plot(x[k], hu[k], color='#2563eb', lw=1.4,
             label='IMU 헤딩 + ' + ('연속 보정 오프셋' if applied_live else '고정 오프셋 (캘리브 1회)'))
    ax0.set_xlim(x[k][0], x[k][-1]); ax0.grid(alpha=.3)
    ax0.set_xlabel('캘리브 완료 후 시간 [s]', fontsize=11); ax0.set_ylabel('방위 [°] (연속)', fontsize=11)
    ax0.legend(fontsize=10.5, loc='best')
    ax0.set_title(f'{name}\n헤딩 오차 평균 {eu.mean():+.1f}° · RMS {math.sqrt((eu ** 2).mean()):.1f}° '
                  f'(직진·주행 표본 {len(eu)}개)', fontsize=13, weight='bold')
    out = os.path.splitext(out_csv)[0] + '_heading.png'
    fig.savefig(out, dpi=130, bbox_inches='tight'); print(f'✅ {out}')
    return 0
  fig, ax = plt.subplots(4, 1, figsize=(15, 15), sharex=True, gridspec_kw=dict(height_ratios=[1.2, 1, 1.1, .8]))
  ax[0].plot(x[k], wrapd(crs[k]), '.', ms=3, color='#16a34a', label='GPS 진행방향')
  ax[0].plot(x[k], wrapd(raw[idx][k] + off0), color='#94a3b8', lw=1.2, label='IMU + 고정 오프셋')
  ax[0].plot(x[k], wrapd(raw[idx][k] + trk[k]), color='#2563eb', lw=1.2, label='IMU + 연속 보정 오프셋')
  ax[0].set_ylabel('방위 [°]'); ax[0].legend(fontsize=10, loc='upper right'); ax[0].grid(alpha=.3)
  ax[0].set_title('① GPS 진행방향 vs IMU 헤딩', fontsize=12.5, weight='bold')
  ax[1].axhline(off0, color='#94a3b8', lw=2, ls='--', label=f'고정 (캘리브 1회) {off0:.1f}°')
  ax[1].plot(x[k], trk[k] if np.nanmax(trk[k]) - np.nanmin(trk[k]) < 300 else wrapd(trk[k]),
             color='#2563eb', lw=2, label=f'연속 보정 — {src}')
  ax[1].set_ylabel('헤딩 오프셋 [°]'); ax[1].legend(fontsize=10); ax[1].grid(alpha=.3)
  ax[1].set_title('② 헤딩 오프셋 — 기울기가 IMU 드리프트', fontsize=12.5, weight='bold')
  u = use & k
  ax[2].plot(x[u], ef[u], '.', ms=3, color='#94a3b8', label=f'고정   평균 {mf:+.1f}° · RMS {rf:.1f}°')
  ax[2].plot(x[u], et[u], '.', ms=3, color='#2563eb', label=f'연속 보정   평균 {mt:+.1f}° · RMS {rt:.1f}°')
  ax[2].axhline(0, color='#333', lw=.8); ax[2].axhspan(-2, 2, color='#16a34a', alpha=.12)
  ax[2].set_ylabel('헤딩 오차 [°]'); ax[2].legend(fontsize=10); ax[2].grid(alpha=.3)
  ax[2].set_title('③ 헤딩 오차 = GPS 진행방향 − 헤딩 (직진·주행 표본)', fontsize=12.5, weight='bold')
  ax[3].plot(x[k], spd[k], color='#0f766e', lw=1.2, label='GPS 속도 [m/s]')
  ax[3].set_ylabel('m/s'); ax[3].grid(alpha=.3); ax[3].set_xlabel('캘리브 완료 후 시간 [s]')
  if not np.all(np.isnan(sig[idx][k])):
    a2 = ax[3].twinx(); a2.plot(x[k], sig[idx][k], color='#ea580c', lw=1, label='RTK σ [cm]')
    a2.set_ylabel('RTK σ [cm]', color='#ea580c'); a2.set_ylim(0, max(6, np.nanmax(sig[idx][k]) * 1.2))
    a2.axhline(5, color='#ea580c', ls=':', lw=1); a2.legend(fontsize=9, loc='upper right')
  ax[3].legend(fontsize=9, loc='upper left')
  ax[3].set_title('④ GPS 품질 — 속도와 RTK σ (5 cm 넘으면 FLOAT)', fontsize=12.5, weight='bold')
  for a_ in ax:
    a_.axvline(0, color='#a855f7', lw=1, ls='--')
  fig.suptitle(f'GPS 기반 헤딩 보정 시계열 — {name}', fontsize=15, weight='bold')
  out = os.path.splitext(out_csv)[0] + '.png'
  fig.savefig(out, dpi=105, bbox_inches='tight'); print(f'✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

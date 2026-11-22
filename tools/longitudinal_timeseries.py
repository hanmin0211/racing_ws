#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""longitudinal_timeseries.py — 종방향(속도) 제어를 **시계열**로 푼다 (논문 종방향 파트).

한 주행(drive.csv)을 기록 행(20 Hz)마다:
  t_s          캘리브 완료 기준 시간 [s]
  s_m          경로 위 위치 [m] (전진 전용 투영 — 8자 교차점에서 안 튄다)
  target_v     목표속도 (/target_speed — 종방향 제어기 출력)
  actual_v     실측속도 — RTK 위치 중심차분(±0.25 s, GPS 갱신 시각 기준, 점프 이어붙임)
  err_v        actual − target
  pwm          실제 구동 PWM (/drive_pwm)
  pwm_ff       그 주행의 FF 공식으로 목표속도에서 계산한 PWM
  pwm_fb       pwm − pwm_ff  = 피드백(PI)이 더하거나 뺀 몫. 개루프 주행은 ≈ 0
  pwm_sat      PWM ≥ 160 이면 1 (구동 포화)
  grade_pct    IMU 피치 경사 [%] (10/8 부터. 가감속이 섞인 원시값)
  curvature    경로 곡률 [1/m]
  vcc_mv       아두이노 5 V 레일 [mV]

사용:
  python3 tools/longitudinal_timeseries.py <drive.csv> --ff 17.2,38.8 --end 143 --name '...' --wp <경로>
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import smooth_path as sp  # noqa: E402


def col(rows, k):
  o = []
  for r in rows:
    try:
      o.append(float(r.get(k)))
    except (TypeError, ValueError):
      o.append(np.nan)
  return np.array(o)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--ff', default='17.2,38.8', help='FF 공식 static,gain (PWM = static + gain·v)')
  ap.add_argument('--end', type=float, default=None, help='이 기록 시각(t) 이후 버림')
  ap.add_argument('--name', default=None)
  ap.add_argument('--wp', default='/home/han/racing_ws/config/chungju_school/wp_heading_1007b_v2.yaml')
  ap.add_argument('--note', action='append', default=[], help='t0,t1,설명 — 그림에 음영 표시 (캘리브 기준 시간)')
  ap.add_argument('--speed-only', action='store_true', help='목표 vs 실측 한 장만 (_speed.png)')
  a = ap.parse_args()
  ff_s, ff_g = map(float, a.ff.split(','))

  rows = list(csv.DictReader(open(a.csv)))
  t, X, Y = col(rows, 't'), col(rows, 'x'), col(rows, 'y')
  tv, cv, yo, pwm = col(rows, 'target_v'), col(rows, 'cmd_v'), col(rows, 'yaw_offset_deg'), col(rows, 'drive_pwm')
  grade, curv, vcc = col(rows, 'grade_pct'), col(rows, 'curvature'), col(rows, 'vcc_mv')
  i_cal = int(np.argmax(~np.isnan(yo)))
  t_cal = t[i_cal]
  auto = ~np.isnan(yo) & (cv > 0.05)
  if a.end is not None:
    auto &= t <= a.end
  i_last = np.where(auto)[0][-1]
  keep = (np.arange(len(t)) >= i_cal) & (np.arange(len(t)) <= i_last)

  # 실측속도 — GPS 갱신 시각으로 중심차분, 점프 이어붙임
  ch = np.r_[True, (np.diff(X) != 0) | (np.diff(Y) != 0)] & ~np.isnan(X)
  idx = np.where(ch)[0]
  ut = t[idx]; ux = X[idx].copy(); uy = Y[idx].copy()
  off = np.zeros(2); last_v = np.zeros(2); nj = 0
  for k in range(1, len(ut)):
    dt = ut[k] - ut[k - 1]
    d = np.array([X[idx[k]] - X[idx[k - 1]], Y[idx[k]] - Y[idx[k - 1]]])
    if dt > 0 and np.hypot(*d) / dt > 3.0:
      off += d - last_v * dt; nj += 1
    elif dt > 0:
      last_v = d / dt
    ux[k] = X[idx[k]] - off[0]; uy[k] = Y[idx[k]] - off[1]
  uv = np.full(len(ut), np.nan)
  for k in range(len(ut)):
    p = np.searchsorted(ut, ut[k] - 0.25); q = np.searchsorted(ut, ut[k] + 0.25, 'right') - 1
    if q > p and ut[q] > ut[p]:
      uv[k] = math.hypot(ux[q] - ux[p], uy[q] - uy[p]) / (ut[q] - ut[p])
  ok = ~np.isnan(uv)
  v = np.interp(t, ut[ok], uv[ok])

  # 경로 위치 — 전진 전용 투영
  wp = np.array(sp.load(a.wp)); S = np.r_[0, np.cumsum(np.hypot(*np.diff(wp, axis=0).T))]
  s_m = np.full(len(t), np.nan); j = None
  for i in np.where(keep & ~np.isnan(X))[0]:
    if j is None:
      j = int(np.argmin(np.hypot(wp[:, 0] - X[i], wp[:, 1] - Y[i])))
    hi = min(len(wp) - 1, j + 30)
    j = j + int(np.argmin(np.hypot(wp[j:hi + 1, 0] - X[i], wp[j:hi + 1, 1] - Y[i])))
    s_m[i] = S[j]

  pff = np.where(tv > 0.05, ff_s + ff_g * tv, 0.0)
  pfb = pwm - pff
  err = v - tv
  base = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  name = a.name or base
  out_csv = f'/home/han/racing_ws/logs/long_ts_{base}.csv'
  fm = lambda x_, p='.3f': '' if np.isnan(x_) else format(x_, p)  # noqa: E731
  with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t_s', 's_m', 'x_m', 'y_m', 'target_v', 'actual_v', 'err_v', 'cmd_v', 'pwm', 'pwm_ff',
                'pwm_fb', 'pwm_sat', 'grade_pct', 'curvature', 'vcc_mv', 'mode'])
    for i in np.where(keep)[0]:
      w.writerow([f'{t[i] - t_cal:.2f}', fm(s_m[i], '.1f'), f'{X[i]:.3f}', f'{Y[i]:.3f}', fm(tv[i], '.2f'),
                  fm(v[i]), fm(err[i]), fm(cv[i], '.2f'), fm(pwm[i], '.0f'), fm(pff[i], '.1f'),
                  fm(pfb[i], '.1f'), int(pwm[i] >= 160), fm(grade[i], '.2f'), fm(curv[i], '.4f'),
                  fm(vcc[i], '.0f'), rows[i].get('mode', '')])

  # 통계 — 주행 중(실측 > 0.15) 표본
  m = auto & (v > 0.15)
  e = err[m]
  stats = dict(rms=math.sqrt((e ** 2).mean()), bias=e.mean(), over=100 * (np.abs(e) > .3).mean(),
               within=100 * (np.abs(e) <= .2).mean(), sat=100 * (pwm[m] >= 160).mean(),
               fb_abs=np.nanmean(np.abs(pfb[m])), dur=m.sum() * 0.05)
  print(f'{name}: 주행 {stats["dur"]:.0f}s · RMS {stats["rms"]:.3f} · 편향 {stats["bias"]:+.3f} · '
        f'±0.2 안 {stats["within"]:.0f}% · 0.3 초과 {stats["over"]:.0f}% · PWM 포화 {stats["sat"]:.0f}% · '
        f'피드백 몫 |평균| {stats["fb_abs"]:.1f} PWM · GPS 점프 {nj}회')
  print(f'✅ {out_csv}')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  x = t - t_cal; k = keep
  if a.speed_only:
    fig, ax0 = plt.subplots(figsize=(14, 4.8))
    for nt in a.note:
      t0_, t1_, lab = nt.split(',', 2)
      ax0.axvspan(float(t0_), float(t1_), color='#fecaca', alpha=.4)
      ax0.text(float(t0_) + .3, 2.22, lab, fontsize=10, color='#b91c1c', va='top')
    ax0.plot(x[k], tv[k], color='#dc2626', lw=2.2, label='목표 속도')
    ax0.plot(x[k], v[k], color='#2563eb', lw=1.4, label='실측 속도 (RTK GPS)')
    ax0.set_ylim(0, 2.3); ax0.set_xlim(x[k][0], x[k][-1]); ax0.grid(alpha=.3)
    ax0.set_xlabel('캘리브 완료 후 시간 [s]', fontsize=11); ax0.set_ylabel('속도 [m/s]', fontsize=11)
    ax0.legend(fontsize=10.5, loc='lower right')
    ax0.set_title(f'{name}\nRMS {stats["rms"]:.2f} m/s · 편향 {stats["bias"]:+.2f} · ±0.2 m/s 안 {stats["within"]:.0f}%',
                  fontsize=13, weight='bold')
    out = os.path.splitext(out_csv)[0] + '_speed.png'
    fig.savefig(out, dpi=130, bbox_inches='tight'); print(f'✅ {out}')
    return 0
  fig, ax = plt.subplots(4, 1, figsize=(15, 15), sharex=True, gridspec_kw=dict(height_ratios=[1.3, .9, 1.1, .8]))
  ax[0].plot(x[k], tv[k], color='#dc2626', lw=2, label='목표 속도')
  ax[0].plot(x[k], v[k], color='#2563eb', lw=1.3, label='실측 속도 (RTK GPS)')
  ax[0].set_ylabel('속도 [m/s]'); ax[0].set_ylim(0, 2.3); ax[0].grid(alpha=.3); ax[0].legend(fontsize=10, loc='lower right')
  ax[0].set_title(f'① 목표 vs 실측 — RMS {stats["rms"]:.2f} m/s · 편향 {stats["bias"]:+.2f} · '
                  f'±0.2 m/s 안 {stats["within"]:.0f}%', fontsize=12.5, weight='bold')
  ax[1].plot(x[k], err[k], color='#7c3aed', lw=1.1)
  ax[1].axhspan(-.2, .2, color='#16a34a', alpha=.13); ax[1].axhline(0, color='#333', lw=.8)
  ax[1].set_ylabel('오차 [m/s]'); ax[1].grid(alpha=.3)
  ax[1].set_title('② 추종 오차 (실측 − 목표) · 초록 띠 ±0.2 m/s', fontsize=12.5, weight='bold')
  ax[2].plot(x[k], pwm[k], color='#111827', lw=1.2, label='실제 PWM')
  ax[2].plot(x[k], pff[k], color='#f59e0b', lw=1.6, ls='--', label=f'FF 몫 ({ff_s:g} + {ff_g:g}·목표)')
  ax[2].fill_between(x[k], pff[k], pwm[k], color='#2563eb', alpha=.18, label='피드백(PI) 몫 = 실제 − FF')
  ax[2].axhline(160, color='#dc2626', ls=':', lw=1, label='포화 160')
  ax[2].set_ylabel('PWM'); ax[2].grid(alpha=.3); ax[2].legend(fontsize=9.5, loc='upper left', ncol=2)
  ax[2].set_title(f'③ 구동 PWM — 피드백 몫 평균 |{stats["fb_abs"]:.0f}| PWM · 포화 {stats["sat"]:.0f}%',
                  fontsize=12.5, weight='bold')
  if not np.all(np.isnan(grade[k])):
    ax[3].plot(x[k], grade[k], color='#0f766e', lw=1, label='경사 [%] (IMU 피치, 가감속 섞임)')
  ax[3].set_ylabel('경사 [%]'); ax[3].grid(alpha=.3); ax[3].axhline(0, color='#333', lw=.6)
  a2 = ax[3].twinx(); a2.plot(x[k], np.abs(curv[k]), color='#a855f7', lw=1, alpha=.8, label='|곡률| [1/m]')
  a2.set_ylabel('|곡률| [1/m]', color='#a855f7')
  ax[3].legend(fontsize=9, loc='upper left'); a2.legend(fontsize=9, loc='upper right')
  ax[3].set_xlabel('캘리브 완료 후 시간 [s]')
  ax[3].set_title('④ 외란과 목표의 원인 — 경사(외란) · 곡률(목표 감속)', fontsize=12.5, weight='bold')
  for nt in a.note:
    t0_, t1_, lab = nt.split(',', 2)
    for a_ in ax:
      a_.axvspan(float(t0_), float(t1_), color='#fecaca', alpha=.35)
    ax[0].text(float(t0_), 2.2, lab, fontsize=9.5, color='#b91c1c', va='top')
  fig.suptitle(f'종방향 속도 제어 시계열 — {name}', fontsize=15, weight='bold')
  out = os.path.splitext(out_csv)[0] + '.png'
  fig.savefig(out, dpi=105, bbox_inches='tight'); print(f'✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

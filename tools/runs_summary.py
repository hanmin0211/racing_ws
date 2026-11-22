#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""runs_summary.py — 10월 실주행을 **같은 기준**으로 한 표·한 그림에 모은다 (논문용).

같은 기준
  · 구간   : 헤딩 캘리브 뒤 AUTO · 실제로 움직인 표본 · 각 주행의 '유효 끝' 까지
             (충돌·고장 이후는 제어 성능이 아니다 — 끝 시각과 이유를 표에 남긴다)
  · 실측속도: RTK 위치 중심차분(±0.25 s, **GPS 갱신 시각 기준**) — 한 걸음 3 m/s 넘는
             GPS 점프는 이어 붙임
             (엔코더는 10/8 부터 간헐 불량이라 모든 주행에 같은 기준을 쓴다)
  · 헤딩오차: 중심차분 GPS 진행방향 − 차가 쓴 헤딩(yaw_deg), 직진·주행 표본
  · 횡오차  : 그 주행이 따라간 경로 파일 기준

사용:
  python3 tools/runs_summary.py
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import smooth_path as sp  # noqa: E402

WS = '/home/han/racing_ws'
S_COMMON = 69.0      # 10/12 헤딩보정 주행이 조향 불능 전까지 간 곳 (출발~왼쪽 루프~교차점)
CFG = f'{WS}/config/chungju_school/'
RUNS = [
    # (이름, csv, 경로, 유효 끝 t(None=끝까지), 끝 이유, 제어, 헤딩)
    ('10/7 개루프 FF', 'data/2026-10-07/heading_1007_2034/drive.csv', 'wp_heading_1007b_final.yaml',
     102.0, '평활화 경로가 코너 안쪽을 잘라 벽 충돌', 'ROS FF 46+42.6v', '1회 캘리브'),
    ('10/8 보드 FF', 'data/2026-10-08/pid_1008b_1727/drive.csv', 'wp_heading_1007b_resampled_0.5.yaml',
     84.6, '오른쪽 루프 안쪽 벽 충돌', '보드 FF 17.2+38.8v', '1회 캘리브'),
    ('10/12 속도 PI', 'data/2026-10-12/pid_run_2106/drive.csv', 'wp_heading_1007b_v2.yaml',
     None, '완주', 'FF + GPS속도 PI', '1회 캘리브'),
    ('10/12 PI + 헤딩보정', 'data/2026-10-12/pid_hdg_2130/drive.csv', 'wp_heading_1007b_v2.yaml',
     143.0, '조향 불능(배터리) — 명령 따라가지 않음', 'FF + GPS속도 PI', '연속 보정'),
]


def col(rows, k):
  o = []
  for r in rows:
    try:
      o.append(float(r.get(k)))
    except (TypeError, ValueError):
      o.append(np.nan)
  return np.array(o)


def dejump(t, X, Y):
  X, Y = X.copy(), Y.copy()
  Xo, Yo = X.copy(), Y.copy()
  ch = np.r_[True, (np.diff(Xo) != 0) | (np.diff(Yo) != 0)] & ~np.isnan(Xo)
  idx = np.where(ch)[0]
  off = np.zeros(2); last_v = np.zeros(2)
  for k, (i0, i1) in enumerate(zip(idx[:-1], idx[1:])):
    dt = t[i1] - t[i0]; d = np.array([Xo[i1] - Xo[i0], Yo[i1] - Yo[i0]])
    if dt > 0 and np.hypot(*d) / dt > 3.0:
      off += d - last_v * dt
    elif dt > 0:
      last_v = d / dt
    end = idx[k + 2] if k + 2 < len(idx) else len(X)
    X[i1:end] = Xo[i1:end] - off[0]; Y[i1:end] = Yo[i1:end] - off[1]
  return X, Y, idx


def analyse(name, path, wpf, t_end, why, ctrl, hdg):
  rows = list(csv.DictReader(open(f'{WS}/{path}')))
  t, X, Y = col(rows, 't'), col(rows, 'x'), col(rows, 'y')
  tv, cv, yo, pwm = col(rows, 'target_v'), col(rows, 'cmd_v'), col(rows, 'yaw_offset_deg'), col(rows, 'drive_pwm')
  yaw = np.radians(col(rows, 'yaw_deg'))
  Xd, Yd, idx = dejump(t, X, Y)
  # ★ 실제 GPS 갱신 시각으로 미분한다. 기록은 20 Hz · GPS 는 10 Hz 라 '행 10개' 창에
  #   GPS 걸음이 4개일 때도 5개일 때도 있어 속도가 0.8배로 찍히는 점이 섞였다.
  ut, ux, uy = t[idx], Xd[idx], Yd[idx]
  uv = np.full(len(ut), np.nan)
  for k in range(len(ut)):
    a = np.searchsorted(ut, ut[k] - 0.25); b = np.searchsorted(ut, ut[k] + 0.25, 'right') - 1
    if b > a and ut[b] > ut[a]:
      uv[k] = math.hypot(ux[b] - ux[a], uy[b] - uy[a]) / (ut[b] - ut[a])
  ok_ = ~np.isnan(uv)
  v = np.interp(t, ut[ok_], uv[ok_])
  auto = ~np.isnan(yo) & (cv > 0.05)
  if t_end is not None:
    auto &= t <= t_end
  m = auto & (v > 0.15) & ~np.isnan(v) & ~np.isnan(tv)
  e = v[m] - tv[m]
  t_auto = t[auto]
  res = dict(name=name, ctrl=ctrl, hdg=hdg, why=why, dur=float(m.sum() * 0.05),
             rms=math.sqrt((e ** 2).mean()), bias=e.mean(), over=100 * (np.abs(e) > .3).mean(),
             r=np.corrcoef(v[m], tv[m])[0, 1], sat=100 * np.mean(pwm[m] >= 160))
  # 헤딩 — 직진·주행 표본, 중심차분 진행방향
  ft = t[idx]; E = []
  for k, i in enumerate(idx):
    if not auto[i]:
      continue
    j0 = np.searchsorted(ft, ft[k] - 0.3); j1 = np.searchsorted(ft, ft[k] + 0.3) - 1
    if j0 < 0 or j1 >= len(ft) or j1 <= j0:
      continue
    a, b = idx[j0], idx[j1]
    d = np.array([Xd[b] - Xd[a], Yd[b] - Yd[a]]); dt = t[b] - t[a]
    if dt <= 0 or np.hypot(*d) / dt < 0.5:
      continue
    dy = math.atan2(math.sin(yaw[b] - yaw[a]), math.cos(yaw[b] - yaw[a]))
    if abs(dy) / dt > math.radians(15):
      continue
    c = math.atan2(d[1], d[0])
    E.append((t[i], math.degrees(math.atan2(math.sin(c - yaw[i]), math.cos(c - yaw[i])))))
  E = np.array(E)
  wp_ = np.array(sp.load(CFG + wpf)); S_ = np.r_[0, np.cumsum(np.hypot(*np.diff(wp_, axis=0).T))]
  # 전진 전용 투영 — 8자 교차점에서 반대편 구간에 붙지 않게 (시간순으로 앞으로만 찾는다)
  S_all = np.full(len(t), np.nan); j = None
  for i in np.where(auto & ~np.isnan(X))[0]:
    if j is None:
      j = int(np.argmin(np.hypot(wp_[:, 0] - X[i], wp_[:, 1] - Y[i])))
    hi = min(len(wp_) - 1, j + 30)
    j = j + int(np.argmin(np.hypot(wp_[j:hi + 1, 0] - X[i], wp_[j:hi + 1, 1] - Y[i])))
    S_all[i] = S_[j]
  # ★ 공통 구간 (모든 주행이 지난 곳만) — 주행마다 끝이 달라 전체 수치는 구간이 다르다
  sm = S_all[m]; c_ = sm <= S_COMMON
  ec = e[c_]
  ti = np.searchsorted(t, E[:, 0]); se = S_all[ti]; ch_ = E[se <= S_COMMON, 1]
  res.update(c_rms=math.sqrt((ec ** 2).mean()), c_bias=ec.mean(), c_over=100 * (np.abs(ec) > .3).mean(),
             c_r=np.corrcoef(v[m][c_], tv[m][c_])[0, 1], c_hrms=math.sqrt((ch_ ** 2).mean()), c_hmean=ch_.mean())
  res.update(h_mean=E[:, 1].mean(), h_rms=math.sqrt((E[:, 1] ** 2).mean()),
             h_slope=np.polyfit(E[:, 0] - E[0, 0], E[:, 1], 1)[0] * 60)
  # 횡오차
  wp = np.array(sp.load(CFG + wpf))
  P = np.c_[X[m], Y[m]]
  lat = np.array([np.min(np.hypot(*(wp - q).T)) for q in P])
  res.update(lat_mean=lat.mean(), lat_p95=np.percentile(lat, 95), c_lat=lat[c_].mean())
  # 경로 위치 s (그림용)
  res['_series'] = (S_all[m], tv[m], v[m])
  return res


def main():
  R = [analyse(*r) for r in RUNS]
  out_csv = f'{WS}/logs/runs_summary.csv'
  keys = ['name', 'ctrl', 'hdg', 'dur', 'rms', 'bias', 'over', 'r', 'sat', 'h_mean', 'h_rms',
          'h_slope', 'lat_mean', 'lat_p95', 'why']
  with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f); w.writerow(keys)
    for r in R:
      w.writerow([r[k] if isinstance(r[k], str) else f'{r[k]:.3f}' for k in keys])
  print(f'{"":20s} {"시간":>5s} {"RMS":>6s} {"편향":>6s} {">0.3":>5s} {"r":>5s} {"포화":>5s} '
        f'{"헤딩평균":>7s} {"헤딩RMS":>6s} {"드리프트":>7s} {"횡평균":>5s} {"횡95":>5s}  끝')
  for r in R:
    print(f'{r["name"]:20s} {r["dur"]:4.0f}s {r["rms"]:6.3f} {r["bias"]:+6.3f} {r["over"]:4.0f}% '
          f'{r["r"]:5.2f} {r["sat"]:4.0f}% {r["h_mean"]:+6.1f}° {r["h_rms"]:5.1f}° {r["h_slope"]:+5.1f}°/분 '
          f'{r["lat_mean"]:4.2f}m {r["lat_p95"]:4.2f}m  {r["why"]}')
  print(f'\n공통 구간 (경로 s ≤ {S_COMMON:.0f} m, 모든 주행이 지난 곳)')
  print(f'{"":20s} {"속도RMS":>7s} {"편향":>6s} {">0.3":>5s} {"r":>5s} {"헤딩평균":>7s} {"헤딩RMS":>6s} {"횡평균":>6s}')
  for r in R:
    print(f'{r["name"]:20s} {r["c_rms"]:7.3f} {r["c_bias"]:+6.3f} {r["c_over"]:4.0f}% {r["c_r"]:5.2f} '
          f'{r["c_hmean"]:+6.1f}° {r["c_hrms"]:5.1f}° {r["c_lat"]:5.2f}m')
  with open(out_csv.replace('.csv', '_common.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['name', 's_max_m', 'speed_rms', 'speed_bias', 'over_0.3_pct', 'shape_r',
                                   'heading_mean_deg', 'heading_rms_deg', 'lateral_mean_m'])
    for r in R:
      w.writerow([r['name'], S_COMMON] + [f'{r[k]:.3f}' for k in
                 ('c_rms', 'c_bias', 'c_over', 'c_r', 'c_hmean', 'c_hrms', 'c_lat')])
  print(f'✅ {out_csv}')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig = plt.figure(figsize=(17, 17))
  gs = fig.add_gridspec(len(R) + 1, 3, height_ratios=[1] * len(R) + [1.2], hspace=.55, wspace=.32)
  for i, r in enumerate(R):
    ax = fig.add_subplot(gs[i, :])
    s, tv, v = r['_series']
    o = np.argsort(s, kind='stable')
    ax.plot(s, tv, '.', ms=2, color='#dc2626', label='목표 속도')
    ax.plot(s, v, '.', ms=2, color='#2563eb', label='실측 속도 (RTK)')
    ax.set_xlim(0, 110); ax.set_ylim(0, 2.2); ax.grid(alpha=.3); ax.set_ylabel('m/s')
    ax.axvspan(0, S_COMMON, color='#e0f2fe', alpha=.5, zorder=0)
    ax.set_title(f'{r["name"]} — {r["ctrl"]} · 헤딩 {r["hdg"]}  |  RMS {r["rms"]:.2f} · r {r["r"]:.2f} · '
                 f'끝: {r["why"]}', fontsize=11.5, weight='bold', loc='left')
    if i == 0:
      ax.legend(fontsize=9, loc='lower left', markerscale=5)
    if i == len(R) - 1:
      ax.set_xlabel('경로 위치 s [m]')
  names = [r['name'].replace(' ', '\n', 1) for r in R]
  xs = np.arange(len(R))
  for k, (key, lab, col_) in enumerate((('c_rms', f'속도 RMS [m/s] (공통 s≤{S_COMMON:.0f}m)', '#2563eb'),
                                        ('c_hrms', f'헤딩 오차 RMS [°] (공통)', '#7c3aed'),
                                        ('c_lat', f'경로 횡오차 평균 [m] (공통)', '#16a34a'))):
    ax = fig.add_subplot(gs[len(R), k])
    vals = [r[key] for r in R]
    ax.bar(xs, vals, color=col_, alpha=.85)
    for x_, v_ in zip(xs, vals):
      ax.text(x_, v_, f'{v_:.2f}', ha='center', va='bottom', fontsize=10)
    ax.set_xticks(xs); ax.set_xticklabels(names, fontsize=9); ax.set_title(lab, fontsize=12, weight='bold')
    ax.grid(axis='y', alpha=.3)
  fig.suptitle('충주 교내 10월 실주행 비교 — 같은 기준', fontsize=16, weight='bold', y=.995)
  out = f'{WS}/logs/runs_summary.png'
  fig.savefig(out, dpi=105, bbox_inches='tight'); print(f'✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heading_offset_plot.py — GPS 이동방향으로 저가형 IMU 방위각을 보정한 결과를 본다.

★ 무엇을 재나
  yaw_offset = course − imu_yaw 를 10m 직진으로 1회 산출해 IMU 에 더한다.
  보정이 맞았다면 주행 내내 '보고된 헤딩(yaw_deg)' 이 'GPS 이동방향' 과 겹쳐야 한다.
    GPS 이동방향 course = atan2(Δy, Δx)  (0.5초 창 위치미분, 전진만)
    잔차 = course − yaw_deg

★ 반드시 직선/커브를 나눠 볼 것 (2026-10-07 에 한 번 속을 뻔했다)
  위치미분 course 는 0.5초 창의 **현(chord)** 방향이라 커브에서 실제 헤딩보다
  뒤처진다. 그래서 커브 구간 잔차는 '헤딩이 틀린 것' 이 아니라 '측정법의 한계' 다.
  전체를 한 번에 회귀하면 커브 비중 변화가 '드리프트' 로 보인다.
    heading_1007_2034:  전체 기울기 +3.41°/분 → 직선만 −1.46°/분 (방향 없음)
  → **헤딩 정확도는 직선 구간 통계로 말할 것.**

★ 캘리브 전/후
  drive.csv 의 yaw_offset_deg 가 처음 생기는 시각 = 캘리브 완료. 그 전 잔차는
  보정 전이라 ≈ yaw_offset 이고, 그 뒤는 ≈ 0 이어야 한다.
  (2026-10-07 이전 기록엔 yaw_offset_deg 컬럼이 없다 — 잔차 계단으로 추정)

사용:
  python3 tools/heading_offset_plot.py data/2026-10-07/heading_1007_2034/drive.csv
"""
import argparse, csv, math, os, sys
import numpy as np

WIN = 10           # 20Hz × 0.5s
MIN_MOVE = 0.25    # 창 동안 최소 이동 [m] — 느리면 course 가 잡음
STRAIGHT_K = 0.05  # |곡률| 이 이보다 작으면 직선


def col(rows, k):
  o = []
  for r in rows:
    try: o.append(float(r.get(k)))
    except (TypeError, ValueError): o.append(np.nan)
  return np.array(o)


def norm(a): return (a + 180) % 360 - 180


def main():
  ap = argparse.ArgumentParser(); ap.add_argument('csv'); ap.add_argument('--out')
  a = ap.parse_args()
  rows = list(csv.DictReader(open(a.csv)))
  c = lambda k: col(rows, k)
  t, X, Y, yaw, cv, yo, cu = c('t'), c('x'), c('y'), c('yaw_deg'), c('cmd_v'), c('yaw_offset_deg'), c('curvature')
  ok = ~np.isnan(t) & ~np.isnan(X) & ~np.isnan(yaw)
  t, X, Y, yaw, cv, yo, cu = [z[ok] for z in (t, X, Y, yaw, cv, yo, cu)]

  T, Cc, Yw, R, K = [], [], [], [], []
  for i in range(WIN, len(X)):
    if not (cv[i] > 0.1): continue
    dx, dy = X[i] - X[i-WIN], Y[i] - Y[i-WIN]
    if math.hypot(dx, dy) < MIN_MOVE: continue
    course = math.degrees(math.atan2(dy, dx))
    T.append(t[i]); Cc.append(course); Yw.append(yaw[i]); R.append(norm(course - yaw[i])); K.append(cu[i])
  T, Cc, Yw, R, K = map(np.array, (T, Cc, Yw, R, K))
  if len(R) < 30:
    print('❌ 전진 표본이 너무 적다'); return 1

  if (~np.isnan(yo)).any():
    k = int(np.argmax(~np.isnan(yo))); tcal = t[k]; off = yo[k]; how = 'yaw_offset_deg 컬럼'
  else:
    # 옛 기록 — 잔차가 크게 계단으로 떨어지는 지점
    big = np.abs(R) > 20; tcal = T[0]; off = float('nan'); how = '잔차 계단 추정'
    for j in np.where(~big)[0]:
      if big[j:].mean() < 0.05: tcal = T[j]; break
  pre = T < tcal; post = T >= tcal + 1.0
  Kk = np.where(np.isnan(K), 0, K)
  st = post & (np.abs(Kk) < STRAIGHT_K); cr = post & (np.abs(Kk) >= STRAIGHT_K)

  def s(m):
    x = R[m]; return x.mean(), x.std(), math.sqrt((x**2).mean()), np.percentile(np.abs(x), 95)
  print(f'yaw_offset {off:.2f}° · 캘리브 완료 t={tcal:.1f}s ({how})')
  if pre.any(): print(f'  보정 전  잔차 평균 {R[pre].mean():+.1f}°')
  for name, m in (('보정 후 전체', post), ('  └ 직선', st), ('  └ 커브', cr)):
    if m.sum() > 5:
      mu, sd, rms, p95 = s(m)
      print(f'  {name:12s} n={m.sum():5d}  평균 {mu:+6.2f}°  표준편차 {sd:5.2f}°  RMS {rms:5.2f}°  95% {p95:5.1f}°')
  if st.sum() > 30:
    print(f'  직선만 드리프트 기울기 {np.polyfit(T[st], R[st], 1)[0]*60:+.2f}°/분')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig = plt.figure(figsize=(15, 10))
  gs = fig.add_gridspec(3, 2, height_ratios=[1.2, 1, 1], hspace=.42, wspace=.22)
  ax = fig.add_subplot(gs[0, :])
  ax.plot(T, Cc, color='#2b6cb0', lw=1.4, label='GPS 이동방향 (위치미분 course)')
  ax.plot(T, Yw, color='#dc2626', lw=1.2, ls='--', label='보고된 헤딩 (IMU yaw + yaw_offset)')
  ax.axvline(tcal, color='#16a34a', lw=2.2, ls=':')
  ax.text(tcal, ax.get_ylim()[1]*0.85, f'  yaw_offset 적용\n  ({off:.1f}°)', color='#16a34a', fontsize=11, weight='bold')
  ax.set_title('① GPS 이동방향 vs 보정 헤딩', fontsize=13, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('방위각 [°]'); ax.grid(alpha=.3); ax.legend(fontsize=9.5, loc='lower right')

  ax = fig.add_subplot(gs[1, 0])
  ax.plot(T, R, color='#7c3aed', lw=1.0)
  ax.axvline(tcal, color='#16a34a', lw=2, ls=':'); ax.axhspan(-5, 5, color='#16a34a', alpha=.12)
  ax.set_title('② 잔차 = course − 헤딩 (초록띠 ±5°)', fontsize=12, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('잔차 [°]'); ax.grid(alpha=.3)

  ax = fig.add_subplot(gs[1, 1])
  ax.scatter(T[cr], R[cr], s=5, color='#f59e0b', alpha=.5, label='커브 (|κ|≥0.05)')
  ax.scatter(T[st], R[st], s=5, color='#2b6cb0', alpha=.7, label='직선 (|κ|<0.05)')
  ax.axhspan(-5, 5, color='#16a34a', alpha=.12); ax.set_ylim(-20, 20)
  ax.set_title('③ 보정 후 — 직선과 커브를 나눠서', fontsize=12, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('잔차 [°]'); ax.grid(alpha=.3); ax.legend(fontsize=9)

  ax = fig.add_subplot(gs[2, 0])
  bins = np.linspace(-20, 20, 61)
  ax.hist(R[st], bins=bins, color='#2b6cb0', alpha=.8, label=f'직선 n={st.sum()}')
  ax.hist(R[cr], bins=bins, color='#f59e0b', alpha=.5, label=f'커브 n={cr.sum()}')
  ax.axvspan(-5, 5, color='#16a34a', alpha=.12)
  ax.set_title('④ 잔차 분포', fontsize=12, weight='bold')
  ax.set_xlabel('잔차 [°]'); ax.set_ylabel('표본수'); ax.grid(alpha=.3); ax.legend(fontsize=9)

  ax = fig.add_subplot(gs[2, 1]); ax.axis('off')
  rws = []
  for name, m in (('보정 후 전체', post), ('직선', st), ('커브', cr)):
    if m.sum() > 5:
      mu, sd, rms, p95 = s(m); rws.append([name, f'{m.sum()}', f'{mu:+.2f}°', f'{sd:.2f}°', f'{rms:.2f}°', f'{p95:.1f}°'])
  tb = ax.table(cellText=rws, colLabels=['구간', '표본', '평균', '표준편차', 'RMS', '95%'],
                cellLoc='center', loc='center', colWidths=[.24, .13, .15, .16, .15, .13])
  tb.auto_set_font_size(False); tb.set_fontsize(10.5); tb.scale(1, 1.9)
  for (i, j), cell in tb.get_celld().items():
    cell.set_edgecolor('#ddd')
    if i == 0: cell.set_facecolor('#eceff1'); cell.set_text_props(weight='bold')
    elif rws and i <= len(rws) and rws[i-1][0] == '직선': cell.set_facecolor('#dbeafe'); cell.set_text_props(weight='bold')
  ax.set_title('⑤ 헤딩 정확도는 직선으로 말한다 — 커브는 측정 지연', fontsize=12, weight='bold', loc='left')

  name = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  fig.suptitle(f'GPS 이동방향으로 저가형 IMU 방위각 보정 — {name}', fontsize=15, weight='bold')
  out = a.out or f'/home/han/racing_ws/logs/heading_{name}.png'
  fig.savefig(out, dpi=125, bbox_inches='tight'); print(f'✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

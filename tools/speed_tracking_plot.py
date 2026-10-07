#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speed_tracking_plot.py — 종방향 목표속도 대비 실제속도를 시계열로 그린다.

★ 왜 이 도구가 필요한가
  "목표를 얼마나 잘 따라갔나" 를 보려면 **목표와 실측을 같은 축에** 놓아야
  하는데, drive.csv 의 어느 컬럼을 쓰느냐로 결론이 달라진다.

    target_v  /target_speed      ← 종방향 제어기가 정한 목표. **이게 목표다.**
    cmd_v     /cmd_vel           ← 먹스 출력. E-stop·teleop·회피가 끼면 달라진다
    v         /odometry twist    ← ⚠ 0.2초 지연하고 ±20 m/s 로 튄다. 쓰지 말 것
    enc_v     /current_speed_enc ← 엔코더 위치차분. **실측은 이것을 쓴다**

  2026-10-07 실측에서 v 는 −20.69~+2.99 로 튀었고 enc_v 는 0~2.35 였다.

사용:
  python3 tools/speed_tracking_plot.py data/2026-10-07/heading_1007_2034/drive.csv
  python3 tools/speed_tracking_plot.py <csv> --out /tmp/x.png --from 30 --to 200
"""
import argparse, csv, math, os, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

POS_WIN = 10        # 위치미분 창(샘플). 20Hz 기록 기준 0.5초


def col(rows, k):
  out = []
  for r in rows:
    try:
      out.append(float(r.get(k)))
    except (TypeError, ValueError):
      out.append(np.nan)
  return np.array(out)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--out', default=None)
  ap.add_argument('--from', dest='t0', type=float, default=None)
  ap.add_argument('--to', dest='t1', type=float, default=None)
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  if not rows:
    print('빈 파일'); return 1
  t = col(rows, 't')
  tv, cv = col(rows, 'target_v'), col(rows, 'cmd_v')
  ev, ov = col(rows, 'enc_v'), col(rows, 'v')
  X, Y = col(rows, 'x'), col(rows, 'y')
  curv, pwm = col(rows, 'curvature'), col(rows, 'drive_pwm')
  if np.all(np.isnan(tv)):
    print('❌ target_v 컬럼이 없다 — 2026-10-07 이전 기록이다.')
    print('   drive_record.py 를 갱신한 뒤 다시 주행해야 한다.')
    return 1

  # 위치미분 속도 — 엔코더와 독립인 '제3의 기준'
  sp = np.full(len(t), np.nan)
  for i in range(POS_WIN, len(t)):
    dt = t[i] - t[i - POS_WIN]
    if dt > 0:
      sp[i] = math.hypot(X[i] - X[i - POS_WIN], Y[i] - Y[i - POS_WIN]) / dt

  m = np.ones(len(t), bool)
  if a.t0 is not None: m &= t >= a.t0
  if a.t1 is not None: m &= t <= a.t1

  # 추종 오차 — **실제로 달린 구간만** 센다.
  # ⚠ 목표>0 만으로 거르면 안 된다. AUTO 진입 전/완주 후에도 종방향은
  #   목표를 계속 내보내는데 차는 서 있다. 2026-10-07 기록에서 그 정지
  #   구간이 80% 라, 거기까지 세면 평균오차가 −0.40 으로 나온다(실제는 +).
  #   위치미분으로 '움직였나' 를 독립 판정한다.
  moving = m & (sp > 0.15) & (tv > 0.05) & ~np.isnan(ev)
  run = moving
  err = ev[run] - tv[run]

  fig = plt.figure(figsize=(15, 10))
  gs = fig.add_gridspec(3, 2, height_ratios=[1.3, 1, 1], hspace=.38, wspace=.2)

  ax = fig.add_subplot(gs[0, :])
  ax.plot(t[m], tv[m], color='#dc2626', lw=2.0, label='목표 target_v (/target_speed)')
  ax.plot(t[m], ev[m], color='#2b6cb0', lw=1.2, label='실측 enc_v (엔코더)')
  ax.plot(t[m], sp[m], color='#16a34a', lw=0.9, alpha=.65, label='위치미분 (독립 기준)')
  if run.any():
    ax.axvspan(t[run].min(), t[run].max(), color='#fde68a', alpha=.25,
               label='주행 구간 (판정에 쓴 범위)')
  ax.set_title('① 종방향 — 목표 vs 실제', fontsize=13, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('속도 [m/s]'); ax.grid(alpha=.3)
  ax.legend(fontsize=9.5, loc='upper right')

  ax = fig.add_subplot(gs[1, 0])
  ax.plot(t[run], err, color='#7c3aed', lw=1.0)
  ax.axhline(0, color='#333', lw=1)
  ax.axhspan(-0.2, 0.2, color='#16a34a', alpha=.12)
  ax.set_title(f'② 추종 오차 (실측−목표) — 평균 {err.mean():+.2f} · RMS {np.sqrt((err**2).mean()):.2f} m/s',
               fontsize=11.5, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('오차 [m/s]'); ax.grid(alpha=.3)

  ax = fig.add_subplot(gs[1, 1])
  ax.hist(err, bins=60, color='#7c3aed', alpha=.8)
  ax.axvline(err.mean(), color='#dc2626', lw=2, ls='--', label=f'평균 {err.mean():+.2f}')
  ax.axvspan(-0.2, 0.2, color='#16a34a', alpha=.12)
  ax.set_title(f'③ 오차 분포 (n={len(err)})', fontsize=11.5, weight='bold')
  ax.set_xlabel('오차 [m/s]'); ax.set_ylabel('표본수'); ax.legend(fontsize=9); ax.grid(alpha=.3)

  ax = fig.add_subplot(gs[2, 0])
  ax.plot(t[m], pwm[m], color='#e76f51', lw=1.0)
  ax.set_title('④ 구동 PWM', fontsize=11.5, weight='bold')
  ax.set_xlabel('t [s]'); ax.set_ylabel('PWM'); ax.grid(alpha=.3)

  ax = fig.add_subplot(gs[2, 1])
  ok = m & ~np.isnan(curv) & ~np.isnan(tv)
  ax.scatter(np.abs(curv[ok]), tv[ok], s=4, alpha=.3, color='#2b6cb0')
  ax.set_title('⑤ 곡률 vs 목표속도 — 곡률 감속이 듣는가', fontsize=11.5, weight='bold')
  ax.set_xlabel('|곡률| [1/m]'); ax.set_ylabel('목표속도 [m/s]'); ax.grid(alpha=.3)

  name = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  fig.suptitle(f'종방향 속도 추종 — {name}', fontsize=15, weight='bold')
  out = a.out or f'/home/han/racing_ws/logs/speed_{name}.png'
  fig.savefig(out, dpi=125, bbox_inches='tight')

  print(f'표본 {len(err)} (목표>0.05 구간)')
  print(f'  평균오차 {err.mean():+.3f}  RMS {np.sqrt((err**2).mean()):.3f}  '
        f'표준편차 {err.std():.3f} m/s')
  print(f'  |오차|>0.3 비율 {100*(np.abs(err)>0.3).mean():.1f}%')
  print(f'✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

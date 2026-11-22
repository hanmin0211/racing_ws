#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speed_pi_sim.py — ROS 쪽 속도 PI(GPS 속도 피드백)를 '같은 지형' 에서 미리 돌려 본다.

★ 왜 (2026-10-09)
  10/8 실측으로 두 가지가 드러났다.
    · 보드 펌웨어에 PID 가 없다 — PWM 이 정확히 17.2+38.8·v (FF 전용 빌드)
    · 엔코더가 간헐적이다 — 움직이는데 카운트가 멈춘다
  그래서 실측은 목표를 **모양으로 전혀 안 따라갔다** (r=0.02). 평균만 맞았다.
  펌웨어 PID 는 엔코더 수리 + 굽기가 먼저라 당장 못 쓴다.
  → 엔코더 없이 **RTK 위치 미분 속도**를 피드백으로 쓰는 PI 를 ROS 쪽
    (serial_bridge, ff_mode:=ros)에 얹는다. 이 도구는 그걸 차 없이 시험한다.

모델 (pid_sim_realistic.py 와 같은 플랜트)
  dv/dt = K·PWM − C·v + d(s)        K=0.0175, C=0.861
  d(s) = 실주행에서 '모델이 설명 못 한 가속도' 를 경로 위치별로 역산한 것
         (경사·노면). 위치에 묶여 있으므로 다른 제어기로 달려도 같은 곳에 온다.

피드백 측정 (실측에서 잰 값 그대로)
  · 포즈 갱신 10 Hz (0.100 s)
  · 0.5 s 창 위치차분 → 지연 약 0.25 s, 잡음 표준편차 0.077 m/s
  ⚠ 지연이 PI 이득의 상한을 정한다. 이득을 올리면 지연 때문에 진동한다.

⚠ 한계: d(s) 를 그 주행에서 뽑았으므로 '개루프 재현' 은 구성상 맞는다(독립 검증
  아님). 비교 대상은 '같은 외란에서 제어기끼리' 다. 실차 확인이 최종이다.

사용:
  python3 tools/speed_pi_sim.py data/2026-10-08/pid_1008b_1727/drive.csv --to 84
"""
import argparse, csv, math, os, sys
import numpy as np
import yaml

K, C = 0.0175, 0.861
FW_STATIC, FW_GAIN = 17.2, 38.8          # 10/8 보드가 실제로 낸 FF
DT = 0.01
POSE_DT = 0.10                           # 포즈 갱신 주기 (실측)
WIN = 0.50                               # 위치차분 창
CTRL_DT = 0.05                           # serial_bridge 송신 주기 20 Hz
MAX_PWM = 160


def col(rows, k):
  o = []
  for r in rows:
    try: o.append(float(r.get(k)))
    except (TypeError, ValueError): o.append(np.nan)
  return np.array(o)


def path_s(wp):
  d = yaml.safe_load(open(wp))
  p = np.array([(float(a['x']), float(a['y'])) for a in d['waypoints']])
  s = np.r_[0, np.cumsum(np.hypot(*np.diff(p, axis=0).T))]
  return p, s


def project(px, py, P, S):
  """전진 전용 투영 — 8자 교차점에서 엉뚱한 구간에 붙지 않게."""
  out = np.full(len(px), np.nan)
  j = int(np.argmin(np.hypot(P[:, 0]-px[0], P[:, 1]-py[0])))
  for i in range(len(px)):
    hi = min(len(P)-1, j+40)
    j = j + int(np.argmin(np.hypot(P[j:hi+1, 0]-px[i], P[j:hi+1, 1]-py[i])))
    out[i] = S[j]
  return out


def smooth(a, n):
  k = np.ones(n) / n
  return np.convolve(np.pad(a, (n//2, n-1-n//2), mode='edge'), k, mode='valid')


def simulate(grid, tgt, dist, v0, ctrl, kp=0., ki=0., imax=0., brake=0., upmax=90.,
             noise=0.077, seed=0, Kx=K, dscale=1.0, tmax=300.):
  """ctrl: 'ff' = 보드 FF 개루프, 'pi' = FF + GPS 속도 PI."""
  rng = np.random.default_rng(seed)
  v, s, t = v0, grid[0], 0.0
  hist = []                 # (t, s) 포즈 표본 — 10 Hz
  next_pose, next_ctrl = 0.0, 0.0
  isum, pwm, vm = 0.0, 0.0, None
  T, S, V, TG, PW, VM = [], [], [], [], [], []
  while s < grid[-1] and t < tmax:
    tv = float(np.interp(s, grid, tgt))
    if t >= next_pose:
      hist.append((t, s)); next_pose += POSE_DT
      while len(hist) > 2 and t - hist[0][0] > WIN: hist.pop(0)
      if t - hist[0][0] >= WIN * 0.8:
        vm = (hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0]) + rng.normal(0, noise)
    if t >= next_ctrl:
      next_ctrl += CTRL_DT
      ff = FW_STATIC + FW_GAIN * tv if tv > 0.05 else 0.0
      if ctrl == 'pi' and vm is not None and tv > 0.05:
        e = tv - vm
        top = min(MAX_PWM, ff + upmax)        # serial_bridge 의 spd_up_max 와 같다
        cand = max(-imax, min(imax, isum + e * CTRL_DT))
        out = ff + kp * e + ki * cand
        # 조건부 적분 — 포화 방향으로 더 쌓지 않는다
        if not ((out > top and e > 0) or (out < -brake and e < 0)):
          isum = cand
        pwm = max(-brake, min(top, ff + kp * e + ki * isum))
      else:
        pwm = ff
    d = dscale * float(np.interp(s, grid, dist))
    v = max(0.0, v + (Kx * pwm - C * v + d) * DT)
    s += v * DT; t += DT
    T.append(t); S.append(s); V.append(v); TG.append(tv); PW.append(pwm)
    VM.append(np.nan if vm is None else vm)
  return {k: np.array(x) for k, x in zip('T S V TG PW VM'.split(), (T, S, V, TG, PW, VM))}


def stats(r):
  m = r['TG'] > 0.05
  e = r['V'][m] - r['TG'][m]
  return dict(rms=math.sqrt((e**2).mean()), bias=e.mean(),
              over=100*(np.abs(e) > 0.3).mean(),
              r=np.corrcoef(r['V'][m], r['TG'][m])[0, 1],
              chatter=np.std(np.diff(r['PW'][::5])), time=r['T'][-1],
              brake=100*(r['PW'] < 0).mean())


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--wp', default='/home/han/racing_ws/config/chungju_school/wp_heading_1007b_resampled_0.5.yaml')
  ap.add_argument('--to', type=float, default=None, help='이 시각 이후 버림(충돌 등)')
  ap.add_argument('--kp', type=float, default=80.0)
  ap.add_argument('--ki', type=float, default=60.0)
  ap.add_argument('--imax', type=float, default=2.5, help='적분 클램프 [m/s·s]')
  ap.add_argument('--brake', type=float, default=30.0, help='역 PWM 상한(감속 권한)')
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  c = lambda k: col(rows, k)
  t, X, Y, pwm, tv, yo = c('t'), c('x'), c('y'), c('drive_pwm'), c('target_v'), c('yaw_offset_deg')
  W = int(round(WIN / 0.05))
  sp = np.full(len(t), np.nan)
  sp[W:] = np.hypot(X[W:]-X[:-W], Y[W:]-Y[:-W]) / (t[W:]-t[:-W])
  m = ~np.isnan(yo) & (sp > 0.15) & ~np.isnan(pwm)
  if a.to is not None: m &= t <= a.to
  idx = np.where(m)[0]; i0, i1 = idx[0], idx[-1]
  sl = slice(i0, i1 + 1)
  t, X, Y, pwm, tv, sp = t[sl], X[sl], Y[sl], pwm[sl], tv[sl], sp[sl]
  # 위치미분은 창 끝에 붙어 있다 → 창 절반만큼 앞당겨 실제 시각에 맞춘다
  shift = W // 2
  vs = np.r_[sp[shift:], np.full(shift, sp[-1])]
  vs = smooth(np.nan_to_num(vs, nan=np.nanmean(vs)), 5)

  P, S = path_s(a.wp)
  s = project(X, Y, P, S)
  acc = np.gradient(smooth(vs, 9), t)
  d = acc - (K * pwm - C * vs)
  grid = np.arange(np.nanmin(s), np.nanmax(s), 0.5)
  def binned(arr):
    o = np.array([np.nanmean(arr[(s >= g) & (s < g+0.5)]) if ((s >= g) & (s < g+0.5)).any() else np.nan
                  for g in grid])
    ok = ~np.isnan(o); return np.interp(grid, grid[ok], o[ok])
  dist = smooth(binned(d), 4); tgt = binned(tv); vreal = binned(vs)
  v0 = vs[0]
  print(f'AUTO {t[0]:.1f}~{t[-1]:.1f}s · 경로 s={grid[0]:.1f}~{grid[-1]:.1f}m')
  print(f'외란 d(s) {dist.min():+.2f}~{dist.max():+.2f} m/s² (경사 환산 '
        f'{100*dist.min()/9.81:+.1f}~{100*dist.max()/9.81:+.1f}%)')

  e = vreal - tgt
  real = dict(rms=math.sqrt((e**2).mean()), bias=e.mean(), over=100*(np.abs(e) > .3).mean(),
              r=np.corrcoef(vreal, tgt)[0, 1])
  ff = simulate(grid, tgt, dist, v0, 'ff')
  pi = simulate(grid, tgt, dist, v0, 'pi', a.kp, a.ki, a.imax, a.brake)
  sf, spi = stats(ff), stats(pi)
  r_ff = np.corrcoef(np.interp(grid, ff['S'], ff['V']), vreal)[0, 1]

  print(f'\n{"":28s} {"RMS":>6s} {"편향":>7s} {"|e|>.3":>7s} {"모양r":>6s} {"소요":>6s}')
  print(f'{"실측 (보드 FF 개루프)":28s} {real["rms"]:6.3f} {real["bias"]:+7.3f} {real["over"]:6.1f}% {real["r"]:6.2f}')
  print(f'{"시뮬 FF 개루프":28s} {sf["rms"]:6.3f} {sf["bias"]:+7.3f} {sf["over"]:6.1f}% {sf["r"]:6.2f} {sf["time"]:5.1f}s'
        f'   실측 재현 r={r_ff:.2f}')
  print(f'{"시뮬 FF + GPS속도 PI":28s} {spi["rms"]:6.3f} {spi["bias"]:+7.3f} {spi["over"]:6.1f}% {spi["r"]:6.2f} {spi["time"]:5.1f}s')
  print(f'  PI 이득 kp={a.kp} ki={a.ki} imax={a.imax} 역PWM상한={a.brake} · '
        f'PWM 떨림 {spi["chatter"]:.1f} · 역PWM 비율 {spi["brake"]:.1f}% · 최대 {pi["PW"].max():.0f}')

  print('\n견고성 — 잡음·외란·모터이득·씨앗을 흔든다 (PI RMS / 모양r)')
  for name, kw in (('기준', {}), ('잡음 ×2', dict(noise=.154)), ('외란 ×1.5', dict(dscale=1.5)),
                   ('K −20%', dict(Kx=K*.8)), ('K +20%', dict(Kx=K*1.2)), ('씨앗 1', dict(seed=1)),
                   ('씨앗 2', dict(seed=2))):
    r1 = stats(simulate(grid, tgt, dist, v0, 'ff', **kw))
    r2 = stats(simulate(grid, tgt, dist, v0, 'pi', a.kp, a.ki, a.imax, a.brake, **kw))
    print(f'  {name:10s} FF {r1["rms"]:.3f}/{r1["r"]:+.2f}   PI {r2["rms"]:.3f}/{r2["r"]:+.2f}   떨림 {r2["chatter"]:.1f}')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig, ax = plt.subplots(3, 1, figsize=(15, 11), sharex=True, gridspec_kw=dict(height_ratios=[1.5, 1, 1]))
  ax[0].plot(grid, tgt, color='#dc2626', lw=2.2, label='목표 속도')
  ax[0].plot(grid, vreal, color='#94a3b8', lw=1.6, label=f'실측 (FF 개루프)  RMS {real["rms"]:.2f} · r={real["r"]:.2f}')
  ax[0].plot(pi['S'], pi['V'], color='#2b6cb0', lw=1.8, label=f'시뮬 FF+PI  RMS {spi["rms"]:.2f} · r={spi["r"]:.2f}')
  ax[0].set_ylabel('속도 [m/s]'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=10)
  ax[0].set_title('① 같은 지형에서 — 실측 개루프 vs GPS 속도 PI (시뮬)', fontsize=13, weight='bold')
  ax[1].plot(pi['S'], pi['PW'], color='#2b6cb0', lw=.9, label='PI PWM')
  ax[1].plot(ff['S'], ff['PW'], color='#e76f51', lw=1.2, ls='--', label='FF PWM')
  ax[1].axhline(0, color='#333', lw=.8); ax[1].set_ylabel('PWM'); ax[1].grid(alpha=.3); ax[1].legend(fontsize=9.5)
  ax[1].set_title('② 구동 PWM — 음수는 감속(역토크)', fontsize=12, weight='bold')
  ax[2].plot(grid, 100*dist/9.81, color='#7c3aed', lw=1.6); ax[2].axhline(0, color='#333', lw=.8)
  ax[2].set_ylabel('경사 환산 [%]'); ax[2].set_xlabel('경로 위치 s [m]'); ax[2].grid(alpha=.3)
  ax[2].set_title('③ 역산한 외란 d(s)', fontsize=12, weight='bold')
  name = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  out = a.out or f'/home/han/racing_ws/logs/speed_pi_sim_{name}.png'
  fig.savefig(out, dpi=115, bbox_inches='tight'); print(f'\n✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

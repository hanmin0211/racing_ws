#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pid_sim_realistic.py — 펌웨어 PID 를 '같은 지형' 에서 돌려 본다 (외란 재현).

★ pid_virtual_test.py 가 틀렸던 이유 (85e1f81) 와 여기서 고친 것
  ① 모델이 평지 가정이라 실측과 모양이 안 맞았다
     → 개루프 실주행에서 **경로 위치별로 모델이 설명 못 한 힘 d(s)** 를 역산해
       PID 시뮬에서 **같은 위치에** 넣는다. 경사·노면은 위치에 묶여 있다.
  ② 목표를 시간축으로 넣었다
     → 목표속도는 곡률 = **위치**에 묶여 있다. PID 가 느리면 커브에 늦게 닿는다.
       목표도 target_v(s) 로 위치에서 읽는다.
  ③ 엔코더 잡음을 가우스로 대충 넣었다
     → 펌웨어 그대로: 정수 카운트(290/회전) · 10ms 차분 · 5샘플 이동평균.
       1 카운트/10ms = 0.29 m/s 라 1.3 m/s 에서 4↔5 카운트를 오간다.
  ④ 초기속도를 0 에서 시작했다 → 실측 첫 속도에서 시작

★ ⚠ 한계 — 이건 '예측' 이 아니라 '같은 외란에서 제어기 비교' 다
  d(s) 를 개루프 주행에서 뽑았으므로 '개루프 재현' 은 구성상 잘 맞는다(독립 검증
  아님). 독립 검증은 다음 실주행에서 pitch_deg 로 잰 경사 g·sin(θ) 와 d(s) 를
  대조해야 한다.

사용:
  python3 tools/pid_sim_realistic.py data/2026-10-07/heading_1007_2034/drive.csv
"""
import argparse, csv, math, os, sys
import numpy as np
import yaml

# ── 펌웨어 원본 (henes_firmware.ino) ──────────────────────────────────
DT = 0.010
STATIC_FF, FF_GAIN = 17.2, 38.8
KP, KI, KD = 30.0, 15.0, 0.5
ISUM_CLAMP = 8.0
MAX_PWM = 160
ACCEL = 0.8
CPR = 290                        # |counts_per_revolution|
WHEEL_R = 0.1327
M_PER_COUNT = 2 * math.pi * WHEEL_R / CPR
VEL_AVG_N = 5
# ── ROS 개루프 FF (실주행 설정) ───────────────────────────────────────
ROS_GAIN, ROS_STATIC = 42.6, 46.0
# ── 플랜트 dv/dt = K·PWM − C·v − F0 + d(s) ───────────────────────────
C = 0.861


def col(rows, k):
  o = []
  for r in rows:
    try: o.append(float(r.get(k)))
    except (TypeError, ValueError): o.append(np.nan)
  return np.array(o)


def path_s(wp_file):
  d = yaml.safe_load(open(wp_file))
  p = np.array([(float(a['x']), float(a['y'])) for a in d['waypoints']])
  s = np.zeros(len(p)); s[1:] = np.cumsum(np.hypot(*np.diff(p, axis=0).T))
  return p, s


def project(px, py, P, S):
  """전진 전용 투영 — 8자 코스라 교차점에서 엉뚱한 구간에 붙지 않게."""
  out = np.full(len(px), np.nan); j = int(np.argmin(np.hypot(P[:, 0]-px[0], P[:, 1]-py[0])))
  for i in range(len(px)):
    lo, hi = j, min(len(P)-1, j+40)
    j = lo + int(np.argmin(np.hypot(P[lo:hi+1, 0]-px[i], P[lo:hi+1, 1]-py[i])))
    out[i] = S[j]
  return out


def smooth(a, n):
  if n <= 1: return a
  k = np.ones(n) / n
  return np.convolve(np.pad(a, (n//2, n-1-n//2), mode='edge'), k, mode='valid')


class Encoder:
  """펌웨어 calculate_velocity() 그대로."""
  def __init__(self):
    self.pos = 0.0; self.cnt = 0; self.prev = 0; self.buf = [0.0]*VEL_AVG_N; self.i = 0
  def step(self, v):
    self.pos += v * DT
    self.cnt = int(math.floor(self.pos / M_PER_COUNT))
    d = self.cnt - self.prev; self.prev = self.cnt
    self.buf[self.i] = d * M_PER_COUNT / DT; self.i = (self.i+1) % VEL_AVG_N
    return sum(self.buf) / VEL_AVG_N


def simulate(s_tab, tgt_tab, d_tab, s0, s1, v0, K, F0, mode, dscale=1.0, tmax=300):
  enc = Encoder(); v = v0; s = s0; cmd = v0; isum = 0.0; eold = 0.0
  T, Sv, V, TG, PW, MEAS = [], [], [], [], [], []
  t = 0.0
  while s < s1 and t < tmax:
    tv = float(np.interp(s, s_tab, tgt_tab))
    meas = enc.step(v)
    if mode == 'ros':
      p = ROS_GAIN * tv + ROS_STATIC if tv > 0.05 else 0.0
    else:
      md = ACCEL * DT
      cmd = min(tv, cmd + md) if tv > cmd else max(tv, cmd - md)
      err = cmd - meas; ed = (err - eold) / DT
      ff = STATIC_FF + FF_GAIN * cmd if cmd > 0.02 else 0.0
      ts = max(-ISUM_CLAMP, min(ISUM_CLAMP, isum + err * DT))
      tout = ff + KP*err + KI*ts + KD*ed
      if not ((tout > MAX_PWM and err > 0) or (tout < -MAX_PWM and err < 0)): isum = ts
      p = ff + KP*err + KI*isum + KD*ed
      if abs(tv) < 0.05 and abs(cmd) < 0.05: p = 0.0; isum = 0.0
      eold = err
    p = max(-MAX_PWM, min(MAX_PWM, p))
    dist = dscale * float(np.interp(s, s_tab, d_tab))
    v = max(0.0, v + (K*p - C*v - F0 + dist) * DT)
    s += v * DT; t += DT
    T.append(t); Sv.append(s); V.append(v); TG.append(tv); PW.append(p); MEAS.append(meas)
  return tuple(map(np.array, (T, Sv, V, TG, PW, MEAS)))


def stats(v, tg):
  m = tg > 0.05; e = v[m] - tg[m]
  return math.sqrt((e**2).mean()), e.mean(), 100*(np.abs(e) > 0.3).mean()


def main():
  ap = argparse.ArgumentParser(); ap.add_argument('csv')
  ap.add_argument('--wp', default='/home/han/racing_ws/config/chungju_school/wp_heading_1007b_final.yaml')
  ap.add_argument('--K', type=float, default=0.0175)
  ap.add_argument('--F0', type=float, default=0.0)
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  c = lambda k: col(rows, k)
  t, X, Y, ev, pwm, cv, tv, yo = (c('t'), c('x'), c('y'), c('enc_v'), c('drive_pwm'),
                                  c('cmd_v'), c('target_v'), c('yaw_offset_deg'))
  ok = ~np.isnan(t) & ~np.isnan(ev) & ~np.isnan(pwm) & ~np.isnan(cv) & ~np.isnan(X)
  t, X, Y, ev, pwm, cv, tv, yo = [z[ok] for z in (t, X, Y, ev, pwm, cv, tv, yo)]
  # AUTO 구간만 — 캘리브(cmd_v≠target_v) 제외
  k = int(np.argmax(~np.isnan(yo))); a0 = k + int(np.argmax(cv[k:] > 0.05))
  e = len(ev) - int(np.argmax(ev[::-1] > 0.1))
  t, X, Y, ev, pwm, cv = [z[a0:e] for z in (t, X, Y, ev, pwm, cv)]

  P, S = path_s(a.wp)
  s = project(X, Y, P, S)
  # 외란 역산 d = a_meas − (K·PWM − C·v − F0)
  acc = np.gradient(smooth(ev, 9), t)
  d = acc - (a.K*pwm - C*ev - a.F0)
  # s 축 0.5m 격자로 모으고 2m 로 평활
  grid = np.arange(np.nanmin(s), np.nanmax(s), 0.5)
  def binned(arr):
    o = np.array([np.nanmean(arr[(s >= g) & (s < g+0.5)]) if ((s >= g) & (s < g+0.5)).any() else np.nan for g in grid])
    okk = ~np.isnan(o); return np.interp(grid, grid[okk], o[okk])
  d_tab = smooth(binned(d), 4); tgt_tab = binned(cv); v_real_s = binned(ev)
  s0, s1, v0 = grid[0], grid[-1], ev[0]

  print(f'AUTO 구간 {t[-1]-t[0]:.0f}초 · 경로 s={s0:.1f}~{s1:.1f}m ({s1-s0:.1f}m)')
  print(f'외란 d(s) 범위 {d_tab.min():+.2f} ~ {d_tab.max():+.2f} m/s²  '
        f'(경사 환산 {100*d_tab.min()/9.81:+.1f} ~ {100*d_tab.max()/9.81:+.1f} %)')

  real = stats(v_real_s, tgt_tab)
  To, So, Vo, TGo, Po, _ = simulate(grid, tgt_tab, d_tab, s0, s1, v0, a.K, a.F0, 'ros')
  ol = stats(Vo, TGo)
  vo_on = np.interp(grid, So, Vo); r_ol = np.corrcoef(vo_on, v_real_s)[0, 1]
  Tf, Sf, Vf, TGf, Pf, Mf = simulate(grid, tgt_tab, d_tab, s0, s1, v0, a.K, a.F0, 'fw')
  fw = stats(Vf, TGf)

  print(f'\n{"":30s} {"RMS":>7s} {"편향":>8s} {"|e|>0.3":>8s} {"소요":>6s}')
  print(f'{"실측 개루프":30s} {real[0]:6.3f} {real[1]:+7.3f} {real[2]:7.1f}% {t[-1]-t[0]:5.0f}s')
  print(f'{"시뮬 개루프 (+외란)":30s} {ol[0]:6.3f} {ol[1]:+7.3f} {ol[2]:7.1f}% {To[-1]:5.0f}s   모양 r={r_ol:.2f}')
  print(f'{"시뮬 펌웨어 PID (+외란·양자화)":30s} {fw[0]:6.3f} {fw[1]:+7.3f} {fw[2]:7.1f}% {Tf[-1]:5.0f}s')
  print(f'\n  ⚠ 개루프 재현(r={r_ol:.2f})은 외란을 그 주행에서 뽑았으니 구성상 맞는 것 — 독립 검증 아님')
  dpw = np.diff(Pf); print(f'  PID PWM 떨림 (스텝간 표준편차) {np.std(dpw):.1f} · 포화 {100*(np.abs(Pf)>=MAX_PWM-0.5).mean():.1f}%')
  q = Mf[Vf > 0.5] - Vf[Vf > 0.5]
  print(f'  펌웨어 속도 양자화 잡음 표준편차 {np.std(q):.3f} m/s')

  print('\n민감도 — 외란 크기·모터이득을 흔들어도 PID 가 나은가')
  print(f'  {"조건":24s} {"개루프":>7s} {"PID":>7s} {"감소":>6s}')
  for name, ds, Kx in (('외란 ×0.5', .5, a.K), ('외란 ×1.0 (기준)', 1, a.K), ('외란 ×1.5', 1.5, a.K),
                       ('K −20%', 1, a.K*.8), ('K +20%', 1, a.K*1.2)):
    _, _, vo2, tg2, _, _ = simulate(grid, tgt_tab, d_tab, s0, s1, v0, Kx, a.F0, 'ros', ds)
    _, _, vf2, tf2, _, _ = simulate(grid, tgt_tab, d_tab, s0, s1, v0, Kx, a.F0, 'fw', ds)
    r1, r2 = stats(vo2, tg2)[0], stats(vf2, tf2)[0]
    print(f'  {name:24s} {r1:6.3f}  {r2:6.3f} {100*(1-r2/r1):5.0f}%')

  import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig, ax = plt.subplots(4, 1, figsize=(15, 14), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.4, 1, 1, .8]))
  ax[0].plot(grid, tgt_tab, color='#dc2626', lw=2.2, label='목표 (위치별)')
  ax[0].plot(grid, v_real_s, color='#94a3b8', lw=1.6, label=f'실측 개루프  RMS {real[0]:.2f}')
  ax[0].plot(So, Vo, color='#e76f51', lw=1.3, ls='--', label=f'시뮬 개루프  RMS {ol[0]:.2f} (r={r_ol:.2f})')
  ax[0].plot(Sf, Vf, color='#2b6cb0', lw=1.8, label=f'시뮬 펌웨어 PID  RMS {fw[0]:.2f}')
  ax[0].set_ylabel('속도 [m/s]'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=9.5)
  ax[0].set_title('① 같은 지형(외란 재현)에서 — 개루프 vs 펌웨어 PID', fontsize=13, weight='bold')
  ax[1].plot(grid, v_real_s-tgt_tab, color='#94a3b8', lw=1.4, label='실측 개루프')
  ax[1].plot(Sf, Vf-TGf, color='#2b6cb0', lw=1.4, label='시뮬 PID')
  ax[1].axhspan(-.2, .2, color='#16a34a', alpha=.12); ax[1].axhline(0, color='#333', lw=.8)
  ax[1].set_ylabel('오차 [m/s]'); ax[1].grid(alpha=.3); ax[1].legend(fontsize=9.5)
  ax[1].set_title('② 추종 오차', fontsize=12, weight='bold')
  ax[2].plot(Sf, Pf, color='#2b6cb0', lw=.6, label='시뮬 PID PWM (양자화 잡음 포함)')
  ax[2].plot(So, Po, color='#e76f51', lw=1.2, ls='--', label='시뮬 개루프 PWM')
  ax[2].axhline(MAX_PWM, color='#dc2626', ls='--', lw=1)
  ax[2].set_ylabel('PWM'); ax[2].grid(alpha=.3); ax[2].legend(fontsize=9.5)
  ax[2].set_title('③ PWM', fontsize=12, weight='bold')
  ax[3].plot(grid, 100*d_tab/9.81, color='#7c3aed', lw=1.6)
  ax[3].axhline(0, color='#333', lw=.8)
  ax[3].set_ylabel('경사 환산 [%]'); ax[3].set_xlabel('경로 위치 s [m]'); ax[3].grid(alpha=.3)
  ax[3].set_title('④ 역산한 외란 d(s) — 다음 주행의 pitch_deg 와 대조해 검증할 것', fontsize=12, weight='bold')
  name = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  out = a.out or f'/home/han/racing_ws/logs/pid_realistic_{name}.png'
  fig.savefig(out, dpi=115, bbox_inches='tight'); print(f'\n✅ {out}')
  np.savez('/home/han/racing_ws/logs/disturbance_' + name + '.npz', s=grid, d=d_tab, target=tgt_tab)
  return 0


if __name__ == '__main__':
  sys.exit(main())

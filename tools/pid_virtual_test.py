#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pid_virtual_test.py — 차를 굴리지 않고 '펌웨어 PID 로 바꾸면 어떻게 되나' 를 본다.

★ 왜 rosbag 재생이 아니라 모델 시뮬인가
  rosbag 은 **이미 일어난 일**이다. 녹화된 enc_v 는 개루프 FF 의 PWM 에 차가
  반응한 결과라, 제어기를 바꿔 끼워도 차가 그 명령을 받은 적이 없으니
  반응이 안 바뀐다. 제어기 교체 효과는 녹화본에 없다.
  → 녹화본에서 **목표속도 궤적만** 빌려 오고, 차는 모델로 대신한다.

★ ⚠⚠ 2026-10-07 정정 — 이 도구의 PID 숫자를 그대로 믿지 말 것
  heading_1007_2034 에서 처음엔 '시뮬 개루프 RMS 0.798 vs 실측 0.806 → 재현됨'
  이라 판정했는데 **틀렸다.**
    · 캘리브 구간(cmd_v 0.5)을 목표 1.29 로 비교했다 — AUTO 가 아니었다
    · AUTO 만 다시 보니 시뮬 개루프 vs 실측 **모양 상관 r = 0.03**
      (초반 과속과 후반 과소가 상쇄돼 RMS 만 우연히 맞았다)
  → RMS 일치만으로 '재현' 이라 하지 말 것. **상관계수를 같이 볼 것.**
  → 모델이 평지 가정이라 경사(학교 코스 15.4%)를 못 따라간다. drive.csv 에
    IMU 피치가 없어 반영할 수도 없다.
  믿을 수 있는 것: 실측 개루프 오차, 그리고 'PID 적분항이 일정 편향을 없앤다'
  는 방향. 믿을 수 없는 것: PID 가 몇 m/s 까지 줄인다는 숫자.

★ 믿을 수 있나 — 먼저 모델을 검증한다
  녹화된 PWM 을 모델에 그대로 넣어 나온 속도가 녹화된 enc_v 와 겹치는지 본다.
  그다음 '개루프 시뮬' 이 실측 추종오차를 재현하는지 본다. 둘 다 맞아야
  PID 예측을 믿는다. 안 맞으면 PID 숫자는 의미 없다.

★ 펌웨어 PID 는 henes_firmware.ino 를 **그대로** 옮겼다
    100Hz · FF = 17.2 + 38.8·v · kp 30 · ki 15 · kd 0.5
    적분 클램프 ±8 · 조건부 안티와인드업 · MAX_DRIVE_PWM 160 · 가속제한 0.8 m/s²

사용:
  python3 tools/pid_virtual_test.py data/2026-10-07/heading_1007_2034/drive.csv
"""
import argparse, csv, math, os, sys
import numpy as np

# ── 펌웨어 원본 상수 (henes_firmware.ino) ──────────────────────────────
FW_DT = 0.010                     # CONTROL_DT_MS 10
FW_STATIC_FF, FW_FF_GAIN = 17.2, 38.8
FW_KP, FW_KI, FW_KD = 30.0, 15.0, 0.5
FW_ISUM_CLAMP = 8.0
FW_MAX_PWM = 160
FW_ACCEL = 0.8                    # ACCEL_LIMIT = BRAKE_LIMIT
# ── ROS 쪽 개루프 FF (지금 주행 설정) ──────────────────────────────────
ROS_FF_GAIN, ROS_FF_STATIC = 42.6, 46.0
# ── 플랜트: dv/dt = K·PWM − C·v − F0 (C 는 저장소 검증값 고정) ─────────
C_FIXED = 0.861


def col(rows, k):
  o = []
  for r in rows:
    try: o.append(float(r.get(k)))
    except (TypeError, ValueError): o.append(np.nan)
  return np.array(o)


def plant_step(v, pwm, K, F0, dt):
  return max(0.0, v + (K * pwm - C_FIXED * v - F0) * dt)


def fit_plant(t, ev, pwm):
  """녹화 PWM 을 그대로 넣었을 때 녹화 enc_v 를 가장 잘 재현하는 K·F0."""
  def run(K, F0):
    v = np.zeros(len(t)); v[0] = ev[0]
    for i in range(1, len(t)):
      v[i] = plant_step(v[i-1], pwm[i-1], K, F0, t[i] - t[i-1])
    return v
  best = None
  for K in np.arange(0.010, 0.040, 0.0005):
    for F0 in np.arange(0.0, 1.5, 0.02):
      e = run(K, F0) - ev; rr = math.sqrt((e**2).mean())
      if best is None or rr < best[0]:
        best = (rr, K, F0, run(K, F0))
  return best


def simulate(tq, target, K, F0, mode, noise, seed=0):
  """100Hz 로 돌린다. mode: 'ros' 개루프 / 'fw' 펌웨어 FF+PID."""
  rng = np.random.default_rng(seed)
  t = np.arange(tq[0], tq[-1], FW_DT)
  tgt = np.interp(t, tq, target)
  v = np.zeros(len(t)); pwm = np.zeros(len(t))
  cmd = 0.0; isum = 0.0; eold = 0.0
  for i in range(1, len(t)):
    tv = tgt[i]
    if mode == 'ros':
      p = ROS_FF_GAIN * tv + ROS_FF_STATIC if tv > 0.05 else 0.0
    else:
      # 가속 제한(소프트스타트) → commanded_velocity
      md = FW_ACCEL * FW_DT
      cmd = min(tv, cmd + md) if tv > cmd else max(tv, cmd - md)
      meas = v[i-1] + (rng.normal(0, noise) if noise else 0.0)
      err = cmd - meas
      ed = (err - eold) / FW_DT
      ff = FW_STATIC_FF + FW_FF_GAIN * cmd if cmd > 0.02 else 0.0
      ts = max(-FW_ISUM_CLAMP, min(FW_ISUM_CLAMP, isum + err * FW_DT))
      tout = ff + FW_KP * err + FW_KI * ts + FW_KD * ed
      sat_p = tout > FW_MAX_PWM and err > 0
      sat_n = tout < -FW_MAX_PWM and err < 0
      if not sat_p and not sat_n: isum = ts
      p = ff + FW_KP * err + FW_KI * isum + FW_KD * ed
      if abs(tv) < 0.05 and abs(cmd) < 0.05: p = 0.0; isum = 0.0
      eold = err
    p = max(-FW_MAX_PWM, min(FW_MAX_PWM, p))
    pwm[i] = p
    v[i] = plant_step(v[i-1], p, K, F0, FW_DT)
  return t, tgt, v, pwm


def track(t, tgt, v):
  m = tgt > 0.05
  e = v[m] - tgt[m]
  return math.sqrt((e**2).mean()), e.mean(), 100 * (np.abs(e) > 0.3).mean()


def main():
  ap = argparse.ArgumentParser(); ap.add_argument('csv')
  ap.add_argument('--noise', type=float, default=0.05,
                  help='엔코더 피드백 잡음 표준편차 [m/s]')
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  t = col(rows, 't'); ev = col(rows, 'enc_v'); pwm = col(rows, 'drive_pwm')
  tv = col(rows, 'target_v')
  ok = ~np.isnan(t) & ~np.isnan(ev) & ~np.isnan(pwm) & ~np.isnan(tv)
  t, ev, pwm, tv = t[ok], ev[ok], pwm[ok], tv[ok]
  i0 = int(np.argmax(ev > 0.1)); i1 = len(ev) - int(np.argmax(ev[::-1] > 0.1))
  t, ev, pwm, tv = t[i0:i1], ev[i0:i1], pwm[i0:i1], tv[i0:i1]
  if len(t) < 50:
    print('❌ 주행 구간이 너무 짧다'); return 1

  # ① 모델 검증 — 녹화 PWM 재생
  rr, K, F0, vfit = fit_plant(t, ev, pwm)
  R2 = 1 - (rr / ev.std())**2
  print(f'① 모델 검증  K={K:.4f} F0={F0:.2f} C={C_FIXED}(고정)')
  print(f'   녹화 PWM 재생 → 녹화 enc_v 대비 RMS {rr:.3f} m/s · R² {R2:.2f}')

  # ② 실측 추종오차
  er, eb, ep = track(t, tv, ev)
  # ③ 시뮬 — 개루프 / 펌웨어 PID
  ts_, tg_, vr, pr = simulate(t, tv, K, F0, 'ros', 0)
  rr_, rb_, rp_ = track(ts_, tg_, vr)
  tsf, tgf, vf, pf = simulate(t, tv, K, F0, 'fw', a.noise)
  fr, fb, fp = track(tsf, tgf, vf)

  print(f'\n{"":24s} {"RMS":>7s} {"편향":>8s} {"|오차|>0.3":>10s}')
  print(f'{"② 실측 (개루프)":24s} {er:6.3f} {eb:+7.3f} {ep:9.1f}%')
  print(f'{"③ 시뮬 개루프":24s} {rr_:6.3f} {rb_:+7.3f} {rp_:9.1f}%   ← ②를 재현해야 믿는다')
  print(f'{"④ 시뮬 펌웨어 PID":24s} {fr:6.3f} {fb:+7.3f} {fp:9.1f}%   ← 예측')
  vr_on = np.interp(t, ts_, vr)
  rcorr = float(np.corrcoef(vr_on[tv > 0.05], ev[tv > 0.05])[0, 1])
  ok_rms = abs(rr_ - er) < 0.25
  ok_shape = rcorr > 0.8
  print(f'\n개루프 재현  RMS 차 {abs(rr_-er):.3f} m/s · 모양 상관 r={rcorr:.2f}')
  if ok_rms and ok_shape:
    print('  ✅ 크기·모양 모두 재현 — PID 예측을 참고할 수 있다')
  else:
    print('  ❌ 모양이 안 맞는다 — PID 숫자는 믿지 말 것 (RMS 만 맞는 건 우연일 수 있다)')
  print('  ⚠ 캘리브 구간(cmd_v≠target_v)이 섞이면 비교가 틀어진다 — AUTO 구간만 쓸 것')
  print(f'PWM 포화(≥{FW_MAX_PWM}) 비율 {100*(np.abs(pf)>=FW_MAX_PWM-0.5).mean():.1f}%')

  # 그림
  import matplotlib; matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  plt.rcParams['font.family'] = 'Noto Sans CJK JP'; plt.rcParams['axes.unicode_minus'] = False
  fig, ax = plt.subplots(3, 1, figsize=(15, 11), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.3, 1, 1]))
  ax[0].plot(t, tv, color='#dc2626', lw=2.2, label='목표 target_v (실측 궤적)')
  ax[0].plot(t, ev, color='#94a3b8', lw=1.0, label=f'실측 enc_v (개루프) RMS {er:.2f}')
  ax[0].plot(ts_, vr, color='#e76f51', lw=1.4, ls='--', label=f'시뮬 개루프 RMS {rr_:.2f}')
  ax[0].plot(tsf, vf, color='#2b6cb0', lw=1.6, label=f'시뮬 펌웨어 PID RMS {fr:.2f}')
  ax[0].set_ylabel('속도 [m/s]'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=9.5)
  ax[0].set_title('① 같은 목표속도 — 개루프 vs 펌웨어 PID', fontsize=13, weight='bold')
  ax[1].plot(t, ev - tv, color='#94a3b8', lw=1.0, label='실측 개루프')
  ax[1].plot(tsf, vf - tgf, color='#2b6cb0', lw=1.3, label='시뮬 PID')
  ax[1].axhspan(-0.2, 0.2, color='#16a34a', alpha=.12); ax[1].axhline(0, color='#333', lw=.8)
  ax[1].set_ylabel('오차 [m/s]'); ax[1].grid(alpha=.3); ax[1].legend(fontsize=9.5)
  ax[1].set_title('② 추종 오차', fontsize=12, weight='bold')
  ax[2].plot(t, pwm, color='#94a3b8', lw=1.0, label='실측 PWM (개루프)')
  ax[2].plot(tsf, pf, color='#2b6cb0', lw=1.0, label='시뮬 PID PWM')
  ax[2].axhline(FW_MAX_PWM, color='#dc2626', ls='--', lw=1, label=f'포화 {FW_MAX_PWM}')
  ax[2].set_ylabel('PWM'); ax[2].set_xlabel('t [s]'); ax[2].grid(alpha=.3); ax[2].legend(fontsize=9.5)
  ax[2].set_title('③ PWM — PID 가 얼마나 일하나', fontsize=12, weight='bold')
  fig.suptitle(f'가상 시험 — 펌웨어 PID (모델 R² {R2:.2f}, 피드백 잡음 {a.noise} m/s)',
               fontsize=15, weight='bold')
  name = os.path.basename(os.path.dirname(os.path.abspath(a.csv)))
  out = a.out or f'/home/han/racing_ws/logs/pid_virtual_{name}.png'
  fig.savefig(out, dpi=120, bbox_inches='tight'); print(f'\n✅ {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

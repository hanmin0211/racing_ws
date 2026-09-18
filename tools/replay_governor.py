#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay_governor.py — 기록된 주행을 **실제 추정기·거버너에 그대로 먹여** 재생한다.

★ 왜
  거버너를 고칠 때마다 경사로에 차를 올릴 수는 없다. 시험(test_governor.py)은
  합성 입력이라 '내가 상상한 잡음'만 본다. 진짜 잡음은 로그에 있다.

  이 도구는 ramp_test.py 가 남긴 CSV 의 enc 열을 serial_bridge_node 의
  _enc_pos_update() 에 그대로 먹이고, 매 표본에서 _governor_pwm() 을 불러
  **그때 거버너가 뭘 했을지**를 다시 계산한다.

⚠ 로그 재샘플링 함정 (2026-09-17 에 한 번 속았다)
  ramp_test 는 41Hz 로 기록하는데 /encoder_count 는 20Hz 로 온다. 그래서
  연속 행의 61% 가 **같은 카운트**다. 그걸 새 표본으로 먹이면 창 안에
  가짜 정지구간이 생겨 가속도가 ±8 로 요동친다 — 노드에서는 안 일어나는
  일이다. 기본값은 --dedupe(카운트가 바뀐 행만) 이며, 이게 노드와 같다.

  python3 tools/replay_governor.py data/2026-09-17-ramp/ramp_5키로_2338.csv --speed 1.11
"""

import argparse
import csv
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))
from velocity_controller.serial_bridge_node import SerialBridgeNode as N  # noqa: E402


class _Clk:
  def __init__(self): self.t = 0.0
  def now(self): return type('', (), {'nanoseconds': self.t * 1e9})()


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass


class Rig:
  """serial_bridge_node 의 상태만 흉내 낸 껍데기. 로직은 원본을 그대로 쓴다."""
  for _k in ('ENC_M_PER_COUNT', 'ENC_FORWARD_SIGN', 'ENC_JUMP_COUNTS',
             'ENC_WIN_S', 'ENC_ACC_CAP', 'ENC_ACC_TAU', 'GOV_LEAD_CAP',
             'BRAKE_MEAS_FRESH_S', 'BRAKE_V_PLAUSIBLE'):
    locals()[_k] = getattr(N, _k)
  del _k

  def __init__(self, gov_pwm, gov_deadband, gov_gain, gov_lead_s):
    self.gov_pwm = gov_pwm
    self.gov_deadband = gov_deadband
    self.gov_gain = gov_gain
    self.gov_lead_s = gov_lead_s
    self.ff_deadband = 0.05
    self._enc_hist = []
    self._enc_last_c = None
    self._enc_rej = 0
    self._meas_v = None
    self._meas_v_t = 0.0
    self._meas_a = 0.0
    self._meas_a_t = 0.0
    self._meas_src = None
    self.encv_pub = None
    self._clk = _Clk()

  def get_clock(self): return self._clk
  def get_logger(self): return _Log()


FF_GAIN, FF_STATIC = 38.8, 17.2      # 실측 FF (ff-open-loop-measured-yongin)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('csv')
  ap.add_argument('--speed', type=float, default=1.11,
                  help='목표속도 [m/s] (기본 1.11 = 4km/h)')
  ap.add_argument('--gov-pwm', type=float, default=60.0)
  ap.add_argument('--gov-deadband', type=float, default=0.10)
  ap.add_argument('--gov-gain', type=float, default=300.0)
  ap.add_argument('--gov-lead', type=float, default=0.30)
  ap.add_argument('--raw', action='store_true',
                  help='중복 행도 그대로 먹인다 (기본은 노드와 같은 dedupe)')
  ap.add_argument('--every', type=float, default=0.2, help='출력 간격 [s]')
  a = ap.parse_args()

  rows = list(csv.DictReader(open(a.csv)))
  if not rows:
    print('빈 파일'); return 2

  rig = Rig(a.gov_pwm, a.gov_deadband, a.gov_gain, a.gov_lead)
  ff = FF_GAIN * a.speed + FF_STATIC

  print(f'재생 {os.path.basename(a.csv)} · {len(rows)}행')
  print(f'목표 {a.speed:.2f} m/s ({a.speed * 3.6:.1f} km/h) · FF PWM {ff:.0f}')
  print(f'거버너 상한 {a.gov_pwm:.0f} · 불감대 {a.gov_deadband:.2f} · '
        f'이득 {a.gov_gain:.0f} · 선행 {a.gov_lead:.2f}s'
        + ('  [raw: 중복 포함]' if a.raw else ''))
  print()

  rec, last_c = [], None
  for r in rows:
    c = int(float(r['enc']))
    if not a.raw and c == last_c:
      continue
    last_c = c
    t = float(r['t'])
    rig._clk.t = t
    N._enc_pos_update(rig, c)
    gov = N._governor_pwm(rig, a.speed, ff, t)
    rec.append((t, float(r['v']), rig._meas_v, rig._meas_a,
                float(r['pwm']), gov))

  print('  t    펌VEL  위치차분  가속도 | 그때PWM  거버너였다면')
  last = -9.0
  for t, fw, mv, ma, pwm, gov in rec:
    if t - last < a.every:
      continue
    last = t
    print('%5.2f %6.2f %8s %7s | %6.0f %12s'
          % (t, fw, '—' if mv is None else '%.2f' % mv,
             '—' if mv is None else '%+.2f' % ma, pwm,
             '개입안함' if gov is None else '%.0f' % gov))

  vals = [(fw, mv, ma) for _, fw, mv, ma, _, _ in rec if mv is not None]
  if len(vals) < 3:
    print('\n표본 부족'); return 1

  def jit(xs):
    return statistics.mean(abs(xs[i] - xs[i - 1]) for i in range(1, len(xs)))

  jf, jp = jit([x[0] for x in vals]), jit([x[1] for x in vals])
  ja = jit([x[2] for x in vals])
  print()
  print('■ 측정 품질 — 표본간 평균 변동 (작을수록 매끄럽다)')
  print('  펌웨어 VEL  %.3f m/s   (최대 %.2f)'
        % (jf, max(x[0] for x in vals)))
  print('  위치차분    %.3f m/s   (최대 %.2f)   %s'
        % (jp, max(x[1] for x in vals),
           '✅ %.1f배 매끄럽다' % (jf / jp) if jp > 0 and jf > jp
           else '⚠ 개선 없음'))
  print('  가속도      %.2f m/s²  (범위 %+.1f ~ %+.1f)'
        % (ja, min(x[2] for x in vals), max(x[2] for x in vals)))

  govs = [g for _, _, _, _, _, g in rec if g is not None]
  n_on = len(govs)
  print()
  print('■ 거버너 — 이 로그를 다시 돌렸다면')
  print('  개입 %d / %d 표본 (%.0f%%)' % (n_on, len(rec), 100.0 * n_on / len(rec)))
  if govs:
    t_on = next(t for t, _, _, _, _, g in rec if g is not None)
    print('  최초 개입 t=%.2fs · PWM %.0f ~ %.0f (중앙 %.0f)'
          % (t_on, min(govs), max(govs), statistics.median(govs)))
    brake = [g for g in govs if g < 0]
    print('  제동(역PWM) %d 표본 · 최대 %.0f'
          % (len(brake), min(brake) if brake else 0))
  else:
    print('  ⚠ 한 번도 개입 안 함 — 목표속도나 불감대를 확인할 것')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

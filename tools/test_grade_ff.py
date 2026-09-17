#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_grade_ff.py — IMU 경사 보상을 **하드웨어 없이** 검증한다.

★ 왜 이 시험이 특히 중요한가
  경사 보상은 피치에 비례해 PWM 을 더한다. **부호가 뒤집히면 내리막에서
  가속한다.** 개루프에는 그걸 막을 것이 거버너뿐이고, 거버너는 이미
  과속한 뒤에야 개입한다. 즉 부호 오류 = 폭주다.

  거버너에서 실제로 겪었다: math.copysign(-80, +1) 이 +80 이라,
  제동해야 할 때 최대 가속을 낼 뻔했다. 시험이 잡았다.

  serial_bridge_node 의 _grade_pwm() / _ff_output() 을 **그대로 불러서**
  시험한다.

  python3 tools/test_grade_ff.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller.serial_bridge_node import SerialBridgeNode as N  # noqa: E402

GRADE = N._grade_pwm
FFOUT = N._ff_output
IMUCB = N._imu_cb


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass


class _Clk:
  def __init__(self, t=100.0): self.t = t
  def now(self): return type('', (), {'nanoseconds': self.t * 1e9})()


class Stub:
  for _k, _v in vars(N).items():
    if _k.isupper() and not callable(_v):
      locals()[_k] = _v
  del _k, _v

  def __init__(self, **kw):
    # 실측 FF (ff-open-loop-measured-yongin)
    self.ff_static = 17.2
    self.ff_gain = 38.8
    self.ff_min_pwm = 0.0
    self.ff_breakaway_pwm = 60.0
    self.grade_ff_gain = 1.0
    self.grade_ff_max = 70.0
    self._pitch = None
    self._pitch_t = 0.0
    self._pitch_off = 0.0
    self._pitch_sign = 1.0
    self._clk = _Clk()
    self.__dict__.update(kw)

  def get_logger(self): return _Log()
  def get_clock(self): return self._clk


class _ImuMsg:
  """피치 p(rad) 를 담은 쿼터니언 메시지."""
  def __init__(self, p, yaw=0.0, roll=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    self.orientation = type('', (), {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy})()


def grade_rad(pct):
  return math.atan(pct / 100.0)


fails = []


def chk(name, ok, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   {detail}' if detail else ''))
  if not ok:
    fails.append(name)


def main():
  print('=' * 68)
  print('  IMU 경사 보상 — 부호가 틀리면 내리막에서 가속한다')
  print('=' * 68)

  # ================================================================
  print('\n■ 기본은 꺼짐 — 켜지 않으면 아무 일도 안 일어난다')
  st = Stub(grade_ff_gain=0.0, _pitch=grade_rad(12.5), _pitch_t=100.0)
  chk('gain=0 이면 0 을 낸다', GRADE(st, 100.0) == 0.0)
  chk('그때 FF 는 경사 보상 전과 같다',
      abs(FFOUT(st, 1.11, GRADE(st, 100.0), False) - (38.8 * 1.11 + 17.2)) < 0.1,
      f'{FFOUT(st, 1.11, 0.0, False):.1f}')

  # ================================================================
  print('\n■ 물리 — g·sinθ/k 를 그대로 낸다 (k=0.0202)')
  for pct, want in ((6.5, 31.5), (9.0, 43.5), (12.5, 60.1), (19.3, 91.6)):
    st = Stub(_pitch=grade_rad(pct), _pitch_t=100.0, grade_ff_max=999.0)
    got = GRADE(st, 100.0)
    chk(f'오르막 {pct}% → PWM +{want:.0f}', abs(got - want) < 1.5, f'{got:+.1f}')
  for pct, want in ((-6.5, -31.5), (-9.0, -43.5), (-19.3, -91.6)):
    st = Stub(_pitch=grade_rad(pct), _pitch_t=100.0, grade_ff_max=999.0)
    got = GRADE(st, 100.0)
    chk(f'내리막 {pct}% → PWM {want:.0f}', abs(got - want) < 1.5, f'{got:+.1f}')

  # ================================================================
  print('\n■ 안전장치')
  st = Stub(_pitch=grade_rad(12.5), _pitch_t=100.0)
  chk('IMU 가 0.5s 넘게 낡으면 0', GRADE(st, 100.6) == 0.0)
  st = Stub(_pitch=None, _pitch_t=100.0)
  chk('IMU 가 아예 없으면 0', GRADE(st, 100.0) == 0.0)
  st = Stub(_pitch=0.40, _pitch_t=100.0)          # 0.35 한계 초과
  chk('피치가 한계(0.35rad)를 넘으면 0 — IMU 이상', GRADE(st, 100.0) == 0.0)
  st = Stub(_pitch=grade_rad(19.3), _pitch_t=100.0, grade_ff_max=70.0)
  chk('상한 grade_ff_max 를 넘지 않는다', abs(GRADE(st, 100.0) - 70.0) < 0.01,
      f'{GRADE(st, 100.0):.1f}')
  st = Stub(_pitch=grade_rad(12.5), _pitch_t=100.0, grade_ff_gain=0.5)
  chk('부분 보상(gain 0.5)이 절반을 낸다', abs(GRADE(st, 100.0) - 30.1) < 1.0,
      f'{GRADE(st, 100.0):.1f}')

  # ================================================================
  print('\n■ 부호 — 여기가 틀리면 폭주한다')
  up, dn = grade_rad(12.5), grade_rad(-12.5)
  flat = 38.8 * 1.11 + 17.2                       # 60.3

  st = Stub(_pitch=up, _pitch_t=100.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), False)
  chk('전진·오르막 → PWM 이 커진다', out > flat + 50,
      f'{flat:.0f} → {out:.0f}')

  st = Stub(_pitch=dn, _pitch_t=100.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), False)
  chk('전진·내리막 → PWM 이 작아진다', out < flat - 50,
      f'{flat:.0f} → {out:.0f}')

  st = Stub(_pitch=grade_rad(-19.3), _pitch_t=100.0, grade_ff_max=999.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), False)
  chk('전진·급내리막 → **음수(제동)** 까지 간다', out < 0, f'{out:.0f}')

  st = Stub(_pitch=up, _pitch_t=100.0)
  out = FFOUT(st, -1.11, GRADE(st, 100.0), False)
  chk('후진·오르막(= 뒤로 내려감) → 역PWM 크기가 준다',
      -flat < out < 0, f'{-flat:.0f} → {out:.0f}')

  st = Stub(_pitch=dn, _pitch_t=100.0)
  out = FFOUT(st, -1.11, GRADE(st, 100.0), False)
  chk('후진·내리막(= 뒤로 올라감) → 역PWM 크기가 는다',
      out < -flat, f'{-flat:.0f} → {out:.0f}')

  # ================================================================
  print('\n■ 하한이 경사 보상을 지우면 안 된다')
  # ff_min_pwm 이 크면, 내리막에서 '최소한 이만큼은 밀어' 가 중력 상쇄를
  # 덮어써 계속 가속한다. 순서가 ①하한 ②경사 여야 한다.
  st = Stub(_pitch=grade_rad(-19.3), _pitch_t=100.0,
            ff_min_pwm=80.0, grade_ff_max=999.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), False)
  chk('하한 80 이어도 급내리막에서는 내려간다', out < 80.0, f'{out:.0f}')
  st = Stub(_pitch=grade_rad(-19.3), _pitch_t=100.0,
            ff_breakaway_pwm=120.0, grade_ff_max=999.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), True)
  chk('정지마찰 120 이어도 급내리막에서는 상쇄된다', out < 60.0, f'{out:.0f}')

  # ================================================================
  print('\n■ 포화 — 어떤 경우에도 ±255 를 안 넘는다')
  st = Stub(_pitch=grade_rad(19.3), _pitch_t=100.0, grade_ff_max=999.0)
  chk('전진 최대', abs(FFOUT(st, 3.0, 9999.0, True)) <= 255.0)
  chk('후진 최대', abs(FFOUT(st, -3.0, -9999.0, True)) <= 255.0)

  # ================================================================
  print('\n■ IMU 콜백 — 쿼터니언·영점·부호·저역통과')
  st = Stub()
  st._clk.t = 100.0
  for _ in range(60):                             # 저역통과가 수렴하도록
    st._clk.t += 0.05
    IMUCB(st, _ImuMsg(grade_rad(12.5)))
  chk('피치를 읽는다', abs(math.degrees(st._pitch) - 7.13) < 0.3,
      f'{math.degrees(st._pitch):+.2f}° (12.5% = 7.13°)')

  st = Stub()
  st._clk.t = 100.0
  for _ in range(60):
    st._clk.t += 0.05
    IMUCB(st, _ImuMsg(grade_rad(12.5), yaw=2.0, roll=0.05))
  chk('yaw 가 돌아도 피치는 그대로', abs(math.degrees(st._pitch) - 7.13) < 0.5,
      f'{math.degrees(st._pitch):+.2f}°')

  st = Stub(_pitch_sign=-1.0)
  st._clk.t = 100.0
  for _ in range(60):
    st._clk.t += 0.05
    IMUCB(st, _ImuMsg(grade_rad(12.5)))
  chk('sign=-1 이 부호를 뒤집는다', st._pitch < 0,
      f'{math.degrees(st._pitch):+.2f}°')

  st = Stub(_pitch_off=grade_rad(12.5))           # 마운트가 12.5% 기울어 붙음
  st._clk.t = 100.0
  for _ in range(60):
    st._clk.t += 0.05
    IMUCB(st, _ImuMsg(grade_rad(12.5)))
  chk('영점이 마운트 기울기를 뺀다 (평지 = 0)', abs(st._pitch) < 0.01,
      f'{math.degrees(st._pitch):+.3f}°')

  # 저역통과가 가감속 피칭을 누르는가 — 한 샘플 튀어도 안 따라간다
  st = Stub()
  st._clk.t = 100.0
  for _ in range(60):
    st._clk.t += 0.05
    IMUCB(st, _ImuMsg(0.0))
  before = st._pitch
  st._clk.t += 0.05
  IMUCB(st, _ImuMsg(grade_rad(30.0)))             # 한 샘플만 크게
  chk('한 샘플 스파이크를 다 안 따라간다',
      abs(st._pitch) < 0.30 * abs(grade_rad(30.0)),
      f'{math.degrees(before):+.2f}° → {math.degrees(st._pitch):+.2f}° '
      f'(원시 {math.degrees(grade_rad(30.0)):+.2f}°)')

  # ================================================================
  print('\n■ 실측 대조 — 2026-09-17 내리막 19.3%')
  #   그 런에서 4km/h 를 유지하는 데 실제로 들어간 PWM 이 평균 -15 였다.
  #   경사 보상이 켜져 있었다면 FF 가 처음부터 그 근처를 냈어야 한다.
  st = Stub(_pitch=grade_rad(-19.3), _pitch_t=100.0, grade_ff_max=999.0)
  out = FFOUT(st, 1.11, GRADE(st, 100.0), False)
  chk('4km/h 명령 → FF 가 -31 근처를 낸다 (실측 평균 -15)',
      -45 < out < -15, f'{out:.0f}')
  print('     실측이 -15 로 더 높은 것은 거버너가 목표보다 빠른 1.58m/s 에')
  print('     물려 있었기 때문이다(그 속도의 FF 는 더 크다). 부호와 자릿수가')
  print('     맞는지를 본다.')

  print()
  if fails:
    print(f'❌ 실패 {len(fails)}건: {", ".join(fails)}')
    return 1
  print('✅ 전부 통과 — 경사 보상의 부호·물리·안전장치가 설계대로다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

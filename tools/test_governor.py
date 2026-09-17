#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_governor.py — 내리막 속도 거버너를 **하드웨어 없이** 검증한다.

★ 왜 이게 필요한가 (2026-09-17)
  개루프에는 '달리는 중' 감속 권한이 없다. 제동(ff_brake_pwm)은 **정지 명령에만**
  붙는다. 그래서 내리막에서 중력이 이기면 명령을 낮춰도 계속 빨라진다.

  실측: 오늘 시험장(18~20%)에서 관성만으로 8m 에 4.1 m/s. 굴절코스 첫 코너
  (R=6.4m) 상한은 3.0 이다.

  거버너는 **과속만 깎는다** — 측정속도가 명령보다 deadband 넘게 빠르면
  초과분에 비례해 역 PWM 을 낸다.

★ 왜 시험이 까다로운가
  전진 중 역 PWM 은 **플러깅**이라 전류가 기동보다도 크다. 잘못 걸리면
  차를 튀게 하거나 드라이버를 태운다. 그래서 **안 걸려야 할 때 안 걸리는지**가
  걸릴 때 걸리는지보다 중요하다.

  serial_bridge_node._governor_pwm() 을 **그대로 불러서** 시험한다.

  python3 tools/test_governor.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller.serial_bridge_node import SerialBridgeNode  # noqa: E402

GOV = SerialBridgeNode._governor_pwm


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass


class Stub:
  def __init__(self, **kw):
    self.gov_pwm = 80.0
    self.gov_deadband = 0.30
    self.gov_gain = 80.0
    self.ff_deadband = 0.05
    self._meas_v = None
    self._meas_v_t = 0.0
    self.BRAKE_MEAS_FRESH_S = SerialBridgeNode.BRAKE_MEAS_FRESH_S
    self.BRAKE_V_PLAUSIBLE = SerialBridgeNode.BRAKE_V_PLAUSIBLE
    self.__dict__.update(kw)

  def get_logger(self): return _Log()


def run(name, cmd_v, meas_v, want, fails, **kw):
  """want: None(개입 안 함) 또는 기대 PWM(부호 포함, ±2 허용)."""
  st = Stub(**kw)
  st._meas_v = meas_v
  st._meas_v_t = kw.get('meas_t', 100.0)
  got = GOV(st, cmd_v, 100.0)
  if want is None:
    ok = got is None
  else:
    ok = got is not None and abs(got - want) <= 2.0
  print(f'  {"OK " if ok else "✗  "} {name}')
  if not ok:
    print(f'        기대 {want} · 실제 {got}')
    fails.append(name)


def main():
  f = []
  print('내리막 속도 거버너 검증\n')

  print('■ 꺼져 있을 때 (기본값)')
  run('gov_pwm=0 이면 절대 안 건다', 1.0, 3.0, None, f, gov_pwm=0.0)

  print('\n■ 개입 조건')
  run('명령대로면 안 건다  (1.0 vs 1.0)', 1.0, 1.0, None, f)
  run('데드밴드 안이면 안 건다  (1.0 vs 1.25)', 1.0, 1.25, None, f)
  run('데드밴드 넘으면 건다  (1.0 vs 1.5 → 초과 0.2 × 80)',
      1.0, 1.5, -16.0, f)
  run('많이 넘으면 상한에 걸린다  (1.0 vs 3.0 → 상한 80)',
      1.0, 3.0, -80.0, f)

  print('\n■ ★ 안 걸려야 할 때 — 플러깅이라 오작동이 위험하다')
  run('정지 명령이면 안 건다 (그건 ff_brake 의 몫)', 0.0, 2.0, None, f)
  run('측정이 낡으면(0.5s 초과) 안 건다', 1.0, 3.0, None, f, meas_t=99.0)
  run('측정이 없으면 안 건다', 1.0, None, None, f)
  run('odom 스파이크(61.9m/s)는 무시한다', 1.0, 61.9, None, f)
  run('측정이 명령보다 **느리면** 안 건다 (오르막)', 1.0, 0.3, None, f)
  run('부호가 다르면 안 건다 (전진명령 · 후진측정)', 1.0, -2.0, None, f)
  run('부호가 다르면 안 건다 (후진명령 · 전진측정)', -1.0, 2.0, None, f)

  print('\n■ 후진 주행에서도 대칭으로 동작한다')
  run('후진 과속이면 **앞으로** 미는 역 PWM', -1.0, -1.5, +16.0, f)

  print('\n■ 실측 시나리오 — 오늘 시험장 18% 내리막')
  # 명령 1.2 로 가는데 중력이 밀어 2.5 가 됐다 → 초과 1.0 × 80 = 80 (상한)
  run('명령 1.2 · 실제 2.5 → 상한 80 으로 제동', 1.2, 2.5, -80.0, f)
  # 법정 9% 라면 2.2 까지만 간다 → 초과 0.7 × 80 = 56
  run('명령 1.2 · 실제 2.2 (법정 9%) → 역 PWM 56', 1.2, 2.2, -56.0, f)

  print('\n■ 상한을 낮추면 그만큼만 낸다 (전류 제한)')
  run('gov_pwm=50 이면 50 을 안 넘는다', 1.0, 3.0, -50.0, f, gov_pwm=50.0)

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 거버너가 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_brake_logic.py — 능동 제동 상태기계를 **하드웨어 없이** 검증한다.

★ 왜 이렇게 시험하나
  제동은 잘못 걸리면 차를 뒤로 민다. 엔코더가 없어 '멈췄다' 를 직접 못 보므로
  안전장치가 셋 겹쳐 있는데(시간상한 · 측정속도 · 트리거속도), 그게 전부
  제대로 맞물리는지는 지면에서 확인하기 전에 알아야 한다.

  serial_bridge_node._brake_pwm() 을 **그대로 불러서** 시험한다. 로직을
  복사해 오면 원본이 바뀔 때 시험이 거짓말을 하므로 그러지 않는다.

  python3 tools/test_brake_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller.serial_bridge_node import SerialBridgeNode  # noqa: E402

BRAKE = SerialBridgeNode._brake_pwm          # 실제 구현을 그대로 쓴다


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass


class Stub:
  """_brake_pwm 이 참조하는 속성만 갖춘 최소 객체."""

  def __init__(self, **kw):
    self.ff_brake_pwm = 70.0
    self.ff_brake_s = 0.8
    self.ff_brake_min_v = 0.15
    self.ff_brake_trigger_v = 0.40
    self.ff_brake_decel = 1.0
    self.ff_deadband = 0.05
    self._brake_until = None
    self._brake_sign = 0
    self._last_cmd_v = 0.0
    self._meas_v = None
    self._meas_v_t = 0.0
    self.__dict__.update(kw)

  def get_logger(self): return _Log()


def run(name, steps, expect, fails):
  """steps = [(t, cmd_v, meas_v 또는 None, meas_t 또는 None)] → 제동 PWM 목록"""
  st = Stub(**steps['stub']) if isinstance(steps, dict) else Stub()
  seq = steps['seq'] if isinstance(steps, dict) else steps
  got = []
  for item in seq:
    t, cmd = item[0], item[1]
    if len(item) > 2 and item[2] is not None:
      st._meas_v = item[2]
      st._meas_v_t = item[3] if len(item) > 3 else t
    out = BRAKE(st, cmd, t)
    got.append(None if out is None else round(out, 1))
    st._last_cmd_v = cmd          # 노드가 매 주기 하는 일과 같다
  ok = got == expect
  print(f'  {"OK " if ok else "✗  "} {name}')
  if not ok:
    print(f'        기대 {expect}')
    print(f'        실제 {got}')
    fails.append(name)


def main():
  f = []
  print('능동 제동 상태기계 검증\n')

  print('■ 꺼져 있을 때 (기본값)')
  run('ff_brake_pwm=0 이면 절대 안 건다',
      {'stub': {'ff_brake_pwm': 0.0},
       'seq': [(0.0, 1.2), (0.1, 0.0), (0.2, 0.0), (0.3, 0.0)]},
      [None, None, None, None], f)

  print('\n■ 진입 조건')
  run('빠르게 달리다 정지명령 → 역방향으로 건다',
      [(0.0, 1.2), (0.1, 0.0), (0.2, 0.0)],
      [None, -70.0, -70.0], f)
  run('느린 기동(주차)에서는 안 건다  0.3 < trigger 0.4',
      [(0.0, 0.30), (0.1, 0.0), (0.2, 0.0)],
      [None, None, None], f)
  run('후진하다 정지 → 앞방향으로 건다',
      [(0.0, -1.0), (0.1, 0.0), (0.2, 0.0)],
      [None, +70.0, +70.0], f)
  run('처음부터 서 있으면 안 건다',
      [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)],
      [None, None, None], f)

  print('\n■ 해제 조건')
  run('시간 상한 — 1.2m/s 면 decel 1.0 으로 1.2초, 상한 0.8초에 끊긴다',
      [(0.0, 1.2), (0.1, 0.0), (0.5, 0.0), (0.85, 0.0), (0.95, 0.0)],
      [None, -70.0, -70.0, -70.0, None], f)
  run('느린 속도는 상한보다 먼저 끝난다  0.5/1.0 = 0.5초',
      [(0.0, 0.5), (0.1, 0.0), (0.3, 0.0), (0.65, 0.0)],
      [None, -70.0, -70.0, None], f)
  run('측정속도가 0.15 밑이면 즉시 해제',
      [(0.0, 1.2), (0.1, 0.0, 0.9, 0.1), (0.2, 0.0, 0.10, 0.2), (0.3, 0.0)],
      [None, -70.0, None, None], f)
  run('다시 가라는 명령이 오면 즉시 해제',
      [(0.0, 1.2), (0.1, 0.0), (0.2, 0.8), (0.3, 0.8)],
      [None, -70.0, None, None], f)

  print('\n■ 안전 — 가장 중요한 것들')
  run('제동이 끝난 뒤 서 있는 동안 **재진입하지 않는다**',
      [(0.0, 1.2), (0.1, 0.0), (0.95, 0.0), (1.5, 0.0), (3.0, 0.0),
       (5.0, 0.0)],
      [None, -70.0, None, None, None, None], f)
  # 측정이 '낡았다' 를 만들려면 수신 시각을 0.5초 넘게 과거로 둬야 한다.
  # (IMU 가 끊기면 _meas_v 에 옛 값이 남는데, 그걸 믿고 제동을 끊으면 안 된다)
  run('측정이 낡으면(0.5초 초과) 믿지 않고 시간상한만 쓴다',
      [(0.0, 1.2), (0.1, 0.0, 0.05, -1.0), (0.3, 0.0), (0.95, 0.0)],
      [None, -70.0, -70.0, None], f)
  run('측정이 신선하고 아직 빠르면 제동을 유지한다',
      [(0.0, 1.2), (0.1, 0.0, 0.90, 0.1), (0.3, 0.0, 0.60, 0.3),
       (0.5, 0.0, 0.10, 0.5)],
      [None, -70.0, -70.0, None], f)

  print('\n■ 돌발정지 시나리오 — 정지 후 5초 대기, 그 뒤 재출발')
  st = Stub()
  log = []
  seq = [(0.0, 1.2), (0.05, 0.0), (0.4, 0.0), (0.9, 0.0)]
  seq += [(1.0 + 0.5 * i, 0.0) for i in range(10)]     # 5초 대기
  seq += [(6.5, 0.9), (6.55, 0.9)]                     # 재출발
  for t, cmd in seq:
    log.append(BRAKE(st, cmd, t))
    st._last_cmd_v = cmd
  braked = [i for i, v in enumerate(log) if v is not None]
  ok = (braked and braked[0] == 1
        and all(log[i] is None for i in range(4, len(log))))
  print(f'  {"OK " if ok else "✗  "} 제동은 초반에만, 5초 대기 중엔 없고, '
        f'재출발을 막지 않는다')
  if not ok:
    print(f'        제동이 걸린 주기: {braked}')
    f.append('돌발정지 시나리오')

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 제동 상태기계가 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

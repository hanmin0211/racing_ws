#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_speed_pi.py — 속도 PI(RTK 위치미분 피드백)를 **하드웨어 없이** 검증한다.

★ 왜 (2026-10-09)
  10/8 실측: 보드 펌웨어에 PID 가 없고 엔코더는 간헐적이라, 실측이 목표를
  모양으로 전혀 안 따라갔다(r=0.02). serial_bridge 에 위치미분 속도로 도는 PI 를
  얹었다(spd_kp/spd_ki, 기본 0 = 꺼짐). 이득 근거는 tools/speed_pi_sim.py.

  차를 못 돌리는 상태라 실차 전 마지막 관문이다. serial_bridge_node 의 함수를
  **그대로 불러서** 시험한다(베끼면 갈라진다).

  python3 tools/test_speed_pi.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller.serial_bridge_node import SerialBridgeNode  # noqa: E402

STEP = SerialBridgeNode._spd_pi_step
WSPEED = SerialBridgeNode._spd_window_speed
PI = SerialBridgeNode._spd_pi


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass
  def error(self, *a, **k): pass


class Stub:
  def __init__(self, **kw):
    self.spd_kp, self.spd_ki = 80.0, 40.0
    self.spd_imax, self.spd_brake_max = 2.5, 30.0
    self.spd_up_max = 90.0
    self.spd_stuck_s = 3.0
    self._spd_still_since, self._spd_stuck = None, False
    self.spd_fresh_s = 0.4
    self.ff_deadband = 0.05
    self._spd_v, self._spd_v_t, self._spd_isum = None, 0.0, 0.0
    self.__dict__.update(kw)

  _spd_pi_step = staticmethod(STEP)

  def get_logger(self): return _Log()


def check(name, ok, fails, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   ({detail})' if detail else ''))
  if not ok:
    fails.append(name)


def main():
  f = []
  print('속도 PI 검증\n')

  print('■ 꺼져 있을 때 (기본값) — 이전과 완전히 같아야 한다')
  st = Stub(spd_kp=0.0, spd_ki=0.0, _spd_v=0.0, _spd_v_t=100.0)
  check('kp=ki=0 이면 개입 안 함', PI(st, 1.5, 75.0, 100.0) is None, f)

  print('\n■ 안 걸려야 할 때')
  st = Stub(_spd_v=1.0, _spd_v_t=100.0)
  check('정지 명령이면 개입 안 함', PI(st, 0.0, 0.0, 100.0) is None, f)
  st = Stub(_spd_v=1.0, _spd_v_t=99.5, _spd_isum=0.7)
  out = PI(st, 1.5, 75.0, 100.0)
  check('측정이 낡으면(0.5s) FF 만 — 적분도 비운다', out is None and st._spd_isum == 0.0, f)
  st = Stub()
  check('측정이 아직 없으면 개입 안 함', PI(st, 1.5, 75.0, 100.0) is None, f)

  print('\n■ 방향')
  out, _ = STEP(1.5, 1.5, 75.0, 0.0, 0.05, 80, 40, 1.5, 30)
  check('목표대로면 FF 그대로', abs(out - 75.0) < 0.5, f, f'{out:.1f}')
  out, _ = STEP(1.5, 1.0, 75.0, 0.0, 0.05, 80, 40, 1.5, 30)
  check('느리면 더 민다 (오르막)', out > 75.0 + 35, f, f'{out:.1f}')
  out, _ = STEP(1.5, 2.0, 75.0, 0.0, 0.05, 80, 40, 1.5, 30)
  check('빠르면 줄인다 (내리막)', out < 75.0 - 35, f, f'{out:.1f}')

  print('\n■ 상한 — 플러깅 전류·와인드업')
  out, _ = STEP(1.5, 4.0, 75.0, 0.0, 0.05, 80, 40, 1.5, 30)
  check('역 PWM 은 spd_brake_max 에서 멈춘다', out == -30.0, f, f'{out:.1f}')
  out, _ = STEP(1.5, 0.0, 150.0, 0.0, 0.05, 300, 40, 1.5, 30)
  check('정 PWM 은 255 에서 멈춘다', out == 255.0, f, f'{out:.1f}')
  st = Stub(_spd_v=0.0, _spd_v_t=100.0)
  for _ in range(200):                          # 벽에 박혀 10초간 측정 0
    out = PI(st, 1.3, 67.6, 100.0)
  check('★ 벽에 박혀 서 있어도 FF+90 위로는 안 민다', out <= 67.6 + 90.0 + 1e-6, f,
        f'{out:.1f}')
  st._spd_v_t = 103.5                           # 측정은 계속 들어오고 있다(값만 0)
  out2 = PI(st, 1.3, 67.6, 103.5)
  check('★ 3초 넘게 서 있으면 PI 가산을 빼고 FF 만 (박힘 보호)', out2 is None and st._spd_stuck, f)
  st._spd_v, st._spd_v_t = 0.5, 104.0
  out3 = PI(st, 1.3, 67.6, 104.0)
  check('다시 움직이면 PI 복귀', out3 is not None and not st._spd_stuck, f)
  st = Stub(_spd_v=0.0, _spd_v_t=100.0)
  out = PI(st, 1.3, 67.6, 100.0)
  st._spd_v_t = 102.0
  out = PI(st, 1.3, 67.6, 102.0)
  check('출발 중(3초 안)에는 PI 로 민다 — 방전 배터리 출발', out is not None and out > 100, f,
        f'{out:.1f}')
  isum = 0.0
  for _ in range(400):                          # 20초 내내 느림
    _, isum = STEP(1.5, 0.5, 75.0, isum, 0.05, 80, 40, 2.5, 30)
  check('적분은 spd_imax 에서 멈춘다', abs(isum - 2.5) < 1e-9, f, f'{isum:.2f}')
  isum = 0.0
  for _ in range(400):                          # 포화된 채 계속 느림
    _, isum = STEP(1.5, 0.0, 200.0, isum, 0.05, 80, 40, 1.5, 30)
  check('포화 방향으로는 적분을 더 안 쌓는다', isum < 0.2, f, f'{isum:.2f}')

  print('\n■ 위치미분 속도')
  h = [(0.0, 0.0, 0.0), (0.1, 0.15, 0.0), (0.2, 0.30, 0.0), (0.3, 0.45, 0.0),
       (0.4, 0.60, 0.0), (0.5, 0.75, 0.0)]
  v = WSPEED(h, 0.5)
  check('직선 1.5 m/s 를 1.5 로 읽는다', v is not None and abs(v - 1.5) < 1e-6, f, f'{v}')
  check('창이 덜 찼으면 None', WSPEED(h[:3], 0.5) is None, f)
  check('표본 하나면 None', WSPEED(h[:1], 0.5) is None, f)

  print('\n■ GPS 위치 점프 (10/12 실측: 0.1초에 0.37·0.46 m)')
  J = SerialBridgeNode._spd_is_jump
  check('정상 한 걸음 (1.5 m/s · 0.15 m) 은 점프 아님', not J((0, 0, 0), (0.1, 0.15, 0)), f)
  check('0.37 m / 0.1 s (3.7 m/s) 는 점프', J((0, 0, 0), (0.1, 0.37, 0)), f)
  check('GPS 한 번 빠져 0.2 s 에 0.30 m 는 정상', not J((0, 0, 0), (0.2, 0.30, 0)), f)
  # 콜백을 그대로 돌린다 — 1.5 m/s 직진 중 t=2.0 에 0.4 m 점프
  class Clk:
    def __init__(self): self.t = 0.0
    def now(self):
      t = self.t
      class N: nanoseconds = int(t * 1e9)
      return N()
  class Msg:
    def __init__(self, x):
      class P: pass
      self.pose = P(); self.pose.pose = P(); self.pose.pose.position = P()
      self.pose.pose.position.x, self.pose.pose.position.y = x, 0.0
  st = Stub(spd_win_s=0.5, spd_max_sigma=0.05, _spd_sigma=0.012, _spd_hist=[],
            _spd_valid_t=0.0, spd_pub=None)
  clk = Clk(); st.get_clock = lambda: clk
  st._spd_is_jump = SerialBridgeNode._spd_is_jump
  st._spd_window_speed = SerialBridgeNode._spd_window_speed
  for k in ('SPD_V_PLAUSIBLE', 'SPD_STEP_JUMP', 'SPD_HOLD_MAX_S'):
    setattr(st, k, getattr(SerialBridgeNode, k))
  seen, x = [], 0.0
  for i in range(40):
    clk.t = 0.1 * i
    x += 0.15 + (0.40 if i == 20 else 0.0)
    SerialBridgeNode._spd_pos_cb(st, Msg(x))
    if st._spd_v is not None:
      seen.append((clk.t, st._spd_v, clk.t - st._spd_v_t))
  vmax = max(v for tt, v, _a in seen if tt >= 1.0)
  stale = max(ag for tt, _v, ag in seen if tt >= 1.0)
  check('★ 점프가 속도로 새지 않는다 (최대 1.6 이하)', vmax < 1.6, f, f'최대 {vmax:.2f} m/s')
  check('★ 점프 뒤에도 측정이 끊기지 않는다 (적분 유지)', stale < 0.4, f, f'최대 공백 {stale:.2f}s')
  check('점프 뒤 다시 1.5 로 계산한다', abs(seen[-1][1] - 1.5) < 0.05, f, f'{seen[-1][1]:.2f}')

  print('\n■ 폐루프 — 노드 함수로 플랜트를 돌린다 (dv/dt = K·PWM − C·v + d)')
  K, C, dt = 0.0175, 0.861, 0.01
  def run(dist, kp=80, ki=40, delay_s=0.25, target=1.5, t_end=20.0):
    v, isum, pwm, t = 1.0, 0.0, 0.0, 0.0
    buf = []
    vs = []
    while t < t_end:
      buf.append(v)
      meas = buf[max(0, len(buf) - 1 - int(delay_s / dt))]
      if int(round(t / dt)) % 5 == 0:           # 20 Hz 제어
        ff = 17.2 + 38.8 * target
        pwm, isum = STEP(target, meas, ff, isum, 0.05, kp, ki, 1.5, 30)
      v = max(0.0, v + (K * pwm - C * v + dist) * dt)
      t += dt
      vs.append(v)
    tail = vs[-500:]                             # 마지막 5초
    return sum(tail) / len(tail), max(tail) - min(tail)
  for name, d in (('평지', 0.0), ('오르막 +2.4% (10/8 t68~80)', -0.24),
                  ('내리막 −2.4%', +0.24), ('오르막 +5%', -0.49)):
    mean, pp = run(d)
    check(f'{name}: 1.5 에 수렴', abs(mean - 1.5) < 0.05 and pp < 0.05, f,
          f'정상 {mean:.3f} · 흔들림 {pp:.3f}')
  mean, pp = run(-0.24, delay_s=0.6)
  check('지연 0.6s 에서도 발산 안 함', abs(mean - 1.5) < 0.05 and pp < 0.1, f,
        f'정상 {mean:.3f} · 흔들림 {pp:.3f}')
  mean_ff, _ = run(-0.24, kp=0, ki=0)
  check('비교: PI 없으면 오르막에서 처진다', mean_ff < 1.4, f, f'FF 만 {mean_ff:.3f}')

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 속도 PI 가 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

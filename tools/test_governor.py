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
    self.gov_deadband = 0.10
    self.gov_gain = 300.0
    self.ff_deadband = 0.05
    self.gov_lead_s = 0.0          # 시험 기본은 순수 P — 선행은 따로 시험한다
    self._meas_v = None
    self._meas_v_t = 0.0
    self._meas_a = 0.0
    self._meas_a_t = 0.0
    # ★ 측정 출처. 기본 None = 예전과 같은 상한(3.0). 'encpos' 로 주면
    #   위치차분 상한(6.0)이 적용된다 — 2026-09-18 추가분.
    self._meas_src = None
    # 자세 게이트용 (2026-09-18). 기본은 게이트 꺼짐 + 피치 없음.
    self.gov_min_grade = 0.0
    self._pitch = None
    self._pitch_t = 0.0
    # 원본의 대문자 상수를 전부 가져온다 (위 S2 주석 참고)
    for _k, _v in vars(SerialBridgeNode).items():
      if _k.isupper() and not callable(_v):
        setattr(self, _k, _v)
    self.__dict__.update(kw)

  # 출처별 상한 판정도 **원본을 그대로** 쓴다. 베끼면 갈라진다.
  _gov_v_plausible = SerialBridgeNode._gov_v_plausible
  _gov_grade_ok = SerialBridgeNode._gov_grade_ok

  def get_logger(self): return _Log()


def run(name, cmd_v, meas_v, want, fails, ff=None, **kw):
  """want: None(개입 안 함) 또는 기대 PWM(부호 포함, ±2 허용).

  ff 를 안 주면 명령속도에서 FF 를 계산한다 (PWM = 38.8·v + 17.2).
  """
  st = Stub(**kw)
  st._meas_v = meas_v
  st._meas_v_t = kw.get('meas_t', 100.0)
  if ff is None:
    ff = 38.8 * abs(cmd_v) + 17.2
  got = GOV(st, cmd_v, ff, 100.0)
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
  run('데드밴드 안이면 안 건다  (1.0 vs 1.05)', 1.0, 1.05, None, f)
  # FF(1.0) = 56.0 · 초과 0.4 × 300 = 120 → 56 − 120 = −64 (상한 80 안)
  run('조금 넘으면 **스로틀만 줄인다** (FF 56 · 초과 0.1 → 26)',
      1.0, 1.2, 26.0, f)
  run('많이 넘으면 제동으로 넘어간다 (FF 56 · 초과 0.4 → −64)',
      1.0, 1.5, -64.0, f)
  run('아주 많이 넘으면 상한에 걸린다 (상한 80)',
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
  run('후진 과속이면 **앞으로** 미는 방향으로 깎는다', -1.0, -1.5, +64.0, f)

  print('\n■ 실측 시나리오 — 오늘 시험장 18% 내리막')
  # 명령 1.2 로 가는데 중력이 밀어 2.5 가 됐다 → 초과 1.0 × 80 = 80 (상한)
  run('명령 1.2 · 실제 2.5 → 상한 80 으로 제동', 1.2, 2.5, -80.0, f)
  # FF(1.2)=63.8 · 초과 (2.2-1.2-0.1)=0.9 × 300 = 270 → 63.8−270 = −206 → 상한 80
  run('명령 1.2 · 실제 2.2 → 상한 80', 1.2, 2.2, -80.0, f)
  # 5 km/h(1.39) 목표를 살짝 넘긴 경우 — 스로틀만 줄어야 한다
  run('5km/h 목표 · 실제 1.55 → FF 71 에서 44 로 (제동 아님)',
      1.39, 1.55, 71.1 - 300 * 0.06, f)

  print('\n■ ★ 측정 출처별 타당성 상한 (2026-09-18)')
  # 3.0 은 /odometry/filtered 의 twist 스파이크(61.9 m/s)를 거르려던 값이다.
  # 9/17 에 거버너 입력을 엔코더 위치차분으로 바꿨는데 상한이 그대로 남아,
  # **3.0 을 넘는 순간 거버너가 스스로 꺼졌다** — 제일 필요한 지점에서.
  # 용인은 경사로가 끝나고 4m 뒤 굴절코스다. 3.9 로 내려오면 이탈이다.
  run('위치차분 3.5 m/s — 예전엔 포기했다. 이제 잡는다',
      1.11, 3.5, -50.0, f, gov_pwm=50.0, _meas_src='encpos')
  run('위치차분 5.5 m/s 도 잡는다 (평지 PWM255 정상속도)',
      1.11, 5.5, -50.0, f, gov_pwm=50.0, _meas_src='encpos')
  # ⚠ 여기서부터가 **깨뜨리면 안 되는 것**이다.
  run('위치차분이라도 6.0 초과는 여전히 무시한다 (카운터 오독)',
      1.11, 6.5, None, f, gov_pwm=50.0, _meas_src='encpos')
  run('odom 은 상한 그대로 3.0 — 3.5 를 무시한다',
      1.11, 3.5, None, f, gov_pwm=50.0, _meas_src='odom')
  run('펌웨어 VEL(enc) 도 상한 그대로 3.0 — 3.5 를 무시한다',
      1.11, 3.5, None, f, gov_pwm=50.0, _meas_src='enc')
  run('출처 미상(None)도 상한 그대로 3.0',
      1.11, 3.5, None, f, gov_pwm=50.0)
  run('odom 스파이크 61.9 는 출처가 뭐든 무시한다',
      1.11, 61.9, None, f, gov_pwm=50.0, _meas_src='encpos')
  # 상한을 올려도 '꺼져 있으면 안 건다' 는 그대로여야 한다
  run('gov_pwm=0 이면 위치차분 5.0 이어도 안 건다',
      1.11, 5.0, None, f, gov_pwm=0.0, _meas_src='encpos')

  print('\n■ ★ 자세 게이트 — 내리막일 때만 개입한다 (2026-09-18)')
  # 거버너는 중력이 미는 내리막을 위한 것이다. 평지·오르막에서 도는 것은
  # 곡률 제한이 만든 추종 지연이지 진짜 과속이 아니다. 실측(학교 128m):
  # 38회 개입 중 25회가 깎은 뒤 PWM 이 구동 문턱 아래였고 차가 섰다.
  import math as _m
  DOWN = _m.asin(-0.10)      # 10% 내리막
  UP   = _m.asin(+0.10)      # 10% 오르막
  FLAT = 0.0
  run('게이트 꺼짐(기본 0) 이면 평지에서도 예전처럼 건다',
      1.0, 1.5, -64.0, f, _pitch=FLAT, _pitch_t=100.0)
  run('게이트 켜고 **내리막** 이면 건다',
      1.0, 1.5, -64.0, f, gov_min_grade=0.03, _pitch=DOWN, _pitch_t=100.0)
  # ⚠ 아래 셋이 이 기능의 존재 이유다
  run('게이트 켜고 **평지** 면 안 건다 (곡률이 만든 가짜 과속)',
      1.0, 1.5, None, f, gov_min_grade=0.03, _pitch=FLAT, _pitch_t=100.0)
  run('게이트 켜고 **오르막** 이면 안 건다',
      1.0, 1.5, None, f, gov_min_grade=0.03, _pitch=UP, _pitch_t=100.0)
  run('문턱보다 완만한 내리막(1%)이면 안 건다',
      1.0, 1.5, None, f, gov_min_grade=0.03, _pitch=_m.asin(-0.01),
      _pitch_t=100.0)
  # ⚠ 실패 방향 — 모르면 **허용**해야 한다. 내리막에서 못 잡으면 이탈이고,
  #   평지에서 잘못 잡으면 최악이 '한 번 선다' 다. 이탈보다 정지가 싸다.
  run('IMU 가 없으면 허용한다 (모를 때는 안전한 쪽)',
      1.0, 1.5, -64.0, f, gov_min_grade=0.03, _pitch=None, _pitch_t=100.0)
  run('IMU 가 0.5s 넘게 낡으면 허용한다',
      1.0, 1.5, -64.0, f, gov_min_grade=0.03, _pitch=FLAT, _pitch_t=99.0)
  # 게이트가 열려도 나머지 안전장치는 그대로여야 한다
  run('게이트 열려도 gov_pwm=0 이면 안 건다',
      1.0, 1.5, None, f, gov_pwm=0.0, gov_min_grade=0.03, _pitch=DOWN,
      _pitch_t=100.0)
  run('게이트 열려도 부호가 다르면 안 건다',
      1.0, -2.0, None, f, gov_min_grade=0.03, _pitch=DOWN, _pitch_t=100.0)

  print('\n■ 상한을 낮추면 그만큼만 낸다 (전류 제한)')
  run('gov_pwm=50 이면 역방향 50 을 안 넘는다', 1.0, 3.0, -50.0, f, gov_pwm=50.0)

  print('\n■ 측정속도 공급원 — odom 우선, 없으면 엔코더 (2026-09-17)')
  # 실차에서 teleop 만 띄웠더니 /odometry/filtered 가 없어 거버너가 **한 번도
  # 개입하지 않았다**. 엔코더는 이 노드가 직접 파싱하므로 항상 있다.
  # 트랙에서는 odom 이 나오지만, IMU 가 끊기면(9/15 에 32번) odom 도 멈춘다 —
  # 거버너가 정확히 필요한 순간에 눈이 머는 것을 막는다.
  ENC_CB = SerialBridgeNode._enc_speed_cb
  ODOM_CB = SerialBridgeNode._odom_cb

  class _Msg:
    def __init__(self, v): self.data = v

  class _Odom:
    def __init__(self, v):
      self.twist = type('', (), {'twist': type('', (), {
          'linear': type('', (), {'x': v})()})()})()

  class _Clock:
    def __init__(self, t): self.t = t
    def now(self): return type('', (), {'nanoseconds': self.t * 1e9})()

  class S2(Stub):
    # ★ 상수를 하나씩 베껴 쓰면 원본에 상수가 늘 때마다 AttributeError 가
    #   난다(실제로 ENC_ACC_TAU 를 추가하고 바로 겪었다). 대문자 클래스
    #   속성은 **전부** 가져온다.
    for _k, _v in vars(SerialBridgeNode).items():
      if _k.isupper() and not callable(_v):
        locals()[_k] = _v
    del _k, _v

    def __init__(self, **kw):
      super().__init__(**kw)
      self._meas_src = None
      self._enc_v_buf = []
      self._enc_hist = []
      self._enc_last_c = None
      self._enc_rej = 0
      self.encv_pub = None
      self._clk = _Clock(100.0)
    def get_clock(self): return self._clk

  st = S2()
  for _ in range(5):
    ENC_CB(st, _Msg(1.0))
  ok = st._meas_src == 'enc' and abs(st._meas_v - 1.0) < 1e-6
  print(f'  {"OK " if ok else "✗  "} odom 이 없으면 엔코더를 쓴다 '
        f'(src={st._meas_src} v={st._meas_v:.2f})')
  if not ok:
    f.append('엔코더 폴백')

  ODOM_CB(st, _Odom(2.0))
  before = (st._meas_src, st._meas_v)
  ENC_CB(st, _Msg(9.0))                    # 엔코더가 이상한 값을 줘도
  ok = st._meas_src == 'odom' and abs(st._meas_v - 2.0) < 1e-6
  print(f'  {"OK " if ok else "✗  "} odom 이 신선하면 엔코더가 못 덮어쓴다 '
        f'(src={st._meas_src} v={st._meas_v:.2f})')
  if not ok:
    f.append('odom 우선')

  st._clk.t = 101.0                        # odom 이 1초 낡았다
  ENC_CB(st, _Msg(1.5))
  ok = st._meas_src == 'enc'
  print(f'  {"OK " if ok else "✗  "} odom 이 낡으면(0.3s 초과) 엔코더로 넘어간다 '
        f'(src={st._meas_src})')
  if not ok:
    f.append('odom 만료')

  # 이동평균이 양자화 잡음을 누르는가 (1카운트 = 0.0575 m/s)
  st2 = S2()
  for v in (0.0, 0.115, 0.0, 0.115, 0.0):
    ENC_CB(st2, _Msg(v))
  ok = 0.02 < st2._meas_v < 0.08
  print(f'  {"OK " if ok else "✗  "} 이동평균이 잡음을 누른다 '
        f'(0/0.115 반복 → {st2._meas_v:.3f})')
  if not ok:
    f.append('이동평균')


  # ================================================================
  # 엔코더 위치차분 추정기 (2026-09-17) — 펌웨어 VEL 이 못 쓸 물건이라서
  # ================================================================
  print()
  print('■ 엔코더 위치차분 추정기 — 잡음을 구조적으로 없앤다')
  ENCPOS = SerialBridgeNode._enc_pos_update
  MPC = SerialBridgeNode.ENC_M_PER_COUNT

  def feed(st, speed_fn, secs, dt=0.01, t0=100.0):
    """speed_fn(t) 로 움직이는 차를 dt 간격으로 먹인다. 카운트는 정수로 끊는다."""
    d = 0.0
    t = t0
    st._clk.t = t
    ENCPOS(st, 0)
    n = int(secs / dt)
    for i in range(n):
      t += dt
      d += speed_fn(t - t0) * dt
      st._clk.t = t
      ENCPOS(st, -int(round(d / MPC)))      # 전진하면 카운트가 준다
    return d

  # ① 등속 — 정확도
  st = S2()
  feed(st, lambda t: 1.00, 1.0)
  ok = st._meas_src == 'encpos' and abs(st._meas_v - 1.00) < 0.02
  print(f'  {"OK " if ok else "✗  "} 등속 1.00 m/s → {st._meas_v:.3f} (오차 <0.02)')
  if not ok: f.append('등속 정확도')

  # ② 저속 양자화 — 펌웨어 10ms 차분과 정면 비교
  st = S2()
  feed(st, lambda t: 0.30, 1.0)
  fw_quantum = MPC / 0.01                   # 펌웨어가 10ms 로 차분할 때의 눈금
  ok = abs(st._meas_v - 0.30) < 0.03
  print(f'  {"OK " if ok else "✗  "} 저속 0.30 m/s → {st._meas_v:.3f} '
        f'(펌웨어 눈금은 {fw_quantum:.3f} m/s — 0.30 을 낼 수가 없다)')
  if not ok: f.append('저속 양자화')

  # ③ SPI 오독 1발은 무시한다
  st = S2()
  feed(st, lambda t: 1.00, 0.5)
  good = st._meas_v
  st._clk.t += 0.01
  ENCPOS(st, st._enc_last_c - 5000)         # 점프
  ok = abs(st._meas_v - good) < 1e-9        # 값이 안 바뀌었다
  print(f'  {"OK " if ok else "✗  "} 1샘플 5000카운트 점프를 기각한다 '
        f'({good:.2f} 유지)')
  if not ok: f.append('점프 기각')

  # ④ 진짜 리셋(3연속)이면 복구한다 — 영영 굳으면 안 된다
  st = S2()
  feed(st, lambda t: 1.00, 0.5)
  base = st._enc_last_c - 50000
  for i in range(4):
    st._clk.t += 0.01
    ENCPOS(st, base - i)
  ok = st._enc_last_c is not None and abs(st._enc_last_c - base) <= 4
  print(f'  {"OK " if ok else "✗  "} 3연속이면 리셋으로 보고 창을 버린다 '
        f'(추종 재개)')
  if not ok: f.append('리셋 복구')

  # ⑤ 가속도 — 등가속 1.5 m/s^2
  st = S2()
  feed(st, lambda t: 1.5 * t, 1.0)
  ok = abs(st._meas_a - 1.5) < 0.3
  print(f'  {"OK " if ok else "✗  "} 등가속 1.50 m/s² → {st._meas_a:.2f} (오차 <0.3)')
  if not ok: f.append('가속도 추정')

  # ⑥ 창이 덜 차면 아직 안 믿는다
  st = S2()
  st._clk.t = 100.0
  ENCPOS(st, 0)
  for i in range(3):
    st._clk.t += 0.01
    ENCPOS(st, -i)
  ok = st._meas_v is None
  print(f'  {"OK " if ok else "✗  "} 창이 덜 차면 측정을 내놓지 않는다')
  if not ok: f.append('창 미충족')

  # ⑦ 우선순위 — encpos 가 odom·펌웨어VEL 을 이긴다
  st = S2()
  feed(st, lambda t: 1.00, 0.5)
  ODOM_CB(st, _Odom(61.9))                  # EKF 스파이크
  ENC_CB(st, _Msg(3.05))                    # 펌웨어 VEL 스파이크
  ok = st._meas_src == 'encpos' and abs(st._meas_v - 1.00) < 0.02
  print(f'  {"OK " if ok else "✗  "} odom 61.9 · 펌웨어 3.05 가 와도 안 밀린다 '
        f'(src={st._meas_src} v={st._meas_v:.2f})')
  if not ok: f.append('encpos 우선')

  # ================================================================
  # 선행(가속도) 항 — 넘고 나서가 아니라 넘기 전에 잡는다
  # ================================================================
  print()
  print('■ 선행항 — 내리막 runaway 를 미리 잡는다')

  # ⑧ 아직 안 넘었지만 붙는 중이면 개입한다
  run('P 단독이면 아직 안 건다 (측정=명령)',
      1.11, 1.11, None, f, gov_lead_s=0.0)
  run('선행 0.3s · 가속 +2.0 이면 미리 깎는다',
      1.11, 1.11, 60.3 - 100 * (0.6 - 0.10), f,   # lead=0.6 → 초과 0.5
      gov_lead_s=0.3, gov_gain=100.0, _meas_a=2.0, _meas_a_t=1e9)

  # ⑨ 감속 중이면 개입을 푼다 (헛제동 방지)
  run('과속이어도 감속 중이면 안 건다',
      1.11, 1.30, None, f, gov_lead_s=0.3, _meas_a=-8.0, _meas_a_t=1e9)

  # ⑩ 선행 포화 — 잡음이 지배하지 못한다
  st = Stub(gov_lead_s=0.3, gov_gain=50.0, _meas_a=5.0, _meas_a_t=1e9,
            _meas_v=1.11, _meas_v_t=1e9)
  st.get_clock = lambda: None
  out = GOV(st, 1.11, 60.3, 1e9)
  want = 60.3 - 50 * (1.0 - 0.10)           # lead 1.5 가 1.0 으로 포화
  ok = out is not None and abs(out - want) < 2
  print(f'  {"OK " if ok else "✗  "} 선행은 ±1.0 m/s 로 포화한다 '
        f'(a=5 → {out:.0f}, 기대 {want:.0f} · 포화 없으면 -54)')
  if not ok: f.append('선행 포화')

  # ⑪ 가속도가 낡으면 선행 없이 P 로만
  run('가속도가 낡으면 선행을 안 쓴다',
      1.11, 1.11, None, f, gov_lead_s=0.3, _meas_a=2.0, _meas_a_t=0.0)

  # ================================================================
  # 실측 재현 — 4 km/h(1.11 m/s) 목표로 그 순간을 다시 돌린다
  # ================================================================
  print()
  print('■ 실측 재현 — 2026-09-17 내리막 t=4.07s (실속 0.73, 붙는 중)')
  #   그때 거버너는 PWM 71 을 유지했고 0.4초 뒤 2.53 m/s 로 중단됐다.
  #   4km/h 목표 + 선행이면 그 순간에 이미 깎아야 한다.
  ff4 = 38.8 * 1.11 + 17.2                  # 60.3
  st = Stub(gov_lead_s=0.3, gov_pwm=60.0, _meas_a=2.4, _meas_a_t=1e9,
            _meas_v=0.73, _meas_v_t=1e9)
  out = GOV(st, 1.11, ff4, 1e9)
  # v_pred = 0.73 + 0.72 = 1.45 → 초과 0.24 → 60.3 - 72 = -12 (제동)
  ok = out is not None and out < 10
  print(f'  {"OK " if ok else "✗  "} 그 순간 이미 깎는다 '
        f'(FF {ff4:.0f} → {out if out is None else round(out)})')
  if not ok: f.append('실측 재현')

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 거버너가 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

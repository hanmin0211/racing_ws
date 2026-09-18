#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ramp_profile.py — 경사로 주행을 **현장에 가기 전에** 끝까지 돌려 본다.

★ 왜 (2026-09-18)
  어제까지의 경사로 시험은 웨이포인트 없이 teleop 으로 직진만 했다. 오늘부터는
  GPS+IMU 헤딩과 웨이포인트를 쓴다. 그러면 순서가 이렇게 된다:

      s=0 ────10m 헤딩 캘리브────► s=10 (정지) ────가속────► s=21 경사로 진입

  여기서 물어야 할 것은 셋인데, 전부 **차를 올리기 전에** 답이 나와야 한다.

      ① 캘리브가 끝나고 남은 거리로 목표속도에 **닿는가**
      ② 그 속도로 경사로를 **올라가는가** (개루프 FF 는 중력을 모른다)
      ③ 내리막에서 속도가 **물리는가** (거버너·경사보상이 실제로 잡는가)

  ①은 거리 예산, ②③은 PWM 예산이다. 셋 다 계산으로 답이 나오고, 답이
  '아니오' 인 조합을 현장에서 발견하면 그날 하루가 날아간다.

★ 무엇을 시뮬레이션하지 '않는가'
  제어 로직은 **하나도 다시 구현하지 않았다.** 실제 노드의 함수를 그대로
  부른다(RAMP_RUNBOOK 의 원칙 4번, replay_governor.py 와 같은 방식):

      longitudinal_controller_node._base_speed()   ← 구간 속도
      longitudinal_controller_node.decide_target() ← 곡률·정지선·장애물 min
      serial_bridge_node._grade_pwm()              ← IMU 경사 보상
      serial_bridge_node._governor_pwm()           ← 내리막 거버너
      serial_bridge_node._ff_output()              ← 최종 PWM
      serial_bridge_node._enc_pos_update()         ← 엔코더 위치차분 추정기

  새로 쓴 것은 **차량 물리 모델 하나뿐**이고, 그건 아래처럼 검증했다.

★ 차량 모델과 그 근거 (여기가 이 도구의 신뢰도 전부다)

      a = K·PWM − C·v − F0 − 9.81·sin(θ)          [m/s²]

  세 상수는 **로그 하나에서만** 뽑았다 — data/2026-09-17-ramp/ramp_평지기준_2259.csv
  (평지, PWM 105 고정, 14m 주행). 그 로그의 세 지점으로 연립했다:

      t=0.88→1.42  v 0.31→0.97   a = 1.22 m/s²
      t=3.54→5.14  v 1.74→1.97   a = 0.144 m/s²
      정상상태                    v = 2.06 m/s

      → C = 0.861 /s,  K·105 − F0 = 1.737
      → F0 = 0.37 (관성 감속 실측 COAST_DECEL 0.37 을 그대로 씀)
      → K = (1.737+0.37)/105 = 0.02007

  K 가 저장소의 실측 상수 **0.0202 와 0.5% 안에서 같다.** 그 값은 경사
  등반으로 따로 구한 것이라(HANDOFF §3-1), 서로 다른 실험이 같은 답을 냈다.

  ★ 교차검증 — 아래 로그들은 상수를 뽑을 때 **보지 않았다.** 모델로 경사를
    역산하면:

      ramp_평지기준_2259.csv   →  +0.8 %   (평지니까 0 이어야 한다)
      ramp_PWM160_2302.csv     → +19.3 %   (등반 로그)
      ramp_조주5_2212.csv      →  +9.2 %   (경사 진입 도중)

    +19.3% 는 HANDOFF §3-1 이 **운동방정식으로 따로 역산한 19.3%** 와 같다.
    재현 명령은 tools/test_ramp_profile.py 에 시험으로 박아 두었다.

  ⚠ 이 모델이 말해 주지 않는 것: 바퀴 슬립, 배터리 전압 강하, 조향 부하.
    로그마다 같은 PWM 에서 최고속도가 1.7~2.3 으로 흩어지는데(전압·노면),
    이 모델은 그 한가운데를 간다. **여유를 보고 읽을 것.**

★ 이 도구가 찾아낸 것 (기본값으로 돌려 보면 나온다)
  개루프 FF 식 `PWM = 38.8·v + 17.2` 은 위 모델 기준으로 **기울기가 얕다.**
  정상상태가 맞으려면 `42.6·v + 18.3` 이어야 한다. 그래서 2.0 m/s 를
  명령하면 PWM 94.8 이 나가고 차는 **약 1.8 m/s 에서 물린다.** 못 가는 게
  아니라 덜 가는 것이고(안전한 방향), 대신 '2.0 을 시켰는데 왜 1.8 이냐' 로
  현장에서 헤매지 않으려면 알고 있어야 한다.

사용:
  # 오늘 계획 그대로 (캘리브 10m + 가속 11m + 법정 경사)
  python3 tools/ramp_profile.py --grade 12.5 --v-up 2.0 --v-down 1.11 \\
      --grade-ff-gain 1.0 --gov-pwm 50

  # 경사 보상을 안 켜면 어떻게 되는지 (= 어제 상태)
  python3 tools/ramp_profile.py --grade 12.5 --grade-ff-gain 0

  # ⚠ IMU 부호가 뒤집혔을 때 무슨 일이 나는지 (RAMP_RUNBOOK ② 를 왜 하는가)
  python3 tools/ramp_profile.py --grade 12.5 --grade-ff-gain 1.0 --imu-sign -1

  # 경사도를 모를 때 — 훑어서 표로 본다
  python3 tools/ramp_profile.py --sweep 8,10,12.5,15,19.3 --v-up 2.0
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller import longitudinal_controller_node as L   # noqa: E402
from velocity_controller.serial_bridge_node import SerialBridgeNode as SB  # noqa: E402

# ── 차량 물리 모델 (근거는 위 docstring) ────────────────────────────────
K_PWM = 0.0202      # [m/s²/PWM] 저장소 실측 상수. 평지 로그 역산과 0.5% 일치
C_VEL = 0.861       # [1/s] 역기전력 항. ramp_평지기준_2259.csv 에서 연립
F0 = 0.37           # [m/s²] 구동 중 구름저항. 관성감속 실측(COAST_DECEL)
G = 9.81

SIM_HZ = 200.0      # 물리 적분
CTRL_HZ = 20.0      # longitudinal_controller / serial_bridge 주기
TELEM_HZ = 20.0     # STATUS_10ms 실제 도착률 (이름과 달리 20Hz — HANDOFF §3)


class _Clock:
  """rclpy Clock 흉내. 시뮬 시각을 노드 코드에 그대로 먹인다."""

  def __init__(self):
    self.t = 0.0

  def now(self):
    return type('', (), {'nanoseconds': self.t * 1e9})()


class _Log:
  """노드 로거 자리. 거버너가 매 틱 info 를 찍으므로 삼킨다."""

  def info(self, *a, **k):
    pass

  def warn(self, *a, **k):
    pass

  def error(self, *a, **k):
    pass


class Bridge:
  """serial_bridge_node 의 **상태만** 흉내 낸 껍데기.

  로직은 원본 메서드를 그대로 붙여 쓴다 (replay_governor.py 와 같은 방식).
  여기서 재구현한 계산은 하나도 없다 — 있으면 시험이 실제 코드를 안 본다.
  """

  # 원본 클래스 상수를 그대로 가져온다. 값을 베끼면 원본이 바뀔 때 갈라진다.
  for _k in ('ENC_M_PER_COUNT', 'ENC_FORWARD_SIGN', 'ENC_JUMP_COUNTS',
             'ENC_WIN_S', 'ENC_ACC_CAP', 'ENC_ACC_TAU', 'GOV_LEAD_CAP',
             'BRAKE_MEAS_FRESH_S', 'BRAKE_V_PLAUSIBLE', 'GRADE_FRESH_S',
             'GOV_V_PLAUSIBLE_ENCPOS',
             'GRADE_MAX_RAD', 'GRADE_TAU', 'K_ACCEL_PER_PWM'):
    locals()[_k] = getattr(SB, _k)
  del _k

  # 실제 메서드 — 이 네 개가 이 도구가 검증하려는 대상 전부다.
  _grade_pwm = SB._grade_pwm
  _ff_output = SB._ff_output
  _governor_pwm = SB._governor_pwm
  _enc_pos_update = SB._enc_pos_update
  _imu_cb = SB._imu_cb
  _gov_v_plausible = SB._gov_v_plausible
  _gov_grade_ok = SB._gov_grade_ok

  def __init__(self, a):
    self.clock = _Clock()
    self._log = _Log()
    # FF
    self.ff_static = a.ff_static
    self.ff_gain = a.ff_gain
    self.ff_deadband = 0.05
    self.ff_min_pwm = a.ff_min_pwm
    self.ff_breakaway_pwm = a.ff_breakaway_pwm
    self.ff_breakaway_s = a.ff_breakaway_ms / 1000.0
    # 거버너
    self.gov_pwm = a.gov_pwm
    self.gov_deadband = a.gov_deadband
    self.gov_gain = a.gov_gain
    self.gov_lead_s = a.gov_lead_s
    self.gov_min_grade = getattr(a, 'gov_min_grade', 0.0)
    # 경사 보상
    self.grade_ff_gain = a.grade_ff_gain
    self.grade_ff_max = a.grade_ff_max
    self._pitch_sign = float(a.imu_sign)
    self._pitch_off = 0.0
    self._pitch = None
    self._pitch_t = -1e9
    # 추정기 상태
    self._enc_hist = []
    self._enc_last_c = None
    self._enc_rej = 0
    self._meas_v = None
    self._meas_v_t = -1e9
    self._meas_a = 0.0
    self._meas_a_t = 0.0
    self._meas_src = None
    self.encv_pub = None
    self._moving_since = None

  # 노드가 부르는 것들
  def get_clock(self):
    return self.clock

  def get_logger(self):
    return self._log

  def feed_pitch(self, pitch_rad):
    """IMU 대신 참 경사각을 넣는다. _imu_cb 의 저역통과를 그대로 태운다."""
    q = type('', (), {'w': math.cos(pitch_rad / 2), 'x': 0.0,
                      'y': math.sin(pitch_rad / 2), 'z': 0.0})()
    self._imu_cb(type('', (), {'orientation': q})())

  def command_pwm(self, v):
    """serial_bridge_node 의 ff_mode='ros' 분기를 **같은 순서로** 재현한다.

    순서가 곧 안전장치다 (원본 868~918 줄):
        ① 경사 보상 → ② 거버너(개입하면 하한을 건너뛰고 즉시 반환)
        → ③ 정지마찰 → ④ _ff_output(하한·포화)
    """
    now = self.clock.t
    if abs(v) < self.ff_deadband:
      self._moving_since = None
      return 0.0, 0.0, None
    sgn = 1.0 if v > 0 else -1.0
    grade = self._grade_pwm(now)
    ff_mag = self.ff_static + self.ff_gain * abs(v) + grade * sgn
    gov = self._governor_pwm(v, ff_mag, now)
    if gov is not None:
      return float(round(gov)), grade, gov
    if self._moving_since is None:
      self._moving_since = now
    brk = (now - self._moving_since) < self.ff_breakaway_s
    return float(round(self._ff_output(v, grade, brk))), grade, None


class Longitudinal:
  """longitudinal_controller_node 의 상태 껍데기 (test_ramp_section.py 와 동형)."""

  _base_speed = L.LongitudinalController._base_speed
  decide_target = L.LongitudinalController.decide_target

  def __init__(self, a):
    self.v_max = a.v_max
    self.v_min = a.v_min
    self.v_slow = 0.8
    self.curv_gain = 6.0
    self.kappa = 0.0            # 경사로는 직선이다. 곡률이 있으면 여기에 넣는다
    self.mission = 'DRIVE'
    self.stop_line_dist = 999.0
    self.stop_trigger = 19.0
    self.stop_decel = 1.0
    self.obstacle_dist = 999.0
    self.obs_trigger = 4.0
    self.obs_stop = 0.8
    self.v_ramp_up = a.v_up
    self.v_ramp_down = a.v_down
    self.ramp_arm_timeout = 1.0
    self._ramp_up_on = False
    self._ramp_up_t = 0.0
    self._ramp_down_on = False
    self._ramp_down_t = 0.0
    self.max_accel = a.max_accel
    self.max_decel = a.max_decel
    self.profiled_speed = 0.0
    self.dt = 1.0 / CTRL_HZ

  def tick(self, now):
    """control_loop 의 슬루레이트 부분. 완주·경로끊김 분기는 여기 없다."""
    raw = self.decide_target()
    d = raw - self.profiled_speed
    if d > 0:
      self.profiled_speed += min(d, self.max_accel * self.dt)
    elif d < 0:
      self.profiled_speed -= min(-d, self.max_decel * self.dt)
    return self.profiled_speed


class Course:
  """경사로 종단면. s [m] → 경사 sinθ (오르막이 +)."""

  def __init__(self, a):
    self.s_ramp = a.calib + a.approach       # 오르막 시작
    self.s_crest = self.s_ramp + a.ramp_len  # 정상부 시작
    self.s_down = self.s_crest + a.crest_len  # 내리막 시작
    self.s_end = self.s_down + a.down_len    # 평지 복귀
    self.grade = a.grade / 100.0
    # 경사 진입/이탈은 축거(0.785m)만큼 서서히 실린다. 계단으로 두면
    # 없는 충격이 생기고, 거버너 선행항이 그 계단을 가속으로 읽는다.
    self.blend = 0.785

  def sin_theta(self, s):
    def ramp(x):      # 0→1 선형 전이
      return max(0.0, min(1.0, x / self.blend))
    up = ramp(s - self.s_ramp) * (1.0 - ramp(s - self.s_crest))
    dn = ramp(s - self.s_down) * (1.0 - ramp(s - self.s_end))
    return self.grade * (up - dn)


def simulate(a, k_scale=1.0):
  """캘리브 종료 지점(정지 상태)에서 출발해 경사로를 넘을 때까지 돌린다.

  k_scale — 구동 효율을 깎는다. 같은 PWM 이 덜 미는 상황(배터리 소모,
  노면, 개체차)을 본다. **이게 왜 필요한가**: 9/17 로그는 비슷한 PWM 에서
  최고속도가 1.4~2.3 m/s 로 흩어졌다. 한 점만 보고 '된다' 고 하면
  현장에서 그 흩어짐의 나쁜 쪽에 걸린다.
  """
  course = Course(a)
  br = Bridge(a)
  lon = Longitudinal(a)

  s = a.calib          # 캘리브가 끝난 자리. 차는 서 있다 (heading_init 이 세운다)
  v = 0.0
  t = 0.0
  dt = 1.0 / SIM_HZ
  enc = 0.0            # 엔코더 카운트(실수 누적 → 정수로 양자화해 먹인다)
  next_ctrl = 0.0
  next_tel = 0.0
  pwm = 0.0
  grade_pwm = 0.0
  gov = None
  a_veh = 0.0        # 직전 주기의 차량 가속도 (IMU 오염 모델 입력)
  rows = []
  t_max = a.timeout

  while t < t_max and s < course.s_end + a.runout:
    # ── 텔레메트리 (20Hz) — 엔코더 카운트를 정수로 양자화해서 먹인다.
    #    양자화를 빼면 거버너가 실제보다 얌전해 보인다(그게 어제의 오진이었다).
    if t >= next_tel:
      next_tel += 1.0 / TELEM_HZ
      br.clock.t = t
      br._enc_pos_update(int(round(enc)))
      # ★ 가속 오염 (2026-09-18)
      #   IMU 는 중력 방향으로 기울기를 잰다. 앞으로 가속하면 관성력이
      #   중력에 더해져 **코가 들린 것처럼** 보인다: atan(a/g).
      #   1.0 m/s² 가속 = 5.8° = 경사 10% 오독 = PWM 49 의 헛보상이다.
      #   하필 오늘 계획이 11m 를 가속하다 바로 경사로에 드는 거라, 가속
      #   구간 내내 이게 실린다.
      #
      #   A9 는 자이로 융합(AHRS)이라 지속 가속을 **얼마간 거부한다.**
      #   얼마나 거부하는지는 코드로 알 수 없다 — 평지에서 재야 한다
      #   (RAMP_RUNBOOK ③: --measure --expect 0).
      #   그래서 거부율을 인자로 두고 **최악(전혀 거부 못 함)까지** 본다.
      bias = 0.0
      if a.imu_accel_frac > 0.0:
        bias = a.imu_accel_frac * math.atan(a_veh / G)
      br.feed_pitch(math.asin(course.sin_theta(s)) + bias)

    # ── 제어 (20Hz)
    if t >= next_ctrl:
      next_ctrl += 1.0 / CTRL_HZ
      br.clock.t = t
      # 시퀀서: course_s 로 arm 을 낸다 (mission_sequencer 가 하는 일)
      lon._ramp_up_on = a.up_arm_s <= s < course.s_crest
      lon._ramp_down_on = course.s_crest <= s < course.s_end + a.down_exit
      # ★ arm 시각만은 시뮬 시계가 아니라 **벽시계**로 준다.
      #   _base_speed() 가 time.time() 으로 신선도를 보기 때문이다. 시뮬
      #   시계(0부터 시작)를 넣으면 전부 '낡았다' 로 판정돼 구간 속도가
      #   통째로 안 걸린다 — 그러면 이 시뮬이 아무것도 검증하지 못한다.
      lon._ramp_up_t = lon._ramp_down_t = time.time()
      cmd = lon.tick(t)
      pwm, grade_pwm, gov = br.command_pwm(cmd)
      rows.append((t, s, v, cmd, pwm, grade_pwm, gov,
                   course.sin_theta(s) * 100.0))

    # ── 물리
    sin_t = course.sin_theta(s)
    if pwm == 0.0 and v <= 0.01:
      a_veh = 0.0
    elif pwm == 0.0:
      # 무동력 관성: 역기전력이 없다(프리휠). 구름저항만 — 실측 0.37
      a_veh = -math.copysign(F0, v) - G * sin_t
    else:
      a_veh = (K_PWM * k_scale) * pwm - C_VEL * v - math.copysign(F0, v) \
          - G * sin_t
    v += a_veh * dt
    if v < 0.0 and pwm <= 0.0:
      v = 0.0           # 뒤로 밀리는 것은 이 모델의 범위 밖 (§미해결 1번)
    s += v * dt
    enc += -(v * dt) / Bridge.ENC_M_PER_COUNT   # 전진하면 카운트가 준다
    t += dt

  return course, rows


def report(a, course, rows):
  """판정. 숫자 하나가 아니라 '무엇이 왜 위험한가' 를 낸다."""
  if not rows:
    print('표본 없음')
    return 1

  def seg(lo, hi):
    return [r for r in rows if lo <= r[1] < hi]

  approach = seg(a.calib, course.s_ramp)
  climb = seg(course.s_ramp, course.s_crest)
  desc = seg(course.s_down, course.s_end)

  print()
  print(f'  경사 {a.grade:.1f}%   오르막 {a.ramp_len:.0f}m · 정상부 '
        f'{a.crest_len:.0f}m · 내리막 {a.down_len:.0f}m')
  print(f'  캘리브 {a.calib:.0f}m → 가속 {a.approach:.0f}m → '
        f'경사로 진입 s={course.s_ramp:.0f}m')
  print(f'  목표  오르막 {a.v_up:.2f} · 내리막 {a.v_down:.2f} m/s   '
        f'grade_ff_gain={a.grade_ff_gain} (max {a.grade_ff_max:.0f}) · '
        f'gov_pwm={a.gov_pwm:.0f}')
  print()
  print(f'  {"s[m]":>6} {"v[m/s]":>7} {"명령":>6} {"PWM":>6} {"경사PWM":>8} '
        f'{"거버너":>7} {"경사%":>7}')
  step = max(0.5, (course.s_end - a.calib) / 40.0)
  nxt = a.calib
  for (t, s, v, cmd, pwm, gp, gov, g) in rows:
    if s + 1e-9 < nxt:
      continue
    nxt += step
    print(f'  {s:6.1f} {v:7.2f} {cmd:6.2f} {pwm:6.0f} {gp:8.1f} '
          f'{("제동" if (gov is not None and gov < 0) else ("개입" if gov is not None else "-")):>7} '
          f'{g:7.1f}')

  fails, warns = [], []

  # ★ 구간 속도를 안 쓰면(v_ramp_*=0) 기준속도는 v_max 다. 그게 '목표' 다.
  #   예전엔 a.v_up 을 그대로 나눠서 0 이면 죽었다 — 구간 속도를 끄고
  #   비교해 보는 것이 정상적인 사용법인데 도구가 먼저 터졌다.
  v_up_eff = a.v_up if a.v_up > 0.0 else a.v_max
  v_dn_eff = a.v_down if a.v_down > 0.0 else a.v_max

  # ① 접근 구간에서 목표에 닿는가
  if approach:
    v_enter = approach[-1][2]
    reach = v_enter >= 0.90 * v_up_eff
    print()
    print(f'  ① 경사로 진입속도      {v_enter:5.2f} m/s  '
          f'(목표 {v_up_eff:.2f} 의 {v_enter / v_up_eff * 100:.0f}%)')
    if not reach:
      warns.append(
          f'가속 {a.approach:.0f}m 로는 목표 {v_up_eff:.2f} 에 못 닿는다 '
          f'({v_enter:.2f}). 개루프 FF 의 정상편차이고 위험하진 않다 — '
          f'다만 오르막 여유가 그만큼 준다.')

  # ② 오르막을 올라가는가  ← 여기서 떨어지면 그날이 끝난다
  if climb:
    v_min = min(r[2] for r in climb)
    s_min = [r[1] for r in climb if r[2] == v_min][0]
    print(f'  ② 오르막 최저속도      {v_min:5.2f} m/s  @ s={s_min:.1f}m')
    if v_min < 0.15:
      fails.append(
          f'오르막에서 **선다** (s={s_min:.1f}m 에서 {v_min:.2f} m/s). '
          f'개루프 FF 는 중력을 모른다 — grade_ff 를 켜거나 경사가 '
          f'차의 등반한계를 넘은 것이다.')
    elif v_min < 0.5 * v_up_eff:
      warns.append(f'오르막에서 목표의 절반 밑으로 떨어진다 ({v_min:.2f}). '
                   f'grade_ff_gain 을 올리거나 v_up 을 내릴 것.')
    pk = max(abs(r[4]) for r in climb)
    print(f'  ③ 오르막 최대 PWM      {pk:5.0f}      (포화 255)')
    if pk >= 254:
      fails.append(f'PWM 이 포화했다({pk:.0f}). 모터가 더 낼 것이 없다 — '
                   f'이 경사에서 이 속도는 불가능하다.')
    elif pk > 220:
      warns.append(f'PWM {pk:.0f} — 포화까지 여유가 {255 - pk:.0f} 뿐이다. '
                   f'배터리가 빠지면 넘어간다.')

  # ④ 내리막에서 속도가 물리는가
  if desc:
    v_pk = max(r[2] for r in desc)
    s_pk = [r[1] for r in desc if r[2] == v_pk][0]
    over = v_pk - v_dn_eff
    print(f'  ④ 내리막 최고속도      {v_pk:5.2f} m/s  @ s={s_pk:.1f}m  '
          f'(목표 {v_dn_eff:.2f}, 초과 {over:+.2f})')
    if v_pk > 3.0:
      fails.append(
          f'내리막에서 {v_pk:.2f} m/s 까지 붙는다. 굴절코스 진입 상한(3.0)을 '
          f'넘는다 — gov_pwm 을 올리거나 grade_ff 를 켤 것.')
    elif over > 0.6:
      warns.append(f'내리막이 목표보다 {over:.2f} m/s 빠르다. P 제어 정상편차다'
                   f'(HANDOFF §3-2). gov_gain 을 올리면 줄지만 진동과 맞바꾼다.')
    neg = min((r[4] for r in desc), default=0.0)
    if neg < -1e-9:
      print(f'     내리막 최대 역 PWM  {neg:5.0f}      '
            f'(gov_pwm 상한 {a.gov_pwm:.0f})')

  # ⑤ 여유 — 구동이 15% 약해져도 올라가는가.
  #    9/17 로그에서 같은 PWM 의 최고속도가 1.4~2.3 으로 흩어졌다. 한 점이
  #    통과했다고 '된다' 고 쓰면, 현장에서 그 흩어짐의 나쁜 쪽에 걸린다.
  if climb:
    _, weak = simulate(a, k_scale=0.85)
    wc = [r for r in weak if course.s_ramp <= r[1] < course.s_crest]
    w_lo = min((r[2] for r in wc), default=0.0)
    ok = w_lo >= 0.15 and any(r[1] >= course.s_crest for r in weak)
    print(f'  ⑤ 구동 15% 약화 시      {w_lo:5.2f} m/s  '
          f'{"→ 그래도 넘는다" if ok else "→ ❌ 못 넘는다"}')
    if not ok:
      fails.append(
          '구동이 15% 만 약해져도 경사로를 못 넘는다. 배터리 상태에 결과가 '
          '좌우된다 — 이 설정으로는 현장에서 될 때도 있고 안 될 때도 있다.')

  print()
  for w in warns:
    print(f'  ⚠ {w}')
  for f in fails:
    print(f'  ❌ {f}')
  if not fails and not warns:
    print('  ✅ 전 구간 통과 — 진입·등반·하강 모두 여유 안에 있다.')
  elif not fails:
    print('  ✅ 치명 항목 없음 (위 경고는 감점/체감 수준)')
  print()
  return 1 if fails else 0


def main():
  p = argparse.ArgumentParser(
      description='경사로 종방향 프로파일을 실제 제어 코드로 미리 돌려 본다.')
  # 코스 — 오늘 현장 값
  p.add_argument('--calib', type=float, default=10.0,
                 help='헤딩 캘리브 직진 거리[m]. 이 지점에서 차는 선다 (기본 10)')
  p.add_argument('--approach', type=float, default=11.0,
                 help='캘리브 끝 → 경사로 진입까지 남은 가속 거리[m] (기본 11)')
  p.add_argument('--grade', type=float, default=12.5, help='경사도[%%] (기본 12.5)')
  p.add_argument('--ramp-len', type=float, default=8.0, help='오르막 길이[m]')
  p.add_argument('--crest-len', type=float, default=3.0, help='정상부 평탄부[m]')
  p.add_argument('--down-len', type=float, default=8.0, help='내리막 길이[m]')
  p.add_argument('--runout', type=float, default=5.0, help='내리막 뒤 평지[m]')
  # 구간 속도
  p.add_argument('--v-up', type=float, default=2.0, help='오르막 기준속도[m/s]')
  p.add_argument('--v-down', type=float, default=1.11, help='내리막 기준속도[m/s]')
  p.add_argument('--v-max', type=float, default=2.0, help='평소 v_max[m/s]')
  p.add_argument('--v-min', type=float, default=0.25)
  p.add_argument('--up-arm-s', type=float, default=None,
                 help='오르막 구간 arm 시작 s[m]. 기본은 캘리브 끝 '
                      '(= 남은 거리를 전부 가속에 쓴다)')
  p.add_argument('--down-exit', type=float, default=3.0,
                 help='내리막 arm 을 평지 복귀 뒤 몇 m 까지 유지할지')
  p.add_argument('--max-accel', type=float, default=1.0)
  p.add_argument('--max-decel', type=float, default=1.8)
  # serial_bridge
  p.add_argument('--ff-static', type=float, default=17.2)
  p.add_argument('--ff-gain', type=float, default=38.8)
  p.add_argument('--ff-min-pwm', type=float, default=0.0,
                 help='경사 시험에서는 0 으로 준다 (RAMP_RUNBOOK ③)')
  p.add_argument('--ff-breakaway-pwm', type=float, default=60.0)
  p.add_argument('--ff-breakaway-ms', type=float, default=1200.0)
  p.add_argument('--gov-pwm', type=float, default=50.0, help='0 = 거버너 꺼짐')
  p.add_argument('--gov-deadband', type=float, default=0.10)
  p.add_argument('--gov-gain', type=float, default=300.0)
  p.add_argument('--gov-lead-s', type=float, default=0.30)
  p.add_argument('--gov-min-grade', type=float, default=0.0,
                 help='거버너 자세 게이트 — sinθ 기준 내리막 경사. '
                      '0=판정 안 함. 0.03 이면 3%% 보다 급한 내리막에서만')
  p.add_argument('--grade-ff-gain', type=float, default=1.0,
                 help='0 = 경사 보상 꺼짐 (노드 기본값)')
  p.add_argument('--grade-ff-max', type=float, default=70.0)
  p.add_argument('--imu-sign', type=float, default=1.0,
                 help='IMU 피치 부호. -1 을 주면 **부호가 뒤집힌 경우**를 본다')
  p.add_argument('--imu-accel-frac', type=float, default=0.0,
                 help='가속이 피치로 새는 비율. 0=AHRS 가 완전히 거부(기본), '
                      '1=전혀 못 거부(최악). 실제값은 평지에서 잰다 '
                      '(imu_grade.py --measure --expect 0)')
  p.add_argument('--timeout', type=float, default=120.0)
  p.add_argument('--sweep', type=str, default=None,
                 help='경사도 목록을 쉼표로. 표만 낸다 (예: 8,10,12.5,15,19.3)')
  a = p.parse_args()
  if a.up_arm_s is None:
    a.up_arm_s = a.calib

  if a.sweep:
    print(f'\n경사도 훑기 — v_up={a.v_up} v_down={a.v_down} '
          f'grade_ff_gain={a.grade_ff_gain} gov_pwm={a.gov_pwm:.0f}')
    print(f'  {"경사%":>7} {"진입v":>7} {"등반최저":>9} {"최대PWM":>8} '
          f'{"하강최고":>9} {"15%약화":>8}  판정')
    bad = 0
    for g in [float(x) for x in a.sweep.split(',')]:
      a.grade = g
      course, rows = simulate(a)
      ap = [r for r in rows if a.calib <= r[1] < course.s_ramp]
      cl = [r for r in rows if course.s_ramp <= r[1] < course.s_crest]
      de = [r for r in rows if course.s_down <= r[1] < course.s_end]
      v_in = ap[-1][2] if ap else 0.0
      v_lo = min((r[2] for r in cl), default=0.0)
      pk = max((abs(r[4]) for r in cl), default=0.0)
      # 내리막에 **닿지 못했으면** 0 이 아니라 '—' 다. 0 으로 찍으면
      # '천천히 잘 내려왔다' 로 읽힌다 — 사실은 오르막에서 선 것이다.
      v_hi = max((r[2] for r in de), default=None)
      _, weak = simulate(a, k_scale=0.85)
      w_lo = min((r[2] for r in weak
                  if course.s_ramp <= r[1] < course.s_crest), default=0.0)
      if v_lo < 0.15:
        verdict, bad = '❌ 못 올라감', bad + 1
      elif pk >= 254:
        verdict, bad = '❌ PWM 포화', bad + 1
      elif v_hi is not None and v_hi > 3.0:
        verdict, bad = '❌ 하강 과속', bad + 1
      elif w_lo < 0.15:
        verdict, bad = '❌ 여유없음', bad + 1
      elif v_lo < 0.5 * (a.v_up if a.v_up > 0 else a.v_max):
        verdict = '⚠ 등반 둔화'
      else:
        verdict = '✅'
      print(f'  {g:7.1f} {v_in:7.2f} {v_lo:9.2f} {pk:8.0f} '
            f'{("—" if v_hi is None else f"{v_hi:.2f}"):>9} {w_lo:8.2f}  {verdict}')
    print()
    print('  15%약화 = 같은 PWM 이 15% 덜 미는 경우의 등반 최저속도.')
    print('  9/17 로그가 같은 PWM 에서 1.4~2.3 m/s 로 흩어졌으므로, 이 칸이')
    print('  0.15 밑이면 "될 때도 있고 안 될 때도 있는" 설정이다.')
    print()
    return 1 if bad else 0

  course, rows = simulate(a)
  return report(a, course, rows)


if __name__ == '__main__':
  sys.exit(main())

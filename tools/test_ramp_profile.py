#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_ramp_profile.py — ramp_profile.py 의 **차량 모델과 판정**을 못 박는다.

★ 왜 이 시험이 따로 필요한가
  ramp_profile.py 는 "현장에 가기 전에 답을 낸다" 는 도구다. 그 답이 틀리면
  틀린 확신을 갖고 현장에 간다 — 아무 도구도 없는 것보다 나쁘다. 그래서
  이 시험은 두 가지를 본다:

    ① 모델이 **저장소 로그를 재현하는가** (합성 입력이 아니라 실측이다)
    ② 판정이 **위험을 실제로 잡아내는가** (반례가 ❌ 로 떨어지는가)

  ②가 없으면 '전부 ✅ 를 내는 도구' 가 되고, 그건 검증이 아니라 위안이다.

★ 모델을 어디서 뽑았고 어디로 검증했는가
  상수(K·C·F0)는 **ramp_평지기준_2259.csv 한 개**에서만 뽑았다.
  아래 시험이 쓰는 ramp_PWM160_2302.csv 와 조주5 로그는 그때 **보지 않았다.**
  그래서 이 시험은 진짜 교차검증이다 — 같은 데이터로 맞추고 같은 데이터로
  확인하는 순환이 아니다.

  python3 tools/test_ramp_profile.py
"""

import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, HERE)

import ramp_profile as RP                                    # noqa: E402

DATA = os.path.join(ROOT, 'data', '2026-09-17-ramp')
M_PER_COUNT = 2.0 * math.pi * 0.1327 / 290.0

fails = []
skipped = []


def chk(name, ok, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   {detail}' if detail else ''))
  if not ok:
    fails.append(name)


# ─────────────────────────────────────────────────────────────────────
# ① 모델이 저장소 로그를 재현하는가
#
# a = K·PWM − C·v − F0 − 9.81·sinθ  를 sinθ 에 대해 풀어, 로그마다
# '이 주행이 몇 % 경사였나' 를 역산한다. 답을 우리는 독립적으로 안다:
#   평지기준  → 0 %     (평평한 데서 쟀으니까)
#   PWM160    → 19.3 %  (HANDOFF §3-1 이 운동방정식으로 따로 역산한 값)
# ─────────────────────────────────────────────────────────────────────

def _grade_from_log(path, win=0.30):
  """로그의 엔코더로 속도·가속도를 내고, 모델로 경사를 역산해 중앙값을 낸다."""
  rows = []
  with open(path) as fh:
    for r in csv.DictReader(fh):
      try:
        rows.append((float(r['t']), int(r['enc']), float(r['pwm'])))
      except (ValueError, KeyError, TypeError):
        continue
  # 카운트가 바뀐 행만 — 텔레메트리는 20Hz 인데 기록은 41Hz 라 61% 가
  # 같은 값이다. 그대로 먹이면 가짜 정지구간이 생긴다
  # (replay_governor.py 의 --dedupe 와 같은 이유).
  ded = []
  for x in rows:
    if not ded or ded[-1][1] != x[1]:
      ded.append(x)
  prof = []
  for i, (t, e, p) in enumerate(ded):
    j = i
    while j > 0 and t - ded[j][0] < win:
      j -= 1
    dt = t - ded[j][0]
    if dt >= win * 0.5:
      prof.append((t, -(e - ded[j][1]) * M_PER_COUNT / dt, p))
  out = []
  for i in range(4, len(prof) - 4):
    t, v, p = prof[i]
    if v < 0.4:                      # 저속은 양자화가 지배한다
      continue
    dt = prof[i + 4][0] - prof[i - 4][0]
    if dt <= 0.2:
      continue
    a = (prof[i + 4][1] - prof[i - 4][1]) / dt
    out.append((RP.K_PWM * p - RP.C_VEL * v - RP.F0 - a) / 9.81 * 100.0)
  out.sort()
  return (out[len(out) // 2], len(out)) if out else (None, 0)


print('① 차량 모델이 저장소 로그를 재현하는가 (상수는 평지 로그에서만 뽑았다)')

CASES = [
    ('ramp_평지기준_2259.csv', 0.0, 2.0, '평지 — 0% 여야 한다'),
    ('ramp_PWM160_2302.csv', 19.3, 3.0, '등반 — HANDOFF §3-1 의 19.3%'),
]
for fname, want, tol, why in CASES:
  path = os.path.join(DATA, fname)
  if not os.path.exists(path):
    # ★ 픽스처가 없으면 조용히 건너뛰지 않는다. 예전에 그래서 '초록'
    #   이 거짓이었다 (memory: tests-silently-skip-missing-fixtures).
    skipped.append(fname)
    print(f'  ⚠  건너뜀 — 로그가 없다: {path}')
    continue
  got, n = _grade_from_log(path)
  chk(f'{fname} → {want}% ({why})',
      got is not None and abs(got - want) <= tol,
      f'역산 {got:+.1f}% (허용 ±{tol}, 표본 {n})' if got is not None else '표본 0')

# 평지 정상속도: PWM 105 를 계속 주면 모델이 2.0~2.1 로 수렴해야 한다
#   (로그 실측 2.06). 모델의 정상상태가 로그와 맞는지 직접 본다.
v_ss = (RP.K_PWM * 105.0 - RP.F0) / RP.C_VEL
chk('평지 PWM 105 정상속도 = 로그의 2.06 m/s',
    abs(v_ss - 2.06) <= 0.10, f'모델 {v_ss:.2f} m/s')


# ─────────────────────────────────────────────────────────────────────
# ② 판정이 위험을 잡아내는가 — 반례가 ❌ 로 떨어져야 한다
# ─────────────────────────────────────────────────────────────────────

class A:
  """ramp_profile 의 인자 묶음. 기본은 오늘 현장 계획값."""

  def __init__(self, **kw):
    self.calib, self.approach = 10.0, 11.0
    self.grade = 12.5
    self.ramp_len, self.crest_len, self.down_len, self.runout = 8.0, 3.0, 8.0, 5.0
    self.v_up, self.v_down, self.v_max, self.v_min = 2.0, 1.11, 2.0, 0.25
    self.up_arm_s, self.down_exit = 10.0, 3.0
    self.max_accel, self.max_decel = 1.0, 1.8
    self.ff_static, self.ff_gain = 17.2, 38.8
    self.ff_min_pwm = 0.0
    self.ff_breakaway_pwm, self.ff_breakaway_ms = 60.0, 1200.0
    self.gov_pwm, self.gov_deadband = 50.0, 0.10
    self.gov_gain, self.gov_lead_s = 300.0, 0.30
    self.grade_ff_gain, self.grade_ff_max = 1.0, 70.0
    self.imu_sign = 1.0
    self.imu_accel_frac = 0.0   # 가속 오염 없음이 기준선
    self.timeout = 120.0
    self.sweep = None
    self.__dict__.update(kw)


def run(a, k_scale=1.0):
  """(진입속도, 등반최저, 등반최대PWM, 하강최고 or None) 을 낸다."""
  course, rows = RP.simulate(a, k_scale=k_scale)
  ap = [r for r in rows if a.calib <= r[1] < course.s_ramp]
  cl = [r for r in rows if course.s_ramp <= r[1] < course.s_crest]
  de = [r for r in rows if course.s_down <= r[1] < course.s_end]
  return (ap[-1][2] if ap else 0.0,
          min((r[2] for r in cl), default=0.0),
          max((abs(r[4]) for r in cl), default=0.0),
          max((r[2] for r in de), default=None))


print()
print('② 판정이 위험을 잡아내는가 (반례가 통과하면 이 도구는 쓸모없다)')

v_in, v_lo, pk, v_hi = run(A())
chk('계획값(12.5%·grade_ff 1.0) 은 넘어간다', v_lo > 0.5 and v_hi is not None,
    f'진입 {v_in:.2f} · 등반최저 {v_lo:.2f} · 하강 {v_hi if v_hi else 0:.2f}')

_, v_lo_off, _, _ = run(A(grade_ff_gain=0.0))
chk('grade_ff 를 끄면 12.5% 에서 기어간다', v_lo_off < 0.5,
    f'등반최저 {v_lo_off:.2f} m/s (켰을 때 {v_lo:.2f})')

_, v_lo_w, _, _ = run(A(grade_ff_gain=0.0), k_scale=0.85)
chk('grade_ff 꺼짐 + 구동 15% 약화 = 못 넘는다', v_lo_w < 0.15,
    f'등반최저 {v_lo_w:.2f} m/s')

_, v_lo_ok, _, _ = run(A(), k_scale=0.85)
chk('grade_ff 켜면 15% 약화에도 넘는다', v_lo_ok >= 0.15,
    f'등반최저 {v_lo_ok:.2f} m/s')

# ★ 이것이 RAMP_RUNBOOK ② 를 건너뛰면 안 되는 이유다.
_, v_lo_sgn, _, _ = run(A(imu_sign=-1.0))
chk('IMU 부호가 반대면 오르막에서 선다/밀린다', v_lo_sgn < 0.15,
    f'등반최저 {v_lo_sgn:+.2f} m/s — 보상이 중력을 상쇄하는 게 아니라 더한다')

# ★ 가속 오염 — IMU 는 가속을 경사로 오독한다 (atan(a/g)).
#   최악(AHRS 가 전혀 거부 못 함)에서도 오르막이 깨지지 않아야 한다.
#   깨진다면 grade_ff 를 가속 구간에서 꺼야 한다는 뜻이고, 그건 설계가 바뀐다.
_, v_lo_c, _, _ = run(A(imu_accel_frac=1.0))
chk('가속 오염 최악에도 오르막이 깨지지 않는다', v_lo_c > 0.5,
    f'등반최저 {v_lo_c:.2f} m/s (오염 없을 때 {v_lo:.2f})')

# 오염은 등반을 **느리게** 하지 빠르게 하지 않는다. 방향이 뒤집히면
# 모델이나 부호 처리가 잘못된 것이다.
chk('오염은 등반을 느리게 하는 쪽으로 작용한다', v_lo_c < v_lo,
    f'{v_lo:.2f} → {v_lo_c:.2f} m/s')

# 15% 를 넘어가면 grade_ff_max=70 이 보상을 자른다. 잘리는 지점을 못 박는다.
#   70 / (9.81/0.0202) = sinθ 0.144 = 14.4%
sin_clip = 70.0 * 0.0202 / 9.81
chk('grade_ff_max 70 은 14.4% 까지만 완전보상한다',
    abs(sin_clip * 100 - 14.4) < 0.2,
    f'{sin_clip * 100:.1f}% 초과부터 잘린다 — 더 급하면 grade_ff_max 를 올릴 것')

# 거버너를 끄면 내리막이 빨라져야 한다 (거버너가 일을 하고 있다는 증거)
_, _, _, hi_gov = run(A())
_, _, _, hi_nogov = run(A(gov_pwm=0.0))
chk('거버너를 끄면 내리막이 더 빨라진다',
    hi_gov is not None and hi_nogov is not None and hi_nogov > hi_gov,
    f'거버너 {hi_gov:.2f} → 끄면 {hi_nogov:.2f} m/s')


# ─────────────────────────────────────────────────────────────────────
# ③ 안 켜면 예전과 같은가 (RAMP_RUNBOOK 원칙 1)
# ─────────────────────────────────────────────────────────────────────
print()
print('③ 기능을 안 켜면 예전 동작 그대로인가')

br_off = RP.Bridge(A(grade_ff_gain=0.0))
br_off.clock.t = 100.0
br_off.feed_pitch(math.radians(10.0))          # 경사에 있어도
chk('grade_ff_gain=0 이면 경사 보상이 정확히 0',
    br_off._grade_pwm(100.0) == 0.0)

br_stale = RP.Bridge(A(grade_ff_gain=1.0))
br_stale.clock.t = 0.0
br_stale.feed_pitch(math.radians(10.0))
chk('IMU 가 0.5s 넘게 끊기면 보상이 0 으로 빠진다',
    br_stale._grade_pwm(1.0) == 0.0)

br_gov = RP.Bridge(A(gov_pwm=0.0))
chk('gov_pwm=0 이면 거버너가 절대 개입하지 않는다',
    br_gov._governor_pwm(1.0, 60.0, 0.0) is None)

# 구간 속도를 안 주면 v_max 그대로 — test_ramp_section.py 와 같은 단언이지만,
# 이 도구의 Longitudinal 껍데기가 그 성질을 유지하는지 따로 본다.
lon = RP.Longitudinal(A(v_up=0.0, v_down=0.0))
lon._ramp_up_on = lon._ramp_down_on = False
chk('구간 arm 이 없으면 기준속도 = v_max', lon._base_speed() == lon.v_max,
    f'{lon._base_speed():.2f} m/s')


# ─────────────────────────────────────────────────────────────────────
print()
if skipped:
  print(f'⚠ 건너뛴 로그 {len(skipped)}개: {", ".join(skipped)}')
  print('  → 모델 교차검증이 **안 된 것**이다. 통과로 읽지 말 것.')
if fails:
  print(f'❌ 실패 {len(fails)}건: ' + ', '.join(fails))
  sys.exit(1)
print(f'✅ 전부 통과' + (' (단, 위 건너뜀 주의)' if skipped else ''))
sys.exit(0)

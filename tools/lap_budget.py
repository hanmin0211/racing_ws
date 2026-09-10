#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lap_budget.py — PWM(전원 여력) → 랩타임 → 8분 초과 감점 결정표.

★ 왜 이게 판단 기준인가
  규정상 **8분 초과는 1분당 5점 감점이고 탈락이 아니다**(탈락은 이탈 /
  1분 이상 정지 / DNF / 입력장치뿐). 그런데 8분을 맞추려면 MAX_DRIVE_PWM 을
  물리상한 255 까지 올려야 하고, 그러면 전원 강하로 보드가 리셋될 수 있다.
  **리셋 → 1분 이상 정지 → 탈락**이다.

  즉 "PWM 을 올린다"는 건 '감점 몇 점'과 '탈락 확률'을 맞바꾸는 거래다.
  이 표는 그 거래의 왼쪽(감점)을 정량화한다. 오른쪽(전원 여력)은 멀티미터
  실측으로 정하고, 그 PWM 을 이 표에서 찾아 예상 점수를 읽으면 된다.

★ 모델
  구동은 개루프 FF 다(NO_ENCODER):  PWM = STATIC_FF + VELOCITY_FF_GAIN·v
  펌웨어 실측값 80 / 95 → v_max = (PWM − 80) / 95
  랩타임은 tracking_sim 의 폐루프 시뮬로 낸다. 2026-09-09 HIL 실측
  (648.3m, v_max 0.5, 1670초)과 시뮬이 1670초로 일치해 검증된 모델이다.

사용:
  python3 tools/lap_budget.py --waypoints config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml
  python3 tools/lap_budget.py --waypoints <파일> --mission-s 121 --gains 6 3
  python3 tools/lap_budget.py --waypoints <파일> --pwms 160 200 244 255
"""

import argparse
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tracking_sim as ts  # noqa: E402

# 펌웨어 henes_firmware.ino 의 실측 FF 상수. 펌웨어를 바꾸면 여기도 바꿀 것.
STATIC_FF = 80.0
VELOCITY_FF_GAIN = 95.0
PWM_HARD_MAX = 255            # analogWrite 물리 상한

# 실제 런치/파라미터 기본값과 맞춘다. 여기가 어긋나면 표 전체가 거짓말이 된다.
#   pure_pursuit_params.yaml : k_ld 0.8, min/max_lookahead 1.6/4.0,
#                              max_steer_rate_deg 45, max_steering_deg 18
#   longitudinal_controller  : v_min 0.25, max_accel 1.0, max_decel 1.8
REAL = dict(min_speed=0.25, k_ld=0.8, min_ld=1.6, max_ld=4.0, wheelbase=0.785,
            max_steer_deg=18.0, max_steer_rate_deg=45.0, max_accel=1.0,
            max_decel=1.8, n_back=5, n_forward=20, rate=20.0, max_time=3600.0,
            init_offset=0.0, steer_bias=0.0, steer_scale=1.0,
            init_heading_err=0.0, plot=None)


def pwm_to_v(pwm):
  return max(0.0, (pwm - STATIC_FF) / VELOCITY_FF_GAIN)


def v_to_pwm(v):
  return STATIC_FF + VELOCITY_FF_GAIN * v


def _run(job):
  pwm, gain, wp_file, extra = job
  a = argparse.Namespace(**{**REAL, **extra,
                            'speed': pwm_to_v(pwm), 'curv_gain': gain})
  wps = ts.load_waypoints(wp_file)
  r = ts.simulate(wps, a)
  cte = r['cte']
  if len(cte) == 0:
    return pwm, gain, None
  ss = cte[r['s'] > 5.0]
  sat = np.abs(r['delta']) >= a.max_steer_deg - 0.1
  return pwm, gain, dict(
      goal=r['goal'], reason=r['reason'], drive_s=len(cte) / a.rate,
      dist=r['travelled'], cte_max=float(ss.max()),
      sat=float(sat.mean() * 100), dmax=float(np.abs(r['delta']).max()))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--waypoints', required=True)
  ap.add_argument('--pwms', type=int, nargs='*',
                  default=[160, 180, 200, 213, 230, 244, 255])
  ap.add_argument('--gains', type=float, nargs='*', default=[6.0, 3.0])
  # config/mission_plan.yaml 의 expected_s 합. 기본값은 2026-09-09 시점 추정
  # (crosswalk 12 + 후진주차 45 + 급정지 12 + 경사로 12 + 신호교차로 20 +
  #  S코스 20 = 121초). 직각·평행주차는 아직 추정치가 없다.
  ap.add_argument('--mission-s', type=float, default=121.0)
  ap.add_argument('--budget-s', type=float, default=480.0)
  ap.add_argument('--penalty-per-min', type=float, default=5.0)
  ap.add_argument('--max-steer-deg', type=float, default=None,
                  help='기본은 실제 설정값 18. 좁혀서 볼 때만 지정')
  args = ap.parse_args()

  extra = {}
  if args.max_steer_deg is not None:
    extra['max_steer_deg'] = args.max_steer_deg

  jobs = [(p, g, args.waypoints, extra)
          for g in args.gains for p in args.pwms]
  with Pool(min(len(jobs), os.cpu_count() or 1)) as pool:
    out = pool.map(_run, jobs)

  print('=' * 82)
  print(f'8분 예산 결정표 — {os.path.basename(args.waypoints)}')
  print(f'  미션 소요 가정 {args.mission_s:.0f}s · 제한 {args.budget_s:.0f}s · '
        f'초과 1분당 {args.penalty_per_min:.0f}점 (탈락 아님)')
  print('=' * 82)
  hdr = (f"{'PWM':>4} {'v_max':>6} {'주행':>7} {'+미션':>7} {'총':>7} "
         f"{'초과':>7} {'감점':>5} {'cte최대':>8} {'포화':>6} {'완주':>4}")
  print(hdr)
  print('-' * 82)
  last_gain = None
  for pwm, gain, r in out:
    if gain != last_gain:
      if last_gain is not None:
        print()
      print(f'[ curvature_gain = {gain:.1f} ]')
      last_gain = gain
    if r is None:
      print(f'{pwm:4d}   샘플 없음')
      continue
    total = r['drive_s'] + args.mission_s
    over = max(0.0, total - args.budget_s)
    pen = math.ceil(over / 60.0) * args.penalty_per_min
    mark = '' if pen == 0 else ('  ⚠' if pen <= 15 else '  ❌')
    flag = '★' if pwm > PWM_HARD_MAX else ' '
    print(f'{pwm:4d}{flag}{pwm_to_v(pwm):6.2f} {r["drive_s"]:6.0f}s '
          f'{args.mission_s:6.0f}s {total:6.0f}s {over:6.0f}s '
          f'{pen:4.0f}점 {r["cte_max"]:7.3f}m {r["sat"]:5.1f}% '
          f"{'O' if r['goal'] else 'X':>4}{mark}")
  print('-' * 82)
  print('읽는 법: 전원계가 견디는 PWM 을 멀티미터로 정한 뒤 그 행을 본다.')
  print('         감점 몇 점을 아끼려고 PWM 을 올리는 것이, 브라운아웃 리셋으로')
  print('         1분 이상 서서 **탈락**할 위험보다 큰지 비교할 것.')
  print(f'         현재 펌웨어 MAX_DRIVE_PWM = 160 → v_max {pwm_to_v(160):.2f} m/s')
  print('=' * 82)


if __name__ == '__main__':
  main()

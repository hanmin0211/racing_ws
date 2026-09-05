#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_parking_plan.py — 계획기(parking_planner) + 추종기(parking_follower) 를
자전거모델과 붙여 **폐루프**로 돌린다. ROS 를 안 띄우므로 시작자세 수백 개를
몇 초 만에 훑을 수 있다.

  python3 tools/test_parking_plan.py                 # 3자리, 완주 종점에서
  python3 tools/test_parking_plan.py --sweep         # 자세 오차 격자 전체
  python3 tools/test_parking_plan.py --slot 1 --plot

★ 왜 이 시험이 필요한가
  실차 완주 종점 자세는 매 주행마다 다르다(8/24 는 yaw 142.9°, 8/25 는 166.8°).
  한 자세에서 되는 것만 확인하면 다음 주행에서 또 ABORT 한다. 그래서 위치·헤딩
  오차 격자를 통째로 돌려 **몇 %가 성공하는지**를 본다.
"""

import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'mission_perception',
                                'mission_perception'))

import parking_follower as pf     # noqa: E402
import parking_planner as pp      # noqa: E402

WS = os.path.join(HERE, '..')
TRACK = os.path.join(WS, 'src', 'pure_pursuit_pkg', 'config',
                     'waypoints_recorded_resampled_0.5.yaml')
LAP_END = (-25.605, 103.797, 166.9)      # 완주 경로 종점 (측정값)


def load_track():
  with open(TRACK, encoding='utf-8') as f:
    d = yaml.safe_load(f)
  seq = d['waypoints'] if isinstance(d, dict) and 'waypoints' in d else d
  return [(float(p['x']), float(p['y'])) for p in seq]


def load_goal(slot):
  p = os.path.expanduser(f'~/parking_pose_{slot}.yaml')
  with open(p, encoding='utf-8') as f:
    d = yaml.safe_load(f)
  return ((float(d['pose']['x']), float(d['pose']['y'])),
          math.radians(float(d['pose']['yaw_deg'])))


def simulate(plan, start, *, dt=0.05, t_max=140.0, steer_lag=0.12,
             steer_bias_deg=0.0, noise=0.0, seed=0, trace=False):
  """자전거모델 폐루프. 반환 dict(ok, err, yaw_err, state, t, trail).

  steer_lag: 조향 1차지연[s] (실차 서보는 즉시 안 따라온다)
  steer_bias_deg: 조향 영점 오차 (실차 앞바퀴 틀어짐 재현)
  """
  import random
  rnd = random.Random(seed)
  segs = pf.split_segments(plan['points'])
  fol = pf.ParkingFollower(segs, rate=1.0 / dt)
  x, y, yaw = start
  v, steer = 0.0, 0.0
  t = 0.0
  trail = []
  while t < t_max:
    cmd_v, cmd_d = fol.update((x, y, yaw), v, t)
    if fol.state in ('DONE', 'ABORT'):
      break
    d_target = math.radians(max(-18.0, min(18.0, cmd_d + steer_bias_deg)))
    a = dt / max(dt, steer_lag)
    steer += (d_target - steer) * a
    v += max(-0.6 * dt, min(0.6 * dt, cmd_v - v))    # 가감속 한계
    if noise > 0.0:
      x += rnd.gauss(0.0, noise) * dt
      y += rnd.gauss(0.0, noise) * dt
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += (v / 0.785) * math.tan(steer) * dt
    t += dt
    if trace:
      trail.append((x, y, yaw, v, cmd_d, fol.state, fol.seg_i))
  gx, gy, gth = plan['goal']
  return {
      'state': fol.state, 'reason': fol.reason, 't': t,
      'err': math.hypot(x - gx, y - gy),
      'yaw_err': abs(math.degrees(pf.yaw_wrap(yaw - gth))),
      'pose': (x, y, yaw), 'trail': trail,
      'cusps': sum(1 for e, _ in fol.events if e == 'CUSP'),
  }


def run_one(slot, start_deg, track, *, verbose=True, **simkw):
  goal, th_g = load_goal(slot)
  start = (start_deg[0], start_deg[1], math.radians(start_deg[2]))
  plan = pp.plan_parking(start, goal, th_g, corridor=track)
  if plan is None:
    if verbose:
      print(f'  slot{slot}: ❌ 계획 실패')
    return None, None
  res = simulate(plan, start, **simkw)
  if verbose:
    n_f = sum(1 for p in plan['points'] if p[2] > 0)
    n_r = len(plan['points']) - n_f
    print(f'  slot{slot}: {res["state"]:5s} 오차 {res["err"]*100:5.1f}cm  '
          f'yaw {res["yaw_err"]:4.1f}°  {res["t"]:5.1f}s  '
          f'| 뒤로 {plan["d_back"]:.2f} 전진 {plan["fwd_len"]:.2f} '
          f'후진 {plan["rev_len"]:.2f}  ({n_f}+{n_r}점) {res["reason"]}')
  return plan, res


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--slot', type=int, default=None)
  ap.add_argument('--sweep', action='store_true')
  ap.add_argument('--plot', action='store_true')
  ap.add_argument('--start-pose', default=None, help='"x,y,yaw_deg"')
  ap.add_argument('--tol', type=float, default=0.30, help='성공 판정 오차[m]')
  args = ap.parse_args()

  track = load_track()
  slots = [args.slot] if args.slot else [1, 2, 3]
  base = LAP_END
  if args.start_pose:
    base = tuple(float(v) for v in args.start_pose.split(','))

  print('=' * 66)
  print(f'출발 자세 ({base[0]:.2f}, {base[1]:.2f}) yaw {base[2]:+.1f}°  '
        f'— 완주 경로 종점')
  print('=' * 66)
  plans = {}
  ok_all = True
  for s in slots:
    plan, res = run_one(s, base, track)
    plans[s] = (plan, res)
    ok_all = ok_all and res is not None and res['state'] == 'DONE' \
        and res['err'] <= args.tol

  if args.plot:
    plot(plans, track, base)

  if args.sweep:
    print('\n' + '=' * 66)
    print('자세 오차 격자 — 실차 완주 종점은 매번 다르다')
    print('=' * 66)
    dxs = (-1.0, -0.5, 0.0, 0.5, 1.0)
    dys = (-0.6, -0.3, 0.0, 0.3, 0.6)
    dths = (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0)
    grand = 0
    grand_ok = 0
    for s in slots:
      n = ok = plan_fail = 0
      worst = 0.0
      fails = []
      for dx in dxs:
        for dy in dys:
          for dth in dths:
            st = (base[0] + dx, base[1] + dy, base[2] + dth)
            _p, r = run_one(s, st, track, verbose=False)
            n += 1
            if r is None:
              plan_fail += 1
              fails.append((dx, dy, dth, '계획실패'))
              continue
            if r['state'] == 'DONE' and r['err'] <= args.tol:
              ok += 1
              worst = max(worst, r['err'])
            else:
              fails.append((dx, dy, dth,
                            f'{r["state"]} {r["err"]*100:.0f}cm '
                            f'{r["reason"][:30]}'))
      grand += n
      grand_ok += ok
      print(f'  slot{s}: {ok}/{n} 성공 ({100.0*ok/n:.1f}%)  '
            f'최악 오차 {worst*100:.0f}cm  계획실패 {plan_fail}')
      for f in fails[:6]:
        print(f'      ✗ dx{f[0]:+.1f} dy{f[1]:+.1f} dθ{f[2]:+.0f}° → {f[3]}')
      if len(fails) > 6:
        print(f'      … 외 {len(fails)-6}건')
    print(f'\n  합계 {grand_ok}/{grand} ({100.0*grand_ok/grand:.1f}%)')
    ok_all = ok_all and grand_ok == grand

  print('\n' + '=' * 66)
  print('결론: ✅ 통과' if ok_all else '결론: ❌ 실패 — 위 항목 확인')
  return 0 if ok_all else 1


def plot(plans, track, base):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig, axes = plt.subplots(1, len(plans), figsize=(6 * len(plans), 6))
  if len(plans) == 1:
    axes = [axes]
  for ax, (s, (plan, res)) in zip(axes, sorted(plans.items())):
    ax.plot([p[0] for p in track], [p[1] for p in track], '-',
            lw=1.0, color='#cbd5e0')
    if plan:
      f = [p for p in plan['points'] if p[2] > 0]
      r = [p for p in plan['points'] if p[2] < 0]
      ax.plot([p[0] for p in f], [p[1] for p in f], '.', ms=3,
              color='#2b6cb0', label='전진')
      ax.plot([p[0] for p in r], [p[1] for p in r], '.', ms=3,
              color='#d69e2e', label='후진')
      gx, gy, gth = plan['goal']
      ax.plot(gx, gy, 's', ms=10, color='#e53e3e')
      ax.arrow(gx, gy, 1.0 * math.cos(gth), 1.0 * math.sin(gth),
               head_width=0.2, color='#e53e3e')
    if res and res['trail']:
      ax.plot([p[0] for p in res['trail']], [p[1] for p in res['trail']],
              '-', lw=1.6, color='#38a169', label='sim')
    ax.arrow(base[0], base[1], 1.0 * math.cos(math.radians(base[2])),
             1.0 * math.sin(math.radians(base[2])), head_width=0.2,
             color='#805ad5')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_title(f'slot {s}  {res["state"] if res else "계획실패"} '
                 f'{res["err"]*100:.0f}cm' if res else f'slot {s}')
    ax.set_xlim(base[0] - 6, base[0] + 6)
    ax.set_ylim(base[1] - 4, base[1] + 5)
  out = os.path.expanduser('~/parking_plan_check.png')
  plt.tight_layout()
  plt.savefig(out, dpi=110)
  print(f'\n  그림: {out}')


if __name__ == '__main__':
  sys.exit(main())

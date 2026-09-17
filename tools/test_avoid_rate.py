#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_avoid_rate.py — 회피 조향 슬루레이트 제한을 실측 런으로 검증한다.

★ 왜 이 제한이 생겼나 (drive_0337, 2026-09-16 03:37 학교트랙 완주 런)

  경로조향은 local_pure_pursuit 가 max_steer_rate_deg(90°/s)로 이미 제한한다.
  그런데 회피조향은 그 경로를 **통째로 우회한다**:
      cluster_plot_node → /lidar/avoid_steer → 먹스 → /cmd_vel
  즉 회피각에는 아무 제한이 없었다. 실측 결과:

      arm 구간 9.9초 · 좌/우 전환 6회 중 4회가 0.7초 안에 몰림
      명령 조향 프레임간 변화 최대 30.2° = 604°/s  (실제 조향 한계 ~25°/s)
      한 방향 체류 8회 중 4회(50%)가 조향 지연 0.35s 보다 짧다

  그 0.7초 동안 명령은 +17.4 → −10.9 → +16.8 → −13.4 → −15.8 로 흔들렸고
  **실제 조향각은 −2.9~+2.7 에 머물렀다.** 차가 무사했던 건 액추에이터가
  못 따라가서지 설계 덕이 아니다.

★ 이 제한이 하는 일과 하지 않는 일
  하는 일   : 명령이 액추에이터가 물리적으로 낼 수 없는 변화를 요구하지 못하게
  하지 않는 일: 플래너의 판단을 바꾸지 않는다. 진짜로 방향을 바꾸고 그 판단을
              유지하면 제한 안에서 그대로 통과한다.
  ⚠ 근본 원인은 _find_largest_gap 이 **무상태**라 매 프레임 갭을 처음부터
    다시 고르는 것이다. 이건 증상 차단이지 원인 수정이 아니다.

  python3 tools/test_avoid_rate.py
"""

import csv
import math
import os
import sys

sys.path.insert(0, '/home/han/racing_ws/src/lidar_clustering')
from lidar_clustering.cluster_plot_node import ClusterPlotNode  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drive_fixtures import require_run  # noqa: E402

LIMIT = ClusterPlotNode._rate_limit_steer      # 실제 구현을 그대로 쓴다
NAN = float('nan')
# ★ 2026-09-17 — 예전엔 '/tmp/drive_0337.csv' 를 직접 가리켰고 **없으면 조용히
#   건너뛰고 ✅ 를 냈다**. /tmp 는 재부팅하면 사라진다(= 대회 당일 아침).
#   이제 저장소 사본을 먼저 찾고, 없으면 소리내어 죽는다.
RUN, _ = require_run('0337')


class Stub:
  """_rate_limit_steer 가 참조하는 속성만 갖춘 최소 객체."""

  def __init__(self, rate=0.0, reset_s=1.0):
    self.avoid_steer_rate = float(rate)
    self.avoid_steer_rate_reset_s = float(reset_s)
    self._rl_prev = None
    self._rl_prev_t = None

  def step(self, steer, t):
    return LIMIT(self, steer, t)


def load_run():
  rows = list(csv.DictReader(open(RUN)))
  out = []
  for r in rows:
    raw = r['avoid_steer_deg']
    a = float(raw) if raw not in ('', 'nan', 'None') else NAN
    out.append((float(r['t']), a, '조향OFF' not in r['mode']))
  return out


def replay(run, rate):
  st = Stub(rate=rate)
  return [(t, st.step(a, t), armed) for t, a, armed in run]


def max_rate(series):
  """연속한 유효 명령 사이의 최대 변화율 [도/초]."""
  worst, prev = 0.0, None
  for t, a, _ in series:
    if a is None or math.isnan(a):
      continue
    if prev is not None:
      dt = max(t - prev[0], 1e-3)
      worst = max(worst, abs(a - prev[1]) / dt)
    prev = (t, a)
  return worst


def flips(series, lo=74.0, hi=85.0):
  """부호가 뒤집힌 횟수 (deadband 3° 밖에서만 센다)."""
  n, last = 0, 0
  for t, a, _ in series:
    if a is None or math.isnan(a) or not (lo <= t <= hi) or abs(a) < 3.0:
      continue
    s = 1 if a > 0 else -1
    if last and s != last:
      n += 1
    last = s
  return n


def main():
  fails = []

  def check(name, ok, detail=''):
    print(f'  {"OK " if ok else "✗  "} {name}')
    if not ok:
      if detail:
        print(f'        {detail}')
      fails.append(name)

  # ------------------------------------------------------------------ 단위
  print('\n[단위] 제한기 자체')

  s = Stub(rate=0.0)
  check('rate=0 이면 손대지 않는다 (기본값 = 예전 동작)',
        s.step(30.0, 1.0) == 30.0 and s.step(-30.0, 1.05) == -30.0)

  s = Stub(rate=90.0)
  check('첫 값은 그대로 통과한다', s.step(10.0, 1.0) == 10.0)
  got = s.step(-20.0, 1.05)          # 0.05s 에 90°/s → 4.5° 만 허용
  check('30° 점프가 4.5° 로 잘린다', abs(got - 5.5) < 1e-6, f'받은 값 {got}')

  s = Stub(rate=90.0)
  s.step(10.0, 1.0)
  got = s.step(12.0, 1.05)
  check('제한 안의 변화는 그대로 통과한다', abs(got - 12.0) < 1e-6, f'{got}')

  # NaN 을 사이에 둬도 상태를 버리지 않는다 (이게 핵심이다)
  s = Stub(rate=90.0, reset_s=1.0)
  s.step(16.8, 1.00)
  s.step(NAN, 1.05)
  s.step(NAN, 1.10)
  got = s.step(-15.8, 1.15)
  check('BLOCKED 를 사이에 둔 좌→우 건너뜀도 잘린다 (drive_0337 t78.15 장면)',
        got > 0.0, f'받은 값 {got:.1f}° — 아직 좌에서 내려오는 중이어야 한다')
  check('끊긴 시간만큼 허용량이 커진다 (0.15s → 13.5°)',
        abs(got - (16.8 - 13.5)) < 1e-6, f'{got}')

  s = Stub(rate=90.0, reset_s=1.0)
  s.step(16.8, 1.0)
  for k in range(1, 30):
    s.step(NAN, 1.0 + 0.05 * k)      # 1.45s 끊김 > reset_s
  got = s.step(-15.8, 2.5)
  check('오래 끊기면 상태를 버리고 새 기동으로 본다',
        abs(got + 15.8) < 1e-6, f'{got}')

  # ------------------------------------------------------- 실측 리플레이
  print('\n[실측 리플레이] drive_0337 회피 구간')
  run = load_run()
  if run is None:
    print(f'  ·   {RUN} 없음 — 리플레이 건너뜀')
  else:
    base = replay(run, 0.0)
    r0 = max_rate(base)
    f0 = flips(base)
    print(f'  ·   제한 없음: 최대 {r0:.0f}°/s · 부호전환 {f0}회')
    check('실측 런에 고쳐야 할 대상이 실제로 있다 (>200°/s)',
          r0 > 200.0, f'{r0:.0f}°/s')

    rows = []
    for rate in (180.0, 120.0, 90.0, 60.0):
      ser = replay(run, rate)
      rows.append((rate, max_rate(ser), flips(ser)))
    print(f'\n  {"rate":>7} {"실측 최대":>10} {"부호전환":>9}')
    for rate, mr, fl in rows:
      print(f'  {rate:7.0f} {mr:9.0f}°/s {fl:8d}회')

    for rate, mr, fl in rows:
      check(f'rate={rate:.0f} 이면 명령이 그 이상을 요구하지 않는다',
            mr <= rate * 1.05 + 1.0, f'실측 {mr:.0f}°/s')

    # 진짜 기동은 살아남아야 한다 — t78.25~79.2 의 −18° 우회전
    ser = replay(run, 90.0)
    seg = [a for t, a, _ in ser if 78.2 <= t <= 79.4
           and a is not None and not math.isnan(a)]
    peak = min(seg) if seg else 0.0
    check('제한(90°/s)을 걸어도 −18° 우회 기동은 끝까지 간다',
          peak <= -15.0, f'도달 최저 {peak:.1f}°')

    # 도달까지 얼마나 늦어지나 — 숫자를 눈에 보이게 남긴다
    def reach_t(series, thr=-15.0):
      for t, a, _ in series:
        if t >= 78.2 and a is not None and not math.isnan(a) and a <= thr:
          return t
      return None
    tb, tr = reach_t(base), reach_t(ser)
    if tb and tr:
      print(f'  ·   −15° 도달: 제한없음 t{tb:.2f} → 90°/s t{tr:.2f} '
            f'({(tr - tb) * 1000:.0f}ms 지연)')
      check('지연이 조향 지연(0.35s) 보다 작다', (tr - tb) < 0.35,
            f'{(tr - tb):.2f}s')

    # DISARM 구간을 오염시키지 않는가
    leak = [t for t, a, armed in ser
            if not armed and a is not None and not math.isnan(a)]
    check('DISARM 구간에 회피각이 새지 않는다', not leak, f'{leak[:5]}')

  print()
  if fails:
    print(f'❌ 실패 {len(fails)}건: {", ".join(fails)}')
    return 1
  print('✅ 전부 통과 — 슬루레이트 제한이 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

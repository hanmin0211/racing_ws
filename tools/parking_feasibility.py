#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parking_feasibility.py — 주어진 주차칸에 **기하학적으로 들어갈 수 있는가**.

★ 왜 필요한가
  규정(항목 8): 평행주차 공간은 **제공된 차량규격의 1.5배** 또는 상황에 따라
  조정된다. 경계는 라바콘이고 당일 가변이다. 그런데 우리 차는 최대타각 18°,
  최소회전반경 2.42m 로 **차 길이(1.40m) 대비 회전반경이 매우 크다.**
  그래서 "규정 공간에 물리적으로 들어갈 수 있는가" 부터 확인해야 한다.
  제어를 아무리 잘해도 기하가 안 되면 못 한다.

★ 두 가지를 나눠서 본다
  1) **이상적 기하** — 최소반경 2연속 원호(S자)로 들어가는 이론상 최소 칸 길이.
     제어기와 무관한 물리적 하한이다.
  2) **현재 플래너** — mission_perception.parking_planner.plan_parking 이
     실제로 해를 내는가. 이 플래너는 '후진 원호 1개 + 직선' 구조라
     평행주차에 필요한 S자를 못 만들 수 있다.

  차체는 점이 아니라 **직사각형**으로 쓸어 검사한다(플래너는 뒷축 중심의
  점 경로만 만든다 — 차체 충돌은 보지 않는다).

차량 제원 기본값: HENES T870 — 전장 1.40m · 전폭 0.775m · 축거 0.785m
  (HANDOFF.md: 축거 0.785m, 전폭 775 / 전장 1400mm)

사용:
  python3 tools/parking_feasibility.py                      # 평행주차 2.10m
  python3 tools/parking_feasibility.py --slot-len 2.5
  python3 tools/parking_feasibility.py --perp               # 직각주차
  python3 tools/parking_feasibility.py --sweep              # 칸 길이 쓸어보기
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'mission_perception'))


def body_corners(x, y, th, l_r, l_f_total, half_w):
  """뒷축 중심 (x,y,th) → 차체 네 모서리. l_f_total = 축거+앞오버행."""
  c, s = math.cos(th), math.sin(th)
  pts = []
  for dx, dy in ((-l_r, +half_w), (l_f_total, +half_w),
                 (l_f_total, -half_w), (-l_r, -half_w)):
    pts.append((x + dx * c - dy * s, y + dx * s + dy * c))
  return pts


def seg_rect_clear(corners, rects):
  """차체 사각형이 rects(축정렬 금지구역)과 겹치지 않는가 → 최소 여유[m].

  겹치면 음수. 근사: 차체 모서리 4점 + 변 중점 4점을 검사점으로 쓴다.
  """
  pts = list(corners)
  for i in range(4):
    a, b = corners[i], corners[(i + 1) % 4]
    pts.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
  worst = float('inf')
  for (x0, y0, x1, y1) in rects:
    for (px, py) in pts:
      dx = max(x0 - px, 0.0, px - x1)
      dy = max(y0 - py, 0.0, py - y1)
      d = math.hypot(dx, dy)
      if dx == 0.0 and dy == 0.0:          # 내부 = 침범
        d = -min(px - x0, x1 - px, py - y0, y1 - py)
      worst = min(worst, d)
  return worst


def ideal_parallel(slot_len, slot_depth, veh, R, ds=0.01):
  """최소반경 S자 2원호로 평행주차 — 들어갈 수 있으면 (True, 최소여유, 궤적).

  좌표: 차선은 +x 방향, 칸은 −y 쪽.
    칸      x ∈ [0, slot_len],  y ∈ [−slot_depth, 0]
    앞 장애물 x < 0,  뒤 장애물 x > slot_len   (라바콘/이웃차)
    연석    y < −slot_depth
  최종 자세: 칸 가운데, 헤딩 +x.
  """
  l_r, l_f_total, half_w = veh
  L_v = l_r + l_f_total
  # 목표: 차체를 칸 가운데. 뒷축 중심 x
  gx = slot_len / 2.0 - (L_v / 2.0 - l_r)
  gy = -slot_depth / 2.0
  # 금지구역 (여유를 위해 칸 밖을 크게 잡는다)
  big = 20.0
  rects = [
      (-big, -slot_depth, 0.0, 0.0),              # 앞 장애물
      (slot_len, -slot_depth, big, 0.0),          # 뒤 장애물
      (-big, -big, big, -slot_depth),             # 연석/뒷벽
  ]
  best = None
  # θ: 각 원호가 도는 각. 대칭 S자.
  for th_deg in np.arange(5.0, 80.0, 0.5):
    th = math.radians(th_deg)
    # 후진 S자: goal 에서 거꾸로 전진으로 빠져나오는 궤적을 만든 뒤 뒤집는다.
    pts = []
    x, y, h = gx, gy, 0.0
    ok = True
    worst = float('inf')
    # 1구간: goal 에서 전진하며 좌로 th 만큼 (칸에서 나오는 방향)
    for phase, turn in ((1, +1.0), (2, -1.0)):
      arc = R * th
      n = max(2, int(arc / ds))
      for _ in range(n):
        x += ds * math.cos(h)
        y += ds * math.sin(h)
        h += turn * ds / R
        cor = body_corners(x, y, h, l_r, l_f_total, half_w)
        cl = seg_rect_clear(cor, rects)
        worst = min(worst, cl)
        pts.append((x, y, h))
        if cl < 0:
          ok = False
          break
      if not ok:
        break
    if not ok:
      continue
    # ★ 빠져나온 뒤 **차선 안**에 있어야 한다.
    #   이걸 강제하지 않으면 원호 5°(횡이동 2cm)짜리 '해' 가 나온다 —
    #   그건 평행주차가 아니라 칸 열린 쪽으로 그냥 직진한 것이다.
    #   차선은 칸(y<0) 바깥, 즉 차체 전체가 y >= 0 이어야 한다.
    if abs(wrapdeg(h)) > 3.0:
      continue
    cor = body_corners(x, y, h, l_r, l_f_total, half_w)
    if min(c[1] for c in cor) < 0.0:
      continue        # 아직 칸 안에 걸쳐 있다 = 차선으로 못 나왔다
    if best is None or worst > best[0]:
      best = (worst, th_deg, pts)
  if best is None:
    return False, -1.0, None, None
  return True, best[0], best[1], best[2]


def shuffle_estimate(slot_len, slot_depth, veh, R, v=0.25, cusp_dwell=1.0):
  """끊어 넣기(다중 기동) 소요 추정 — 해석식.

  단일 S자는 종방향 2.9m 를 쓰므로 규정 공간(2.10m)에 못 들어간다. 대신 사람이
  좁은 자리에 넣듯 **짧게 여러 번** 왕복하면 한 사이클마다 조금씩 횡으로 민다.
  종방향 여유 d 로 풀타 왕복 한 번의 횡이득은 근사적으로

      Δy ≈ d² / R

  (반경 R 원호를 d 만큼 따라갔을 때의 새그. 전진·후진 반대타라 회전은 상쇄되고
   횡변위만 남는다.) d 가 제곱으로 들어가므로 칸이 조금만 길어져도 횟수가 급감한다.

  ⚠ 이건 **소요 추정**이지 가능 판정이 아니다. 진짜 판정은 초기 진입(앞차 옆을
    지나 비스듬히 들어가는 구간)이 되는지에 달려 있고, 그건 다중 cusp 를 만드는
    플래너(Reeds-Shepp + 차체 충돌검사)로만 답할 수 있다.
    ※ 2026-09-12 에 탐욕적 왕복 시뮬을 붙여 봤으나 단일 S자로 되는 4.0m 조차
      못 풀었다(초기 진입을 모델링하지 못함). 그 코드는 믿을 수 없어 걷어냈다.
  """
  l_r, l_f_total, half_w = veh
  L_v = l_r + l_f_total
  d = slot_len - L_v
  if d <= 0:
    return None
  gain = d * d / R
  need = half_w + 0.05 + slot_depth / 2.0
  n = need / gain
  cyc_t = 2.0 * d / v + 2.0 * cusp_dwell
  return dict(play=d, gain=gain, need=need, cycles=n, cycle_s=cyc_t,
              total_s=n * cyc_t)


def wrapdeg(h):
  return math.degrees(math.atan2(math.sin(h), math.cos(h)))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--veh-len', type=float, default=1.40)
  ap.add_argument('--veh-width', type=float, default=0.775)
  ap.add_argument('--wheelbase', type=float, default=0.785)
  ap.add_argument('--rear-overhang', type=float, default=None,
                  help='기본은 앞뒤 대칭으로 나눈다')
  ap.add_argument('--max-steer-deg', type=float, default=18.0)
  ap.add_argument('--slot-len', type=float, default=None,
                  help='기본: 평행주차 = 전장 1.5배')
  ap.add_argument('--slot-depth', type=float, default=None,
                  help='기본: 전폭 + 0.4m')
  ap.add_argument('--sweep', action='store_true')
  a = ap.parse_args()

  over = a.veh_len - a.wheelbase
  l_r = a.rear_overhang if a.rear_overhang is not None else over / 2.0
  l_f_total = a.veh_len - l_r
  veh = (l_r, l_f_total, a.veh_width / 2.0)
  R = a.wheelbase / math.tan(math.radians(a.max_steer_deg))
  slot_len = a.slot_len if a.slot_len else a.veh_len * 1.5
  slot_depth = a.slot_depth if a.slot_depth else a.veh_width + 0.40

  print('=' * 70)
  print('평행주차 기하 검토')
  print('=' * 70)
  print(f'  차량      전장 {a.veh_len:.2f}m · 전폭 {a.veh_width:.3f}m · '
        f'축거 {a.wheelbase:.3f}m')
  print(f'            오버행 앞 {l_f_total - a.wheelbase:.3f}m / '
        f'뒤 {l_r:.3f}m (대칭 가정)')
  print(f'  최대타각  {a.max_steer_deg:.0f}° → **최소회전반경 {R:.2f}m** '
        f'(전장의 {R / a.veh_len:.1f}배)')
  print(f'  주차칸    길이 {slot_len:.2f}m · 깊이 {slot_depth:.2f}m')
  print(f'            (규정: 차량규격의 1.5배 = {a.veh_len * 1.5:.2f}m)')
  print('-' * 70)

  if a.sweep:
    print(f"{'칸 길이':>8} {'전장 대비':>9} {'가능':>5} {'최소여유':>9} {'원호각':>7}")
    print('-' * 46)
    for sl in np.arange(a.veh_len * 1.2, a.veh_len * 3.2, 0.1):
      ok, cl, th, _ = ideal_parallel(sl, slot_depth, veh, R)
      print(f'{sl:7.2f}m {sl / a.veh_len:8.2f}배 {"O" if ok else "X":>5} '
            + (f'{cl:8.3f}m {th:6.1f}°' if ok else f'{"—":>8}  {"—":>6}'))
    return 0

  ok, cl, th, _ = ideal_parallel(slot_len, slot_depth, veh, R)
  if ok:
    print(f'  ✅ 이상적 S자 기동으로 들어간다 — 최소여유 {cl:.3f}m, '
          f'원호각 {th:.1f}°')
  else:
    print('  ❌ **최소반경 S자로도 못 들어간다** — 기하학적으로 불가능하다.')
    print('     제어를 아무리 잘해도 안 된다. 여러 번 끊어 넣거나(다중 기동)')
    print('     타각을 늘려야 한다.')
  print('-' * 70)
  print('  ※ 이건 이상적 2원호 기동이다. 실제 플래너(parking_planner)는')
  print('    후진 원호 1개 + 직선 구조라 S자를 못 만들 수 있다 — 따로 볼 것.')
  print('=' * 70)
  return 0


if __name__ == '__main__':
  sys.exit(main())

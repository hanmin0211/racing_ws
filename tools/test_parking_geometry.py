#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_parking_geometry.py — 후진 pure pursuit 기하를 자전거모델로 검증한다.

하드웨어 없이 **로직만** 먼저 가른다. 실차에서 후진 조향이 반대로 걸리면 좁은
주차공간에서 바로 사고이므로, 부호를 코드에 넣기 전에 여기서 확인한다.

  python3 tools/test_parking_geometry.py

검증하는 것:
  1. 후진 조향 부호 — δ>0(좌타)로 후진하면 차가 후방-좌측으로 가는가
  2. parking_node 의 추종 로직이 전진접근→후진진입 궤적을 따라가는가
  3. 조향 부호를 뒤집으면(steer_sign_reverse=-1) 실제로 실패하는가
     (실패해야 한다. 실패하지 않으면 이 테스트가 부호를 못 가르는 것이다)

⚠ 이건 기하·제어 로직 검증이다. 실차의 조향 극성(펌웨어 ADC 방향)은
   PLAN PHASE 0-2 벤치 측정으로 따로 확인해야 한다.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'mission_perception'))

# ⚠ 2026-09-13 — 이 임포트가 parking_node 를 가리키고 있어서 **테스트가
#   임포트 단계에서 죽어 있었다.** 두 함수는 parking_follower 로 옮겨졌는데
#   테스트가 안 따라갔다. 죽은 테스트는 통과도 실패도 아니라 '검증 없음' 이다.
#   (test_parking_plan.py 는 이미 parking_follower 를 보고 있었다)
from mission_perception.parking_follower import (  # noqa: E402
    pursuit_steer, split_segments)

L = 0.785
MAX_STEER = math.radians(18.0)


def step(x, y, yaw, v, delta, dt):
  """자전거모델(rear-axle) 1스텝. v<0 이면 후진."""
  x += v * math.cos(yaw) * dt
  y += v * math.sin(yaw) * dt
  yaw += (v / L) * math.tan(delta) * dt
  return x, y, yaw


def test_reverse_sign():
  """δ>0 로 후진하면 후방-좌측(y 증가)으로 가는가."""
  x, y, yaw = 0.0, 0.0, 0.0
  for _ in range(200):
    x, y, yaw = step(x, y, yaw, -0.3, math.radians(15.0), 0.02)
  ok = x < -0.5 and y > 0.05
  print(f'  1) δ=+15° 후진 1.2m → 최종 ({x:+.2f}, {y:+.2f})  '
        f'{"후방-좌측 ✅" if ok else "❌ 예상과 다름"}')
  return ok


def make_trajectory():
  """전진 접근 → 후진 진입 궤적 (실제 주차궤적을 흉내낸 것).

  ★ cusp(기어전환)에서 후진 경로는 차량 진행축에 **접해야** 한다. 90° 꺾인
    궤적은 어떤 조향으로도 못 따라간다 — 사람이 기록해도 그렇게는 안 나온다.
  ★ 반경은 최소회전반경(L/tan18° = 2.42m)보다 커야 한다. 여기선 2.6m.
  """
  R = 2.6
  cx, cy = 2.0, -R                   # 원 중심. (2,0) 에서 x축에 접한다.
  pts = []
  # 전진 접근: (-6,0) → (2,0), 0.2m 간격
  for i in range(41):
    pts.append((-6.0 + 0.2 * i, 0.0, 1))
  # 후진 사분원: 각도 90° → 180° (진행방향은 -x 에서 -y 로 돈다)
  n = 40
  for k in range(1, n + 1):
    th = math.radians(90.0 + 90.0 * (k / n))
    pts.append((cx + R * math.cos(th), cy + R * math.sin(th), -1))
  # 마지막 직선 후진: 주차칸 안쪽으로 1m 더
  ex, ey = cx - R, cy
  for k in range(1, 6):
    pts.append((ex, ey - 0.2 * k, -1))
  return pts


def follow(traj, rev_sign, ld=0.8, v_mag=0.3, dt=0.05, max_t=200.0):
  """parking_node 와 같은 방식으로 궤적을 추종한다. 최종 목표점 오차를 반환."""
  segs = split_segments(traj)
  x, y = traj[0][0], traj[0][1]
  yaw = 0.0
  for seg_i, (gear, pl) in enumerate(segs):
    cursor = 0
    t = 0.0
    is_last = (seg_i == len(segs) - 1)
    tol = 0.15 if is_last else 0.20
    while t < max_t:
      # 커서 전진 (뒤로 안 감)
      best_i, best_d = cursor, float('inf')
      for i in range(cursor, len(pl)):
        d = math.hypot(pl[i][0] - x, pl[i][1] - y)
        if d < best_d:
          best_d, best_i = d, i
        elif d > best_d + 2.0:
          break
      cursor = best_i
      if best_d > 1.0:
        return None, f'구간{seg_i} 궤적 이탈 {best_d:.2f}m'

      rem = sum(math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1])
                for i in range(cursor, len(pl) - 1))
      end_d = math.hypot(pl[-1][0] - x, pl[-1][1] - y)
      if min(rem, end_d) <= tol:
        break

      # lookahead
      s, la = 0.0, pl[-1]
      for i in range(cursor, len(pl) - 1):
        seg = math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1])
        if s + seg >= ld:
          f = (ld - s) / seg if seg > 1e-6 else 0.0
          la = (pl[i][0] + f * (pl[i + 1][0] - pl[i][0]),
                pl[i][1] + f * (pl[i + 1][1] - pl[i][1]))
          break
        s += seg

      dx, dy = la[0] - x, la[1] - y
      c, sn = math.cos(yaw), math.sin(yaw)
      x_ld, y_ld = dx * c + dy * sn, -dx * sn + dy * c
      delta = pursuit_steer(x_ld, y_ld, L, MAX_STEER)
      if gear < 0:
        delta *= rev_sign
      x, y, yaw = step(x, y, yaw, v_mag * (1 if gear > 0 else -1), delta, dt)
      t += dt
    else:
      return None, f'구간{seg_i} 시간초과'
  goal = traj[-1]
  return math.hypot(x - goal[0], y - goal[1]), f'최종 ({x:+.2f},{y:+.2f}) yaw={math.degrees(yaw):+.0f}°'


def main():
  print('후진 pure pursuit 기하 검증\n' + '=' * 46)
  ok1 = test_reverse_sign()

  traj = make_trajectory()
  segs = split_segments(traj)
  print(f'\n  궤적: {len(traj)}점 → {len(segs)}구간 '
        f'({" ".join("전진" if g > 0 else "후진" for g, _ in segs)})')
  print(f'  목표 정차점: ({traj[-1][0]:.2f}, {traj[-1][1]:.2f})')

  err, why = follow(traj, rev_sign=+1.0)
  ok2 = err is not None and err < 0.25
  print(f'\n  2) steer_sign_reverse=+1 → '
        f'{f"오차 {err*100:.0f}cm" if err is not None else why}  '
        f'{"✅" if ok2 else "❌"}')
  if err is not None:
    print(f'     {why}')

  err_b, why_b = follow(traj, rev_sign=-1.0)
  ok3 = err_b is None or err_b > 0.5
  print(f'  3) steer_sign_reverse=-1 (뒤집음) → '
        f'{f"오차 {err_b*100:.0f}cm" if err_b is not None else why_b}  '
        f'{"✅ 제대로 실패함" if ok3 else "❌ 부호를 가르지 못한다"}')

  allok = ok1 and ok2 and ok3
  print('\n' + '=' * 46)
  print('결론: 후진 조향 부호는 전진과 같다(steer_sign_reverse=+1). ✅'
        if allok else '결론: ❌ 검증 실패 — 부호/로직을 다시 볼 것.')
  return 0 if allok else 1


if __name__ == '__main__':
  sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reverse_waypoints.py — 웨이포인트 경로를 **거꾸로 뒤집는다**.

★ 왜 (2026-09-18)
  같은 경사로를 **내리막으로** 타 보려면 반대 방향 경로가 필요하다. 다시
  기록하려면 RTK 를 다시 잡고 차로 한 바퀴 돌아야 하는데, 이미 찍어 둔
  경로를 뒤집으면 그대로 쓸 수 있다. 같은 노면·같은 좌표계라 비교도 쉽다.

★ 그냥 뒤집으면 안 되는 것 — 그래서 이 도구가 검사한다
  ① **앞 10m 직진성.** 뒤집으면 캘리브 구간이 원래 경로의 **끝부분**이 된다.
     heading_init 은 웨이포인트를 안 보고 '차가 향한 방향' 으로 10m 직진하며
     yaw_offset 을 구한다. 그 구간이 휘어 있으면 캘리브가 거부되거나,
     통과해도 헤딩이 그만큼 틀어진 채 전 구간을 달린다.
     판정은 heading_init 과 **같은 방식**으로 한다(현 2m 이상에서 최근 1.5m
     진행방향과 비교, 거부선 20°).
  ② **원점 스탬프.** 이게 없으면 global_path_publisher 가 '다른 장소 파일'
     로 걸러낸다(strict 모드에서 발행 거부). 원본에서 그대로 옮긴다.
  ③ **최소 회전반경.** 뒤집어도 곡률은 같지만, 값을 확인하고 넘어간다.

사용:
  python3 tools/reverse_waypoints.py <입력.yaml>                 # 미리보기
  python3 tools/reverse_waypoints.py <입력.yaml> --write         # 실제로 쓴다
  python3 tools/reverse_waypoints.py <입력.yaml> --out <경로>    # 파일명 지정

기본 출력 파일명은 입력에 `_rev` 를 붙인다.
"""

import argparse
import math
import os
import sys

import yaml

WHEELBASE, MAX_STEER_DEG = 0.785, 18.0
R_MIN = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))   # 2.42 m
# heading_init 과 같은 값 (gps_heading_init/heading_init_node.py)
CALIB_M = 10.0
MAX_DEV_DEG = 20.0
MIN_CHORD_M = 2.0
SEG_WIN_M = 1.5


def norm(a):
  return (a + 180.0) % 360.0 - 180.0


def arclen(pts):
  s = [0.0]
  for i in range(1, len(pts)):
    s.append(s[-1] + math.dist(pts[i], pts[i - 1]))
  return s


def calib_deviation(pts, s):
  """heading_init 과 같은 방식으로 앞 CALIB_M 의 직진성을 잰다.

  시작점→현재점 '현' 방향과, 최근 SEG_WIN_M 구간의 진행방향을 비교한다.
  현이 MIN_CHORD_M 보다 짧으면 방향 자체가 잡음이라 건너뛴다.
  """
  worst = 0.0
  for i in range(len(pts)):
    if s[i] < MIN_CHORD_M or s[i] > CALIB_M + 2.0:
      continue
    chord = math.degrees(math.atan2(pts[i][1] - pts[0][1],
                                    pts[i][0] - pts[0][0]))
    j = i
    while j > 0 and s[i] - s[j] < SEG_WIN_M:
      j -= 1
    seg = math.degrees(math.atan2(pts[i][1] - pts[j][1],
                                  pts[i][0] - pts[j][0]))
    worst = max(worst, abs(norm(seg - chord)))
  return worst


def lateral_drift(pts, s):
  """출발 방향 기준 직선에서 앞 CALIB_M 동안 옆으로 얼마나 벗어나나."""
  if len(pts) < 5:
    return 0.0
  h = math.atan2(pts[4][1] - pts[0][1], pts[4][0] - pts[0][0])
  c, si = math.cos(h), math.sin(h)
  out = 0.0
  for i in range(len(pts)):
    if s[i] > CALIB_M:
      break
    dx, dy = pts[i][0] - pts[0][0], pts[i][1] - pts[0][1]
    out = max(out, abs(-si * dx + c * dy))
  return out


def min_radius(pts, d=2):
  """저장소 정의: i±2 (기선 2m). course_map.py · mission_plan_check.py 와 같다."""
  best, at = 9999.0, 0.0
  s = arclen(pts)
  for i in range(d, len(pts) - d):
    a, b, c = pts[i - d], pts[i], pts[i + d]
    A, B, C = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    ar = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
    if ar <= 1e-9:
      continue
    R = (A * B * C) / (4 * ar)
    if R < best:
      best, at = R, s[i]
  return best, at


def main():
  ap = argparse.ArgumentParser(
      description='웨이포인트를 뒤집는다 (내리막 시험 등 반대 방향 주행용).')
  ap.add_argument('src', help='입력 웨이포인트 YAML')
  ap.add_argument('--out', default=None, help='출력 경로 (기본: 입력명 + _rev)')
  ap.add_argument('--write', action='store_true',
                  help='실제로 파일을 쓴다 (기본은 미리보기만)')
  a = ap.parse_args()

  with open(a.src) as fh:
    doc = yaml.safe_load(fh)
  if not doc or 'waypoints' not in doc:
    print(f'❌ waypoints 가 없다: {a.src}')
    return 1
  wp = doc['waypoints']
  pts = [(float(p['x']), float(p['y'])) for p in wp]
  rev = list(reversed(pts))
  s = arclen(rev)

  out = a.out or (os.path.splitext(a.src)[0] + '_rev.yaml')
  print(f'입력  {a.src}')
  print(f'출력  {out}')
  print(f'  {len(rev)}점 · {s[-1]:.1f}m')
  print(f'  시작 ({rev[0][0]:+.2f}, {rev[0][1]:+.2f})   ← 원래 종점')
  print(f'  종점 ({rev[-1][0]:+.2f}, {rev[-1][1]:+.2f})   ← 원래 출발점')

  fail = False

  # ① 캘리브 구간 — 뒤집으면 원래 경로의 끝부분이다
  dev = calib_deviation(rev, s)
  drift = lateral_drift(rev, s)
  ok = dev < MAX_DEV_DEG
  print(f'\n■ 앞 {CALIB_M:.0f}m 직진성 (heading_init 과 같은 판정)')
  print(f'  최대 편차 {dev:.1f}°  (거부선 {MAX_DEV_DEG:.0f}°)   '
        f'{"✅" if ok else "❌ 캘리브가 거부된다"}')
  print(f'  횡이탈    {drift:.2f} m')
  if not ok:
    fail = True
    print('  → 이 방향으로는 캘리브를 못 한다. 출발점을 옮겨 다시 기록할 것.')

  # ② 원점 스탬프 — 없으면 global_path_publisher 가 발행을 거부한다
  origin = doc.get('origin')
  print('\n■ 원점 스탬프')
  if origin:
    print(f'  ✅ {origin.get("site", "(site 없음)")} '
          f'({origin.get("x")}, {origin.get("y")})  — 그대로 옮긴다')
  else:
    print('  ⚠ 원본에 원점 스탬프가 없다. global_path_publisher 가 '
          'strict 모드에서 거부할 수 있다 (tools/shift_waypoints.py --stamp-only)')

  # ③ 곡률 — 뒤집어도 같지만 확인하고 넘어간다
  R, at = min_radius(rev)
  print('\n■ 곡률')
  print(f'  최소 R {R:.2f} m @ s={at:.1f}m   (차량 한계 {R_MIN:.2f} m)   '
        f'{"✅" if R >= R_MIN else "❌ 못 도는 커브가 있다"}')
  if R < R_MIN:
    fail = True

  if fail:
    print('\n❌ 위 문제를 해결하기 전에는 쓰지 말 것.')
    return 1

  if not a.write:
    print('\n미리보기만 했다. 실제로 쓰려면 --write 를 줄 것.')
    return 0

  new = dict(doc)
  new['waypoints'] = [{'x': float(x), 'y': float(y)} for x, y in rev]
  with open(out, 'w') as fh:
    fh.write(f'# {os.path.basename(a.src)} 를 뒤집은 경로 '
             f'(tools/reverse_waypoints.py). 손으로 고치지 말 것.\n')
    yaml.safe_dump(new, fh, default_flow_style=False, sort_keys=False,
                   allow_unicode=True)
  print(f'\n✅ 저장 {out}')
  return 0


if __name__ == '__main__':
  sys.exit(main())

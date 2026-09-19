#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""slice_waypoints.py — 경로의 한 구간만 잘라 낸다 (예선용).

★ 왜 (2026-09-19, 용인 예선)
  예선은 굴절코스·S자만 달려서 통과해야 본선에 간다. 648m 본 코스 파일을
  그대로 쓰면 그 구간을 지나서도 계속 달린다.

★ 자를 때 반드시 챙길 것 셋 — 하나라도 빠뜨리면 현장에서 조용히 실패한다
  ① **캘리브 조주구간**. heading_init 은 웨이포인트를 안 보고 '차가 향한
     방향' 으로 calib_distance 만큼 직진한다. 그래서 **구간 시작보다 앞에서**
     잘라야 그 앞부분이 캘리브 구간이 된다. 구간 경계에서 자르면 첫 10m 가
     커브라 캘리브가 거부된다(실측: s58 에서 자르면 편차 32.1°, 거부선 20°).
  ② **origin 스탬프**. 없으면 global_path_publisher 가 발행을 거부한다.
     원본 파일의 스탬프를 그대로 옮긴다.
  ③ **최소 곡률**. 잘린 구간이 차량 한계를 넘는지 본다.

  판정은 전부 heading_init·preflight 와 **같은 방식**으로 한다. 다르게 재면
  '도구는 통과인데 현장에서 거부' 가 난다.

사용:
  python3 tools/slice_waypoints.py <원본> --from 48 --to 140
  python3 tools/slice_waypoints.py <원본> --from 48 --to 140 --out <경로> --write

  --from 은 **구간 시작보다 calib_distance 만큼 앞**으로 잡을 것.
"""

import argparse
import math
import os

import yaml

CALIB_M = 10.0          # heading_init 기본 calib_distance
SEG_WIN_M = 1.5         # heading_init 의 seg_window_m
MIN_CHORD_M = 2.0       # heading_init 의 min_chord_for_dev
MAX_DEV_DEG = 20.0      # heading_init 의 max_deviation_deg
WHEELBASE = 0.785
MAX_STEER_DEG = 18.0
R_MIN = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))   # 2.4157 m


def arclen(pts):
  s = [0.0]
  for a, b in zip(pts, pts[1:]):
    s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
  return s


def calib_deviation(pts, s, dist=CALIB_M):
  """heading_init 과 같은 판정 — 시작→현재 '현' 과 최근 SEG_WIN_M 구간의 차."""
  dev = 0.0
  for i in range(1, len(pts)):
    if s[i] > dist:
      break
    if s[i] < MIN_CHORD_M:
      continue
    chord = math.degrees(math.atan2(pts[i][1] - pts[0][1],
                                    pts[i][0] - pts[0][0]))
    j = i
    while j > 0 and s[i] - s[j] < SEG_WIN_M:
      j -= 1
    seg = math.degrees(math.atan2(pts[i][1] - pts[j][1],
                                  pts[i][0] - pts[j][0]))
    dev = max(dev, abs((seg - chord + 180) % 360 - 180))
  return dev


def lateral_drift(pts, s, dist=CALIB_M):
  """시작 방향 직선에서 얼마나 벗어나는가."""
  k = min(range(len(pts)), key=lambda i: abs(s[i] - MIN_CHORD_M))
  c = math.atan2(pts[k][1] - pts[0][1], pts[k][0] - pts[0][0])
  out = 0.0
  for i, p in enumerate(pts):
    if s[i] > dist:
      break
    out = max(out, abs(-(p[0] - pts[0][0]) * math.sin(c)
                       + (p[1] - pts[0][1]) * math.cos(c)))
  return out


def min_radius(pts, s):
  """인접 3점 외접원. preflight.py 와 같은 보수적 방식."""
  best, at = 1e9, 0.0
  for i in range(1, len(pts) - 1):
    A, B, C = pts[i - 1], pts[i], pts[i + 1]
    a = math.hypot(B[0] - A[0], B[1] - A[1])
    b = math.hypot(C[0] - B[0], C[1] - B[1])
    c = math.hypot(C[0] - A[0], C[1] - A[1])
    ar = abs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1])) / 2
    if ar <= 1e-9:
      continue
    R = (a * b * c) / (4 * ar)
    if R < best:
      best, at = R, s[i]
  return best, at


def main():
  ap = argparse.ArgumentParser(
      description='경로의 한 구간만 잘라 낸다 (예선용).')
  ap.add_argument('src', help='원본 웨이포인트 YAML')
  ap.add_argument('--from', dest='s0', type=float, required=True,
                  help='자를 시작 s [m] — 구간 시작보다 calib 거리만큼 앞으로')
  ap.add_argument('--to', dest='s1', type=float, required=True,
                  help='자를 끝 s [m]')
  ap.add_argument('--calib', type=float, default=CALIB_M,
                  help=f'캘리브 직진거리 [m] (기본 {CALIB_M})')
  ap.add_argument('--out', default=None)
  ap.add_argument('--write', action='store_true',
                  help='실제로 파일을 쓴다 (기본은 미리보기만)')
  a = ap.parse_args()

  with open(a.src) as fh:
    doc = yaml.safe_load(fh)
  if not doc or 'waypoints' not in doc:
    print(f'❌ waypoints 가 없다: {a.src}')
    return 1
  full = [(float(p['x']), float(p['y'])) for p in doc['waypoints']]
  fs = arclen(full)

  if a.s1 <= a.s0:
    print(f'❌ --to({a.s1}) 가 --from({a.s0}) 보다 크지 않다')
    return 1
  if a.s1 > fs[-1]:
    print(f'❌ --to({a.s1}m) 가 코스 길이({fs[-1]:.1f}m) 밖이다')
    return 1

  idx = [i for i in range(len(full)) if a.s0 <= fs[i] <= a.s1]
  if len(idx) < 4:
    print(f'❌ 잘린 점이 {len(idx)}개뿐이다 — 구간이 너무 짧다')
    return 1
  pts = [full[i] for i in idx]
  s = arclen(pts)

  out = a.out or (os.path.splitext(a.src)[0]
                  + f'_s{int(a.s0)}_{int(a.s1)}.yaml')
  print(f'입력  {a.src}  ({len(full)}점 {fs[-1]:.1f}m)')
  print(f'출력  {out}')
  print(f'  s={a.s0:.1f}~{a.s1:.1f}m  →  {len(pts)}점 · {s[-1]:.1f}m')
  print(f'  시작 ({pts[0][0]:+.2f}, {pts[0][1]:+.2f})'
        f'   종점 ({pts[-1][0]:+.2f}, {pts[-1][1]:+.2f})')

  fail = False

  print(f'\n■ 앞 {a.calib:.0f}m 직진성 (heading_init 과 같은 판정)')
  dev = calib_deviation(pts, s, a.calib)
  drift = lateral_drift(pts, s, a.calib)
  ok = dev < MAX_DEV_DEG
  print(f'  최대 편차 {dev:.1f}°  (거부선 {MAX_DEV_DEG:.0f}°)   '
        f'{"✅" if ok else "❌ 캘리브가 거부된다"}')
  print(f'  횡이탈    {drift:.2f} m')
  if not ok:
    fail = True
    print(f'  → --from 을 더 앞으로 당길 것. 이 구간에서 캘리브를 못 한다.')
  elif dev > MAX_DEV_DEG * 0.5:
    print(f'  ⚠ 거부선의 절반을 넘었다 — --from 을 더 앞으로 당기는 편이 안전하다')

  print('\n■ 원점 스탬프')
  origin = doc.get('origin')
  if origin:
    print(f'  ✅ {origin.get("site", "?")} '
          f'({origin.get("x")}, {origin.get("y")})  — 그대로 옮긴다')
  else:
    fail = True
    print('  ❌ 원본에 원점이 없다 — global_path_publisher 가 발행을 거부한다')

  print('\n■ 곡률')
  R, at = min_radius(pts, s)
  need = math.degrees(math.atan(WHEELBASE / R)) if R < 1e8 else 0.0
  okR = R >= R_MIN
  print(f'  최소 R {R:.2f} m @ s={at:.1f}m   (차량 한계 {R_MIN:.2f} m)   '
        f'필요타각 {need:.1f}° / 상한 {MAX_STEER_DEG:.0f}°   '
        f'{"✅" if okR else "❌ 못 돈다"}')
  if not okR:
    fail = True

  print()
  if fail:
    print('❌ 이대로 쓰면 현장에서 실패한다.')
    return 1
  if not a.write:
    print('미리보기만 했다. 실제로 쓰려면 --write 를 줄 것.')
    return 0

  new = {}
  if origin:
    new['origin'] = origin
  new['waypoints'] = [{'x': float(x), 'y': float(y)} for x, y in pts]
  with open(out, 'w') as fh:
    fh.write(f'# {os.path.basename(a.src)} 의 s={a.s0:.1f}~{a.s1:.1f}m 구간.\n')
    fh.write('# tools/slice_waypoints.py 가 만들었다. 손으로 고치지 말 것.\n')
    fh.write(f'# 앞 {a.calib:.0f}m 가 캘리브 조주구간이다 (편차 {dev:.1f}°).\n')
    yaml.safe_dump(new, fh, allow_unicode=True, sort_keys=False)
  print(f'✅ 썼다: {out}')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

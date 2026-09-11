#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""latlon_to_local.py — 구형 위경도 웨이포인트를 현재 파이프라인 포맷으로 바꾼다.

★ 왜 필요한가
  옛날에 기록한 파일(`~/waypoints/all_waypoint.yaml` 등)은 위경도만 들어 있다:
      waypoints: [{latitude: .., longitude: ..}, ...]
  지금 스택은 **로컬 좌표(x, y) + 원점 스탬프**를 쓴다. 원점 스탬프가 있어야
  `global_path_publisher` 가 "다른 장소 파일" 을 거부할 수 있다(origin_check).
  스탬프가 없으면 장소가 바뀌었을 때 조용히 150km 어긋난 경로로 달린다.

★ 이 도구가 하는 일 (좌표 변환만. 리샘플·스무딩은 smooth_path.py 가 한다)
  위경도 → UTM(EPSG:32652) → 원점 빼기 → 로컬 x,y
  + 중복점 제거(같은 자리 연속 기록)
  + 진단: 길이 / 점간격 / 닫힌 루프 여부 / 최소 회전반경 / 되꺾임(cusp)

★ 원점을 어디서 가져오나
  기본은 `config/site_origin.yaml`(현재 장소). 다른 장소 경로를 변환할 땐
  `--auto` 로 경로 자체에서 원점을 만든다(100m 내림).
  ⚠ **변환한 경로로 실제 주행하려면 site_origin.yaml 도 그 장소여야 한다.**
    대회 직전이라면 원점을 바꿨다가 되돌리는 것을 잊지 말 것 —
    `tools/set_origin_from_fix.py` 가 .bak 를 남긴다.

사용:
  # 진단만 (파일 안 씀)
  python3 tools/latlon_to_local.py --in ~/waypoints/all_waypoint.yaml --auto
  # 변환해서 저장
  python3 tools/latlon_to_local.py --in ~/waypoints/all_waypoint.yaml --auto \
      --out /tmp/wp_school_raw.yaml --site "충주 캠퍼스" --write
  # 일부 구간만 (회피 시험용 짧은 구간)
  python3 tools/latlon_to_local.py --in <파일> --auto --segment 0:120 --out .. --write

  그 다음 0.5m 리샘플 + 스무딩:
  python3 tools/smooth_path.py --in /tmp/wp_school_raw.yaml \
      --out config/school/wp_school_0.5.yaml --step 0.5 --write
"""

import argparse
import math
import os
import sys

import yaml


def load_latlon(path):
  d = yaml.safe_load(open(path, encoding='utf-8')) or {}
  raw = d.get('waypoints') or d.get('poses') or []
  pts = []
  for p in raw:
    if 'latitude' in p and 'longitude' in p:
      pts.append((float(p['latitude']), float(p['longitude'])))
    elif 'lat' in p and 'lon' in p:
      pts.append((float(p['lat']), float(p['lon'])))
  return pts


def radii(w):
  """세 점 외접원 반지름 — 못 도는 커브를 찾는다."""
  out = []
  for i in range(1, len(w) - 1):
    a, b, c = w[i - 1], w[i], w[i + 1]
    A = math.dist(a, b)
    B = math.dist(b, c)
    C = math.dist(a, c)
    ar = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
    out.append(A * B * C / (4 * ar) if ar > 1e-9 else 9999.0)
  return out


def cusps(w, thresh_deg=120.0):
  """되꺾임 — 전진 전용 pure_pursuit 는 여기서 멈춘다."""
  out = []
  for i in range(1, len(w) - 1):
    h0 = math.atan2(w[i][1] - w[i - 1][1], w[i][0] - w[i - 1][0])
    h1 = math.atan2(w[i + 1][1] - w[i][1], w[i + 1][0] - w[i][0])
    d = abs(math.degrees(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))))
    if d > thresh_deg:
      out.append((i, d))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--in', dest='inp', required=True)
  ap.add_argument('--out', default=None)
  ap.add_argument('--epsg', type=int, default=32652)
  ap.add_argument('--origin', type=float, nargs=2, default=None,
                  metavar=('X', 'Y'), help='UTM 원점을 직접 지정')
  ap.add_argument('--auto', action='store_true',
                  help='경로에서 원점을 만든다(100m 내림). 다른 장소 경로용')
  ap.add_argument('--site', default='', help='원점 스탬프에 적을 장소 이름')
  ap.add_argument('--segment', default=None, metavar='A:B',
                  help='인덱스 구간만 자른다 (예: 0:120)')
  ap.add_argument('--dedup', type=float, default=0.05,
                  help='이 거리 이내 연속점은 중복으로 보고 제거[m]')
  ap.add_argument('--min-radius', type=float, default=2.42,
                  help='최대타각 18°의 최소 회전반경')
  ap.add_argument('--write', action='store_true')
  args = ap.parse_args()

  try:
    from pyproj import Transformer
  except ImportError:
    print('pyproj 필요: pip install pyproj', file=sys.stderr)
    return 1

  ll = load_latlon(args.inp)
  if len(ll) < 2:
    print(f'위경도 점이 없다: {args.inp}', file=sys.stderr)
    return 1

  tr = Transformer.from_crs('EPSG:4326', f'EPSG:{args.epsg}', always_xy=True)
  utm = [tr.transform(lon, lat) for (lat, lon) in ll]

  # 원점 결정
  if args.origin:
    ox, oy = args.origin
    src = '명시'
  elif args.auto:
    ox = math.floor(min(p[0] for p in utm) / 100.0) * 100.0
    oy = math.floor(min(p[1] for p in utm) / 100.0) * 100.0
    src = '경로에서 자동(100m 내림)'
  else:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'src', 'waypoint_follower'))
    from waypoint_follower.site_origin import load_site_origin
    _, ox, oy, s = load_site_origin()
    src = f'site_origin.yaml ({s})'

  pts = [(x - ox, y - oy) for (x, y) in utm]

  if args.segment:
    a, b = args.segment.split(':')
    a = int(a) if a else 0
    b = int(b) if b else len(pts)
    pts = pts[a:b]
    print(f'구간 잘라냄: [{a}:{b}] → {len(pts)}점')

  # 중복 제거
  clean = [pts[0]]
  for p in pts[1:]:
    if math.dist(p, clean[-1]) > args.dedup:
      clean.append(p)
  dropped = len(pts) - len(clean)
  pts = clean

  seg = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
  total = sum(seg)
  gap = math.dist(pts[0], pts[-1])
  r = radii(pts)
  tight = [i for i, v in enumerate(r) if v < args.min_radius * 1.45]
  cant = [(i, v) for i, v in enumerate(r) if v < args.min_radius]
  cs = cusps(pts)

  print('=' * 68)
  print(f'위경도 → 로컬 변환 — {os.path.basename(args.inp)}')
  print('=' * 68)
  print(f'  원점     : ({ox:.1f}, {oy:.1f}) EPSG:{args.epsg}  [{src}]')
  print(f'  점       : {len(pts)}개 (중복 {dropped}개 제거)')
  print(f'  길이     : {total:.1f}m · 점간격 {min(seg):.2f}~{max(seg):.2f}m '
        f'(평균 {total / len(seg):.2f})')
  print(f'  범위     : x {min(p[0] for p in pts):.1f}~{max(p[0] for p in pts):.1f}  '
        f'y {min(p[1] for p in pts):.1f}~{max(p[1] for p in pts):.1f}')
  print(f'  시작-끝  : {gap:.2f}m → '
        + ('닫힌 루프(순환 주행 가능)' if gap < 5.0 else '열린 경로'))
  print('-' * 68)
  if max(seg) > 2.0:
    print(f'  ⚠ 점간격이 최대 {max(seg):.1f}m 다. local_sliding_window 가 전방 20점을')
    print('     쓰므로 이대로는 로컬 경로가 수십 m 로 늘어나 추종이 망가진다.')
    print('     → smooth_path.py --step 0.5 로 리샘플할 것 (필수)')
  if cant:
    print(f'  ❌ 못 도는 커브 {len(cant)}개 (R < {args.min_radius:.2f}m):')
    for i, v in cant[:5]:
      print(f'       idx{i} R={v:.2f}m → 필요타각 '
            f'{math.degrees(math.atan(0.785 / v)):.1f}°')
  elif tight:
    print(f'  ⚠ 빠듯한 커브 {len(tight)}개 (R < {args.min_radius * 1.45:.2f}m) '
          f'— 최소 R={min(r):.2f}m')
  else:
    print(f'  ✅ 커브 여유 있음 (최소 R={min(r):.2f}m)')
  if cs:
    print(f'  ❌ 되꺾임 {len(cs)}개 — 전진 전용 pure_pursuit 는 여기서 멈춘다:')
    for i, d in cs[:5]:
      print(f'       idx{i} {d:.0f}° 반전  → tools/split_path.py 로 도려낼 것')
  else:
    print('  ✅ 되꺾임 없음')
  print('=' * 68)

  if not args.write:
    print('  (미리보기 — 저장하려면 --write --out <파일>)')
    return 0
  if not args.out:
    print('--write 에는 --out 이 필요하다', file=sys.stderr)
    return 1

  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  doc = {
      'origin': {'x': float(ox), 'y': float(oy), 'epsg': int(args.epsg),
                 'site': args.site or f'변환: {os.path.basename(args.inp)}'},
      'waypoints': [{'x': round(p[0], 4), 'y': round(p[1], 4)} for p in pts],
  }
  with open(args.out, 'w', encoding='utf-8') as f:
    yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
  print(f'  저장: {args.out}')
  print('  다음: python3 tools/smooth_path.py --in '
        f'{args.out} --out <최종본> --step 0.5 --write')
  return 0


if __name__ == '__main__':
  sys.exit(main())

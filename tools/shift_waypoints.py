#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shift_waypoints.py — 좌표 YAML 을 다른 원점 기준으로 옮긴다(+원점 스탬프).

★ 왜 필요한가
  로컬좌표는 (UTM − 원점)이다. 원점을 바꾸면(장소 이동) 기존 파일의 좌표는
  전부 통째로 평행이동해야 한다. config/site_origin.yaml 주석이 예전부터 이
  도구를 가리키고 있었는데 **실제로는 없었다.** 여기서 만든다.

★ 두 가지 용도
  1) 환산 — 옛 원점 기준 파일을 현재 원점 기준으로 옮긴다.
       local_new = local_old + (old_origin − new_origin)
  2) 스탬프 (--stamp-only) — 좌표는 그대로 두고 "이 파일은 이 원점 것"이라고
       기록만 한다. 원점이 안 적힌 구버전 파일에 쓴다. 스탬프가 있어야
       global_path_publisher 가 다른 장소 파일을 걸러낼 수 있다.

★ 안전장치
  - 기본은 **미리보기**. --write 를 줘야 실제로 쓴다.
  - 쓸 때 원본을 .bak 로 백업한다.
  - 환산량이 1km 를 넘으면 "다른 장소" 로 크게 경고한다.

지원 포맷 (한 파일에 섞여 있어도 된다)
  waypoints / stop_points / detected / points : [{x, y, ...}, ...]
  pose                                        : {x, y, ...}
  원점 스탬프 : origin: {x, y, epsg, site}  또는  origin_x / origin_y

사용:
  # 현재 원점(site_origin.yaml) 기준으로 환산 — 파일의 스탬프를 출발점으로
  python3 tools/shift_waypoints.py path.yaml

  # 스탬프가 없는 구버전 파일: 옛 원점을 알려준다
  python3 tools/shift_waypoints.py path.yaml --from-origin 477800 3964400 --write

  # 좌표는 그대로, 원점만 기록 (이 파일이 대구 것임을 명시)
  python3 tools/shift_waypoints.py path.yaml --from-origin 477800 3964400 \
      --site "대구권 시험장" --stamp-only --write
"""

import argparse
import math
import os
import shutil
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'src', 'waypoint_follower'))
sys.path.insert(0, '/home/han/racing_ws/src/waypoint_follower')
from waypoint_follower.site_origin import (  # noqa: E402
    load_site_origin, origin_stamp, read_origin_stamp)

# 좌표 리스트가 들어 있는 키들. 없는 키는 그냥 넘어간다.
LIST_KEYS = ('waypoints', 'stop_points', 'detected', 'points', 'poses')
DICT_KEYS = ('pose',)


def shift_doc(d, dx, dy):
  """dict 안의 모든 좌표를 (dx, dy) 만큼 옮긴다. 옮긴 점 개수를 돌려준다."""
  n = 0
  for k in LIST_KEYS:
    seq = d.get(k)
    if not isinstance(seq, list):
      continue
    for p in seq:
      if isinstance(p, dict) and 'x' in p and 'y' in p:
        p['x'] = round(float(p['x']) + dx, 6)
        p['y'] = round(float(p['y']) + dy, 6)
        n += 1
  for k in DICT_KEYS:
    p = d.get(k)
    if isinstance(p, dict) and 'x' in p and 'y' in p:
      p['x'] = round(float(p['x']) + dx, 6)
      p['y'] = round(float(p['y']) + dy, 6)
      n += 1
  return n


def bbox(d):
  """파일 안 모든 좌표의 (xmin, xmax, ymin, ymax). 없으면 None."""
  xs, ys = [], []
  for k in LIST_KEYS:
    for p in (d.get(k) or []) if isinstance(d.get(k), list) else []:
      if isinstance(p, dict) and 'x' in p and 'y' in p:
        xs.append(float(p['x']))
        ys.append(float(p['y']))
  for k in DICT_KEYS:
    p = d.get(k)
    if isinstance(p, dict) and 'x' in p and 'y' in p:
      xs.append(float(p['x']))
      ys.append(float(p['y']))
  if not xs:
    return None
  return (min(xs), max(xs), min(ys), max(ys))


def set_stamp(d, epsg, ox, oy, site):
  """원점 스탬프를 갱신한다. 파일이 쓰던 포맷을 존중한다."""
  if 'origin_x' in d or 'origin_y' in d:
    d['origin_x'] = float(ox)
    d['origin_y'] = float(oy)
    if epsg is not None:
      d['utm_epsg'] = int(epsg)
    if site:
      d['site'] = str(site)
  else:
    d['origin'] = origin_stamp(epsg, ox, oy, site)


def reorder(d):
  """원점 스탬프를 파일 맨 위로 올린다 — 열자마자 어느 장소 것인지 보이게."""
  head = [k for k in ('origin', 'origin_x', 'origin_y', 'utm_epsg', 'site')
          if k in d]
  return {**{k: d[k] for k in head},
          **{k: v for k, v in d.items() if k not in head}}


def process(path, args, cur):
  cur_epsg, cur_ox, cur_oy, cur_site = cur
  print(f'\n=== {path} ===')
  try:
    with open(path) as f:
      d = yaml.safe_load(f)
  except Exception as e:  # noqa: BLE001
    print(f'  ❌ 읽기 실패: {e}')
    return False
  if not isinstance(d, dict):
    print('  ❌ 최상위가 dict 가 아니다 — 지원하지 않는 포맷')
    return False

  f_ox, f_oy, f_epsg, f_site = read_origin_stamp(d)
  if args.from_origin is not None:
    f_ox, f_oy = args.from_origin
    src_desc = f'--from-origin ({f_ox:.1f}, {f_oy:.1f})'
  elif f_ox is not None:
    src_desc = f'파일 스탬프 ({f_ox:.1f}, {f_oy:.1f}) — {f_site}'
  else:
    print('  ❌ 파일에 원점이 없고 --from-origin 도 없다. '
          '이 파일이 어느 원점 것인지 모르면 옮길 수 없다.')
    return False
  print(f'  출발 원점 : {src_desc}')

  to_ox, to_oy = args.to_origin if args.to_origin else (cur_ox, cur_oy)
  to_site = args.site or (cur_site if not args.to_origin else '(지정)')
  print(f'  도착 원점 : ({to_ox:.1f}, {to_oy:.1f}) — {to_site}')

  b = bbox(d)
  if b:
    print(f'  현재 좌표 범위 : x[{b[0]:.1f}, {b[1]:.1f}] y[{b[2]:.1f}, {b[3]:.1f}]')

  if args.stamp_only:
    dx = dy = 0.0
    n = sum(len(d[k]) for k in LIST_KEYS
            if isinstance(d.get(k), list))
    print(f'  --stamp-only : 좌표는 그대로 두고 원점만 기록한다 ({n}점 영향 없음)')
    set_stamp(d, f_epsg or cur_epsg, f_ox, f_oy, args.site or f_site or cur_site)
  else:
    dx, dy = float(f_ox) - float(to_ox), float(f_oy) - float(to_oy)
    dist = math.hypot(dx, dy)
    print(f'  이동량 : Δx={dx:+.3f}  Δy={dy:+.3f}   (|Δ|={dist:.3f}m)')
    if dist < 1e-6:
      print('  → 원점이 같다. 스탬프만 갱신한다.')
    elif dist > 1000.0:
      print(f'  ⚠ 이동량이 {dist / 1000:.1f}km 다 — 두 원점은 **다른 장소**다. '
            '옮겨도 그 장소의 경로가 되지는 않는다(좌표만 맞춰질 뿐).')
    n = shift_doc(d, dx, dy)
    print(f'  → {n}개 좌표 이동')
    set_stamp(d, f_epsg or cur_epsg, to_ox, to_oy, to_site)
    b2 = bbox(d)
    if b2:
      print(f'  이동 후 범위 : x[{b2[0]:.1f}, {b2[1]:.1f}] '
            f'y[{b2[2]:.1f}, {b2[3]:.1f}]')

  if not args.write:
    print('  (미리보기 — 실제로 쓰려면 --write)')
    return True

  bak = path + '.bak'
  shutil.copy2(path, bak)
  with open(path, 'w', encoding='utf-8') as f:
    yaml.safe_dump(reorder(d), f, default_flow_style=False, sort_keys=False,
                   allow_unicode=True)
  print(f'  ✅ 저장 완료 (백업 {bak})')
  return True


def main():
  ap = argparse.ArgumentParser(
      description='좌표 YAML 을 다른 원점 기준으로 옮기고 원점을 스탬프한다.')
  ap.add_argument('files', nargs='+', help='대상 YAML (여러 개 가능)')
  ap.add_argument('--from-origin', nargs=2, type=float, metavar=('X', 'Y'),
                  help='옛 원점. 생략하면 파일의 스탬프를 쓴다')
  ap.add_argument('--to-origin', nargs=2, type=float, metavar=('X', 'Y'),
                  help='새 원점. 생략하면 config/site_origin.yaml')
  ap.add_argument('--site', help='새로 기록할 장소 이름')
  ap.add_argument('--stamp-only', action='store_true',
                  help='좌표는 그대로 두고 원점만 기록(구버전 파일 표시용)')
  ap.add_argument('--write', action='store_true', help='실제로 파일에 쓴다')
  args = ap.parse_args()

  cur = load_site_origin()
  print(f'현재 원점 : ({cur[1]:.1f}, {cur[2]:.1f}) EPSG:{cur[0]} — {cur[3]}')

  ok = all([process(p, args, cur) for p in args.files])
  sys.exit(0 if ok else 1)


if __name__ == '__main__':
  main()

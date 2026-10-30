#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hdmap_features.py — 경로 위의 정지선·횡단보도·방지턱·신호등을 정밀지도에서 뽑는다.

★ 왜 필요한가
  미션 지점(정지선 등)은 지금까지 현장에서 차를 세워 찍었다(mission_s.py --live,
  stop_point_recorder). 정밀지도에는 그 위치가 이미 들어 있다. 웨이포인트를
  주면 경로가 지나는 지점의 **진행거리 s** 를 계산해
    · 표로 보여주고
    · mission_plan.yaml 에 붙일 조각(crosswalk, traffic_light)을 찍고
    · --stop-points 로 ~/stop_points.yaml 형식 파일을 쓴다.

★ 판정 방식
  정지선(B2 Kind 530)   경로 선분과 **실제로 교차**하는 점. 지나가지 않으면 안 뽑는다.
  횡단보도(B3 5321)·방지턱(C4)   경로가 다각형에 **처음 들어가는** s ~ 나오는 s.
  신호등(C1)   신호등이 붙은 링크(LinkID)의 끝점이 경로에서 --corridor 안이면,
               그 끝점의 s (= 신호를 기다릴 자리, 보통 정지선 근처).
  주차면(A5)   중심이 경로에서 --park-radius 안인 칸.

★ 정지점 위치
  정지선 교차점은 '선' 자체다. 차의 기준점(base_link)이 어디서 서야 하는지는
  차마다 다르다 → --setback 으로 경로를 따라 그만큼 앞당긴 점을 쓴다.

사용:
  export HDMAP_DIR=".../교통대-정밀지도/교통대"
  python3 tools/hdmap_features.py --waypoints config/chungju_school/wp_school_track_0.5.yaml
  python3 tools/hdmap_features.py --waypoints wp.yaml --setback 0.6 --stop-points ~/stop_points.yaml
"""

import argparse
import math

from hdmap_common import (MAP_EPSG, cumlen, in_polygon, load_layer, load_origin,
                          load_waypoints, map_dir, project, seg_cross)

LIGHT_TYPE = {'1': '횡형 삼색', '2': '횡형 사색A', '3': '횡형 사색B', '4': '횡형 화살표삼색',
              '5': '종형 삼색', '10': '경보형 가변', '11': '보행등', '14': '보조 종형삼색'}
# mission_s.py 의 KNOWN 과 같은 값 (그쪽은 ROS·yaml 을 import 해서 여기서 못 쓴다)
KNOWN = {'crosswalk': ('/crosswalk/arm', '/crosswalk/done', 20.0, 12.0),
         'traffic_light': ('/traffic_light/arm', '/traffic_light/done', 45.0, 30.0)}


def point_at(pts, s, v):
  for i in range(len(s) - 1):
    if s[i] <= v <= s[i + 1]:
      t = (v - s[i]) / max(s[i + 1] - s[i], 1e-9)
      return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t,
              pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t)
  return pts[-1] if v > s[-1] else pts[0]


def crossings(pts, s, line):
  """경로가 폴리라인 line 을 가로지르는 s 목록."""
  out = []
  for i in range(len(pts) - 1):
    for a, b in zip(line, line[1:]):
      t = seg_cross(pts[i], pts[i + 1], a, b)
      if t is not None:
        out.append(s[i] + (s[i + 1] - s[i]) * t)
  return sorted(set(round(v, 2) for v in out))


def spans(pts, s, poly):
  """경로가 다각형 안에 있는 [s_in, s_out] 구간들 (경계 교차로 보간)."""
  edges = crossings(pts, s, poly + [poly[0]])
  inside0 = in_polygon(pts[0], poly)
  marks = ([0.0] if inside0 else []) + edges + ([s[-1]] if in_polygon(pts[-1], poly) else [])
  marks = sorted(set(marks))
  out = []
  for a, b in zip(marks, marks[1:]):
    if in_polygon(point_at(pts, s, (a + b) / 2), poly):
      out.append((a, b))
  return out


def emit(name, sv, total, lead, tail):
  arm, done, tmo, exp = KNOWN[name]
  return '\n'.join([f'  - name: {name}', f'    arm_topic: {arm}', f'    done_topic: {done}',
                    '    trigger:', '      type: course_s',
                    f'      s_enter: {max(0.0, sv - lead):.1f}',
                    f'      s_exit:  {min(total + 20.0, sv + tail):.1f}',
                    f'    timeout_s: {tmo:.1f}', f'    expected_s: {exp:.1f}',
                    '    critical: false'])


def main():
  ap = argparse.ArgumentParser(description='경로 위 미션 지점을 정밀지도에서 뽑는다')
  ap.add_argument('--waypoints', required=True)
  ap.add_argument('--map', help='A2_LINK.shp 가 있는 폴더 (기본: $HDMAP_DIR)')
  ap.add_argument('--origin', help='웨이포인트에 원점 스탬프가 없을 때만')
  ap.add_argument('--corridor', type=float, default=2.5,
                  help='신호등 링크 끝점을 우리 경로로 볼 횡거리 [m]')
  ap.add_argument('--park-radius', type=float, default=6.0, help='주차면 탐색 반경 [m]')
  ap.add_argument('--setback', type=float, default=0.0,
                  help='정지점을 정지선보다 경로상 이만큼 앞에 [m]')
  ap.add_argument('--lead', type=float, default=12.0, help='미션 구간 시작 여유 [m]')
  ap.add_argument('--tail', type=float, default=15.0, help='미션 구간 끝 여유 [m]')
  ap.add_argument('--stop-points', help='정지점을 이 파일로 쓴다 (stop_points.yaml 형식)')
  args = ap.parse_args()

  d = map_dir(args.map)
  loc, org = load_waypoints(args.waypoints)
  if org is None:
    if not args.origin:
      raise SystemExit('❌ 웨이포인트에 원점 스탬프가 없다 — --origin 을 줄 것')
    ox, oy, epsg, _ = load_origin(args.origin)
  else:
    ox, oy, epsg = org
  if epsg != MAP_EPSG:
    raise SystemExit(f'❌ 경로가 EPSG:{epsg} 다 — 지도는 EPSG:{MAP_EPSG}')
  if len(loc) < 2:
    raise SystemExit('❌ 경로가 너무 짧다')
  pts = [(x + ox, y + oy) for x, y in loc]
  s = cumlen(pts)
  total = s[-1]
  xs, ys = [p[0] for p in pts], [p[1] for p in pts]
  box = (min(xs) - 30, max(xs) + 30, min(ys) - 30, max(ys) + 30)
  near = lambda part: any(box[0] < p[0] < box[1] and box[2] < p[1] < box[3] for p in part)
  print(f'경로: {args.waypoints}  {len(pts)}점 · {total:.1f}m')

  links = {r['ID']: [p for pt in parts for p in pt] for r, parts in load_layer(d, 'A2_LINK')}
  if not any(near(v) for v in links.values()):
    raise SystemExit('❌ 경로 근처(30 m)에 지도 링크가 하나도 없다 — 지도 범위 밖이거나 원점이 틀렸다')

  found = []   # (s, 종류, 설명, utm 점)
  for r, parts in load_layer(d, 'B2_SURFACELINEMARK'):
    if r['Kind'] != '530':
      continue
    for part in parts:
      if near(part):
        for sv in crossings(pts, s, part):
          found.append((sv, '정지선', r['ID'], point_at(pts, s, sv)))
  for layer, kind, label in [('B3_SURFACEMARK', '5321', '횡단보도'),
                             ('C4_SPEEDBUMP', None, '과속방지턱')]:
    for r, parts in load_layer(d, layer):
      if kind and r['Kind'] != kind:
        continue
      for part in parts:
        if near(part):
          for a, b in spans(pts, s, part[:-1] if part[0] == part[-1] else part):
            found.append((a, label, f'{r["ID"]}  s {a:.1f}~{b:.1f} (폭 {b - a:.1f}m)',
                          point_at(pts, s, a)))
  # 한 차로에 신호등이 여러 개 달린다(같은 LinkID) — 링크별로 묶어 한 지점으로
  by_link = {}
  for r, parts in load_layer(d, 'C1_TRAFFICLIGHT'):
    by_link.setdefault(r['LinkID'], []).append(r)
  for lid, rs in by_link.items():
    ln = links.get(lid)
    if ln and near(ln):
      sv, dist, _, _ = project(pts, s, ln[-1])
      if dist <= args.corridor and 0.5 < sv < total - 0.5:
        kinds = ', '.join(f'{r["ID"]} {LIGHT_TYPE.get(r["Type"], r["Type"])}' for r in rs)
        found.append((sv, '신호등', f'{kinds} (링크 {lid} 끝, 경로에서 {dist:.1f}m)',
                      point_at(pts, s, sv)))
  for r, parts in load_layer(d, 'A5_PARKINGLOT'):
    poly = parts[0] if parts else []
    if poly and near(poly):
      c = (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
      sv, dist, off, _ = project(pts, s, c)
      if dist <= args.park_radius:
        found.append((sv, '주차면', f'{r["ID"]} 중심 ({c[0] - ox:.2f}, {c[1] - oy:.2f}) '
                      f'{"좌" if off > 0 else "우"} {dist:.1f}m', c))
  found.sort()

  print(f'\n{"s[m]":>7}  종류        내용')
  for sv, k, desc, _ in found:
    print(f'{sv:7.1f}  {k:<8}  {desc}')
  if not found:
    print('  (경로가 지나는 정지선·횡단보도·방지턱·신호등이 없다)')

  stops = [(sv, q) for sv, k, _, q in found if k == '정지선']
  lights = [sv for sv, k, _, _ in found if k == '신호등']
  out = [emit('crosswalk', sv, total, args.lead, args.tail) for sv, _ in stops]
  out += [emit('traffic_light', sv, total, args.lead, args.tail) for sv in lights]
  if out:
    print('\n' + '=' * 66)
    print('config/mission_plan.yaml 의 missions: 아래에 붙여넣을 것 (필요한 것만)')
    print('=' * 66)
    print('\n'.join(out))
    print('\n⚠ 정지선마다 crosswalk 조각이 나온다. 대회 미션이 아닌 정지선은 지울 것.')

  if args.stop_points:
    sp = [point_at(pts, s, max(0.0, sv - args.setback)) for sv, _ in stops]
    lines = ['# hdmap_features.py 로 생성 — 정밀지도 정지선(B2 530)과 경로의 교차점',
             f'# 경로: {args.waypoints}  setback {args.setback} m',
             'origin:', f'  x: {ox}', f'  y: {oy}', f'  epsg: {epsg}',
             '  site: 정밀지도 정지선', 'stop_points:']
    if sp:
      lines += [f'- x: {x - ox:.3f}\n  y: {y - oy:.3f}' for x, y in sp]
    else:
      lines[-1] = 'stop_points: []'
    with open(args.stop_points, 'w', encoding='utf-8') as f:
      f.write('\n'.join(lines) + '\n')
    print(f'\n정지점 {len(sp)}개 저장: {args.stop_points}')


if __name__ == '__main__':
  main()

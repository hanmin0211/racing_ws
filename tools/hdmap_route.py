#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hdmap_route.py — 정밀지도 차로 중심선(A2_LINK)으로 웨이포인트를 만든다.

★ 왜 필요한가
  지금까지 경로는 차를 몰고 다니며 GPS 로 기록했다. 기록하려면 현장에 가야 하고,
  사람이 몬 궤적이라 노이즈와 차선 치우침이 섞인다(smooth_path.py 가 그걸 편다).
  정밀지도에는 차로 중심선이 이미 cm 급으로 들어 있다. 출발·경유·도착 지점만
  주면 링크 연결(FromNode → ToNode)을 따라 최단 경로를 찾아 그대로 쓴다.

★ 한계 — 꼭 읽을 것
  · 차로 변경은 하지 않는다. 링크의 앞뒤 연결만 따라간다.
    다른 차로로 가야 하면 그 차로 위에 --via 를 찍을 것.
  · 지도는 2019년 구축본이다. 공사·도색 변경은 현장에서 확인할 것.
  · 지도 좌표와 우리 RTK 좌표의 일치는 아직 실측하지 않았다(교내 루프가
    중심선과 중앙값 0.5 m 로 맞는 정도까지만 확인). 첫 주행은 저속으로.

사용 (좌표는 기본이 로컬 — --origin 기준. 음수는 = 로 붙일 것):
  export HDMAP_DIR=".../교통대-정밀지도/교통대"

  # 이 근처에 어떤 링크가 있나
  python3 tools/hdmap_route.py --origin config/site_origin.chungju.yaml --near=-20,-6

  # 출발 → 경유 → 도착 (출발=도착이면 한 바퀴)
  python3 tools/hdmap_route.py --origin config/site_origin.chungju.yaml \\
      --from=-17,-4 --via=-30,-12 --to=-17,-4 -o /tmp/wp_hdmap.yaml

  # 링크 ID 를 직접 줄 때 (앞뒤로 이어져 있어야 한다)
  python3 tools/hdmap_route.py --origin ... --links A221BW001275,A221BW001279 -o ...
"""

import argparse
import heapq
import math
import os
import sys

from hdmap_common import (MAP_EPSG, cumlen, load_layer, load_origin, map_dir,
                          min_radius, project, resample, write_waypoints)

R_MIN = 2.42      # 최대 타각 18°, 축거 0.785 m 의 최소 회전반경 [m]
SNAP_MAX = 6.0    # 지점이 링크에서 이보다 멀면 거부 [m]


def load_links(d):
  links = {}
  for r, parts in load_layer(d, 'A2_LINK'):
    pts = [p for part in parts for p in part]
    if len(pts) >= 2:
      links[r['ID']] = {'row': r, 'pts': pts, 's': cumlen(pts)}
  succ = {}
  by_from = {}
  for lid, L in links.items():
    by_from.setdefault(L['row']['FromNodeID'], []).append(lid)
  for lid, L in links.items():
    succ[lid] = by_from.get(L['row']['ToNodeID'], [])
  return links, succ


def snap(links, q, n=1):
  """q 에 가까운 링크 n 개 → [(거리, id, s, 횡오프셋)]."""
  out = []
  for lid, L in links.items():
    xs = [p[0] for p in L['pts']]
    ys = [p[1] for p in L['pts']]
    if not (min(xs) - 30 < q[0] < max(xs) + 30 and
            min(ys) - 30 < q[1] < max(ys) + 30):
      continue
    s, dist, off, _ = project(L['pts'], L['s'], q)
    out.append((dist, lid, s, off))
  out.sort()
  return out[:n]


def cut(L, s0, s1):
  """링크 폴리라인의 s0~s1 구간."""
  pts, s = L['pts'], L['s']
  def at(v):
    for i in range(len(s) - 1):
      if s[i] <= v <= s[i + 1]:
        t = (v - s[i]) / max(s[i + 1] - s[i], 1e-9)
        return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t,
                pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t)
    return pts[-1] if v >= s[-1] else pts[0]
  inner = [p for p, v in zip(pts, s) if s0 < v < s1]
  return [at(s0)] + inner + [at(s1)]


def leg(links, succ, a, b):
  """(링크, s) a → b 의 링크 열. 같은 링크에서 앞으로 가면 그 링크 하나."""
  (la, sa), (lb, sb) = a, b
  if la == lb and sb >= sa:
    return [la]
  # Dijkstra — 비용은 링크 길이. a 의 후속 링크부터 시작한다.
  dist, prev, pq = {}, {}, []
  for n in succ[la]:
    c = links[la]['s'][-1] - sa + links[n]['s'][-1]
    if c < dist.get(n, float('inf')):
      dist[n], prev[n] = c, la
      heapq.heappush(pq, (c, n))
  while pq:
    c, u = heapq.heappop(pq)
    if u == lb:
      break
    if c > dist.get(u, float('inf')):
      continue
    for n in succ[u]:
      nc = c + links[n]['s'][-1]
      if nc < dist.get(n, float('inf')):
        dist[n], prev[n] = nc, u
        heapq.heappush(pq, (nc, n))
  if lb not in prev:
    raise SystemExit(f'❌ {la} → {lb} 로 이어지는 경로가 없다 '
                     '(일방 방향이 반대거나 차로 변경이 필요하다 — --via 를 '
                     '목표 차로 위에 찍어 볼 것)')
  # la == lb(같은 링크 뒤쪽으로 돌아오는 순환)도 있으니 prev[lb] 부터 거슬러 간다
  seq, u = [lb], prev[lb]
  while u != la:
    seq.append(u)
    u = prev[u]
  seq.append(la)
  return seq[::-1]


def build(links, seq, s_start, s_end):
  pts = []
  for i, lid in enumerate(seq):
    L = links[lid]
    s0 = s_start if i == 0 else 0.0
    s1 = s_end if i == len(seq) - 1 else L['s'][-1]
    for p in cut(L, s0, s1):
      if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 0.05:
        pts.append(p)
  return pts


def xy(arg):
  try:
    x, y = (float(v) for v in arg.split(','))
  except ValueError:
    raise argparse.ArgumentTypeError(f'"x,y" 형식이어야 한다: {arg}')
  return x, y


def main():
  ap = argparse.ArgumentParser(description='정밀지도 차로 중심선으로 웨이포인트 생성')
  ap.add_argument('--map', help='A2_LINK.shp 가 있는 폴더 (기본: $HDMAP_DIR)')
  ap.add_argument('--origin', default='config/site_origin.yaml',
                  help='로컬 원점 파일 (충주면 config/site_origin.chungju.yaml)')
  ap.add_argument('--utm', action='store_true', help='입력 좌표가 UTM 이다')
  ap.add_argument('--near', type=xy, help='이 지점 근처 링크만 보여준다')
  ap.add_argument('--from', dest='start', type=xy)
  ap.add_argument('--via', type=xy, action='append', default=[])
  ap.add_argument('--to', type=xy)
  ap.add_argument('--links', help='링크 ID 를 쉼표로 (앞뒤로 이어진 순서)')
  ap.add_argument('--step', type=float, default=0.5, help='리샘플 간격 [m]')
  ap.add_argument('-o', '--out', help='저장할 웨이포인트 YAML (없으면 미리보기만)')
  args = ap.parse_args()
  if not (math.isfinite(args.step) and args.step > 0):
    ap.error(f'--step 은 양수여야 한다: {args.step}')

  d = map_dir(args.map)
  ox, oy, epsg, site = load_origin(args.origin)
  if epsg != MAP_EPSG:
    raise SystemExit(f'❌ 원점이 EPSG:{epsg} 다 — 지도는 EPSG:{MAP_EPSG}')
  links, succ = load_links(d)
  xs = [p[0] for L in links.values() for p in L['pts']]
  ys = [p[1] for L in links.values() for p in L['pts']]
  if not (min(xs) - 3000 < ox < max(xs) + 3000 and
          min(ys) - 3000 < oy < max(ys) + 3000):
    raise SystemExit(f'❌ 원점 {args.origin} ({site}) 이 지도 범위에서 3 km 넘게 '
                     '떨어져 있다 — 충주면 --origin config/site_origin.chungju.yaml')
  to_utm = (lambda q: q) if args.utm else (lambda q: (q[0] + ox, q[1] + oy))
  print(f'지도: {d}  링크 {len(links)}개')
  print(f'원점: {args.origin} ({site})  ({ox}, {oy})')

  if args.near:
    print(f'\n({args.near[0]}, {args.near[1]}) 근처 링크:')
    for dist, lid, s, off in snap(links, to_utm(args.near), 8):
      r = links[lid]['row']
      print(f'  {lid}  {dist:5.2f}m ({"좌" if off > 0 else "우"})  '
            f's={s:6.1f}/{links[lid]["s"][-1]:.1f}m  '
            f'제한 {r["MaxSpeed"]}km/h  차로 {r["LaneNo"]}  '
            f'→ 후속 {",".join(succ[lid]) or "없음"}')
    return

  if args.links:
    seq = [v.strip() for v in args.links.split(',') if v.strip()]
    for v in seq:
      if v not in links:
        raise SystemExit(f'❌ 없는 링크 ID: {v}')
    for a, b in zip(seq, seq[1:]):
      if b not in succ[a]:
        raise SystemExit(f'❌ {a} 다음이 {b} 가 아니다 (후속: {succ[a]})')
    s_start, s_end = 0.0, links[seq[-1]]['s'][-1]
    marks = []
  elif args.start and args.to:
    marks = []
    for name, q in [('출발', args.start)] + [(f'경유{i + 1}', v) for i, v in
                                            enumerate(args.via)] + [('도착', args.to)]:
      dist, lid, s, _ = snap(links, to_utm(q))[0]
      flag = '✅' if dist <= 2.0 else ('⚠' if dist <= SNAP_MAX else '❌')
      print(f'  {name} ({q[0]}, {q[1]}) → {lid} s={s:.1f}  {dist:.2f}m {flag}')
      if dist > SNAP_MAX:
        raise SystemExit(f'❌ {name} 지점이 차로 중심선에서 {dist:.1f}m — '
                         '도로 위인지, 원점이 맞는지 확인할 것')
      marks.append((lid, s))
    seq = [marks[0][0]]
    qs = [to_utm(q) for q in [args.start] + args.via + [args.to]]
    for i, (a, b) in enumerate(zip(marks, marks[1:])):
      ls = leg(links, succ, a, b)
      seq += ls[1:]   # 첫 링크는 앞 구간의 끝과 같다
      # ★ 우회 감시: 링크 방향이 막혀 있으면 최단 경로가 동네를 한 바퀴 돈다
      #   (교내 주차장 통로가 지도엔 일방으로 그려져 있어 실제로 겪었다)
      run = cumlen(build(links, ls, a[1], b[1]))[-1]
      line = math.hypot(qs[i + 1][0] - qs[i][0], qs[i + 1][1] - qs[i][1])
      bad = run > 3 * line + 30
      print(f'  구간{i + 1}: 직선 {line:.1f}m → 경로 {run:.1f}m (링크 {len(ls)}개)'
            + ('  ⚠ 크게 돌아간다 — 지도상 그 방향 연결이 없다. '
               '링크 방향을 --near 로 확인할 것' if bad else ''))
    s_start, s_end = marks[0][1], marks[-1][1]
  else:
    ap.error('--near / --links / (--from, --to) 중 하나를 줄 것')

  utm = build(links, seq, s_start, s_end)
  pts = resample([(x - ox, y - oy) for x, y in utm], args.step)
  total = cumlen(pts)[-1]
  r, ri = min_radius(pts)
  print(f'\n링크 {len(seq)}개: {" → ".join(seq)}')
  speeds = sorted({links[v]["row"]["MaxSpeed"] for v in seq if links[v]["row"]["MaxSpeed"]})
  print(f'경로 {len(pts)}점 · {total:.1f}m · 지도 제한속도 {speeds} km/h')
  print(f'최소 회전반경 {r:.2f}m (idx{ri}, s≈{ri * args.step:.1f}m)  '
        + ('✅' if r >= R_MIN * 1.2 else
           ('⚠ 빠듯하다' if r >= R_MIN else
            f'❌ 차가 못 돈다(최소 {R_MIN}m) — smooth_path.py 를 거칠 것')))

  if args.out:
    header = ['hdmap_route.py 로 생성 — 정밀지도 A2_LINK 차로 중심선',
              f'지도: {os.path.basename(os.path.normpath(d))}  링크: {",".join(seq)}',
              f'명령: {" ".join(sys.argv[1:])}', '',
              '⚠ 지도(2019 구축)와 RTK 좌표 일치는 첫 주행에서 저속으로 확인할 것']
    write_waypoints(args.out, pts, ox, oy, epsg, f'{site} · 정밀지도 경로', header)
    print(f'\n저장: {args.out}')
  else:
    print('\n(미리보기만 — 저장하려면 -o 파일)')


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""split_path.py — 웨이포인트를 진행거리(s) 기준으로 잘라 여러 파일로 나눈다.

★ 왜 필요한가
  완주 경로 하나에 **후진이 필요한 구간**(주차·방향전환)이 섞이면 그 파일은
  통째로 못 쓴다. pure_pursuit 은 전진 전용이라 경로가 차 뒤로 이어지는 순간
  정지하고(local_pure_pursuit_node: `lookahead가 차량 뒤`), 되꺾인 두 가지가
  겹쳐 있어 최근접점 탐색도 왔다갔다 한다.

  규정(항목 6·8)도 주차는 "후진으로 차고 진입 → 전진으로 진입확인선 통과" 라
  웨이포인트로 표현할 수 없다. 그 구간은 parking_node 가 **그 자리에서**
  궤적을 만든다. 그래서 경로에서 **도려내고** 앞뒤만 남긴다.

  원점 스탬프는 그대로 물려준다(안 그러면 조각들이 어느 장소 것인지 모르게 된다).

사용:
  # 주차 구간(s 293~337)을 도려내고 앞뒤 두 조각으로
  python3 tools/split_path.py --in ~/wp_yongin_resampled_0.5.yaml \
      --cut 293 337 --out-prefix ~/wp_yongin --write

  # 여러 구간을 도려내려면 --cut 를 반복
  python3 tools/split_path.py --in a.yaml --cut 293 337 --cut 500 520 ...
"""

import argparse
import math
import os

import yaml


def load(path):
  d = yaml.safe_load(open(path, encoding='utf-8')) or {}
  pts = [(float(q['x']), float(q['y'])) for q in (d.get('waypoints') or [])]
  s = [0.0]
  for i in range(1, len(pts)):
    s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                pts[i][1] - pts[i - 1][1]))
  return d, pts, s


def save(path, pts, origin):
  out = {}
  if origin is not None:
    out['origin'] = origin
  out['waypoints'] = [{'x': float(x), 'y': float(y)} for x, y in pts]
  with open(path, 'w', encoding='utf-8') as f:
    yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False,
                   allow_unicode=True)


def seglen(pts):
  return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
             for i in range(len(pts) - 1))


def heading(pts, i):
  j = min(max(i, 0), len(pts) - 2)
  return math.degrees(math.atan2(pts[j + 1][1] - pts[j][1],
                                 pts[j + 1][0] - pts[j][0]))


def main():
  ap = argparse.ArgumentParser(
      description='웨이포인트를 s 구간 기준으로 잘라낸다')
  ap.add_argument('--in', dest='inp', required=True)
  ap.add_argument('--cut', nargs=2, type=float, action='append',
                  metavar=('S_FROM', 'S_TO'), required=True,
                  help='도려낼 구간 [m]. 여러 번 줄 수 있다')
  ap.add_argument('--out-prefix', default=None,
                  help='조각 파일 접두어 (기본: 입력파일 이름)')
  ap.add_argument('--min-len', type=float, default=5.0,
                  help='이보다 짧은 조각은 버린다 [m]')
  ap.add_argument('--write', action='store_true')
  args = ap.parse_args()

  d, pts, s = load(args.inp)
  if len(pts) < 2:
    raise SystemExit('❌ 점이 부족하다')
  origin = d.get('origin')
  prefix = args.out_prefix or os.path.splitext(args.inp)[0]

  print(f'입력: {args.inp}')
  print(f'  {len(pts)}점 · {s[-1]:.1f}m'
        + ('' if origin else '  ⚠ 원점 스탬프 없음'))
  cuts = sorted(args.cut)
  for a, b in cuts:
    print(f'  도려낼 구간: s {a:.1f} ~ {b:.1f}m  ({b - a:.1f}m)')

  # 남길 구간 계산
  keep, cur = [], 0.0
  for a, b in cuts:
    if a > cur:
      keep.append((cur, a))
    cur = max(cur, b)
  if cur < s[-1]:
    keep.append((cur, s[-1]))

  print()
  names = []
  for k, (a, b) in enumerate(keep):
    sub = [p for p, sv in zip(pts, s) if a <= sv <= b]
    if len(sub) < 2 or seglen(sub) < args.min_len:
      print(f'  조각 {k}: s {a:.1f}~{b:.1f}m — 너무 짧아 버린다')
      continue
    tag = chr(ord('A') + len(names))
    out = f'{prefix}_{tag}.yaml'
    names.append(out)
    print(f'  조각 {tag}: s {a:7.1f}~{b:7.1f}m  {len(sub):>5}점  {seglen(sub):7.1f}m')
    print(f'          시작 ({sub[0][0]:7.2f}, {sub[0][1]:7.2f}) 헤딩 '
          f'{heading(sub, 0):+7.1f}°')
    print(f'          종점 ({sub[-1][0]:7.2f}, {sub[-1][1]:7.2f}) 헤딩 '
          f'{heading(sub, len(sub) - 2):+7.1f}°')
    if args.write:
      save(out, sub, origin)
      print(f'          → {out}')
    print()

  if not args.write:
    print('(미리보기 — 저장하려면 --write)')
  else:
    print('저장 완료. 각 조각을 preflight 로 확인할 것:')
    for n in names:
      print(f'  python3 tools/preflight.py --waypoints {n} --skip-parking')


if __name__ == '__main__':
  main()

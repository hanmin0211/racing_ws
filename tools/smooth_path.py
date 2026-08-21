#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""smooth_path.py — 기록된 웨이포인트를 '차가 실제로 돌 수 있는' 경로로 다듬는다.

왜 필요한가 (2026-08-22):
  실차 주행에서 커브마다 차선을 밟고 이탈했다. 경로를 분석해 보니 최대 타각 18°
  (최소 회전반경 2.42m)로는 **물리적으로 못 도는 커브**가 있었다:
      idx355 R=1.73m(필요 24.4°), idx357 R=1.91m(22.4°) + 빠듯한 구간(R<3.5m) 16곳
  이런 곳은 제어가 완벽해도 반드시 이탈한다.

  또한 원본은 사람이 주행하며 GPS 로 기록한 것이라 수 cm 노이즈가 깔려 있다.
  (0.3m 로 촘촘히 리샘플하면 '못 도는 커브'가 2개→25개로 늘어나는 것이 그 증거다.
   경로가 실제로 급한 게 아니라 노이즈가 곡률로 드러나는 것이다)

방법: 제약 하의 반복 스무딩 (path smoothing with deviation constraint)
  각 점을 두 힘의 균형으로 옮긴다.
    · 원본 유지력(alpha): 원래 기록에서 멀어지지 않게 당긴다
    · 스무딩력(beta)   : 이웃의 중점 쪽으로 당겨 꺾임을 편다
  그리고 **원본에서 max_dev 이상 벗어나면 잘라낸다**(clamp).
  이 제약이 핵심이다 — 제약 없이 스무딩하면 코너를 잘라먹어 오히려 트랙을 벗어난다.

사용:
  python3 tools/smooth_path.py                      # 기본값으로 실행(미리보기만)
  python3 tools/smooth_path.py --write              # 결과를 파일로 저장
  python3 tools/smooth_path.py --max-dev 0.4 --target-r 3.0 --write
"""

import argparse
import math

import yaml

DEF_IN = '/home/han/racing_ws/src/pure_pursuit_pkg/config/waypoints_recorded.yaml'
DEF_OUT = '/home/han/racing_ws/src/pure_pursuit_pkg/config/waypoints_smoothed_0.5.yaml'


def load(path):
  d = yaml.safe_load(open(path))
  return [(float(p['x']), float(p['y'])) for p in d['waypoints']]


def save(path, pts):
  d = {'waypoints': [{'x': float(x), 'y': float(y)} for x, y in pts]}
  with open(path, 'w') as f:
    yaml.safe_dump(d, f, default_flow_style=False, sort_keys=False)


def resample(pts, step):
  """등간격 재추출 (선분 위 선형보간)."""
  out = [pts[0]]
  acc = 0.0
  for i in range(len(pts) - 1):
    x1, y1 = pts[i]
    x2, y2 = pts[i + 1]
    seg = math.hypot(x2 - x1, y2 - y1)
    if seg < 1e-9:
      continue
    t = 0.0
    while acc + seg - t >= step:
      t += step - acc
      r = t / seg
      out.append((x1 + (x2 - x1) * r, y1 + (y2 - y1) * r))
      acc = 0.0
    acc += seg - t
  if out[-1] != pts[-1]:
    out.append(pts[-1])
  return out


def radii(w):
  """연속 3점의 외접원 반경 목록 [(R, idx), ...]."""
  out = []
  for i in range(1, len(w) - 1):
    (x1, y1), (x2, y2), (x3, y3) = w[i - 1], w[i], w[i + 1]
    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    s = (a + b + c) / 2.0
    ar2 = s * (s - a) * (s - b) * (s - c)
    if ar2 <= 1e-12:
      continue
    out.append(((a * b * c) / (4.0 * math.sqrt(ar2)), i))
  return out


def report(w, label, min_r):
  rs = sorted(radii(w))
  if not rs:
    print(f'{label}: 곡률 계산 불가')
    return
  bad = [r for r, _ in rs if r < min_r]
  warn = [r for r, _ in rs if min_r <= r < min_r * 1.45]
  length = sum(math.hypot(w[i + 1][0] - w[i][0], w[i + 1][1] - w[i][1])
               for i in range(len(w) - 1))
  print(f'{label}')
  print(f'   점 {len(w)}개  길이 {length:.1f}m  최소R={rs[0][0]:.2f}m')
  print(f'   R<{min_r}m(못 돔) {len(bad)}개   ~{min_r*1.45:.1f}m(빠듯) {len(warn)}개')
  return rs


def smooth(pts, alpha, beta, max_dev, iters):
  """원본에서 max_dev 이내로만 움직이며 반복 스무딩. 양 끝점은 고정."""
  orig = [tuple(p) for p in pts]
  cur = [list(p) for p in pts]
  for _ in range(iters):
    for i in range(1, len(cur) - 1):
      for k in (0, 1):
        # 원본으로 당기는 힘 + 이웃 중점으로 당기는 힘
        cur[i][k] += (alpha * (orig[i][k] - cur[i][k]) +
                      beta * (cur[i - 1][k] + cur[i + 1][k] - 2.0 * cur[i][k]))
      # ★ 이탈 제약: 원본에서 max_dev 를 넘으면 그 방향으로 잘라낸다
      dx = cur[i][0] - orig[i][0]
      dy = cur[i][1] - orig[i][1]
      d = math.hypot(dx, dy)
      if d > max_dev:
        s = max_dev / d
        cur[i][0] = orig[i][0] + dx * s
        cur[i][1] = orig[i][1] + dy * s
  return [(p[0], p[1]) for p in cur]


def max_deviation(a, b):
  """같은 인덱스끼리의 최대/평균 이동량."""
  ds = [math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(a, b)]
  return max(ds), sum(ds) / len(ds)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--in', dest='inp', default=DEF_IN)
  ap.add_argument('--out', default=DEF_OUT)
  ap.add_argument('--step', type=float, default=0.5, help='리샘플 간격(m)')
  ap.add_argument('--max-dev', type=float, default=0.30,
                  help='원본에서 벗어날 수 있는 최대 거리(m)')
  ap.add_argument('--target-r', type=float, default=2.8,
                  help='목표 최소 회전반경(m). 차량 한계 2.42m + 여유')
  ap.add_argument('--alpha', type=float, default=0.25, help='원본 유지력')
  ap.add_argument('--beta', type=float, default=0.35, help='스무딩력')
  ap.add_argument('--iters', type=int, default=400)
  ap.add_argument('--write', action='store_true', help='파일로 저장')
  args = ap.parse_args()

  raw = load(args.inp)
  print(f'원본: {args.inp}  ({len(raw)}점)\n')

  before = resample(raw, args.step)
  report(before, f'=== 스무딩 전 ({args.step}m 리샘플) ===', args.target_r)
  print()

  sm = smooth(raw, args.alpha, args.beta, args.max_dev, args.iters)
  after = resample(sm, args.step)
  report(after, f'=== 스무딩 후 ({args.step}m 리샘플) ===', args.target_r)

  mx, avg = max_deviation(raw, sm)
  print()
  print(f'원본 대비 이동량:  최대 {mx:.2f}m   평균 {avg:.2f}m   (제한 {args.max_dev}m)')

  rs = sorted(radii(after))
  bad = [(r, i) for r, i in rs if r < args.target_r]
  if bad:
    print(f'\n★ 아직 목표({args.target_r}m) 미달 구간 {len(bad)}개:')
    for r, i in bad[:6]:
      need = math.degrees(math.atan(0.785 / r))
      print(f'    idx{i:4d}  R={r:5.2f}m  필요타각 {need:4.1f}도')
    print('  → --max-dev 를 키우거나 --beta 를 올려 더 다듬을 수 있다.')
  else:
    print(f'\n✅ 전 구간 R >= {args.target_r}m — 최대 타각 18도로 모두 통과 가능')

  if args.write:
    save(args.out, after)
    print(f'\n저장: {args.out}')
  else:
    print('\n(미리보기만 — 저장하려면 --write)')


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_resample.py
====================
웨이포인트를 호길이(거리) 기준으로 균일 간격 리샘플링하는 유틸.

레코더는 대략 1m 간격으로 점을 찍는데, 실제 주행 속도/GPS 주기 때문에 간격이
들쭉날쭉하다. 이 모듈은 경로를 따라 누적 호길이를 계산한 뒤, 0.5m 같은 균일
간격으로 선형보간해서 점을 다시 뽑는다. (곡선 피팅은 하류의
local_sliding_window_node가 3차 다항식으로 하므로 여기선 선형보간으로 충분.)

독립 실행:
  ros2 run waypoint_follower resample_waypoints \
      --input  .../waypoints_recorded.yaml \
      --output .../waypoints_resampled_0.5.yaml \
      --spacing 0.5
"""

import argparse

import numpy as np
import yaml


def load_waypoints(path):
  data = yaml.safe_load(open(path, 'r'))
  raw = data.get('waypoints', data.get('poses', list(data.values())[0]))
  pts = []
  for p in raw:
    if isinstance(p, dict):
      pts.append([float(p['x']), float(p['y'])])
    else:
      pts.append([float(p[0]), float(p[1])])
  return np.array(pts, dtype=float)


def resample(points, spacing, closed=False):
  """points(N x 2)를 호길이 spacing[m] 균일 간격으로 선형보간 리샘플.

  closed=True면 마지막→처음 구간도 이어 붙여 폐루프로 리샘플한다."""
  pts = np.asarray(points, dtype=float)
  if len(pts) < 2:
    return pts
  if closed and not np.allclose(pts[0], pts[-1]):
    pts = np.vstack([pts, pts[0]])

  # 누적 호길이
  seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
  s = np.concatenate(([0.0], np.cumsum(seg)))
  total = float(s[-1])
  if total < spacing:
    return pts

  # 목표 거리: 0, spacing, 2*spacing, ... <= total
  n = int(np.floor(total / spacing))
  targets = np.arange(0, n + 1) * spacing
  xs = np.interp(targets, s, pts[:, 0])
  ys = np.interp(targets, s, pts[:, 1])
  return np.column_stack([xs, ys])


def save_waypoints(points, path, origin=None):
  """웨이포인트 저장. origin 을 주면 원점 스탬프를 함께 남긴다.

  ★ 원점을 같이 저장하는 이유
    로컬좌표는 (UTM − 원점)이라 원점 없이는 해석이 불가능하다. 스탬프가
    없으면 장소가 바뀐 뒤 옛 파일이 조용히 150km 어긋난 경로로 읽힌다.
    global_path_publisher 가 이 값을 보고 환산·거부한다.
  """
  data = {}
  if origin is not None:
    data['origin'] = origin
  data['waypoints'] = [{'x': float(x), 'y': float(y)} for x, y in points]
  with open(path, 'w') as f:
    yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False,
                   allow_unicode=True)


def main():
  ap = argparse.ArgumentParser(description='웨이포인트 균일간격 리샘플링')
  ap.add_argument('--input', required=True, help='입력 YAML')
  ap.add_argument('--output', required=True, help='출력 YAML')
  ap.add_argument('--spacing', type=float, default=0.5, help='간격[m] (기본 0.5)')
  ap.add_argument('--closed', action='store_true', help='폐루프로 리샘플')
  args = ap.parse_args()

  pts = load_waypoints(args.input)
  out = resample(pts, args.spacing, closed=args.closed)
  # 입력의 원점 스탬프를 그대로 물려준다(있으면).
  src_origin = None
  try:
    d = yaml.safe_load(open(args.input, 'r')) or {}
    if isinstance(d.get('origin'), dict):
      src_origin = d['origin']
  except Exception:  # noqa: BLE001
    pass
  save_waypoints(out, args.output, origin=src_origin)
  total = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
  print(f'리샘플 완료: {len(pts)}점({total:.1f}m) → {len(out)}점 '
        f'@ {args.spacing}m 간격 → {args.output}')


if __name__ == '__main__':
  main()

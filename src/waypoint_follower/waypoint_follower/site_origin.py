#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""site_origin.py — 로컬 좌표계 원점을 config/site_origin.yaml 에서 읽는다.

원점이 여러 파일에 하드코딩돼 있으면 장소가 바뀔 때 하나만 놓쳐도 150km 어긋난다
(2026-08-17 실제 발생). 그래서 값은 YAML 한 곳에만 두고 모두 여기로 읽는다.

패키지 간 의존을 늘리지 않으려고 워크스페이스 고정 경로를 본다. 파일이 없거나
깨져 있어도 노드가 죽지 않도록 폴백을 두되, 폴백이 쓰이면 반드시 경고한다.
"""

import os

SITE_YAML = '/home/han/racing_ws/config/site_origin.yaml'

# 폴백 — YAML 을 못 읽을 때만 쓰인다 (충주 구 원점).
FALLBACK = {'utm_epsg': 32652, 'origin_x': 399848.522, 'origin_y': 4092209.171}


def load_site_origin(logger=None):
  """(utm_epsg, origin_x, origin_y, source) 반환. source 는 사람이 읽을 설명."""
  try:
    import yaml
    with open(SITE_YAML) as f:
      d = yaml.safe_load(f) or {}
    epsg = int(d['utm_epsg'])
    ox = float(d['origin_x'])
    oy = float(d['origin_y'])
    src = d.get('site', SITE_YAML)
    return epsg, ox, oy, str(src)
  except Exception as e:  # noqa: BLE001
    msg = (f'⚠ {SITE_YAML} 을 읽지 못해 폴백 원점을 쓴다 ({e}). '
           '장소가 다르면 좌표가 크게 어긋난다!')
    if logger is not None:
      logger.warn(msg)
    else:
      print(msg)
    return (FALLBACK['utm_epsg'], FALLBACK['origin_x'], FALLBACK['origin_y'],
            'FALLBACK(충주 구 원점)')


def declare_and_get(node):
  """ROS 노드용 헬퍼: YAML 값을 기본값으로 파라미터를 선언하고 최종값을 돌려준다.

  CLI 로 -p origin_x:=... 를 주면 그쪽이 우선한다(현장 임시 대응용).
  """
  epsg, ox, oy, src = load_site_origin(node.get_logger())
  node.declare_parameter('utm_epsg', epsg)
  node.declare_parameter('origin_x', ox)
  node.declare_parameter('origin_y', oy)
  epsg = int(node.get_parameter('utm_epsg').value)
  ox = float(node.get_parameter('origin_x').value)
  oy = float(node.get_parameter('origin_y').value)
  node.get_logger().info(
      f'로컬 원점: {ox:.1f}, {oy:.1f} (EPSG:{epsg}) — {src}')
  return epsg, ox, oy


if __name__ == '__main__':
  e, x, y, s = load_site_origin()
  print(f'EPSG:{e}  origin=({x}, {y})  site={s}')
  print(f'파일: {SITE_YAML}  존재={os.path.exists(SITE_YAML)}')

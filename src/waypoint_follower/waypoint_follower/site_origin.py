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



# ---------------------------------------------------------------------------
# 원점 스탬프 — 좌표 파일이 "어느 원점에서 찍혔는지" 스스로 말하게 한다
# ---------------------------------------------------------------------------
#
# ★ 왜 필요한가
#   로컬좌표는 (UTM − 원점) 이라 원점을 모르면 해석이 불가능하다. 그런데
#   웨이포인트 YAML 에는 원점이 안 적혀 있었다. 장소가 바뀌어 원점을 갱신하면
#   옛 파일이 **조용히** 150km 어긋난 경로로 읽힌다(2026-08-17 실사고와 동일 계열).
#   stop_point_recorder / parking_pose_recorder 는 이미 원점을 같이 저장한다.
#   여기서 그 두 포맷을 모두 읽는 공통 함수를 두고, 웨이포인트도 같게 만든다.
#
#   포맷 A (stop_point_recorder):  origin_x: 477800.0
#                                  origin_y: 3964400.0
#   포맷 B (parking_pose_recorder): origin: {x: ..., y: ..., epsg: ..., site: ...}
#
#   두 포맷을 모두 지원한다. 새로 쓰는 건 포맷 B(정보가 더 많다).


def read_origin_stamp(d):
  """YAML dict 에서 원점 스탬프를 꺼낸다.

  Returns:
    (ox, oy, epsg, site) — 스탬프가 없으면 (None, None, None, None).
  """
  if not isinstance(d, dict):
    return (None, None, None, None)
  o = d.get('origin')
  if isinstance(o, dict) and 'x' in o and 'y' in o:
    try:
      return (float(o['x']), float(o['y']),
              int(o['epsg']) if o.get('epsg') is not None else None,
              o.get('site'))
    except (TypeError, ValueError):
      return (None, None, None, None)
  if d.get('origin_x') is not None and d.get('origin_y') is not None:
    try:
      return (float(d['origin_x']), float(d['origin_y']),
              int(d['utm_epsg']) if d.get('utm_epsg') is not None else None,
              d.get('site'))
    except (TypeError, ValueError):
      return (None, None, None, None)
  return (None, None, None, None)


def origin_stamp(epsg, ox, oy, site):
  """새로 저장할 파일에 넣을 원점 블록(포맷 B)."""
  return {'x': float(ox), 'y': float(oy), 'epsg': int(epsg),
          'site': str(site)}


def origin_delta(file_ox, file_oy, cur_ox, cur_oy):
  """파일 원점 → 현재 원점 변환량. 파일의 로컬좌표에 **더하면** 현재 좌표계 값.

  local_file = utm − file_o,  local_cur = utm − cur_o
  ⇒ local_cur = local_file + (file_o − cur_o)
  (bringup.load_stop_points 가 쓰는 규칙과 동일하다.)
  """
  return (float(file_ox) - float(cur_ox), float(file_oy) - float(cur_oy))


def reconcile_origin(d, path='(파일)', logger=None):
  """파일의 원점 스탬프를 현재 site_origin 과 대조해 (dx, dy, note) 를 낸다.

  로컬좌표에 (dx, dy) 를 더하면 현재 원점 기준 좌표가 된다.
  스탬프가 없으면 (0, 0) 을 주되 **반드시 경고한다** — 조용히 넘어가는 것이
  바로 150km 사고의 형태였다.
  """
  def _say(msg, warn=False):
    if logger is not None:
      (logger.warn if warn else logger.info)(msg)
    else:
      print(msg, flush=True)

  cur_epsg, cur_ox, cur_oy, cur_site = load_site_origin(logger)
  f_ox, f_oy, f_epsg, f_site = read_origin_stamp(d)

  if f_ox is None:
    _say(f'⚠ {path} 에 원점 기록이 없다(구버전 파일). '
         f'현재 원점({cur_ox:.0f}, {cur_oy:.0f} — {cur_site}) 기준으로 그냥 '
         '해석한다. 다른 장소에서 찍은 파일이면 좌표가 통째로 어긋난다.', warn=True)
    return (0.0, 0.0, 'no-stamp')

  if f_epsg is not None and cur_epsg is not None and f_epsg != cur_epsg:
    _say(f'❌ {path} 의 UTM 대역(EPSG:{f_epsg})이 현재(EPSG:{cur_epsg})와 다르다. '
         '환산할 수 없다 — 이 파일은 이 장소 것이 아니다.', warn=True)
    return (0.0, 0.0, 'epsg-mismatch')

  dx, dy = origin_delta(f_ox, f_oy, cur_ox, cur_oy)
  if abs(dx) < 0.01 and abs(dy) < 0.01:
    return (0.0, 0.0, 'match')

  shift = (dx * dx + dy * dy) ** 0.5
  _say(f'{path} 원점 환산: ({f_ox:.0f}, {f_oy:.0f} — {f_site}) → '
       f'({cur_ox:.0f}, {cur_oy:.0f} — {cur_site})  Δ=({dx:+.1f}, {dy:+.1f})',
       warn=shift > 1000.0)
  if shift > 1000.0:
    _say(f'❌ 원점 차이가 {shift / 1000:.1f}km 다. 이 파일은 **다른 장소**에서 '
         '기록된 것이다. 환산해도 좌표는 현 위치에서 그만큼 떨어진다 — '
         '이 장소에서 다시 기록할 것.', warn=True)
  return (dx, dy, 'shifted')


if __name__ == '__main__':
  e, x, y, s = load_site_origin()
  print(f'EPSG:{e}  origin=({x}, {y})  site={s}')
  print(f'파일: {SITE_YAML}  존재={os.path.exists(SITE_YAML)}')

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hdmap_common.py — 정밀도로지도(NGII v2 shp) 공용 읽기 도구.

★ 왜 필요한가
  교통대 정밀지도는 국토지리정보원 '정밀도로지도 v2' 형식의 shapefile 묶음이다
  (A2_LINK 차로 중심선, B2 정지선, B3 횡단보도 …). hdmap_route.py 와
  hdmap_features.py 가 같이 쓰는 부분을 여기 모았다.

  pyshp/geopandas 없이 **표준 라이브러리만** 쓴다. 노트북(Windows)에도,
  차량 PC(Ubuntu)에도 추가 설치가 필요 없게 하려는 것이다.
  PyYAML 이 있으면 쓰고, 없으면 우리 파일 형식만 읽는 최소 파서로 대신한다.

★ 좌표
  지도 원본(최상위 *.shp)은 EPSG:32652(UTM 52N) — 저장소 좌표계와 같다.
  epsg_4326/ 하위 위경도판은 쓰지 않는다.
"""

import math
import os
import re
import struct

try:
  import yaml
except ImportError:  # Windows 노트북엔 없다
  yaml = None

MAP_EPSG = 32652
ENV_MAP = 'HDMAP_DIR'


# ── shapefile ─────────────────────────────────────────────────────────

def _read_dbf(path, enc):
  with open(path, 'rb') as f:
    hdr = f.read(32)
    nrec, hlen, rlen = struct.unpack('<IHH', hdr[4:12])
    fields = []
    while True:
      b = f.read(32)
      if not b or b[0] == 0x0D:
        break
      name = b[:11].split(b'\0')[0].decode('ascii', 'replace')
      fields.append((name, chr(b[11]), b[16], b[17]))
    f.seek(hlen)
    rows = []
    for _ in range(nrec):
      r = f.read(rlen)
      if len(r) < rlen:
        break
      pos, d = 1, {}
      for name, t, ln, dec in fields:
        s = r[pos:pos + ln].decode(enc, 'replace').strip()
        pos += ln
        if t in 'NF':
          try:
            s = float(s) if (dec or '.' in s) else int(s)
          except ValueError:
            s = None
        d[name] = s
      rows.append(d)
  return rows


def _read_shp(path):
  """→ 레코드마다 [part, ...], part = [(x, y), ...]. z 는 버린다(2D 로 충분)."""
  with open(path, 'rb') as f:
    data = f.read()
  pos, shapes = 100, []
  while pos + 8 <= len(data):
    clen = struct.unpack('>i', data[pos + 4:pos + 8])[0]
    pos += 8
    rec = data[pos:pos + clen * 2]
    pos += clen * 2
    t = struct.unpack('<i', rec[:4])[0]
    if t == 0:
      shapes.append([])
    elif t in (1, 11, 21):
      shapes.append([[struct.unpack('<2d', rec[4:20])]])
    else:  # polyline / polygon (3, 5, 13, 15, 23, 25)
      npart, npt = struct.unpack('<2i', rec[36:44])
      idx = list(struct.unpack(f'<{npart}i', rec[44:44 + 4 * npart])) + [npt]
      o = 44 + 4 * npart
      xy = struct.unpack(f'<{2 * npt}d', rec[o:o + 16 * npt])
      pts = [(xy[2 * i], xy[2 * i + 1]) for i in range(npt)]
      shapes.append([pts[idx[i]:idx[i + 1]] for i in range(npart)])
  return shapes


def map_dir(arg):
  """--map 인자 → 실제 폴더. 없으면 환경변수 HDMAP_DIR."""
  d = arg or os.environ.get(ENV_MAP)
  if not d:
    raise SystemExit(f'❌ 정밀지도 폴더를 --map 또는 {ENV_MAP} 로 줄 것 '
                     '(A2_LINK.shp 가 있는 폴더)')
  if not os.path.exists(os.path.join(d, 'A2_LINK.shp')):
    raise SystemExit(f'❌ {d} 에 A2_LINK.shp 가 없다 — 최상위 UTM 폴더를 줄 것 '
                     '(epsg_4326 하위 폴더 아님)')
  return d


def load_layer(d, name):
  """레이어 → [(row dict, [part, ...]), ...]. 없는 레이어면 빈 목록."""
  base = os.path.join(d, name)
  if not os.path.exists(base + '.shp'):
    return []
  prj = base + '.prj'
  if os.path.exists(prj) and 'UTM_Zone_52N' not in open(prj).read():
    raise SystemExit(f'❌ {name}.prj 가 UTM 52N 이 아니다 — 좌표계를 확인할 것')
  enc = 'cp949'
  if os.path.exists(base + '.cpg'):
    c = open(base + '.cpg').read().strip().lower()
    enc = 'utf-8' if c in ('utf-8', 'utf8') else 'cp949'
  rows = _read_dbf(base + '.dbf', enc)
  shapes = _read_shp(base + '.shp')
  return list(zip(rows, shapes))


# ── 원점 · 웨이포인트 YAML ─────────────────────────────────────────────

def _num(t, key):
  m = re.search(rf'^\s*{key}:\s*([-\d.eE]+)', t, re.M)
  return float(m.group(1)) if m else None


def load_origin(path):
  """config/site_origin*.yaml → (ox, oy, epsg, site)."""
  t = open(path, encoding='utf-8').read()
  ox, oy = _num(t, 'origin_x'), _num(t, 'origin_y')
  if ox is None or oy is None:
    raise SystemExit(f'❌ {path} 에 origin_x/origin_y 가 없다')
  epsg = int(_num(t, 'utm_epsg') or MAP_EPSG)
  m = re.search(r'^site:\s*"?([^"\n]*)"?', t, re.M)
  return ox, oy, epsg, (m.group(1).strip() if m else '')


def load_waypoints(path):
  """웨이포인트 YAML → (pts 로컬, (ox, oy, epsg)). 원점 스탬프가 없으면 None."""
  if yaml:
    d = yaml.safe_load(open(path, encoding='utf-8')) or {}
    pts = [(float(p['x']), float(p['y'])) for p in d.get('waypoints') or []]
    o = d.get('origin') if isinstance(d.get('origin'), dict) else None
    org = (float(o['x']), float(o['y']), int(o.get('epsg') or MAP_EPSG)) if o else None
    return pts, org
  t = open(path, encoding='utf-8').read()
  m = re.search(r'^origin:\s*\n\s+x:\s*([-\d.eE]+)\s*\n\s+y:\s*([-\d.eE]+)'
                r'(?:\s*\n\s+epsg:\s*(\d+))?', t, re.M)
  org = (float(m.group(1)), float(m.group(2)),
         int(m.group(3) or MAP_EPSG)) if m else None
  wp = t[t.index('waypoints:'):] if 'waypoints:' in t else ''
  pts = [(float(a), float(b)) for a, b in
         re.findall(r'-\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)', wp)]
  return pts, org


def write_waypoints(path, pts, ox, oy, epsg, site, header):
  """저장소 웨이포인트 형식(포맷 B 원점 스탬프)으로 쓴다."""
  out = [f'# {ln}' if ln else '#' for ln in header]
  out += ['origin:', f'  x: {ox}', f'  y: {oy}', f'  epsg: {epsg}',
          f'  site: {site}', 'waypoints:']
  for x, y in pts:
    out += [f'- x: {x:.4f}', f'  y: {y:.4f}']
  with open(path, 'w', encoding='utf-8') as f:
    f.write('\n'.join(out) + '\n')


# ── 기하 ──────────────────────────────────────────────────────────────

def cumlen(pts):
  s = [0.0]
  for a, b in zip(pts, pts[1:]):
    s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
  return s


def project(pts, s, q):
  """q 를 폴리라인에 내린 발 → (s, 거리, 부호있는 횡오프셋[+좌], idx)."""
  best = (float('inf'), 0.0, 0.0, 0)
  for i in range(len(pts) - 1):
    ax, ay = pts[i]
    vx, vy = pts[i + 1][0] - ax, pts[i + 1][1] - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-12:
      continue
    t = max(0.0, min(1.0, ((q[0] - ax) * vx + (q[1] - ay) * vy) / L2))
    px, py = ax + vx * t, ay + vy * t
    d = math.hypot(q[0] - px, q[1] - py)
    if d < best[0]:
      side = vx * (q[1] - ay) - vy * (q[0] - ax)
      best = (d, s[i] + math.sqrt(L2) * t, math.copysign(d, side), i)
  return best[1], best[0], best[2], best[3]


def resample(pts, step):
  """등간격 재추출 (선분 위 선형보간). 끝점은 항상 넣는다."""
  out, acc = [pts[0]], 0.0
  for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
    seg = math.hypot(x2 - x1, y2 - y1)
    if seg < 1e-9:
      continue
    d = step - acc
    while d <= seg:
      out.append((x1 + (x2 - x1) * d / seg, y1 + (y2 - y1) * d / seg))
      d += step
    acc = seg - (d - step)
  gap = math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1])
  if gap > step * 0.3 or len(out) == 1:
    out.append(pts[-1])
  elif gap > 1e-6:
    out[-1] = pts[-1]   # 아주 짧은 끝 선분은 곡률을 튀게 하니 마지막 점을 옮긴다
  return out


def seg_cross(p1, p2, q1, q2):
  """선분 p1p2 와 q1q2 의 교점에서 p 쪽 비율 t (0~1). 안 만나면 None."""
  rx, ry = p2[0] - p1[0], p2[1] - p1[1]
  sx, sy = q2[0] - q1[0], q2[1] - q1[1]
  den = rx * sy - ry * sx
  if abs(den) < 1e-12:
    return None
  qpx, qpy = q1[0] - p1[0], q1[1] - p1[1]
  t = (qpx * sy - qpy * sx) / den
  u = (qpx * ry - qpy * rx) / den
  return t if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0 else None


def in_polygon(q, poly):
  x, y, c = q[0], q[1], False
  for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
    if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
      c = not c
  return c


def min_radius(pts, span=2):
  """i-span, i, i+span 세 점 외접원 반지름의 최소값과 그 위치."""
  best = (float('inf'), 0)
  for i in range(span, len(pts) - span):
    a, b, c = pts[i - span], pts[i], pts[i + span]
    ab = math.hypot(b[0] - a[0], b[1] - a[1])
    bc = math.hypot(c[0] - b[0], c[1] - b[1])
    ca = math.hypot(a[0] - c[0], a[1] - c[1])
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if cross > 1e-9:
      r = ab * bc * ca / (2 * cross)
      if r < best[0]:
        best = (r, i)
  return best

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""list_waypoints.py — 이 컴퓨터에 있는 모든 웨이포인트 파일을 찾아 한눈에 보여준다.

왜 필요한가: 트랙을 여러 곳에서 찍다 보면 파일이 여기저기 흩어지고,
로컬좌표(x,y)만 있는 파일은 열어봐도 '어디 트랙인지' 알 수가 없다.
이 도구는 site_origin.yaml 의 원점을 적용해 실제 위경도를 역산하고,
길이·크기·곡률까지 뽑아 표로 보여준다.

사용:
  python3 tools/list_waypoints.py              # 표로 목록
  python3 tools/list_waypoints.py --plot       # 전부 그림으로 그려 PNG 저장
  python3 tools/list_waypoints.py --dir ~/다른경로   # 검색 위치 추가
"""
import argparse, math, os, sys, yaml

try:
    from pyproj import Transformer
    INV = Transformer.from_crs('EPSG:32652', 'EPSG:4326', always_xy=True)
    FWD = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
except Exception:
    INV = FWD = None

# 알려진 원점 (site_origin.yaml 이 정본이지만, 과거 트랙 판별용으로 후보를 둔다)
KNOWN = [
    ('한국교통대(충주)', 399848.522, 4092209.171, 36.968, 36.976, 127.866, 127.880),
    ('대구권 시험장',     477800.0,   3964400.0,  35.80,  35.85,  128.73,  128.78),
]

def load_points(path):
    """→ (로컬좌표 리스트, 위경도 리스트 or None)"""
    d = yaml.safe_load(open(path, encoding='utf-8', errors='ignore'))
    w = d.get('waypoints') if isinstance(d, dict) else (d if isinstance(d, list) else None)
    if not w or not isinstance(w, list) or not isinstance(w[0], dict):
        return None, None
    k = w[0]
    if 'latitude' in k or 'lat' in k:
        la = 'latitude' if 'latitude' in k else 'lat'
        lo = 'longitude' if 'longitude' in k else 'lon'
        ll = [(float(p[la]), float(p[lo])) for p in w if la in p and lo in p]
        if FWD:
            utm = [FWD.transform(b, a) for a, b in ll]
            x0, y0 = utm[0]
            return [(a - x0, b - y0) for a, b in utm], ll
        return None, ll
    if 'x' in k and 'y' in k:
        return [(float(p['x']), float(p['y'])) for p in w if 'x' in p and 'y' in p], None
    return None, None

def stats(pts):
    xs = [a for a, _ in pts]; ys = [b for _, b in pts]
    L = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1))
    gap = math.hypot(pts[0][0]-pts[-1][0], pts[0][1]-pts[-1][1])
    radii = []
    for i in range(1, len(pts)-1):
        (x1,y1),(x2,y2),(x3,y3) = pts[i-1], pts[i], pts[i+1]
        a = math.hypot(x2-x1, y2-y1); b = math.hypot(x3-x2, y3-y2); c = math.hypot(x3-x1, y3-y1)
        s = (a+b+c)/2; ar2 = s*(s-a)*(s-b)*(s-c)
        if ar2 > 1e-12:
            radii.append((a*b*c)/(4*math.sqrt(ar2)))
    return L, max(xs)-min(xs), max(ys)-min(ys), gap, (min(radii) if radii else 0)

def guess_site(pts, latlon):
    """실제 위치 추정. 위경도가 있으면 그대로, 없으면 알려진 원점을 대입해본다."""
    if latlon:
        lat, lon = latlon[0]
        for nm, _, _, la0, la1, lo0, lo1 in KNOWN:
            if la0 <= lat <= la1 and lo0 <= lon <= lo1:
                return f'{nm} ({lat:.5f}N,{lon:.5f}E)'
        return f'{lat:.5f}N,{lon:.5f}E'
    if not INV:
        return '(원점미상)'
    hits = []
    for nm, ox, oy, la0, la1, lo0, lo1 in KNOWN:
        lon, lat = INV.transform(ox + pts[0][0], oy + pts[0][1])
        if la0 <= lat <= la1 and lo0 <= lon <= lo1:
            hits.append(nm)
    if len(hits) == 1:
        return f'{hits[0]} 원점이면 일치'
    return '(로컬좌표 — 원점에 따라 달라짐)'

def find_files(dirs):
    out = set()
    skip = ('/build/', '/install/', '/.git/', '/log/', '/.cache/', '/node_modules/', '/.local/lib', '/site-packages/')
    for root_dir in dirs:
        for root, _, files in os.walk(os.path.expanduser(root_dir)):
            if any(s in root for s in skip):
                continue
            for fn in files:
                if fn.lower().endswith(('.yaml', '.yml')) and \
                   any(t in fn.lower() for t in ('waypoint', 'path', 'track')):
                    out.add(os.path.join(root, fn))
    return sorted(out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', action='append', default=None, help='검색 디렉터리(여러 번 지정 가능)')
    ap.add_argument('--plot', action='store_true', help='PNG 로 그려 저장')
    ap.add_argument('--out', default='/tmp/waypoints_all.png')
    a = ap.parse_args()
    dirs = a.dir or ['~']

    # 현재 설정 표시
    cur = None
    for p in ('config/site_origin.yaml', os.path.expanduser('~/racing_ws/config/site_origin.yaml')):
        if os.path.exists(p):
            og = yaml.safe_load(open(p, encoding='utf-8'))
            cur = (og.get('site'), float(og['origin_x']), float(og['origin_y']))
            break
    if cur:
        print(f'현재 원점 설정: {cur[0]}  ({cur[1]}, {cur[2]})\n')

    rows = []
    for f in find_files(dirs):
        try:
            if os.path.getsize(f) < 100:
                continue
            pts, ll = load_points(f)
            if not pts or len(pts) < 3:
                continue
            L, W, H, gap, minR = stats(pts)
            rows.append((f, len(pts), L, W, H, gap, minR, guess_site(pts, ll), pts))
        except Exception:
            continue

    if not rows:
        print('웨이포인트 파일을 찾지 못했습니다.'); return

    print(f'{"파일":<62}{"점수":>6}{"길이":>9}{"크기(m)":>14}{"시종":>7}{"최소R":>7}  위치')
    print('-' * 132)
    for f, n, L, W, H, gap, minR, site, _ in rows:
        short = f.replace(os.path.expanduser('~'), '~')
        if len(short) > 60:
            short = '...' + short[-57:]
        warn = '!' if minR < 2.42 else ' '
        print(f'{short:<62}{n:>6}{L:>8.1f}m{W:>6.0f}x{H:<7.0f}{gap:>6.1f}m{minR:>6.2f}m{warn} {site}')
    print(f'\n총 {len(rows)}개    (최소R 옆 ! = 차량 한계 2.42m 미만 구간 있음)')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        k = len(rows); cols = min(4, k); rowsn = (k + cols - 1)//cols
        fig, axes = plt.subplots(rowsn, cols, figsize=(5.5*cols, 5.2*rowsn), squeeze=False)
        for ax, (f, n, L, W, H, gap, minR, site, pts) in zip(axes.ravel(), rows):
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            ax.plot(xs, ys, '-', lw=1.5)
            ax.plot(xs[0], ys[0], 'o', ms=9, color='green')
            ax.plot(xs[-1], ys[-1], 's', ms=9, color='red')
            ax.set_title(f'{os.path.basename(f)[:42]}\n{n}pt {L:.0f}m {W:.0f}x{H:.0f}m', fontsize=9)
            ax.set_aspect('equal'); ax.grid(alpha=.3)
        for ax in axes.ravel()[k:]:
            ax.axis('off')
        plt.tight_layout(); plt.savefig(a.out, dpi=100)
        print(f'\n그림 저장: {a.out}')

if __name__ == '__main__':
    main()

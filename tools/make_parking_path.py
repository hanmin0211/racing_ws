#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_parking_path.py — 주차 완료 자세 하나로 진입 궤적을 만든다.

  parking_pose_N.yaml (자세 1개)  ──→  parking_N.yaml (parking_node 가 먹는 궤적)

★ 왜 만들어 쓰나
  teleop 으로 좁은 칸에 후진 진입하며 기록하면, 사람이 급하게 꺾은 구간이
  최소회전반경(L/tan18° = 2.42m)을 넘어 **재생 때 못 따라간다.**
  여기서는 원호 반경을 R ≥ 2.6m 로 **고정해서** 만들므로 실현 가능성이
  설계상 보장된다. 곡률 검사도 마지막에 한 번 더 돌린다.

기하 (목표 자세에서 거꾸로 쌓는다)
  차는 주차칸에 **후진으로** 들어간다. 즉 주차 완료 헤딩이 θg 라면 차는
  +θg 쪽(칸 입구)에서 들어온 것이다.

    goal ←── 후진직선 d_str ── P1 ←── 후진원호 R, Δθ ── P2 ←── 전진직선 d_app ── P0
             (헤딩 θg 유지)                (θa→θg 로 회전)      (헤딩 θa 유지)

  P2 가 유일한 cusp(기어 전환점)다. P1→goal 과 P2→P1 은 둘 다 후진이라 한 구간으로
  합쳐진다.

  후진 원호 적분: s 를 P2→P1 로 잰 이동거리라 하면
      θ(s) = θa + Δθ·(s/S),   S = R·|Δθ|
      후진이므로  dP/ds = −(cos θ, sin θ)
      ⇒ P2 = P1 + ∫₀^S (cos θ(s), sin θ(s)) ds

접근 헤딩 θa 는 기본적으로 **트랙에서** 가져온다(그 지점의 주행 방향).
P2 가 θa 에 의존하고 θa 가 P2 위치에 의존하므로 몇 번 반복해 수렴시킨다.

사용:
  python3 tools/make_parking_path.py --slot 1
  python3 tools/make_parking_path.py --slot 1 --radius 3.0 --approach-deg 90
  python3 tools/make_parking_path.py --slot 1 --plot        # 그림으로 확인
"""

import argparse
import math
import os
import sys

import yaml

WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
      'waypoints_recorded_resampled_0.5.yaml')
WHEELBASE = 0.785
MAX_STEER_DEG = 18.0
R_MIN = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))   # 2.42 m


def wrap(a):
  return math.atan2(math.sin(a), math.cos(a))


def load_track(path):
  try:
    with open(path) as f:
      d = yaml.safe_load(f)
    seq = d['waypoints'] if isinstance(d, dict) and 'waypoints' in d else d
    return [(float(p['x']), float(p['y'])) for p in seq]
  except Exception as e:  # noqa: BLE001
    print(f'⚠ 트랙을 못 읽었다({e}) — --approach-deg 로 직접 줘야 한다.')
    return []


def track_heading_at(track, x, y):
  """(x,y) 에서 가장 가까운 트랙 점의 진행 방향."""
  if len(track) < 2:
    return None, None
  best_i, best_d = 0, float('inf')
  for i, (px, py) in enumerate(track):
    d = math.hypot(px - x, py - y)
    if d < best_d:
      best_d, best_i = d, i
  j = min(best_i, len(track) - 2)
  th = math.atan2(track[j + 1][1] - track[j][1],
                  track[j + 1][0] - track[j][0])
  return th, best_d


def arc_back(p1, th_g, d_theta, R, ds=0.02):
  """P1 에서 원호를 거슬러 올라가 P2 와 경로점들을 구한다.

  반환: (P2, [P2→P1 순서의 점들])
  """
  S = R * abs(d_theta)
  n = max(2, int(S / ds))
  # s 를 P2→P1 로 재므로, P1 에서 거꾸로 가려면 s 를 S→0 으로 훑는다.
  pts = [tuple(p1)]
  x, y = p1
  for k in range(n):
    s = S * (1.0 - k / n)
    th = th_g - d_theta * (1.0 - s / S)      # θ(s) = θa + Δθ·(s/S)
    step = S / n
    # dP/ds = -(cosθ, sinθ) 이므로 거꾸로 가면 +(cosθ, sinθ)
    x += math.cos(th) * step
    y += math.sin(th) * step
    pts.append((x, y))
  pts.reverse()                              # P2 → P1 순서로
  return (x, y), pts


def resample(pts, spacing):
  """폴리라인을 균일 간격으로 다시 뽑는다."""
  if len(pts) < 2:
    return list(pts)
  out = [pts[0]]
  acc = 0.0
  for i in range(1, len(pts)):
    x0, y0 = pts[i - 1]
    x1, y1 = pts[i]
    seg = math.hypot(x1 - x0, y1 - y0)
    if seg < 1e-9:
      continue
    t = 0.0
    while acc + (seg - t) >= spacing:
      t += spacing - acc
      f = t / seg
      out.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
      acc = 0.0
    acc += seg - t
  if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-6:
    out.append(pts[-1])
  return out


def max_curvature(pts):
  """세 점 외접원으로 최대 곡률(=최소 반경) 추정."""
  worst_r = float('inf')
  for i in range(1, len(pts) - 1):
    (x1, y1), (x2, y2), (x3, y3) = pts[i - 1], pts[i], pts[i + 1]
    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
    if area2 < 1e-12 or a * b * c < 1e-12:
      continue
    # 외접원 반지름 R = abc / (4·넓이).  area2 는 |외적| = 2·넓이 이므로
    # R = abc / (2·area2).  여기서 2 를 빼먹으면 반경이 2배로 나와
    # **실현 불가능한 경로를 통과시킨다**(2026-08-24 에 실제로 그랬다).
    worst_r = min(worst_r, a * b * c / (2.0 * area2))
  return worst_r


def build(goal, th_g, th_a, R, d_str, d_app, spacing):
  """전진직선 + 후진원호 + 후진직선 을 만든다. 반환: (points, meta)"""
  gx, gy = goal
  # 1) 후진 직선: P1 = goal + d_str·(cosθg, sinθg)
  p1 = (gx + d_str * math.cos(th_g), gy + d_str * math.sin(th_g))
  # 2) 후진 원호: θa → θg
  d_theta = wrap(th_g - th_a)
  p2, arc_pts = arc_back(p1, th_g, d_theta, R)
  # 3) 전진 직선: P0 = P2 - d_app·(cosθa, sinθa)
  p0 = (p2[0] - d_app * math.cos(th_a), p2[1] - d_app * math.sin(th_a))

  fwd = resample([p0, p2], spacing)
  rev = resample(arc_pts + [ (gx, gy) ], spacing)

  pts = [(x, y, 1) for (x, y) in fwd]
  # 원호 첫 점은 전진 구간의 마지막 점과 같으므로 건너뛴다
  pts += [(x, y, -1) for (x, y) in rev[1:]]

  meta = {
      'P0_approach': p0, 'P2_cusp': p2, 'P1': p1, 'goal': (gx, gy),
      'turn_deg': math.degrees(d_theta), 'arc_len': R * abs(d_theta),
      'radius': R,
  }
  return pts, meta


def arc_fwd(pose, R, dth, ds=0.05):
  """(x,y,th) 에서 반경 R 로 dth 만큼 전진 선회. 반환 (끝자세, 점들).

  dth>0 이면 좌선회, <0 이면 우선회.
  """
  xs, ys, ths = pose
  s = 1.0 if dth >= 0 else -1.0
  cx = xs - s * R * math.sin(ths)
  cy = ys + s * R * math.cos(ths)
  n = max(2, int(R * abs(dth) / ds))
  pts = []
  for k in range(n + 1):
    t = ths + dth * (k / n)
    pts.append((cx + s * R * math.sin(t), cy - s * R * math.cos(t)))
  return (pts[-1][0], pts[-1][1], ths + dth), pts


def connect_pose_to_line(pose, p2, th_a, r_lo=2.9, r_hi=9.0,
                         min_straight=0.3):
  """차의 실제 자세 → (P2, 방향 th_a) 로 가는 전진 경로: 선회 + 직선.

  ★ 왜 필요한가 (2026-08-24 실차)
    완주 종점 자세는 매번 다르고, 이번엔 yaw 142.9° 로 접근 방향과 25~45°
    어긋나 있었다. 접근로를 **직선으로만** 만들면 차가 그 직선에 올라탈 방법이
    없어 출발하자마자 '전진 구간인데 목표점이 뒤에 있다'로 중단된다.
    그래서 차가 실제로 서 있는 자세에서 시작하는 **선회 구간**을 앞에 붙인다.

  반환: (점들, 회전각[도], R1) 또는 None
  """
  # 직선(d0) → 선회(R1) → 직선.  단일 원호만으로는 도달 가능한 자세가 제한적이라
  # (실제로 완주 자세에서 해가 없었다) 앞에 짧은 직선을 두어 자유도를 하나 더 준다.
  best = None
  ca, sa = math.cos(th_a), math.sin(th_a)
  c0, s0 = math.cos(pose[2]), math.sin(pose[2])
  for d0_10 in range(0, 41, 2):                 # 초기 직선 0~4.0m
    d0 = d0_10 / 10.0
    p_start = (pose[0] + d0 * c0, pose[1] + d0 * s0, pose[2])
    for r10 in range(int(r_lo * 10), int(r_hi * 10) + 1, 2):
      R1 = r10 / 10.0
      for turn in (+1.0, -1.0):
        d = (th_a - pose[2]) % (2.0 * math.pi)
        if turn < 0:
          d = d - 2.0 * math.pi
        if abs(d) > math.radians(200.0):
          continue
        end, apts = arc_fwd(p_start, R1, d)
        vx, vy = p2[0] - end[0], p2[1] - end[1]
        along = vx * ca + vy * sa
        perp = abs(-vx * sa + vy * ca)
        if along < min_straight:
          continue
        pts = ([(pose[0] + t / 10.0 * c0, pose[1] + t / 10.0 * s0)
                for t in range(0, int(d0 * 10), 2)] + apts)
        if best is None or perp < best[0]:
          best = (perp, pts, end, along, math.degrees(d), R1)
  if best is None or best[0] > 0.25:
    return None
  perp, pts, end, along, dth_deg, R1 = best
  n = max(2, int(along / 0.2))
  for k in range(1, n + 1):
    t = along * (k / n)
    pts.append((end[0] + t * ca, end[1] + t * sa))
  return pts, dth_deg, R1


def from_exit(exit_file, d_app, spacing):
  """'칸에서 나오며 찍은 궤적'을 뒤집어 후진 진입 궤적으로 만든다.

  ★ 왜 뒤집어도 되나
    좁은 칸에 **후진으로** 들어가는 건 어렵지만 **전진으로 나오는** 건 쉽다.
    그런데 나올 때 지나간 자리를 거꾸로 되짚으면 그게 곧 들어가는 길이다.
    곡률이 같으므로 실현 가능성도 그대로 유지된다.

    헤딩도 맞는다: 나올 때 점 P 에서 차는 출구 쪽(θ)을 향했다. 들어갈 때
    같은 점에서 차는 칸 쪽으로 **후진**하므로, 헤딩은 여전히 출구 쪽 θ 다.
    → 뒤집은 경로를 그대로 후진으로 따라가면 된다.

  마지막으로 전진 접근 직선을 앞에 붙인다(트랙에서 cusp 까지 오는 구간).
  """
  with open(os.path.expanduser(exit_file), encoding='utf-8') as f:
    d = yaml.safe_load(f)
  pts = [(float(p['x']), float(p['y'])) for p in d['points']]
  if len(pts) < 3:
    raise ValueError('점이 너무 적다')

  rev = list(reversed(pts))                 # 트랙쪽 → 칸 안쪽
  # 출구 헤딩 = 나올 때 마지막 진행 방향
  th_exit = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
  p_cusp = rev[0]
  p0 = (p_cusp[0] - d_app * math.cos(th_exit),
        p_cusp[1] - d_app * math.sin(th_exit))

  fwd = resample([p0, p_cusp], spacing)
  rev_rs = resample(rev, spacing)
  out = [(x, y, 1) for (x, y) in fwd]
  out += [(x, y, -1) for (x, y) in rev_rs[1:]]
  meta = {'P0_approach': p0, 'P2_cusp': p_cusp, 'P1': rev[len(rev) // 2],
          'goal': rev[-1], 'turn_deg': float('nan'),
          'arc_len': 0.0, 'radius': float('nan')}
  return out, meta, th_exit, d.get('origin')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--slot', type=int, default=1)
  ap.add_argument('--pose-file', default=None)
  ap.add_argument('--from-exit', default=None,
                  help='칸에서 나오며 parking_recorder 로 찍은 yaml. '
                       '이걸 주면 자세 대신 이 궤적을 뒤집어 쓴다')
  ap.add_argument('--out', default=None)
  ap.add_argument('--radius', type=float, default=2.8,
                  help=f'후진 원호 반경[m]. 최소 {R_MIN:.2f} 초과여야 한다')
  ap.add_argument('--straight', type=float, default=1.0,
                  help='칸 안으로 곧게 들어가는 마지막 후진 직선[m]')
  ap.add_argument('--approach', type=float, default=3.0,
                  help='cusp 앞의 전진 접근 직선[m]')
  ap.add_argument('--approach-deg', type=float, default=None,
                  help='접근 헤딩[도]. 안 주면 트랙에서 자동으로 가져온다')
  ap.add_argument('--spacing', type=float, default=0.2)
  ap.add_argument('--track', default=WP)
  ap.add_argument('--start-pose', default=None,
                  help='"x,y,yaw_deg" — 완주 종점의 실제 차 자세. 주면 이 자세에서'
                       ' 출발하는 선회 접근로를 만든다(직선 대신).')
  ap.add_argument('--no-fit', action='store_true',
                  help='트랙 정합 탐색을 끄고 --radius/--straight 를 그대로 쓴다')
  ap.add_argument('--plot', action='store_true')
  args = ap.parse_args()

  out = args.out or os.path.expanduser(f'~/parking_{args.slot}.yaml')

  # ---- 모드 B: 칸에서 나오며 찍은 궤적을 뒤집는다 ----
  if args.from_exit:
    print('=' * 60)
    print(f'주차 궤적 생성 — 자리 {args.slot}  (나온 궤적을 뒤집는 방식)')
    print('=' * 60)
    pts, meta, th_exit, origin = from_exit(args.from_exit, args.approach,
                                           args.spacing)
    track = load_track(args.track)
    n_f = sum(1 for p in pts if p[2] > 0)
    n_r = len(pts) - n_f
    total = sum(math.hypot(pts[i + 1][0] - pts[i][0],
                           pts[i + 1][1] - pts[i][1])
                for i in range(len(pts) - 1))
    print(f'  원본: {args.from_exit}')
    print(f'  출구 헤딩 {math.degrees(th_exit):+.1f}°')
    print(f'  기어 전환점 P2 ({meta["P2_cusp"][0]:.2f}, '
          f'{meta["P2_cusp"][1]:.2f})')
    print(f'  목표 정차점 ({meta["goal"][0]:.2f}, {meta["goal"][1]:.2f})')
    print(f'  점 {len(pts)}개 (전진 {n_f} / 후진 {n_r})  총 {total:.2f}m')

    rev_pts = [(x, y) for (x, y, g) in pts if g < 0]
    r_min_actual = max_curvature(rev_pts)
    feasible = r_min_actual >= R_MIN
    print(f'\n  곡률 검사: 최소반경 {r_min_actual:.2f}m  '
          f'{"✅ 실현 가능" if feasible else "❌ 못 도는 구간 — 더 크게 돌아 다시 찍을 것"}')
    if track:
      d_p0 = min(math.hypot(meta['P0_approach'][0] - a,
                            meta['P0_approach'][1] - b) for (a, b) in track)
      print(f'  접근 시작점이 트랙에서 {d_p0:.2f}m  '
            f'{"✅" if d_p0 < 3.0 else "⚠ 멀다"}')

    data = {
        'origin': origin,
        'generated': {'from': os.path.basename(args.from_exit),
                      'mode': 'reversed_exit',
                      'exit_heading_deg': round(math.degrees(th_exit), 2),
                      'min_radius_m': round(r_min_actual, 3)},
        'points': [{'x': round(float(x), 3), 'y': round(float(y), 3),
                    'yaw': 0.0, 'speed': 0.3 * g, 'gear': int(g)}
                   for (x, y, g) in pts],
        'stats': {'count': len(pts), 'forward': n_f, 'reverse': n_r,
                  'length_m': round(total, 2)},
    }
    with open(out, 'w', encoding='utf-8') as f:
      yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f'\n  → {out}')
    if args.plot:
      plot(pts, meta, track, args.slot, th_exit, th_exit)
    print('\n  다음: python3 tools/test_parking_node.py --file ' + out)
    print('=' * 60)
    return 0 if feasible else 1

  # ---- 모드 A: 주차 자세 하나로 생성 ----
  pose_file = args.pose_file or os.path.expanduser(
      f'~/parking_pose_{args.slot}.yaml')

  if not os.path.exists(pose_file):
    print(f'❌ {pose_file} 가 없다.')
    print(f'   먼저: ros2 run mission_perception parking_pose_recorder '
          f'--ros-args -p slot:={args.slot}')
    return 1

  with open(pose_file, encoding='utf-8') as f:
    pd = yaml.safe_load(f)
  gx = float(pd['pose']['x'])
  gy = float(pd['pose']['y'])
  th_g = math.radians(float(pd['pose']['yaw_deg']))
  origin = pd.get('origin')

  if args.radius <= R_MIN:
    print(f'❌ 반경 {args.radius}m 는 최소회전반경 {R_MIN:.2f}m 이하다. '
          f'{R_MIN + 0.2:.1f} 이상으로 줄 것.')
    return 1

  print('=' * 60)
  print(f'주차 궤적 생성 — 자리 {args.slot}')
  print('=' * 60)
  print(f'  주차 완료 자세: ({gx:.2f}, {gy:.2f})  '
        f'yaw {math.degrees(th_g):+.1f}°')

  track = load_track(args.track)

  # 접근 헤딩 결정
  if args.approach_deg is not None:
    th_a = math.radians(args.approach_deg)
    src = '수동 지정'
  else:
    if not track:
      print('❌ 트랙이 없어 접근 헤딩을 못 정한다 — --approach-deg 로 줄 것.')
      return 1
    # P2 가 θa 에 의존하고 θa 가 P2 에 의존하므로 몇 번 반복해 수렴시킨다.
    th_a, _ = track_heading_at(track, gx, gy)
    for _ in range(6):
      pts, meta = build((gx, gy), th_g, th_a, args.radius,
                        args.straight, args.approach, args.spacing)
      th_new, dist = track_heading_at(track, *meta['P2_cusp'])
      if th_new is None or abs(wrap(th_new - th_a)) < math.radians(0.5):
        th_a = th_new if th_new is not None else th_a
        break
      th_a = th_new
    src = '트랙에서 자동'

  # ★ 반경·직선길이를 탐색해 **접근 시작점 P0 가 트랙 위에 오도록** 맞춘다.
  #
  #   왜 필요한가: 헤딩만 트랙에 맞추면 접근선이 트랙과 나란하기만 하고
  #   **옆으로 밀려 있을 수 있다.** 실제로 자리1에서 P0 가 트랙 2.08m 남쪽에
  #   생겼다. 랩을 끝낸 차는 트랙 위에 있으므로 거기서 2m 넘게 떨어진 경로는
  #   parking_node 가 시작하자마자 이탈로 판정해 ABORT 한다.
  #   (abort_cross_track 기본 1.0m)
  if track and not args.no_fit:
    # 차가 실제로 서 있을 자리 = 랩 종료점(경로 마지막 점). 거기서 주차를 시작한다.
    if args.start_pose:
      _sx, _sy, _syaw = [float(v) for v in args.start_pose.split(',')]
      start_pose = (_sx, _sy, math.radians(_syaw))
      anchor = (_sx, _sy)
      print(f'  실제 완주 종점 자세에서 출발: ({_sx:.2f}, {_sy:.2f}) '
            f'yaw {_syaw:+.1f}°')
    else:
      start_pose = None
      anchor = tuple(track[-1])
    # cusp 가 갈 수 있는 곳인지 볼 때는 **자르기 전 경로**를 쓴다. 경로를 잘라
    # 끝을 앞당기면 그 뒤 차선이 트랙 목록에서 사라져 멀쩡한 cusp 도 걸린다.
    corridor = load_track(os.path.join(os.path.dirname(args.track),
                                       'waypoints_full_beforetrim.yaml')) or track
    th_track = th_a                                # 트랙 방향(기준)
    best = None
    # 최소회전반경(2.42m)에 바짝 붙이면 조향이 계속 포화(±18°)라 여유가 없다.
    # 실차의 조향 오차·지연을 흡수하려면 마진이 필요하므로 하한을 올려 잡는다.
    r_lo = max(R_MIN + 0.4, 2.9)
    # ★ 접근 각도 th_a 도 자유변수로 푼다.
    #   트랙 방향으로 고정하면 접근선이 랩 종료점을 못 지나는 경우가 생긴다
    #   (자리1에서 최선 1.04m — parking_node 이탈한계 1.0m 를 넘겨 시작하자마자
    #   ABORT). 각도를 조금 틀면 종료점을 정확히 지나게 만들 수 있고, 차는
    #   그 각도로 서 있지 않더라도 pursuit 이 초반에 맞춰 들어간다.
    #   다만 트랙 방향에서 너무 벗어나면 부자연스러우므로 벌점을 준다.
    for r10 in range(int(r_lo * 10), 81):          # 반경 2.5~8.0m
      R = r10 / 10.0
      for s10 in range(3, 31):                     # 직선 0.3~3.0m
        d_str = s10 / 10.0
        for dth in range(-30, 31, 2):              # 접근각 ±30°
          th = th_track + math.radians(dth)
          _p, m = build((gx, gy), th_g, th, R, d_str, 1.0, args.spacing)
          cx_, sy_ = math.cos(th), math.sin(th)
          vx = anchor[0] - m['P2_cusp'][0]
          vy = anchor[1] - m['P2_cusp'][1]
          along = vx * cx_ + vy * sy_        # 음수여야 anchor 가 P2 뒤
          perp = abs(-vx * sy_ + vy * cx_)
          d_app = -along
          if d_app < 1.0 or d_app > 9.0:
            continue
          # 종료점에서 벗어난 거리 + 트랙 방향에서 틀어진 각도(벌점)
          # ★ 접근 활주로(d_app)를 길게 잡는 것이 가장 중요하다.
          #   실차 완주 종점 자세는 궤적과 1m·40° 씩 어긋난다. 그걸 바로잡으려면
          #   cusp 까지 달릴 거리가 있어야 한다. 활주로가 1.3m 뿐이던 자리1 은
          #   시뮬에서도 -25°/-44° 오차를 못 잡고 중단했다.
          #   반경이 클수록 조향 여유도 생기므로 함께 보상한다.
          # ★ cusp 는 차가 실제로 갈 수 있는 곳이어야 한다. 활주로만 키우면
          #   기어전환점이 차선 밖(트랙에서 4~6m)으로 밀려나 장애물과 만난다.
          d_cusp = min(math.hypot(m['P2_cusp'][0] - a, m['P2_cusp'][1] - b)
                       for (a, b) in corridor)
          if d_cusp > 6.5:
            continue
          if start_pose is not None:
            # 실제 자세에서 선회로 접근선에 올라탈 수 있어야만 후보다.
            conn = connect_pose_to_line(start_pose, m['P2_cusp'], th)
            if conn is None:
              continue
            perp = 0.0                     # 선회로 정확히 붙는다
            d_app = len(conn[0]) * 0.05    # 대략적인 접근 길이(보상용)
          elif perp > 2.0:
            continue          # 접근 허용(3m) 안쪽으로만
          cost = (0.30 * perp + 0.02 * abs(dth) + 0.40 * d_cusp
                  - 0.25 * min(d_app, 9.0) - 0.03 * min(R, 5.0))
          if best is None or cost < best[0]:
            best = (cost, perp, R, d_str, th, d_app, dth)
    if best is not None:
      _c, perp, args.radius, args.straight, th_a, args.approach, dth = best
      print(f'  ▸ 랩 종료점 정합: 반경 {args.radius:.1f}m  직선 {args.straight:.1f}m  '
            f'접근 {args.approach:.1f}m  접근각 트랙대비 {dth:+d}°')
      print(f'    랩 종료점 ({anchor[0]:.2f}, {anchor[1]:.2f}) 에서 '
            f'접근선까지 {perp:.2f}m')
    else:
      print('  ⚠ 랩 종료점에 맞는 조합을 못 찾았다 — 기본값으로 만든다.')

  pts, meta = build((gx, gy), th_g, th_a, args.radius,
                    args.straight, args.approach, args.spacing)

  # 전진 구간을 '실제 자세에서 출발하는 선회 + 직선' 으로 교체한다.
  if track and not args.no_fit and args.start_pose:
    _sx, _sy, _syaw = [float(v) for v in args.start_pose.split(',')]
    conn = connect_pose_to_line((_sx, _sy, math.radians(_syaw)),
                                meta['P2_cusp'], th_a)
    if conn is None:
      print('  ⚠ 이 자세에서 접근선에 올라탈 선회를 못 찾았다 — 직선 접근을 쓴다.')
    else:
      fwd_pts, dth_deg, R1 = conn
      rev_pts = [(x, y) for (x, y, g) in pts if g < 0]
      pts = ([(x, y, 1) for (x, y) in resample(fwd_pts, args.spacing)]
             + [(x, y, -1) for (x, y) in rev_pts])
      meta['P0_approach'] = fwd_pts[0]
      print(f'  ▸ 선회 접근: 반경 {R1:.1f}m 로 {dth_deg:+.0f}° 선회 후 직선')

  print(f'  접근 헤딩: {math.degrees(th_a):+.1f}°  ({src})')
  print(f'  회전량: {meta["turn_deg"]:+.1f}°   원호 반경 {args.radius:.2f}m '
        f'(최소 {R_MIN:.2f}m)   호길이 {meta["arc_len"]:.2f}m')
  print(f'  접근 시작점 P0 ({meta["P0_approach"][0]:.2f}, '
        f'{meta["P0_approach"][1]:.2f})')
  print(f'  기어 전환점 P2 ({meta["P2_cusp"][0]:.2f}, '
        f'{meta["P2_cusp"][1]:.2f})')

  n_f = sum(1 for p in pts if p[2] > 0)
  n_r = len(pts) - n_f
  total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
              for i in range(len(pts) - 1))
  print(f'  점 {len(pts)}개 (전진 {n_f} / 후진 {n_r})  총 {total:.2f}m')

  # ---- 검증 ----
  ok = True
  rev_pts = [(x, y) for (x, y, g) in pts if g < 0]
  r_min_actual = max_curvature(rev_pts)
  feasible = r_min_actual >= R_MIN
  print(f'\n  곡률 검사: 최소반경 {r_min_actual:.2f}m  '
        f'{"✅ 실현 가능" if feasible else "❌ 못 도는 구간 있음"}')
  ok = ok and feasible

  if track:
    d_p0 = min(math.hypot(meta['P0_approach'][0] - a,
                          meta['P0_approach'][1] - b) for (a, b) in track)
    near = d_p0 < 3.0
    print(f'  접근 시작점이 트랙에서 {d_p0:.2f}m  '
          f'{"✅" if near else "⚠ 멀다 — 차가 여기까지 갈 수 있는지 확인"}')

  data = {
      'origin': origin,
      'generated': {
          'from': os.path.basename(pose_file),
          'radius': args.radius, 'straight': args.straight,
          'approach': args.approach,
          'approach_heading_deg': round(math.degrees(th_a), 2),
          'turn_deg': round(meta['turn_deg'], 2),
          'min_radius_m': round(r_min_actual, 3),
      },
      'points': [{'x': round(float(x), 3), 'y': round(float(y), 3),
                  'yaw': 0.0, 'speed': 0.3 * g, 'gear': int(g)}
                 for (x, y, g) in pts],
      'stats': {'count': len(pts), 'forward': n_f, 'reverse': n_r,
                'length_m': round(total, 2)},
  }
  with open(out, 'w', encoding='utf-8') as f:
    yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
  print(f'\n  → {out}')

  if args.plot:
    plot(pts, meta, track, args.slot, th_g, th_a)

  print('\n  다음: 이 궤적이 실제로 재생되는지 확인')
  print(f'    python3 tools/test_parking_node.py --file {out}')
  print('=' * 60)
  return 0 if ok else 1


def plot(pts, meta, track, slot, th_g, th_a):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt

  fig, ax = plt.subplots(figsize=(10, 9))
  if track:
    ax.plot([p[0] for p in track], [p[1] for p in track], '-',
            lw=1.2, color='#cbd5e0', label='track')
  f = [(x, y) for (x, y, g) in pts if g > 0]
  r = [(x, y) for (x, y, g) in pts if g < 0]
  ax.plot([p[0] for p in f], [p[1] for p in f], '-', lw=3,
          color='#2b6cb0', label='forward')
  ax.plot([p[0] for p in r], [p[1] for p in r], '-', lw=3,
          color='#d69e2e', label='reverse')
  for name, p, c, m in [('P0 start', meta['P0_approach'], '#38a169', 'o'),
                        ('P2 cusp', meta['P2_cusp'], '#805ad5', 'D'),
                        ('GOAL', meta['goal'], '#e53e3e', 's')]:
    ax.plot(p[0], p[1], m, ms=12, color=c, zorder=5)
    ax.annotate(name, p, fontsize=10, weight='bold', color=c,
                xytext=(8, 8), textcoords='offset points')
  # 주차 자세 화살표
  gx, gy = meta['goal']
  ax.arrow(gx, gy, 1.2 * math.cos(th_g), 1.2 * math.sin(th_g),
           head_width=0.28, color='#e53e3e', zorder=6)
  ax.set_aspect('equal')
  ax.grid(alpha=0.3)
  ax.legend(loc='best')
  ax.set_title(f'slot {slot}  |  turn {meta["turn_deg"]:+.0f}deg  '
               f'R={meta["radius"]:.1f}m')
  ax.set_xlabel('x [m]')
  ax.set_ylabel('y [m]')
  pad = 4
  xs = [p[0] for p in pts]
  ys = [p[1] for p in pts]
  ax.set_xlim(min(xs) - pad, max(xs) + pad)
  ax.set_ylim(min(ys) - pad, max(ys) + pad)
  out = os.path.expanduser(f'~/parking_{slot}_path.png')
  plt.tight_layout()
  plt.savefig(out, dpi=110)
  print(f'  그림: {out}')


if __name__ == '__main__':
  sys.exit(main())

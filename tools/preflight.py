#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preflight.py — 주행 데이터(원점·경로·정지점·주차)를 한 번에 검사한다.

★ 왜 필요한가
  현장에서 기록한 데이터가 못 쓰는 것이었다는 사실을 **현장을 떠난 뒤에**
  알면 되돌릴 방법이 없다. 이 프로젝트에서 실제로 그런 일들이 있었다.

    · 원점이 남의 장소로 남아 좌표가 150km 어긋남      (2026-08-17)
    · 경로에 최소회전반경보다 급한 커브가 있어 반드시 이탈 (2026-08-22)
    · 평지 3m + 경사로 구간에서 헤딩 캘리브 → 40° 틀어짐  (2026-09-02)
    · 완주 종점 자세가 주차 궤적과 58~78° 어긋나 진입 실패  (2026-08-25)

  넷 다 **파일만 보면 미리 알 수 있는 것**이었다. 여기서 전부 검사한다.
  기록 직후 이 스크립트를 돌려 ❌ 가 없으면 그날 데이터는 쓸 수 있다.

사용:
  python3 tools/preflight.py                       # 기본 파일들로 전체 검사
  python3 tools/preflight.py --waypoints path.yaml # 특정 경로 파일
  python3 tools/preflight.py --calib-distance 6    # 캘리브 거리를 줄였을 때
"""

import argparse
import glob
import math
import os
import sys
import time

import yaml

sys.path.insert(0, '/home/han/racing_ws/src/waypoint_follower')
from waypoint_follower.site_origin import (  # noqa: E402
    load_site_origin, read_origin_stamp)

WS = '/home/han/racing_ws'
DEF_WAYPOINTS = (f'{WS}/src/pure_pursuit_pkg/config/'
                 'waypoints_recorded_resampled_0.5.yaml')
WHEELBASE = 0.785          # m (실측)
MAX_STEER_DEG = 18.0       # 제어 상한 (포화 회피)

FAILS = []
WARNS = []


def ok(msg):
  print(f'  ✅ {msg}')


def warn(msg):
  print(f'  ⚠  {msg}')
  WARNS.append(msg)


def bad(msg):
  print(f'  ❌ {msg}')
  FAILS.append(msg)


def head(title):
  print(f'\n{"─" * 68}\n{title}\n{"─" * 68}')


# ---------------------------------------------------------------- 기하 유틸
def hypot(a, b):
  return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(pts):
  return sum(hypot(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def heading_at(pts, i):
  """idx i 에서의 진행방향 [rad]. 끝점은 이웃 선분을 쓴다."""
  j = min(max(i, 0), len(pts) - 2)
  return math.atan2(pts[j + 1][1] - pts[j][1], pts[j + 1][0] - pts[j][0])


def ang_diff_deg(a, b):
  """두 각(rad) 차이를 [-180, 180] 도로."""
  return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def radii(w):
  """연속 3점 외접원 반경 [(R, idx), ...]. smooth_path.py 와 같은 정의."""
  out = []
  for i in range(1, len(w) - 1):
    (x1, y1), (x2, y2), (x3, y3) = w[i - 1], w[i], w[i + 1]
    a = hypot((x1, y1), (x2, y2))
    b = hypot((x2, y2), (x3, y3))
    c = hypot((x1, y1), (x3, y3))
    s = (a + b + c) / 2.0
    ar2 = s * (s - a) * (s - b) * (s - c)
    if ar2 <= 1e-12:
      continue
    out.append(((a * b * c) / (4.0 * math.sqrt(ar2)), i))
  return out


def nearest(pts, q):
  """q 에 가장 가까운 점의 (거리, idx)."""
  best = (float('inf'), -1)
  for i, p in enumerate(pts):
    d = hypot(p, q)
    if d < best[0]:
      best = (d, i)
  return best


def read_pts(d, key='waypoints'):
  seq = d.get(key) or []
  return [(float(p['x']), float(p['y'])) for p in seq
          if isinstance(p, dict) and 'x' in p and 'y' in p]


# ---------------------------------------------------------------- 검사 A
def check_origin(cur):
  head('A. 원점 (config/site_origin.yaml)')
  epsg, ox, oy, site = cur
  f = f'{WS}/config/site_origin.yaml'
  age_h = (time.time() - os.path.getmtime(f)) / 3600.0 if os.path.exists(f) else -1
  print(f'  site   : {site}')
  print(f'  origin : ({ox:.1f}, {oy:.1f})  EPSG:{epsg}')
  print(f'  수정   : {age_h:.1f}시간 전')
  try:
    from pyproj import Transformer
    inv = Transformer.from_crs(f'EPSG:{epsg}', 'EPSG:4326', always_xy=True)
    lon, lat = inv.transform(ox, oy)
    print(f'  위경도 : {lat:.5f}, {lon:.5f}')
  except Exception as e:  # noqa: BLE001
    warn(f'위경도 역산 실패({e}) — pyproj 확인')
  if 'FALLBACK' in str(site):
    bad('폴백 원점이 쓰이고 있다 — site_origin.yaml 을 못 읽는다')
  else:
    ok('원점 파일 정상')
  return ox, oy


def check_stamp(d, label, cur):
  """파일의 원점 스탬프를 현재 원점과 대조. True 면 이 장소 데이터."""
  epsg, ox, oy, _ = cur
  f_ox, f_oy, f_epsg, f_site = read_origin_stamp(d)
  if f_ox is None:
    warn(f'{label}: 원점 기록 없음 — 어느 장소 것인지 파일만으로는 알 수 없다. '
         '(tools/shift_waypoints.py --stamp-only 로 찍어둘 것)')
    return True
  dist = math.hypot(f_ox - ox, f_oy - oy)
  if f_epsg is not None and f_epsg != epsg:
    bad(f'{label}: UTM 대역 불일치 (파일 EPSG:{f_epsg} ≠ 현재 EPSG:{epsg})')
    return False
  if dist < 0.01:
    ok(f'{label}: 원점 일치 ({f_site or "장소명 미기록"})')
    return True
  if dist > 1000.0:
    bad(f'{label}: 원점이 {dist / 1000:.1f}km 다르다 — 다른 장소({f_site}) 데이터다')
    return False
  warn(f'{label}: 원점이 {dist:.1f}m 다르다 — 로드 시 자동 환산된다')
  return True


# ---------------------------------------------------------------- 검사 B
def check_waypoints(path, cur, args):
  head(f'B. 주행 경로 — {os.path.basename(path)}')
  if not os.path.exists(path):
    bad(f'경로 파일 없음: {path}')
    return None
  d = yaml.safe_load(open(path)) or {}
  same_site = check_stamp(d, '경로', cur)
  pts = read_pts(d)
  if len(pts) < 3:
    bad(f'점이 {len(pts)}개뿐 — 경로로 쓸 수 없다')
    return None

  total = path_length(pts)
  seg = [hypot(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
  gap = hypot(pts[0], pts[-1])
  print(f'  점 {len(pts)}개 · 길이 {total:.1f}m · 간격 '
        f'{min(seg):.2f}~{max(seg):.2f}m (평균 {total / len(seg):.2f})')
  print(f'  시작 ({pts[0][0]:.2f}, {pts[0][1]:.2f}) → '
        f'종점 ({pts[-1][0]:.2f}, {pts[-1][1]:.2f})  시작-끝 {gap:.2f}m '
        f'[{"열린" if gap > 2.0 else "닫힌"} 경로]')

  # B-1 곡률 — 최소회전반경보다 급하면 제어가 완벽해도 반드시 이탈한다
  min_r = args.min_radius
  rs = sorted(radii(pts))
  if rs:
    hard = [(r, i) for r, i in rs if r < min_r]
    tight = [(r, i) for r, i in rs if min_r <= r < min_r * 1.45]
    need = math.degrees(math.atan(WHEELBASE / rs[0][0]))
    print(f'  최소R {rs[0][0]:.2f}m (idx{rs[0][1]}) → 필요타각 {need:.1f}° '
          f'/ 상한 {MAX_STEER_DEG:.0f}°')
    if hard:
      bad(f'못 도는 커브 {len(hard)}개 (R<{min_r}m) — '
          f'tools/smooth_path.py 로 평활화할 것')
      for r, i in hard[:5]:
        print(f'       idx{i:4d}  R={r:.2f}m')
    elif tight:
      warn(f'빠듯한 커브 {len(tight)}개 (R<{min_r * 1.45:.1f}m) — 속도 낮출 것')
    else:
      ok(f'전 구간 R ≥ {min_r}m — 최대타각 {MAX_STEER_DEG:.0f}° 로 통과 가능')

  # B-2 시작부 run-up 직진성 — 헤딩 캘리브가 여기서 끝나야 한다
  L = args.calib_distance
  acc, k = 0.0, 0
  while k < len(pts) - 1 and acc + seg[k] <= L:
    acc += seg[k]
    k += 1
  print(f'\n  [헤딩 캘리브 run-up] 시작 {L:.0f}m = 앞 {k + 1}개 점 ({acc:.1f}m)')
  if acc < L * 0.9:
    bad(f'경로 전체가 {total:.1f}m 라 {L:.0f}m run-up 을 확보할 수 없다')
  else:
    run = pts[:k + 1]
    h0 = heading_at(run, 0)
    devs = [abs(ang_diff_deg(heading_at(run, i), h0))
            for i in range(len(run) - 1)]
    # 시작→끝 직선에서의 최대 수직 이탈
    ax, ay = run[0]
    bx, by = run[-1]
    cl = math.hypot(bx - ax, by - ay)
    lat = max(abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / cl
              for px, py in run) if cl > 1e-6 else 0.0
    print(f'    헤딩 최대편차 {max(devs):.1f}°   직선 대비 최대이탈 {lat:.2f}m   '
          f'호길이/직선 {acc / cl:.3f}' if cl > 1e-6 else '')
    if max(devs) <= args.runup_heading and lat <= args.runup_lateral:
      ok(f'run-up 직선 — 여기서 캘리브하면 안전')
    elif max(devs) <= args.runup_heading * 2:
      warn(f'run-up 이 약간 휘었다(헤딩 {max(devs):.1f}°, 이탈 {lat:.2f}m) — '
           f'calib_distance 를 줄이거나 시작점을 앞으로 뺄 것')
    else:
      bad(f'run-up 이 직선이 아니다(헤딩 {max(devs):.1f}°, 이탈 {lat:.2f}m). '
          f'캘리브 중 직진하면 코스를 벗어난다 — '
          f'맨 앞에 평지 직진 {L:.0f}m 를 포함해 다시 기록할 것')
  return pts if same_site else None


# ---------------------------------------------------------------- 검사 C
def check_stop_points(path, wp, cur):
  head(f'C. 정지점 — {path}')
  if not os.path.exists(path):
    warn(f'정지점 파일 없음: {path} (횡단보도/신호등 미션 비활성)')
    return
  d = yaml.safe_load(open(path)) or {}
  check_stamp(d, '정지점', cur)
  pts = read_pts(d, 'stop_points')
  if not pts:
    warn('정지점이 0개')
    return
  print(f'  정지점 {len(pts)}개')
  for i, q in enumerate(pts):
    if wp:
      dmin, idx = nearest(wp, q)
      s = f'({q[0]:8.2f}, {q[1]:8.2f})  경로까지 '
      if dmin > 50.0:
        bad(s + f'{dmin / 1000:.1f}km — 다른 장소에서 찍은 것이다')
      elif dmin > 2.0:
        warn(s + f'{dmin:.2f}m — 경로에서 멀다(오기록 의심)')
      else:
        ok(s + f'{dmin:.2f}m (경로 idx{idx})')
    else:
      print(f'  ({q[0]:8.2f}, {q[1]:8.2f})  — 경로가 없어 대조 못 함')


# ---------------------------------------------------------------- 검사 D
def check_parking(wp, cur, args):
  """완주 종점 자세에서 각 주차칸까지 **실제 planner 로** 궤적이 나오는지 본다.

  ★ 왜 planner 를 직접 부르나
    예전엔 미리 만든 궤적(~/parking_N.yaml)을 재생하는 방식이라 '완주 종점이
    궤적 위 어느 점과 자세가 맞는가' 를 봐야 했다. 지금 parking_node 는
    parking_planner(Dubins 전진 + 후진 원호)로 **그 자리에서** 궤적을 만든다.
    그래서 판정 기준도 '자세가 맞는가' 가 아니라 '해가 나오는가' 다.
    노드와 같은 코드를 부르므로 여기 결과가 곧 실차 노드의 결과다.
  """
  head('D. 후진주차 — 완주 종점 자세에서 계획이 나오는가')
  try:
    sys.path.insert(0, f'{WS}/src/mission_perception')
    from mission_perception import parking_planner as pp
  except Exception as e:  # noqa: BLE001
    warn(f'parking_planner 를 못 불렀다({e}) — 주차 검사 생략')
    return

  files = sorted(glob.glob(os.path.expanduser('~/parking_pose_[0-9].yaml')))
  if not files:
    warn('주차 자세 파일(~/parking_pose_N.yaml)이 없다')
    return
  if not wp:
    warn('주행 경로가 없어 출발 자세를 알 수 없다')
    return

  end_xy = wp[-1]
  end_h = heading_at(wp, len(wp) - 2)
  start = (end_xy[0], end_xy[1], end_h)
  print(f'  출발(완주 종점) ({end_xy[0]:.3f}, {end_xy[1]:.3f}) '
        f'헤딩 {math.degrees(end_h):+.1f}°')

  goals = []
  for f in files:
    d = yaml.safe_load(open(f)) or {}
    if not check_stamp(d, os.path.basename(f), cur):
      continue
    ps = d.get('pose') or {}
    try:
      goals.append((f, (float(ps['x']), float(ps['y'])),
                    math.radians(float(ps['yaw_deg']))))
    except (KeyError, TypeError, ValueError):
      bad(f'{os.path.basename(f)}: pose(x, y, yaw_deg) 를 읽을 수 없다')
  if not goals:
    return

  print()
  print(f'  {"자리":<20}{"거리":>8}{"뒤로":>8}{"전진":>8}{"후진":>8}'
        f'{"R_rev":>8}   판정')
  for f, goal, th_g in goals:
    name = os.path.basename(f)
    try:
      plan = pp.plan_parking(
          start, goal, th_g,
          r_fwd=args.plan_r_fwd,
          r_rev=(args.plan_r_rev_min, args.plan_r_rev_max, 0.35),
          d_back=(0.0, args.plan_back_max, 0.75),
          spacing=0.2)
    except Exception as e:  # noqa: BLE001
      bad(f'{name}: 계획 중 예외 {e}')
      continue
    dist = math.hypot(goal[0] - end_xy[0], goal[1] - end_xy[1])
    if plan is None:
      print(f'  {name:<20}{dist:>7.2f}m{"—":>8}{"—":>8}{"—":>8}{"—":>8}'
            f'   ❌ 계획 실패')
      FAILS.append(f'{name}: 완주 종점 자세에서 궤적을 만들 수 없다 — ABORT 된다')
      continue
    print(f'  {name:<20}{dist:>7.2f}m{plan["d_back"]:>7.2f}m'
          f'{plan["fwd_len"]:>7.2f}m{plan["rev_len"]:>7.2f}m'
          f'{plan["r_rev"]:>7.2f}m   ✅ 진입 가능')
  print('\n  ※ parking_node 와 같은 parking_planner 를 호출한 결과다.')
  print('     계획이 나와도 실차 추종 오차는 별개다 — '
        'tools/test_parking_node.py --slot N 으로 폐루프 확인할 것.')


def check_mission_plan(path, wp, args):
  """미션 계획이 **이 코스의 것인지** 본다.

  ★ 왜 체크리스트에 넣나 (2026-09-10 발견)
    config/mission_plan.yaml 이 대구(184m) 값 그대로인 채 용인(648m)을 달리면
    후진주차 트리거 s_enter=175m 가 코스 한복판에 떨어진다. 주행 중간에 후진
    기동이 시작되고 그건 이탈 = **탈락**이다. 다른 치명 항목(정지점·주차자세)은
    런타임 가드가 '안 쓰고 넘어가는' 식으로 안전하게 실패하는데, 미션 계획은
    **엉뚱한 위치에서 실제로 동작한다.** 가장 비싼 실수라 주행 전에 본다.

  (mission_sequencer 도 같은 대조를 런타임에 하지만, 주행 전에 알아야 고친다)
  """
  head(f'E. 미션 계획 — {os.path.basename(path)}')
  if not os.path.exists(path):
    warn(f'미션 계획 파일 없음: {path} (시퀀서 미사용이면 정상)')
    return
  try:
    d = yaml.safe_load(open(path, encoding='utf-8')) or {}
  except Exception as e:  # noqa: BLE001
    bad(f'미션 계획 로드 실패: {e}')
    return

  course = d.get('course') or {}
  site = course.get('site', '(site 미기재)')
  print(f'  site   : {site}')
  path_len = path_length(wp) if wp else 0.0

  want_len = course.get('path_length_m')
  if want_len is None:
    warn('계획에 course.path_length_m 이 없다 — 코스 대조 불가. 채울 것')
  elif path_len > 0:
    rel = abs(path_len - float(want_len)) / float(want_len)
    if rel > 0.10:
      bad(f'미션 계획: 코스 길이 불일치 (계획 {float(want_len):.1f}m vs '
          f'실제 {path_len:.1f}m) — 다른 장소의 계획이다')
    else:
      ok(f'코스 길이 일치 ({float(want_len):.1f}m vs {path_len:.1f}m)')

  want_wp = course.get('waypoints')
  if want_wp:
    a, b = os.path.basename(str(want_wp)), os.path.basename(args.waypoints)
    if a != b:
      bad(f'미션 계획: 웨이포인트 불일치 (계획 "{a}" vs 실제 "{b}")')
    else:
      ok(f'웨이포인트 일치 ({a})')

  missions = d.get('missions') or []
  on = [m for m in missions if m.get('enabled', True)]
  print(f'  미션   : 전체 {len(missions)}개 중 활성 {len(on)}개')
  if not on:
    warn('활성 미션이 0개 — 자율 완주만 한다(미션 전부 감점). '
         '현장에서 s 를 재고 enabled:true 로 바꿀 것')
  for m in on:
    t = m.get('trigger') or {}
    if str(t.get('type', 'course_s')) != 'course_s':
      continue
    e0, e1 = float(t.get('s_enter', 0.0)), float(t.get('s_exit', 0.0))
    name = m.get('name', '?')
    if e0 == 0.0 and e1 == 0.0:
      bad(f'{name}: s 가 0~0 인데 enabled 다 — 실측값을 안 채웠다')
      continue
    if path_len > 0 and e0 > path_len:
      bad(f'{name}: s_enter {e0:.0f}m 가 코스 끝({path_len:.0f}m) 밖이다')
      continue
    span = min(e1, path_len or e1) - max(e0, 0.0)
    if (not m.get('exclusive', True) and not t.get('allow_full_course')
        and path_len > 0 and span >= 0.9 * path_len):
      bad(f'{name}: 배경 기능이 코스의 {span / path_len * 100:.0f}% 를 덮는다 '
          '— 어디서든 조향을 뺏을 수 있다(이탈 위험). 구간으로 좁힐 것')
      continue
    print(f'    · {name:<14} s {e0:6.1f}~{e1:6.1f}m')


def main():
  ap = argparse.ArgumentParser(
      description='주행 데이터 일관성·주행가능성 사전 검사')
  ap.add_argument('--waypoints', default=DEF_WAYPOINTS)
  ap.add_argument('--stop-points',
                  default=os.environ.get('STOP_POINTS_FILE',
                                         os.path.expanduser('~/stop_points.yaml')))
  ap.add_argument('--calib-distance', type=float, default=10.0,
                  help='헤딩 캘리브 직진거리 [m] (bringup 인자와 같게)')
  ap.add_argument('--min-radius', type=float, default=2.42,
                  help='최소 회전반경 [m] = 축거/tan(최대타각)')
  ap.add_argument('--runup-heading', type=float, default=5.0,
                  help='run-up 허용 헤딩편차 [deg]')
  ap.add_argument('--runup-lateral', type=float, default=0.30,
                  help='run-up 허용 직선이탈 [m]')
  # parking_node 의 기본값과 같게 둔다(다르면 판정이 어긋난다)
  ap.add_argument('--plan-r-fwd', type=float, default=3.0)
  ap.add_argument('--plan-r-rev-min', type=float, default=2.9)
  ap.add_argument('--plan-r-rev-max', type=float, default=5.0)
  ap.add_argument('--plan-back-max', type=float, default=3.0)
  ap.add_argument('--skip-parking', action='store_true')
  ap.add_argument('--mission-plan',
                  default='/home/han/racing_ws/config/mission_plan.yaml')
  args = ap.parse_args()

  print('=' * 68)
  print('  주행 데이터 사전 검사 (preflight)')
  print('=' * 68)

  cur = load_site_origin()
  check_origin(cur)
  wp = check_waypoints(args.waypoints, cur, args)
  check_stop_points(args.stop_points, wp, cur)
  if not args.skip_parking:
    check_parking(wp, cur, args)
  check_mission_plan(args.mission_plan, wp, args)

  head('요약')
  if FAILS:
    print(f'  ❌ 치명 {len(FAILS)}건 — 이대로 주행하면 실패한다')
    for m in FAILS:
      print(f'     · {m}')
  if WARNS:
    print(f'  ⚠  주의 {len(WARNS)}건')
    for m in WARNS:
      print(f'     · {m}')
  if not FAILS and not WARNS:
    print('  ✅ 전부 통과 — 이 데이터로 주행 가능')
  elif not FAILS:
    print('\n  치명적 문제 없음 — 주의사항 확인 후 주행 가능')
  print()
  sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
  main()

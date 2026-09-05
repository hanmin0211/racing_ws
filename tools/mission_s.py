#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mission_s.py — 미션 지점의 **코스 진행거리 s** 를 뽑아 mission_plan.yaml 조각을 만든다.

★ 왜 필요한가
  `mission_sequencer` 는 미션을 `s_enter ~ s_exit` 구간에서 켠다. 그런데 s 는
  waypoint 를 기록해야 나오는 값이라, 현장에서 코스를 찍은 **직후에** 채워야 한다.
  그걸 눈대중으로 적으면 미션이 엉뚱한 데서 켜진다. 여기서 실제 경로로 계산한다.

★ 세 가지 방법 — 현장에서는 --live 가 제일 빠르다
  1) --live      차를 미션 지점에 세우고 실행 → 지금 위치의 s 를 읽는다
  2) --at x,y    좌표를 알고 있을 때
  3) --auto      이미 찍어둔 정지점·주차자세 파일에서 자동으로 뽑는다

사용:
  # 현장: 차를 경사로 검지구역에 세운 뒤
  python3 tools/mission_s.py --waypoints ~/wp.yaml --live --name ramp

  # 이미 찍은 정지점·주차자세 전부
  python3 tools/mission_s.py --waypoints ~/wp.yaml --auto

  # 좌표 직접 (음수는 = 로 붙인다 — 안 그러면 argparse 가 옵션으로 읽는다)
  python3 tools/mission_s.py --waypoints ~/wp.yaml --at=-25.09,78.55 --name crosswalk

출력은 그대로 config/mission_plan.yaml 의 missions: 아래에 붙여넣으면 된다.
"""

import argparse
import glob
import math
import os
import sys

import yaml

sys.path.insert(0, '/home/han/racing_ws/src/waypoint_follower')
from waypoint_follower.site_origin import reconcile_origin  # noqa: E402

WS = '/home/han/racing_ws'
DEF_WP = (f'{WS}/src/pure_pursuit_pkg/config/'
          'waypoints_recorded_resampled_0.5.yaml')

# 미션 이름 → (arm 토픽, done 토픽, 타임아웃, 예상소요, exclusive)
KNOWN = {
    'crosswalk':     ('/crosswalk/arm', '/crosswalk/done', 20.0, 12.0, True),
    'ramp':          ('/ramp/arm', '/ramp/done', 25.0, 12.0, True),
    'traffic_light': ('/traffic_light/arm', '/traffic_light/done', 45.0, 30.0, True),
    'sudden_stop':   ('/sudden_stop/arm', '/sudden_stop/done', 20.0, 10.0, True),
    'parking':       ('/parking/start', '/parking/done', 50.0, 45.0, True),
    'parking_perp':  ('/parking/start', '/parking/done', 50.0, 45.0, True),
    'parking_para':  ('/parking_para/start', '/parking_para/done', 55.0, 50.0, True),
    's_course':      ('/lidar/arm', None, 30.0, 20.0, False),
    'lidar_avoid':   ('/lidar/arm', None, 30.0, 20.0, False),
}


def load_path(path):
  """웨이포인트 → [(x, y)], 누적 s. 원점이 다르면 현재 원점으로 환산한다."""
  d = yaml.safe_load(open(path, encoding='utf-8')) or {}
  pts = [(float(p['x']), float(p['y'])) for p in (d.get('waypoints') or [])]
  if len(pts) < 2:
    raise SystemExit(f'❌ 경로가 너무 짧다: {path}')
  dx, dy, note = reconcile_origin(d, path)
  if note == 'epsg-mismatch':
    raise SystemExit('❌ UTM 대역이 달라 쓸 수 없다.')
  if dx or dy:
    pts = [(x + dx, y + dy) for x, y in pts]
  s = [0.0]
  for i in range(1, len(pts)):
    s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                pts[i][1] - pts[i - 1][1]))
  return pts, s


def s_at(pts, s, q):
  """q 에 가장 가까운 경로 점의 (s, 거리, idx).

  ★ 최근접 '점' 이 아니라 최근접 '선분 위의 발' 을 쓴다. 리샘플 간격이 0.5m 면
    점만 볼 때 최대 0.25m 오차가 나는데, 미션 구간을 s 로 자르는 데는
    그 정도면 충분하지만 정지점처럼 정밀한 값은 선분 보간이 맞다.
  """
  best = (float('inf'), 0.0, 0)
  for i in range(len(pts) - 1):
    ax, ay = pts[i]
    bx, by = pts[i + 1]
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-12:
      continue
    t = ((q[0] - ax) * vx + (q[1] - ay) * vy) / L2
    t = max(0.0, min(1.0, t))
    px, py = ax + vx * t, ay + vy * t
    d = math.hypot(q[0] - px, q[1] - py)
    if d < best[0]:
      best = (d, s[i] + math.sqrt(L2) * t, i)
  return best[1], best[0], best[2]


def emit(name, s_val, total, args):
  """mission_plan.yaml 에 붙일 조각."""
  arm, done, tmo, exp, excl = KNOWN.get(
      name, (f'/{name}/arm', f'/{name}/done', 25.0, 15.0, True))
  lo = max(0.0, s_val - args.lead)
  hi = min(total + 20.0, s_val + args.tail)
  out = [f'  - name: {name}',
         f'    arm_topic: {arm}']
  if done:
    out.append(f'    done_topic: {done}')
  if not excl:
    out.append('    exclusive: false')
  out += ['    trigger:',
          '      type: course_s',
          f'      s_enter: {lo:.1f}',
          f'      s_exit:  {hi:.1f}']
  if excl:
    out += [f'    timeout_s: {tmo:.1f}',
            f'    expected_s: {exp:.1f}']
  out.append('    critical: false')
  return '\n'.join(out)


def nice_name(raw):
  """파일명 → 미션 이름. arm 토픽이 파일명이 되어버리는 걸 막는다."""
  b = os.path.basename(str(raw)).replace('.yaml', '')
  if b.startswith('parking_pose_'):
    return 'parking'          # 직각주차 (평행은 parking_para 로 따로)
  if b.startswith('정지점'):
    return 'crosswalk'
  return b


def report(name, q, pts, s, args, results):
  sv, dist, idx = s_at(pts, s, q)
  total = s[-1]
  flag = '✅' if dist <= 3.0 else ('⚠ 경로에서 멀다' if dist <= 20.0
                                  else '❌ 다른 장소인가?')
  print(f'\n  {name}')
  print(f'    좌표 ({q[0]:.2f}, {q[1]:.2f})  경로까지 {dist:.2f}m  {flag}')
  print(f'    진행거리 s = {sv:.1f}m  (전체 {total:.1f}m, idx{idx})')
  results.append(emit(nice_name(name), sv, total, args))


def live_pose(topic, samples):
  """차를 세워둔 상태에서 현재 위치를 읽는다."""
  import rclpy
  from nav_msgs.msg import Odometry
  from rclpy.node import Node

  class Grab(Node):

    def __init__(self):
      super().__init__('mission_s_grab')
      self.buf = []
      self.create_subscription(Odometry, topic, self.cb, 10)
      print(f'  {topic} 수신 대기 — {samples}개 평균 (차를 세워둘 것)')

    def cb(self, m):
      p = m.pose.pose.position
      self.buf.append((p.x, p.y))

  rclpy.init()
  n = Grab()
  try:
    import time
    t0 = time.time()
    while len(n.buf) < samples and time.time() - t0 < 20.0:
      rclpy.spin_once(n, timeout_sec=0.1)
  finally:
    got = list(n.buf)
    n.destroy_node()
    rclpy.shutdown()
  if not got:
    raise SystemExit(f'❌ {topic} 을 못 받았다 — 스택이 떠 있는지 확인할 것.')
  x = sum(p[0] for p in got) / len(got)
  y = sum(p[1] for p in got) / len(got)
  spread = max(math.hypot(p[0] - x, p[1] - y) for p in got)
  print(f'  {len(got)}개 평균 → ({x:.3f}, {y:.3f})  퍼짐 {spread:.3f}m')
  if spread > 0.2:
    print('  ⚠ 위치가 흔들린다 — 차가 멈춰 있는지, RTK Fixed 인지 확인할 것')
  return (x, y)


def main():
  ap = argparse.ArgumentParser(
      description='미션 지점의 코스 진행거리(s)를 뽑아 계획 조각을 만든다')
  ap.add_argument('--waypoints', default=DEF_WP)
  ap.add_argument('--live', action='store_true',
                  help='지금 차가 서 있는 위치를 쓴다')
  ap.add_argument('--at',
                  help='"x,y" 좌표 직접 지정. 음수면 = 로 붙일 것: --at=-25.09,78.55')
  ap.add_argument('--name', default='mission', help='미션 이름')
  ap.add_argument('--auto', action='store_true',
                  help='정지점·주차자세 파일에서 자동으로 전부 뽑는다')
  ap.add_argument('--odom-topic', default='/odometry/filtered')
  ap.add_argument('--samples', type=int, default=20)
  ap.add_argument('--lead', type=float, default=12.0,
                  help='구간 시작을 지점보다 이만큼 앞에 [m] (감속 여유)')
  ap.add_argument('--tail', type=float, default=15.0,
                  help='구간 끝을 지점보다 이만큼 뒤에 [m]')
  args = ap.parse_args()

  pts, s = load_path(args.waypoints)
  print(f'\n경로: {args.waypoints}')
  print(f'  {len(pts)}점 · 전체 {s[-1]:.1f}m')

  results = []
  if args.auto:
    # 정지점
    for f in [os.environ.get('STOP_POINTS_FILE', ''),
              os.path.expanduser('~/stop_points.yaml'),
              f'{WS}/stop_points_0824_1814.yaml']:
      if f and os.path.exists(f):
        d = yaml.safe_load(open(f, encoding='utf-8')) or {}
        for i, p in enumerate(d.get('stop_points') or []):
          report(f'정지점{i} ({os.path.basename(f)})',
                 (float(p['x']), float(p['y'])), pts, s, args, results)
        break
    # 주차 자세
    for f in sorted(glob.glob(os.path.expanduser('~/parking_pose_[0-9].yaml'))):
      d = yaml.safe_load(open(f, encoding='utf-8')) or {}
      ps = d.get('pose') or {}
      if 'x' in ps:
        report(os.path.basename(f), (float(ps['x']), float(ps['y'])),
               pts, s, args, results)
  elif args.live:
    report(args.name, live_pose(args.odom_topic, args.samples),
           pts, s, args, results)
  elif args.at:
    x, y = [float(v) for v in args.at.split(',')]
    report(args.name, (x, y), pts, s, args, results)
  else:
    ap.error('--live / --at / --auto 중 하나를 줄 것')

  if results:
    print('\n' + '=' * 66)
    print('config/mission_plan.yaml 의 missions: 아래에 붙여넣을 것')
    print('=' * 66)
    for r in results:
      print(r)
    print()
    print('⚠ s_enter 는 감속·정렬 여유다. 미션마다 --lead 로 조절할 것.')
    print('  붙여넣은 뒤 python3 tools/test_mission_sequencer.py 로 순서 확인.')


if __name__ == '__main__':
  main()

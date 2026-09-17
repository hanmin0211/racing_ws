#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_mission_sequencer.py — 미션 시퀀서를 하드웨어 없이 폐루프 검증한다.

가짜 차량이 실제 웨이포인트 경로를 따라 달리고, mission_sequencer 가 구간마다
미션을 arm/disarm 하는지 본다.

검증하는 것 (전부 '못 하면 탈락' 과 직결된다)
  1. 구간에 들어가면 arm 되는가
  2. **한 번에 하나만** arm 되는가 (동시 활성 = 토픽 덮어쓰기 사고)
  3. done 을 받으면 즉시 disarm 하고 다음으로 가는가
  4. done 이 안 와도 timeout 에 **강제 복귀**하는가 (1분 정지 = 탈락)
  5. 시간예산이 모자라면 감점-only 미션을 건너뛰는가

  python3 tools/test_mission_sequencer.py                 # 세 가지 다
  python3 tools/test_mission_sequencer.py --case timeout   # 하나만

⚠ 로직 검증이다. 실제 미션 노드의 동작·GPS 오차는 실차로 확인해야 한다.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

# ★ 실차 스택과 격리 (test_crosswalk_stop.py 와 같은 이유).
os.environ['ROS_DOMAIN_ID'] = os.environ.get('MISSION_TEST_DOMAIN', '78')

import rclpy                                                  # noqa: E402
import yaml                                                   # noqa: E402
from geometry_msgs.msg import PoseStamped                     # noqa: E402
from nav_msgs.msg import Odometry, Path                       # noqa: E402
from rclpy.executors import (ExternalShutdownException,   # noqa: E402
                             SingleThreadedExecutor)
from rclpy.node import Node                                   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile            # noqa: E402
from std_msgs.msg import Bool, String                         # noqa: E402

WS = '/home/han/racing_ws'
# ★ 로직 검증은 픽스처로 한다 (2026-09-10).
#   실전 계획(config/mission_plan.yaml)은 현장 s 를 재기 전까지 전부
#   enabled:false 여야 안전하다 — 모르는 s 로 미션을 켜면 이탈=탈락이다.
#   그러면 arm 이 하나도 안 일어나 로직 테스트가 통째로 죽는다. 분리한다.
PLAN = f'{WS}/tools/fixtures/mission_plan_test.yaml'
LIVE_PLAN = f'{WS}/config/mission_plan.yaml'
DAEGU_PLAN = f'{WS}/config/daegu_2026-08-17/mission_plan.yaml'
YONGIN_WP = f'{WS}/config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml'


def load_path(plan):
  wp = plan.get('course', {}).get('waypoints')
  path = os.path.join(WS, wp) if wp and not os.path.isabs(wp) else wp
  d = yaml.safe_load(open(path, encoding='utf-8'))
  return [(float(p['x']), float(p['y'])) for p in d['waypoints']]


class Harness(Node):
  """가짜 차량 + 미션 노드 흉내. 시퀀서가 arm 하면 정해진 대로 응답한다."""

  def __init__(self, pts, missions, respond, speed, context=None):
    # ★ context — 케이스마다 **독립 컨텍스트**를 쓴다(run_case 주석 참고).
    super().__init__('mission_test_harness', context=context)
    self.pts = pts
    self.speed = speed
    self.respond = respond          # {미션이름: 완료까지 걸리는 초 or None(무응답)}
    self.i = 0.0
    self.s = 0.0

    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.path_pub = self.create_publisher(Path, '/global_path', qos_tl)
    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.done_pubs = {m['name']: self.create_publisher(Bool, m['done_topic'], 10)
                      for m in missions if m.get('done_topic')}

    self.armed = {}                 # 이름 → bool
    self.armed_at = {}
    self.log = []                   # (t, 이름, 'ARM'/'DISARM')
    self.max_concurrent = 0         # exclusive 미션의 동시 활성 최대치
    self.overlap = []               # overlay 와 exclusive 가 같이 켜진 순간들
    self.exclusive = {m['name']: bool(m.get('exclusive', True))
                      for m in missions}
    for m in missions:
      self.create_subscription(
          Bool, m['arm_topic'],
          (lambda n: (lambda msg: self.arm_cb(n, msg)))(m['name']), 10)
    self.create_subscription(String, '/mission/active', self.active_cb, 10)
    self.active = 'NONE'

    self.publish_path()
    self.create_timer(1.0, self.publish_path)   # 늦게 뜬 구독자 대비
    self.t0 = time.time()
    self.create_timer(0.05, self.drive)

  def publish_path(self):
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    for x, y in self.pts:
      ps = PoseStamped()
      ps.header = msg.header
      ps.pose.position.x, ps.pose.position.y = float(x), float(y)
      ps.pose.orientation.w = 1.0
      msg.poses.append(ps)
    self.path_pub.publish(msg)

  def active_cb(self, msg):
    self.active = msg.data

  def arm_cb(self, name, msg):
    was = self.armed.get(name, False)
    self.armed[name] = bool(msg.data)
    t = time.time() - self.t0
    if msg.data and not was:
      self.armed_at[name] = t
      self.log.append((t, name, 'ARM'))
      print(f'  [{t:6.1f}s] ▶ {name} ARM')
    elif was and not msg.data:
      self.log.append((t, name, 'DISARM'))
      print(f'  [{t:6.1f}s] ■ {name} DISARM')
    on = [n for n, v in self.armed.items() if v]
    ex_on = [n for n in on if self.exclusive.get(n, True)]
    ov_on = [n for n in on if not self.exclusive.get(n, True)]
    self.max_concurrent = max(self.max_concurrent, len(ex_on))
    if ex_on and ov_on:
      self.overlap.append((ex_on[0], ov_on[0]))

  def drive(self):
    """경로를 따라 등속 전진. 미션이 arm 되어 있으면 그 자리에 선다."""
    t = time.time() - self.t0
    # ★ 배경 기능(overlay)은 차를 세우지 않는다 — 조향만 덧씌울 뿐이다.
    #   여기서 overlay 까지 세우면 코스 전체에 걸린 라이다 때문에 차가
    #   출발조차 못 해 구간 미션에 영영 도달하지 않는다.
    ex_busy = any(v for n, v in self.armed.items()
                  if self.exclusive.get(n, True))
    if not ex_busy:
      self.s += self.speed * 0.05
    # s → 좌표
    acc, idx = 0.0, 0
    for k in range(len(self.pts) - 1):
      d = math.hypot(self.pts[k + 1][0] - self.pts[k][0],
                     self.pts[k + 1][1] - self.pts[k][1])
      if acc + d >= self.s:
        idx = k
        break
      acc += d
      idx = k + 1
    x, y = self.pts[min(idx, len(self.pts) - 1)]
    o = Odometry()
    o.header.frame_id = 'map'
    o.header.stamp = self.get_clock().now().to_msg()
    o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
    o.pose.pose.orientation.w = 1.0
    self.odom_pub.publish(o)

    # arm 된 미션에 정해진 대로 응답
    for name, on in self.armed.items():
      if not on or name not in self.done_pubs:
        continue
      delay = self.respond.get(name)
      if delay is None:
        continue                     # 무응답 → 시퀀서 타임아웃을 시험
      if t - self.armed_at.get(name, t) >= delay:
        self.done_pubs[name].publish(Bool(data=True))


# ─────────────────────────────────────────────────────────────────────
# ★ 2026-09-18 — SIGTERM 에서도 자식을 반드시 죽인다.
#
#   `timeout 300 python3 tools/test_*.py` 로 돌리다 상한에 걸리면 SIGTERM 이
#   온다. 파이썬은 **SIGTERM 에서 finally 를 돌리지 않고 즉사**한다. 그래서
#   finally 의 kill_group 이 건너뛰어지고, start_new_session=True 로 띄운
#   `ros2 run` 자식이 **살아남는다.**
#
#   살아남은 시퀀서는 같은 토픽에 두 벌이 서로 다른 값을 쏴서 arm 이
#   10Hz 로 깜빡이게 만든다. 그걸 보고 '로직이 깨졌다' 고 오판한다 —
#   이 저장소에서 **세 번** 그랬다.
#
#   SIGTERM 을 KeyboardInterrupt 로 바꿔 finally 가 돌게 하고,
#   그래도 새는 경우를 대비해 띄운 프로세스그룹을 atexit 로 한 번 더 쓸어낸다.
import atexit as _atexit
import signal as _sig

_SPAWNED = []          # 이 실행이 띄운 자식들 (kill_group 대상)


def _reap_all():
  for p in _SPAWNED:
    try:
      os.killpg(os.getpgid(p.pid), _sig.SIGKILL)
    except Exception:  # noqa: BLE001
      pass


def _on_term(_signum, _frame):
  # finally 가 돌도록 예외로 바꾼다. 바꾸지 않으면 즉사하며 좀비를 남긴다.
  raise KeyboardInterrupt('SIGTERM')


_sig.signal(_sig.SIGTERM, _on_term)
_atexit.register(_reap_all)
# ─────────────────────────────────────────────────────────────────────


def kill_group(proc, hard=False):
  """프로세스 그룹째 종료. 남으면 다음 케이스를 오염시킨다."""
  try:
    os.killpg(os.getpgid(proc.pid), signal.SIGKILL if hard else signal.SIGTERM)
  except (ProcessLookupError, PermissionError):
    pass
  try:
    proc.wait(timeout=3)
  except subprocess.TimeoutExpired:
    try:
      os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
      pass


def run_case(title, plan, respond, extra_params, duration, speed=3.0,
             plan_file=None):
  print(f'\n{"=" * 66}\n  {title}\n{"=" * 66}')
  cmd = ['ros2', 'run', 'mission_perception', 'mission_sequencer',
         '--ros-args', '-p', f'plan_file:={plan_file or PLAN}']
  # 계획 파일이 이 코스의 것인지 시퀀서가 대조할 수 있게 넘긴다.
  cmd += ['-p', f'waypoints_file:={YONGIN_WP}']
  for k, v in extra_params.items():
    cmd += ['-p', f'{k}:={v}']
  env = dict(os.environ)
  # ★ start_new_session — `ros2 run` 은 실제 노드를 **자식 프로세스**로 띄운다.
  #   Popen.terminate() 는 그 자식까지 못 죽여서, 케이스를 연달아 돌리면
  #   이전 것이 살아남아 같은 토픽에 반대 값을 쏜다(실제로 4개가 남아
  #   arm 이 10Hz 로 깜빡였다). 프로세스 그룹째 죽여야 한다.
  proc = subprocess.Popen(cmd, env=env, start_new_session=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True)
  _SPAWNED.append(proc)
  time.sleep(2.5)   # 노드 기동 대기

  # ★ 2026-09-16 — 케이스마다 **독립 컨텍스트**를 만든다.
  #   예전엔 전역 컨텍스트에 rclpy.init()/shutdown() 을 케이스마다 반복했는데,
  #   6번째 케이스쯤에서 컨텍스트가 망가져
  #     RCLError: failed to initialize wait set: the given context is not valid
  #   로 죽었다. ⑥번을 **단독으로** 돌리면 42.8초를 끝까지 돌고 전부 통과한다
  #   — 즉 ⑥의 문제가 아니라 케이스 간 전역 상태 오염이었다.
  #   독립 컨텍스트 + 독립 executor 면 케이스끼리 아무것도 공유하지 않는다.
  ctx = rclpy.Context()
  rclpy.init(context=ctx)
  h = Harness(load_path(plan), plan['missions'], respond, speed, context=ctx)
  ex = SingleThreadedExecutor(context=ctx)
  ex.add_node(h)
  t0 = time.time()
  try:
    while time.time() - t0 < duration:
      ex.spin_once(timeout_sec=0.02)
  except ExternalShutdownException:
    # 컨텍스트가 밖에서 내려가면 spin 이 이걸 던진다. 이 케이스의 결과는
    # 이미 h.log 에 모여 있으므로 조용히 빠져나와 정리만 제대로 한다.
    pass
  finally:
    # ★ 2026-09-16 — 정리 순서를 바꿨다. **이게 좀비의 출처였다.**
    #   예전 순서는 destroy_node → rclpy.shutdown → kill_group 이었는데,
    #   마지막 케이스에서 shutdown 이 'rcl_shutdown already called' 로 터지면
    #   **그 뒤의 kill_group 이 통째로 건너뛰어진다.** 그러면 시퀀서 노드가
    #   살아남아 다음 실행을 오염시킨다 — 같은 토픽에 두 벌이 서로 다른 값을
    #   쏘고, /lidar/arm 이 10Hz 로 ARM↔DISARM 깜빡인다.
    #   그 증상을 보고 '시퀀서 로직이 깨졌다' 고 오판했다(실제로는 멀쩡했다).
    #   → 자식 프로세스를 **먼저** 죽이고, rclpy 정리는 전부 예외를 삼킨다.
    kill_group(proc)
    try:
      ex.shutdown()
    except Exception:  # noqa: BLE001
      pass
    try:
      h.destroy_node()
    except Exception:  # noqa: BLE001
      pass
    try:
      if ctx.ok():
        rclpy.shutdown(context=ctx)
    except Exception:  # noqa: BLE001
      pass
    try:
      out = proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
      kill_group(proc, hard=True)
      out = ''
  return h, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--case', default='all',
                  choices=['all', 'normal', 'timeout', 'budget', 'inhibit',
                           'course'])
  ap.add_argument('--plan', default=PLAN,
                  help='로직 검증에 쓸 계획 파일 (기본: 픽스처)')
  args = ap.parse_args()
  plan = yaml.safe_load(open(args.plan, encoding='utf-8'))
  names = [m['name'] for m in plan['missions']]
  ex_names = [m['name'] for m in plan['missions'] if m.get('exclusive', True)]
  ov_names = [m['name'] for m in plan['missions']
              if not m.get('exclusive', True)]
  results = []

  if args.case in ('all', 'normal'):
    h, out = run_case(
        '① 정상 — 구간마다 arm → done → 다음',
        plan, respond={n: 3.0 for n in names},
        extra_params={'time_budget_s': 480.0}, duration=90)
    arms = [n for _, n, k in h.log if k == 'ARM']
    ex_arms = [n for n in arms if h.exclusive.get(n, True)]
    ov_arms = [n for n in arms if not h.exclusive.get(n, True)]
    ok1 = ex_arms == ex_names
    ok2 = h.max_concurrent <= 1
    ok3 = (not ov_names) or bool(ov_arms)
    ok4 = (not ov_names) or bool(h.overlap)
    print(f'\n  1) 구간 미션 순서대로 arm  {"✅" if ok1 else "❌"} '
          f'{ex_arms} (기대 {ex_names})')
    print(f'  2) 구간 미션 동시 최대 1개 {"✅" if ok2 else "❌"} '
          f'최대 {h.max_concurrent}개')
    print(f'  2b) 배경 기능 arm          {"✅" if ok3 else "❌"} {ov_arms}')
    print(f'  2c) 배경이 구간 미션을 안 막음 {"✅" if ok4 else "❌"} '
          f'공존 {len(h.overlap)}회')
    results += [ok1, ok2, ok3, ok4]

  if args.case in ('all', 'timeout'):
    # crosswalk 가 끝났다고 말하지 않는다 → 시퀀서가 강제로 풀어야 한다
    h, out = run_case(
        '② 무응답 — 타임아웃에 강제 복귀하는가 (1분 정지 = 탈락)',
        plan, respond={ex_names[0]: None,
                 **{n: 3.0 for n in names if n != ex_names[0]}},
        extra_params={'time_budget_s': 480.0}, duration=100)
    disarms = [(t, n) for t, n, k in h.log if k == 'DISARM']
    to = [t for t, n in disarms if n == ex_names[0]]
    tmo = float([m for m in plan['missions']
                 if m['name'] == ex_names[0]][0]['timeout_s'])
    ok = bool(to)
    held = (to[0] - h.armed_at.get(ex_names[0], 0.0)) if ok else -1
    print(f'\n  3) 타임아웃 강제 disarm  {"✅" if ok else "❌"} '
          f'{held:.1f}s 만에 해제 (설정 {tmo:.0f}s)')
    later = [n for t, n, k in h.log if k == 'ARM'
             and n != ex_names[0] and h.exclusive.get(n, True)]
    ok2 = bool(later)
    print(f'  4) 이후 미션 계속 진행   {"✅" if ok2 else "❌"} {later}')
    results += [ok, ok2]

  if args.case in ('all', 'budget'):
    # 예산을 바짝 줄여 감점-only 미션이 스킵되는지
    h, out = run_case(
        '③ 시간부족 — 감점-only 미션을 건너뛰는가 (완주 우선)',
        plan, respond={n: 3.0 for n in names},
        extra_params={'time_budget_s': 40.0, 'reserve_s': 20.0}, duration=70)
    skipped = '건너뜀' in out
    armed = [n for _, n, k in h.log if k == 'ARM'
             and h.exclusive.get(n, True)]
    ok = skipped or len(armed) < len(ex_names)
    print(f'\n  5) 예산부족 시 스킵      {"✅" if ok else "❌"} '
          f'arm 된 것 {armed}')
    if not ok:
      print(out[-1500:])
    results += [ok]

  if args.case in ('all', 'inhibit') and ov_names:
    # ★ 규정 항목 7 검증: "회피기동은 미션 성공으로 인정하지 않음".
    #   구간 미션이 inhibits 를 선언하면, 그 구간 동안 배경 기능(라이다 조향
    #   회피)이 **강제로 꺼져야** 한다. 계획 파일을 임시로 만들어 시험한다.
    import copy, tempfile
    plan2 = copy.deepcopy(plan)
    target = ex_names[0]
    for m in plan2['missions']:
      if m['name'] == target:
        m['inhibits'] = [ov_names[0]]
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False,
                                      encoding='utf-8')
    yaml.safe_dump(plan2, tmp, allow_unicode=True, sort_keys=False)
    tmp.close()
    h, out = run_case(
        f'④ 회피 금지 — {target} 구간에 {ov_names[0]} 가 강제로 꺼지는가',
        plan2, respond={n: 6.0 for n in names},
        extra_params={'time_budget_s': 480.0}, duration=60, plan_file=tmp.name)
    # target 이 켜져 있던 구간과 overlay 가 꺼져 있던 구간이 겹치는가
    ov = ov_names[0]
    tgt_on = [(t, k) for t, n, k in h.log if n == target]
    ov_off = [t for t, n, k in h.log if n == ov and k == 'DISARM']
    blocked = False
    if tgt_on and ov_off:
      t_arm = next((t for t, k in tgt_on if k == 'ARM'), None)
      t_dis = next((t for t, k in tgt_on if k == 'DISARM'), None)
      if t_arm is not None:
        end = t_dis if t_dis is not None else 1e9
        blocked = any(t_arm - 0.5 <= t <= end + 0.5 for t in ov_off)
    inhibit_log = '강제 차단' in out
    print(f'\n  6) 구간 중 배경 강제 차단  {"✅" if blocked else "❌"} '
          f'{ov} DISARM @ {[round(t,1) for t in ov_off]}')
    print(f'  7) 차단 로그 남김        {"✅" if inhibit_log else "❌"}')
    if not blocked:
      print(out[-1200:])
    results += [blocked, inhibit_log]
    os.unlink(tmp.name)

  if args.case in ('all', 'course'):
    # ★ 2026-09-10 — 탈락급 오류 방지.
    #   대구(184m) 계획으로 용인(648m) 코스를 달리면 후진주차 트리거
    #   s_enter=175m 가 용인 코스 한복판에 떨어진다. 주행 중간에 후진 기동이
    #   시작되고 그건 이탈 = 탈락이다. 시퀀서가 코스를 대조해 **전부 건너뛰어야**
    #   한다. 미션 미실행은 감점이고, 엉뚱한 곳에서의 실행은 탈락이다.
    h, out = run_case(
        '⑤ 코스 대조 — 다른 장소의 계획이면 아무것도 arm 하지 않는가',
        plan, respond={n: 3.0 for n in names},
        extra_params={'time_budget_s': 480.0}, duration=45,
        plan_file=DAEGU_PLAN)
    armed_any = [n for _, n, k in h.log if k == 'ARM']
    ok1 = not armed_any
    ok2 = '이 코스의 것이 아니다' in out
    print(f'\n  8) 불일치 시 arm 없음    {"✅" if ok1 else "❌"} '
          f'{armed_any if armed_any else "(하나도 안 켬)"}')
    print(f'  9) 불일치를 로그로 알림  {"✅" if ok2 else "❌"}')
    if not (ok1 and ok2):
      print(out[-1500:])
    results += [ok1, ok2]

    # 배경 기능이 코스 전체를 덮으면 거부하는가 (관중·연석에 조향을 뺏김 = 이탈)
    import copy, tempfile
    plan3 = copy.deepcopy(plan)
    for m in plan3['missions']:
      if not m.get('exclusive', True):
        m['trigger'] = {'type': 'course_s', 's_enter': 0.0, 's_exit': 9999.0}
    tmp3 = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False,
                                       encoding='utf-8')
    yaml.safe_dump(plan3, tmp3, allow_unicode=True, sort_keys=False)
    tmp3.close()
    h, out = run_case(
        '⑥ 배경 기능이 코스 전체(0~9999)를 덮으면 거부하는가',
        plan3, respond={n: 3.0 for n in names},
        extra_params={'time_budget_s': 480.0}, duration=40, plan_file=tmp3.name)
    ov_armed = [n for _, n, k in h.log
                if k == 'ARM' and not h.exclusive.get(n, True)]
    ok3 = not ov_armed
    ok4 = '코스의' in out and '건너뜀' in out
    print(f'\n 10) 전 구간 배경 거부     {"✅" if ok3 else "❌"} '
          f'{ov_armed if ov_armed else "(안 켬)"}')
    print(f' 11) 거부 사유 로그        {"✅" if ok4 else "❌"}')
    if not (ok3 and ok4):
      print(out[-1500:])
    results += [ok3, ok4]
    os.unlink(tmp3.name)

  print(f'\n{"=" * 66}')
  print('결론: ' + ('미션 시퀀서 정상 ✅' if all(results)
                    else '❌ 실패 항목 있음'))
  sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
  main()

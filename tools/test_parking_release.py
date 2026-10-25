#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_parking_release.py — 주차가 끝나거나 취소되면 /teleop/cmd_vel 을 **놓는가** (FMA-001).

★ 이 시험이 지키려는 것
  mux 는 신선한 teleop 을 자율보다 우선한다. 주차 노드가 0 을 계속 내면 차는
  그 자리에 선다 = 1분 정지 = 탈락. 시퀀서가 disarm 하면 반드시 놓아야 한다.
  반대로 **손 트리거(--once)·완주 트리거 경로는 예전과 똑같아야** 한다(데드맨).

  parking_node 의 실제 메서드(arm_cb, goal_cb, start_cb, begin, tick, release,
  reset_cb, _owns_control)를 Stub self 로 부른다. 기하(planner/follower)는
  가짜로 바꾼다 — 기하는 test_parking_geometry / test_parking_roundtrip 몫이다.
  기존/신규 비교는 `git show BASE:`(수정 직전 커밋) 로 예전 파일을 불러 같은 입력을 넣는다.

  python tools/test_parking_release.py
"""

import importlib.util
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'mission_perception'))

import ros_stub  # noqa: E402

ros_stub.install()

from std_msgs.msg import Bool  # noqa: E402

from mission_perception import parking_node as NEW  # noqa: E402
from mission_perception import parking_planner as PP  # noqa: E402

# ★ 비교 기준은 **수정 직전 커밋으로 고정**한다. HEAD 로 하면 이 수정이 커밋된 뒤
#   새 코드끼리 비교하게 되어 시험이 의미를 잃는다(시작 실패 비교는 아예 깨진다).
BASE = '375250591a592fe0026931884e6f9a57abba1184'
REL = 'src/mission_perception/mission_perception/parking_node.py'


def load_old():
  src = subprocess.run(['git', 'show', f'{BASE}:{REL}'], cwd=ROOT,
                       capture_output=True, check=True).stdout
  d = tempfile.mkdtemp(prefix='parking_old_')
  p = os.path.join(d, 'parking_node_old.py')
  with open(p, 'wb') as f:
    f.write(src)
  spec = importlib.util.spec_from_file_location('parking_node_old', p)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


OLD = load_old()

# ---- 가짜 기하 ------------------------------------------------------------
PLAN_OK = {'points': [(0.0, 0.0, 1), (1.0, 0.0, 1), (2.0, 0.0, -1)],
           'word': 'LSL', 'd_back': 0.0, 'fwd_len': 1.0, 'rev_len': 1.0,
           'turn_deg': 0.0, 'r_rev': 3.0}
_plan = {'ret': PLAN_OK}
PP.plan_parking = lambda *a, **k: _plan['ret']


class FakeFollower:
  """상태만 조종하는 follower. 주행 중엔 (0.3, 5.0) 을 낸다."""

  def __init__(self):
    self.state = 'DRIVE'
    self.status = ''

  def update(self, pose, v, t):
    return (0.3, 5.0) if self.state not in ('DONE', 'ABORT') else (0.0, 0.0)

  def drain_events(self):
    return []


class Rec:

  def __init__(self):
    self.msgs = []

  def publish(self, m):
    self.msgs.append(m)


class Log:

  def info(self, *a, **k):
    pass

  warn = error = warning = debug = info


class Clock:
  t = 100.0


def make(mod, **kw):
  cls = mod.ParkingNode
  S = type('S', (), {k: v for k, v in cls.__dict__.items()
                     if callable(v) and k != '__init__'})
  o = S()
  clk = Clock()
  o.clk = clk
  o._now = lambda: clk.t
  o.get_logger = lambda: Log()
  o._make_follower = lambda segs: FakeFollower()
  o.slot = 3
  o.goals = {3: ((5.0, 0.0), 0.0)}
  o.pose = (0.0, 0.0, 0.0)
  o.linear_speed = 0.0
  o.follower = None
  o.plan = None
  o.state = 'WAIT'
  o._started = False
  o._arm_level = False
  o._release_since = None
  o.exit_goal = None
  o.exit_started = False
  o.exit_enabled = True
  o.release_hold = 1.0
  o.release_max = 3.0
  o.stopped_v = 0.05
  o.corridor = None
  o.plan_r_fwd = 3.0
  o.plan_r_rev = (2.9, 5.0, 0.35)
  o.plan_back = 3.0
  o.plan_spacing = 0.2
  o.pub = Rec()
  o.done_pub = Rec()
  o.state_pub = Rec()
  for k, v in kw.items():
    setattr(o, k, v)
  return o


def arm(o, val):
  """/parking/start — 신규는 arm_cb, 예전은 start_cb 가 받았다."""
  cb = getattr(o, 'arm_cb', None) or o.start_cb
  cb(Bool(data=val))


def goal(o, val=True):
  cb = getattr(o, 'goal_cb', None) or o.start_cb
  cb(Bool(data=val))


def tick(o, n=1, dt=0.05):
  for _ in range(n):
    o.tick()
    o.clk.t += dt


def teleop(o):
  return [(m.linear.x, m.angular.z) for m in o.pub.msgs]


def done(o):
  return [m.data for m in o.done_pub.msgs]


FAILS = []


def check(name, cond, detail=''):
  print(f'  [{"PASS" if cond else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
  if not cond:
    FAILS.append(name)


def drive_to_released(o):
  """RELEASED 가 될 때까지 tick, 걸린 시간(s)."""
  t0 = o.clk.t
  for _ in range(200):
    tick(o)
    if o.state == 'RELEASED':
      return o.clk.t - t0
  return None


# ---------------------------------------------------------------------------
def t1_sequencer_done():
  print('1. 시퀀서 패턴: False… → True… → DONE → False')
  o = make(NEW)
  for _ in range(10):
    arm(o, False)
    tick(o)
  check('arm 전 False 만으로는 아무것도 안 낸다', not teleop(o) and o.state == 'WAIT')
  for _ in range(10):
    arm(o, True)
    tick(o)
  check('True 로 주행 시작', o.state == 'DRIVE' and teleop(o)[-1] == (0.3, 5.0))
  o.follower.state = 'DONE'
  for _ in range(5):
    arm(o, True)
    tick(o)
  check('DONE 에서 0 + done True', o.state == 'DONE' and teleop(o)[-1] == (0.0, 0.0)
        and done(o)[-1] is True)
  n_before = len(teleop(o))
  arm(o, False)
  check('하강엣지 → RELEASING', o.state == 'RELEASING')
  took = drive_to_released(o)
  rel_cmds = teleop(o)[n_before:]
  check('RELEASING 동안 0 만 낸다', rel_cmds and all(c == (0.0, 0.0) for c in rel_cmds))
  check('정지 상태면 release_hold(1.0s) 뒤 RELEASED', took is not None and 1.0 <= took <= 1.11,
        f'{took}')
  nt, nd = len(teleop(o)), len(done(o))
  for _ in range(40):
    arm(o, False)
    tick(o)
  check('RELEASED 뒤 teleop·done 0건', len(teleop(o)) == nt and len(done(o)) == nd)
  check('_owns_control() False', not o._owns_control())


def t2_cancel_mid_drive():
  print('2. DRIVE 중 disarm (시퀀서 타임아웃), 속도가 안 줄어듦')
  o = make(NEW)
  arm(o, True)
  tick(o, 5)
  o.linear_speed = 0.5
  arm(o, False)
  took = drive_to_released(o)
  check('release_max(3.0s) 에 RELEASED', took is not None and 3.0 <= took <= 3.11, f'{took}')


def t3_start_failures():
  print('3. 시작 실패 세 가지 (실제 _trigger→begin 경로)')
  cases = {
      '계획 실패': dict(),
      'pose 파일 없음': dict(goals={}),
      '측위 미수신': dict(pose=None),
  }
  for name, kw in cases.items():
    _plan['ret'] = None if name == '계획 실패' else PLAN_OK
    o = make(NEW, **kw)
    arm(o, True)
    tick(o, 5)
    check(f'{name}: ABORT, follower 없음', o.state == 'ABORT' and o.follower is None)
    check(f'{name}: done False, teleop 0건', done(o) and all(d is False for d in done(o))
          and not teleop(o))
    arm(o, False)
    nd = len(done(o))
    tick(o, 10)
    check(f'{name}: 하강엣지 → 바로 RELEASED, teleop 0건, done 멈춤',
          o.state == 'RELEASED' and not teleop(o) and len(done(o)) == nd)
  _plan['ret'] = PLAN_OK


def t4_false_only():
  print('4. arm 전 False 만 계속')
  o = make(NEW)
  for _ in range(50):
    arm(o, False)
    tick(o)
  check('WAIT, 발행 0건', o.state == 'WAIT' and not teleop(o) and not done(o))


def t5_goal_trigger_with_false_level():
  print('5. /goal_reached 트리거 + /parking/start False 수준')
  o = make(NEW)
  for _ in range(5):
    arm(o, False)
    tick(o)
  goal(o, True)
  for _ in range(20):
    arm(o, False)
    goal(o, True)
    tick(o)
  check('해제되지 않고 DRIVE 유지', o.state == 'DRIVE' and teleop(o)[-1] == (0.3, 5.0))


def run_seq(mod, seq):
  o = make(mod)
  for step in seq:
    if step == 'arm':
      arm(o, True)
    elif step == 'goal':
      goal(o, True)
    elif step == 'done':
      o.follower.state = 'DONE'
    elif step == 'abort':
      o.follower.state = 'ABORT'
    tick(o)
  return o


def t6_old_vs_new():
  print('6. 기존/신규 비교 — 하강엣지 없는 입력')
  seqs = {
      '--once True → DONE': ['arm'] + ['-'] * 20 + ['done'] + ['-'] * 40,
      'goal_reached 반복 → DONE': ['goal'] * 20 + ['done'] + ['goal'] * 40,
      '--once True → ABORT': ['arm'] + ['-'] * 10 + ['abort'] + ['-'] * 30,
  }
  for name, seq in seqs.items():
    a, b = run_seq(OLD, seq), run_seq(NEW, seq)
    check(f'{name}: teleop 열 동일', teleop(a) == teleop(b), f'{len(teleop(a))}개')
    check(f'{name}: done 열 동일', done(a) == done(b), f'{len(done(a))}개')
    check(f'{name}: 끝까지 0 유지(데드맨)', teleop(b)[-1] == (0.0, 0.0))
  # A7 의 예외 — 시작 실패에서만 done 이 다르다 (예전: 아무것도 안 냄)
  a = make(OLD, goals={})
  b = make(NEW, goals={})
  arm(a, True)
  arm(b, True)
  tick(a, 5)
  tick(b, 5)
  check('예외(시작 실패): 예전 done 0건 → 신규 done False, teleop 은 둘 다 0건',
        not done(a) and done(b) and not teleop(a) and not teleop(b))


def t7_reset():
  print('7. reset')
  o = make(NEW)
  arm(o, True)
  tick(o, 3)
  o.linear_speed = 0.5
  arm(o, False)
  tick(o)
  o.reset_cb(Bool(data=True))
  check('RELEASING 중 reset 무시', o.state == 'RELEASING')
  drive_to_released(o)
  o.reset_cb(Bool(data=True))
  check('RELEASED 에서 reset → WAIT, 래치 해제', o.state == 'WAIT' and not o._started)


def t8_owns_control():
  print('8. _owns_control()')
  o = make(NEW)
  check('WAIT → False', not o._owns_control())
  arm(o, True)
  tick(o)
  check('DRIVE → True', o._owns_control())
  o.follower.state = 'DONE'
  tick(o)
  check('DONE → True', o.state == 'DONE' and o._owns_control())
  arm(o, False)
  check('RELEASING → True', o.state == 'RELEASING' and o._owns_control())
  drive_to_released(o)
  check('RELEASED → False', not o._owns_control())
  f = make(NEW, goals={})
  arm(f, True)
  tick(f)
  check('follower 없는 ABORT → False', f.state == 'ABORT' and not f._owns_control())


def t9_idempotent():
  print('9. 반납 멱등성')
  o = make(NEW)
  arm(o, True)
  tick(o, 3)
  o.linear_speed = 0.5
  arm(o, False)
  t_rel = o._release_since
  for _ in range(5):
    tick(o, 10)
    arm(o, True)
    arm(o, False)
  check('RELEASING 중 엣지 반복 → _release_since 불변', o._release_since == t_rel)
  took = drive_to_released(o)
  total = o.clk.t - t_rel
  check('그래도 3s 상한에 RELEASED', o.state == 'RELEASED' and 3.0 <= total <= 3.11,
        f'{total:.2f}s')
  nt = len(teleop(o))
  for _ in range(3):
    arm(o, True)
    tick(o, 2)
    arm(o, False)
    tick(o, 5)
  check('RELEASED 뒤 True→False → teleop 0건, 상태 유지',
        len(teleop(o)) == nt and o.state == 'RELEASED')


if __name__ == '__main__':
  for t in (t1_sequencer_done, t2_cancel_mid_drive, t3_start_failures, t4_false_only,
            t5_goal_trigger_with_false_level, t6_old_vs_new, t7_reset,
            t8_owns_control, t9_idempotent):
    t()
  print()
  if FAILS:
    print(f'FAIL {len(FAILS)}: {FAILS}')
    sys.exit(1)
  print('ALL PASS')

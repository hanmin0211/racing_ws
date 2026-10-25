#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_stop_line_lease.py — 묵은 /stop_line_distance 를 버리는가 (FMA-003).

★ 이 시험이 지키려는 것
  ① 정지 노드가 HOLD(0) 중에 죽으면 stop_line_timeout(1.0s) 뒤 제약이 풀린다
  ② 10Hz 로 계속 오는 정상 HOLD 는 **안 풀린다**
  ③ 신선한 입력에서는 예전 코드와 /target_speed 가 **완전히 같다**
  ④ 벽시계가 뒤로 가도 만료가 늦어지지 않는다(monotonic)

  longitudinal_controller_node 의 실제 control_loop/decide_target/stop_cb/
  _expire_stop_line 을 Stub self 로 부른다. 시간은 모듈의 `time` 을 바꿔 주입한다.
  기존/신규 비교는 `git show BASE:`(수정 직전 커밋) 로 예전 파일을 불러 같은 입력을 넣는다.

  python tools/test_stop_line_lease.py
"""

import importlib.util
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'velocity_controller'))

import ros_stub  # noqa: E402

ros_stub.install()

from std_msgs.msg import Float64  # noqa: E402

from velocity_controller import longitudinal_controller_node as NEW  # noqa: E402

# ★ 비교 기준은 **수정 직전 커밋으로 고정**한다. HEAD 로 하면 이 수정이 커밋된 뒤
#   새 코드끼리 비교하게 되어 시험이 의미를 잃는다(시작 실패 비교는 아예 깨진다).
BASE = '375250591a592fe0026931884e6f9a57abba1184'
REL = 'src/velocity_controller/velocity_controller/longitudinal_controller_node.py'


def load_old():
  src = subprocess.run(['git', 'show', f'{BASE}:{REL}'], cwd=ROOT,
                       capture_output=True, check=True).stdout
  d = tempfile.mkdtemp(prefix='longi_old_')
  p = os.path.join(d, 'longi_old.py')
  with open(p, 'wb') as f:
    f.write(src)
  spec = importlib.util.spec_from_file_location('longi_old', p)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


OLD = load_old()


class FakeTime:
  """벽시계와 monotonic 을 따로 움직일 수 있다."""

  def __init__(self):
    self.wall = 1.79e9
    self.mono = 5000.0

  def time(self):
    return self.wall

  def monotonic(self):
    return self.mono

  def step(self, dt):
    self.wall += dt
    self.mono += dt


class Rec:

  def __init__(self):
    self.msgs = []

  def publish(self, m):
    self.msgs.append(m)


class Log:

  def __init__(self):
    self.warns = []

  def warn(self, msg, **k):
    self.warns.append(msg)

  def info(self, *a, **k):
    pass

  warning = warn


def make(mod, **kw):
  cls = mod.LongitudinalController
  S = type('S', (), {k: v for k, v in cls.__dict__.items()
                     if callable(v) and k != '__init__'})
  o = S()
  ft = FakeTime()
  mod.time = ft          # 모듈 전역 time 을 바꾼다 — 노드 코드가 time.xxx() 로 부른다
  o.ft = ft
  log = Log()
  o.log = log
  o.get_logger = lambda: log
  # 노드 기본값과 같게
  o.v_max, o.v_min, o.v_slow = 1.0, 0.25, 0.8
  o.v_slalom, o.v_reverse = 0.6, -0.5
  o.curv_gain = 6.0
  o.max_accel, o.max_decel = 1.0, 1.8
  o.stop_trigger, o.stop_decel = 19.0, 1.0
  o.obs_trigger, o.obs_stop = 4.0, 0.8
  o.curv_timeout = 0.5
  o.dt = 0.05
  o.kappa = 0.0
  o.current_speed = 0.0
  o.stop_line_dist = 999.0
  o._stop_t = None
  o.stop_line_timeout = 1.0
  o.obstacle_dist = 999.0
  o.mission = 'DRIVE'
  o.profiled_speed = 0.0
  o.last_curv_time = None
  o.goal_reached = False
  o.v_ramp_up = o.v_ramp_down = 0.0
  o.ramp_arm_timeout = 1.0
  o._ramp_up_on = o._ramp_down_on = False
  o._ramp_up_t = o._ramp_down_t = 0.0
  o.pub = Rec()
  for k, v in kw.items():
    setattr(o, k, v)
  return o


def curv(o, k=0.0):
  o.curvature_cb(Float64(data=k))


def stop(o, d):
  o.stop_cb(Float64(data=d))


def loop(o, n=1, dt=0.05, k=0.0):
  """곡률은 매 tick 신선하게 — 경로는 살아 있고 정지선만 본다."""
  for _ in range(n):
    curv(o, k)
    o.control_loop()
    o.ft.step(dt)


def last(o):
  return o.pub.msgs[-1].data


FAILS = []


def check(name, cond, detail=''):
  print(f'  [{"PASS" if cond else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
  if not cond:
    FAILS.append(name)


def hold_then_die(o):
  """달리다가 정지선 HOLD(0) 를 한 번 받고 발행자가 죽는다."""
  loop(o, 40)               # 2s 가속
  stop(o, 0.0)
  loop(o, 1)


def t1_expiry():
  print('1. HOLD(0) 후 발행자 죽음')
  o = make(NEW)
  hold_then_die(o)
  check('직후 정지', last(o) <= o.profiled_speed and o.stop_line_dist == 0.0)
  loop(o, 16)               # 수신 후 ~0.85s
  check('0.9s 미만: 아직 0 유지', o.stop_line_dist == 0.0 and not o.log.warns)
  loop(o, 6)                # ~1.15s
  check('1.0s 초과: 999 로 복귀', o.stop_line_dist == 999.0)
  check('경고 1회', len(o.log.warns) == 1, str(len(o.log.warns)))
  loop(o, 60)
  check('다시 가속', last(o) > 0.5, f'{last(o):.2f}')
  check('경고 여전히 1회', len(o.log.warns) == 1)


def t2_steady_hold():
  print('2. 10Hz 로 0 이 계속 옴 (정상 HOLD)')
  o = make(NEW)
  loop(o, 40)
  for i in range(200):      # 10s, loop 20Hz, stop 10Hz
    if i % 2 == 0:
      stop(o, 0.0)
    loop(o, 1)
  check('만료 없음, 목표속도 0', o.stop_line_dist == 0.0 and last(o) == 0.0
        and not o.log.warns)


def t3_normal_disarm():
  print('3. 정상 disarm: 999 한 번 후 무수신')
  o = make(NEW)
  loop(o, 10)
  stop(o, 999.0)
  loop(o, 60)
  check('경고 없음', not o.log.warns and o.stop_line_dist == 999.0)


def t4_disabled():
  print('4. stop_line_timeout=0 → 예전 동작')
  o = make(NEW, stop_line_timeout=0.0)
  hold_then_die(o)
  loop(o, 200)
  check('10s 지나도 0 유지', o.stop_line_dist == 0.0 and last(o) == 0.0)


def t5_old_vs_new():
  print('5. 기존/신규 비교 — 신선한 입력')
  kappas = [0.0, 0.05, 0.16, 0.37, 0.1]
  # 25m 에서 정지선까지 접근 → 3s HOLD → 999 로 해제 → 다시 주행
  dists = ([25.0 - 0.06 * i for i in range(420)] + [0.0] * 60 + [999.0] * 100)
  outs = {}
  for name, mod in (('old', OLD), ('new', NEW)):
    o = make(mod)
    for i, d in enumerate(dists):
      if i % 2 == 0:        # stop 10Hz / loop 20Hz — 둘 다 신선
        stop(o, max(d, 0.0))
      loop(o, 1, k=kappas[(i // 50) % len(kappas)])
    outs[name] = [m.data for m in o.pub.msgs]
  same = outs['old'] == outs['new']
  check('/target_speed 열이 완전히 같다', same, f'{len(outs["new"])}개')
  check('그 열 안에 0 정지 구간이 있다(시험이 실제로 정지를 탐)',
        0.0 in outs['new'] and max(outs['new']) > 0.5)


def t6_wall_clock_backwards():
  print('6. 벽시계 역행 (NTP/GPS 보정)')
  o = make(NEW)
  hold_then_die(o)
  o.ft.wall -= 3600.0       # 1시간 뒤로. monotonic 은 그대로 흐른다
  # 곡률 freshness 는 기존 time.time() 기준이라 역행 후 '미래 시각' 이 된다 — 신선으로 본다.
  loop(o, 23)               # ~1.15s
  check('monotonic 기준 1.1s 에 만료', o.stop_line_dist == 999.0)


if __name__ == '__main__':
  for t in (t1_expiry, t2_steady_hold, t3_normal_disarm, t4_disabled,
            t5_old_vs_new, t6_wall_clock_backwards):
    t()
  print()
  if FAILS:
    print(f'FAIL {len(FAILS)}: {FAILS}')
    sys.exit(1)
  print('ALL PASS')

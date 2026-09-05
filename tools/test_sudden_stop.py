#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_sudden_stop.py — 돌발 급정지 미션을 폐루프로 검증한다 (하드웨어 없이).

가짜 차량이 달리다 더미를 만난다. sudden_stop_node 가 내는
/stop_line_distance 를 longitudinal_controller 와 **같은 식**으로 속도에 반영한다.

  가짜차량 ──/obstacle_distance, /odometry/filtered──→ sudden_stop_node
      ↑                                                      │
      └── stop_line≤0.3 또는 obstacle≤0.8 이면 v=0 ──/stop_line_distance

검증하는 것
  ① 더미가 치워지는 경우
     1. 완전정지하는가
     2. **정확히 5초** 유지하는가
     3. 치워지면 곧바로 재출발하는가
  ② 더미가 안 치워지는 경우 (탈락 방지 경로)
     4. dwell + clear_timeout 뒤 /lidar/mute 를 켜고 빠져나오는가
     5. 통과 후 mute 를 다시 끄는가

  python3 tools/test_sudden_stop.py

⚠ 로직 검증이다. 실제 제동거리·라이다 검출은 실차로 확인해야 한다.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('SUDDEN_TEST_DOMAIN', '83')

import rclpy                                       # noqa: E402
from nav_msgs.msg import Odometry                  # noqa: E402
from rclpy.node import Node                        # noqa: E402
from std_msgs.msg import Bool, Float64, String     # noqa: E402

DUMMY_X = 20.0        # 더미 위치 [m]
CRUISE = 0.7          # 순항 속도 [m/s]
OBS_STOP = 0.8        # longitudinal 의 obstacle_stop_dist
LIDAR_RANGE = 8.0     # 이 밖은 '아무것도 없음'
DWELL = 5.0
CLEAR_TIMEOUT = 4.0   # 시험용으로 줄인다(기본 10)
PASS_DURATION = 3.0


class Car(Node):
  """가짜 차량 + longitudinal 축약판."""

  def __init__(self, remove_at):
    super().__init__('sudden_stop_test_car')
    self.x = 0.0
    self.v = 0.0
    self.remove_at = remove_at      # 정지 후 몇 초에 더미를 치울지 (None=안 치움)
    self.stop_line = 999.0
    self.removed = False
    self.t0 = time.time()
    self.stopped_since = None
    self.states = []                # (t, state)
    self.mutes = []                 # (t, bool)
    self.last_state = None

    self.obs_pub = self.create_publisher(Float64, '/obstacle_distance', 10)
    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.create_subscription(Float64, '/stop_line_distance', self.stop_cb, 10)
    self.create_subscription(String, '/sudden_stop/state', self.state_cb, 10)
    self.create_subscription(Bool, '/lidar/mute', self.mute_cb, 10)
    self.create_timer(0.05, self.step)

  def stop_cb(self, m):
    self.stop_line = float(m.data)

  def state_cb(self, m):
    if m.data != self.last_state:
      self.last_state = m.data
      t = time.time() - self.t0
      self.states.append((t, m.data))
      print(f'  [{t:6.2f}s] state → {m.data}   (x={self.x:.2f} v={self.v:.2f})')

  def mute_cb(self, m):
    v = bool(m.data)
    if not self.mutes or self.mutes[-1][1] != v:
      t = time.time() - self.t0
      self.mutes.append((t, v))
      print(f'  [{t:6.2f}s] /lidar/mute → {v}')

  def obstacle(self):
    if self.removed:
      return 999.0
    d = DUMMY_X - self.x
    return d if 0.0 < d <= LIDAR_RANGE else 999.0

  def step(self):
    t = time.time() - self.t0
    obs = self.obstacle()

    # 더미 치우기 — '완전히 선 뒤' 기준으로 센다
    if self.stopped_since is not None and self.remove_at is not None \
            and not self.removed \
            and (time.time() - self.stopped_since) >= self.remove_at:
      self.removed = True
      print(f'  [{t:6.2f}s] ── 더미 치움 ──')

    # longitudinal 축약: 두 제약 중 보수적인 값
    v = CRUISE
    if self.stop_line < 19.0:
      v = min(v, (2.0 * 1.0 * max(0.0, self.stop_line)) ** 0.5)
      if self.stop_line <= 0.3:
        v = 0.0
    if obs < 4.0:
      v = min(v, CRUISE * max(0.0, obs - OBS_STOP) / (4.0 - OBS_STOP))
      if obs <= OBS_STOP:
        v = 0.0
    self.v = v
    self.x += v * 0.05
    if v < 0.02 and self.stopped_since is None:
      self.stopped_since = time.time()
    elif v >= 0.02:
      self.stopped_since = None

    o = Odometry()
    o.header.frame_id = 'map'
    o.pose.pose.position.x = float(self.x)
    o.pose.pose.orientation.w = 1.0
    o.twist.twist.linear.x = float(self.v)
    self.odom_pub.publish(o)
    self.obs_pub.publish(Float64(data=float(self.obstacle())))


def kill_group(proc):
  try:
    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
  except (ProcessLookupError, PermissionError):
    pass


def run(title, remove_at, duration):
  print(f'\n{"=" * 66}\n  {title}\n{"=" * 66}')
  cmd = ['ros2', 'run', 'mission_perception', 'sudden_stop_node', '--ros-args',
         '-p', f'dwell:={DWELL}',
         '-p', f'clear_timeout:={CLEAR_TIMEOUT}',
         '-p', f'pass_duration:={PASS_DURATION}',
         '-p', 'trigger_dist:=1.2']
  proc = subprocess.Popen(cmd, start_new_session=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True)
  time.sleep(2.5)
  rclpy.init()
  car = Car(remove_at)
  t0 = time.time()
  try:
    while time.time() - t0 < duration:
      rclpy.spin_once(car, timeout_sec=0.02)
  finally:
    car.destroy_node()
    rclpy.shutdown()
    kill_group(proc)
    try:
      proc.communicate(timeout=3)
    except subprocess.TimeoutExpired:
      pass
  return car


def at(states, name):
  for t, s in states:
    if s == name:
      return t
  return None


def main():
  argparse.ArgumentParser().parse_args()
  ok = []

  # ── ① 더미가 치워지는 경우 ──
  car = run('① 더미가 치워지는 경우 — 정지 → 5초 → 재출발',
            remove_at=DWELL + 1.0, duration=55)
  t_hold = at(car.states, 'HOLD')
  t_wait = at(car.states, 'WAIT_CLEAR')
  t_clear = at(car.states, 'CLEARED')
  stopped = t_hold is not None
  held = (t_wait - t_hold) if (t_hold and t_wait) else -1
  # HOLD 진입 직후 완전정지까지 약간 걸리므로 dwell 은 그 뒤부터다 → 여유 1.5s
  dwell_ok = t_wait is not None and DWELL <= held <= DWELL + 1.5
  resumed = t_clear is not None and car.v > 0.1
  print(f'\n  1) 완전정지          {"✅" if stopped else "❌"}')
  print(f'  2) {DWELL:.0f}초 유지        '
        f'{"✅" if dwell_ok else "❌"} 실제 {held:.2f}s '
        f'(정지 확인까지의 지연 포함)')
  print(f'  3) 치워지자 재출발    {"✅" if resumed else "❌"} '
        f'v={car.v:.2f}m/s x={car.x:.1f}m')
  ok += [stopped, dwell_ok, resumed]

  # ── ② 안 치워지는 경우 ──
  car = run('② 안 치워지는 경우 — mute 로 빠져나오는가 (1분 정지 = 탈락)',
            remove_at=None, duration=60)
  t_pass = at(car.states, 'PASSING')
  t_clear = at(car.states, 'CLEARED')
  muted_on = any(v for _, v in car.mutes)
  muted_off = len(car.mutes) >= 2 and car.mutes[-1][1] is False
  escaped = t_pass is not None and t_clear is not None
  print(f'\n  4) 막혔을 때 통과     {"✅" if escaped else "❌"} '
        f'PASSING@{t_pass if t_pass else -1:.1f}s → '
        f'CLEARED@{t_clear if t_clear else -1:.1f}s')
  print(f'  5) mute 켜고 다시 끔  '
        f'{"✅" if (muted_on and muted_off) else "❌"} {car.mutes}')
  ok += [escaped, muted_on and muted_off]

  print(f'\n{"=" * 66}')
  print('결론: ' + ('돌발 급정지 로직 정상 ✅' if all(ok) else '❌ 실패 항목 있음'))
  sys.exit(0 if all(ok) else 1)


if __name__ == '__main__':
  main()

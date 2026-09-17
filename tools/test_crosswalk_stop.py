#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_crosswalk_stop.py — 횡단보도 정지 미션을 폐루프로 검증한다 (하드웨어 없이).

가짜 차량이 정지선을 향해 달리고, crosswalk_stop_node 가 내는
/stop_line_distance 를 longitudinal_controller 와 **똑같은 식**으로 속도에 반영한다.

  가짜차량 ──/odometry/filtered──→ crosswalk_stop_node
      ↑                                    │
      └──── v=√(2ad), d≤0.3 이면 0 ────/stop_line_distance

검증하는 것 (대회 규정):
  1. 정지선 64cm 안에 서는가
  2. 정지 후 3초(±0.5s) 대기하는가
  3. 대기 후 다시 출발하는가
  4. 지나간 정지선에 다시 걸리지 않는가

  python3 tools/test_crosswalk_stop.py

⚠ 이건 로직 검증이다. 실제 제동거리·GPS 오차는 실차 주행으로 확인해야 한다.
"""

import math
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

# ★ 실차 스택과 격리한다.
#   bringup 이 떠 있으면 direct_localization 도 /odometry/filtered 를 발행해서
#   가짜 차량과 충돌한다(실제로 노드가 실차 위치를 보고 "궤적에서 34m 벗어남"
#   으로 중단했다). 별도 ROS_DOMAIN_ID 를 쓰면 스택을 켜둔 채로 시험할 수 있다.
TEST_DOMAIN = os.environ.get('PARKING_TEST_DOMAIN', '77')
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN

STOP_X, STOP_Y = 20.0, 0.0        # 정지선 위치
V_MAX = 0.7
STOP_DECEL = 1.0                  # longitudinal_controller 기본값
MAX_ACCEL, MAX_DECEL = 1.0, 1.8
TOLERANCE = 0.64
DWELL = 3.0


# ─────────────────────────────────────────────────────────────────────
# ★ 2026-09-18 — SIGTERM 에서도 자식을 반드시 죽인다.
#   `timeout N python3 ...` 로 돌리다 상한에 걸리면 SIGTERM 이 오는데,
#   파이썬은 SIGTERM 에서 **finally 를 안 돌리고 즉사**한다. 그러면
#   start_new_session=True 로 띄운 `ros2 run` 자식이 살아남아 다음 실행을
#   오염시킨다(같은 토픽에 두 벌 → arm 이 10Hz 로 깜빡임).
#   이 저장소에서 세 번 그랬다. SIGTERM 을 예외로 바꿔 finally 가 돌게 하고,
#   그래도 새면 atexit 이 한 번 더 쓸어낸다.
import atexit as _atexit
import signal as _sig

_SPAWNED = []


def _reap_all():
  for p in _SPAWNED:
    try:
      os.killpg(os.getpgid(p.pid), _sig.SIGKILL)
    except Exception:  # noqa: BLE001
      pass


def _on_term(_signum, _frame):
  raise KeyboardInterrupt('SIGTERM')


_sig.signal(_sig.SIGTERM, _on_term)
_atexit.register(_reap_all)
# ─────────────────────────────────────────────────────────────────────


def _kill_group(proc):
  """ros2 run 래퍼와 그 자식 노드를 통째로 죽인다.

  ★ proc.terminate() 만 하면 래퍼만 죽고 **실제 노드는 살아남는다.**
    살아남은 노드가 다음 테스트에서 같은 토픽에 끼어들어 결과가 뒤섞인다
    (실제로 좀비 parking_node 6개가 쌓여 DONE/CUSP 가 동시에 찍혔다).
  """
  import os as _os
  import signal as _signal
  try:
    _os.killpg(_os.getpgid(proc.pid), _signal.SIGTERM)
    proc.wait(timeout=5)
  except Exception:  # noqa: BLE001
    try:
      _os.killpg(_os.getpgid(proc.pid), _signal.SIGKILL)
    except Exception:  # noqa: BLE001
      pass


class FakeVehicle(Node):
  """정지선을 향해 직진하는 가짜 차량 + longitudinal_controller 속도식 복제."""

  def __init__(self):
    super().__init__('fake_vehicle')
    self.x, self.y, self.v = 0.0, 0.0, 0.0
    self.stop_dist = 999.0
    self.dt = 0.05
    self.t = 0.0
    self.profiled = 0.0

    self.events = []          # (t, 사건)
    self.stopped_at = None    # 정지한 시각
    self.stop_error = None    # 정지선까지 남은 거리
    self.resumed_at = None

    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.create_subscription(Float64, '/stop_line_distance', self.stop_cb, 10)
    self.create_timer(self.dt, self.step)

  def stop_cb(self, msg):
    self.stop_dist = float(msg.data)

  def target_speed(self):
    """longitudinal_controller_node.decide_target() 의 정지선 부분과 동일."""
    v = V_MAX
    if self.stop_dist < 19.0:
      v = min(v, math.sqrt(2.0 * STOP_DECEL * max(0.0, self.stop_dist)))
      if self.stop_dist <= 0.3:
        return 0.0
    return v

  def step(self):
    tgt = self.target_speed()
    # 슬루레이트 (가감속 한계)
    if tgt > self.profiled:
      self.profiled = min(tgt, self.profiled + MAX_ACCEL * self.dt)
    else:
      self.profiled = max(tgt, self.profiled - MAX_DECEL * self.dt)
    self.v = self.profiled
    self.x += self.v * self.dt
    self.t += self.dt

    remaining = STOP_X - self.x
    # --- 사건 기록 ---
    if self.stopped_at is None and self.v < 0.02 and self.t > 1.0:
      self.stopped_at = self.t
      self.stop_error = remaining
      self.events.append((self.t, f'정지 — 정지선까지 {remaining*100:.1f}cm'))
    elif self.stopped_at is not None and self.resumed_at is None and self.v > 0.05:
      self.resumed_at = self.t
      self.events.append((self.t, f'재출발 (정지 후 {self.t-self.stopped_at:.2f}s)'))

    od = Odometry()
    od.header.stamp = self.get_clock().now().to_msg()
    od.header.frame_id = 'map'
    od.pose.pose.position.x = self.x
    od.pose.pose.position.y = self.y
    od.pose.pose.orientation.w = 1.0     # yaw = 0 (+x 방향)
    od.twist.twist.linear.x = self.v
    self.odom_pub.publish(od)


def main():
  print('횡단보도 정지 미션 검증 (가짜 차량 폐루프)')
  print('=' * 52)
  print(f'  정지선 x={STOP_X}m,  주행속도 {V_MAX}m/s,  '
        f'규정 {TOLERANCE*100:.0f}cm / {DWELL:.0f}초\n')

  node_proc = subprocess.Popen(
      ['ros2', 'run', 'mission_perception', 'crosswalk_stop_node',
       '--ros-args',
       '-p', f'stop_points:=[{STOP_X}, {STOP_Y}]',
       '-p', f'dwell:={DWELL}',
       '-p', 'stop_enter_dist:=0.5'],
      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
      start_new_session=True)
  _SPAWNED.append(node_proc)
  time.sleep(2.5)      # 노드가 뜰 때까지

  rclpy.init()
  car = FakeVehicle()
  try:
    deadline = time.time() + 40.0
    while time.time() < deadline:
      rclpy.spin_once(car, timeout_sec=0.01)
      time.sleep(0.01)
      # 재출발 후 5초 더 달려보면 '다시 안 걸리는지' 확인된다
      if car.resumed_at is not None and car.t > car.resumed_at + 5.0:
        break
  finally:
    car_x, events = car.x, list(car.events)
    stop_err, stopped_at, resumed_at = car.stop_error, car.stopped_at, car.resumed_at
    final_v = car.v
    car.destroy_node()
    rclpy.shutdown()
    _kill_group(node_proc)

  for (t, e) in events:
    print(f'  [{t:6.2f}s] {e}')
  print()

  ok = True

  if stop_err is None:
    print('  1) 정지선 앞 정지          ❌ 서지 않았다')
    ok = False
  else:
    within = 0.0 <= stop_err <= TOLERANCE
    print(f'  1) 정지선 앞 정지          '
          f'{"✅" if within else "❌"} {stop_err*100:.1f}cm '
          f'(규정 {TOLERANCE*100:.0f}cm 이내, 넘어가면 안 됨)')
    ok = ok and within

  if resumed_at is None:
    print(f'  2) {DWELL:.0f}초 대기 후 재출발     ❌ 재출발하지 않았다')
    ok = False
  else:
    waited = resumed_at - stopped_at
    good = abs(waited - DWELL) <= 0.5
    print(f'  2) {DWELL:.0f}초 대기 후 재출발     '
          f'{"✅" if good else "❌"} 실제 {waited:.2f}s')
    ok = ok and good

  moved_on = final_v > 0.3 and car_x > STOP_X + 1.0
  print(f'  3) 재출발 후 계속 주행     '
          f'{"✅" if moved_on else "❌"} x={car_x:.1f}m v={final_v:.2f}m/s '
          f'(지나간 정지선에 다시 안 걸림)')
  ok = ok and moved_on

  print('\n' + '=' * 52)
  print('결론: 횡단보도 정지 미션 로직 정상 ✅' if ok
        else '결론: ❌ 실패 — 위 항목을 확인할 것.')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

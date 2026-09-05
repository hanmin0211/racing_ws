#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_parking_roundtrip.py — 기록 → 파일 → 재현 전 과정을 이어서 검증한다.

test_parking_node.py 는 **손으로 만든** yaml 로 노드를 검증한다. 그런데 대회장에서
쓰는 파일은 parking_recorder 가 만든다. 둘 사이에 스키마나 기어 판정이 어긋나면
**현장에서야** 알게 된다. 그걸 막는 테스트다.

  가짜 teleop 주행 ──odom──→ parking_recorder ──parking_9.yaml──→ parking_node
                                                                      │
                                          가짜 차량 ←──/teleop/cmd_vel─┘

검증하는 것:
  1. recorder 가 파일을 만드는가 (SIGINT 저장 경로)
  2. **기어 판정이 맞는가** — 전진구간은 +1, 후진구간은 -1 로 찍혔는가
  3. 원점이 파일에 기록되는가 (장소 뒤바뀜 방지)
  4. 그 파일을 parking_node 가 그대로 읽어 재현하는가

  python3 tools/test_parking_roundtrip.py
"""

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ★ 실차 스택과 격리한다.
#   bringup 이 떠 있으면 direct_localization 도 /odometry/filtered 를 발행해서
#   가짜 차량과 충돌한다(실제로 노드가 실차 위치를 보고 "궤적에서 34m 벗어남"
#   으로 중단했다). 별도 ROS_DOMAIN_ID 를 쓰면 스택을 켜둔 채로 시험할 수 있다.
TEST_DOMAIN = os.environ.get('PARKING_TEST_DOMAIN', '77')
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN

L = 0.785
SLOT = 9                      # 실제 자리(1~3)를 덮어쓰지 않도록 9번을 쓴다


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


class TeleopDriver(Node):
  """사람이 teleop 으로 주차하는 상황을 재현해 odom 을 쏜다."""

  def __init__(self):
    super().__init__('fake_teleop_driver')
    self.x, self.y, self.yaw = -6.0, 0.0, 0.0
    self.dt = 0.05
    self.phase = 0            # 0=전진, 1=정지, 2=후진, 3=끝
    self.t_phase = 0.0
    self.v = 0.0
    self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.create_timer(self.dt, self.step)
    self.finished = False

  def step(self):
    self.t_phase += self.dt
    steer = 0.0
    if self.phase == 0:                       # 전진 8m 직진
      self.v = 0.5
      if self.x >= 2.0:
        self.phase, self.t_phase = 1, 0.0
    elif self.phase == 1:                     # 완전정지 1초 (기어 바꾸는 시간)
      self.v = 0.0
      if self.t_phase >= 1.0:
        self.phase, self.t_phase = 2, 0.0
    elif self.phase == 2:                     # 후진하며 좌타 → 후방-좌측으로 진입
      self.v = -0.4
      steer = math.radians(17.0)
      if self.yaw <= math.radians(-88.0):
        self.phase, self.t_phase = 3, 0.0
    else:
      self.v = 0.0
      if self.t_phase >= 1.0:
        self.finished = True

    self.x += self.v * math.cos(self.yaw) * self.dt
    self.y += self.v * math.sin(self.yaw) * self.dt
    self.yaw += (self.v / L) * math.tan(steer) * self.dt

    od = Odometry()
    od.header.stamp = self.get_clock().now().to_msg()
    od.header.frame_id = 'map'
    od.pose.pose.position.x = self.x
    od.pose.pose.position.y = self.y
    od.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
    od.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
    od.twist.twist.linear.x = self.v
    self.pub.publish(od)


def main():
  tmpdir = tempfile.mkdtemp(prefix='roundtrip_')
  out = os.path.join(tmpdir, f'parking_{SLOT}.yaml')

  print('기록 → 파일 → 재현  왕복 검증')
  print('=' * 54)
  print('  가짜 teleop: 전진 8m → 정지 1s → 좌타 후진 90°\n')

  rec = subprocess.Popen(
      ['ros2', 'run', 'mission_perception', 'parking_recorder', '--ros-args',
       '-p', f'slot:={SLOT}', '-p', f'output:={out}'],
      stdout=open('/tmp/claude-1000/-home-han-racing-ws-src/ee5c56e0-3ae9-4936-84a1-7f6964f50175/scratchpad/rec2.log','w'),
      stderr=subprocess.STDOUT, start_new_session=True)
  time.sleep(2.5)

  rclpy.init()
  drv = TeleopDriver()
  try:
    deadline = time.time() + 60.0
    while time.time() < deadline and not drv.finished:
      rclpy.spin_once(drv, timeout_sec=0.01)
      time.sleep(0.01)
    final = (drv.x, drv.y, math.degrees(drv.yaw))
  finally:
    drv.destroy_node()
    rclpy.shutdown()

  # Ctrl-C 로 저장시킨다 (현장에서 쓰는 것과 같은 경로)
  os.killpg(os.getpgid(rec.pid), signal.SIGINT)
  try:
    rec.wait(timeout=10)
  except subprocess.TimeoutExpired:
    _kill_group(rec)

  print(f'  주행 종료: ({final[0]:+.2f}, {final[1]:+.2f}) yaw={final[2]:+.0f}°\n')

  ok = True

  # 1) 파일 생성
  exists = os.path.exists(out)
  print(f'  1) recorder 가 파일 저장     {"✅" if exists else "❌ 파일 없음"}')
  if not exists:
    print('\n결론: ❌ 기록 자체가 안 된다 — 현장에서 못 쓴다.')
    return 1

  with open(out, encoding='utf-8') as f:
    data = yaml.safe_load(f)
  pts = data['points']
  fwd = [p for p in pts if p['gear'] > 0]
  rev = [p for p in pts if p['gear'] < 0]

  # 2) 기어 판정
  #    전진구간은 y≈0 에 몰려 있고, 후진구간은 y 가 크게 움직인다.
  gear_ok = len(fwd) >= 10 and len(rev) >= 10
  if gear_ok:
    # 전진으로 찍힌 점들이 실제로 전진 구간(초반)에 몰려 있는지
    last_fwd = max(i for i, p in enumerate(pts) if p['gear'] > 0)
    first_rev = min(i for i, p in enumerate(pts) if p['gear'] < 0)
    # 전환이 한 번만 일어나야 한다 (앞이 전진, 뒤가 후진)
    switches = sum(1 for i in range(1, len(pts))
                   if pts[i]['gear'] != pts[i - 1]['gear'])
    gear_ok = switches == 1 and first_rev > last_fwd - 3
    print(f'  2) 기어 판정                 {"✅" if gear_ok else "❌"} '
          f'전진 {len(fwd)}점 / 후진 {len(rev)}점, 전환 {switches}회 '
          f'(1회여야 정상)')
  else:
    print(f'  2) 기어 판정                 ❌ 전진 {len(fwd)} / 후진 {len(rev)}')
  ok = ok and gear_ok

  # 3) 원점 기록
  org = data.get('origin')
  org_ok = org is not None and 'x' in org and 'y' in org
  print(f'  3) 원점 기록                 {"✅" if org_ok else "❌ origin 없음"} '
        f'{org if org_ok else ""}')
  ok = ok and org_ok

  # 4) 그 파일로 parking_node 재현
  print(f'\n  4) 이 파일로 parking_node 재현 →')
  r = subprocess.run(
      [sys.executable, os.path.join(os.path.dirname(__file__),
                                    'test_parking_node.py'),
       '--file', out, '--slot', str(SLOT)],
      capture_output=True, text=True, timeout=240)
  tail = [ln for ln in r.stdout.splitlines() if ln.strip()]
  for ln in tail[-9:]:
    print('     ' + ln)
  node_ok = r.returncode == 0
  ok = ok and node_ok

  print('\n' + '=' * 54)
  print('결론: 기록→재현 전 과정 정상 ✅' if ok
        else '결론: ❌ 실패 — 현장에서 터진다. 위 항목 확인.')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

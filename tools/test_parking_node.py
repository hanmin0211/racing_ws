#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_parking_node.py — parking_node 를 가짜 차량으로 폐루프 검증한다.

test_parking_geometry.py 는 **추출한 함수**를 검증한다. 이건 **실제 노드**를
띄워서 /teleop/cmd_vel 을 받아 자전거모델로 굴린다. 상태기계(구간전환·cusp 정지·
완료 판정·중단 조건)까지 함께 검증된다.

  가짜차량 ──/odometry/filtered──→ parking_node ──/teleop/cmd_vel──→ 가짜차량

  python3 tools/test_parking_node.py [--slot N] [--file parking_N.yaml]

인자를 안 주면 합성 궤적(전진 접근 → 후진 진입)을 만들어 쓴다.
대회장에서 실제로 찍은 뒤에는:

  python3 tools/test_parking_node.py --file ~/parking_1.yaml

⚠ 로직 검증이다. 실차 조향 극성은 PLAN PHASE 0-2 벤치 측정으로 따로 확인할 것.
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ★ 실차 스택과 격리한다.
#   bringup 이 떠 있으면 direct_localization 도 /odometry/filtered 를 발행해서
#   가짜 차량과 충돌한다(실제로 노드가 실차 위치를 보고 "궤적에서 34m 벗어남"
#   으로 중단했다). 별도 ROS_DOMAIN_ID 를 쓰면 스택을 켜둔 채로 시험할 수 있다.
TEST_DOMAIN = os.environ.get('PARKING_TEST_DOMAIN', '77')
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN

L = 0.785
MAX_STEER_DEG = 18.0


def make_synthetic(path):
  """전진 접근 → 후진 진입 궤적. 반경은 최소회전반경(2.42m)보다 크게."""
  R = 2.6
  cx, cy = 2.0, -R
  pts = []
  for i in range(41):
    pts.append({'x': -6.0 + 0.2 * i, 'y': 0.0, 'yaw': 0.0,
                'speed': 0.3, 'gear': 1})
  n = 40
  for k in range(1, n + 1):
    th = math.radians(90.0 + 90.0 * (k / n))
    pts.append({'x': cx + R * math.cos(th), 'y': cy + R * math.sin(th),
                'yaw': 0.0, 'speed': -0.3, 'gear': -1})
  for k in range(1, 6):
    pts.append({'x': cx - R, 'y': cy - 0.2 * k, 'yaw': 0.0,
                'speed': -0.3, 'gear': -1})
  with open(path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'points': pts}, f, allow_unicode=True, sort_keys=False)
  return pts


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
  """/teleop/cmd_vel 을 받아 자전거모델로 적분하고 odom 을 낸다."""

  def __init__(self, x0, y0, yaw0):
    super().__init__('fake_parking_vehicle')
    self.x, self.y, self.yaw = x0, y0, yaw0
    self.v, self.steer = 0.0, 0.0
    self.dt = 0.05
    self.t = 0.0
    self.state = '?'
    self.done = False
    self.trail = []
    self.states = []

    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.start_pub = self.create_publisher(Bool, '/parking/start', 10)
    self.create_subscription(Twist, '/teleop/cmd_vel', self.cmd_cb, 10)
    self.create_subscription(String, '/parking/state', self.state_cb, 10)
    self.create_subscription(Bool, '/parking/done', self.done_cb, 10)
    self.create_timer(self.dt, self.step)

  def cmd_cb(self, msg):
    self.v = float(msg.linear.x)
    d = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, float(msg.angular.z)))
    self.steer = math.radians(d)

  def state_cb(self, msg):
    if msg.data != self.state:
      self.states.append((self.t, msg.data))
      self.state = msg.data

  def done_cb(self, msg):
    if bool(msg.data):
      self.done = True

  def step(self):
    self.x += self.v * math.cos(self.yaw) * self.dt
    self.y += self.v * math.sin(self.yaw) * self.dt
    self.yaw += (self.v / L) * math.tan(self.steer) * self.dt
    self.t += self.dt
    self.trail.append((self.x, self.y))

    od = Odometry()
    od.header.stamp = self.get_clock().now().to_msg()
    od.header.frame_id = 'map'
    od.pose.pose.position.x = self.x
    od.pose.pose.position.y = self.y
    od.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
    od.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
    od.twist.twist.linear.x = self.v
    self.odom_pub.publish(od)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--file', default=None,
                  help='(무시됨 — 신 노드는 자세 파일을 쓴다)')
  ap.add_argument('--pose-dir', default=os.path.expanduser('~'),
                  help='parking_pose_N.yaml 위치')
  ap.add_argument('--slot', type=int, default=1)
  ap.add_argument('--timeout', type=float, default=120.0)
  ap.add_argument('--offset', type=float, default=0.0,
                  help='시작을 궤적에서 옆으로 이만큼 밀어놓는다[m] '
                       '(실차 완주 종점 오차 재현)')
  ap.add_argument('--yaw-offset', type=float, default=0.0,
                  help='시작 헤딩을 이만큼 틀어놓는다[도]')
  ap.add_argument('--start-pose', default=None,
                  help='"x,y,yaw_deg" — 차를 이 자세에 놓고 시작한다. '
                       '실차 완주 종점 자세를 그대로 넣어 검증할 때 쓴다')
  ap.add_argument('--verbose', action='store_true',
                  help='parking_node 로그를 그대로 보여준다(중단 사유 확인용)')
  args = ap.parse_args()

  # ★ 신 노드는 목표 자세만 필요하다. parking_pose_N.yaml 를 그대로 쓴다.
  pose_path = os.path.join(args.pose_dir, f'parking_pose_{args.slot}.yaml')
  with open(pose_path, encoding='utf-8') as f:
    pd = yaml.safe_load(f)
  goal = (float(pd['pose']['x']), float(pd['pose']['y']))
  goal_yaw = math.radians(float(pd['pose']['yaw_deg']))
  # 시작 자세 기본값: 완주 경로 종점 (실차 상황 재현). --start-pose 로 덮어씀.
  start = (-25.605, 103.797)
  yaw0 = math.radians(166.9)
  # 신 노드는 tmpdir 를 쓰지 않는다 — pose_dir 를 그대로 넘긴다.
  tmpdir = args.pose_dir

  print('parking_node 폐루프 검증')
  print('=' * 52)
  print(f'  출발 ({start[0]:.2f}, {start[1]:.2f})  '
        f'목표 정차점 ({goal[0]:.2f}, {goal[1]:.2f})\n')

  proc = subprocess.Popen(
      ['ros2', 'run', 'mission_perception', 'parking_node', '--ros-args',
       '-p', f'slot:={args.slot}', '-p', f'pose_dir:={tmpdir}',
       '-p', 'speed:=0.3', '-p', 'cusp_dwell:=1.0'],
      stdout=(None if args.verbose else subprocess.DEVNULL),
      stderr=(subprocess.STDOUT if args.verbose else subprocess.DEVNULL),
      start_new_session=True)
  time.sleep(2.5)

  rclpy.init()
  # 실차에서는 완주 종점 자세가 궤적 시작과 정확히 같지 않다. 그 오차를 주입해
  # '초기 접근에서 붙을 수 있는가'를 검증한다.
  if args.start_pose:
    px, py, pyaw = [float(v) for v in args.start_pose.split(',')]
    sx0, sy0 = px, py
    yaw0 = math.radians(pyaw)
    print(f'  시작 자세 지정: ({px:.2f}, {py:.2f}) yaw={pyaw:+.1f}°')
  else:
    sx0 = start[0] - args.offset * math.sin(yaw0)
    sy0 = start[1] + args.offset * math.cos(yaw0)
  car = FakeVehicle(sx0, sy0, yaw0 + math.radians(args.yaw_offset))
  park_xy = [None]      # 차고 도달 시점 좌표
  if args.offset or args.yaw_offset:
    print(f'  시작 오차 주입: 옆으로 {args.offset:+.2f}m, '
          f'헤딩 {args.yaw_offset:+.0f}°')
  try:
    # odom 을 조금 흘려보낸 뒤 시작 신호 (노드가 위치를 알아야 시작한다)
    t0 = time.time()
    while time.time() - t0 < 1.5:
      rclpy.spin_once(car, timeout_sec=0.01)
      time.sleep(0.01)
    # ★ 한 번만 쏘면 구독자 발견 전이라 유실된다(실제로 3회 연속 WAIT 로 멈췄다).
    #   상태가 바뀔 때까지 계속 쏜다.
    deadline = time.time() + args.timeout
    last_start = 0.0
    while time.time() < deadline:
      if car.state in ('?', 'WAIT') and time.time() - last_start > 0.5:
        car.start_pub.publish(Bool(data=True))
        last_start = time.time()
      rclpy.spin_once(car, timeout_sec=0.01)
      time.sleep(0.01)
      # ★ 차고 도달(DONE) 시점의 좌표를 따로 기록한다.
      #   규정(항목 6·8)대로 탈출이 붙으면 차가 계속 움직이므로, 정차점 오차를
      #   '최종 위치'로 재면 탈출 거리만큼 틀린 값이 나온다.
      if park_xy[0] is None and car.state in ('DONE', 'EXIT'):
        park_xy[0] = (car.x, car.y)
      if car.state in ('ABORT', 'FINISHED') and car.t > 2.0:
        end = car.t + 2.0
        while car.t < end:
          rclpy.spin_once(car, timeout_sec=0.01)
          time.sleep(0.01)
        break
      # 탈출이 없는 구성(탈출 자세 파일 없음)에서는 DONE 이 종점이다.
      if car.state == 'DONE' and car.t > 2.0:
        end = car.t + 2.0
        while car.t < end:
          rclpy.spin_once(car, timeout_sec=0.01)
          time.sleep(0.01)
        if car.state == 'DONE':      # 그 사이 EXIT 로 안 넘어갔으면 종료
          break
  finally:
    fx, fy, fyaw = car.x, car.y, car.yaw
    fv, state, states = car.v, car.state, list(car.states)
    sim_t = car.t
    car.destroy_node()
    rclpy.shutdown()
    _kill_group(proc)

  for (t, s) in states:
    print(f'  [{t:6.2f}s] {s}')

  px, py = park_xy[0] if park_xy[0] else (fx, fy)
  err = math.hypot(px - goal[0], py - goal[1])
  did_exit = any(s == 'EXIT' for _, s in states)
  print(f'\n  차고 도달 ({px:+.2f}, {py:+.2f})')
  print(f'  최종 위치 ({fx:+.2f}, {fy:+.2f}) yaw={math.degrees(fyaw):+.0f}°  '
        f'소요 {sim_t:.1f}s')

  ok = True
  # 규정(항목 6·8): 후진으로 차고 진입 → 뒷바퀴 확인선 접촉 →
  #                 **전진으로 진입확인선 통과**. 탈출 자세가 있으면 FINISHED 가 종점.
  want = 'FINISHED' if did_exit else 'DONE'
  done = state == want
  print(f'\n  1) 미션 완료({want})   {"✅" if done else f"❌ {state}"}')
  ok = ok and done

  good = err <= 0.30
  print(f'  2) 목표 정차점 오차        {"✅" if good else "❌"} {err*100:.0f}cm '
        f'(30cm 이내)')
  ok = ok and good

  stopped = abs(fv) < 0.01
  print(f'  3) 완료 후 정지 유지       {"✅" if stopped else "❌"} '
        f'v={fv:+.3f}m/s (0 을 계속 발행해야 자율로 안 돌아감)')
  ok = ok and stopped

  had_cusp = any(s == 'CUSP' for _, s in states)
  print(f'  4) cusp 완전정지 후 기어전환 {"✅" if had_cusp else "❌ CUSP 없음"}')
  ok = ok and had_cusp

  if did_exit:
    # 탈출 목표 자세까지 나왔는가 (규정: 전진으로 진입확인선 통과)
    ex_p = os.path.join(args.pose_dir, f'parking_exit_{args.slot}.yaml')
    try:
      with open(ex_p, encoding='utf-8') as f:
        ep = yaml.safe_load(f)['pose']
      exd = math.hypot(fx - float(ep['x']), fy - float(ep['y']))
      exok = exd <= 0.40
      print(f'  5) 전진 탈출 완료          {"✅" if exok else "❌"} '
            f'탈출 자세까지 {exd*100:.0f}cm (40cm 이내)')
      ok = ok and exok
    except Exception as e:  # noqa: BLE001
      print(f'  5) 전진 탈출 완료          ❌ 탈출 자세 로드 실패 {e}')
      ok = False

  print('\n' + '=' * 52)
  print('결론: 후진주차 로직 정상 ✅' if ok else '결론: ❌ 실패 — 위 항목 확인.')
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())

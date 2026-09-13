#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""breakaway_check.py — 정지마찰 하한(breakaway)이 실제로 나가는지 본다.

★ 무엇을 확인하나 (2026-09-13)
  그날 차가 출발을 못 했다. 원인은 ff_min_pwm 50 이 이 차의 정지마찰
  문턱(PWM 55~56) 아래였던 것이다. 고친 뒤 '정지에서 출발할 때만 잠깐
  더 센 PWM' 이 나가야 하는데, **그게 정말 나가는지** 눈으로 봐야 한다.

  기대 파형:
      정지        PWM 0
      출발 직후   PWM = ff_breakaway_pwm (기본 60)   ← 600ms 동안
      그 뒤       PWM = ff_min_pwm (기본 50)
      정지        PWM 0

⚠ **바퀴를 들고 하라.** 차가 실제로 굴러간다. 바퀴를 들면 정지마찰은
  못 보지만 'PWM 이 제대로 나가는가' 는 확인된다. 지면 시험은 밖에서.

먼저 제어 스택을 띄울 것 (자율 아님, teleop 경로만 쓴다):
    ros2 launch pure_pursuit_pkg control.launch.py

  python3 tools/breakaway_check.py              # 1.0초 전진
  python3 tools/breakaway_check.py --speed 0.4 --secs 1.5
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--speed', type=float, default=0.5, help='명령 속도[m/s]')
  ap.add_argument('--secs', type=float, default=1.0, help='굴릴 시간[s]')
  ap.add_argument('--settle', type=float, default=1.0, help='정지 관측[s]')
  a = ap.parse_args()

  rclpy.init()
  n = Node('breakaway_check')
  pub = n.create_publisher(Twist, '/teleop/cmd_vel', 10)
  st = {'pwm': None, 'vcc': None}
  n.create_subscription(Int32, '/drive_pwm',
                        lambda m: st.update(pwm=m.data), 10)
  n.create_subscription(Int32, '/vcc_mv', lambda m: st.update(vcc=m.data), 10)

  print('⚠ 바퀴를 들었는지 확인할 것. 3초 뒤 시작한다.')
  for i in (3, 2, 1):
    print(f'  {i}...')
    t = time.time()
    while time.time() - t < 1.0:
      rclpy.spin_once(n, timeout_sec=0.05)

  trace = []
  t0 = time.time()

  def tick(v):
    m = Twist()
    m.linear.x = v
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.02)
    trace.append((time.time() - t0, v, st['pwm'], st['vcc']))

  # 0.3초 정지 → secs 전진 → settle 정지
  while time.time() - t0 < 0.3:
    tick(0.0)
  while time.time() - t0 < 0.3 + a.secs:
    tick(a.speed)
  while time.time() - t0 < 0.3 + a.secs + a.settle:
    tick(0.0)
  for _ in range(10):
    tick(0.0)

  print(f"\n{'t[s]':>6} {'명령v':>7} {'PWM':>5} {'VCC':>6}")
  seen = set()
  for t, v, pwm, vcc in trace:
    key = (round(t, 1), pwm)
    if key in seen:
      continue
    seen.add(key)
    print(f'{t:6.2f} {v:7.2f} '
          f"{pwm if pwm is not None else '-':>5} "
          f"{vcc if vcc is not None else '-':>6}")

  pwms = [p for _, v, p, _ in trace if v > 0 and p is not None]
  if not pwms:
    print('\n❌ /drive_pwm 을 한 건도 못 받았다 — control.launch.py 가 '
          '떠 있는지, 아두이노가 붙었는지 확인할 것.')
    rclpy.shutdown()
    return 1
  peak, tail = max(pwms), pwms[-1]
  print(f'\n  출발 구간 최대 PWM {peak}   끝부분 PWM {tail}')
  if peak >= 58:
    print('  ✅ breakaway 가 나갔다 (정지마찰 하한 인가 확인)')
  else:
    print(f'  ❌ breakaway 가 안 나갔다 — 최대가 {peak} 뿐이다. '
          'ff_breakaway_pwm 이 전달됐는지 확인할 것.')
  if tail < peak:
    print(f'  ✅ 그 뒤 크리프 하한으로 내려갔다 ({peak} → {tail})')
  rclpy.shutdown()
  return 0


if __name__ == '__main__':
  sys.exit(main())

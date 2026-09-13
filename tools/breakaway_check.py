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

두 가지 방식이 있다:
  · **바퀴를 들고** — 'PWM 이 제대로 나가는가' 만 본다. 안전하다.
  · **바퀴를 내리고** — **정지마찰을 실제로 뚫는가**를 본다. 이게 본 시험이다.
    GPS 가 필요 없으므로 **복도에서도 된다**(캘리브·경로추종은 GPS 가 있어야
    하지만, '차가 굴러가는가' 는 구동계 문제라 실내에서 답이 나온다).
    앞을 3m 이상 비우고 손을 댈 수 있게 할 것.

★ 움직였는지를 눈으로 판단하지 않는다 — **IMU 가속도로 잰다.**
  엔코더가 없고 실내엔 GPS 도 없다. 대신 정지 구간의 가속도 평균을 기준으로
  잡고, 굴리는 동안의 편차가 유의미하면 '움직였다' 로 본다.

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
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


def _motion_verdict(trace):
  """IMU 가속도로 '정말 움직였는가' 를 판정한다.

  정지 구간(명령 0, 앞쪽)의 축별 평균을 기준으로 잡고, 굴리는 구간의
  최대 편차를 본다. 중력이 어느 축에 실리는지 모르므로 축을 고르지 않고
  **세 축의 편차 크기**로 본다. 차가 굴러가면 노면 진동과 가감속이
  같이 들어와 편차가 확실히 커진다.
  """
  import math
  rest = [a for t, v, _p, _c, a in trace if v == 0.0 and a is not None
          and t < 0.3]
  move = [a for _t, v, _p, _c, a in trace if v > 0.0 and a is not None]
  if len(rest) < 3 or len(move) < 5:
    print('\n  ⚠ IMU 표본이 부족해 움직임을 판정 못 했다 '
          '(/handsfree/imu 가 오는지 확인할 것). 눈으로 볼 것.')
    return
  base = [sum(c[i] for c in rest) / len(rest) for i in range(3)]
  peak = max(math.dist(a, base) for a in move)
  rest_noise = max(math.dist(a, base) for a in rest) if len(rest) > 1 else 0.0
  print(f'\n  IMU 가속도 편차: 정지 중 {rest_noise:.2f} → 굴릴 때 {peak:.2f} m/s²')
  if peak > max(0.6, rest_noise * 3.0):
    print('  ✅ 차가 실제로 움직였다 (정지마찰을 뚫었다)')
  else:
    print('  ❌ 움직임이 안 잡힌다 — 정지마찰을 못 뚫었을 수 있다.')
    print('     ff_breakaway_pwm 을 올려서 다시 볼 것 '
          '(--ros-args 가 아니라 런치 인자로: ff_breakaway_pwm:=70)')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--speed', type=float, default=0.5, help='명령 속도[m/s]')
  ap.add_argument('--secs', type=float, default=1.0, help='굴릴 시간[s]')
  ap.add_argument('--settle', type=float, default=1.0, help='정지 관측[s]')
  a = ap.parse_args()

  rclpy.init()
  n = Node('breakaway_check')
  pub = n.create_publisher(Twist, '/teleop/cmd_vel', 10)
  st = {'pwm': None, 'vcc': None, 'acc': None}
  n.create_subscription(Int32, '/drive_pwm',
                        lambda m: st.update(pwm=m.data), 10)
  n.create_subscription(Int32, '/vcc_mv', lambda m: st.update(vcc=m.data), 10)

  from rclpy.qos import qos_profile_sensor_data
  n.create_subscription(
      Imu, '/handsfree/imu',
      lambda m: st.update(acc=(m.linear_acceleration.x,
                               m.linear_acceleration.y,
                               m.linear_acceleration.z)),
      qos_profile_sensor_data)

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
    trace.append((time.time() - t0, v, st['pwm'], st['vcc'], st['acc']))

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
  for t, v, pwm, vcc, _acc in trace:
    key = (round(t, 1), pwm)
    if key in seen:
      continue
    seen.add(key)
    print(f'{t:6.2f} {v:7.2f} '
          f"{pwm if pwm is not None else '-':>5} "
          f"{vcc if vcc is not None else '-':>6}")

  _motion_verdict(trace)

  pwms = [p for _, v, p, _, _a in trace if v > 0 and p is not None]
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

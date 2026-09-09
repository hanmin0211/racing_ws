#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""steer_sweep.py — 조향만 격리해서 최대 타각·응답·전압강하를 잰다.

★ 왜 따로 재는가
  주행 중에는 조향각이 코스 곡률에 따라 정해져서 최대치가 안 나온다(용인
  코스는 최대 10° 남짓, 상한 18° 의 절반). 하드웨어가 **끝까지 꺾이는지**,
  거기서 **전압이 얼마나 떨어지는지**, **스톨이 나는지**는 따로 시험해야 한다.

  구동은 0 으로 두고 /teleop/cmd_vel 의 조향각만 계단 입력으로 준다.
  teleop 은 먹스에서 자율보다 우선이므로 스택이 떠 있어도 안전하게 덮어쓴다.
  (먹스 데드맨 0.5s 때문에 20Hz 로 계속 쏜다.)

★ 준비
  · **바퀴를 들어 올릴 것** — 구동은 0 이지만 조향이 실제로 돈다
  · hil.launch.py 나 teleop_drive.launch.py 로 serial_bridge + mux 가 떠 있을 것

사용:
  python3 tools/steer_sweep.py                 # 0 → +18 → 0 → -18 → 0
  python3 tools/steer_sweep.py --max 12        # ±12° 까지만
  python3 tools/steer_sweep.py --hold 4        # 각 단계 4초 유지
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32


class Sweep(Node):

  def __init__(self, args):
    super().__init__('steer_sweep')
    self.a = args
    self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
    self.ang = None
    self.adc = None
    self.err = None
    self.vcc = None
    self.vmin = None
    self.stall = False
    self.create_subscription(Float64, '/steering_angle',
                             lambda m: setattr(self, 'ang', m.data), 10)
    self.create_subscription(Int32, '/steering_adc',
                             lambda m: setattr(self, 'adc', m.data), 10)
    self.create_subscription(Float64, '/steering_error',
                             lambda m: setattr(self, 'err', m.data), 10)
    self.create_subscription(Int32, '/vcc_mv',
                             lambda m: setattr(self, 'vcc', m.data), 10)
    self.create_subscription(Int32, '/vcc_min_mv',
                             lambda m: setattr(self, 'vmin', m.data), 10)
    self.create_subscription(Bool, '/vehicle_stall',
                             lambda m: setattr(self, 'stall', m.data), 10)
    self.cmd = 0.0
    self.create_timer(0.05, self.tick)     # 20Hz — 먹스 데드맨(0.5s) 대응

  def tick(self):
    t = Twist()
    t.linear.x = 0.0                       # ★ 구동은 항상 0
    t.angular.z = float(self.cmd)
    self.pub.publish(t)

  def spin(self, sec):
    end = time.time() + sec
    while time.time() < end:
      rclpy.spin_once(self, timeout_sec=0.02)

  def step(self, deg, rows):
    self.cmd = deg
    self.spin(self.a.hold)                 # 도달 대기
    # 마지막 0.5초 평균으로 정착값을 잡는다
    samples = []
    end = time.time() + 0.5
    while time.time() < end:
      rclpy.spin_once(self, timeout_sec=0.02)
      if self.adc is not None and self.ang is not None:
        samples.append((self.adc, self.ang, self.vcc or 0, self.vmin or 0))
    if not samples:
      print(f'  {deg:+6.1f}°   (응답 없음 — serial_bridge 가 떠 있는지 확인)')
      return
    adc = sum(s[0] for s in samples) / len(samples)
    ang = sum(s[1] for s in samples) / len(samples)
    vcc = min(s[2] for s in samples)
    vmn = min(s[3] for s in samples if s[3]) if any(s[3] for s in samples) else 0
    e = ang - deg
    flag = '  ⚠스톨' if self.stall else ''
    print(f'  {deg:+6.1f}°  {ang:+7.1f}°  {adc:7.0f}  {e:+6.1f}°  '
          f'{vcc:6d}  {vmn:6d}{flag}')
    rows.append((deg, ang, adc, e, vcc, vmn, self.stall))


def main():
  ap = argparse.ArgumentParser(description='조향 최대 타각·응답 시험')
  ap.add_argument('--max', type=float, default=18.0, help='최대 타각[도]')
  ap.add_argument('--step', type=float, default=6.0, help='계단 간격[도]')
  ap.add_argument('--hold', type=float, default=2.5, help='각 단계 유지[s]')
  args, _ = ap.parse_known_args()

  rclpy.init()
  n = Sweep(args)
  print('\n★ 바퀴가 들려 있는지 확인하십시오. 구동은 0 이지만 조향이 돕니다.\n')
  n.spin(1.5)                               # 구독 연결 대기
  if n.adc is None:
    print('❌ /steering_adc 가 안 옵니다 — serial_bridge 가 떠 있어야 합니다.')
    n.destroy_node(); rclpy.shutdown(); return

  print('  명령각    실제각     ADC     오차     VCC   VMIN')
  print('  ' + '-' * 52)
  rows = []
  seq = []
  s = args.step
  seq += [x * s for x in range(0, int(args.max / s) + 1)]        # 0 → +max
  if seq[-1] != args.max: seq.append(args.max)
  seq += [x for x in reversed(seq[:-1])]                          # → 0
  seq += [-x for x in seq[1:int(args.max / s) + 2] if x != 0]     # → -max
  seq += [x for x in reversed(seq[len(seq) - int(args.max / s):])] # → 0
  seq.append(0.0)
  for d in seq:
    n.step(d, rows)

  n.cmd = 0.0
  n.spin(1.0)

  print('\n' + '=' * 52)
  if rows:
    pos = [r for r in rows if r[0] > 0]
    neg = [r for r in rows if r[0] < 0]
    if pos:
      m = max(pos, key=lambda r: r[0])
      print(f'  + 최대   명령 {m[0]:+.1f}°  실제 {m[1]:+.1f}°  '
            f'ADC {m[2]:.0f}  오차 {m[3]:+.1f}°')
    if neg:
      m = min(neg, key=lambda r: r[0])
      print(f'  − 최대   명령 {m[0]:+.1f}°  실제 {m[1]:+.1f}°  '
            f'ADC {m[2]:.0f}  오차 {m[3]:+.1f}°')
    worst = max(rows, key=lambda r: abs(r[3]))
    print(f'  최대 오차  {worst[3]:+.1f}°  (명령 {worst[0]:+.1f}°)')
    lo = min(r[5] for r in rows if r[5])
    print(f'  최저 VMIN  {lo} mV' +
          ('   ⚠ 3900 미만 — 전원 확인' if lo < 3900 else '   ✅'))
    if any(r[6] for r in rows):
      print('  ⚠ 스톨 발생 — 기구 걸림 또는 전압 부족')
    else:
      print('  스톨 없음  ✅')
  n.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

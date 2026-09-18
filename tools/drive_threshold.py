#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_threshold.py — **구동이 굴러가기 시작하는 PWM** 을 직접 잰다.

★ 왜 (2026-09-18 밤)
  FF 식 `PWM = ff_gain·v + ff_static` 은 직선이라 **정지마찰 문턱을 못 건넌다.**
  control.launch.py 주석이 이미 경고하고 있었다:
      "45→0.00, 56→1.00 사이의 스틱슬립 구간이라 직선 적합을 그대로 믿으면
       안 된다"
  그날 밤 한 점(PWM 67 → 1.03 m/s)으로 직선을 그어 ff_static 23.4 를 냈는데,
  명령 0.5 의 PWM 35 는 문턱 아래라 차가 아예 안 움직였다. 계산은 맞았지만
  직선이 못 건너는 구간이었다.

  그래서 **재서** 정한다. 이 값이 ff_min_pwm 의 근거가 된다.

★ 명령과 측정을 한 프로세스에서 한다
  따로 돌리면 타이밍이 어긋나 데이터를 놓친다(그날 두 번 놓쳤다).

★ 전제
  · serial_bridge + mux 가 떠 있을 것 (teleop_drive.launch.py 로 충분)
  · **바퀴를 지면에 둘 것.** 들고 재면 무부하 문턱이라 실제와 다르다
    (그날 아침 무부하 50, 지면에서는 훨씬 높았다)
  · 앞을 비우고 E-stop 을 쥘 것 — 굴러가기 시작하면 실제로 나간다

  python3 tools/drive_threshold.py
  python3 tools/drive_threshold.py --from 40 --to 90 --step 5 --hold 2.5
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

M_PER_COUNT = 0.002875          # 2π·0.1327/290
MOVED_M = 0.05                  # 이만큼 굴렀으면 '움직였다'


class Rig(Node):

  def __init__(self):
    super().__init__('drive_threshold')
    self.pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
    self.enc = None
    self.vcc = None
    self.create_subscription(Int32, '/encoder_count',
                             lambda m: setattr(self, 'enc', m.data), 20)
    self.create_subscription(Int32, '/vcc_mv',
                             lambda m: setattr(self, 'vcc', m.data), 20)

  def spin(self, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
      rclpy.spin_once(self, timeout_sec=0.02)

  def hold(self, pwm, sec):
    """PWM 을 sec 초 인가하고 (이동거리, 최저 VCC) 를 낸다.

    openloop_timeout 0.5s 라 계속 발행해야 유지된다.
    """
    t0 = time.time()
    e0, vmin = self.enc, 9999
    while time.time() - t0 < sec:
      self.pub.publish(Int32(data=int(pwm)))
      rclpy.spin_once(self, timeout_sec=0.02)
      if e0 is None:
        e0 = self.enc
      if self.vcc:
        vmin = min(vmin, self.vcc)
    d = abs(self.enc - e0) * M_PER_COUNT if (self.enc is not None
                                             and e0 is not None) else 0.0
    return d, (vmin if vmin < 9999 else 0)

  def stop(self, sec=1.5):
    t0 = time.time()
    while time.time() - t0 < sec:
      self.pub.publish(Int32(data=0))
      rclpy.spin_once(self, timeout_sec=0.02)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--from', dest='lo', type=int, default=35)
  ap.add_argument('--to', dest='hi', type=int, default=85)
  ap.add_argument('--step', type=int, default=5)
  ap.add_argument('--hold', type=float, default=2.5)
  a = ap.parse_args()

  rclpy.init()
  n = Rig()
  print('★ 바퀴를 지면에 두고, 앞을 비우고, E-stop 을 쥘 것.')
  print('  /encoder_count 수신 대기...', flush=True)
  n.spin(2.0)
  if n.enc is None:
    print('❌ /encoder_count 가 안 온다 — serial_bridge 가 떠 있는지 확인할 것')
    return 1

  print(f'\n  {"PWM":>5} {"이동":>9} {"속도":>9} {"VMIN":>7}  판정')
  first = None
  rows = []
  for pwm in range(a.lo, a.hi + 1, a.step):
    d, vmin = n.hold(pwm, a.hold)
    n.stop()
    v = d / a.hold
    moved = d >= MOVED_M
    if moved and first is None:
      first = pwm
    rows.append((pwm, d, v, vmin, moved))
    print(f'  {pwm:5d} {d:8.3f}m {v:8.2f} {vmin:7d}  '
          f'{"← 굴렀다" if moved else "정지"}', flush=True)
  n.stop(1.0)

  print('\n' + '─' * 52)
  if first is None:
    print(f'  ❌ PWM {a.hi} 까지 올려도 안 굴렀다 — 배터리·기계를 볼 것')
    rc = 1
  else:
    print(f'  ★ 굴러가기 시작하는 PWM = {first}')
    print(f'    → ff_min_pwm 을 이 값 **위**로 잡는다 (여유 10 정도): '
          f'{first + 10}')
    print('    ⚠ 하한은 곧 최저 속도다. 필요 이상으로 올리면 곡률 제한이')
    print('      무력해져 급커브에서 이탈한다. 딱 문턱 위로만.')
    rc = 0
  print('─' * 52)
  rclpy.shutdown()
  return rc


if __name__ == '__main__':
  raise SystemExit(main())

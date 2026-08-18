#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""topspeed_test.py — 최고속도 측정 (공중 무부하 or 지상).

PWM 을 단계적으로 올리며 각 단계의 정상상태 속도를 기록해 'PWM → 속도' 곡선과
물리적 최고속을 뽑는다. 개루프(PWM:)라 PID 개입이 없어 순수한 모터 특성이 나온다.

★ 공중(무부하) 측정의 의미와 한계
  · 무부하라 지상보다 속도가 **높게** 나온다(구름저항·하중 없음).
  · 하지만 이 값은 드라이브트레인의 **물리적 상한**이다. 지상은 이보다 낮을 뿐
    절대 넘지 못한다. "목표 속도가 아예 불가능한지" 판별에는 충분하다.
  · FF 식별에는 쓰면 안 된다(지상 곡선이 필요). 그건 ff_sweep 을 지면에서.

사용 (serial_bridge 실행 중):
  ros2 run velocity_controller serial_bridge     # 다른 터미널
  python3 tools/topspeed_test.py                 # 이 스크립트

  # 바퀴를 들어 공중에 띄운 상태로 실행. 각 PWM 단계에서 2초 유지하며 속도 평균.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class TopSpeed(Node):

  def __init__(self, args):
    super().__init__('topspeed_test')
    self.steps = list(range(args.start, args.max + 1, args.step))
    self.hold = args.hold
    self.settle = args.settle
    self.pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
    self.create_subscription(Float64, '/current_speed', self.spd_cb, 10)
    self.speed = 0.0
    self.results = []          # (pwm, avg_speed)
    self.idx = 0
    self.buf = []
    self.t_step = time.time()
    self.done = False
    self.create_timer(0.05, self.tick)
    self.get_logger().info(
        f'최고속 측정: PWM {self.steps[0]}→{self.steps[-1]} '
        f'({args.step}씩), 각 {self.hold:.0f}초 유지. '
        '⚠ 바퀴를 공중에 띄웠는지 확인하고 E-stop 준비.')

  def spd_cb(self, msg):
    self.speed = abs(float(msg.data))

  def tick(self):
    if self.done:
      self.pub.publish(Int32(data=0))
      return
    pwm = self.steps[self.idx]
    self.pub.publish(Int32(data=pwm))
    el = time.time() - self.t_step
    # settle 이후 구간만 평균에 넣는다(가속 과도구간 제외)
    if el > self.settle:
      self.buf.append(self.speed)
    if el >= self.hold:
      avg = sum(self.buf) / len(self.buf) if self.buf else 0.0
      self.results.append((pwm, avg))
      self.get_logger().info(
          f'  PWM {pwm:3d} → {avg:.2f} m/s ({avg * 3.6:.1f} km/h)')
      self.buf = []
      self.idx += 1
      self.t_step = time.time()
      if self.idx >= len(self.steps):
        self.done = True
        self.pub.publish(Int32(data=0))
        self.summary()

  def summary(self):
    print('\n' + '=' * 56)
    print('최고속도 측정 결과 (공중 무부하일 수 있음 — 지상은 이 이하)')
    print('-' * 56)
    if not self.results:
      print('  데이터 없음'); print('=' * 56); return
    top = max(self.results, key=lambda r: r[1])
    print(f'  최고속도  : {top[1]:.2f} m/s = {top[1] * 3.6:.1f} km/h  (PWM {top[0]})')
    print('-' * 56)
    print(f'  {"PWM":>5} {"m/s":>7} {"km/h":>7}')
    for pwm, v in self.results:
      print(f'  {pwm:>5} {v:>7.2f} {v * 3.6:>7.1f}')
    print('-' * 56)
    print('  ※ 지상 최고속은 하중·구름저항으로 이보다 낮다.')
    print('    이 값보다 높은 목표속도는 지상에서 물리적으로 불가능하다.')
    print('=' * 56)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--start', type=int, default=60)
  ap.add_argument('--max', type=int, default=255)
  ap.add_argument('--step', type=int, default=20)
  ap.add_argument('--hold', type=float, default=2.5)    # 각 단계 유지[s]
  ap.add_argument('--settle', type=float, default=1.0)  # 이 이후만 평균
  args = ap.parse_args()
  rclpy.init()
  n = TopSpeed(args)
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      n.pub.publish(Int32(data=0))
    except Exception:  # noqa: BLE001
      pass
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

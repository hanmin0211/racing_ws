#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_grade_live.py — 경사를 **계속** 찍는다. 세워 놓고 읽어도, 밀고 다녀도 된다.

★ 왜 (2026-09-19 밤, 학교)
  imu_grade.py --measure 는 5초 평균을 한 번 낸다. 경사로처럼 **어디가 제일
  가파른지** 를 찾아야 할 때는 그걸로 부족하다 — 지점마다 세우고 다시 돌려야
  한다. 피치는 이미 계속 발행되고 있으니 그냥 계속 읽으면 된다.

  영점·부호는 imu_grade.py --level 이 이미 확정해 config/imu_pitch_offset.yaml
  에 넣어 뒀다. 여기서는 **읽기만** 한다 — 아무것도 저장하지 않는다.

★ 베끼지 않는다
  피치 변환·영점 적용은 imu_grade.py 에서, 필터/상한/게이트 상수는
  serial_bridge_node 에서 **그대로 가져온다**. 복사해 오면 원본이 바뀔 때
  이 도구가 거짓말한다(test_governor.py 가 쓰는 방식과 같다).

⚠ 가감속은 피치를 오염시킨다 — 1 m/s² 이 10%p 다(atan(a/g)).
  **세워 놓거나 등속일 때** 읽을 것. 거버너 게이트는 엔코더 가속도로 이 오염을
  빼지만, 이 도구는 엔코더를 안 본다.

  python3 tools/imu_grade_live.py
  python3 tools/imu_grade_live.py --gov-min-grade 0.06
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

import rclpy                                              # noqa: E402
from rclpy.node import Node                               # noqa: E402
from sensor_msgs.msg import Imu                           # noqa: E402

from imu_grade import load_cfg, pitch_from_quat           # noqa: E402
from velocity_controller.serial_bridge_node import SerialBridgeNode as SB  # noqa: E402


class Live(Node):

  def __init__(self, topic, gate):
    super().__init__('imu_grade_live')
    self.off, self.sign = load_cfg()
    self.gate = gate
    self.lp = None            # serial_bridge 와 같은 저역통과 상태
    self.lp_t = 0.0
    self.gate_open = False
    self.lo, self.hi = None, None
    self.n = 0
    self.last_print = 0.0
    self.create_subscription(Imu, topic, self._cb, 50)

  def _cb(self, msg):
    now = time.time()
    raw = self.sign * (pitch_from_quat(msg.orientation) - self.off)

    # GRADE_TAU 저역통과 — grade_ff 가 실제로 쓰는 값과 같게 본다
    if self.lp is None or self.lp_t == 0.0:
      self.lp, self.lp_t = raw, now
    else:
      dt = now - self.lp_t
      if dt > 0.0:
        self.lp += (dt / (SB.GRADE_TAU + dt)) * (raw - self.lp)
        self.lp_t = now

    self.n += 1
    grade = math.tan(self.lp) * 100.0
    self.lo = grade if self.lo is None else min(self.lo, grade)
    self.hi = grade if self.hi is None else max(self.hi, grade)

    # 거버너 자세 게이트 — 전진 기준 내리막이 +. 히스테리시스까지 같게.
    down = -math.sin(self.lp)
    if self.gate > 0.0:
      thr = self.gate * (SB.GOV_GATE_HYST if self.gate_open else 1.0)
      self.gate_open = down > thr

    if now - self.last_print < 0.2:
      return
    self.last_print = now

    if abs(self.lp) > SB.GRADE_MAX_RAD:
      flag = f'❌ 한계초과(>{math.degrees(SB.GRADE_MAX_RAD):.0f}°) 보상 0'
    else:
      flag = ''
    pwm = 9.81 * math.sin(self.lp) / SB.K_ACCEL_PER_PWM
    g = '열림' if self.gate_open else '닫힘'
    gs = f'  거버너게이트 {g}' if self.gate > 0.0 else ''
    print(f'\r  피치 {math.degrees(self.lp):+6.2f}°  경사 {grade:+6.1f}%  '
          f'grade_pwm {pwm:+6.1f}  (구간 {self.lo:+.1f}~{self.hi:+.1f}%)'
          f'{gs} {flag}   ', end='', flush=True)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--topic', default='handsfree/imu')
  ap.add_argument('--gov-min-grade', type=float, default=0.06,
                  help='거버너 자세 게이트 문턱. 0 이면 표시 안 함')
  a = ap.parse_args()

  rclpy.init()
  n = Live(a.topic, a.gov_min_grade)
  print(f'영점 {math.degrees(n.off):+.2f}° · 부호 {n.sign:+.0f}  '
        f'(config/imu_pitch_offset.yaml)')
  print(f'저역통과 GRADE_TAU={SB.GRADE_TAU}s · 상한 '
        f'{math.degrees(SB.GRADE_MAX_RAD):.1f}° · k={SB.K_ACCEL_PER_PWM}')
  print('⚠ 가감속은 피치를 오염시킨다(1 m/s² = 10%p). 세우거나 등속에서 읽을 것.')
  print('Ctrl+C 로 끝낸다.\n')
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    print()
    if n.n:
      print(f'\n  샘플 {n.n}개 · 본 경사 {n.lo:+.1f}% ~ {n.hi:+.1f}%')
    else:
      print(f'\n❌ {a.topic} 에서 아무것도 못 받았다 — IMU 가 떠 있는지 확인할 것')
      print(f'   ros2 topic hz {a.topic}')
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

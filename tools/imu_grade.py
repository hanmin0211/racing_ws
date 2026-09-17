#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_grade.py — IMU 피치로 경사를 잰다. 부호·영점·정확도를 **현장에서 확정**한다.

★ 왜 이게 먼저인가
  경사 보상(grade_ff)은 IMU 피치에 비례해 PWM 을 더한다. **부호가 뒤집혀
  있으면 내리막에서 가속한다.** 코드만 읽어서는 부호를 단정할 수 없다:
    · hfi_a9_ros2.py:207 이 이미 한 번 뒤집는다 (`-angle_degree[1]`)
    · IMU 를 차에 어떻게 얹었는지(앞/뒤, 뒤집힘)가 섞인다
    · 마운트가 수평이 아니면 평지에서도 0 이 아니다
  그래서 **차로 직접 재서** 확정한다. 재기 전에는 grade_ff 를 켜지 않는다.

  이미 독립적인 답이 있다: 2026-09-17 내리막 런에서 운동방정식으로 역산한
  **19.3%** (data/2026-09-17-ramp/ramp_4키로_2349.csv). IMU 가 같은 경사에서
  같은 값을 내면 두 방법이 서로를 검증한다.

사용 — 순서대로
  ① 평지에서 영점 (차를 평평한 곳에 세우고)
        python3 tools/imu_grade.py --level
     → config/imu_pitch_offset.yaml 에 마운트 기울기를 저장한다.

  ② 경사로에 세우고 측정 (코를 **위로** 두고)
        python3 tools/imu_grade.py --measure --expect 19.3
     → 부호가 +, 값이 19% 근처면 통과.

  ③ 코를 **아래로** 돌려 다시
        python3 tools/imu_grade.py --measure --expect -19.3

  ②가 음수로 나오면 부호가 반대다 → --sign -1 로 저장한다.
        python3 tools/imu_grade.py --level --sign -1
"""

import argparse
import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'config', 'imu_pitch_offset.yaml')


def pitch_from_quat(q):
    """쿼터니언 → 피치(rad). tf_transformations 없이 직접 — 의존성을 줄인다."""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def load_cfg():
    off, sign = 0.0, 1.0
    if os.path.exists(CFG):
        for line in open(CFG):
            line = line.split('#')[0].strip()
            if line.startswith('pitch_offset_rad:'):
                off = float(line.split(':', 1)[1])
            elif line.startswith('sign:'):
                sign = float(line.split(':', 1)[1])
    return off, sign


class Listener(Node):
    def __init__(self, topic):
        super().__init__('imu_grade')
        self.samples = []
        self.create_subscription(Imu, topic, self._cb, 50)

    def _cb(self, msg):
        self.samples.append(pitch_from_quat(msg.orientation))


def collect(topic, secs):
    rclpy.init()
    n = Listener(topic)
    print(f'  {topic} 수신 대기…', flush=True)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.samples and (time.time() - t0) > 0.5 and len(n.samples) % 50 == 0:
            print(f'\r  {len(n.samples)}샘플 · {time.time() - t0:.0f}s',
                  end='', flush=True)
    print()
    s = list(n.samples)
    n.destroy_node()
    rclpy.shutdown()
    return s


def report(s, label):
    if len(s) < 10:
        print(f'❌ 샘플 {len(s)}개 — IMU 가 발행 중인지 확인할 것')
        print('   ros2 topic hz handsfree/imu')
        return None
    med = statistics.median(s)
    sd = statistics.pstdev(s)
    print(f'  {label}  {len(s)}샘플')
    print(f'    피치 중앙 {math.degrees(med):+.2f}°  표준편차 {math.degrees(sd):.2f}°')
    if sd > math.radians(1.0):
        print('    ⚠ 흔들린다 — 차가 완전히 멈춰 있는지, 손을 뗐는지 확인')
    return med, sd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='handsfree/imu')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--level', action='store_true', help='평지 영점 저장')
    ap.add_argument('--measure', action='store_true', help='경사 측정')
    ap.add_argument('--expect', type=float, default=None,
                    help='기대 경사 [%%] — 부호 포함')
    ap.add_argument('--sign', type=float, default=None,
                    help='부호 뒤집기 (-1). --level 과 같이 저장된다')
    a = ap.parse_args()

    if not (a.level or a.measure):
        ap.error('--level 또는 --measure 중 하나를 줄 것')

    s = collect(a.topic, a.seconds)
    r = report(s, '평지' if a.level else '경사')
    if r is None:
        return 2
    med, sd = r

    if a.level:
        sign = a.sign if a.sign is not None else load_cfg()[1]
        os.makedirs(os.path.dirname(CFG), exist_ok=True)
        with open(CFG, 'w') as f:
            f.write('# imu_grade.py --level 이 쓴 파일. 손으로 고치지 말 것.\n')
            f.write('# 마운트 기울기 — 평지에서 읽은 피치. 측정에서 뺀다.\n')
            f.write(f'pitch_offset_rad: {med:.6f}\n')
            f.write('# 차가 코를 들었을 때 +가 되도록 하는 부호.\n')
            f.write(f'sign: {sign:.1f}\n')
        print()
        print(f'✅ 저장 {CFG}')
        print(f'   마운트 기울기 {math.degrees(med):+.2f}° · 부호 {sign:+.0f}')
        if abs(med) > math.radians(5.0):
            print('   ⚠ 5° 넘는다 — IMU 가 기울어 붙어 있거나 바닥이 평평하지 않다')
        return 0

    off, sign = load_cfg()
    if not os.path.exists(CFG):
        print()
        print('⚠ 영점 파일이 없다 — 마운트 기울기가 그대로 섞인다.')
        print('  먼저 평지에서: python3 tools/imu_grade.py --level')
    pitch = sign * (med - off)
    grade = math.tan(pitch) * 100.0
    print()
    print(f'  영점 {math.degrees(off):+.2f}° · 부호 {sign:+.0f}')
    print(f'  ▶ 보정 피치 {math.degrees(pitch):+.2f}°  →  경사 {grade:+.1f}%')
    # 이 차의 물성으로 환산 — 얼마나 밀어야 하는가
    #   g·sinθ / k,  k = 0.0202 m/s²/PWM (vehicle-climb-limits)
    pwm = 9.81 * math.sin(pitch) / 0.0202
    print(f'    이 경사를 상쇄하는 PWM = g·sinθ/k = {pwm:+.0f}')

    if a.expect is None:
        return 0
    print()
    if grade * a.expect < 0:
        print(f'❌ **부호가 반대다** (기대 {a.expect:+.1f}%, 측정 {grade:+.1f}%)')
        print('   이대로 grade_ff 를 켜면 내리막에서 가속한다.')
        print('   평지에서 다시: python3 tools/imu_grade.py --level --sign '
              f'{-sign:+.0f}')
        return 1
    err = abs(grade - a.expect)
    tol = max(2.0, abs(a.expect) * 0.20)
    if err <= tol:
        print(f'✅ 기대 {a.expect:+.1f}% · 측정 {grade:+.1f}% · 오차 {err:.1f}%p '
              f'(허용 {tol:.1f})')
        print('   운동방정식 역산과 IMU 가 같은 답을 냈다 — 서로 검증됐다.')
        return 0
    print(f'❌ 기대 {a.expect:+.1f}% · 측정 {grade:+.1f}% · 오차 {err:.1f}%p '
          f'(허용 {tol:.1f})')
    print('   영점을 다시 잡거나, 차가 경사 한가운데 있는지 확인할 것')
    print('   (경사로 시작·끝에 걸치면 차체가 경사를 다 안 탄다)')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())

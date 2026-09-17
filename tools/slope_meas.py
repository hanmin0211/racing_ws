#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""slope_meas.py — 경사를 **차로 직접** 잰다. 엔코더만 쓰고 모터는 안 쓴다.

★ 원리
  차를 경사 위로 밀었다 놓으면 올라가며 감속하고, 멈췄다 내려오며 가속한다.
  두 구간을 같이 재면 구름저항이 상쇄된다:

      올라갈 때  |a_up|  = g·sinθ + f
      내려올 때   a_down = g·sinθ − f
        ⇒  g·sinθ = (|a_up| + a_down) / 2      ← 경사
           f       = (|a_up| − a_down) / 2      ← 구름저항(덤)

  한 번 밀면 두 구간이 다 잡힌다. 모터를 안 쓰므로 드라이버 발열과 무관하고,
  IMU·GPS 도 필요 없다.

★ 왜 필요한가 (2026-09-17)
  경사 시험에서 차가 0.8m 올라가고 섰는데, 그 감속(4.56 m/s²)을 경사로만
  설명하려면 **52% 경사**여야 한다. 정상 경사로가 아니다 — 진입부 단차를
  의심했지만 **경사를 잰 적이 없어** 확정을 못 했다. 이 도구가 그걸 끝낸다.

사용:
  1) serial_bridge 가 떠 있어야 한다 (teleop_drive.launch.py)
  2) 차를 경사 **아래쪽**에 두고, 위로 손으로 밀었다 놓는다
  3) python3 tools/slope_meas.py

  ⚠ 놓은 뒤 차가 굴러 내려온다. 아래를 비우고 받을 준비를 할 것.
"""

import argparse
import math
import time

G = 9.81
COUNTS_PER_REV, WHEEL_R = 290.0, 0.1327
M_PER_COUNT = (2 * math.pi * WHEEL_R) / COUNTS_PER_REV


def fit_accel(ts, vs):
  """(시각, 속도) 에 직선을 맞춰 가속도 [m/s²] 와 표본수를 돌려준다."""
  n = len(ts)
  if n < 5:
    return None, n
  mt = sum(ts) / n
  mv = sum(vs) / n
  den = sum((x - mt) ** 2 for x in ts)
  if den < 1e-9:
    return None, n
  return sum((x - mt) * (y - mv) for x, y in zip(ts, vs)) / den, n


def analyse(t, v, quiet=False):
  """전진(+)으로 밀었다 놓은 기록에서 경사를 낸다."""
  # 올라가는 구간: v > 0 이면서 줄어드는 곳 (손을 뗀 뒤)
  peak = max(range(len(v)), key=lambda i: v[i])
  up_t, up_v = [], []
  for i in range(peak, len(v)):
    if v[i] <= 0.05:
      break
    up_t.append(t[i])
    up_v.append(v[i])
  # 내려오는 구간: 그 뒤 v < 0 이 이어지는 곳
  down_t, down_v = [], []
  started = False
  for i in range(peak, len(v)):
    if v[i] < -0.05:
      started = True
      down_t.append(t[i])
      down_v.append(v[i])
    elif started:
      break

  # ★ 손을 안 뗀 런을 걸러낸다. 계속 밀면 속도가 **일정**하게 유지된다.
  #   실측(2026-09-17): 0.8 m/s 가 10초 동안 유지돼 '올라감 가속도 -0.03' 이
  #   나왔다 — 그건 감속이 아니라 등속 밀기다.
  if len(up_v) > 20:
    mu = sum(up_v) / len(up_v)
    sd = (sum((x - mu) ** 2 for x in up_v) / len(up_v)) ** 0.5
    span = up_t[-1] - up_t[0]
    if span > 3.0 and sd / max(mu, 1e-6) < 0.25:
      print(f'\n❌ {span:.1f}초 동안 속도가 {mu:.2f} m/s 로 거의 일정했다 '
            f'(변동 {100 * sd / mu:.0f}%).')
      print('   **손을 안 뗀 것**이다. 세게 밀고 **완전히 놓아야** 한다 —')
      print('   감속 → 정지 → 되돌아 내려옴, 이 세 구간이 다 나와야 계산된다.')
      return None

  a_up, n_up = fit_accel(up_t, up_v)
  a_dn, n_dn = fit_accel(down_t, down_v)
  if not quiet:
    print(f'\n   올라감  {n_up:3d}샘플 · 가속도 '
          f'{a_up if a_up is not None else float("nan"):+.2f} m/s²')
    print(f'   내려옴  {n_dn:3d}샘플 · 가속도 '
          f'{a_dn if a_dn is not None else float("nan"):+.2f} m/s²')

  if a_up is None or a_dn is None:
    print('\n❌ 두 구간을 다 못 잡았다. 더 세게 밀어 올리고, 놓은 뒤 차가'
          ' 되돌아 내려올 때까지 기록을 유지할 것.')
    return None

  gs = (abs(a_up) + abs(a_dn)) / 2.0          # g·sinθ
  f = (abs(a_up) - abs(a_dn)) / 2.0           # 구름저항
  if gs >= G:
    print('\n❌ 계산값이 중력보다 크다 — 손이 닿았거나 벽에 부딪힌 구간이 섞였다')
    return None
  th = math.degrees(math.asin(gs / G))
  grade = math.tan(math.radians(th)) * 100
  print('\n   ── 결과 ──')
  print(f'   경사      {grade:.1f}%   ({th:.1f}°)')
  print(f'   구름저항   {f:+.2f} m/s²   (평지 실측 이력 0.31~0.43)')
  if f < 0:
    print('   ⚠ 구름저항이 음수다 — 두 구간의 노면이 다르거나 측정이 짧다')
  print()
  # 해석
  if grade < 5:
    print('   → 거의 평지다. 경사 때문에 못 올라간 게 아니다')
  elif grade < 15:
    print('   → 일반적인 경사로다(운전면허 시험장 수준). 유아용 전동차가'
          ' 못 오를 경사가 아니다.')
    print('     그래도 못 올라갔다면 **진입부 단차·이물**을 볼 것.')
  elif grade < 30:
    print('   → 가파르다. 조주 없이는 어렵다. 필요 진입속도를 아래에 낸다.')
  else:
    print('   → 매우 가파르다. 이 차의 한계를 넘을 수 있다.')
  # 필요 진입속도 (모터 없이 순수 관성으로 L 미터를 오르려면)
  print('\n   모터 없이 관성만으로 오르려면 (참고):')
  for L in (2, 5, 10):
    need = math.sqrt(2 * (gs + max(f, 0.0)) * L)
    print(f'     {L:2d}m  →  진입 {need:.2f} m/s')
  return grade


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--seconds', type=float, default=20.0)
  ap.add_argument('--report', default=None, help='기록된 CSV 를 다시 계산만')
  ap.add_argument('--out', default=None)
  a = ap.parse_args()

  if a.report:
    import csv
    rows = list(csv.DictReader(open(a.report)))
    t = [float(r['t']) for r in rows]
    v = [float(r['v']) for r in rows]
    return 0 if analyse(t, v) is not None else 1

  import csv
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import Float64, Int32

  class S(Node):
    def __init__(self):
      super().__init__('slope_meas')
      self.enc = None
      self.v = 0.0
      self.create_subscription(Int32, '/encoder_count',
                               lambda m: setattr(self, 'enc', m.data), 10)
      self.create_subscription(Float64, '/current_speed',
                               lambda m: setattr(self, 'v', m.data), 10)

  rclpy.init()
  n = S()
  end = time.time() + 8
  while n.enc is None and time.time() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
  if n.enc is None:
    print('❌ /encoder_count 가 안 온다 — serial_bridge 가 떠 있는지 확인할 것')
    rclpy.shutdown()
    return 1

  print(f'\n▶ {a.seconds:.0f}초 기록. **차를 경사 위로 밀었다 놓으세요.**')
  print('  ⚠ 놓으면 굴러 내려옵니다. 아래를 비우고 받을 준비를 하세요.')
  print(f'  {"t":>6} {"속도":>7}')
  t0 = time.time()
  T, V, last = [], [], 0.0
  try:
    while time.time() - t0 < a.seconds and rclpy.ok():
      rclpy.spin_once(n, timeout_sec=0.01)
      tt = time.time() - t0
      if T and tt - T[-1] < 0.02:
        continue
      T.append(tt)
      V.append(n.v)
      if tt - last >= 0.5:
        last = tt
        bar = '█' * min(30, int(abs(n.v) * 12))
        print(f'  {tt:6.1f} {n.v:7.2f}  {bar}')
  except KeyboardInterrupt:
    print('\n(중단)')
  rclpy.shutdown()

  out = a.out or f'/tmp/slope_{time.strftime("%H%M")}.csv'
  with open(out, 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['t', 'v'])
    w.writerows(zip(T, V))
  print(f'\n저장: {out}  ({len(T)}샘플)')
  return 0 if analyse(T, V) is not None else 1


if __name__ == '__main__':
  raise SystemExit(main())

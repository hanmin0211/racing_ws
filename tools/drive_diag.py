#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_diag.py — 구동계가 전력을 받고 있는지 **조향과 비교해서** 가른다.

★ 왜 (2026-09-17)
  경사 시험 중 구동이 죽었다. 증상:
    바퀴를 들어 부하 0 인데 PWM 200 에 안 돔 · 전압 변동 47mV
  잠긴 모터는 역기전력이 없어 전류를 최대로 빤다(9/13 실측: PWM 50 으로 안
  굴렀을 때 VMIN 2938mV). 전류 흔적이 없다는 건 **모터까지 전력이 안 간다**는
  뜻이다. 그런데 조향은 멀쩡하다 — 같은 배터리에서.

  그래서 둘을 같은 기준으로 재서 가른다:
    · 조향은 전압을 끌어내리는데 구동은 안 끌어내린다 → **구동 전원 계통**
    · 둘 다 안 끌어내린다                              → 공통 전원/배터리
    · 구동도 끌어내리는데 안 돈다                      → 모터/기계 고착

⚠ **바퀴를 들고** 할 것. 갑자기 살아나면 차가 튀어나간다.
  구동이 반응하는 순간 즉시 멈춘다(엔코더 2카운트면 중단).

  python3 tools/drive_diag.py
"""

import argparse
import statistics
import time


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--max-pwm', type=int, default=200)
  ap.add_argument('--step', type=int, default=50)
  ap.add_argument('--hold', type=float, default=1.2, help='각 단계 인가 시간 [s]')
  ap.add_argument('--yes', action='store_true', help='바퀴를 들었다는 확인')
  a = ap.parse_args()

  if not a.yes:
    print('⚠ 바퀴를 들고 하세요. 갑자기 살아나면 차가 튀어나갑니다.')
    print('  확인했으면 --yes 를 붙여 다시 실행하세요.')
    return 2

  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist
  from std_msgs.msg import Float64, Int32

  class D(Node):
    def __init__(self):
      super().__init__('drive_diag')
      self.enc = None
      self.vcc = 0
      self.steer = 0.0
      self.create_subscription(Int32, '/encoder_count',
                               lambda m: setattr(self, 'enc', m.data), 10)
      self.create_subscription(Int32, '/vcc_mv',
                               lambda m: setattr(self, 'vcc', m.data), 10)
      self.create_subscription(Float64, '/steering_angle',
                               lambda m: setattr(self, 'steer', m.data), 10)
      self.pwm_pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
      self.tel_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)

  rclpy.init()
  n = D()
  t_end = time.time() + 8
  while n.enc is None and time.time() < t_end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
  if n.enc is None:
    print('❌ /encoder_count 가 안 온다 — serial_bridge 가 떠 있는지 확인할 것')
    rclpy.shutdown()
    return 1

  def sample(sec):
    """sec 동안 돌면서 (vcc 목록, 엔코더 변화) 를 모은다."""
    vs, e0 = [], n.enc
    end = time.time() + sec
    while time.time() < end and rclpy.ok():
      rclpy.spin_once(n, timeout_sec=0.02)
      if n.vcc > 500:
        vs.append(n.vcc)
    return vs, abs(n.enc - e0)

  def pwm(v):
    n.pwm_pub.publish(Int32(data=int(v)))

  def steer(deg):
    m = Twist()
    m.linear.x = 0.0
    m.angular.z = float(deg)
    n.tel_pub.publish(m)

  results = {}
  try:
    print('\n[기준] 아무것도 안 걸고 3초')
    base, _ = sample(3.0)
    b_med = statistics.median(base) if base else 0
    print(f'   vcc 중앙 {b_med:.0f} · 최저 {min(base) if base else 0}')

    print(f'\n[구동] PWM 을 0 → {a.max_pwm} 로 올린다 (각 {a.hold}s)')
    print(f'   {"PWM":>5} {"vcc중앙":>8} {"vcc최저":>8} {"강하":>6} {"엔코더":>7}')
    drive_sag, moved_at = 0, None
    for p in range(a.step, a.max_pwm + 1, a.step):
      end = time.time() + a.hold
      vs, e0 = [], n.enc
      while time.time() < end and rclpy.ok():
        pwm(p)                      # openloop_timeout 0.5s → 계속 갱신해야 한다
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.vcc > 500:
          vs.append(n.vcc)
        if abs(n.enc - e0) > 2:     # 살아났다 — 즉시 멈춘다
          moved_at = p
          break
      med = statistics.median(vs) if vs else 0
      lo = min(vs) if vs else 0
      sag = b_med - lo
      drive_sag = max(drive_sag, sag)
      print(f'   {p:5d} {med:8.0f} {lo:8.0f} {sag:6.0f} {abs(n.enc - e0):7d}'
            + ('  ← 돌았다!' if moved_at else ''))
      pwm(0)
      if moved_at:
        break
      time.sleep(0.3)
    pwm(0)
    results['drive_sag'] = drive_sag
    results['moved'] = moved_at

    print('\n[조향] 좌우로 끝까지 — 같은 배터리에서 전류를 끄는지 본다')
    steer_sag = 0
    for deg in (-18.0, 18.0, -18.0, 0.0):
      end = time.time() + 1.5
      vs = []
      while time.time() < end and rclpy.ok():
        steer(deg)
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.vcc > 500:
          vs.append(n.vcc)
      lo = min(vs) if vs else 0
      steer_sag = max(steer_sag, b_med - lo)
      print(f'   {deg:+6.1f}°  vcc최저 {lo:.0f} · 강하 {b_med - lo:.0f} '
            f'· 실제각 {n.steer:+.1f}°')
    steer(0.0)
    results['steer_sag'] = steer_sag
  except KeyboardInterrupt:
    print('\n(중단)')
  finally:
    for _ in range(5):
      try:
        pwm(0)
        steer(0.0)
        rclpy.spin_once(n, timeout_sec=0.02)
      except Exception:            # noqa: BLE001
        break

  print('\n' + '─' * 60)
  ds, ss = results.get('drive_sag', 0), results.get('steer_sag', 0)
  print(f'  전압 강하   구동 {ds:.0f} mV   ·   조향 {ss:.0f} mV')
  # ★ 조향 강하는 구동 판정과 **독립적으로** 봐야 한다. 실측에서 구동이
  #   멀쩡한데 조향이 레일을 2038mV 까지 끌어내렸다 — ATmega2560 의 동작
  #   하한 아래이고, 과거 리셋 발생선(3483)보다도 한참 아래다.
  #   ⚠ 바퀴를 **든 채로** 잰 값이다. 접지 마찰이 없는데도 이만큼 무너진다.
  if ss >= 1000:
    print(f'  ❌❌ **조향이 전원을 무너뜨린다** — {ss:.0f}mV 강하. '
          f'브라운아웃 위험선(4300)·과거 리셋선(3483) 을 한참 아래로 지난다.')
    print('      기록된 주행의 VMIN 2282~2528 도 구동이 아니라 이것이다.')
    print('      S자·굴절처럼 조향이 계속 큰 구간에서 링크가 끊길 수 있다.')
  elif ss >= 500:
    print(f'  ⚠  조향 강하 {ss:.0f}mV — 큰 편이다. 주행 중 VMIN 을 감시할 것.')

  if results.get('moved'):
    print(f'  ✅ PWM {results["moved"]} 에서 **돌았다** — 구동은 살아 있다. '
          f'그 아래는 정지마찰 문턱이다')
    rc = 1 if ss >= 1000 else 0
  elif ss >= 150 and ds < 100:
    print('  ❌ **구동 전원 계통 고장** — 조향은 전류를 끄는데 구동은 안 끈다.')
    print('     같은 배터리이므로 배터리 문제가 아니다. 구동 모터 커넥터 ·')
    print('     퓨즈 · 모터 드라이버(열보호 포함) 를 볼 것.')
    rc = 1
  elif ds >= 150:
    print('  ❌ 전류는 흐르는데 안 돈다 — **모터/기계 고착** 또는 모터 자체 고장.')
    print('     바퀴를 손으로 돌려 보고, 기어·구동축이 걸리는지 볼 것.')
    rc = 1
  else:
    print('  ❌ 구동·조향 **둘 다** 전류 흔적이 약하다 — 공통 전원을 의심.')
    print(f'     배터리 잔량 · 메인 커넥터 · 접지를 볼 것 (기준 vcc {b_med:.0f}mV).')
    rc = 1
  print('─' * 60)
  rclpy.shutdown()
  return rc


if __name__ == '__main__':
  raise SystemExit(main())

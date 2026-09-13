#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ff_identify.py — 지면에서 **PWM → 실제 속도** 를 재서 구동 FF 를 다시 잡는다.

★ 왜 (2026-09-12 학교 현장)
  펌웨어의 개루프 FF 는  PWM = STATIC_FF + VELOCITY_FF_GAIN·v  (= 80 + 95·v) 인데,
  실측해 보니 **전혀 안 맞는다.** 명령 0.15 m/s → PWM 94 인데 차가 **2 m/s 까지
  가속**했다(13배). tools/calib_trace.py 로 teleop→cmd_vel→PWM 전달은 완벽하고
  PWM 도 식대로 나가는 것을 확인했으므로, 틀린 것은 **상수 자체**다.
  특히 정지마찰 항 80 이 이 지면에서 과도하다 — 최저 명령에도 PWM 94 가 걸린다.

  엔코더가 꺼져 있어(NO_ENCODER 1) 차는 자기 속도를 모른다. 그래서 **GPS 로** 잰다.

★ 방법
  /drive_pwm_cmd (Int32) 로 개루프 PWM 을 직접 건다. serial_bridge 가 FF 를
  건너뛰고 `PWM:x` 를 그대로 보낸다(0 을 보내거나 0.5s 끊기면 폐루프 복귀).
  각 단계마다 일정 거리(--run-dist)를 갈 때까지 굴리고 평균·최대 속도를 잰다.

  ⚠⚠ **차가 실제로 달린다.** 앞을 충분히 비우고 E-stop 을 손에 쥘 것.
     단계마다 정지 후 차를 되돌려야 할 수 있다(--pause 로 대기).

사용 (bringup 은 control:=true 로 떠 있어야 한다):
  python3 tools/ff_identify.py --pwms 50 60 70 80 90 --run-dist 3.0 --pause
  python3 tools/ff_identify.py --breakaway          # 움직이기 시작하는 PWM 만 탐색
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

M_PER_DEG = 111320.0


class Runner(Node):

  def __init__(self):
    super().__init__('ff_identify')
    self.pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
    self.lat = None
    self.lon = None
    self.sig = float('nan')
    # ★ 2026-09-13 — 공급전압을 같이 기록한다.
    #   같은 PWM 55 에서 0.46 m/s(ff_identify) 와 1.00 m/s(캘리브 10m 주행)가
    #   나왔다. 둘 다 GPS 실측이라 '측정이 틀린' 게 아니라 **조건이 달랐다**.
    #   가장 유력한 건 모터 전압이다 — 이 프로젝트에서 '배터리 연결 안 함' 으로
    #   측정 하나가 통째로 무효가 된 적이 이미 있다.
    #   전압을 안 남기면 나중에 어느 런이 유효한지 판별할 방법이 없다.
    self.vcc = None
    self.vmin = None
    self.vmin_seen = None          # 스텝 동안 본 최저값
    # ⚠ 토픽 이름은 /vcc_mv · /vcc_min_mv 다(serial_bridge_node 에서 확인).
    #   /vcc · /vmin 으로 적으면 **조용히 아무것도 안 들어온다**.
    self.create_subscription(Int32, '/vcc_mv',
                             lambda m: setattr(self, 'vcc', m.data), 10)
    self.create_subscription(Int32, '/vcc_min_mv', self._vmin, 10)
    self.create_subscription(NavSatFix, '/fix', self._fix,
                             qos_profile_sensor_data)

  def _vmin(self, m):
    self.vmin = m.data
    if m.data > 0 and (self.vmin_seen is None or m.data < self.vmin_seen):
      self.vmin_seen = m.data

  def _fix(self, m):
    self.lat, self.lon = m.latitude, m.longitude
    self.sig = math.sqrt(m.position_covariance[0])

  def wait_fix(self, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout and self.lat is None:
      rclpy.spin_once(self, timeout_sec=0.1)
    return self.lat is not None

  def dist_from(self, lat0, lon0):
    return math.hypot((self.lon - lon0) * M_PER_DEG
                      * math.cos(math.radians(lat0)),
                      (self.lat - lat0) * M_PER_DEG)

  def send(self, pwm):
    # 종료(Ctrl-C) 후에는 컨텍스트가 죽어 publish 가 예외를 낸다 — 조용히 무시.
    try:
      self.pub.publish(Int32(data=int(pwm)))
    except Exception:  # noqa: BLE001
      pass

  def hold_stop(self, sec=3.0):
    """PWM 0 을 계속 보내며 완전히 멈출 때까지 기다린다."""
    t0 = time.time()
    while time.time() - t0 < sec:
      self.send(0)
      rclpy.spin_once(self, timeout_sec=0.05)

  def wait_rtk(self, sigma_max, timeout=120.0):
    """RTK 수렴을 기다린다.

    ★ 이걸 안 해서 1회차 측정이 통째로 거짓이었다 (2026-09-12).
      σ=15.6cm 상태로 재니 PWM 30 에서 '1.52m 이동' 이 나왔는데, 차는 가만히
      있었고 **GPS 가 수렴하며 떠다닌 거리**를 이동으로 센 것이었다.
      같은 PWM 을 σ=0.0cm 에서 다시 재니 0.00m 였다.
      위치로 속도를 재는 도구는 측위 품질을 먼저 확인해야 한다.
    """
    t0 = time.time()
    warned = False
    while time.time() - t0 < timeout:
      rclpy.spin_once(self, timeout_sec=0.1)
      if self.sig == self.sig and self.sig <= sigma_max:
        return True
      if not warned and time.time() - t0 > 3.0:
        warned = True
        print(f'  RTK 수렴 대기 중 (σ={self.sig * 100:.1f}cm > '
              f'{sigma_max * 100:.0f}cm) — 수렴 전 측정은 거짓이 된다')
    return False

  def refresh(self, sec=1.0):
    """위치를 최신으로 갱신한다.

    ★ --pause 의 input() 은 ROS 를 안 돌린다. 그 동안 self.lat/lon 이 멈춰 있어,
      사용자가 차를 되돌린 뒤 그 **옛 위치**를 기준점으로 잡으면 되돌린 거리가
      통째로 '이동' 으로 잡힌다 (2026-09-12: '이동 28.30m / 0.0s').
      단계 시작 전에 반드시 이걸 부를 것.
    """
    t0 = time.time()
    while time.time() - t0 < sec:
      rclpy.spin_once(self, timeout_sec=0.05)

  def run_step(self, pwm, run_dist, max_t, rate=20.0):
    """PWM 을 걸고 run_dist 를 갈 때까지(또는 max_t) 굴린다."""
    self.refresh(1.0)          # 기준점을 최신 위치로
    self.vmin_seen = None      # 이 스텝 동안의 최저 공급전압
    lat0, lon0 = self.lat, self.lon
    t0 = time.time()
    samples = []          # (t, dist)
    last_send = 0.0
    while True:
      now = time.time()
      if now - last_send >= 1.0 / rate:
        last_send = now
        self.send(pwm)
      rclpy.spin_once(self, timeout_sec=0.02)
      d = self.dist_from(lat0, lon0)
      samples.append((now - t0, d))
      if d >= run_dist or (now - t0) >= max_t:
        break
    # 정지
    self.hold_stop(3.0)
    d_end = self.dist_from(lat0, lon0)
    took = samples[-1][0] if samples else 0.0
    # 후반 절반의 평균속도 = 정상상태에 가깝다
    half = [s for s in samples if s[0] > took / 2]
    v_ss = 0.0
    if len(half) >= 2:
      v_ss = (half[-1][1] - half[0][1]) / max(half[-1][0] - half[0][0], 1e-3)
    # 최대속도 — 0.5초 창으로 본다.
    # (예전엔 '연속 두 샘플의 dt > 0.2s' 를 봤는데 샘플이 0.02s 간격이라
    #  그 조건이 절대 참이 안 돼 항상 0.00 이 찍혔다)
    v_peak = 0.0
    j = 0
    for i in range(len(samples)):
      while samples[i][0] - samples[j][0] > 0.5:
        j += 1
      dt = samples[i][0] - samples[j][0]
      if dt >= 0.3:
        v_peak = max(v_peak, (samples[i][1] - samples[j][1]) / dt)
    # ★ 덜컹거린 런은 '정상상태' 를 뽑으면 안 된다.
    #   PWM 55 런이 12초에 3.60m(평균 0.30) 를 가 놓고 정상상태 0.46, 최대 0.77
    #   이었다. 정지마찰 근처에서 섰다 굴렀다 한 것이라 어떤 한 숫자로도
    #   대표할 수 없다. 그런데 그 0.46 이 그대로 FF 상수가 됐다.
    #   평균과 정상상태가 크게 어긋나면 그 스텝은 신뢰할 수 없다고 표시한다.
    v_mean = (samples[-1][1] / took) if took > 0 else 0.0
    steady = True
    if v_ss > 0.05 and v_mean > 0.05:
      # 가속 구간이 있으니 v_ss > v_mean 은 정상. 반대로 벌어지거나
      # 최대속도가 정상상태의 1.5배를 넘으면 덜컹댄 것이다.
      if v_peak > v_ss * 1.5 or v_mean > v_ss * 1.2:
        steady = False
    return dict(pwm=pwm, moved=samples[-1][1] if samples else 0.0,
                took=took, v_mean=v_mean,
                v_ss=v_ss, v_peak=v_peak, steady=steady,
                vmin=self.vmin_seen,
                coast=d_end - (samples[-1][1] if samples else 0.0))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--pwms', type=int, nargs='*',
                  default=[50, 60, 70, 80, 90])
  ap.add_argument('--run-dist', type=float, default=3.0,
                  help='단계마다 굴릴 거리[m]')
  ap.add_argument('--max-t', type=float, default=8.0,
                  help='단계 최대 시간[s] (안 움직이면 여기서 끝)')
  ap.add_argument('--pause', action='store_true',
                  help='단계 사이에 Enter 를 기다린다 (차를 되돌릴 때)')
  ap.add_argument('--breakaway', action='store_true',
                  help='움직이기 시작하는 PWM 만 낮은 값부터 탐색')
  ap.add_argument('--sigma-max', type=float, default=0.05,
                  help='이 수평정확도[m] 안으로 수렴해야 측정한다 (기본 5cm)')
  ap.add_argument('--yes', action='store_true')
  a = ap.parse_args()

  print('=' * 70)
  print('구동 FF 식별 — 지면에서 PWM → 실제 속도 (GPS 측정)')
  print('=' * 70)
  print('  ⚠⚠ 차가 실제로 달린다. 앞을 충분히 비우고 E-stop 을 손에 쥘 것.')
  print(f'  단계 {a.pwms if not a.breakaway else "브레이크어웨이 탐색"} · '
        f'단계당 {a.run_dist:.1f}m 또는 {a.max_t:.0f}초')
  print('=' * 70)
  if not a.yes:
    try:
      if input('진행? [y/N] ').strip().lower() not in ('y', 'yes'):
        return 0
    except EOFError:
      print('입력 없음 — 중단. (자동 실행이면 --yes)')
      return 0

  rclpy.init()
  n = Runner()
  if not n.wait_fix():
    print('❌ /fix 수신 없음 — bringup 이 떠 있는지 확인할 것')
    rclpy.shutdown()
    return 1
  print(f'GPS 수신 (σ={n.sig * 100:.1f}cm) — RTK 수렴을 기다린다…')
  if not n.wait_rtk(a.sigma_max):
    print(f'❌ RTK 가 {a.sigma_max * 100:.0f}cm 안으로 수렴하지 않았다. '
          '수렴 전 측정은 거짓이 된다 — 기다렸다 다시 할 것.')
    rclpy.shutdown()
    return 1
  print(f'✅ RTK 수렴 (σ={n.sig * 100:.1f}cm) — 측정 시작\n')

  rows = []
  try:
    pwms = a.pwms
    if a.breakaway:
      pwms = list(range(30, 101, 5))
    for pwm in pwms:
      if a.pause and rows:
        try:
          input(f'  차를 되돌리고 Enter → 다음 단계 PWM {pwm} ')
        except EOFError:
          pass
      print(f'  ▶ PWM {pwm} …', end='', flush=True)
      r = n.run_step(pwm, a.run_dist, a.max_t)
      rows.append(r)
      if r['took'] < 0.5:
        print(f' ❌ 무효 (측정시간 {r["took"]:.1f}s) — 기준점이 갱신되기 전에 '
              '거리 조건을 넘었다. 다시 잴 것')
        rows.pop()
        continue
      vtxt = f'{r["vmin"]}mV' if r['vmin'] else '전압?'
      print(f' 이동 {r["moved"]:5.2f}m / {r["took"]:4.1f}s  '
            f'정상상태 {r["v_ss"]:5.2f} m/s  최대 {r["v_peak"]:5.2f}  '
            f'관성 {r["coast"]:4.2f}m  σ {n.sig * 100:.1f}cm  {vtxt}')
      if not r['steady']:
        print(f'     ⚠ 덜컹거렸다 (평균 {r["v_mean"]:.2f} · 정상상태 '
              f'{r["v_ss"]:.2f} · 최대 {r["v_peak"]:.2f}). 정지마찰 근처라 '
              'FF 적합에서 **제외**한다')
      if r['vmin'] and r['vmin'] < 4300:
        print(f'     ⚠ 공급전압이 {r["vmin"]}mV 까지 떨어졌다 — 배터리 상태를 '
              '확인할 것. 전압이 다르면 같은 PWM 이 다른 속도를 낸다')
      if r['vmin'] is None:
        print('     ⚠ 공급전압 텔레메트리가 없다 — 배터리가 연결돼 있는지 '
              '확인할 것 (연결 안 하고 잰 측정이 과거에 통째로 무효였다)')
      if n.sig > a.sigma_max:
        print(f'     ⚠ 측정 중 σ 가 {n.sig * 100:.1f}cm 로 올라갔다 — '
              '이 단계는 믿지 말 것')
      if a.breakaway and r['moved'] > 0.3:
        print(f'\n  → 이 차/이 지면의 **브레이크어웨이 PWM ≈ {pwm}**')
        break
  except KeyboardInterrupt:
    print('\n중단')
  finally:
    n.hold_stop(2.0)
    try:
      n.destroy_node()
    except Exception:  # noqa: BLE001
      pass
    if rclpy.ok():
      rclpy.shutdown()

  # 덜컹거린 스텝은 적합에서 뺀다 — 그 한 점이 FF 를 통째로 흔든다.
  moved = [r for r in rows
           if r['moved'] > 0.3 and r['v_ss'] > 0.02 and r['steady']]
  dropped = [r for r in rows
             if r['moved'] > 0.3 and r['v_ss'] > 0.02 and not r['steady']]
  print('\n' + '=' * 70)
  print(f"{'PWM':>5} {'이동':>7} {'시간':>6} {'정상상태':>9} {'최대':>7} "
        f"{'관성':>6} {'최저전압':>9} {'판정':>6}")
  print('-' * 70)
  for r in rows:
    v = f'{r["vmin"]}mV' if r['vmin'] else '—'
    print(f'{r["pwm"]:5d} {r["moved"]:6.2f}m {r["took"]:5.1f}s '
          f'{r["v_ss"]:8.2f} {r["v_peak"]:6.2f} {r["coast"]:5.2f}m '
          f'{v:>9} {"OK" if r["steady"] else "덜컹":>6}')
  print('-' * 70)
  if dropped:
    print(f'  덜컹거려 제외한 단계: '
          f'{", ".join(str(r["pwm"]) for r in dropped)}')
  vs = [r['vmin'] for r in rows if r['vmin']]
  if vs and (max(vs) - min(vs)) > 300:
    print(f'  ⚠ 단계별 공급전압이 {min(vs)}~{max(vs)}mV 로 벌어졌다. '
          '전압이 다르면 같은 PWM 이 다른 속도를 낸다 — 이 적합은 믿을 수 없다.')
  if len(moved) >= 2:
    # v = (PWM - b) / a  →  PWM = a·v + b  로 최소제곱
    xs = [r['v_ss'] for r in moved]
    ys = [float(r['pwm']) for r in moved]
    nn = len(xs)
    mx = sum(xs) / nn
    my = sum(ys) / nn
    den = sum((x - mx) ** 2 for x in xs)
    aa = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else 0.0
    bb = my - aa * mx
    print(f'  실측 적합:  PWM = {aa:.1f}·v + {bb:.1f}')
    print(f'  현재 ROS 쪽 FF: PWM = 38.8·v + 17.2   '
          f'(bringup 의 ff_gain / ff_static, ff_mode:=ros)')
    print()
    print('  → 펌웨어를 굽지 않고 런치 인자로 바로 적용할 수 있다:')
    print(f'       ff_static:={bb:.1f} ff_gain:={aa:.1f}')
    print('     굳히려면 bringup.launch.py 의 기본값과')
    print('     tools/lap_budget.py 의 STATIC_FF/VELOCITY_FF_GAIN 도 맞출 것')
    print('     (랩타임·8분 예산 계산이 이 상수 위에 서 있다)')
  else:
    print('  ⚠ 유효한 단계가 부족하다 — PWM 범위를 넓히거나 거리를 늘릴 것')
  print('=' * 70)
  return 0


if __name__ == '__main__':
  sys.exit(main() or 0)

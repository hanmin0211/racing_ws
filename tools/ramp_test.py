#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ramp_test.py — 경사(오르막/내리막)를 **GPS 없이** 재고 판정한다.

★ 왜 (2026-09-17)
  용인 s11~54 에 경사가 있는데, 내리막에서 속도가 붙으면 두 가지가 걱정이다:
    ① 바닥 속도가 다음 커브(굴절코스 R=6.4m)를 돌 수 있는 범위인가
    ② 그 부하에서 전압이 무너져 링크가 끊기거나 리셋되는가
  둘 다 **웨이포인트도 GPS도 필요 없다.** 엔코더가 살아 있으므로
  /current_speed 로 속도를, /vcc_mv 로 전압을 직접 잰다.

  ⚠ 내리막은 **반드시 능동 제동을 켜고** 할 것. 안 켜면 W 를 놓아도
    PWM 0 = 관성뿐이고, 경사 3.8% 를 넘으면 놓아도 계속 빨라진다.
      ros2 launch tools/teleop_drive.launch.py ff_mode:=ros ff_brake_pwm:=50.0

사용:
  # ★ 자율 — 차가 스스로 지정 거리만큼 가고 선다 (키보드 불필요, GPS 불필요)
  python3 tools/ramp_test.py --label 오르막1 --drive 15 --speed 0.8

  # 수동 — teleop 으로 몰면서 기록만
  python3 tools/ramp_test.py --label 내리막1
  python3 tools/ramp_test.py --label 오르막1 --seconds 60
  python3 tools/ramp_test.py --report /tmp/ramp_내리막1_2130.csv   # 다시 판정만
"""

import argparse
import csv
import math
import os
import sys
import time

# 굴절코스 첫 급커브 R=6.4m 를 돌 수 있는 상한 (횡가속·조향반응 거리로 산출)
SPEED_OK, SPEED_WARN = 3.0, 4.0
# ATmega2560 판정선 (tools/vcc_check.py 와 같다)
VCC_OK, VCC_LOW, VCC_BOD = 4700, 4300, 3483
COAST_DECEL = 0.37          # 평지 관성 감속 실측 0.31~0.43 의 중앙
COUNTS_PER_REV, WHEEL_R = 290.0, 0.1327


def judge(rows, label):
  """기록된 행들로 판정표를 낸다."""
  if len(rows) < 10:
    print(f'❌ 표본 {len(rows)}개뿐 — 기록이 안 됐다')
    return 1
  t = [r['t'] for r in rows]
  v = [r['v'] for r in rows]
  cv = [r['cmd_v'] for r in rows]
  vcc = [r['vcc'] for r in rows if r['vcc'] > 500]
  enc = [r['enc'] for r in rows]
  fails, warns = [], []

  def say(g, item, msg):
    mark = {'실패': '❌', '경고': '⚠ ', '통과': '✅'}[g]
    print(f'  {mark} {item:<14} {msg}')
    (fails if g == '실패' else warns if g == '경고' else []).append(item)

  dur = t[-1] - t[0]
  # 첫 샘플이 초기값(0)에서 점프한 것이면 버린다. 20Hz 에서 한 샘플에
  # 1000카운트(=2.9m)가 변하는 것은 물리적으로 불가능하다.
  e = list(enc)
  while len(e) > 2 and abs(e[1] - e[0]) > 1000:
    e.pop(0)
  dist = abs(e[-1] - e[0]) / COUNTS_PER_REV * (2 * math.pi * WHEEL_R)
  vmax = max(abs(x) for x in v)
  print(f'\n[{label}]  {dur:.1f}s · {len(rows)}샘플 · 이동 {dist:.1f} m')
  print(f'   속도  평균 {sum(abs(x) for x in v) / len(v):.2f} · 최대 {vmax:.2f} m/s')

  # ── 속도 — 다음 커브를 돌 수 있나 ────────────────────────────────
  if vmax > SPEED_WARN:
    say('실패', '최고 속도',
        f'{vmax:.2f} m/s — 굴절코스 R=6.4m 를 못 돈다(횡가속 '
        f'{vmax ** 2 / 6.4:.1f} m/s²). 속도를 낮출 방법이 필요하다')
  elif vmax > SPEED_OK:
    say('경고', '최고 속도',
        f'{vmax:.2f} m/s — R=6.4m 코너가 빠듯하다(상한 {SPEED_OK:.0f})')
  else:
    say('통과', '최고 속도', f'{vmax:.2f} m/s (상한 {SPEED_OK:.0f})')

  # ── 명령 vs 실제 — 개루프 오차 ───────────────────────────────────
  pairs = [(c, a) for c, a in zip(cv, v) if abs(c) > 0.05 and abs(a) > 0.05]
  if pairs:
    ratio = sum(abs(a) / abs(c) for c, a in pairs) / len(pairs)
    g = '통과' if 0.8 <= ratio <= 1.25 else '경고'
    say(g, '명령 대 실제',
        f'실제/명령 = {ratio:.2f} 배 (개루프면 1 에서 멀어진다. '
        f'NO_ENCODER 0 + ff_mode:=firmware 로 닫으면 1 에 가까워져야 한다)')

  # ── PWM — '못 올라감' 의 원인을 가른다 ───────────────────────────
  #   PWM 이 높은데 안 움직이면 토크 부족(또는 잠김).
  #   PWM 이 낮은데 안 움직이면 **명령이 모자란 것** — 설정을 올리면 된다.
  pw = [abs(r.get('pwm', 0)) for r in rows]
  if any(pw):
    moving = [abs(x) > 0.05 for x in v]
    run_pw = sorted(p for p, mv in zip(pw, moving) if mv)
    med = run_pw[len(run_pw) // 2] if run_pw else 0
    stuck_pw = [p for p, c, mv in zip(pw, cv, moving)
                if abs(c) > 0.1 and not mv]
    print(f'   PWM   최대 {max(pw)} · 달릴 때 중앙 {med}')
    if stuck_pw:
      mx = max(stuck_pw)
      if mx >= 100:
        say('실패', '토크 부족',
            f'PWM {mx} 를 주고도 안 움직였다 — 설정으로 더 못 올린다. '
            f'경사가 이 차의 한계를 넘는다')
      else:
        say('경고', '명령 부족',
            f'안 움직일 때 PWM 이 {mx} 뿐이었다 — 아직 여유가 있다. '
            f'ff_min_pwm 을 올려 볼 것 (평지 정지마찰 문턱 55~56)')

  # ── 전압 ─────────────────────────────────────────────────────────
  if vcc:
    lo = min(vcc)
    if lo < VCC_BOD:
      say('실패', '공급전압',
          f'최저 {lo:.0f}mV — 과거 리셋 발생선({VCC_BOD}) 아래다')
    elif lo < VCC_LOW:
      say('경고', '공급전압',
          f'최저 {lo:.0f}mV — 브라운아웃 위험선({VCC_LOW}) 아래. 배터리 확인')
    else:
      say('통과', '공급전압', f'최저 {lo:.0f}mV')

  # ── 엔코더 연속성 — 이게 끊기면 PID 를 못 켠다 ────────────────────
  # 움직이는 중(명령이 있는데)에 카운트가 멈추면 끊긴 것이다.
  stuck = 0
  for i in range(1, len(rows)):
    if abs(cv[i]) > 0.1 and enc[i] == enc[i - 1] and abs(v[i - 1]) > 0.2:
      stuck += 1
  if stuck > len(rows) * 0.05:
    say('실패', '엔코더 연속성',
        f'달리는 중 카운트가 멈춘 샘플 {stuck}개 ({100 * stuck / len(rows):.0f}%) '
        f'— 이 상태로 NO_ENCODER 0 을 켜면 PID 가 오동작한다')
  else:
    say('통과', '엔코더 연속성', f'멈춘 샘플 {stuck}개')

  # ── 링크 ─────────────────────────────────────────────────────────
  gaps = [t[i] - t[i - 1] for i in range(1, len(t)) if t[i] - t[i - 1] > 0.5]
  if gaps:
    say('실패', '링크 끊김',
        f'{len(gaps)}회 · 최장 {max(gaps):.1f}s — 0.5s 넘으면 경로추종이 선다')
  else:
    say('통과', '링크 끊김', '없음 (0.5s 넘는 공백 없음)')

  # ── 경사 역산 (참고) ─────────────────────────────────────────────
  # 구동을 놓은 뒤(명령 0) 속도가 **늘면** 내리막이다. 그 가속에서 경사를 뽑는다.
  coast = [(t[i], v[i]) for i in range(len(rows)) if abs(cv[i]) < 0.05]
  if len(coast) > 10:
    a = (coast[-1][1] - coast[0][1]) / max(coast[-1][0] - coast[0][0], 1e-6)
    if a > 0.05:
      grade = math.tan(math.asin(min((a + COAST_DECEL) / 9.81, 1.0)))
      print(f'   ※ 구동을 놓은 뒤 {a:+.2f} m/s² 로 **가속** — '
            f'경사 약 {grade * 100:.0f}% 로 추정(관성감속 {COAST_DECEL} 가정)')
    else:
      print(f'   ※ 구동을 놓은 뒤 {a:+.2f} m/s² (감속) — 폭주 경사는 아니다')

  print()
  if fails:
    print(f'❌ 실패 {len(fails)}건 · 경고 {len(warns)}건')
    return 1
  if warns:
    print(f'⚠  경고 {len(warns)}건 — 원인을 알고 넘어갈 것')
    return 0
  print('✅ 전부 통과')
  return 0


def record(a):
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist
  from std_msgs.msg import Bool, Float64, Int32

  class R(Node):
    def __init__(self):
      super().__init__('ramp_test')
      self.v = self.cmd_v = 0.0
      self.enc = None   # 첫 메시지 전에는 None
      self.vcc = 0
      self.pwm = 0
      self.steer = 0.0
      self.stall = False
      self.create_subscription(Float64, '/current_speed',
                               lambda m: setattr(self, 'v', m.data), 10)
      self.create_subscription(Int32, '/encoder_count',
                               lambda m: setattr(self, 'enc', m.data), 10)
      self.create_subscription(Int32, '/drive_pwm',
                               lambda m: setattr(self, 'pwm', m.data), 10)
      self.create_subscription(Int32, '/vcc_mv',
                               lambda m: setattr(self, 'vcc', m.data), 10)
      self.create_subscription(Float64, '/steering_angle',
                               lambda m: setattr(self, 'steer', m.data), 10)
      self.create_subscription(Bool, '/vehicle_stall',
                               lambda m: setattr(self, 'stall', m.data), 10)
      self.create_subscription(Twist, '/cmd_vel',
                               lambda m: setattr(self, 'cmd_v', m.linear.x), 10)

  rclpy.init()
  n = R()
  drive = a.drive is not None
  if drive:
    from geometry_msgs.msg import Twist as _T
    # ★ /teleop/cmd_vel 로 낸다. 먹스에서 teleop 이 자율보다 우선이고
    #   E-stop(/e_stop)은 그보다도 위다 — 비상정지 권한을 살려 둔다.
    n.drive_pub = n.create_publisher(_T, '/teleop/cmd_vel', 10)
  out = a.out or f'/tmp/ramp_{a.label}_{time.strftime("%H%M")}.csv'
  rows = []
  # ★ 엔코더 첫 메시지를 기다린다. 안 기다리면 초기값(0)에서 실제 카운트로
  #   점프하는 첫 샘플이 **이동거리로 계산된다** — 정지 상태에서 '8.9m 이동'
  #   이 나왔다(실제로 겪음).
  print('  엔코더 첫 수신 대기…')
  wait_end = time.time() + 10
  while n.enc is None and time.time() < wait_end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
  if n.enc is None:
    print('❌ /encoder_count 가 안 온다 — serial_bridge 가 떠 있는지 확인할 것')
    rclpy.shutdown()
    return 1
  t0 = time.time()
  print(f'▶ {a.seconds}초 기록 · {out}')
  print('  ⚠ 내리막이면 ff_brake_pwm 이 켜져 있는지 확인할 것 '
        '(꺼져 있으면 놓아도 안 선다)')
  print(f'  {"t":>6} {"명령":>6} {"실제":>6} {"PWM":>5} {"vcc":>6} {"이동":>6}')
  last = 0.0
  e0 = n.enc
  stop_reason = '시간 종료'
  stuck_since = None
  limit = a.seconds
  if drive:
    # 예상시간의 3배 + 10초. 스톨·미끄러짐으로 안 끝나는 것을 막는다.
    limit = min(a.seconds, a.drive / max(a.speed, 0.05) * 3.0 + 10.0)
    print(f'  ▶ 자율 {a.drive:.1f}m · 목표 {a.speed:.2f} m/s · '
          f'상한 {limit:.0f}s · 중단속도 {a.abort_speed:.1f} m/s')

  def send(v):
    if drive:
      from geometry_msgs.msg import Twist as _T
      m = _T()
      m.linear.x = float(v)
      m.angular.z = 0.0          # 직진 (조향 중앙)
      n.drive_pub.publish(m)

  try:
    while time.time() - t0 < limit and rclpy.ok():
      rclpy.spin_once(n, timeout_sec=0.05)
      t = time.time() - t0
      dist = abs(n.enc - e0) / COUNTS_PER_REV * (2 * math.pi * WHEEL_R)

      if drive:
        # ── 중단 조건 (먼저 판정하고 그 다음에 명령을 낸다) ──
        if dist >= a.drive:
          stop_reason = f'목표 거리 도달 ({dist:.1f}m)'
          break
        if abs(n.v) > a.abort_speed:
          stop_reason = f'❌ 과속 중단 — {abs(n.v):.2f} > {a.abort_speed:.1f} m/s'
          break
        if 500 < n.vcc < 3200:
          stop_reason = f'❌ 전압 중단 — {n.vcc}mV'
          break
        if n.stall:
          stop_reason = '❌ 펌웨어 스톨 감지'
          break
        # 명령은 있는데 안 움직이면 2초 뒤 중단 (경사에서 못 올라가는 경우)
        if t > 2.0 and abs(n.v) < 0.05:
          stuck_since = stuck_since if stuck_since is not None else t
          if t - stuck_since > 2.0:
            stop_reason = f'❌ 안 움직인다 ({t - stuck_since:.1f}s) — 못 올라감'
            break
        else:
          stuck_since = None
        # 출발 램프 — 1초에 걸쳐 올린다(덜컹 방지)
        send(a.speed * min(1.0, t / 1.0))

      rows.append({'t': t, 'v': n.v, 'cmd_v': n.cmd_v, 'enc': n.enc,
                   'vcc': n.vcc, 'pwm': n.pwm, 'steer': n.steer,
                   'stall': int(n.stall)})
      if t - last >= 0.5:
        last = t
        flag = ''
        if abs(n.v) > SPEED_WARN:
          flag = '  ❌ 너무 빠르다'
        elif abs(n.v) > SPEED_OK:
          flag = '  ⚠ 빠르다'
        if 500 < n.vcc < VCC_BOD:
          flag += '  ❌ 전압'
        print(f'  {t:6.1f} {n.cmd_v:6.2f} {n.v:6.2f} {n.pwm:5d} '
              f'{n.vcc:6.0f} {dist:6.1f}{flag}')
  except KeyboardInterrupt:
    stop_reason = '사용자 중단 (Ctrl-C)'
  finally:
    if drive:
      # 정지 명령을 여러 번 낸다. 한 번은 놓칠 수 있고, 펌웨어 워치독이
      # 0.5초 안에 받쳐 주지만 그 전에 확실히 끊는다.
      for _ in range(10):
        send(0.0)
        rclpy.spin_once(n, timeout_sec=0.02)
  print(f'\n  ■ 종료: {stop_reason}')
  if drive:
    print('  ⚠ 경사에서는 정지 후 **뒤로 밀린다** (엔코더 홀드 없음). '
          '차를 잡거나 굄목을 댈 것')
  rclpy.shutdown()

  with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
  print(f'\n저장: {out}')
  return judge(rows, a.label)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--label', default='ramp')
  ap.add_argument('--seconds', type=float, default=40.0)
  ap.add_argument('--out', default=None)
  ap.add_argument('--report', default=None, help='기록된 CSV 를 다시 판정만 한다')
  ap.add_argument('--drive', type=float, default=None, metavar='m',
                  help='자율 주행: 이 거리만큼 스스로 가고 선다 [m]. '
                       '거리는 **엔코더로** 잰다 (GPS 불필요)')
  ap.add_argument('--speed', type=float, default=0.8, metavar='m/s',
                  help='--drive 목표 속도')
  ap.add_argument('--abort-speed', type=float, default=2.5, metavar='m/s',
                  help='이 속도를 넘으면 즉시 중단 (내리막 폭주 방어)')
  a = ap.parse_args()
  if a.report:
    if not os.path.exists(a.report):
      sys.exit(f'없는 파일: {a.report}')
    rows = [{k: (int(v) if k in ('enc', 'vcc', 'stall') else float(v))
             for k, v in r.items()}
            for r in csv.DictReader(open(a.report))]
    return judge(rows, os.path.basename(a.report))
  return record(a)


if __name__ == '__main__':
  raise SystemExit(main())

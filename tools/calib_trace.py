#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib_trace.py — 헤딩 캘리브 주행을 **명령부터 바퀴까지** 통째로 기록한다.

★ 왜 (2026-09-12 현장)
  캘리브가 8회 연속 실패했다. auto_calib_speed 를 0.30 → 0.20 → 0.15 로 낮춰도
  주행 타임라인이 **완전히 같았다**(t+6.7s 1.1m → t+7.4s 2.0m → t+8.1s 3.1m).
  명령 속도가 모터까지 안 가고 있다는 뜻인데, 어느 단계에서 끊기는지 모른다.
  체인 전체를 한 줄에 찍어 그 지점을 특정한다:

      heading_init → /teleop/cmd_vel → [먹스] → /cmd_vel → [serial_bridge]
        → VEL: → [펌웨어 FF] → /drive_pwm → 바퀴 → /fix(GPS 실제 이동)

사용:
  python3 tools/calib_trace.py            # Ctrl-C 까지 기록, 요약 + CSV
  (bringup 을 auto_calib:=true 로 띄운 **직후** 실행할 것)
"""

import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Int32

WS = '/home/han/racing_ws'
M_PER_DEG = 111320.0


def main():
  rclpy.init()
  n = Node('calib_trace')
  st = {'tele_v': float('nan'), 'cmd_v': float('nan'), 'pwm': float('nan'),
        'odom_v': float('nan'), 'lat': None, 'lon': None, 'sig': float('nan')}
  rows = []
  t0 = time.time()
  origin = {}

  n.create_subscription(Twist, '/teleop/cmd_vel',
                        lambda m: st.__setitem__('tele_v', m.linear.x), 10)
  n.create_subscription(Twist, '/cmd_vel',
                        lambda m: st.__setitem__('cmd_v', m.linear.x), 10)
  n.create_subscription(Int32, '/drive_pwm',
                        lambda m: st.__setitem__('pwm', float(m.data)), 10)
  n.create_subscription(Odometry, '/odometry/filtered',
                        lambda m: st.__setitem__('odom_v',
                                                 m.twist.twist.linear.x), 10)

  def fix_cb(m):
    st['lat'], st['lon'] = m.latitude, m.longitude
    st['sig'] = math.sqrt(m.position_covariance[0])
  n.create_subscription(NavSatFix, '/fix', fix_cb, qos_profile_sensor_data)

  def tick():
    if st['lat'] is None:
      return
    if not origin:
      origin['lat'], origin['lon'] = st['lat'], st['lon']
    d = math.hypot((st['lon'] - origin['lon']) * M_PER_DEG
                   * math.cos(math.radians(origin['lat'])),
                   (st['lat'] - origin['lat']) * M_PER_DEG)
    r = dict(t=round(time.time() - t0, 3), dist=round(d, 3),
             tele_v=round(st['tele_v'], 3), cmd_v=round(st['cmd_v'], 3),
             pwm=st['pwm'], odom_v=round(st['odom_v'], 3),
             sig=round(st['sig'], 4))
    rows.append(r)
    if len(rows) % 10 == 0:
      # GPS 로 잰 순간속도
      k = max(0, len(rows) - 11)
      dt = r['t'] - rows[k]['t']
      gv = (r['dist'] - rows[k]['dist']) / dt if dt > 0 else 0.0
      print(f"  t+{r['t']:6.2f}s  이동 {r['dist']:6.2f}m  GPS속도 {gv:5.2f}m/s │ "
            f"teleop {r['tele_v']:5.2f}  cmd_vel {r['cmd_v']:5.2f}  "
            f"PWM {r['pwm']:5.0f}  odom_v {r['odom_v']:5.2f}  σ {r['sig']:.3f}m")
  n.create_timer(0.1, tick)

  print('기록 시작 — Ctrl-C 로 종료하고 요약합니다.')
  print('  (bringup 을 auto_calib:=true 로 띄운 직후 실행할 것)')
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      n.destroy_node()
    except Exception:  # noqa: BLE001
      pass
    if rclpy.ok():
      rclpy.shutdown()

  if not rows:
    print('\n기록 없음 — 스택이 안 돌거나 토픽이 안 왔다.')
    return 1
  os.makedirs(f'{WS}/logs', exist_ok=True)
  out = f'{WS}/logs/calib_trace_{time.strftime("%Y%m%d_%H%M%S")}.csv'
  with open(out, 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

  print('\n' + '=' * 72)
  print('캘리브 주행 요약 — 명령이 어디서 끊기는가')
  print('=' * 72)
  mv = [r for r in rows if r['dist'] > 0.15]
  if not mv:
    print('  차가 움직이지 않았다.')
  else:
    a, b = mv[0], mv[-1]
    dt = b['t'] - a['t']
    print(f'  이동 {b["dist"] - a["dist"]:.2f}m / {dt:.1f}s  '
          f'→ 평균 GPS속도 {(b["dist"] - a["dist"]) / max(dt, 1e-3):.2f} m/s')
    for k, lab in (('tele_v', 'teleop 명령'), ('cmd_v', 'cmd_vel(먹스 출력)'),
                   ('pwm', '구동 PWM'), ('odom_v', 'odom 속도')):
      v = [r[k] for r in mv if r[k] == r[k]]
      if v:
        print(f'  {lab:20} 최소 {min(v):7.2f}  최대 {max(v):7.2f}  '
              f'평균 {sum(v) / len(v):7.2f}')
      else:
        print(f'  {lab:20} ❌ 수신 없음')
  print('-' * 72)
  print('  읽는 법:')
  print('   · teleop 명령이 설정값(auto_calib_speed)과 다르면 → 캘리브 노드 문제')
  print('   · cmd_vel 이 teleop 과 다르면 → 먹스가 덮어썼다')
  print('   · PWM 이 명령과 안 맞으면 → serial_bridge/펌웨어 FF 문제')
  print('   · PWM 은 맞는데 GPS속도가 훨씬 크면 → **FF 상수가 실제와 안 맞는다**')
  print(f'\nCSV: {out}')
  print('=' * 72)
  return 0


if __name__ == '__main__':
  sys.exit(main() or 0)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vcc_watch.py — 공급전압(VCC)을 **bringup 을 끄지 않고** 실시간으로 본다.

왜 vcc_check.py 와 따로 있나
  vcc_check.py 는 시리얼 포트를 직접 연다. 그래서 bringup(serial_bridge)이
  돌고 있으면 **포트를 독점할 수 없어 못 쓴다.** 그런데 전원 경로를 만지는
  일은 보통 스택을 띄워 둔 채로 한다(커넥터를 흔들면서 전압이 어떻게
  변하는지 봐야 하므로). 이 도구는 ROS 토픽 /vcc_mv · /vcc_min_mv 를 읽어
  같은 판정을 내므로 **bringup 과 동시에** 쓸 수 있다.

판정선은 vcc_check.py 와 같다 (ATmega2560 @16MHz):
  ≥ 4700mV  정상
  4300~4700 낮음 (ADC 부정확·마진 부족)
  < 4300    브라운아웃 위험 (BOD 트립 영역 — 언제든 리셋)

참고로 이 차의 실측 이력(henes_firmware.ino 주석):
  9/3 접지 수리 후 무부하 5091~5115 · 직선주행 4262 · 커브+조향 3215
  과거 리셋 발생선 3483 · GND 접점 불량일 때의 VMIN 3920

사용:
  python3 tools/vcc_watch.py            # 커넥터를 흔들면서 본다
  python3 tools/vcc_watch.py --csv /tmp/vcc.csv
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


def verdict(mv):
  if mv is None:
    return '—', ''
  if mv >= 4700:
    return '정상', ''
  if mv >= 4300:
    return '낮음', '  마진 부족'
  if mv >= 3900:
    return '위험', '  ⚠ BOD 트립 영역 — 주행하면 리셋된다'
  return '치명', '  ❌ 접점/배터리 이상 — 주행 금지'


def bar(mv, lo=3200, hi=5200, w=34):
  if mv is None:
    return ' ' * w
  f = max(0.0, min(1.0, (mv - lo) / (hi - lo)))
  n = int(f * w)
  return '█' * n + '·' * (w - n)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--csv', default=None, help='기록 파일(선택)')
  a = ap.parse_args()

  rclpy.init()
  node = Node('vcc_watch')
  st = {'vcc': None, 'vmin': None, 'n': 0}
  node.create_subscription(Int32, '/vcc_mv',
                           lambda m: st.update(vcc=m.data, n=st['n'] + 1), 10)
  node.create_subscription(Int32, '/vcc_min_mv',
                           lambda m: st.update(vmin=m.data), 10)

  f = open(a.csv, 'w') if a.csv else None
  if f:
    f.write('t,vcc_mv,vcc_min_mv\n')

  print('VCC 감시 — Ctrl-C 로 종료. 커넥터/배터리를 만지면서 숫자를 볼 것.')
  print(f"{'t':>5} {'VCC':>6} {'세션최저':>8}  {'판정':<4}")
  t0 = time.time()
  lo = 99999
  last = -1.0
  try:
    while rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.1)
      t = time.time() - t0
      if st['vcc'] is not None and st['vcc'] < lo:
        lo = st['vcc']
      if t - last < 0.5:
        continue
      last = t
      v = st['vcc']
      tag, note = verdict(v)
      shown_lo = lo if lo != 99999 else None
      print(f'{t:5.0f} {v if v is not None else "-":>6} '
            f'{shown_lo if shown_lo is not None else "-":>8}  {tag:<4} '
            f'|{bar(v)}|{note}')
      if f and v is not None:
        f.write(f'{t:.2f},{v},{st["vmin"] if st["vmin"] is not None else ""}\n')
        f.flush()
  except KeyboardInterrupt:
    pass
  finally:
    if st['n'] == 0:
      print('\n❌ /vcc_mv 를 한 건도 못 받았다 — bringup(serial_bridge)이 '
            '떠 있는지, 아두이노가 붙어 있는지 확인할 것.')
    else:
      tag, note = verdict(lo if lo != 99999 else None)
      print(f'\n관측 최저 {lo} mV → {tag}{note}')
    if f:
      f.close()
    rclpy.shutdown()
  return 0


if __name__ == '__main__':
  sys.exit(main())

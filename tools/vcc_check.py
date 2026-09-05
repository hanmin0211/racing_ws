#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vcc_check.py — 아두이노 5V 로직 전원(VCC) 감시. 전원 경로 진단용.

STATUS_10ms 텔레메트리의 VCC= 필드를 읽어 1초마다 현재값 + 구간 min/max + 판정.
전원 구성을 바꿔가며(USB 직결/허브, 배터리 연결/분리) VCC 변화를 비교한다.

★ Arduino IDE Serial Monitor 를 닫고 실행 (포트 하나만 열림).

  python3 tools/vcc_check.py                # /dev/arduino 자동
  python3 tools/vcc_check.py /dev/ttyACM0
  Ctrl-C 정지 (전체 요약 출력).

판정 기준 (ATmega2560 @16MHz):
  ≥ 4700mV  정상
  4300~4700 낮음 (ADC 부정확·마진 부족)
  < 4300    브라운아웃 위험 (BOD 트립 영역 — 언제든 리셋 가능)
"""

import glob
import re
import sys
import time

try:
  import serial
except ImportError:
  print('pyserial 필요: pip install pyserial', file=sys.stderr)
  sys.exit(1)

BAUD = 57600
VCC = re.compile(r'VCC=(\d+)')


def verdict(mv):
  if mv >= 4700:
    return '정상 ✅'
  if mv >= 4300:
    return '낮음 ⚠'
  return '브라운아웃 위험 ❌'


def pick_port(argv):
  if len(argv) > 1:
    return argv[1]
  c = glob.glob('/dev/arduino') + sorted(glob.glob('/dev/ttyACM*'))
  return c[0] if c else '/dev/ttyACM0'


def main():
  port = pick_port(sys.argv)
  print(f'포트 {port} @ {BAUD} — VCC 감시. Ctrl-C 정지.\n')
  ser = serial.Serial(port, BAUD, timeout=1)
  win = []
  allv = []
  t_last = time.time()
  while True:
    raw = ser.readline().decode('ascii', 'ignore')
    m = VCC.search(raw)
    if m:
      v = int(m.group(1))
      if v > 0:
        win.append(v)
        allv.append(v)
    now = time.time()
    if now - t_last >= 1.0 and win:
      cur = win[-1]
      lo, hi = min(win), max(win)
      print(f'VCC {cur:5d} mV   (min {lo}  max {hi})   {verdict(cur)}')
      win = []
      t_last = now


if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    pass
  except serial.SerialException as e:
    print(f'\n시리얼 열기 실패: {e} → Serial Monitor 닫았는지 확인.')

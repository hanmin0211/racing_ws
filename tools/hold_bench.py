#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hold_bench.py — 경사로 홀드 벤치 확인용 느린 시리얼 뷰어.

아두이노 텔레메트리(STATUS_10ms)는 20Hz라 시리얼 모니터로는 너무 빠르다.
이 뷰어는 ENC1 이 변할 때만(또는 idle 시 0.7s마다) 한 줄씩 느리게 찍는다.

★ Arduino IDE 의 Serial Monitor 를 먼저 닫을 것 (포트는 하나만 열 수 있다).

사용:
  python3 tools/hold_bench.py                 # /dev/arduino 자동
  python3 tools/hold_bench.py /dev/ttyACM0    # 포트 지정
  Ctrl-C 로 정지.

보는 법 (부호/거동 확인):
  · 앞바퀴(엔코더 달린 쪽)를 손으로 굴리면 ENC1 이 변한다.
  · 모터 전원 ON 상태에서 바퀴를 밀었다 놓았을 때:
      - ENC1 이 원래값(latch)으로 되돌아오면  → ✅ 홀드 정상(되민다)
      - ENC1 이 점점 커지며 달아나면(runaway)  → ❌ 즉시 E-stop, 부호 반대
  · PWM 부호는 항상 변위를 따라가므로(설계상), 진짜 판정은
    '놓았을 때 ENC1 이 돌아오나 달아나나' 다.
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
RX = re.compile(r'ENC1=(-?\d+).*?VEL=(-?[\d.]+).*?PWM=(-?\d+)')
VCC = re.compile(r'VCC=(\d+)')


def pick_port(argv):
  if len(argv) > 1:
    return argv[1]
  cands = glob.glob('/dev/arduino') + sorted(glob.glob('/dev/ttyACM*'))
  return cands[0] if cands else '/dev/ttyACM0'


def main():
  port = pick_port(sys.argv)
  print(f'포트 {port} @ {BAUD}  —  앞바퀴 굴리며 ENC1 관찰. Ctrl-C 정지.')
  print('놓았을 때 ENC1 이 원래값으로 돌아오면 ✅, 달아나면 ❌(즉시 E-stop)\n')
  print(f'{"시각":>6}  {"ENC1":>8} {"Δ":>6}  {"drivePWM":>8}  {"VCC":>6}')
  print('-' * 44)

  ser = serial.Serial(port, BAUD, timeout=1)
  last_enc = None
  last_print = 0.0
  vcc = 0
  t0 = time.time()
  while True:
    raw = ser.readline().decode('ascii', 'ignore').strip()
    mv = VCC.search(raw)
    if mv:
      vcc = int(mv.group(1))
    m = RX.search(raw)
    if not m:
      continue
    enc = int(m.group(1))
    pwm = int(m.group(3))
    now = time.time()
    moved = (last_enc is None) or (abs(enc - last_enc) >= 3)
    if moved or (now - last_print) > 0.7:
      d = 0 if last_enc is None else (enc - last_enc)
      flag = ''
      if abs(enc) > 400:
        flag = '  ⚠ 변위 큼 — 달아나면 E-stop'
      print(f'{now - t0:6.1f}  {enc:8d} {d:+6d}  {pwm:+8d}  {vcc:6d}{flag}')
      last_enc = enc
      last_print = now


if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    print('\n정지.')
  except serial.SerialException as e:
    print(f'\n시리얼 열기 실패: {e}\n → Arduino IDE Serial Monitor 를 닫았는지 확인.')

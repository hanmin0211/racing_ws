#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""adc_find_sensor.py — adc_scan.ino 출력을 읽어 어느 아날로그 핀이 조향 센서인지 찾는다.
바퀴를 좌우 끝까지 몇 번 돌리면, '변동폭(max-min)'이 압도적으로 큰 핀이 센서다.
사용: python3 tools/adc_find_sensor.py   (adc_scan.ino 플래시된 상태에서)
"""
import re, sys, serial, time
PORT = '/dev/arduino'
mn = [9999]*16; mx = [0]*16
try:
    ser = serial.Serial(PORT, 57600, timeout=1)
except Exception as e:
    print(f'포트 열기 실패: {e}'); sys.exit(1)
time.sleep(0.5)
print('바퀴를 왼쪽 끝 ↔ 오른쪽 끝으로 3~4번 천천히 돌려. (Ctrl-C 로 결과)')
pat = re.compile(r'A(\d+)=(\d+)')
try:
    while True:
        line = ser.readline().decode('utf-8', 'ignore')
        for m in pat.finditer(line):
            i, v = int(m.group(1)), int(m.group(2))
            if 0 <= i < 16:
                mn[i] = min(mn[i], v); mx[i] = max(mx[i], v)
        # 현재까지 변동폭 상위 표시
        ranges = sorted(range(16), key=lambda i: mx[i]-mn[i], reverse=True)
        top = ranges[0]
        sys.stdout.write(f'\r최대변동 핀: A{top} (범위 {mn[top]}~{mx[top]}, 폭 {mx[top]-mn[top]})   ')
        sys.stdout.flush()
except KeyboardInterrupt:
    print('\n\n=== 핀별 변동폭 (큰 순) ===')
    for i in sorted(range(16), key=lambda i: mx[i]-mn[i], reverse=True):
        bar = '█' * min(40, (mx[i]-mn[i])//20)
        print(f'  A{i:<2}: {mn[i]:>4}~{mx[i]:<4} 폭{mx[i]-mn[i]:>4}  {bar}')
    print('\n→ 폭이 압도적으로 큰 핀이 조향 센서다 (예: 폭 700+).')
    print('  전부 폭이 작으면(<100) 센서 신호가 아두이노에 안 옴 = 배선 끊김.')
    ser.close()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stability_check.py — 장치·노드·토픽 안정성을 한 번에 오래 지켜본다.

★ 왜 필요한가
  이 프로젝트에서 하루를 날린 사고는 대부분 "끊김" 이었다.
    · IMU 가 허브 뒤에서 8~24초마다 끊김 (2026-08-25)
    · IMU 미연결 상태로 주행하다 연석 충돌 (2026-08-24)
    · GPS 가 8회 재열거 뒤 빠져 캘리브·주차자세 전부 못 함 (2026-09-05)
  셋 다 **주행 전에 몇 분만 지켜봤으면** 잡혔을 것들이다. 그런데 그때마다
  임시로 for 루프를 짜서 봤고, 기록이 안 남아 다음에 또 당했다.

  여기서 장치(USB)·노드(ROS)·토픽(실제 데이터)을 동시에 보고,
  **끊긴 시각과 횟수를 숫자로 남긴다.**

★ 무엇을 보나
  장치   lsusb 로 VID:PID 존재 + /dev 심볼릭
  노드   ros2 node list 에 있는가
  토픽   실제로 메시지가 오는가 (발행자 수가 아니라 수신 여부)
         ※ topic info 의 Publisher count 는 연결만 보여줄 뿐 데이터 흐름을
           보장하지 않는다. 반드시 실제 수신으로 확인한다.
  파이   ping

사용:
  python3 tools/stability_check.py                    # 기본 5분
  python3 tools/stability_check.py --minutes 30       # 주행 전 장시간
  python3 tools/stability_check.py --no-ros           # 장치만 (스택 없이)
"""

import argparse
import os
import subprocess
import sys
import time

# (표시이름, VID:PID, 심볼릭경로 or None)
DEVICES = [
    ('GPS',   '1546:01a9', None),
    ('IMU',   '10c4:ea60', '/dev/imu'),
    ('아두이노', '2341:0042', '/dev/arduino'),
    ('라이다',  None,        '/dev/ldlidar'),
]

# (표시이름, 토픽, 필수인가)
TOPICS = [
    ('fix',    '/fix', True),
    ('imu',    '/handsfree/imu', True),
    ('scan',   '/scan', False),
    ('odom',   '/odometry/filtered', False),
]

NODES = ['ublox_dgnss', 'imu', 'direct_localization', 'serial_bridge']


def sh(cmd, timeout=4):
  try:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout).stdout
  except subprocess.TimeoutExpired:
    return ''


def check_devices():
  lsusb = sh('lsusb')
  out = {}
  for name, vidpid, link in DEVICES:
    ok = True
    if vidpid:
      ok = vidpid in lsusb
    if link:
      ok = ok and os.path.exists(link)
    if vidpid is None and link:
      ok = os.path.exists(link)
    out[name] = ok
  return out


def check_nodes():
  txt = sh('ros2 node list', timeout=6)
  return {n: (n in txt) for n in NODES}


def check_topic(topic):
  """실제 수신 여부. 발행자 수가 아니라 메시지가 오는지를 본다."""
  r = subprocess.run(f'ros2 topic echo {topic} --once', shell=True,
                     capture_output=True, timeout=None if False else 5,
                     text=True) if False else None
  try:
    r = subprocess.run(['bash', '-lc',
                        f'timeout 3 ros2 topic echo {topic} --once'],
                       capture_output=True, text=True, timeout=6)
    return bool(r.stdout.strip())
  except Exception:  # noqa: BLE001
    return False


def ping(host):
  return subprocess.run(['ping', '-c1', '-W1', host],
                        capture_output=True).returncode == 0


def main():
  ap = argparse.ArgumentParser(description='장치·노드·토픽 안정성 감시')
  ap.add_argument('--minutes', type=float, default=5.0)
  ap.add_argument('--interval', type=float, default=3.0)
  ap.add_argument('--no-ros', action='store_true', help='장치만 본다')
  ap.add_argument('--pi', default='192.168.99.8', help='빈 문자열이면 생략')
  ap.add_argument('--topics', action='store_true',
                  help='토픽 실수신까지 확인(느리다. 간격 5s 이상 권장)')
  args = ap.parse_args()

  end = time.time() + args.minutes * 60
  n = 0
  fails = {}          # 이름 -> 끊긴 횟수
  first_fail = {}     # 이름 -> 처음 끊긴 시각(경과초)
  t0 = time.time()

  print(f'안정성 감시 {args.minutes:.0f}분 · {args.interval:.0f}초 간격')
  print('한 번이라도 ❌ 가 나오면 그 구성으로 주행하면 안 된다.\n')

  hdr = ['경과'] + [d[0] for d in DEVICES]
  if not args.no_ros:
    hdr += ['노드']
    if args.topics:
      hdr += [t[0] for t in TOPICS]
  if args.pi:
    hdr += ['파이']
  print('  ' + ' '.join(f'{h:>8}' for h in hdr))
  print('  ' + '-' * (9 * len(hdr)))

  while time.time() < end:
    n += 1
    el = time.time() - t0
    row = [f'{el:6.0f}s']
    state = {}

    for name, ok in check_devices().items():
      state[name] = ok
      row.append('      ✅' if ok else '      ❌')

    if not args.no_ros:
      nd = check_nodes()
      alive = sum(1 for v in nd.values() if v)
      state['노드'] = alive > 0
      row.append(f'{alive}/{len(NODES)}'.rjust(8))
      if args.topics:
        for tname, topic, _req in TOPICS:
          ok = check_topic(topic)
          state[tname] = ok
          row.append('      ✅' if ok else '      ❌')

    if args.pi:
      ok = ping(args.pi)
      state['파이'] = ok
      row.append('      ✅' if ok else '      ❌')

    for k, v in state.items():
      if not v:
        fails[k] = fails.get(k, 0) + 1
        first_fail.setdefault(k, el)

    print('  ' + ' '.join(row), flush=True)
    time.sleep(max(0.0, args.interval))

  print(f'\n{"=" * 70}')
  print(f'  {n}회 확인 · {(time.time()-t0)/60:.1f}분')
  print('=' * 70)
  if not fails:
    print('  ✅ 전 항목 이상 없음 — 이 구성으로 주행 가능')
    sys.exit(0)
  print('  ❌ 끊김 발생')
  for k, c in sorted(fails.items(), key=lambda x: -x[1]):
    print(f'     {k:<10} {c}/{n}회 실패 ({c/n*100:.0f}%)  '
          f'첫 실패 {first_fail[k]:.0f}초')
  print('\n  한 번이라도 끊기면 주행 중에도 끊긴다. 원인을 잡기 전엔 주행 금지.')
  print('  케이블·커넥터·허브 포트를 바꿔가며 이 시험을 반복할 것.')
  sys.exit(1)


if __name__ == '__main__':
  main()

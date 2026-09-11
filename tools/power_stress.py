#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""power_stress.py — 구동 PWM 을 단계적으로 올리며 **전원계가 어디서 무너지는지** 잰다.

★ 왜 이 측정이 대회 성패를 가르나
  8분 예산을 맞추려면 MAX_DRIVE_PWM 을 올려야 한다(도달속도 = (PWM−80)/95).
  그런데 전원이 약하면 PWM 을 올리는 순간 보드가 브라운아웃으로 리셋된다.
  리셋 = 제어 상실 = 1분 이상 정지 = **탈락**. 감점 몇 점과 탈락을 맞바꾸는
  거래라, "우리 전원이 견디는 PWM" 을 숫자로 알아야 결정할 수 있다.
  (감점 쪽 계산은 tools/lap_budget.py 가 한다)

★ 2026-09-11 이 도구가 만들어진 이유
  v_max 1.84 로 HIL 랩을 돌렸더니 구동 PWM 이 상한 160 에 붙은 채로
  **USB 시리얼이 37회 끊겼다**(주행 시작 48초 뒤부터). 어제 v_max 0.5
  (PWM≈127)로 28분 완주했을 땐 없던 현상이다. 즉 이미 160 에서 한계다.
  ⚠ 그런데 이건 **바퀴를 든 무부하**다. 지상에서는 더 나쁘다.

★★ 전제: **차량 배터리가 연결되어 있어야 한다.**
  배터리 없이 USB 전원만으로 돌리면 이 시험은 **아무 의미가 없다.**
  모터 드라이버가 전류를 당기는 순간 USB 5V 레일이 주저앉아 보드가 리셋되고,
  결과는 "PWM 60 부터 끊긴다" 처럼 나온다 — 그건 차량 전원계의 한계가 아니라
  USB 의 한계다. (2026-09-11 에 실제로 이렇게 잘못 측정했다)
  구분법: 배터리가 없으면 아주 낮은 PWM(60 안팎)부터 바로 끊기고, 끊긴 뒤
  텔레메트리가 아예 안 돌아온다. 그런 결과가 나오면 먼저 배터리를 확인할 것.

★ ROS 를 거치지 않는다
  serial_bridge 가 떠 있으면 포트를 두 개가 열어 서로 방해한다.
  먼저 `bash tools/ros_cleanup.sh` 로 스택을 내리고 실행할 것.

측정 항목 (PWM 단계마다)
  · VCC 최저/중앙값 — ATmega 내부 밴드갭 자가측정.
    ⚠ PWM 노이즈에 오염될 수 있다(2026-09-09: 무부하 4121mV 보다 높은
      5115mV 가 관측된 적 있다 — 물리적으로 불가능). **멀티미터와 대조할 것.**
  · 링크 끊김 횟수 — 이건 오염되지 않는 객관적 신호다. 보드가 리셋되면
    USB CDC 가 끊긴다.
  · 보드 리부팅 횟수 — 부팅 배너("HENES firmware ready")를 세어 직접 잡는다.

사용:
  bash tools/ros_cleanup.sh
  python3 tools/power_stress.py                      # 기본 단계
  python3 tools/power_stress.py --pwms 60 100 140 160 180 200 --hold 8
  python3 tools/power_stress.py --steer              # 조향도 같이 흔든다(최악조건)

⚠ **바퀴를 들어 공중에 띄운 상태에서만** 실행할 것. 지상이면 차가 달린다.
"""

import argparse
import re
import statistics
import subprocess
import sys
import time

try:
  import serial
except ImportError:
  print('pyserial 필요: pip install pyserial', file=sys.stderr)
  sys.exit(1)

BAUD = 57600
VCC_RE = re.compile(r'VCC=(\d+)')
BANNER = 'HENES firmware ready'


# Arduino Mega 2560 의 USB VID:PID. 이걸로만 찾는다.
ARDUINO_VID, ARDUINO_PID = 0x2341, 0x0042


def resolve_port(want, tries=6):
  """아두이노 포트를 **신원을 확인해서** 찾는다.

  ★ 절대 '/dev/ttyACM* 중 아무거나' 로 폴백하지 않는다 (2026-09-11 사고).
    재연결 중 /dev/arduino 심볼릭 링크가 잠깐 사라진 사이 ttyACM0 에 붙었는데,
    그건 **u-blox RTK GPS 수신기**였다. 모터 명령 문자열을 GPS 에 40초간 썼다.
    (UBX 동기바이트가 없는 ASCII 라 수신기가 버려서 실해는 없었지만,
     구조적으로 위험하고 그 뒤 텔레메트리가 통째로 비어 측정도 무효가 됐다)
    아두이노를 못 찾으면 **차라리 실패하는 게 맞다.**
  """
  if want != 'auto':
    return want
  import os
  from serial.tools import list_ports
  for i in range(tries):
    # /dev/arduino 가 있어도 그게 정말 아두이노인지 VID/PID 로 확인한다.
    for p in list_ports.comports():
      if p.vid == ARDUINO_VID and p.pid == ARDUINO_PID:
        return p.device
    if os.path.exists('/dev/arduino'):
      real = os.path.realpath('/dev/arduino')
      for p in list_ports.comports():
        if p.device == real and p.vid == ARDUINO_VID:
          return real
    if i < tries - 1:
      time.sleep(1.0)   # 재열거 직후엔 목록에 아직 안 뜬다
  return None


def open_port(port):
  """DTR 리셋 없이 연다 (serial_bridge 와 같은 방식).

  포트를 그냥 열면 DTR 토글로 아두이노가 리셋된다. stty 로 HUPCL 을 먼저
  풀어야 '여는 것 자체가 리셋' 을 막을 수 있다.
  """
  try:
    subprocess.run(['stty', '-F', port, '-hupcl'], check=False,
                   capture_output=True)
  except Exception:  # noqa: BLE001
    pass
  s = serial.Serial()
  s.port = port
  s.baudrate = BAUD
  s.timeout = 0.1
  s.exclusive = False
  s.dsrdtr = False
  s.rtscts = False
  s.open()
  try:
    s.dtr = True
    s.rts = True
  except Exception:  # noqa: BLE001
    pass
  return s


def step(ser, port, pwm, hold, steer_amp, log):
  """한 단계 유지하며 VCC/끊김/리부팅을 센다. 끊기면 다시 열어 계속한다."""
  vccs, drops, reboots = [], 0, 0
  t_end = time.time() + hold
  t_next_cmd = 0.0
  phase = 0
  while time.time() < t_end:
    now = time.time()
    if now >= t_next_cmd:
      t_next_cmd = now + 0.05                       # 20Hz — 워치독(500ms) 유지
      cmd = f'PWM:{pwm}'
      if steer_amp:
        phase += 1
        ang = steer_amp if (phase // 20) % 2 == 0 else -steer_amp
        cmd += f',STEER:{ang}'
      try:
        ser.write((cmd + '\n').encode())
      except Exception as e:  # noqa: BLE001
        drops += 1
        log.append(f'    전송실패: {e}')
        ser = reopen(ser, port, log)
        continue
    try:
      line = ser.readline().decode('utf-8', errors='ignore').strip()
    except Exception as e:  # noqa: BLE001
      drops += 1
      log.append(f'    수신실패: {e}')
      ser = reopen(ser, port, log)
      continue
    if not line:
      continue
    if BANNER in line:
      reboots += 1
      log.append(f'    ★ 보드 리부팅 감지: {line[:60]}')
    m = VCC_RE.search(line)
    if m:
      vccs.append(int(m.group(1)))
  return ser, vccs, drops, reboots


def reopen(old, port, log):
  try:
    old.close()
  except Exception:  # noqa: BLE001
    pass
  time.sleep(0.5)
  for _ in range(6):
    p = resolve_port('auto')          # 신원 확인된 아두이노만 돌려준다
    if p is None:
      time.sleep(0.5)
      continue
    try:
      s = open_port(p)
      # ★ 재연결 직후 곧바로 읽으면 아무것도 안 온다.
      #   보드가 방금 리셋됐다면 부팅 + 조향 소프트스타트(STEER_SOFT_MS 2.5s)가
      #   끝나야 STATUS 를 제대로 뱉는다. 배너를 기다려 동기를 맞춘다.
      #   (이걸 안 해서 '끊긴 뒤 텔레메트리 0' 이 보드 탓인지 도구 탓인지
      #    구분이 안 됐다 — 2026-09-11)
      s.reset_input_buffer()
      t_wait = time.time() + 5.0
      saw = False
      while time.time() < t_wait:
        try:
          ln = s.readline().decode('utf-8', errors='ignore').strip()
        except Exception:  # noqa: BLE001
          break
        if ln:
          saw = True
          if BANNER in ln:
            log.append('    재연결 후 부팅 배너 확인 — 보드가 리셋됐다')
            break
      log.append(f'    재연결 성공: {p}'
                 + ('' if saw else ' (그러나 5초간 수신 0바이트)'))
      return s
    except Exception as e:  # noqa: BLE001
      log.append(f'    재연결 실패({e})')
      time.sleep(0.5)
  raise RuntimeError(
      '재연결 실패 — VID:PID 2341:0042 인 아두이노를 못 찾았다.\n'
      '  다른 ttyACM 에 붙지 않고 여기서 멈춘다(ttyACM0 은 u-blox GPS 다).\n'
      '  USB 케이블/허브와 보드 전원을 확인할 것.')


def verdict_vcc(mv):
  if mv >= 4700:
    return '정상'
  if mv >= 4300:
    return '낮음'
  return '브라운아웃 위험'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', default='auto')
  ap.add_argument('--pwms', type=int, nargs='*',
                  default=[0, 60, 90, 120, 140, 160, 180, 200])
  ap.add_argument('--hold', type=float, default=6.0, help='단계별 유지 시간[s]')
  ap.add_argument('--steer', action='store_true',
                  help='조향도 좌우로 흔든다 (구동+조향 동시 = 최악 조건)')
  ap.add_argument('--steer-amp', type=float, default=10.0)
  ap.add_argument('--yes', action='store_true', help='안전 확인 생략')
  args = ap.parse_args()

  port = resolve_port(args.port)
  if not port:
    print('아두이노 포트를 못 찾았다.', file=sys.stderr)
    return 1

  print('=' * 72)
  print('전원 스트레스 시험 — 구동 PWM 을 올리며 전원계 한계를 찾는다')
  print(f'  포트 {port} · 단계 {args.pwms} · 각 {args.hold:.0f}초'
        + ('  · 조향 동시 흔듦' if args.steer else ''))
  print('  ⚠ 바퀴가 공중에 떠 있어야 한다. 지상이면 차가 달린다.')
  print('  ⚠ **차량 배터리 연결 필수.** USB 전원만으로는 측정이 무의미하다')
  print('     (모터 전류에 USB 레일이 무너져 PWM 60 부터 끊기는 것처럼 보인다).')
  print('=' * 72)
  if not args.yes:
    try:
      if input('진행? [y/N] ').strip().lower() not in ('y', 'yes'):
        print('중단.')
        return 0
    except EOFError:
      print('입력 없음 — 중단. (자동 실행이면 --yes)')
      return 0

  ser = open_port(port)
  time.sleep(1.0)
  rows = []
  log = []
  try:
    for pwm in args.pwms:
      log.append(f'  [PWM {pwm}]')
      ser, vccs, drops, reboots = step(ser, port, pwm, args.hold,
                                       args.steer_amp if args.steer else 0.0,
                                       log)
      rows.append((pwm, vccs, drops, reboots))
      print(f'  PWM {pwm:3d} … VCC 샘플 {len(vccs)}개, 끊김 {drops}, '
            f'리부팅 {reboots}')
      # 다음 단계 전 감속 — 급격한 단차를 피한다(펌웨어 레이트 제한도 있다)
      try:
        ser.write(b'PWM:0\n')
      except Exception:  # noqa: BLE001
        pass
      time.sleep(1.5)
  except KeyboardInterrupt:
    print('\n중단 — 지금까지 결과로 요약한다.')
  except RuntimeError as e:
    print(f'\n{e}')
  finally:
    for _ in range(5):
      try:
        ser.write(b'PWM:0\n')
        ser.write(b'VEL:0,STEER:0\n')
      except Exception:  # noqa: BLE001
        pass
      time.sleep(0.05)
    try:
      ser.close()
    except Exception:  # noqa: BLE001
      pass

  print()
  print('=' * 72)
  hdr = (f"{'PWM':>4} {'추정속도':>8} {'VCC최저':>8} {'VCC중앙':>8} "
         f"{'샘플':>5} {'끊김':>5} {'리부팅':>6}  판정")
  print(hdr)
  print('-' * 72)
  first_bad = None
  for pwm, vccs, drops, reboots in rows:
    v = (pwm - 80) / 95.0 if pwm > 80 else 0.0
    if vccs:
      lo, med = min(vccs), int(statistics.median(vccs))
      vs = verdict_vcc(lo)
    else:
      lo = med = 0
      vs = '텔레메트리 없음'
    bad = drops > 0 or reboots > 0
    if bad and first_bad is None and pwm > 0:
      first_bad = pwm
    mark = ' ❌' if bad else ''
    print(f'{pwm:4d} {v:7.2f}m/s {lo:7d} {med:7d} {len(vccs):5d} '
          f'{drops:5d} {reboots:6d}  {vs}{mark}')
  print('-' * 72)
  # 배터리 미연결 오진을 먼저 걸러낸다.
  low_fail = (first_bad is not None and first_bad <= 90)
  no_recovery = all(len(r[1]) == 0 for r in rows if r[0] > (first_bad or 1e9))
  if low_fail and no_recovery:
    print('  ⚠ 아주 낮은 PWM 부터 끊기고 그 뒤 텔레메트리가 안 돌아온다.')
    print('     → **차량 배터리가 연결돼 있는지 먼저 확인할 것.** USB 전원만으로')
    print('        모터를 돌리면 정확히 이 모양이 나온다(2026-09-11 오진 사례).')
    print('        배터리를 연결하고 다시 측정하기 전에는 이 표를 믿지 말 것.')
    print('-' * 72)
  if first_bad is None:
    top = max(r[0] for r in rows)
    print(f'  ✅ 시험한 전 구간({top}까지) 끊김·리부팅 없음.')
    print('     더 높은 PWM 도 시험해 한계를 찾을 것 (--pwms 로 지정).')
  else:
    safe = max([r[0] for r in rows if r[0] < first_bad] or [0])
    print(f'  ❌ PWM {first_bad} 부터 끊김/리부팅이 발생한다.')
    print(f'     무부하 기준 안전 상한은 그 아래 단계인 PWM {safe} '
          f'(≈{max(0.0, (safe - 80) / 95.0):.2f} m/s) 다.')
    print('     ⚠ 이건 바퀴를 든 무부하다. **지상에서는 더 낮다.**')
    print('     tools/lap_budget.py 에 이 PWM 을 넣어 예상 감점을 확인할 것.')
  print('-' * 72)
  print('  VCC 는 ATmega 내부 밴드갭 자가측정이라 PWM 노이즈에 오염될 수 있다.')
  print('  상향을 결정하기 전에 **멀티미터로 5V 레일을 실측**해 대조할 것.')
  print('=' * 72)
  if log:
    print('\n[이벤트 로그]')
    for line in log[:80]:
      print(line)
  return 0


if __name__ == '__main__':
  sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reverse_bench.py — PLAN PHASE 0: 후진이 물리적으로 되는지 측정으로 확정한다.

★ 반드시 바퀴를 공중에 띄우고 실행할 것. 차가 튀어나간다.
★ ROS 스택은 꺼두고 실행할 것 (serial_bridge 가 포트를 잡고 있으면 열리지 않는다).
      bash tools/ros_cleanup.sh

측정하는 것 (전부 눈이 아니라 숫자로):
  0-1 후진 구동 부호 — PWM:-60 에서
        · ENC1 이 **증가**하는가 (전진이 감소였으므로 후진은 증가여야 함)
        · VEL 이 **음수**로 계산되는가
        · (눈으로) 바퀴가 후진 방향으로 도는가
  0-2 후진 중 조향  — PWM:-60 + STEER:±10 에서 조향 ADC 가 목표로 가는가

  python3 tools/reverse_bench.py            # 전체
  python3 tools/reverse_bench.py --pwm -50  # 더 약하게
  python3 tools/reverse_bench.py --port /dev/ttyACM0

⚠ 엔코더 부호를 손으로 굴려 판단하지 말 것 — 방향을 착각하기 쉽다.
   반드시 모터로 구동해서 확인한다(HANDOFF 4절).
"""

import argparse
import glob
import re
import sys
import time

try:
  import serial
  from serial.tools import list_ports
except ImportError:
  print('pyserial 이 없다:  pip3 install pyserial')
  sys.exit(1)

ENC_RE = re.compile(r'ENC1=(-?\d+)')
VEL_RE = re.compile(r'VEL=(-?[\d.]+)')
ADC_RE = re.compile(r'STEER: ADC=(-?\d+)')
TGT_RE = re.compile(r'STEER: ADC=-?\d+\s+TGT=(-?\d+)')
VMIN_RE = re.compile(r'VMIN=(\d+)')
# 조향 PWM 과 스톨가드 — 조향이 '안 되는' 원인을 가르는 핵심 정보다.
#   PWM 0        → 애초에 명령이 안 걸렸다(데드밴드/쿨다운)
#   PWM 높은데 안 움직임 → 전압 부족 또는 기계적 고착
#   STALL steer=1 → 스톨가드가 끊었다(250ms 무이동 + 1.5s 쿨다운)
SPWM_RE = re.compile(r'STEER: ADC=-?\d+\s+TGT=-?\d+\s+PWM=(-?\d+)')
STALL_RE = re.compile(r'STALL: drive=(\d+)\s+steer=(\d+)')


def find_port(explicit=None):
  """serial_bridge 와 같은 순서: /dev/arduino → Mega(2341:0042) → ttyACM*."""
  if explicit:
    return explicit
  import os
  if os.path.exists('/dev/arduino'):
    return '/dev/arduino'
  for p in list_ports.comports():
    if p.vid == 0x2341 and p.pid == 0x0042:
      return p.device
  acms = sorted(glob.glob('/dev/ttyACM*'))
  if acms:
    print(f'⚠ Mega 를 특정 못 해 {acms[0]} 를 쓴다 (u-blox 도 ACM 이라 위험). '
          f'틀리면 --port 로 지정할 것.')
    return acms[0]
  return None


class Bench:

  # ★ 57600 이다. 펌웨어가 Serial.begin(57600) 이고 serial_bridge 기본값도 같다.
  #   115200 으로 열면 포트는 열리지만 텔레메트리가 한 줄도 안 온다(깨져서 버려짐).
  def __init__(self, port, baud=57600):
    self.ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(2.0)          # 아두이노 리셋 대기
    self.ser.reset_input_buffer()

  def send(self, cmd):
    self.ser.write((cmd + '\n').encode())

  def read_state(self, duration=0.6):
    """duration 동안 읽어 마지막으로 본 값들을 돌려준다."""
    enc = vel = adc = tgt = vmin = None
    spwm = None
    stall = None
    t_end = time.time() + duration
    while time.time() < t_end:
      try:
        line = self.ser.readline().decode(errors='ignore')
      except Exception:  # noqa: BLE001
        continue
      if not line:
        continue
      m = ENC_RE.search(line)
      if m:
        enc = int(m.group(1))
      m = VEL_RE.search(line)
      if m:
        vel = float(m.group(1))
      m = ADC_RE.search(line)
      if m:
        adc = int(m.group(1))
      m = TGT_RE.search(line)
      if m:
        tgt = int(m.group(1))
      m = VMIN_RE.search(line)
      if m:
        vmin = int(m.group(1))
      m = SPWM_RE.search(line)
      if m:
        spwm = int(m.group(1))
      m = STALL_RE.search(line)
      if m:
        stall = (int(m.group(1)), int(m.group(2)))
    return enc, vel, adc, tgt, vmin, spwm, stall

  def link_check(self, secs=3.0):
    """모터를 건드리기 전에 통신이 살아있는지 먼저 보여준다.

    ★ 왜 필요한가
      바로 구동 시험에 들어가면, 텔레메트리를 못 읽고 있는 건지 그냥 기다리는
      건지 화면만 봐서는 알 수 없다(멈춘 것처럼 보인다). 모터를 돌리기 전에
      숫자가 흐르는 걸 눈으로 확인하는 게 안전하기도 하다.
    """
    print('  통신 확인 중… (모터 안 돌린다)', flush=True)
    t_end = time.time() + secs
    seen = 0
    last = None
    while time.time() < t_end:
      enc, vel, adc, tgt, vmin, _sp, _st = self.read_state(0.3)
      if enc is not None:
        seen += 1
        last = (enc, vel, adc, vmin)
        print(f'    ENC1={enc}  VEL={vel if vel is not None else "?"}  '
              f'STEER_ADC={adc if adc is not None else "?"}  '
              f'VCC_MIN={vmin if vmin is not None else "?"}mV', flush=True)
    if seen and last is not None:
      enc, vel, adc, vmin = last
      # ★ 전압 확인. ATmega2560 의 BOD 는 보통 2.7V 다. 그 근처에서 모터를 돌리면
      #   부하 시 더 떨어져 **보드가 리셋된다.** 그 상태의 측정값은 못 믿는다.
      if vmin is not None:
        if vmin < 3000:
          print(f'\n  ⛔ 공급전압 {vmin}mV — 브라운아웃 임계(약 2700mV) 근처다.',
                flush=True)
          print('     이대로 모터를 돌리면 보드가 리셋되고, 측정값도 못 믿는다.',
                flush=True)
          print('     배터리를 충전/교체한 뒤 다시 할 것.', flush=True)
          print('     (정상대는 3900~4100mV. 무부하인데 낮으면 배터리 문제다)',
                flush=True)
          return 'LOW_VOLTAGE'
        if vmin < 3600:
          print(f'\n  ⚠ 공급전압 {vmin}mV — 정상대(3900~4100)보다 낮다. '
                f'부하를 걸면 더 떨어진다.', flush=True)
      if adc is not None:
        print(f'  조향 ADC {adc} '
              f'(캘리브 중앙 412 기준 {(adc - 412) / 26.2:+.1f}°)', flush=True)

    if seen == 0:
      print('\n  ❌ 텔레메트리가 안 온다. 확인할 것:', flush=True)
      print('     · 아두이노가 리셋 중일 수 있다 — 몇 초 뒤 다시 실행', flush=True)
      print('     · 다른 프로세스가 포트를 잡고 있는가 '
            '(bash tools/ros_cleanup.sh)', flush=True)
      print(f'     · 보드레이트가 맞는가 (지금 {self.ser.baudrate}, '
            f'펌웨어는 Serial.begin 값 — 이 차는 57600)', flush=True)
      return None
    print(f'  ✅ 통신 정상 ({seen}회 수신)', flush=True)
    return last

  def stop(self):
    """정지 명령. ★ Ctrl-C 가 이걸 끊지 못하게 한다.

    인터럽트가 정리 코드를 관통하면 모터가 계속 도는 채로 스크립트만 죽는다.
    (펌웨어 워치독이 500ms 뒤 세워주긴 하지만, 그건 마지막 방어선이지
     여기서 안 보내도 되는 이유가 아니다.)
    """
    for _ in range(5):
      try:
        self.ser.write(b'PWM:0\nSTEER:0\n')
        self.ser.flush()
        time.sleep(0.05)
      except KeyboardInterrupt:
        continue          # Ctrl-C 를 먹고 계속 — 반드시 보낸다
      except Exception:   # noqa: BLE001
        break

  def close(self):
    try:
      self.stop()
    finally:
      for _ in range(3):
        try:
          self.ser.close()
          break
        except KeyboardInterrupt:
          continue
        except Exception:  # noqa: BLE001
          break


def drive_test(b, pwm, secs, label):
  """지정 PWM 을 걸고 ENC 변화량과 VEL 을 잰다."""
  b.send('PWM:0')
  time.sleep(0.5)
  enc0, *_ = b.read_state(0.8)
  if enc0 is None:
    print('  ❌ 텔레메트리(STATUS_10ms)를 못 읽었다 — 포트/보드레이트 확인')
    return None

  print(f'  {label}: PWM:{pwm} 을 {secs:.0f}초 인가한다…', flush=True)
  t_end = time.time() + secs
  vels, vmins = [], []
  while time.time() < t_end:
    b.send(f'PWM:{pwm}')          # 워치독이 있으므로 계속 보낸다
    _, v, _, _, vm, _sp, _st = b.read_state(0.15)
    if v is not None:
      vels.append(v)
    if vm is not None:
      vmins.append(vm)
  b.send('PWM:0')
  time.sleep(0.6)
  enc1, *_ = b.read_state(0.8)

  d_enc = (enc1 - enc0) if (enc1 is not None) else None
  v_mid = sorted(vels)[len(vels) // 2] if vels else None
  vmin = min(vmins) if vmins else None
  print(f'    ENC1 {enc0} → {enc1}   변화 {d_enc:+d}' if d_enc is not None
        else '    ENC1 읽기 실패')
  print(f'    VEL 중앙값 {v_mid:+.3f} m/s' if v_mid is not None
        else '    VEL 읽기 실패')
  if vmin is not None:
    print(f'    VMIN {vmin} mV' + ('   ⚠ 3.5V 이하 — 전압강하 주의'
                                   if vmin < 3500 else ''))
  return d_enc, v_mid


def steer_test(b, pwm, angle, secs=2.5):
  """후진 구동 중 조향이 목표로 가는지.

  '안 움직인다' 를 세 가지로 갈라준다 — 구분 못 하면 원인을 못 찾는다.
    · 조향 PWM 이 계속 0       → 명령이 안 걸림(데드밴드 안이거나 스톨 쿨다운)
    · PWM 은 걸리는데 ADC 고정 → 힘 부족(전압) 또는 기계적 고착
    · STALL steer=1            → 스톨가드가 끊음(250ms 무이동 → 1.5s 쿨다운)
  """
  b.send('PWM:0')
  time.sleep(0.3)
  _, _, adc0, _, _, _, _ = b.read_state(0.6)
  print(f'    STEER:{angle:+d}° 인가 중… ({secs:.1f}초)', flush=True)
  t_end = time.time() + secs
  pwms, adcs, stalls = [], [], 0
  while time.time() < t_end:
    b.send(f'PWM:{pwm}')
    b.send(f'STEER:{angle}')
    _, _, a, _, _, sp, st = b.read_state(0.1)
    if sp is not None:
      pwms.append(sp)
    if a is not None:
      adcs.append(a)
    if st is not None and st[1]:
      stalls += 1
  _, _, adc1, tgt, _, _, _ = b.read_state(0.6)
  b.send('PWM:0')
  b.send('STEER:0')
  if adc0 is None or adc1 is None:
    print(f'    STEER:{angle:+d}° → ADC 읽기 실패', flush=True)
    return None
  moved = adc1 - adc0
  pwm_max = max((abs(p) for p in pwms), default=0)
  span = (max(adcs) - min(adcs)) if adcs else 0
  print(f'    STEER:{angle:+3d}° → ADC {adc0} → {adc1} ({moved:+d}), '
        f'목표 TGT={tgt}', flush=True)
  print(f'        조향PWM 최대 {pwm_max}   ADC 움직인 폭 {span}'
        f'{f"   ⚠ 스톨가드 {stalls}회" if stalls else ""}', flush=True)
  if abs(moved) < 10:
    if pwm_max == 0:
      print('        → 조향 PWM 이 0 이다. 데드밴드 안이거나 스톨 쿨다운 중.',
            flush=True)
    elif stalls:
      print('        → 스톨가드가 끊었다. 전압 부족이거나 조향이 물려 있다.',
            flush=True)
    else:
      print(f'        → PWM {pwm_max} 을 걸었는데 안 움직인다. '
            f'전압/기계 고착 의심.', flush=True)
  return moved, adc1, tgt


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--port', default=None)
  ap.add_argument('--baud', type=int, default=57600,
                  help='펌웨어 Serial.begin 과 같아야 한다 (기본 57600)')
  ap.add_argument('--pwm', type=int, default=-60,
                  help='후진 개루프 PWM (음수). 기본 -60')
  ap.add_argument('--secs', type=float, default=3.0)
  ap.add_argument('--skip-steer', action='store_true')
  ap.add_argument('--force', action='store_true',
                  help='저전압 경고를 무시하고 강행 (권하지 않음)')
  ap.add_argument('--steer-only', action='store_true',
                  help='구동 시험(0-1)을 건너뛰고 후진 중 조향(0-2)만 한다')
  args = ap.parse_args()

  port = find_port(args.port)
  if not port:
    print('❌ 아두이노 포트를 못 찾았다. 연결 확인 후 --port 로 지정할 것.')
    return 1

  print('=' * 58)
  print('PHASE 0 — 후진 최소 검증   ★ 바퀴 공중 확인했는가?')
  print('=' * 58)
  print(f'포트: {port}')
  print('아두이노 연결 중… (리셋 대기 2초)', flush=True)

  b = Bench(port, args.baud)
  # 모터를 건드리기 전에 링크와 전압부터 확인한다.
  chk = b.link_check()
  if chk is None:
    b.close()
    return 1
  if chk == 'LOW_VOLTAGE' and not args.force:
    print('\n  (그래도 강행하려면 --force. 권하지 않는다.)', flush=True)
    b.close()
    return 1

  if args.steer_only:
    print(f'\n후진 중 조향만 시험한다 (PWM:-{abs(args.pwm)} + STEER:±10°)',
          flush=True)
  else:
    print(f'\n이제 모터를 돌린다. 전진 {args.secs:.0f}초 → 후진 {args.secs:.0f}초 '
          f'(PWM ±{abs(args.pwm)})', flush=True)
  try:
    input('★ 바퀴가 공중에 떠 있으면 Enter, 아니면 Ctrl-C: ')
  except KeyboardInterrupt:
    print('\n중단.')
    b.close()
    return 1

  ok_enc = ok_vel = None
  steer_ok = None
  try:
    if not args.steer_only:
      # ---- 0-1 전진 기준선 (부호 비교 대상) ----
      print('\n[0-1a] 전진 기준선')
      fwd = drive_test(b, abs(args.pwm), args.secs, '전진')
      time.sleep(1.0)

      print('\n[0-1b] 후진')
      rev = drive_test(b, -abs(args.pwm), args.secs, '후진')

      ok_enc = ok_vel = False
      if fwd and rev and fwd[0] is not None and rev[0] is not None:
        ok_enc = (fwd[0] * rev[0] < 0) and abs(rev[0]) > 20
      if fwd and rev and fwd[1] is not None and rev[1] is not None:
        ok_vel = fwd[1] > 0.05 and rev[1] < -0.05

    # ---- 0-2 후진 중 조향 ----
    if not args.skip_steer:
      print('\n[0-2] 후진 중 조향')
      left = steer_test(b, -abs(args.pwm), 10)
      time.sleep(0.5)
      right = steer_test(b, -abs(args.pwm), -10)
      if left and right:
        # 좌우가 서로 반대 방향으로 움직여야 정상
        steer_ok = (left[0] * right[0] < 0) and \
                   (abs(left[0]) > 10 and abs(right[0]) > 10)
  except KeyboardInterrupt:
    print('\n사용자 중단 — 정지 명령을 보낸다.', flush=True)
  finally:
    b.close()

  print('\n' + '=' * 58)
  print('판정')
  print('=' * 58)
  if ok_enc is not None:
    print(f'  엔코더 부호 (전진↔후진 반대)   '
          f'{"✅" if ok_enc else "❌"}')
  if ok_vel is not None:
    print(f'  속도 계산 부호 (후진이 음수)   '
          f'{"✅" if ok_vel else "❌"}')
  if steer_ok is not None:
    print(f'  후진 중 조향 (좌우 반대로 감)  '
          f'{"✅" if steer_ok else "❌"}')

  ran = [v for v in (ok_enc, ok_vel, steer_ok) if v is not None]
  if not ran:
    print('  (측정된 항목 없음 — 중단되었다)')
    return 1
  allok = all(ran)
  print()
  if allok:
    if ok_enc is None:
      print('✅ 후진 중 조향 정상. (구동 부호 0-1 은 이번에 안 쟀다)')
    else:
      print('✅ PHASE 0 통과 — 현장 좌표 기록으로 진행해도 된다.')
  else:
    print('❌ 실패 — 여기서 멈추고 원인부터 잡을 것.')
    if ok_enc is False:
      print('   ENC 부호가 같다: counts_per_revolution 부호 / 엔코더 배선')
    if ok_vel is False:
      print('   VEL 이 0/부호 이상: NO_ENCODER 설정 또는 SPI 카운터(CS22/23)')
    if steer_ok is False:
      print('   조향: 조향핀(A15) / 데드밴드 / STEER 캘리브 확인.')
      print('   ※ 좌우가 반대로 움직이면 극성 문제이므로, 주차 노드는')
      print('      -p steer_sign_reverse:=-1.0 로 돌리면 된다(코드 수정 불필요).')
  return 0 if allok else 1


if __name__ == '__main__':
  sys.exit(main())

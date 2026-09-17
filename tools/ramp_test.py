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
M_PER_COUNT = (2 * math.pi * WHEEL_R) / COUNTS_PER_REV


def _enc_speed(t, e, win=0.25):
  """엔코더 카운트를 win 초 중심차분해 속도[m/s] 를 낸다 (전진 +).

  펌웨어의 /current_speed 보다 훨씬 조용하다 — 창이 25배 길기 때문이다.
  대가는 0.25초의 시간 분해능인데, 경사 판정에는 충분하다.
  """
  n = len(t)
  out = [0.0] * n
  for i in range(n):
    lo = hi = i
    while lo > 0 and t[i] - t[lo] < win / 2:
      lo -= 1
    while hi < n - 1 and t[hi] - t[i] < win / 2:
      hi += 1
    dt = t[hi] - t[lo]
    out[i] = (-(e[hi] - e[lo]) * M_PER_COUNT / dt) if dt > win * 0.4 else 0.0
  return out


def judge(rows, label):
  """기록된 행들로 판정표를 낸다."""
  if len(rows) < 10:
    print(f'❌ 표본 {len(rows)}개뿐 — 기록이 안 됐다')
    return 1
  t = [r['t'] for r in rows]
  # ★ 2026-09-17 — 펌웨어 /current_speed 를 그대로 쓰면 안 된다.
  #   10ms 창 미분이라 양자화 잡음이 그대로 실린다(엔코더 1카운트 = 0.0575 m/s).
  #   실측: 갱신 간 변화량 중앙 0.115 = 정확히 2카운트, 0.5m/s 넘는 점프가
  #   평지·경사 똑같이 8~9%. 그 잡음을 '바퀴 슬립' 으로 읽어 오진했다.
  #   카운트 자체는 멀쩡하므로 **0.25초 중심차분**으로 다시 만든다.
  v = _enc_speed(t, [r['enc'] for r in rows])
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
  # 전진 = 카운트 **감소**(counts_per_revolution 이 음수). 부호를 살려야
  # '뒤로 밀림' 을 '이동' 으로 세지 않는다.
  fwd = -(e[-1] - e[0]) * M_PER_COUNT      # + 면 전진
  dist = abs(fwd)
  fwd_all = [-(r['enc'] - e[0]) * M_PER_COUNT for r in rows]
  vmax = max(abs(x) for x in v)
  arrow = '전진' if fwd >= 0 else '**뒤로**'
  print(f'\n[{label}]  {dur:.1f}s · {len(rows)}샘플 · {arrow} {dist:.1f} m')
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

  # ── 거리별 속도 프로파일 — 조주(run-up) 시험의 핵심 ──────────────
  #   경사는 정지출발이 아니라 **달려와서** 오른다(대회도 그렇다). 그러면
  #   "어디서 얼마나 느려졌나" 가 전부다. 최대 속도만 봐서는 안 보인다.
  # ⚠ 부호 있는 전진거리로 나눠야 한다. abs() 로 하면 올라갔다 되밀린 런에서
  #   칸이 뒤엉킨다(실차에서 3.76m 올라갔다 −2.94m 까지 내려온 런이 있었다).
  # ⚠ **최고점까지만** 본다. 되밀린 런은 같은 지점을 두 번 지나므로
  #   그대로 평균하면 올라갈 때와 내려올 때가 상쇄돼 전부 0 근처가 된다
  #   (실차 런에서 실제로 그랬다).
  i_peak = fwd_all.index(max(fwd_all))
  d_all = fwd_all[:i_peak + 1]
  v_up = v[:i_peak + 1]
  span = max(d_all) if d_all else 0.0
  if span > 1.0:
    # 긴 런은 칸을 더 잘게. 100m 를 12칸으로 나누면 8m 씩이라 경사 진입점이
    # 뭉개진다. 대략 2~5m 한 칸이 되게 잡는다.
    nb = min(24, max(4, int(span / 2) if span > 24 else int(span)))
    tail = '' if i_peak >= len(rows) - 5 else '  ※ 최고점까지만 (그 뒤는 되밀림)'
    print(f'\n   거리별 속도 (전진 {span:.1f}m 를 {nb}칸으로){tail}')
    step = span / nb
    prev_v = None
    for b in range(nb):
      lo, hi = b * step, (b + 1) * step
      vs = [vv for dd, vv in zip(d_all, v_up) if lo <= dd < hi]
      if not vs:
        continue
      mv = sum(vs) / len(vs)
      bar = '█' * max(1, int(abs(mv) / max(0.1, max(abs(x) for x in v)) * 24))
      tag = ''
      if mv < -0.05:
        tag = '  ← **뒤로**'
      elif prev_v is not None and mv < prev_v * 0.6:
        tag = '  ← 급감속'
      prev_v = mv
      print(f'   {lo:5.1f}~{hi:4.1f}m  {mv:5.2f} m/s  {bar}{tag}')

    # 감속 구간에서 경사를 역산한다 (구동이 걸린 채이므로 '유효' 경사다)
    half = len(d_all) // 2
    if len(d_all) > 20 and abs(v_up[-1]) < abs(v_up[half]) * 0.7:
      dv = abs(v_up[half]) ** 2 - abs(v_up[-1]) ** 2
      dd = max(d_all[-1] - d_all[half], 1e-6)
      dec = dv / (2 * dd)
      print(f'   ※ 후반 감속 {dec:.2f} m/s² — 이만큼 더 밀어야 등속이 된다')
      need = math.sqrt(2 * dec * span) if dec > 0 else 0.0
      print(f'     같은 설정으로 {span:.0f}m 를 다 오르려면 '
            f'진입속도 **{need:.2f} m/s** 이상이어야 한다')

  # ── 뒤로 밀림 (롤백) ─────────────────────────────────────────────
  #   이 차는 엔코더 홀드가 없다. 경사에서 서면 그냥 굴러 내려간다.
  #   실제로 겪었다: 명령 0 · PWM 0 인데 19초 동안 **뒤로 5.05m** 굴렀다.
  #   경사로 미션의 핵심 위험이고, 부호를 안 보면 '이동 5m' 로 보여 놓친다.
  peak = max(fwd_all)
  lost = peak - fwd_all[-1]          # 최고점에서 되밀린 양
  # ⚠ −0.05 로 잡으면 양자화 잡음(±0.0575)을 후진으로 센다. 2카운트 위로.
  back_cmd = sum(1 for cc, vv in zip(cv, v) if cc > 0.05 and vv < -0.12)
  if back_cmd > len(rows) * 0.05:
    say('실패', '경사 못 버팀',
        f'**전진 명령 중에** 뒤로 간 샘플 {back_cmd}개 '
        f'({100 * back_cmd / len(rows):.0f}%) · 최고점에서 {lost:.2f}m 되밀렸다. '
        f'추진력이 경사를 못 버틴다 — PWM 을 올리거나 이 경사를 포기할 것')
  elif lost > 0.3:
    say('실패', '뒤로 밀림',
        f'최고점 {peak:.2f}m 에서 **{lost:.2f}m** 되밀렸다. 엔코더 홀드가 '
        f'없어 경사에서 서면 굴러간다 — 멈추면 잡거나 굄목을 댈 것')
  else:
    say('통과', '뒤로 밀림', f'되밀림 {max(lost, 0.0):.2f}m')

  # ── PWM — '못 올라감' 의 원인을 가른다 ───────────────────────────
  #   PWM 이 높은데 안 움직이면 토크 부족(또는 잠김).
  #   PWM 이 낮은데 안 움직이면 **명령이 모자란 것** — 설정을 올리면 된다.
  pw = [abs(r.get('pwm', 0)) for r in rows]
  if any(pw):
    moving = [abs(x) > 0.05 for x in v]
    run_pw = sorted(p for p, mv in zip(pw, moving) if mv)
    med = run_pw[len(run_pw) // 2] if run_pw else 0
    # ⚠ 출발 램프(첫 1초)만 뺀다. 2초로 잡았더니 **breakaway 구간(기본 1.2초)이
    #   통째로 빠져서**, PWM 110 으로 밀어도 안 움직인 런을 'PWM 90 뿐' 이라고
    #   보고했다. 출발 정지마찰을 뚫는 값이 얼마였는지가 여기서 제일 중요하다.
    stuck_pw = [p for p, c, mv, tt in zip(pw, cv, moving, t)
                if abs(c) > 0.1 and not mv and tt > 1.0]
    print(f'   PWM   최대 {max(pw)} · 달릴 때 중앙 {med}')
    if stuck_pw:
      mx = max(stuck_pw)
      # ⚠ 전압으로 구동 전류를 판단하면 안 된다. tools/drive_diag.py 실측:
      #   구동 모터가 **정상 회전**할 때도 /vcc_mv 강하가 0mV 였고, 조향은
      #   같은 배터리에서 2160mV 를 끌어내렸다. 구동은 로직 레일과 **별도
      #   전원**이라 이 신호에 안 나타난다. (한때 '전압이 안 흔들리니 전력이
      #   안 간다' 는 검사를 넣었다가 정상 런에도 뜨는 것을 보고 지웠다.)
      if mx >= 180:
        say('실패', '추진력 한계',
            f'PWM {mx} (상한 230) 로도 안 움직였다 — 설정으로 올릴 여지가 '
            f'거의 없다. 경사라면 이 차의 한계이고, 평지라면 고장이다')
      else:
        say('경고', '추진력 부족',
            f'PWM {mx} 로 안 움직였다 — 상한 230 까지 여유가 있다. '
            f'ff_breakaway_pwm(출발) 과 ff_min_pwm(주행) 을 올려 볼 것')

  # ── 바퀴 슬립 ────────────────────────────────────────────────────
  #   같은 PWM · 같은 경사면 속도는 매끄러워야 한다. 0 ↔ 2.3 m/s 를 오가면
  #   접지를 잃고 헛돌다(엔코더는 빠르게 읽는다) 다시 물리는 것이다.
  #   ⚠ 문턱을 0.55 로 잡았다가 **정상 런까지 슬립으로 찍었다**. 원인은 두 개였다:
  #     ① 펌웨어 속도의 양자화 잡음(위 _enc_speed 주석)
  #     ② 가속 구간과 등반 감속을 한 덩어리로 재면 슬립이 없어도 변동이 크다
  #   엔코더 기준 실측: 평지 0.42 · 경사 0.48~0.59 — **경사가 더 조용하다**.
  #   즉 이 코스에서 슬립 증거는 없었다. 문턱을 0.95 로 올렸다.
  #   ⚠ 슬립이면 **PWM 을 올리면 더 나빠진다** — 처방이 정반대다.
  if any(pw):
    from collections import Counter
    common = Counter(p for p, mv in zip(pw, [abs(x) > 0.05 for x in v]) if mv)
    if common:
      p_mode, n_mode = common.most_common(1)[0]
      vs_mode = [abs(x) for x, p in zip(v, pw) if p == p_mode and abs(x) > 0.05]
      if len(vs_mode) >= 30:
        mean = sum(vs_mode) / len(vs_mode)
        var = sum((x - mean) ** 2 for x in vs_mode) / len(vs_mode)
        cv_ = (var ** 0.5) / max(mean, 1e-6)
        zero = sum(1 for x, p in zip(v, pw)
                   if p == p_mode and abs(x) <= 0.05)
        print(f'   슬립   PWM {p_mode} 고정 {len(vs_mode)}샘플 · '
              f'속도 평균 {mean:.2f} · 변동계수 {cv_:.2f} · 정지 {zero}회')
        if cv_ > 0.95:
          say('실패', '바퀴 슬립',
              f'PWM {p_mode} 로 일정한데 속도 변동계수가 {cv_:.2f} 다 '
              f'(0~{max(vs_mode):.2f} m/s 를 오간다). 접지를 잃고 헛도는 것 — '
              f'**PWM 을 올리면 더 나빠진다.** 바퀴가 도는데 차가 안 나가는지 '
              f'눈으로 확인할 것')
        elif cv_ > 0.75:
          say('경고', '바퀴 슬립', f'속도 변동계수 {cv_:.2f} — 슬립 의심')
        else:
          say('통과', '바퀴 슬립', f'속도 변동계수 {cv_:.2f} (매끄럽다)')

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
  # ★ 2026-09-17 수정 — 예전엔 **연속 샘플**끼리 비교했다. 기록기는 120Hz 로
  #   도는데 엔코더는 18~20Hz 로 갱신되므로, 같은 값이 연속으로 찍히는 게
  #   정상이다. 그걸 끊김으로 세서 **정상 런에 '72% 멈춤' 실패**를 냈다.
  #   (이 저장소에서 같은 실수를 두 번째로 했다 — drive_review 의 변화율 분모와
  #    같은 함정이다. 비교는 샘플이 아니라 **시간**으로 할 것.)
  #   진짜 끊김은 '움직이는 중인데 갱신 주기의 몇 배 동안 값이 그대로' 다.
  UPD_S = 0.25          # 20Hz 기준 5주기. 이보다 오래 멈추면 진짜다
  worst, held_t, held_v = 0.0, None, None
  for i in range(len(rows)):
    if held_v is None or enc[i] != held_v:
      held_v, held_t = enc[i], t[i]
      continue
    if abs(cv[i]) > 0.1 and abs(v[i]) > 0.2:
      worst = max(worst, t[i] - held_t)
  if worst > UPD_S:
    say('실패', '엔코더 연속성',
        f'달리는 중 카운트가 {worst:.2f}s 동안 멈췄다 (한계 {UPD_S}s) '
        f'— 이 상태로 NO_ENCODER 0 을 켜면 PID 가 오동작한다')
  else:
    say('통과', '엔코더 연속성',
        f'최장 정지 {worst:.3f}s (갱신주기 ~0.05s · 한계 {UPD_S}s)')

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
  import signal
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

  # ★ 2026-09-17 — Ctrl-C 를 **직접** 잡는다.
  #   rclpy 기본 핸들러는 컨텍스트를 먼저 무효화해서, finally 의 정지 명령이
  #   `publisher's context is invalid` 로 터진다. 실차에서 실제로 그랬다:
  #   **정지 명령이 안 나가고 기록도 저장 안 됐다**(펌웨어 워치독 0.5초가
  #   차를 세우긴 했지만, 우리가 세운 게 아니다).
  stop_flag = {'hit': False}

  def _sigint(_sig, _frm):
    stop_flag['hit'] = True

  signal.signal(signal.SIGINT, _sigint)

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
  stuck_dist = 0.0
  slip_since = None
  limit = a.seconds
  if drive:
    # 예상시간의 3배 + 10초. 스톨·미끄러짐으로 안 끝나는 것을 막는다.
    limit = min(a.seconds, a.drive / max(a.speed, 0.05) * 3.0 + 10.0)
    print(f'  ▶ 자율 {a.drive:.1f}m · 목표 {a.speed:.2f} m/s · '
          f'상한 {limit:.0f}s · 중단속도 {a.abort_speed:.1f} m/s')

  def send(v):
    # 어떤 이유로든 발행이 실패해도 **죽지 않는다**. 여기서 예외가 나면
    # 정지 명령 반복이 중단되고 기록도 저장되지 않는다.
    if not drive:
      return
    try:
      from geometry_msgs.msg import Twist as _T
      m = _T()
      m.linear.x = float(v)
      m.angular.z = 0.0          # 직진 (조향 중앙)
      n.drive_pub.publish(m)
    except Exception:            # noqa: BLE001
      pass

  try:
    while time.time() - t0 < limit and rclpy.ok():
      if stop_flag['hit']:
        stop_reason = '사용자 중단 (Ctrl-C)'
        break
      rclpy.spin_once(n, timeout_sec=0.01)
      t = time.time() - t0
      # 발행이 ~20Hz 다. 50Hz 면 충분하고, 그 이상은 같은 값을 베껴 쓸 뿐이다.
      if rows and t - rows[-1]['t'] < 0.02:
        continue
      dist = abs(n.enc - e0) / COUNTS_PER_REV * (2 * math.pi * WHEEL_R)

      # ★ 기록을 **먼저** 한다. 예전엔 중단 판정이 먼저라 중단을 유발한
      #   샘플이 CSV 에 안 남았다 — '최대 2.47' 인데 2.5 로 중단돼 원인이
      #   기록에서 사라졌다(실제로 겪음).
      rows.append({'t': t, 'v': n.v, 'cmd_v': n.cmd_v, 'enc': n.enc,
                   'vcc': n.vcc, 'pwm': n.pwm, 'steer': n.steer,
                   'stall': int(n.stall)})

      if drive:
        # ── 중단 조건 ──
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
        # ★ 전진 명령인데 **뒤로 가면** 세운다. 경사를 못 버티고 미끄러지는
        #   상황이다. 예전엔 '안 움직인다' 만 봐서 이걸 못 잡았고, 실차에서
        #   명령 1.20 · PWM 90 이 걸린 채 **6.7m 를 미끄러져 내려갔다**.
        if a.speed > 0 and n.v < -0.15:
          slip_since = slip_since if slip_since is not None else t
          if t - slip_since > 1.0:
            stop_reason = (f'❌ 뒤로 미끄러짐 {abs(n.v):.2f} m/s — '
                           f'이 설정으로는 경사를 못 버틴다')
            break
        else:
          slip_since = None
        # ★ 정지 판정은 **거리**로 한다. 속도로 하면 안 잡힌다 —
        #   엔코더 1카운트 = 2.875mm 이고 20Hz 갱신이라 최소 분해속도가
        #   **0.0575 m/s** 다. 문턱 0.05 는 그 양자화 잡음 아래라서, 실제로
        #   선 차가 ±0.06 으로 떨리며 판정을 계속 리셋했다(실차에서 17초 동안
        #   중단이 안 걸렸다).
        if t > 2.0:
          if stuck_since is None or dist > stuck_dist + 0.15:
            stuck_since, stuck_dist = t, dist
          elif t - stuck_since > a.stuck_after:
            stop_reason = (f'❌ {t - stuck_since:.1f}s 동안 {dist - stuck_dist:.2f}m '
                           f'밖에 못 갔다 — 못 올라감')
            break
        # 출발 램프 — 1초에 걸쳐 올린다(덜컹 방지)
        send(a.speed * min(1.0, t / 1.0))

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

  # ★ 저장은 판정보다 **먼저**, 그리고 어떤 경우에도 한다. 실차에서 Ctrl-C
  #   한 번에 25초짜리 런이 통째로 날아갔다.
  if not rows:
    print('❌ 기록된 샘플이 없다')
    return 1
  try:
    with open(out, 'w', newline='') as f:
      w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
      w.writeheader()
      w.writerows(rows)
    print(f'\n저장: {out}  ({len(rows)}샘플)')
  except Exception as exc:       # noqa: BLE001
    print(f'❌ 저장 실패: {exc}')
  r = judge(rows, a.label)
  print(f'  ■ 종료 사유: {stop_reason}')
  return r


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
  ap.add_argument('--stuck-after', type=float, default=2.0, metavar='s',
                  help='안 움직인다고 판정하기까지의 시간. 조주 시험에서는 '
                       '경사에 올라 잠깐 멈칫할 수 있으니 늘려 잡는다')
  a = ap.parse_args()
  if a.report:
    if not os.path.exists(a.report):
      sys.exit(f'없는 파일: {a.report}')
    rows = [{k: (int(float(v)) if k in ('enc', 'vcc', 'stall', 'pwm')
                 else float(v))
             for k, v in r.items()}
            for r in csv.DictReader(open(a.report))]
    return judge(rows, os.path.basename(a.report))
  return record(a)


if __name__ == '__main__':
  raise SystemExit(main())

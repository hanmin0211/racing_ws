#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mission_plan_check.py — 미션 계획의 s 구간이 **코스 기하와 맞는지** 검사한다.

★ 왜 이게 필요한가 (2026-09-17)

  용인 계획은 9/19 현장에서 처음부터 짜야 한다. 그런데 s 구간을 잘못 잡으면
  증상이 '조용한 탈락' 이다 — 노드는 정상이고 로그도 멀쩡한데 **엉뚱한 지점에서
  미션이 돌거나 안 돈다.** 지금까지 이 판단은 전부 사람이 주석에 적어 둔
  것뿐이고, 검사하는 코드가 없었다.

  실제로 밟은 것들 (전부 학교 트랙에서 일어난 일):
    · arm 이 늦어 첫 장애물에 박았다 — 감지는 s38.6 에 끝났는데 arm 이 s40.5
      였고, 그 순간 이미 BLOCKED(2.8m) 안이었다. BLOCKED 는 조향을 안 낸다.
    · 회피와 돌발 구간이 겹쳐, 돌발의 inhibits 가 회피를 꺼버려 의자를 못 피했다.
    · 커브에서 회피를 arm 해 먹스가 경로조향을 버리고 가드레일로 밀었다.
    · 미션을 캘리브 구간(s<10)에 두어 아예 안 돌았다 — teleop 이 먹스 우선.

  시퀀서는 **런타임에** course 블록(웨이포인트 파일·길이)만 대조한다. 구간
  자체가 기하학적으로 말이 되는지는 아무도 안 본다. 그걸 여기서 본다.

사용:
  python3 tools/mission_plan_check.py config/chungju_school/mission_plan_school.yaml
  python3 tools/mission_plan_check.py <계획> --obstacle lidar_avoid=44
  python3 tools/mission_plan_check.py <계획> --obstacle sudden_stop=56 --wp <다른 wp>

판정 기준의 근거는 각 상수 옆에 적어 두었다. 바꿀 거면 **왜** 를 같이 적을 것.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── 판정 문턱 (전부 실측 근거) ───────────────────────────────────────────
# 곡률: AVOID 중에는 먹스가 경로조향을 **통째로 버린다**. 그래서 조향을
#   override 하는 미션은 커브에서 arm 하는 것 자체가 이탈 방향이다.
#   실측(학교 wp): s33 R=6.4m 급커브 / s35 R=8.1m '여기부터 경계'
#                  s37 R=11.0m 에서 arm → 실차 이탈 0.18m (경고선 0.72m 의 1/4)
ARM_R_FAIL, ARM_R_WARN = 8.0, 11.0
# 경로조향을 버린 채 달리는 거리 [m]. 이탈 ≈ d²/(2R) (원호에서 현까지의 거리).
#   2.0m 는 실측에서 AVOID 가 연속으로 유지된 전형적 거리다.
OVERRIDE_DIST = 2.0
# 이탈 경고선 — drive_review.py 와 **같은 값**을 쓴다(0.90m 넘으면 조향
#   최대치로도 복귀 불가, 실측 drive_0312).
DEV_WARN = 0.72
# 감지 트리거 4.0m (fg_obstacle_trigger) + BLOCKED 반경 2.8m.
#   정면 장애물이 2.8m 안에 들면 안전버블 asin(0.6375/r) 이 조준창 20.6° 를
#   덮어 무조건 BLOCKED 이고, BLOCKED 는 조향을 아예 안 낸다(NaN).
#   즉 **감지가 시작되기 전에 이미 arm 돼 있어야** 빠져나갈 길이 있다.
#   실측: arm s37·장애물 s44 → 여유 7.0m → obs 최소 8.00m (안 박음)
#         arm s40·장애물 s44 → 여유 4.0m → arm 순간 obs 2.04m (박음)
DETECT_TRIGGER, BLOCKED_R = 4.0, 2.8
LEAD_MIN = DETECT_TRIGGER + BLOCKED_R          # 6.8m
# ★ 2026-09-17 — BLOCKED 시작 거리는 **장애물 크기에 따라 달라진다**.
#   위 2.8m 는 학교 의자(반폭 0.20m)로 실측한 값이라 더 큰 장애물엔 안 맞는다.
#   안전버블이 조준창을 덮는 거리:  r = (장애물반폭 + 안전반경) / sin(조준창)
#   용인은 **헤네스 브룬 차체(바퀴 뺀 것) 2개를 무작위 배치**한다:
#       세로로 놓이면 반폭 0.30~0.35m → BLOCKED 2.67~2.81m
#       가로로 놓이면 반폭 0.60m      → BLOCKED 3.52m   ← 무작위면 이것도 나온다
#   --obstacle-width 로 폭을 주면 이 식으로 다시 잰다.
VEHICLE_WIDTH, SAFETY_MARGIN = 0.775, 0.25
SAFETY_RADIUS = VEHICLE_WIDTH / 2 + SAFETY_MARGIN


def blocked_radius(obstacle_w):
  """장애물 폭[m] 에서 BLOCKED 시작 거리[m] 를 낸다."""
  aim = math.atan((1.2 - VEHICLE_WIDTH / 2 - SAFETY_MARGIN) / 1.5)
  return (obstacle_w / 2 + SAFETY_RADIUS) / math.sin(aim)


def pass_offset(obstacle_w):
  """정중앙에 놓인 장애물을 지나가려면 경로에서 옆으로 얼마나 가야 하나."""
  return SAFETY_RADIUS + obstacle_w / 2
# 캘리브 구간 — heading_init 이 /teleop/cmd_vel 로 몰고, 먹스에서 teleop 이
#   자율보다 우선이라 감속·회피 분기를 아예 안 탄다. 여기 둔 미션은 안 돈다.
CALIB_END = 10.0
# 코스 길이 대조 허용 오차 (시퀀서 기본값과 같은 개념)
LEN_TOL = 0.05
# 차량 기하 — 조향 여유 계산용
WHEELBASE, MAX_STEER_DEG = 0.785, 18.0
# 플래너 조준창 = atan((트랙폭/2 − 차폭/2 − 안전여유) / 조준거리)
#   기본값 track_width 2.4 · lookahead 1.5 → ±20.6°
AIM_WINDOW_DEG = math.degrees(math.atan((1.2 - 0.3875 - 0.25) / 1.5))
# 회피가 쓸 수 있는 각의 최소 여유 [도]. 경로조향이 이미 조준창을 거의 다
#   먹으면 플래너는 '피할 방향' 을 표현할 자리가 없다.
AIM_MARGIN_MIN = 6.0


def curvature_radius(P):
  """세 점의 외접원 반지름 [m]. arm_window_plot.py 와 **같은 식**이다.

  학교 wp 로 검증: s25→4.7 · s33→6.4 · s35→8.1 · s37→11.0 · s38→13.4 ·
  s40→21.8 — 인계문서 표를 전부 재현한다.
  """
  kap = np.zeros(len(P))
  for i in range(2, len(P) - 2):
    p0, p1, p2 = P[i - 2], P[i], P[i + 2]
    ar = abs((p1[0] - p0[0]) * (p2[1] - p0[1])
             - (p2[0] - p0[0]) * (p1[1] - p0[1])) / 2
    den = (np.hypot(*(p1 - p0)) * np.hypot(*(p2 - p1))
           * np.hypot(*(p2 - p0)))
    kap[i] = (4 * ar / den) if den > 1e-9 else 0.0
  return np.where(kap > 1e-6, 1.0 / np.maximum(kap, 1e-6), 999.0)


def load_course(plan_path, wp_override):
  d = yaml.safe_load(open(plan_path))
  course = dict(d.get('course') or {})
  wp = wp_override or course.get('waypoints')
  if not wp:
    sys.exit('❌ 계획에 course.waypoints 가 없고 --wp 도 안 줬다. '
             '시퀀서는 이걸로 코스를 대조하므로 반드시 채울 것.')
  if not os.path.isabs(wp):
    wp = os.path.join(WS, wp)
  if not os.path.exists(wp):
    sys.exit(f'❌ 웨이포인트 파일이 없다: {wp}')
  w = yaml.safe_load(open(wp))['waypoints']
  P = np.array([[p['x'], p['y']] for p in w])
  s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
  return d, course, wp, P, s, curvature_radius(P)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('plan')
  ap.add_argument('--wp', default=None, help='웨이포인트 파일 (계획 것을 덮어씀)')
  ap.add_argument('--obstacle', action='append', default=[],
                  metavar='미션=s', help='장애물 위치 s [m]. 예: lidar_avoid=44')
  ap.add_argument('--include-disabled', action='store_true',
                  help='enabled=false 인 미션도 검사한다 (자리표시자를 채우는 중)')
  ap.add_argument('--obstacle-width', type=float, default=None, metavar='m',
                  help='장애물 폭 [m]. BLOCKED 거리와 필요 이탈을 다시 잰다. '
                       '용인 브룬 차체: 세로 0.6~0.7 · 가로 1.1~1.25')
  ap.add_argument('--curve-avoid', action='store_true',
                  help='커브 안에서 회피해야 하는 코스(예: 용인 S자). 곡률 실패를 '
                       '경고로 낮추는 대신 **이탈 상한 설정을 요구**한다')
  ap.add_argument('--suggest', default=None, metavar='s',
                  help='장애물이 s [m] 에 있을 때 **안전한 arm 지점**을 역산한다')
  a = ap.parse_args()

  obstacles = {}
  for item in a.obstacle:
    if '=' not in item:
      sys.exit(f'❌ --obstacle 형식은 미션=s 다: {item}')
    k, v = item.split('=', 1)
    obstacles[k.strip()] = float(v)

  lead_min, blocked_r = LEAD_MIN, BLOCKED_R
  if a.obstacle_width:
    blocked_r = blocked_radius(a.obstacle_width)
    lead_min = DETECT_TRIGGER + blocked_r

  d, course, wp, P, s, R = load_course(a.plan, a.wp)
  L = float(s[-1])
  verdicts = []

  def say(grade, item, msg):
    mark = {'통과': '✅', '경고': '⚠ ', '실패': '❌', '미검사': '·　'}[grade]
    verdicts.append((grade, item, msg))
    print(f'  {mark} {item:<16} {msg}')

  def at(sv):
    """s [m] 위치의 웨이포인트 인덱스."""
    return int(np.argmin(np.abs(s - sv)))

  print(f'\n계획   {a.plan}')
  print(f'코스   {os.path.relpath(wp, WS)}  ·  {len(P)}점 · {L:.2f} m')
  print(f'현장   {course.get("site", "(site 미기재)")}\n')

  # ── 1. course 블록 ───────────────────────────────────────────────────
  print('[코스 대조] 시퀀서가 런타임에 거부하는 항목들')
  want = course.get('path_length_m')
  if want is None:
    say('실패', 'path_length_m',
        '없다 — 시퀀서가 코스를 대조하지 못하고 경고만 낸다. 채울 것')
  elif abs(float(want) - L) / max(L, 1e-6) > LEN_TOL:
    say('실패', 'path_length_m',
        f'계획 {float(want):.1f}m vs 실제 {L:.2f}m — '
        f'{100 * abs(float(want) - L) / L:.1f}% 차이. 시퀀서가 거부한다')
  else:
    say('통과', 'path_length_m', f'계획 {float(want):.1f}m · 실제 {L:.2f}m')
  if not course.get('site'):
    say('경고', 'site', '비어 있다 — 다른 현장 계획을 쓴 것을 사람이 못 알아챈다')

  # ── 2. 구간별 검사 ───────────────────────────────────────────────────
  miss = [m for m in (d.get('missions') or [])
          if m.get('enabled', True) or a.include_disabled]
  if not miss:
    sys.exit('❌ enabled 된 미션이 하나도 없다.\n'
             '   자리표시자를 채우는 중이면 --include-disabled 를 줄 것.')
  spans = {}
  for m in miss:
    name = m.get('name', '(이름없음)')
    t = m.get('trigger') or {}
    if str(t.get('type', 'course_s')) != 'course_s':
      continue
    e, x = t.get('s_enter'), t.get('s_exit')
    if e is None or x is None:
      say('실패', name, 's_enter/s_exit 가 없다')
      continue
    spans[name] = (float(e), float(x), m)

  for name, (e, x, m) in spans.items():
    print(f'\n[{name}]  s {e:.1f} → {x:.1f}  ({x - e:.1f}m)')
    if e >= x:
      say('실패', '구간', f's_enter {e:.1f} ≥ s_exit {x:.1f}')
      continue
    if e < 0 or x > L:
      say('실패', '구간', f'코스 밖 — 경로는 0~{L:.2f}m 다')
    if e < CALIB_END:
      say('실패', '캘리브 구간',
          f's_enter {e:.1f} < {CALIB_END:.0f} — 여기선 teleop 이 먹스 우선이라 '
          f'미션이 아예 안 돈다')

    # 곡률 — 조향을 override 하는 미션에만 엄격하다
    i0, i1 = at(e), at(x)
    seg = R[i0:i1 + 1]
    r_arm = float(R[i0])
    r_min = float(seg.min()) if len(seg) else float('nan')
    overrides = (name == 'lidar_avoid') or bool(m.get('overrides_steer'))
    if overrides:
      dev = OVERRIDE_DIST ** 2 / (2 * max(r_min, 1e-6))
      grade = ('실패' if r_arm < ARM_R_FAIL
               else '경고' if r_arm < ARM_R_WARN else '통과')
      # ★ 2026-09-17 — 용인은 **S자 코스 안에서 회피해야 한다**(대회 요구사항).
      #   '커브에서 arm 금지' 는 학교 트랙 경험에서 나온 발견적 규칙이고,
      #   그 트랙에서는 회피를 arm 할 이유가 커브에 없었다. 대회에서는 있다.
      #   그래서 --curve-avoid 면 곡률 실패를 경고로 낮추되, 대신 이탈 상한
      #   (avoid_max_lateral_m · avoid_max_heading_deg)을 **요구**한다.
      #   근거: 먹스의 cap_reason() 은 예측이 아니라 **실제 경로 이탈**을 재서
      #   초과하면 회피를 버리고 경로조향으로 복귀한다(히스테리시스 포함).
      #   커브에서 arm 할 때 기댈 곳은 곡률 문턱이 아니라 이 장치다.
      if a.curve_avoid and grade == '실패':
        grade = '경고'
      say(grade, 'arm 지점 곡률',
          f'R={r_arm:.1f}m (실패<{ARM_R_FAIL:.0f} · 경고<{ARM_R_WARN:.0f})'
          + ('  ※ --curve-avoid 로 실패→경고' if a.curve_avoid
             and r_arm < ARM_R_FAIL else ''))

      # 조향 여유 — 커브에서 회피할 때 **진짜 물리 한계**는 이쪽이다.
      #   경로조향 δ = atan(축거/R) 이 먼저 조향을 먹고, 남는 만큼만 회피에 쓴다.
      d_path = np.degrees(np.arctan(WHEELBASE / np.maximum(seg, 1e-6)))
      d_max = float(d_path.max())
      aim_left = AIM_WINDOW_DEG - d_max
      steer_left = MAX_STEER_DEG - d_max
      g3 = ('실패' if aim_left < 0 or steer_left < 0
            else '경고' if aim_left < AIM_MARGIN_MIN else '통과')
      say(g3, '조향 여유',
          f'경로조향이 최대 {d_max:.1f}° 를 먼저 쓴다 → '
          f'조준창(±{AIM_WINDOW_DEG:.1f}°) 여유 {aim_left:.1f}° · '
          f'조향상한(±{MAX_STEER_DEG:.0f}°) 여유 {steer_left:.1f}°')
      if a.obstacle_width:
        off = pass_offset(a.obstacle_width)
        g4 = '실패' if off > 1.2 else '경고' if off > 0.85 else '통과'
        say(g4, '통과 이탈량',
            f'폭 {a.obstacle_width:.2f}m 가 경로 정중앙에 놓이면 옆으로 '
            f'{off:.2f}m 가야 지나간다 (트랙창 1.20m · 권장 상한 0.85m). '
            + ('트랙창 밖이라 갭이 없다 → BLOCKED' if off > 1.2 else
               f'avoid_max_lateral_m 을 {off + 0.15:.2f} 이상으로 올릴 것 — '
               f'0.85 로 두면 필요한 순간에 회피를 버린다'))
      if a.curve_avoid:
        say('미검사', '이탈 상한',
            'avoid_max_lateral_m · avoid_max_heading_deg 를 **반드시 켤 것** — '
            '커브에서 arm 하면 이게 유일한 보호장치다. 값은 장애물 없는 랩의 '
            '실측 이탈(p95)에 여유를 더해 정할 것')
      g2 = ('실패' if dev > DEV_WARN else
            '경고' if dev > DEV_WARN / 2 else '통과')
      say(g2, '구간 최악 곡률',
          f'R최소={r_min:.1f}m → 경로조향 {OVERRIDE_DIST:.0f}m 포기 시 '
          f'이탈 {dev:.2f}m (경고선 {DEV_WARN:.2f})')
    else:
      say('통과', '곡률', f'arm R={r_arm:.1f}m · 구간 최소 R={r_min:.1f}m '
                          f'(조향 override 안 함 — 참고용)')

    # arm 여유 — 장애물 위치를 줘야만 검사할 수 있다
    if name in obstacles:
      so = obstacles[name]
      lead = so - e
      if not (e <= so <= x):
        say('실패', '장애물 위치',
            f's={so:.1f} 이 구간 {e:.1f}~{x:.1f} 밖이다 — 미션이 안 돈다')
      elif lead < lead_min:
        say('실패', 'arm 여유',
            f'{lead:.1f}m < {lead_min:.1f}m — 감지({DETECT_TRIGGER:.0f}m)가 '
            f'시작될 때 아직 arm 전이거나, BLOCKED({blocked_r:.1f}m) 안에서 '
            f'arm 된다. BLOCKED 는 조향을 안 낸다(실측: 그대로 박았다)')
      else:
        say('통과', 'arm 여유',
            f'{lead:.1f}m ≥ {lead_min:.1f}m (감지 {DETECT_TRIGGER:.0f} + '
            f'BLOCKED {blocked_r:.1f})')
    else:
      say('미검사', 'arm 여유',
          f'장애물 s 를 모른다 — `--obstacle {name}=<s>` 로 주면 검사한다')

  # ── 3. 구간 겹침 ─────────────────────────────────────────────────────
  print('\n[구간 겹침] 동시에 arm 되면 서로를 끈다')
  names = list(spans)
  found = False
  for i, n1 in enumerate(names):
    for n2 in names[i + 1:]:
      e1, x1, m1 = spans[n1]
      e2, x2, m2 = spans[n2]
      lo, hi = max(e1, e2), min(x1, x2)
      if lo >= hi:
        continue
      found = True
      inh = (n2 in (m1.get('inhibits') or [])
             or n1 in (m2.get('inhibits') or []))
      if inh:
        say('실패', f'{n1}↔{n2}',
            f's {lo:.1f}~{hi:.1f} ({hi - lo:.1f}m) 겹친다 — 한쪽이 inhibits 로 '
            f'다른 쪽을 끈다. 실측: 돌발이 회피를 꺼 의자를 못 피했다')
      else:
        say('경고', f'{n1}↔{n2}',
            f's {lo:.1f}~{hi:.1f} ({hi - lo:.1f}m) 겹친다 — '
            f'/stop_line_distance 를 공유하면 서로 덮어쓴다')
  if not found:
    gaps = []
    srt = sorted(names, key=lambda n: spans[n][0])
    for n1, n2 in zip(srt, srt[1:]):
      gaps.append(f'{n1}→{n2} {spans[n2][0] - spans[n1][1]:+.1f}m')
    say('통과', '겹침 없음', ' · '.join(gaps) if gaps else '미션 1개')

  # ── 4. arm 지점 역산 (--suggest) ─────────────────────────────────────
  if a.suggest is not None:
    so = float(a.suggest)
    print(f'\n[arm 지점 역산] 장애물이 s={so:.1f}m 에 있다고 할 때')
    if not (0 <= so <= L):
      say('실패', 'suggest', f's={so:.1f} 이 코스(0~{L:.2f}) 밖이다')
    else:
      # ★ arm 지점 하나만 보면 안 된다. AVOID 는 **armed 창 어디서든** 뜰 수
      #   있고, 뜨는 순간 먹스가 경로조향을 버린다. 그래서 창 [s_enter, 장애물]
      #   **전체의 최악 곡률**로 판단한다.
      #   (처음에 arm 지점만 봤더니 학교 코스에서 s21.5 를 권했다. 그 창 안에는
      #    s25 R=4.7m 헤어핀이 들어 있다 — 실제로 거기서 벽에 박은 적이 있다.)
      #
      #   그 조건 안에서는 **가장 늦은** 지점이 최선이다. 늦을수록 경로조향을
      #   버린 채 달리는 거리가 짧다.
      hi = so - lead_min                       # 이보다 늦으면 BLOCKED 를 못 피한다
      lo = max(0.0, so - 30.0)                 # 30m 앞까지만 본다
      i_obs = at(so)
      cand = []
      for i in range(len(s)):
        if not (lo <= s[i] <= hi):
          continue
        win = R[i:i_obs + 1]
        if len(win):
          cand.append((float(s[i]), float(win.min())))
      ok = [z for z in cand if z[1] >= ARM_R_FAIL]
      print(f'  · arm 은 s ≤ {hi:.1f} 여야 한다 '
            f'(감지 {DETECT_TRIGGER:.0f} + BLOCKED {blocked_r:.1f} = {lead_min:.1f}m)')
      print(f'  · 판정은 arm 지점이 아니라 **창 [s_enter, {so:.1f}] 전체의 '
            f'최악 곡률**로 한다')
      if not cand:
        say('실패', '역산', f'장애물이 코스 시작에서 {so:.1f}m — 여유가 없다')
      elif not ok:
        best = max(cand, key=lambda z: z[1])
        say('실패', '역산',
            f'{lo:.0f}~{hi:.1f} 어디서 arm 해도 창 안에 R<{ARM_R_FAIL:.0f}m 커브가 '
            f'들어온다 (최선이 s{best[0]:.1f} 창최악 R={best[1]:.1f}m). '
            f'여기는 회피 장애물을 두면 안 되는 위치다')
      else:
        sv, rmin = ok[-1]                       # 가장 늦은 = 이탈 최소
        dev = OVERRIDE_DIST ** 2 / (2 * max(rmin, 1e-6))
        grade = '통과' if rmin >= ARM_R_WARN else '경고'
        say(grade, '권장 arm',
            f's_enter = {sv:.1f}  (창최악 R={rmin:.1f}m · 여유 {so - sv:.1f}m · '
            f'{OVERRIDE_DIST:.0f}m 포기 시 이탈 {dev:.2f}m)')
        print(f'  · 쓸 수 있는 구간: s {ok[0][0]:.1f} ~ {ok[-1][0]:.1f} '
              f'({len(ok)}점). 늦을수록 이탈이 작아 가장 늦은 곳을 권한다')
        print(f'  · 계획에 적을 값:  s_enter: {sv:.1f}')

  # ── 판정 ─────────────────────────────────────────────────────────────
  nf = sum(1 for g, _, _ in verdicts if g == '실패')
  nw = sum(1 for g, _, _ in verdicts if g == '경고')
  nu = sum(1 for g, _, _ in verdicts if g == '미검사')
  print()
  if nu:
    print(f'·　 미검사 {nu}건 — 장애물 s 를 주지 않으면 arm 여유는 못 본다')
  if nf:
    print(f'❌ 실패 {nf}건 · 경고 {nw}건 — 이 계획으로 달리면 안 된다')
    return 1
  if nw:
    print(f'⚠  경고 {nw}건 — 왜 괜찮은지 설명할 수 있어야 한다')
    return 0
  print('✅ 전부 통과')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

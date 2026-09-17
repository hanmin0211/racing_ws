#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_review.py — 주행 기록 하나를 **같은 기준으로** 판정한다.

★ 왜 이게 필요한가 (2026-09-16)
  런을 돌 때마다 사람이 CSV 를 열어 눈으로 보면, 볼 때마다 다른 것을 본다.
  그날 밤 실제로 그랬다 — 0.2초 간격으로 솎아 본 표를 근거로 '먹스가 장애물
  쪽으로 조향했다' 고 판단했는데, 0.05초 원본을 보니 **플래너가 좌우를 초당
  10번 뒤집고 있었다.** 원인 진단이 통째로 틀렸다.

  그래서 판정 기준을 코드로 못 박는다. 런을 돌면 이걸 돌리고, **출력의 마지막
  판정표만** 본다.

사용:
  python3 tools/drive_review.py /tmp/drive_0337.csv
  python3 tools/drive_review.py /tmp/drive_0337.csv --waypoints <wp.yaml>
  python3 tools/drive_review.py /tmp/drive_0337.csv --detail   # 회피구간 전 프레임

판정 기준의 근거는 각 항목 옆에 적어 두었다. 기준을 바꿀 거면 **왜 바꾸는지**
같이 적을 것 — 안 그러면 다음 사람이 숫자만 보고 통과시킨다.
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter

import numpy as np

DEF_WP = ('/home/han/racing_ws/config/chungju_school/'
          'wp_school_track_0.5.yaml')

# ── 판정 문턱 ────────────────────────────────────────────────────────────
# 이탈: 0.90m/25° 를 넘으면 조향 최대치(18°·~25°/s)로도 못 돌아온다(drive_0312
#   실측). 성공한 회피(drive_0337)의 최댓값이 0.72m/17.6° 이므로 그 사이에 둔다.
DEV_WARN, DEV_FAIL = 0.72, 0.90
HEAD_WARN, HEAD_FAIL = 18.0, 25.0
# 조향 지연 0.35s · 실제 각속도 ~25°/s (drive_0337 상호상관 실측, 상관 0.92)
STEER_LAG_S, STEER_RATE_MAX = 0.35, 25.0
# 라이다 마운트: 트랙창 안 점이 **지도고정**이면 진짜 장애물, **차체고정**이면
#   지면 타격이다. 차체고정 쪽이 더 몰려 있으면 마운트를 의심한다.
MOUNT_WARN_RATIO = 1.0
# 공급전압: 3000mV 밑은 모터가 잠기기 시작하는 영역(2026-09-13 실측)
VCC_LOW = 3000
# 대회 규정: 1분 이상 정지 = 탈락
STOP_FAIL_S = 60.0


def _cmd_rate(tt, cc):
  """명령 조향의 변화율 [°/s] — **명령이 실제로 바뀐 간격**으로 나눈다.

  기록 주기로 나누면 안 된다. 기록기는 '마지막으로 받은 명령' 을 20Hz 로
  베껴 쓸 뿐이라, 명령이 10Hz 로 바뀌면 계단 하나가 기록상 0.05s 안에
  일어난 것처럼 보인다(= 2배 부풀음). 액추에이터에게 주어진 시간은 다음
  명령이 올 때까지이므로, 나눌 값은 **변화 사이의 실제 간격**이다.

  주기를 상수로 박지 않는 이유: 같은 런에서도 회피창 안(10Hz)과 밖(20Hz)이
  다르다. 그래서 데이터에서 재고, 잰 주기를 같이 돌려준다.

  반환: (최대, 95백분위, 최댓값 시각, 갱신주기 중앙값, 변화 횟수)
  """
  m = np.isfinite(cc)
  v, ts = cc[m], tt[m]
  if len(v) < 3:
    return 0.0, 0.0, float('nan'), float('nan'), 0
  ch = np.flatnonzero(np.abs(np.diff(v)) > 1e-9)   # 값이 바뀐 지점
  if len(ch) < 2:
    return 0.0, 0.0, float('nan'), float('nan'), int(len(ch))
  t_ch, v_ch = ts[ch + 1], v[ch + 1]               # 바뀐 시각과 바뀐 값
  gap = np.diff(t_ch)
  good = gap > 1e-6
  if not good.any():
    return 0.0, 0.0, float('nan'), float('nan'), int(len(ch))
  r = np.abs(np.diff(v_ch))[good] / gap[good]
  t_r = t_ch[1:][good]
  k = int(np.argmax(r))
  return (float(r.max()), float(np.percentile(r, 95)), float(t_r[k]),
          float(np.median(gap[good])), int(len(ch)))


def load_drive(path):
  rows = list(csv.DictReader(open(path)))
  if not rows:
    sys.exit(f'빈 파일: {path}')

  def col(k):
    out = []
    for r in rows:
      v = r.get(k, '')
      out.append(float(v) if v not in ('', 'nan', 'None', None) else np.nan)
    return np.array(out)

  d = {k: col(k) for k in ('t', 'x', 'y', 'yaw_deg', 'v', 'avoid_steer_deg',
                           'obstacle_m', 'cmd_v', 'cmd_steer_deg',
                           'steer_actual_deg', 'vcc_mv')}
  d['mode'] = [r.get('mode', '') for r in rows]
  return d


def load_path(path):
  import yaml
  d = yaml.safe_load(open(path))
  pts = [(float(p['x']), float(p['y'])) for p in (d.get('waypoints') or [])]
  return np.array(pts) if len(pts) >= 2 else None


def project_all(P, X, Y, yaw):
  """각 샘플의 (호길이 s, 부호있는 이탈[좌+], 경로헤딩 오차[도])."""
  A, B = P[:-1], P[1:]
  AB = B - A
  L2 = (AB ** 2).sum(1)
  L2[L2 == 0] = 1e-9
  S = np.concatenate([[0.0], np.cumsum(np.hypot(AB[:, 0], AB[:, 1]))])
  sv = np.full(len(X), np.nan)
  dv = np.full(len(X), np.nan)
  he = np.full(len(X), np.nan)
  for i in range(len(X)):
    if not np.isfinite(X[i]) or not np.isfinite(Y[i]):
      continue
    tt = (((X[i] - A[:, 0]) * AB[:, 0]
           + (Y[i] - A[:, 1]) * AB[:, 1]) / L2).clip(0, 1)
    Q = A + tt[:, None] * AB
    dd = np.hypot(Q[:, 0] - X[i], Q[:, 1] - Y[i])
    k = int(np.argmin(dd))
    th = math.atan2(AB[k, 1], AB[k, 0])
    sv[i] = S[k] + tt[k] * math.hypot(*AB[k])
    dv[i] = -math.sin(th) * (X[i] - Q[k, 0]) + math.cos(th) * (Y[i] - Q[k, 1])
    if np.isfinite(yaw[i]):
      he[i] = (yaw[i] - math.degrees(th) + 180.0) % 360.0 - 180.0
  return sv, dv, he, float(S[-1])


def base_mode(m):
  return m.split('|')[0].split('(')[0] if m else '-'


def is_armed(m):
  """회피 조향 override 가 켜져 있던 프레임인가."""
  return bool(m) and '조향OFF' not in m and base_mode(m) != '-'


def runs_of(mask):
  """True 가 연속인 구간 [(start, end_inclusive)]."""
  out, i, n = [], 0, len(mask)
  while i < n:
    if mask[i]:
      j = i
      while j < n and mask[j]:
        j += 1
      out.append((i, j - 1))
      i = j
    else:
      i += 1
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('csv')
  ap.add_argument('--waypoints', default=DEF_WP)
  ap.add_argument('--section', action='append', default=[],
                  metavar='이름=s0:s1',
                  help='구간별 이탈 통계. 예: --section S자=176:250 '
                       '(반복 가능). 장애물 없는 랩으로 재면 이탈 상한의 근거가 된다')
  ap.add_argument('--obstacle-width', type=float, default=None, metavar='m',
                  help='장애물 폭 [m]. --section 과 같이 주면 '
                       'avoid_max_lateral_m 권장값을 낸다')
  ap.add_argument('--lane-width', type=float, default=None, metavar='m',
                  help='차선 폭 [m] (현장 실측). 회피가 차선 안에 드는지 본다')
  ap.add_argument('--detail', action='store_true',
                  help='회피 구간을 전 프레임(0.05s) 찍는다 — 솎아 보면 오판한다')
  args = ap.parse_args()

  d = load_drive(args.csv)
  t, X, Y = d['t'], d['x'], d['y']
  v, yaw, mode = d['v'], d['yaw_deg'], d['mode']
  av, cs, sa = d['avoid_steer_deg'], d['cmd_steer_deg'], d['steer_actual_deg']
  obs, vcc = d['obstacle_m'], d['vcc_mv']
  n = len(t)
  dt = float(np.median(np.diff(t))) if n > 1 else 0.05
  verdict = []          # (등급, 항목, 한 줄)

  def say(grade, item, line):
    verdict.append((grade, item, line))

  print(f'\n══ {os.path.basename(args.csv)} ══')
  print(f'  {n}행 · t {t[0]:.1f}~{t[-1]:.1f}s ({t[-1] - t[0]:.0f}s) · dt {dt:.3f}s')

  # ── 데이터 품질: odom twist 스파이크 ────────────────────────────────
  # EKF 가 odom 프레임을 늦게 받으면 dt≈0 으로 나누며 튄다. 위치는 멀쩡하다.
  # 이걸 안 걸러내면 '평균속도' 같은 숫자가 통째로 오염된다.
  spike = np.isfinite(v) & (np.abs(v) > 3.0)
  vg = v[np.isfinite(v) & ~spike]
  print(f'\n[데이터] v 스파이크 {int(spike.sum())}개'
        + (f' (최대 {np.nanmax(np.abs(v)):.1f} m/s — 제외하고 계산)'
           if spike.any() else ''))
  if spike.any():
    say('경고', 'odom twist',
        f'{int(spike.sum())}개 스파이크(최대 {np.nanmax(np.abs(v)):.0f}m/s). '
        'local_pure_pursuit 의 max_plausible_speed 가 막아준다')
  if len(vg):
    print(f'         속도 평균 {vg.mean():.2f} · 최대 {vg.max():.2f} m/s')

  # ── 구(舊) 기록 단위버그 ─────────────────────────────────────────────
  # 2026-09-16 이전 drive_record.py 는 `math.degrees(angular.z)` 를 썼는데
  # angular.z 는 이미 **도(°)** 다(vehicle_cmd_mux_node.py:18). 그래서 그
  # 전에 찍힌 CSV 의 cmd_steer_deg 는 전부 57.2958 배로 부풀어 있다.
  # 조향은 소프트웨어에서 ±18° 로 클램프되므로 그보다 크면 물리적으로 불가능.
  # 실측: drive_0020 = 814° (=14.2×57.3) · 수정 후 런 전부 ≤ 18.0°
  cs_max = float(np.nanmax(np.abs(cs))) if np.isfinite(cs).any() else 0.0
  if cs_max > 40.0:
    say('실패', '기록 단위',
        f'cmd_steer 최대 {cs_max:.0f}° — 물리적으로 불가능(클램프 18°). '
        f'2026-09-16 이전 기록이다. 조향 관련 수치는 전부 57.2958 로 '
        f'나눠야 하고, 이 파일의 조향 판정은 믿을 수 없다')

  # ── 전압 ────────────────────────────────────────────────────────────
  vv = vcc[np.isfinite(vcc) & (vcc > 500)]
  if len(vv):
    low = float((vv < VCC_LOW).mean())
    print(f'\n[전압] min {vv.min():.0f} · p5 {np.percentile(vv, 5):.0f} · '
          f'중앙 {np.median(vv):.0f} mV · {VCC_LOW}mV 미만 {100 * low:.1f}%')
    if vv.min() < 2600:
      say('경고', '전압',
          f'최저 {vv.min():.0f}mV — 모터가 잠길 수 있다. 배터리/분리 확인')
    else:
      say('통과', '전압', f'최저 {vv.min():.0f}mV')

  # ── 정지 ────────────────────────────────────────────────────────────
  still = np.isfinite(v) & (np.abs(v) < 0.05)
  longest = 0.0
  print('\n[정지] 2초 이상')
  for a, b in runs_of(still):
    dur = t[b] - t[a]
    if dur < 2.0:
      continue
    longest = max(longest, dur)
    print(f'   t{t[a]:6.1f}~{t[b]:6.1f} ({dur:5.1f}s) ({X[a]:.1f},{Y[a]:.1f})')
  if longest >= STOP_FAIL_S:
    say('실패', '정지', f'최장 {longest:.0f}s — 1분 이상 정지는 탈락이다')
  else:
    say('통과', '정지', f'최장 {longest:.0f}s (탈락선 {STOP_FAIL_S:.0f}s)')

  # ── 모드 ────────────────────────────────────────────────────────────
  bm = [base_mode(m) for m in mode]
  armed = np.array([is_armed(m) for m in mode])
  print('\n[모드] 전체')
  for k, c in Counter(bm).most_common():
    print(f'   {k:9s} {c:5d}  {100 * c / n:5.1f}%')

  # ── 경로 대조 ────────────────────────────────────────────────────────
  sv = dv = he = None
  P = load_path(args.waypoints) if os.path.exists(args.waypoints) else None
  if P is None:
    print(f'\n[경로] {args.waypoints} 없음 — 이탈 판정 건너뜀')
    say('미확인', '이탈', '경로 파일이 없어 판정 못 함')
  else:
    sv, dv, he, total = project_all(P, X, Y, yaw)
    ok = np.isfinite(dv)
    print(f'\n[경로] {args.waypoints.split("/")[-1]} · {len(P)}점 {total:.1f}m')
    print(f'   이탈 평균 {np.abs(dv[ok]).mean():.2f} · '
          f'p95 {np.percentile(np.abs(dv[ok]), 95):.2f} · '
          f'최대 {np.abs(dv[ok]).max():.2f} m')
    # 랩: s>1 에서 움직이기 시작해 경로 끝까지
    moving = np.isfinite(v) & (np.abs(v) > 0.2) & ~spike
    go = np.where(moving & np.isfinite(sv) & (sv > 1.0))[0]
    end = np.where(np.isfinite(sv) & (sv > total - 0.5))[0]
    if len(go) and len(end) and t[end[0]] > t[go[0]]:
      lap = t[end[0]] - t[go[0]]
      print(f'   랩: t{t[go[0]]:.1f} → t{t[end[0]]:.1f} = {lap:.1f}s '
            f'({lap / 60:.2f}분) · 평균 {total / lap:.2f} m/s')
      say('통과' if lap < 480 else '경고', '랩타임',
          f'{lap:.0f}s (예산 480s)')
    else:
      say('미확인', '랩타임', '경로 끝(s=%.0f)에 도달한 기록이 없다' % total)

  # ── 구간별 이탈 (--section) ──────────────────────────────────────────
  # ★ 왜 (2026-09-17) — 용인 S자는 **커브 안에서** 회피해야 한다. 거기서
  #   기댈 보호장치는 먹스의 이탈 상한뿐인데, 그 값을 학교 트랙 숫자(0.85)로
  #   두면 안 된다. 커브는 추종오차 자체가 크고, 브룬 장애물을 지나가려면
  #   0.96m 를 비켜야 한다. **장애물 없는 랩을 재서** 거기에 통과 필요량을
  #   더하는 것이 유일하게 근거 있는 방법이다.
  if args.section:
    if sv is None:
      say('실패', '구간 통계', '경로 파일이 없어 구간을 못 나눈다')
    else:
      print('\n[구간별 이탈] ⚠ **장애물 없는 랩**으로 재야 의미가 있다')
      VEH_HALF, SAFE_R = 0.3875, 0.6375
      for spec in args.section:
        nm, rng = spec.split('=', 1) if '=' in spec else ('구간', spec)
        a0, a1 = (float(z) for z in rng.split(':'))
        m = np.isfinite(sv) & np.isfinite(dv) & (sv >= a0) & (sv <= a1)
        if int(m.sum()) < 5:
          say('실패', nm, f's {a0:.0f}~{a1:.0f} 에 표본이 {int(m.sum())}개뿐 — '
                          f'이 구간을 안 지났거나 측위가 끊겼다')
          continue
        ad = np.abs(dv[m])
        p95 = float(np.percentile(ad, 95))
        hh = he[m]
        ah = np.abs(hh[np.isfinite(hh)])
        vv2 = v[m][np.isfinite(v[m])]
        print(f'   {nm:<10} s{a0:.0f}~{a1:.0f} · 표본 {int(m.sum())} · '
              f'속도 {vv2.mean() if len(vv2) else float("nan"):.2f} m/s')
        print(f'   {"":10} 이탈 평균 {ad.mean():.2f} · p95 {p95:.2f} · '
              f'최대 {ad.max():.2f} m')
        if len(ah):
          print(f'   {"":10} 헤딩 p95 {np.percentile(ah, 95):.1f} · '
                f'최대 {ah.max():.1f}°')

        if args.obstacle_width:
          need = SAFE_R + args.obstacle_width / 2
          cap = p95 + need + 0.15
          print(f'   {"":10} → 통과 필요 {need:.2f} + 기준 p95 {p95:.2f} '
                f'+ 여유 0.15 = **avoid_max_lateral_m {cap:.2f}**')
          if cap <= 0.85:
            say('통과', f'{nm} 이탈상한', f'{cap:.2f}m — 권장 0.85 안에 든다')
          else:
            say('경고', f'{nm} 이탈상한',
                f'{cap:.2f}m 가 필요하다. 학교에서 쓰던 0.85 로 두면 '
                f'**필요한 순간에 회피를 버린다**')
          if args.lane_width:
            outer = cap + VEH_HALF
            half = args.lane_width / 2.0
            g = ('실패' if outer > half
                 else '경고' if outer > half - 0.15 else '통과')
            say(g, f'{nm} 차선 여유',
                f'상한 {cap:.2f} + 차 반폭 {VEH_HALF:.2f} = {outer:.2f}m 가 '
                f'차선 반폭 {half:.2f}m 안에 들어야 한다 '
                f'(여유 {half - outer:+.2f}m)')
          else:
            say('미확인', f'{nm} 차선 여유',
                '차선 폭을 모른다 — `--lane-width <m>` 로 주면 검사한다')

  # ── arm(회피) 구간 ───────────────────────────────────────────────────
  print('\n[회피] arm 구간')
  segs = [(a, b) for a, b in runs_of(armed) if b - a >= 3]
  if not segs:
    print('   arm 된 구간이 없다 (--missions 를 빼고 달렸나?)')
    say('미확인', '회피', 'arm 구간이 없다')
  for a, b in segs:
    cnt = Counter(bm[a:b + 1])
    tot = b - a + 1
    loc = (f's{sv[a]:.1f}~{sv[b]:.1f}' if sv is not None
           and np.isfinite(sv[a]) else f'({X[a]:.0f},{Y[a]:.0f})')
    print(f'   t{t[a]:.1f}~{t[b]:.1f} ({t[b] - t[a]:.1f}s) {loc}')
    print('     모드 ' + ' '.join(
        f'{k} {100 * c / tot:.0f}%' for k, c in cnt.most_common()))
    blocked = 100 * cnt.get('BLOCKED', 0) / tot
    if blocked > 40:
      say('실패', '회피 BLOCKED',
          f'{blocked:.0f}% — 갭을 못 찾는다. 라이다 마운트부터 볼 것 '
          '(tools/lidar_mount_check.py)')
    elif blocked > 10:
      say('경고', '회피 BLOCKED', f'{blocked:.0f}%')
    else:
      say('통과', '회피 BLOCKED', f'{blocked:.0f}%')

    if dv is not None and np.isfinite(dv[a:b + 1]).any():
      mx = np.nanmax(np.abs(dv[a:b + 1]))
      mh = np.nanmax(np.abs(he[a:b + 1]))
      print(f'     최대이탈 {mx:.2f}m · 최대 헤딩오차 {mh:.1f}°')
      g = ('실패' if mx > DEV_FAIL else '경고' if mx > DEV_WARN else '통과')
      say(g, '회피 이탈',
          f'{mx:.2f}m (경고 {DEV_WARN} · 복귀불능 {DEV_FAIL})')
      g = ('실패' if mh > HEAD_FAIL else '경고' if mh > HEAD_WARN else '통과')
      say(g, '회피 헤딩', f'{mh:.1f}° (경고 {HEAD_WARN} · 복귀불능 {HEAD_FAIL})')

    # 회피 조향 안정성 — 여기가 이 도구의 핵심이다
    sub = av[a:b + 1]
    tt = t[a:b + 1]
    valid = np.isfinite(sub)
    flips, last = 0, 0
    for k in range(len(sub)):
      if not valid[k] or abs(sub[k]) < 3.0:
        continue
      s = 1 if sub[k] > 0 else -1
      if last and s != last:
        flips += 1
      last = s
    # 한 방향 체류시간
    # ★ 2026-09-17 — 전환 카운터와 **같은 데드밴드(3°)** 를 쓴다.
    #   예전엔 np.sign() 만 봤는데, 정확히 0.0 이 별도 부호로 취급돼
    #   명령이 0 근처를 맴돌기만 해도 여러 조각으로 쪼개졌다.
    #   drive_0117 에서 이것 때문에 '체류 13회 중 8회 실행 불가 ❌' 가 떴는데,
    #   그 8개는 전부 크기 0.0~0.4° 였다(= 조향을 안 하고 있었다는 뜻).
    #   데드밴드를 걸면 1개 2.80s 로, 떨림이 아예 없었다.
    DEAD = 3.0
    dwell, i = [], 0
    while i < len(sub):
      if not valid[i] or abs(sub[i]) < DEAD:
        i += 1
        continue
      s0 = np.sign(sub[i])
      j = i
      while j < len(sub) and valid[j] and (abs(sub[j]) < DEAD
                                           or np.sign(sub[j]) == s0):
        j += 1
      dwell.append(tt[j - 1] - tt[i] + dt)
      i = j
    short = [x for x in dwell if x < STEER_LAG_S]
    # 명령 변화율
    # ★ 2026-09-17 수정 — 예전엔 **기록 주기**(0.05s)로 나눴다. 명령을 만드는
    #   쪽은 그보다 느리다. 회피창 안에서 명령이 실제로 바뀌는 간격은 실측
    #   **0.100s**(라이다 10Hz — 슬루 제한이 cluster_plot 에 있고 먹스가 20Hz 로
    #   재발행할 뿐이다). 그래서 변화율이 정확히 2배로 부풀었고, 슬루 제한을
    #   **켜고 달린 런 3개(0126·0137·0152)가 전부 위반으로 찍혔다**:
    #       0137  기존 182°/s ❌  →  수정 83°/s ✅ (제한 90 이하)
    #       0312  기존 720°/s     →  수정 580°/s   (제한 꺼짐 — 양쪽 다 위반)
    #   ⚠ 일괄 ÷2 는 틀린 고침이다. 같은 CSV 라도 회피창 **밖**에서는 명령이
    #     20Hz 로 바뀐다. 그래서 주기를 가정하지 않고 **데이터에서 잰다**.
    cc = cs[a:b + 1]
    rate, rate_p95, rate_t, period, n_chg = _cmd_rate(tt, cc)
    print(f'     좌우 전환 {flips}회 · 한방향 체류 {len(dwell)}회 중 '
          f'{len(short)}회가 조향지연({STEER_LAG_S}s) 미만')
    if n_chg:
      print(f'     명령 변화율 95% {rate_p95:.0f}°/s · 최대 {rate:.0f}°/s '
            f'(t={rate_t:.2f}) · 갱신간격 중앙 {period:.3f}s · 변화 {n_chg}회 '
            f'(실제 조향 한계 ~{STEER_RATE_MAX:.0f}°/s)')
    else:
      print('     명령 변화율 — 회피구간에서 명령이 바뀌지 않았다')
    if len(dwell):
      frac = len(short) / len(dwell)
      g = '실패' if frac > 0.6 else '경고' if frac > 0.25 else '통과'
      say(g, '회피 떨림',
          f'체류 {len(dwell)}회 중 {len(short)}회({100 * frac:.0f}%)가 '
          f'실행 불가. avoid_steer_rate_deg:=90 을 볼 것')
    # ★ 2026-09-17 — 판정을 **최댓값이 아니라 95백분위**로 바꿨다.
    #   최댓값은 거의 항상 '먹스가 조향 소스를 바꾸는 한 계단' 이다(실측):
    #     0137 t76.85  CLEAR|STRAIGHT +0.9°  →  AVOID|RIGHT −8.2°   182°/s
    #     0152 t127.55 AVOID −14.7°          →  BLOCKED|STOP −2.4°  123°/s
    #   두 소스가 실제로 9° 다르면 그 계단은 피할 수 없고, 액추에이터가 한 번
    #   0.36s 뒤처질 뿐 **진동이 아니다**. 반면 이 지표가 잡아야 할 것은
    #   플래너가 좌우로 계속 뒤집는 '지속' 변화율이다.
    #   실측이 이걸 뒷받침한다 — 최댓값은 두 집단을 못 가르고, 95%는 가른다:
    #     제한 켬 (0126·0137·0152)  최대 80·182·123  |  95%  24·26·24
    #     제한 끔 (0312·0337)       최대 580·550     |  95%  169·137
    #   문턱은 데이터에 맞춘 게 아니라 물리에서 왔다 — 액추에이터가 25°/s 인데
    #   그 2배를 **지속** 요구하면 명령의 절반은 실행되지 않는다.
    if rate_p95 > 2 * STEER_RATE_MAX:
      say('경고', '조향 변화율',
          f'지속 {rate_p95:.0f}°/s(95%) 요구 — 액추에이터의 '
          f'{rate_p95 / STEER_RATE_MAX:.0f}배. avoid_steer_rate_deg:=90')
    elif n_chg:
      say('통과', '조향 변화율',
          f'지속 {rate_p95:.0f}°/s(95%) · 한계 {2 * STEER_RATE_MAX:.0f} '
          f'(최대 {rate:.0f} 는 t={rate_t:.2f} 소스 전환 계단)')

    # 도달률 (조향 지연 보정)
    L = max(int(round(STEER_LAG_S / dt)), 1)
    sel = [i for i in range(a, b + 1 - L)
           if np.isfinite(cs[i]) and np.isfinite(sa[i + L]) and abs(cs[i]) > 5]
    if sel:
      r = np.array([sa[i + L] / cs[i] for i in sel])
      print(f'     도달률(지연 {L}샘플 보정) 평균 {100 * r.mean():.0f}% · '
            f'중앙 {100 * np.median(r):.0f}% · 반대로 감 {100 * (r < 0).mean():.0f}%')

    if args.detail:
      print('\n     ── 전 프레임 (솎아 보면 오판한다) ──')
      print(f'     {"t":>6} {"mode":<20} {"회피":>6} {"명령":>6} {"실제":>6} {"obs":>6}')
      for i in range(a, b + 1):
        f = lambda z: ('  nan' if not np.isfinite(z) else f'{z:6.1f}')  # noqa: E731
        print(f'     {t[i]:6.2f} {mode[i]:<20} {f(av[i])} {f(cs[i])} '
              f'{f(sa[i])} {f(obs[i])}')

  # ── 라이다 마운트 (스캔 파일이 있으면) ────────────────────────────────
  scan = args.csv.replace('.csv', '_scan.csv')
  if os.path.exists(scan) and os.path.getsize(scan) > 100:
    sc = list(csv.DictReader(open(scan)))
    if sc:
      st = np.array([float(r['t']) for r in sc])
      sx = np.array([float(r['x']) for r in sc])
      sy = np.array([float(r['y']) for r in sc])
      idx = np.searchsorted(t, st).clip(0, n - 1)
      c = np.cos(np.radians(-yaw[idx]))
      s = np.sin(np.radians(-yaw[idx]))
      bx = c * (sx - X[idx]) - s * (sy - Y[idx])
      by = s * (sx - X[idx]) + c * (sy - Y[idx])
      win = (bx > 0.3) & (bx < 8.0) & (np.abs(by) <= 1.2)
      print(f'\n[라이다] 스캔점 {len(sc)} · 트랙창 안 {int(win.sum())} '
            f'({100 * win.mean():.1f}%)')
      if win.sum() > 50:
        body = Counter(zip(np.round(bx[win] * 2).astype(int),
                           np.round(by[win] * 2).astype(int)))
        world = Counter(zip(np.round(sx[win] * 2).astype(int),
                            np.round(sy[win] * 2).astype(int)))
        b_top = body.most_common(1)[0][1]
        w_top = world.most_common(1)[0][1]
        print(f'   창 안 최다 격자 — 차체고정 {b_top}회 vs 지도고정 {w_top}회')
        print('   (지도고정이 크면 진짜 장애물, 차체고정이 크면 지면 타격)')
        for (gx, gy), cnt in body.most_common(3):
          print(f'     차체 전방 {gx / 2:+.1f} 횡 {gy / 2:+.1f} '
                f'거리 {math.hypot(gx / 2, gy / 2):.2f}m  {cnt}회')
        if b_top > w_top * MOUNT_WARN_RATIO:
          say('경고', '라이다 마운트',
              f'창 안이 차체고정 우세({b_top} vs {w_top}) — 지면 타격 의심. '
              'tools/lidar_mount_check.py 로 잴 것')
        else:
          say('통과', '라이다 마운트',
              f'창 안이 지도고정 우세({w_top} vs {b_top}) = 진짜 장애물')

  # ── 판정표 ──────────────────────────────────────────────────────────
  print('\n' + '─' * 68)
  print('  판정')
  print('─' * 68)
  mark = {'통과': '✅', '경고': '⚠ ', '실패': '❌', '미확인': '· '}
  order = {'실패': 0, '경고': 1, '미확인': 2, '통과': 3}
  for g, item, line in sorted(verdict, key=lambda z: order[z[0]]):
    print(f'  {mark[g]} {item:14s} {line}')
  nf = sum(1 for g, _, _ in verdict if g == '실패')
  nw = sum(1 for g, _, _ in verdict if g == '경고')
  print('─' * 68)
  if nf:
    print(f'  ❌ 실패 {nf}건 · 경고 {nw}건 — 이 상태로 대회에 나가면 안 된다')
    return 1
  if nw:
    print(f'  ⚠  경고 {nw}건 — 원인을 알고 넘어갈 것')
    return 0
  print('  ✅ 전부 통과')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

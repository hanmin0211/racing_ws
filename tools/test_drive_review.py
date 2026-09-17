#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_drive_review.py — 판정 도구가 **실행 로그의 정답과 맞는지** 본다.

★ 왜 이게 필요한가 (2026-09-17)

  drive_review.py 는 런을 돌고 나서 사람이 보는 **유일한 판정표**다. 그게
  틀리면 현장 판단이 통째로 틀린다. 실제로 그랬다 — 회피 조향 변화율을
  **기록 주기**(0.05s)로 나누고 있었는데, 회피창 안에서 명령이 실제로 바뀌는
  간격은 0.100s 다. 그래서 값이 정확히 2배로 부풀었고, 슬루레이트 제한을
  **켜고 달린 런 3개가 전부 위반**으로 찍혔다.

  이 시험의 정답은 추측이 아니다 — `data/2026-09-17/run_*.log` 에
  `★ 회피 조향 슬루레이트 제한 90°/s` 줄이 있느냐 없느냐가 정답이다.

  python3 tools/test_drive_review.py
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drive_fixtures import find_run, require_run  # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REVIEW = os.path.join(WS, 'tools', 'drive_review.py')

# 정답 — 실행 로그에서 읽는다(아래 log_says_limited 로 대조한다).
#   제한을 켜고 달린 런: 지속 변화율이 액추에이터 2배(50°/s) 밑이어야 한다
#   제한 없이 달린 런  : 위반으로 찍혀야 한다
LIMITED = ('0126', '0137', '0152')
UNLIMITED = ('0312', '0337')

# ★ 검증된 기준값 [°/s] — 지속(95백분위) 변화율.
#   이 숫자들은 2026-09-17 에 실행 로그의 설정과 대조해 확인한 것이다.
#   **문턱만 보는 시험은 약하다** — 실제로 확인했다: 분모를 예전처럼 기록
#   주기(0.05s)로 되돌려 일부러 깨뜨렸더니 값이 24→48 로 2배가 됐는데도
#   문턱 50 아래라 시험이 통과했다. 그래서 값 자체를 못 박는다.
REF_P95 = {'0126': 24, '0137': 26, '0152': 24, '0312': 169, '0337': 137}
REF_TOL = 1


def log_says_limited(tag):
  """실행 로그가 '제한 켬' 이라고 말하는가. (정답의 출처)"""
  for d in ('data/2026-09-17',):
    p = os.path.join(WS, d, f'run_{tag}.log')
    if os.path.exists(p):
      return '슬루레이트 제한 9' in open(p, encoding='utf-8',
                                        errors='replace').read()
  return None


def verdict_line(out):
  """판정표의 '조향 변화율' 줄 하나를 통째로 돌려준다."""
  for ln in out.splitlines():
    if '조향 변화율' in ln:
      return ln.strip()
  return ''


def review(tag, *extra):
  csv, _ = require_run(tag)
  r = subprocess.run([sys.executable, REVIEW, csv] + list(extra),
                     capture_output=True, text=True, cwd=WS)
  return r.stdout + r.stderr


def main():
  f = []
  print('판정 도구 검증 — 실행 로그의 정답과 맞는가\n')

  print('■ 정답의 출처부터 확인한다 (추측이 아니다)')
  for tag in LIMITED + UNLIMITED:
    want = tag in LIMITED
    got = log_says_limited(tag)
    ok = (got == want)
    print(f'  {"OK " if ok else "✗  "} run_{tag}.log — '
          f'슬루 제한 {"켬" if want else "끔"}')
    if not ok:
      f.append(f'정답 출처 {tag}')

  print('\n■ 제한을 켜고 달린 런은 **통과** 해야 한다 (예전엔 전부 오경보)')
  for tag in LIMITED:
    line = verdict_line(review(tag))
    m = re.search(r'지속 (\d+)°/s', line)
    ok = bool(m) and int(m.group(1)) <= 50 and '⚠' not in line
    print(f'  {"OK " if ok else "✗  "} drive_{tag} — '
          f'지속 {m.group(1) if m else "?"}°/s ≤ 50 · 경고 없음')
    if not ok:
      f.append(f'제한 켠 런 {tag}')
      print(f'        | {line}')

  print('\n■ 제한 없이 달린 런은 **경고** 가 나야 한다')
  for tag in UNLIMITED:
    line = verdict_line(review(tag))
    m = re.search(r'지속 (\d+)°/s', line)
    ok = bool(m) and int(m.group(1)) > 50 and '⚠' in line
    print(f'  {"OK " if ok else "✗  "} drive_{tag} — '
          f'지속 {m.group(1) if m else "?"}°/s > 50 → 경고')
    if not ok:
      f.append(f'제한 없는 런 {tag}')
      print(f'        | {line}')

  print('\n■ 기준값 자체를 못 박는다 (문턱만 보면 2배 오류를 놓친다)')
  for tag, want in REF_P95.items():
    line = verdict_line(review(tag))
    m = re.search(r'지속 (\d+)°/s', line)
    got = int(m.group(1)) if m else -1
    ok = abs(got - want) <= REF_TOL
    print(f'  {"OK " if ok else "✗  "} drive_{tag} 지속 {got}°/s '
          f'(기준 {want}±{REF_TOL})')
    if not ok:
      f.append(f'기준값 {tag}')
      print(f'        | {line}')

  print('\n■ 갱신 간격을 데이터에서 재는가 (상수로 박으면 안 된다)')
  # 회피창 안은 10Hz. 이 값이 0.05 로 나오면 기록 주기를 그대로 쓰고 있다는 뜻.
  out = review('0337')
  m = re.search(r'갱신간격 중앙 ([\d.]+)s', out)
  ok = bool(m) and abs(float(m.group(1)) - 0.100) < 1e-6
  print(f'  {"OK " if ok else "✗  "} drive_0337 갱신간격 '
        f'{m.group(1) if m else "?"}s = 0.100s (라이다 10Hz)')
  if not ok:
    f.append('갱신 간격')

  print('\n■ 9/16 이전 기록의 단위버그를 잡는가')
  # drive_record.py 가 math.degrees(angular.z) 를 써서 57.2958 배로 부풀었다.
  # 조향은 ±18° 로 클램프되므로 814° 는 물리적으로 불가능하다.
  out = review('0020')
  ok = '기록 단위' in out and '57.2958' in out
  print(f'  {"OK " if ok else "✗  "} drive_0020 (cmd_steer 814°) → 실패로 찍는다')
  if not ok:
    f.append('단위버그 탐지')
  out = review('0337')
  ok = '기록 단위' not in out
  print(f'  {"OK " if ok else "✗  "} drive_0337 (정상, 최대 18°) → 안 찍는다')
  if not ok:
    f.append('단위버그 오탐')

  print('\n■ 구간별 이탈 — 이탈 상한 값의 근거를 만드는 곳 (2026-09-17)')
  # drive_0337 의 s20~34 은 회피 구간(s40~50) 밖이라 **기준 추종오차**다.
  # 커브(R 4.7~13m)인데도 p95 0.17m — S자에서 기댈 수 있는 숫자다.
  out = review('0337', '--section', '커브=20:34')
  m = re.search(r'커브.*?\n.*?이탈 평균 ([\d.]+) · p95 ([\d.]+)', out)
  ok = bool(m) and abs(float(m.group(2)) - 0.17) <= 0.02
  print(f'  {"OK " if ok else "✗  "} 커브 구간 기준 이탈 p95 '
        f'{m.group(2) if m else "?"}m (기준 0.17±0.02)')
  if not ok:
    f.append('구간 이탈 p95')

  # 상한 = 기준 p95 + 통과 필요량 + 여유.  0.17 + (0.6375+0.325) + 0.15 = 1.28
  out = review('0337', '--section', '커브=20:34', '--obstacle-width', '0.65')
  ok = 'avoid_max_lateral_m 1.28' in out
  print(f'  {"OK " if ok else "✗  "} 브룬 0.65m → 상한 권장 1.28m 을 계산한다')
  if not ok:
    f.append('상한 권장값')
  ok = '0.85' in out and '필요한 순간에 회피를 버린다' in out
  print(f'  {"OK " if ok else "✗  "} 학교 값 0.85 로는 안 된다고 말한다')
  if not ok:
    f.append('0.85 경고')

  # 차선 폭을 주면 이탈이 차선 안에 드는지 본다 (3.0m 면 못 든다)
  out = review('0337', '--section', '커브=20:34', '--obstacle-width', '0.65',
               '--lane-width', '3.0')
  ok = '차선 여유' in out and '❌' in out
  print(f'  {"OK " if ok else "✗  "} 차선 3.0m 면 회피가 차선을 벗어난다고 잡는다')
  if not ok:
    f.append('차선 여유')
  # 넉넉한 차선이면 통과해야 한다 — 항상 실패만 내는 검사는 쓸모없다
  out = review('0337', '--section', '커브=20:34', '--obstacle-width', '0.65',
               '--lane-width', '4.0')
  ok = '차선 여유' in out and '✅' in out
  print(f'  {"OK " if ok else "✗  "} 차선 4.0m 면 통과시킨다 (항상 실패가 아니다)')
  if not ok:
    f.append('차선 여유 통과')

  # 조용한 통과 방지 — 지나지도 않은 구간을 달라고 하면 실패해야 한다
  out = review('0337', '--section', '없는구간=500:600')
  ok = '표본이' in out and '❌' in out
  print(f'  {"OK " if ok else "✗  "} 안 지난 구간은 **소리내어 실패**한다')
  if not ok:
    f.append('빈 구간')

  print('\n■ 픽스처는 저장소 사본을 쓴다 (/tmp 는 재부팅하면 사라진다)')
  csv, _ = find_run('0337')
  ok = csv is not None and '/data/' in csv
  print(f'  {"OK " if ok else "✗  "} {csv}')
  if not ok:
    f.append('픽스처 경로')

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 판정표가 실행 로그의 정답과 맞는다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

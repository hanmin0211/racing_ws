#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_mission_plan_check.py — 계획 검사기가 **과거에 실제로 밟은 지뢰**를
   잡아내는지 본다.

좋은 계획을 통과시키는 검사기는 쉽다. 쓸모는 나쁜 계획을 잡는 데 있으므로,
케이스는 전부 **실제로 일어난 실패**에서 가져왔다(출처를 각 케이스에 적었다).

  python3 tools/test_mission_plan_check.py
"""

import os
import subprocess
import sys
import tempfile

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECK = os.path.join(WS, 'tools', 'mission_plan_check.py')
WP = 'config/chungju_school/wp_school_track_0.5.yaml'


def plan(missions, length=73.0, wp=WP, site='시험'):
  return {'course': {'site': site, 'waypoints': wp, 'path_length_m': length},
          'missions': missions}


def avoid(e, x, **kw):
  m = {'name': 'lidar_avoid', 'enabled': True, 'arm_topic': '/lidar/arm',
       'exclusive': False,
       'trigger': {'type': 'course_s', 's_enter': e, 's_exit': x}}
  m.update(kw)
  return m


def sudden(e, x, **kw):
  m = {'name': 'sudden_stop', 'enabled': True, 'arm_topic': '/sudden_stop/arm',
       'inhibits': ['lidar_avoid'],
       'trigger': {'type': 'course_s', 's_enter': e, 's_exit': x}}
  m.update(kw)
  return m


def run(name, p, args, want_rc, want_text, fails):
  """want_text = 출력에 반드시 있어야 하는 문자열들."""
  import yaml
  fd, path = tempfile.mkstemp(suffix='.yaml')
  with os.fdopen(fd, 'w') as f:
    yaml.safe_dump(p, f, allow_unicode=True)
  try:
    r = subprocess.run([sys.executable, CHECK, path] + args,
                       capture_output=True, text=True, cwd=WS)
  finally:
    os.unlink(path)
  out = r.stdout + r.stderr
  ok = (r.returncode == want_rc) and all(w in out for w in want_text)
  print(f'  {"OK " if ok else "✗  "} {name}')
  if not ok:
    print(f'        기대 rc={want_rc} · 포함 {want_text}')
    print(f'        실제 rc={r.returncode}')
    for line in out.strip().splitlines():
      print(f'        | {line}')
    fails.append(name)


def main():
  f = []
  print('미션 계획 검사기 — 과거에 실제로 밟은 지뢰를 잡는가\n')

  print('■ 검증된 계획은 통과시킨다')
  # 2026-09-17 실차: arm s37 · BLOCKED 0% · obs 최소 8.00m (안 박았다)
  run('학교 계획 그대로 (arm s37 · 돌발 s51) — 실패 없음',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]),
      ['--obstacle', 'lidar_avoid=44'], 0, ['arm 여유', '✅'], f)

  print('\n■ arm 이 늦다 — drive_0117/0126 에서 첫 장애물에 박은 그 설정')
  # 실측: arm s40.5 되는 순간 obs 2.04m = 이미 BLOCKED 안. 조향 NaN → 직진 → 충돌
  run('arm s40 · 장애물 s44 (여유 4.0m) → 실패',
      plan([avoid(40.0, 50.0), sudden(51.0, 70.0)]),
      ['--obstacle', 'lidar_avoid=44'], 1, ['arm 여유', 'BLOCKED'], f)
  run('arm s37 · 장애물 s44 (여유 7.0m) → 통과',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]),
      ['--obstacle', 'lidar_avoid=44'], 0, ['arm 여유'], f)
  run('장애물이 구간 밖이면 미션이 안 돈다',
      plan([avoid(37.0, 42.0), sudden(51.0, 70.0)]),
      ['--obstacle', 'lidar_avoid=44'], 1, ['장애물 위치'], f)

  print('\n■ 구간 겹침 — 돌발의 inhibits 가 회피를 꺼 의자를 못 피했다 (9/14)')
  run('회피 37~56 · 돌발 51~70 이 5m 겹친다 → 실패',
      plan([avoid(37.0, 56.0), sudden(51.0, 70.0)]), [], 1,
      ['겹친다', 'inhibits'], f)
  run('1.0m 띄우면 통과',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]), [], 0, ['겹침 없음'], f)

  print('\n■ 커브에서 arm — 먹스가 경로조향을 버려 가드레일로 밀었다 (9/13)')
  # s33 R=6.4m 급커브 · s35 R=8.1m 경계 · s37 R=11.0m 실차 허용(이탈 0.18m)
  run('급커브 s33 (R=6.4m) 에서 arm → 실패',
      plan([avoid(33.0, 50.0), sudden(51.0, 70.0)]), [], 1,
      ['arm 지점 곡률'], f)
  run('s35 (R=8.1m) 는 경계 — 경고는 나되 실패는 아니다',
      plan([avoid(35.0, 50.0), sudden(51.0, 70.0)]), [], 0,
      ['arm 지점 곡률'], f)

  print('\n■ 캘리브 구간 — teleop 이 먹스 우선이라 미션이 아예 안 돈다')
  run('s8 에 두면 실패 (예전 돌발 8~18 이 실제로 안 돌았다)',
      plan([avoid(37.0, 50.0), sudden(8.0, 18.0)]), [], 1,
      ['캘리브 구간'], f)

  print('\n■ 다른 현장의 계획 — 메모리에 박힌 탈락 경로')
  run('길이가 안 맞으면 실패 (시퀀서가 런타임에 거부하는 것과 같은 조건)',
      plan([avoid(37.0, 50.0)], length=210.0), [], 1, ['path_length_m'], f)

  print('\n■ 형식 오류')
  run('s_enter ≥ s_exit',
      plan([avoid(50.0, 37.0)]), [], 1, ['구간'], f)
  run('코스 밖',
      plan([avoid(37.0, 95.0)]), [], 1, ['코스 밖'], f)

  print('\n■ arm 지점 역산 — 실측으로 얻은 답을 기하에서 다시 도출하는가')
  # 2026-09-17 밤: 여러 번 박고 나서 s_enter 40.0 → 37.0 으로 바꿔 해결했다.
  # 그 값을 **감지 트리거 + BLOCKED 반경 + 창 곡률** 만으로 다시 얻어야 한다.
  run('장애물 s44 → s_enter 37.0 을 권한다 (실측으로 얻은 값과 같다)',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]),
      ['--suggest', '44'], 0, ['s_enter: 37.0', '창최악 R=11.0'], f)
  run('쓸 수 있는 구간의 아래끝이 s35 다 (인계문서: s35 아래로 내리지 말 것)',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]),
      ['--suggest', '44'], 0, ['s 35.0 ~ 37.0'], f)
  # 창 전체를 안 보면 s21.5 를 권하게 된다 — 그 창엔 s25 R=4.7m 헤어핀이 있다.
  run('급커브 뒤에 장애물이 있으면 **둘 곳이 없다**고 말한다',
      plan([avoid(20.0, 34.0)]), ['--suggest', '30'], 1,
      ['두면 안 되는 위치'], f)

  print('\n■ 자리표시자 (대회 계획을 채우는 중)')
  run('전부 비활성이면 --include-disabled 를 알려준다',
      plan([avoid(0.0, 0.0, enabled=False)]), [], 1,
      ['--include-disabled'], f)
  # s=0~0 자리표시자는 '구간 퇴화' 가 먼저 잡힌다. 그게 맞는 순서다 —
  # 아직 안 채웠다는 뜻이므로 곡률·여유를 따질 단계가 아니다.
  run('--include-disabled 면 자리표시자(s=0~0)도 검사해 미완성을 잡는다',
      plan([avoid(0.0, 0.0, enabled=False)]), ['--include-disabled'], 1,
      ['s_enter 0.0 ≥ s_exit 0.0'], f)
  run('채우다 만 것도 잡는다 — 캘리브 구간에 들어간 경우',
      plan([avoid(5.0, 20.0, enabled=False)]), ['--include-disabled'], 1,
      ['캘리브 구간'], f)

  print('\n■ 장애물 크기 — 용인은 헤네스 브룬 차체(바퀴 뺀 것) 무작위 배치')
  YWP = 'config/yongin_2026-09-05/wp_yongin_drive_0.5.yaml'

  def yplan(e, x):
    return plan([avoid(e, x)], length=648.3, wp=YWP, site='용인')

  # 세로로 놓이면 폭 0.65m → 옆으로 0.96m 가야 지나간다. 권장 상한 0.85 로는 안 된다.
  run('브룬 세로(0.65m) → 상한 0.85 로는 못 지나간다고 말한다',
      yplan(169.0, 250.0), ['--obstacle-width', '0.65', '--curve-avoid'], 0,
      ['통과 이탈량', 'avoid_max_lateral_m'], f)
  # 가로로 놓이면 폭 1.25m → 필요 1.26m > 트랙창 1.20m → 갭이 없다
  run('브룬 가로(1.25m) → 트랙창 밖이라 갭이 없다(BLOCKED)',
      yplan(169.0, 250.0), ['--obstacle-width', '1.25', '--curve-avoid'], 1,
      ['갭이 없다'], f)
  # BLOCKED 거리가 커지면 arm 여유 요구도 커진다
  run('큰 장애물이면 arm 여유 요구가 커진다 (2.8m 고정이 아니다)',
      yplan(172.0, 250.0),
      ['--obstacle-width', '1.25', '--obstacle', 'lidar_avoid=176',
       '--curve-avoid'], 1, ['arm 여유'], f)

  print('\n■ 커브 안에서 회피해야 하는 코스 (용인 S자)')
  run('--curve-avoid 없으면 S자 arm 을 실패로 찍는다',
      yplan(190.0, 250.0), [], 1, ['arm 지점 곡률'], f)
  run('--curve-avoid 면 경고로 낮추고 **이탈 상한을 요구**한다',
      yplan(190.0, 250.0), ['--curve-avoid'], 0,
      ['실패→경고', '이탈 상한'], f)
  run('조향 여유를 보고한다 — 커브에선 경로조향이 먼저 조향을 먹는다',
      yplan(169.0, 250.0), ['--curve-avoid'], 0, ['조향 여유', '경로조향이'], f)

  print('\n■ 조용한 통과를 막는다')
  run('장애물 s 를 안 주면 arm 여유는 **미검사**라고 말한다',
      plan([avoid(37.0, 50.0), sudden(51.0, 70.0)]), [], 0, ['미검사'], f)

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}')
    return 1
  print('✅ 전부 통과 — 검사기가 과거의 실패를 전부 잡는다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

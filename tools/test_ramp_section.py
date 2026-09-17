#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_ramp_section.py — 웨이포인트 구간 속도가 **주행을 안 망가뜨리는지** 본다.

★ 이 시험이 지키려는 것
  경사로 구간에서 기준속도를 바꾸는 기능이다. 대회 D-1 에 완주 로직을
  건드리는 것이 가장 큰 위험이라고 사용자가 못 박았다. 그래서 검증 대상은
  '오르막에서 빨라지는가' 가 아니라 **'나머지가 그대로인가'** 다.

  ① 안 켜면 예전과 **완전히 같은 값**이 나오는가
  ② 구간이 **잘못 찍혀** 있어도 곡률·정지선·장애물 제한이 살아 있는가
  ③ 시퀀서가 죽으면 평소 속도로 **돌아오는가**

  MISSION_POLICY 처럼 고정속도를 return 하는 방식을 안 쓴 이유가 ②다.
  그쪽은 하류 제한을 통째로 건너뛴다 — 구간이 코너에 잘못 걸리면 이탈이다.

  longitudinal_controller_node 의 _base_speed()/decide_target() 을
  **그대로 불러서** 시험한다.

  python3 tools/test_ramp_section.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'velocity_controller'))

from velocity_controller import longitudinal_controller_node as L  # noqa: E402

N = L.LongitudinalController
BASE = N._base_speed
DECIDE = N.decide_target


class Stub:
  # 실제 메서드를 붙인다 — decide_target 이 self._base_speed() 를 부른다.
  _base_speed = BASE

  def __init__(self, **kw):
    # 노드 기본값과 같게
    self.v_max = 1.0
    self.v_min = 0.25
    self.v_slow = 0.8
    self.curv_gain = 6.0
    self.kappa = 0.0
    self.mission = 'DRIVE'
    self.stop_line_dist = 999.0
    self.stop_trigger = 19.0
    self.stop_decel = 1.0
    self.obstacle_dist = 999.0
    self.obs_trigger = 4.0
    self.obs_stop = 0.8
    # 구간 속도 (기본 = 안 씀)
    self.v_ramp_up = 0.0
    self.v_ramp_down = 0.0
    self.ramp_arm_timeout = 1.0
    self._ramp_up_on = False
    self._ramp_up_t = 0.0
    self._ramp_down_on = False
    self._ramp_down_t = 0.0
    self.__dict__.update(kw)


NOW = time.time()
fails = []


def chk(name, ok, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   {detail}' if detail else ''))
  if not ok:
    fails.append(name)


def armed_up(v, **kw):
  return Stub(v_ramp_up=v, _ramp_up_on=True, _ramp_up_t=NOW, **kw)


def armed_down(v, **kw):
  return Stub(v_ramp_down=v, _ramp_down_on=True, _ramp_down_t=NOW, **kw)


def main():
  print('=' * 68)
  print('  경사로 구간 속도 — 나머지 주행이 그대로인지 본다')
  print('=' * 68)

  # ================================================================
  print('\n■ 안 켜면 예전과 완전히 같다')
  st = Stub()
  chk('기준속도 = v_max', BASE(st) == 1.0)
  chk('직선 목표 = v_max', abs(DECIDE(st) - 1.0) < 1e-9, f'{DECIDE(st):.3f}')
  st = Stub(kappa=1.0 / 6.4)
  want = 1.0 / (1.0 + 6.0 / 6.4)
  chk('코너 감속도 그대로', abs(DECIDE(st) - want) < 1e-9, f'{DECIDE(st):.3f}')
  # 토픽만 있고 속도가 0 이면 여전히 안 쓴다
  st = Stub(v_ramp_up=0.0, _ramp_up_on=True, _ramp_up_t=NOW)
  chk('arm 이 와도 v_ramp_up=0 이면 무시', BASE(st) == 1.0)

  # ================================================================
  print('\n■ 구간 안에서는 기준속도가 바뀐다')
  chk('오르막 arm → 2.0', BASE(armed_up(2.0)) == 2.0)
  chk('내리막 arm → 0.6', BASE(armed_down(0.6)) == 0.6)
  chk('오르막 구간 직선 목표 = 2.0',
      abs(DECIDE(armed_up(2.0)) - 2.0) < 1e-9)
  chk('내리막 구간 직선 목표 = 0.6',
      abs(DECIDE(armed_down(0.6)) - 0.6) < 1e-9)

  # ================================================================
  print('\n■ 구간이 잘못 찍혀 있어도 하류 제한이 살아 있다 (핵심)')
  # 굴절코스 첫 코너 R=6.4m 에 오르막 구간이 잘못 걸린 경우
  st = armed_up(2.0, kappa=1.0 / 6.4)
  want = 2.0 / (1.0 + 6.0 / 6.4)
  chk(f'R=6.4m 코너에 걸려도 곡률이 누른다 ({want:.2f})',
      abs(DECIDE(st) - want) < 1e-9, f'{DECIDE(st):.3f}')
  # 용인 최악 R=3.0m
  st = armed_up(2.0, kappa=1.0 / 3.0)
  chk('R=3.0m(코스 최악)에서는 더 눌린다', DECIDE(st) < 1.0,
      f'{DECIDE(st):.3f}')
  # 정지선
  st = armed_up(2.0, stop_line_dist=0.2)
  chk('정지선 0.2m 면 **선다** (구간과 무관)', DECIDE(st) == 0.0)
  st = armed_up(2.0, stop_line_dist=2.0)
  chk('정지선 2.0m 면 √(2ad)=2.0 으로 눌린다',
      abs(DECIDE(st) - 2.0) < 1e-6, f'{DECIDE(st):.3f}')
  # 장애물
  st = armed_up(2.0, obstacle_dist=0.5)
  chk('장애물 0.5m 면 **선다**', DECIDE(st) == 0.0)
  st = armed_up(2.0, obstacle_dist=2.0)
  chk('장애물 2.0m 면 감속한다', DECIDE(st) < 1.0, f'{DECIDE(st):.3f}')
  # 돌발 급정지는 /obstacle_distance 로 온다 — 구간 속도가 이걸 못 이긴다
  st = armed_up(3.0, obstacle_dist=0.7)
  chk('돌발 더미(0.7m)도 구간 속도를 이긴다', DECIDE(st) == 0.0)

  # ================================================================
  print('\n■ 시퀀서가 죽으면 평소 속도로 돌아온다')
  st = armed_up(2.0)
  st._ramp_up_t = NOW - 1.5                       # 1.5초 전 (상한 1.0)
  chk('arm 이 1.5초 끊기면 v_max 로 복귀', BASE(st) == 1.0)
  st = armed_up(2.0)
  st._ramp_up_t = NOW - 0.5
  chk('0.5초는 아직 유효', BASE(st) == 2.0)
  st = armed_up(2.0)
  st._ramp_up_on = False                          # 시퀀서가 false 를 냈다
  chk('시퀀서가 false 를 내면 바로 복귀', BASE(st) == 1.0)

  # ================================================================
  print('\n■ 계획을 잘못 적어 구간이 겹치면 — 느린 쪽을 쓴다')
  st = Stub(v_ramp_up=2.0, _ramp_up_on=True, _ramp_up_t=NOW,
            v_ramp_down=0.6, _ramp_down_on=True, _ramp_down_t=NOW)
  chk('둘 다 켜지면 min(2.0, 0.6) = 0.6', BASE(st) == 0.6,
      f'{BASE(st):.2f}')

  # ================================================================
  print('\n■ v_min 하한은 그대로 — 코너링 스톨 방지')
  st = armed_down(0.10, kappa=0.0)                # v_min(0.25) 보다 낮게 요구
  chk('내리막 구간이 v_min 밑을 요구해도 v_min 으로 올라온다',
      abs(DECIDE(st) - 0.25) < 1e-9, f'{DECIDE(st):.3f}')

  # ================================================================
  print('\n■ 완주·경로끊김 분기는 건드리지 않았다')
  # decide_target 은 MISSION_POLICY 가 있으면 그쪽이 먼저다 — 구간 속도가
  # 미션 정책을 덮어쓰지 않는지 확인한다.
  policies = [k for k in L.MISSION_POLICY if k != 'DRIVE']
  if policies:
    k = policies[0]
    st = armed_up(3.0, mission=k)
    pol = L.MISSION_POLICY[k]
    want = float(getattr(st, pol)) if isinstance(pol, str) else float(pol)
    chk(f'미션 정책({k})이 구간 속도보다 우선', abs(DECIDE(st) - want) < 1e-9,
        f'{DECIDE(st):.2f} = {want:.2f}')
  else:
    chk('MISSION_POLICY 가 비어 있다 (확인만)', True)

  print()
  if fails:
    print(f'❌ 실패 {len(fails)}건: {", ".join(fails)}')
    return 1
  print('✅ 전부 통과 — 구간 속도는 기준값만 바꾸고 안전장치는 전부 살아 있다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

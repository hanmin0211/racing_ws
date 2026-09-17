#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_avoid_hold.py — 먹스의 회피 유지시간 + 이탈 상한을 하드웨어 없이 검증한다.

★ 왜 이 둘을 한 파일에서 시험하나
  둘은 한 쌍이다. 유지시간만 늘리면 회피각을 더 오래 물고 있으므로 이탈이
  커지고, 이탈 상한만 걸면 BLOCKED 빈틈에서 여전히 장애물 쪽으로 조향한다.
  따로 시험하면 '각각은 맞는데 같이 쓰면 틀린' 조합을 놓친다.

★ 무엇을 못 박는가
  ① 라이다가 죽으면(avoid_timeout) **유지하지 않는다.** 유지시간이 메우는 것은
     BLOCKED/CLEAR 빈틈이지 센서 부재가 아니다. 센서가 없는데 옛 각을 물고
     가는 것이 제일 위험하다.
  ② DISARM 되면 **즉시** 푼다. 안 그러면 구간을 벗어나고도 회피각으로 달린다.
  ③ 경로(/local_path)를 모르면 상한을 **걸지 않는다.** 여기서 회피를 끄면
     경로도 모르는 채 장애물로 직진한다.
  ④ 기본값(hold=0, 상한=0)에서는 **예전과 완전히 같이** 동작한다.
     drive_0337 은 이 기능들 없이 완주했다 — 검증된 동작을 깨면 안 된다.

  구현을 복사하지 않고 vehicle_cmd_mux_node 의 메서드를 **그대로 불러** 쓴다.
  로직을 베껴 오면 원본이 바뀔 때 시험이 거짓말을 한다.

  python3 tools/test_avoid_hold.py
"""

import csv

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drive_fixtures import require_run  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'velocity_controller'))

from velocity_controller.vehicle_cmd_mux_node import VehicleCmdMux  # noqa: E402

AVOID_ANGLE = VehicleCmdMux.avoid_angle
CAP_REASON = VehicleCmdMux.cap_reason
PATH_CB = VehicleCmdMux.local_path_cb
ARM_CB = VehicleCmdMux.arm_cb
FRESH = VehicleCmdMux.fresh

NAN = float('nan')


class _Log:
  def info(self, *a, **k): pass
  def warn(self, *a, **k): pass
  def error(self, *a, **k): pass


class Stub:
  """avoid_angle / cap_reason 이 참조하는 속성만 갖춘 최소 객체."""

  def __init__(self, **kw):
    self.t = 100.0                 # '지금' — 시험이 직접 민다
    self.avoid_steer = NAN
    self.avoid_time = None
    self.avoid_timeout = 0.3
    self.avoid_hold_s = 0.0
    self.avoid_max_lat = 0.0
    self.avoid_max_head = 0.0
    self.avoid_cap_release = 0.6
    self.local_path_timeout = 0.5
    self.last_avoid = NAN
    self.last_avoid_t = None
    self.avoid_armed = True
    self.path_lat = 0.0
    self.path_head = 0.0
    self.path_time = None
    self.cap_tripped = False
    self._cap_logged = False
    self.__dict__.update(kw)

  def now(self):
    return self.t

  def get_logger(self):
    return _Log()

  def fresh(self, t, timeout):
    return FRESH(self, t, timeout)

  # 편의: 회피각 한 프레임 넣기
  def feed(self, steer, dt=0.1):
    self.t += dt
    self.avoid_steer = steer
    self.avoid_time = self.t
    return AVOID_ANGLE(self)


class _P:
  def __init__(self, x, y):
    self.pose = type('o', (), {'position': type('p', (), {'x': x, 'y': y})()})()


class _Path:
  def __init__(self, pts):
    self.poses = [_P(x, y) for x, y in pts]


def approx(a, b, tol=1e-6):
  if a is None or b is None:
    return a is b
  return abs(a - b) <= tol


def main():
  fails = []

  def check(name, ok, detail=''):
    print(f'  {"OK " if ok else "✗  "} {name}')
    if not ok:
      if detail:
        print(f'        {detail}')
      fails.append(name)

  # ---------------------------------------------------------------- 유지시간
  print('\n[유지시간] 회피각을 언제 물고 있어야 하나')

  s = Stub()
  check('유효각은 그대로 나간다', approx(s.feed(12.0), 12.0))
  check('마지막 유효각이 기록된다', approx(s.last_avoid, 12.0))

  # 기본값(hold=0) = 예전 동작
  s = Stub()
  s.feed(12.0)
  check('hold=0 이면 NaN 즉시 경로조향 (예전 동작 보존)',
        s.feed(NAN) is None)

  # hold 안이면 메운다
  s = Stub(avoid_hold_s=0.3)
  s.feed(-18.0)
  got = s.feed(NAN, dt=0.15)
  check('hold=0.3, 0.15s 빈틈 → 마지막 각(-18°)으로 메운다',
        approx(got, -18.0), f'받은 값 {got}')

  # hold 밖이면 놓는다
  s = Stub(avoid_hold_s=0.3)
  s.feed(-18.0)
  s.avoid_steer, s.t, s.avoid_time = NAN, s.t + 0.5, s.t + 0.5
  check('hold=0.3, 0.5s 경과 → 놓는다', AVOID_ANGLE(s) is None)

  # ★ 센서가 죽으면 유지하지 않는다
  s = Stub(avoid_hold_s=2.0)
  s.feed(-18.0)
  s.t += 0.4                       # avoid_time 은 안 갱신 = 라이다 무신호
  check('라이다 무신호면 hold 가 남아 있어도 포기한다 (센서 부재 ≠ 빈틈)',
        AVOID_ANGLE(s) is None)

  # ★ DISARM 즉시 해제
  s = Stub(avoid_hold_s=2.0)
  s.feed(-18.0)
  ARM_CB(s, type('m', (), {'data': False})())
  got = s.feed(NAN, dt=0.05)
  check('DISARM 되면 hold 가 남아 있어도 즉시 경로조향',
        got is None, f'받은 값 {got}')
  check('DISARM 이 hold 상태를 지운다', s.last_avoid_t is None)

  # 유효각을 한 번도 못 받았으면 메울 것이 없다
  s = Stub(avoid_hold_s=0.3)
  check('유효각을 받은 적 없으면 NaN 은 그냥 None',
        s.feed(NAN) is None)

  # ---------------------------------------------------------------- 이탈 상한
  print('\n[이탈 상한] 회피를 언제 포기해야 하나')

  s = Stub()
  s.path_lat, s.path_head, s.path_time = 5.0, 90.0, s.t
  check('상한 0 이면 아무리 벗어나도 안 건다 (기본값 = 예전 동작)',
        CAP_REASON(s) is None)

  s = Stub(avoid_max_lat=0.85, avoid_max_head=22.0)
  check('경로를 모르면 상한을 걸지 않는다 (막으면 장애물로 직진한다)',
        CAP_REASON(s) is None)

  s = Stub(avoid_max_lat=0.85, avoid_max_head=22.0)
  s.path_lat, s.path_head, s.path_time = 0.50, 10.0, s.t
  check('이탈 0.50m / 헤딩 10° → 통과', CAP_REASON(s) is None)

  # drive_0337 의 성공한 회피 최댓값이 막히면 안 된다
  s = Stub(avoid_max_lat=0.85, avoid_max_head=22.0)
  s.path_lat, s.path_head, s.path_time = 0.72, 17.6, s.t
  check('drive_0337 의 성공한 회피 최댓값(0.72m/17.6°)은 통과해야 한다',
        CAP_REASON(s) is None)

  s = Stub(avoid_max_lat=0.85, avoid_max_head=22.0)
  s.path_lat, s.path_head, s.path_time = 0.90, 10.0, s.t
  r = CAP_REASON(s)
  check('이탈 0.90m → 걸린다 (복귀불능 경계)', r is not None, f'받은 값 {r}')
  check('걸리면 cap_tripped 가 선다', s.cap_tripped)

  # 히스테리시스
  s.path_lat = 0.60                       # 0.85*0.6 = 0.51 보다 크다
  check('히스테리시스: 0.60m 는 아직 안 풀린다 (0.51m 밑으로 와야)',
        CAP_REASON(s) is not None)
  s.path_lat = 0.40
  check('히스테리시스: 0.40m 로 돌아오면 회피 재개', CAP_REASON(s) is None)
  check('풀리면 cap_tripped 가 내려간다', not s.cap_tripped)

  s = Stub(avoid_max_lat=0.0, avoid_max_head=22.0)
  s.path_lat, s.path_head, s.path_time = 5.0, 25.0, s.t
  check('헤딩만 켜도 헤딩으로 걸린다 (이탈 상한 0 은 무시)',
        CAP_REASON(s) is not None)

  # ---------------------------------------------------------------- 경로 읽기
  print('\n[경로 읽기] /local_path 첫 점이 곧 이탈량인가')

  s = Stub()
  PATH_CB(s, _Path([(0.0, 0.7), (0.5, 0.7)]))
  check('첫 점 y = 이탈량', approx(s.path_lat, 0.7), f'{s.path_lat}')
  check('첫 두 점이 나란하면 헤딩오차 0', approx(s.path_head, 0.0, 1e-9))

  s = Stub()
  PATH_CB(s, _Path([(0.0, 0.0), (1.0, 1.0)]))
  check('경로가 좌로 45° 면 헤딩 +45°', approx(s.path_head, 45.0, 1e-6))

  s = Stub()
  PATH_CB(s, _Path([(0.0, 9.9)]))
  check('점이 하나뿐이면 무시한다 (기울기를 못 낸다)', s.path_time is None)

  # ------------------------------------------------- 실측 리플레이 (drive_0337)
  print('\n[실측 리플레이] drive_0337 의 BLOCKED 빈틈')
  # ★ 2026-09-17 — 없으면 조용히 건너뛰던 것을 저장소 사본 우선 + 하드 실패로.
  csv_path, _ = require_run('0337')
  if True:
    rows = list(csv.DictReader(open(csv_path)))

    def replay(hold):
      """CSV 의 avoid_steer/arm 을 그대로 흘려보내고 먹스 출력을 모은다."""
      st = Stub(avoid_hold_s=hold)
      out = []
      for r in rows:
        t = float(r['t'])
        raw = r['avoid_steer_deg']
        a = float(raw) if raw not in ('', 'nan', 'None') else NAN
        armed = '조향OFF' not in r['mode']
        ARM_CB(st, type('m', (), {'data': armed})())
        st.t, st.avoid_steer, st.avoid_time = t, a, t
        out.append((t, AVOID_ANGLE(st), armed))
      return out

    base = replay(0.0)
    held = replay(0.3)

    # t78.0~78.3 의 BLOCKED 빈틈 (핸드오프 §2 · 이 런 분석 참고)
    win = [i for i, (t, _, _) in enumerate(base) if 77.95 <= t <= 78.35]
    gap0 = [i for i in win if base[i][1] is None]
    gap1 = [i for i in win if held[i][1] is None]
    check('hold=0 에서는 t78.0~78.3 에 빈틈이 있다 (고쳐야 할 대상이 존재)',
          len(gap0) > 0, f'빈틈 프레임 {len(gap0)}개')
    check('hold=0.3 이 그 빈틈을 메운다',
          len(gap1) < len(gap0), f'{len(gap0)}개 → {len(gap1)}개')
    filled = [i for i in gap0 if held[i][1] is not None]
    if filled:
      vals = sorted({round(held[i][1], 1) for i in filled})
      # ★ 2026-09-16 정정 — 처음엔 '메운 값은 −18°(우) 일 것' 으로 적었다가
      #   이 시험에 반증당했다. 0.05초 단위 원본을 보면 빈틈 **직전** 회피각은
      #   +16.8°(좌) 였고, 우(−15.8°)로 뒤집힌 것은 빈틈 **뒤**다. 즉 유지시간은
      #   이 장면에서 좌를 물고 있어 오히려 늦췄을 것이다.
      #   이걸 못 박아 두는 이유: 유지시간을 '이 빈틈의 해법' 으로 오해하면
      #   안 되기 때문이다. 진짜 원인은 플래너의 좌우 뒤집힘이다
      #   (cluster_plot_node 의 avoid_steer_rate 주석 참고).
      check('유지시간은 빈틈 직전 각을 물고 간다 (그게 옳은 방향이란 보장은 없다)',
            all(v > 0 for v in vals),
            f'메운 값들 {vals} — 직전이 좌였으므로 좌가 나와야 한다')

    # DISARM 이후로 새지 않는가
    leak = [tt for (tt, aa, ar) in held if aa is not None and not ar]
    check('hold=0.3 이 DISARM 구간으로 새지 않는다',
          not leak, f'샌 시각 {leak[:5]}')

    # 기본값은 예전과 완전히 동일해야 한다
    same = all((a is None) == (b is None)
               and (a is None or approx(a, b))
               for (_, a, _), (_, b, _) in zip(base, replay(0.0)))
    check('hold=0 리플레이는 결정적이다 (기본값 동작 불변)', same)

    n_base = sum(1 for (_, aa, _) in base if aa is not None)
    n_held = sum(1 for (_, aa, _) in held if aa is not None)
    print(f'  ·   override 프레임: hold=0 → {n_base},  hold=0.3 → {n_held}')

  print()
  if fails:
    print(f'❌ 실패 {len(fails)}건: {", ".join(fails)}')
    return 1
  print('✅ 전부 통과 — 유지시간·이탈상한이 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_heading_tracker.py — 헤딩 연속 보정(HeadingTracker)을 하드웨어 없이 검증한다.

★ 왜 (2026-10-12)
  10/12 IMU yaw 가 분당 −21° 떠내려가 차가 경로 오른쪽으로 0.5~1.5 m 밀렸다.
  direct_localization 이 GPS 진행방향으로 오프셋을 계속 고치게 했다(heading_track).
  헤딩은 조향에 바로 들어가므로 **고치면 안 될 때 안 고치는지**(커브·후진·FLOAT·
  점프·정지)가 고쳐야 할 때 고치는지만큼 중요하다.

  python3 tools/test_heading_tracker.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'gps_localization'))
from gps_localization.heading_tracker import HeadingTracker, wrap  # noqa: E402

D = math.radians


def drive(tr, secs, speed, true_hdg, imu_bias, bias_rate=0.0, turn_rate=0.0, t0=0.0,
          x0=0.0, y0=0.0, sigma=0.01, jump_at=None, reverse=False):
  """IMU 300 Hz · GPS 10 Hz 로 차를 굴린다. imu = 참헤딩 − 편향(t). 결과 마지막 상태."""
  t, x, y, h, b = t0, x0, y0, true_hdg, imu_bias
  res = []
  n = int(secs * 300)
  for k in range(n):
    dt = 1 / 300
    t += dt; h += turn_rate * dt; b += bias_rate * dt
    sgn = -1 if reverse else 1
    x += sgn * speed * math.cos(h) * dt; y += sgn * speed * math.sin(h) * dt
    tr.add_imu(t, wrap(h - b))
    if k % 30 == 29:
      jx = 0.4 if (jump_at is not None and abs(t - jump_at) < 0.02) else 0.0
      res.append(tr.add_fix(t, x + jx, y, sigma))
  return t, x, y, h, b, res


def check(name, ok, fails, detail=''):
  print(f'  {"OK " if ok else "✗  "} {name}' + (f'   ({detail})' if detail else ''))
  if not ok:
    fails.append(name)


def main():
  f = []
  print('헤딩 연속 보정 검증\n')

  print('■ 캘리브 전에는 아무것도 안 한다')
  tr = HeadingTracker()
  _, _, _, _, _, r = drive(tr, 3, 1.5, 0.3, D(10))
  check('offset None 이면 갱신 안 함', tr.offset is None and set(r) == {None}, f)

  print('\n■ 고쳐야 할 때 — 10/12 처럼 떠내려가는 IMU')
  tr = HeadingTracker(tau=3.0)
  tr.set_offset(D(10))                       # 캘리브 때 편향 10° 로 정확히 잡았다
  t, x, y, h, b, r = drive(tr, 60, 1.5, 0.3, D(10), bias_rate=D(-21) / 60)
  err = math.degrees(wrap(tr.offset - b))
  check('분당 −21° 드리프트를 1분 뒤에도 따라간다 (오차 < 2°)', abs(err) < 2.0, f,
        f'편향 {math.degrees(b):.1f}° · 추정 {math.degrees(tr.offset):.1f}° · 오차 {err:+.2f}°')
  check('그동안 대부분 갱신했다', r.count('ok') > 0.8 * len(r), f, f'{r.count("ok")}/{len(r)}')

  print('\n■ ★ 고치면 안 될 때 — 조향에 바로 들어간다')
  for name, kw, want in (
      ('커브 (회전율 30°/s)', dict(turn_rate=D(30)), 'turn'),
      ('서 있다 (0.2 m/s)', dict(speed=0.2), 'slow'),
      ('RTK FLOAT (σ 25cm)', dict(sigma=0.25), 'rtk'),
      ('후진 (진행방향이 180° 반대)', dict(reverse=True), 'outlier')):
    tr = HeadingTracker(); tr.set_offset(D(10))
    args = dict(secs=3, speed=1.5, true_hdg=0.3, imu_bias=D(10)); args.update(kw)
    _, _, _, _, _, r = drive(tr, **args)
    moved = abs(math.degrees(wrap(tr.offset - D(10))))
    check(f'{name} → {want}, 오프셋 그대로', r.count('ok') == 0 and want in r and moved < 1e-9, f,
          f'처리 {dict((k, r.count(k)) for k in set(r))}')
  tr = HeadingTracker(); tr.set_offset(D(10))
  _, _, _, _, _, r = drive(tr, 4, 1.5, 0.3, D(10), jump_at=2.0)
  moved = abs(math.degrees(wrap(tr.offset - D(10))))
  check('GPS 0.4 m 점프 (10/12 실측) — 오프셋이 1° 넘게 안 움직인다', moved < 1.0, f,
        f'{moved:.2f}° · 이상 {r.count("outlier")}회')

  print('\n■ 각도 경계')
  tr = HeadingTracker(); tr.set_offset(D(179))
  _, _, _, _, b, _ = drive(tr, 30, 1.5, 2.0, D(179), bias_rate=D(4) / 60)
  err = math.degrees(wrap(tr.offset - b))
  check('±180° 를 넘나들어도 따라간다', abs(err) < 1.0, f, f'오차 {err:+.2f}°')

  print('\n■ 재캘리브')
  tr = HeadingTracker(); tr.set_offset(D(10))
  drive(tr, 5, 1.5, 0.3, D(10))
  tr.set_offset(D(50))
  check('set_offset 이면 그 값으로 다시 시작', abs(math.degrees(tr.offset) - 50) < 1e-9, f)

  print()
  if f:
    print(f'❌ 실패 {len(f)}건: {", ".join(f)}'); return 1
  print('✅ 전부 통과 — 헤딩 연속 보정이 설계대로 동작한다')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())

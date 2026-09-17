#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_mount_geom.py — 마운트 점검의 **평면 기하**를 검증한다.

★ 왜 따로 있나 (2026-09-17)
  test_lidar_mount_check.py 는 '거리 판정' 을 본다. 이 파일은 2026-09-17 에
  새로 넣은 **지면인지 물체인지 가르는 기하** 를 본다. 그날 측정에서
  "❌ 고장 2.33m" 이 나왔는데, 그게 지면인지 앞에 놓인 의자인지 도구가
  대답하지 못해 현장이 멈췄다. 그걸 못 박는다.

  고정하는 성질:
    ① 지면 직선의 x0 · 피치 · 롤을 **정확히** 복원한다
    ② 앞에 놓인 물체는 '물체' 로 갈라낸다 (지면이라고 하지 않는다)
    ③ 롤 부호가 맞다 — 어느 쪽에 심을 넣을지 틀리면 더 나빠진다
    ④ **정면 벽은 한 장으로 구분 못 한다** — 알려진 한계. --confirm 이 필요하다는
       사실 자체를 시험으로 박아 둔다(나중에 '왜 확인 모드가 있지' 를 없애지 않게)

  python3 tools/test_mount_geom.py
"""
import math
import os


def _load():
    """rclpy 없이 순수 함수만 꺼낸다."""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'lidar_mount_check.py')).read()
    head = src.split('class MountCheck')[0]
    drop = ('import rclpy', 'from rclpy', 'from sensor_msgs')
    head = '\n'.join(ln for ln in head.split('\n')
                     if not ln.startswith(drop))
    ns = {}
    exec(compile(head, 'lidar_mount_check', 'exec'), ns)
    return ns


def ground(x0, k=0.0, lo=-80, hi=80, maxr=10.0):
    """x = x0 + k*y 인 지면 직선 → {방위: 거리}."""
    prof = {}
    for b in range(lo, hi + 1, 5):
        a = math.radians(b)
        den = math.cos(a) - k * math.sin(a)
        if den <= 1e-6:
            continue
        r = x0 / den
        if 0 < r <= maxr:
            prof[b] = r
    return prof


def obj_only(x, halfw):
    """전방 x 에 반폭 halfw 물체 **하나만**. 나머지 방위는 반사 없음
    (빈 들판에 의자 하나 = 9/17 현장 상황)."""
    prof = {}
    for b in range(-80, 81, 5):
        a = math.radians(b)
        y = x * math.tan(a)
        if abs(y) <= halfw:
            prof[b] = math.hypot(x, y)
    return prof


def main():
    ns = _load()
    to_xy, fit = ns['to_xy'], ns['fit_ground_line']
    classify, pitch_deg, roll_deg = ns['classify'], ns['pitch_deg'], ns['roll_deg']
    advice = ns['shim_advice']
    fails = []

    def check(name, ok, detail=''):
        print('  %s %s' % ('OK ' if ok else '\u2717  ', name))
        if not ok:
            if detail:
                print('        %s' % detail)
            fails.append(name)

    print('\n[\u2460 지면 직선 복원] 실측 이력의 값들')
    print('   %-8s %-9s %-9s %s' % ('참 x0', '추정 x0', '피치', '판정'))
    for x0, want_pitch in [(6.30, 2.27), (3.35, 4.27), (2.86, 5.00), (2.33, 6.12)]:
        f = fit(to_xy(ground(x0)))
        kind, _ = classify(f)
        pit = pitch_deg(f['x0'], 0.25)
        print('   %-8.2f %-9.3f %-9.2f %s' % (x0, f['x0'], pit, kind))
        check('x0=%.2f 을 5cm 안으로 복원' % x0, abs(f['x0'] - x0) < 0.05,
              '추정 %.3f' % f['x0'])
        check('x0=%.2f 의 피치가 %.2f°' % (x0, want_pitch),
              abs(pit - want_pitch) < 0.05, '추정 %.2f°' % pit)
        check('x0=%.2f 을 지면으로 판정' % x0, kind == 'ground')

    print('\n[\u2461 롤 부호] 어느 쪽에 심을 넣을지')
    print('   ★ 2026-09-17 정정 — 처음엔 k>0 을 \'좌측이 낮다\' 로 적었는데')
    print('     반대였다. 현장 실측이 반증했다: 우측 지면이 좌측보다 1.48m')
    print('     가까운 상태의 적합이 k=+1.415 였다. 그건 우측이 낮은 것이다.')
    print('     부호를 틀리면 심을 반대쪽에 넣어 **더 나빠진다.**')
    for k, side in [(0.20, '우측'), (-0.40, '좌측')]:
        f = fit(to_xy(ground(3.0, k=k)))
        rol = roll_deg(f['k'], f['x0'], 0.25)
        adv = advice(pitch_deg(f['x0'], 0.25), rol)
        print('   k=%+.2f → 롤 %+.2f°  |  %s' % (k, rol, adv))
        check('k=%+.2f 이면 %s 이 낮다고 말한다' % (k, side), side in adv, adv)
        check('k=%+.2f 부호가 맞다' % k, (rol > 0) == (k > 0))

    print('\n[\u2462 물체 구분] 빈 들판에 물체 하나')
    for x, hw, name in [(2.33, 0.20, '의자'), (2.33, 0.40, '사람'),
                        (1.50, 0.25, '가까운 콘')]:
        pts = to_xy(obj_only(x, hw))
        f = fit(pts)
        kind, why = classify(f, pts)
        print('   %-10s x=%.2f 반폭 %.2f → %s (%s)' % (name, x, hw, kind, why))
        check('%s 를 지면이라고 하지 않는다' % name, kind != 'ground', why)

    print('\n[\u2461b 실측 반증] 2026-09-17 현장 표')
    field = {60: 5.94, 55: 2.24, 50: 5.46, 45: 5.40, 40: 3.50, 35: 4.85,
             30: 4.87, 25: 4.86, 20: 4.70, 15: 4.60, 10: 4.55, 5: 4.54,
             0: 4.52, -5: 4.42, -10: 4.13, -15: 3.54, -20: 3.13, -25: 2.82,
             -30: 2.58, -35: 2.42, -40: 2.41, -45: 3.02, -50: 4.93,
             -55: 4.98, -60: 5.23}
    pairs, asym = ns['side_tilt'](field)
    print('   좌우 지면차 중앙값 %+.2fm (우측이 %s)'
          % (asym, '가깝다' if asym > 0 else '멀다'))
    check('현장 표에서 우측이 낮다고 판정한다', asym > 0.5,
          '중앙값 %+.2fm' % asym)
    fit2, band = ns['best_fit'](field, 10.0)
    check('적합 기울기 부호가 좌우 대칭과 일치한다',
          (fit2['k'] > 0) == (asym > 0),
          'k=%+.3f · 대칭 %+.2f' % (fit2['k'], asym))
    check('그 상태를 롤로 환산하면 우측이 낮다고 말한다',
          '우측' in advice(pitch_deg(fit2['x0'], 0.25),
                           roll_deg(fit2['k'], fit2['x0'], 0.25)),
          advice(pitch_deg(fit2['x0'], 0.25),
                 roll_deg(fit2['k'], fit2['x0'], 0.25)))

    print('\n[\u2463 알려진 한계] 정면 벽은 한 장으로 구분 못 한다')
    f = fit(to_xy(ground(2.33)))
    kind, _ = classify(f)
    check('벽/지면이 한 장으로는 똑같이 보인다 → --confirm 이 필요한 이유',
          kind == 'ground',
          '이게 깨지면 --confirm 을 없애도 되는지 다시 볼 것')

    print()
    if fails:
        print('\u274c 실패 %d건: %s' % (len(fails), ', '.join(fails[:4])))
        return 1
    print('\u2705 전부 통과 — 지면/물체 구분 기하가 설계대로 동작한다')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

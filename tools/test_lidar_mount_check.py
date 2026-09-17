#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_lidar_mount_check.py — 마운트 점검이 실제 고장을 잡는지 확인한다.

검증 방법이 두 겹이다:
  ① 합성 — 지면을 R m 에서 때리도록 만든 스캔을 넣고 판정이 맞는지
  ② 실측 — 답을 아는 두 런의 스캔 기록을 넣고 판정이 맞는지
       drive_0020  지면 5.59m  회피 성공 (AVOID 49% · BLOCKED 12%)
       drive_0312  지면 3.35m  회피 실패 (AVOID 21% · BLOCKED 79%)

  python3 tools/test_lidar_mount_check.py
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drive_fixtures import require_run  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))


def _load():
    """rclpy 없이 판정 함수만 꺼낸다 (import 시 ROS 를 안 띄우게)."""
    import importlib.util
    src = open(os.path.join(os.path.dirname(__file__),
                            'lidar_mount_check.py')).read()
    # ROS import 를 잘라낸다 — 순수 함수만 필요하다
    head = src.split('class MountCheck')[0]
    head = '\n'.join(l for l in head.split('\n')
                     if not l.startswith(('import rclpy', 'from rclpy',
                                          'from sensor_msgs')))
    ns = {}
    exec(compile(head, 'lidar_mount_check', 'exec'), ns)
    return ns


def synth(ground_r, n=360, height=0.25):
    """지면을 ground_r 에서 때리는 스캔 한 장 (방위별 거리)."""
    out = []
    for i in range(n):
        b = -90.0 + 180.0 * i / (n - 1)
        # 아래로 기운 축에서 멀어질수록 지면까지 멀다 — 단순 근사
        r = ground_r / max(0.25, math.cos(math.radians(b) * 0.9))
        out.append((b, min(r, 12.0)))
    return out


def real(tag, lo, hi):
    """기록된 스캔 CSV → 차체 기준 (방위, 거리) 프레임들."""
    import bisect
    # ★ 2026-09-17 — 저장소 사본 우선. 예전엔 /tmp 만 봤고 없으면 None 을
    #   돌려주며 실측 검증이 통째로 조용히 빠졌다(재부팅하면 그렇게 된다).
    path, scan = require_run(tag, need_scan=True)
    POSE = []
    for r in csv.DictReader(open(path)):
        try:
            POSE.append((float(r['t']), float(r['x']), float(r['y']),
                         float(r['yaw_deg']), float(r['v'])))
        except (KeyError, ValueError):
            continue
    T = [p[0] for p in POSE]
    frames = {}
    for r in csv.DictReader(open(scan)):
        try:
            t, x, y = float(r['t']), float(r['x']), float(r['y'])
        except (KeyError, ValueError):
            continue
        if not (lo <= t <= hi):
            continue
        i = min(max(bisect.bisect_left(T, t), 0), len(POSE) - 1)
        _, px, py, yaw, v = POSE[i]
        if abs(v) < 0.4:            # 움직일 때만 — 지면은 따라다닌다
            continue
        a = math.radians(yaw)
        dx, dy = x - px, y - py
        fx = dx * math.cos(a) + dy * math.sin(a)
        fy = -dx * math.sin(a) + dy * math.cos(a)
        if fx <= 0:
            continue
        frames.setdefault(round(t, 1), []).append(
            (math.degrees(math.atan2(fy, fx)), math.hypot(fx, fy)))
    return list(frames.values())


def main():
    ns = _load()
    profile, verdict, in_window = ns['profile'], ns['verdict'], ns['in_window']
    fails = []

    print('① 합성 — 지면을 R 에서 때릴 때 판정')
    print()
    print('   지면 R    최근접    판정         기대')
    print('   ' + '-' * 46)
    for R, want in [(6.30, 0), (7.00, 0), (5.00, 1), (4.20, 1),
                    (3.35, 3), (2.86, 3)]:
        frames = [synth(R) for _ in range(10)]
        prof, _ = profile(frames, 10)
        near = min(prof.values())
        got = verdict(near, 6.0)
        names = {0: '✅정상', 1: '⚠경계', 3: '❌고장'}
        ok = got == want
        print('   %5.2fm   %5.2fm    %s       %s  %s'
              % (R, near, names[got], names[want], 'OK' if ok else '❌'))
        if not ok:
            fails.append('합성 R=%.2f → %s (기대 %s)' % (R, names[got], names[want]))

    print()
    print('② 실측 — 답을 아는 두 런의 스캔 기록')
    print()
    print('   ⚠ 이 둘은 **주행 중** 기록이라 도구의 전제(전방 8m 비움)가')
    print('     깨진다. 그래서 "최근접" 은 지면이 아니라 실제 장애물일 수 있다.')
    print('     대신 두 런을 실제로 가른 지표 — **트랙창 점유** — 를 본다.')
    print('     지면을 때리면 트랙창이 절대 안 비고, planner 가 CLEAR 를')
    print('     한 번도 못 받아 BLOCKED 로 주저앉는다.')
    print()
    cases = [('0020', 0, 999, False, '회피 성공 (AVOID 49% · BLOCKED 12% · CLEAR 39%)'),
             ('0312', 74.0, 93.5, True, '회피 실패 (AVOID 21% · BLOCKED 79% · CLEAR 0%)')]
    any_real = False
    for tag, lo, hi, want_occupied, note in cases:
        frames = real(tag, lo, hi)
        if not frames:
            print('   drive_%s  — 기록 없음, 건너뜀' % tag)
            continue
        any_real = True
        prof, inwin = profile(frames, len(frames))
        occupied = inwin is not None
        ok = occupied == want_occupied
        print('   drive_%s  %d프레임' % (tag, len(frames)))
        print('      트랙창 안 최근접 : %s'
              % ('%.2fm ← 장애물로 잡힌다' % inwin if inwin else '비어 있음'))
        print('      기대            : %s' % ('점유' if want_occupied else '비어 있음'))
        print('      %s   ← %s' % ('OK' if ok else '❌', note))
        print()
        if not ok:
            fails.append('drive_%s 트랙창 %s (기대 %s)'
                         % (tag, '점유' if occupied else '빔',
                            '점유' if want_occupied else '빔'))

    if not any_real:
        print('   ⚠ /tmp 에 주행 기록이 없다 — 합성 검증만 수행했다.')
        print()

    print('=' * 58)
    if fails:
        print('❌ 실패 %d건' % len(fails))
        for f in fails:
            print('   ·', f)
        return 1
    print('✅ 통과 — 고장난 마운트를 고장으로, 정상을 정상으로 판정한다')
    return 0


if __name__ == '__main__':
    sys.exit(main())

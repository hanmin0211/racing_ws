#!/usr/bin/env python3
"""test_aim_clearance.py — 제동 기준이 '조준 방향' 이어야 한다.

왜 이 테스트가 있나 (2026-09-16)
  의자 두 개 사이를 지나가라고 놨는데 차가 **사이에서 섰다**. 원인은 회피
  로직이 아니라 제동에 쓰는 숫자였다:

      front_distance = min(트랙창 ±1.2m 안 '모든' 장애물의 range)

  옆으로 1.0m 비켜난 의자도 '앞이 1.0m 막혔다' 로 읽힌다. longitudinal 이
  그 숫자로 제동을 건다:
      obstacle_stop_dist 0.8m 이하  → return 0.0
      그 위로도 v < ff_deadband 0.05 → serial_bridge 가 PWM 0
  → 안 서는 배치 창이 ±1.00~1.20m, **폭 0.20m** 뿐이었다. 트랙창이 ±1.2m
    라 그 밖은 아예 안 보이므로, 사실상 못 놓는다.

  고친 것: AVOID 일 때 제동 기준을 aim_clearance 로 바꿨다. 조준 방향을 축
  으로 놓고 차폭+여유(safety_radius) 안으로 들어오는 점만 '앞' 으로 센다.

  이 파일이 고정하는 성질:
    ① 의자 문 사이를 지날 때 제동으로 서지 않는다
    ② **정면을 막는 것은 그대로 잡힌다** (안전 반사 유지)
    ③ 비켜가려는 그 방향이 막히면 잡힌다 (조준축이 기준이지 '직진' 이 아니다)
    ④ 폴백: aim_clearance 가 없으면 예전 동작(front_distance)

  python3 tools/test_aim_clearance.py
"""
import math
import sys

import numpy as np

sys.path.insert(0, '/home/han/racing_ws/src/lidar_clustering')
from lidar_clustering.follow_gap_planner import (  # noqa: E402
    FollowGapDecision, FollowGapPlanner)

# bringup.launch.py 와 같은 값
TRACK_WIDTH, LOOKAHEAD, TRIGGER = 2.4, 1.5, 4.0
# longitudinal_controller_node.py / serial_bridge_node.py 의 기본값
V_SLOW, OBS_TRIGGER, OBS_STOP, DEADBAND = 0.8, 4.0, 0.8, 0.05


class _Scan:
    pass


def make_scan(obstacles, free=6.0, n=1440):
    """obstacles = [(전방 x[m], 좌측 y[m], 폭[m])] → 360° LaserScan 흉내."""
    s = _Scan()
    s.angle_min = -math.pi
    s.angle_increment = 2 * math.pi / n
    s.range_min, s.range_max = 0.05, 16.0
    r = np.full(n, float(free))
    for (x, y, w) in obstacles:
        for dy in np.linspace(-w / 2.0, w / 2.0, 60):
            d = math.hypot(x, y + dy)
            raw = math.atan2(y + dy, x) + math.pi
            raw = (raw + math.pi) % (2 * math.pi) - math.pi
            i = int(round((raw - s.angle_min) / s.angle_increment)) % n
            r[i] = min(r[i], d)
    s.ranges = r.tolist()
    return s


def planner():
    return FollowGapPlanner(
        yaw_offset_deg=180.0, front_fov_deg=180.0, min_range=0.30,
        max_range=8.0, track_width=TRACK_WIDTH,
        planning_lookahead=LOOKAHEAD, obstacle_trigger_distance=TRIGGER,
        vehicle_width=0.775, safety_margin=0.25,
        straight_deadband_deg=5.0, min_gap_width_deg=3.0,
        side_score_margin=0.20, aim='path', aim_margin_deg=2.0)


def cmd_speed(obs):
    """longitudinal_controller_node._speed 의 장애물 분기 + serial 의 deadband.

    반환: (명령속도, 실제로 움직이나)
    """
    v = V_SLOW
    if obs < OBS_TRIGGER:
        span = max(1e-3, OBS_TRIGGER - OBS_STOP)
        v = min(v, V_SLOW * max(0.0, obs - OBS_STOP) / span)
        if obs <= OBS_STOP:
            return 0.0, False
    return v, v >= DEADBAND


def case(pl, title, obstacles, expect_move, target_deg=0.0):
    """expect_move: True = 지나가야 한다, False = 서야 한다(안전)."""
    d = pl.plan(make_scan(obstacles), target_deg=target_deg)
    old = float(d.front_distance)
    new = float(d.aim_clearance) if d.aim_clearance >= 0.0 else old
    v_old, mv_old = cmd_speed(old)
    v_new, mv_new = cmd_speed(new)
    ok = (mv_new == expect_move)
    print('  %-34s %-8s  front %5.2fm→v%.3f %-4s │ aim %5.2fm→v%.3f %-4s  %s'
          % (title, d.mode, old, v_old, '진행' if mv_old else '정지',
             new, v_new, '진행' if mv_new else '정지',
             'OK' if ok else '❌ 기대=' + ('진행' if expect_move else '정지')))
    return ok, mv_old, mv_new


def main():
    pl = planner()
    fails = []
    print('안전반경(차폭/2+여유) = %.4fm · 조준창 ±%.1f°\n'
          % (pl.safety_radius,
             math.degrees(math.atan(pl.max_center_y / LOOKAHEAD))))

    # ── ① 의자 문 — 지날 수 있는 문만 지나야 한다 ────────────────────
    #   통과 가능 최소 이격 = 안전반경 0.6375 + 의자 반폭 0.20 = 0.8375m.
    #   그보다 좁으면 **서는 게 맞다** (물리적으로 못 지나간다).
    passable = 0.6375 + 0.20
    print('① 의자 두 개가 만든 문 (의자 폭 0.4m)')
    print('   통과 가능 최소 이격 ±%.4fm — 그보다 좁으면 서는 게 정답\n'
          % passable)
    speedup = []
    impassable_stopped = 0
    for lat in [0.70, 0.80, 0.90, 1.00, 1.10]:
        for x in [2.0, 1.2, 0.8]:
            # 지날 수 있는 문 → 어느 거리에서도 서면 안 된다.
            # 못 지나가는 문 → 멀리서는 감속 중인 게 정상이고,
            #   **가까이(0.8m) 오면 서야 한다.** 2m 밖에서부터 설 이유는 없다.
            want = True if lat >= passable else (x > 1.0)
            ok, mo, mn = case(
                pl, '좌우 ±%.2fm · %.1fm 앞' % (lat, x),
                [(x, lat, 0.4), (x, -lat, 0.4)], expect_move=want)
            if not ok:
                fails.append('문 ±%.2fm @%.1fm — 기대 %s'
                             % (lat, x, '진행' if want else '정지'))
            if not want:
                impassable_stopped += 1
            if want:
                d = pl.plan(make_scan([(x, lat, 0.4), (x, -lat, 0.4)]),
                            target_deg=0.0)
                if d.mode == 'AVOID':
                    speedup.append((cmd_speed(d.front_distance)[0],
                                    cmd_speed(d.aim_clearance)[0]))
    # 못 지나가는 문: 거리가 줄면 명령속도도 줄어야 한다(감속 프로파일)
    for lat in [0.70, 0.80]:
        vs = []
        for x in [2.0, 1.2, 0.8]:
            d = pl.plan(make_scan([(x, lat, 0.4), (x, -lat, 0.4)]),
                        target_deg=0.0)
            base = (d.aim_clearance if d.aim_clearance >= 0.0
                    else d.front_distance)
            vs.append(cmd_speed(base))
        speeds = [v for v, _ in vs]
        # 정확히 0 일 필요는 없다. deadband(0.05) 미만이면 serial_bridge 가
        # PWM 0 을 내므로 실제로 선다. 그게 '섰다' 의 정의다.
        mono = (all(speeds[i] > speeds[i + 1] for i in range(len(speeds) - 1))
                and not vs[-1][1])
        print('   못 지나가는 문 ±%.2fm  v %s  (마지막 %s)  %s'
              % (lat, ' → '.join('%.3f' % v for v in speeds),
                 '움직임' if vs[-1][1] else 'PWM 0 = 정지',
                 'OK (감속해서 선다)' if mono else '❌'))
        if not mono:
            fails.append('±%.2fm 문에서 감속 프로파일이 깨졌다' % lat)
    print()

    if speedup:
        o = sum(a for a, _ in speedup) / len(speedup)
        n = sum(b for _, b in speedup) / len(speedup)
        print('   → 지날 수 있는 문에서 명령속도 평균 %.3f → %.3f m/s (%.1f배)\n'
              % (o, n, n / max(o, 1e-6)))

    # ── ② 정면 장애물 — 반드시 서야 한다 (안전 반사 유지) ────────────
    print('② 정면을 막는 것 — 그대로 서야 한다 ★안전★')
    for x in [0.8, 0.6, 0.5]:
        ok, _, _ = case(pl, '정면 %.1fm · 폭 1.0m (문 없음)' % x,
                        [(x, 0.0, 1.0)], expect_move=False)
        if not ok:
            fails.append('정면 %.1fm 에서 안 선다 — 안전 반사 깨짐' % x)

    # 벽: 트랙을 가로로 완전히 막는다
    for x in [0.8, 0.6]:
        ok, _, _ = case(pl, '가로벽 %.1fm · 폭 3.0m' % x,
                        [(x, 0.0, 3.0)], expect_move=False)
        if not ok:
            fails.append('가로벽 %.1fm 에서 안 선다 — 안전 반사 깨짐' % x)
    print()

    # ── ③ 조준축이 기준이다 (직진이 아니다) ──────────────────────────
    print('③ 조준 방향이 막히면 잡는다 — 기준은 조준축이지 직진이 아니다')
    d = pl.plan(make_scan([(2.5, 0.45, 0.5)]), target_deg=0.0)
    inline_ok = d.mode == 'AVOID' and d.aim_clearance > d.front_distance
    print('   단일 장애물 +0.45m/2.5m   mode=%s  front %.2f → aim %.2f   %s'
          % (d.mode, d.front_distance, d.aim_clearance,
             'OK (비켜가는 쪽은 열려 있다)' if inline_ok else '❌'))
    if not inline_ok:
        fails.append('비켜가는 방향이 열렸는데 aim 이 안 늘었다')

    # 조준 방향에 또 하나 있으면 그건 잡혀야 한다
    d2 = pl.plan(make_scan([(2.5, 0.45, 0.5), (1.5, -0.55, 0.5)]),
                 target_deg=0.0)
    aimed_blocked = (d2.mode != 'AVOID') or (d2.aim_clearance <= 2.2)
    print('   비켜갈 쪽에도 장애물     mode=%s  front %.2f → aim %.2f   %s'
          % (d2.mode, d2.front_distance,
             d2.aim_clearance, 'OK (잡힘)' if aimed_blocked else '❌ 놓쳤다'))
    if not aimed_blocked:
        fails.append('조준 방향의 장애물을 놓쳤다')
    print()

    # ── ④ 폴백 — 계산 안 된 모드는 예전 동작 ─────────────────────────
    print('④ 폴백 — aim_clearance 가 없으면 예전 동작(front_distance)')
    legacy = FollowGapDecision('BLOCKED', 'STOP', 0.0, 0.55,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    fb = (float(legacy.aim_clearance) if legacy.aim_clearance >= 0.0
          else float(legacy.front_distance))
    ok = (legacy.aim_clearance == -1.0) and (fb == 0.55)
    print('   위치인자 10개 생성 → aim_clearance=%.1f, 폴백값 %.2f   %s'
          % (legacy.aim_clearance, fb, 'OK' if ok else '❌'))
    if not ok:
        fails.append('폴백이 깨졌다')

    d3 = pl.plan(make_scan([(0.6, 0.0, 3.0)]), target_deg=0.0)
    ok3 = (d3.mode != 'AVOID' and d3.aim_clearance == -1.0)
    print('   BLOCKED 는 계산 안 함     mode=%s aim=%.1f   %s\n'
          % (d3.mode, d3.aim_clearance, 'OK' if ok3 else '❌'))
    if not ok3:
        fails.append('BLOCKED 인데 aim_clearance 가 채워졌다')

    # ── ⑤ 8초 ESCAPE 가 안 떠야 한다 ────────────────────────────────
    #   cluster_plot_node: AVOID 인데 거리 <= stuck_distance(1.2m) 가
    #   blocked_escape_s(8.0s) 지속되면 '회피를 포기하고 감속을 푼다'.
    #   현장에서 본 8초 멈춤 → 그대로 박고 지나감 이 이 경로다.
    STUCK = 1.2
    print('⑤ 지날 수 있는 문에서 8초 ESCAPE 가 뜨면 안 된다 (stuck %.1fm)' % STUCK)
    for lat in [0.90, 1.00, 1.10]:
        for x in [1.2, 0.8]:
            d = pl.plan(make_scan([(x, lat, 0.4), (x, -lat, 0.4)]),
                        target_deg=0.0)
            if d.mode != 'AVOID':
                continue
            o_stuck = d.front_distance <= STUCK
            n_stuck = d.aim_clearance <= STUCK
            print('   ±%.2fm · %.1fm 앞   front %.2f %-9s → aim %.2f %-9s  %s'
                  % (lat, x, d.front_distance,
                     'ESCAPE발동' if o_stuck else '정상',
                     d.aim_clearance,
                     'ESCAPE발동' if n_stuck else '정상',
                     'OK' if not n_stuck else '❌'))
            if n_stuck:
                fails.append('±%.2fm @%.1fm 에서 여전히 ESCAPE' % (lat, x))
    print()

    print('=' * 70)
    if fails:
        print('❌ 실패 %d건' % len(fails))
        for f in fails:
            print('   ·', f)
        return 1
    print('✅ 통과 — 문은 지나가고, 정면은 그대로 선다')
    return 0


if __name__ == '__main__':
    sys.exit(main())

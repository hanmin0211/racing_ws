#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_mount_check.py — 라이다가 지면을 때리는지 30초 만에 본다.

★ 왜 다시 만들었나 (2026-09-17)
  예전 판은 '트랙창 안 최근접 거리' **하나만** 봤다. 그러면 지면을 때리는 것과
  **앞에 뭐가 놓여 있는 것**을 구분하지 못한다. 둘 다 똑같이 "❌ 고장 — 2.33m"
  으로 찍힌다. 실제로 그 모호함 때문에 한 번 오판했고(핸드오프 함정 #5),
  9/17 측정에서도 같은 질문이 또 나왔다.

  기하를 쓰면 갈린다.

★ 원리
  라이다가 아래로 θ 만큼 숙으면, 스캔 평면과 지면이 만나는 자리는
  **직선**이다. 방위 β 의 반사를 전방거리 x = r·cosβ 로 바꾸면

      x = h / tanθ        (h = 마운트 높이)  → 방위와 무관하게 **일정**

  롤 φ 가 섞이면 그 직선이 기운다:

      x = h·cotθ·cosφ  +  (cotθ·sinφ)·y        (y = r·sinβ, 좌 +)

  즉 전방거리를 횡거리로 회귀하면
      x₀ = 절편  → 피치 θ = atan(h / x₀)
      k  = 기울기 → 롤   φ = atan(k·h / x₀)     ← **어느 쪽에 심을 넣을지**

  그리고 결정적으로:
      · **지면**은 이 직선이 좌우로 **아주 넓게** 이어진다(횡 수 m).
      · **의자·사람**은 좁은 구간에만 찍힌다(횡 0.5m 안팎).
  이 폭이 둘을 가른다.

  ⚠ 한계 — **정면 벽/펜스는 한 장으로는 지면과 구분이 안 된다.** 둘 다 넓은
    평면이다. 그래서 --confirm 을 두었다: 차를 앞으로 2m 옮기고 다시 잰다.
      지면이면 x₀ 가 그대로다 (차체 고정)
      벽이면  x₀ 가 2m 줄어든다 (지도 고정)
    drive_review.py 가 주행 기록에서 쓰는 '지도고정 vs 차체고정' 판정과
    같은 논리를, 세워 놓고 하는 형태로 바꾼 것이다.

사용법
  ① 차를 평평한 곳에 세운다
  ② **전방 8m 안을 비운다** (사람·의자·벽·차 전부)
  ③ 라이다가 돌고 있어야 한다:
       ros2 launch lidar_clustering lidar_dual.launch.py rear:=false

      python3 tools/lidar_mount_check.py
      python3 tools/lidar_mount_check.py --confirm    # 벽인지 지면인지까지 가른다

  판정 (지면으로 확인된 경우)
    ✅ 6.0m 이상   정상 — 그대로 주행
    ⚠ 4.0~6.0m    경계 — 심을 조여라
    ❌ 4.0m 미만   고장 — 이대로 달리면 회피가 BLOCKED 로 죽는다

  실측 이력
      2026-09-13  고장      2.86m (5.0°)   BLOCKED 43 : AVOID 1
      2026-09-13  심 보강   6.30m (2.3°)   AVOID 50 : BLOCKED 0   ← 검증값
      2026-09-16  재발      3.35m (4.3°)   BLOCKED 79% · CLEAR 0%
      2026-09-16  정상 상태로 학교트랙 완주 (drive_0337)
"""
import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# follow_gap_planner 와 같은 값이어야 한다 (bringup.launch.py)
TRACK_HALF_WIDTH = 1.2      # track_width 2.4 / 2
OBSTACLE_TRIGGER = 4.0      # 이 안에 들어오면 장애물로 센다
# ★ 2026-09-17 — planner 의 최소거리와 **같아야 한다** (fg_min_range).
#   follow_gap_planner._prepare_front_scan:144 가 이보다 가까운 반사를 0.0 으로
#   만들고, _find_track_obstacles 는 (ranges > 0.0) 으로 그걸 통째로 뺀다.
#   즉 이 거리 미만은 **플래너에게 아예 안 보인다.**
#   이 도구가 0.05m 부터 세는 바람에, 라이다 17cm 앞의 브래킷을 '트랙창 점유
#   100%' 로 경고했다 — 플래너는 신경도 안 쓰는 것이었다. 기준을 맞춘다.
PLANNER_MIN_RANGE = 0.30

# 평면으로 인정할 최소 횡 폭 [m]. 의자 반폭이 0.20m 이므로 넉넉히 위에 둔다.
PLANE_MIN_SPAN = 2.0
# 평면 적합 잔차 상한 [m]. 이보다 흩어지면 평면이 아니다.
PLANE_MAX_RMS = 0.30
# ★ 2026-09-17 — 지면일 수 **없는** 근접거리.
#   지면 타격거리는 x = h/tanθ 다. 마운트가 아무리 삐뚤어도 하향각 15° 는
#   넘지 않는다(그 정도면 눈으로 보인다). h=0.25 에서 15° 면 x=0.93m 이므로,
#   그보다 가까운 반사는 **지면이 아니라 차체에 붙은 것**이다.
#   현장 실측(2026-09-17): +45°~+60° 에 0.46~0.49m 가 일정하게 잡혔다.
#   그걸 지면으로 치면 하향각 40° 라는 말이 되는데 말이 안 된다.
#   브래킷·심·공구·사람 중 하나다. 그걸 '피치가 높다' 로 읽으면 **엉뚱한 곳에
#   심을 넣게 된다** — 실제로 그렇게 조언할 뻔했다.
MAX_PLAUSIBLE_TILT_DEG = 15.0
# 좌우 판정에 필요한 최소 쌍 수. 한 쌍으로 '좌측이 낮다' 고 말하면 안 된다.
MIN_SIDE_PAIRS = 3


def blind_radius(height):
    """이보다 가까운 반사는 지면일 수 없다 [m] (위 MAX_PLAUSIBLE_TILT_DEG 주석)."""
    return height / math.tan(math.radians(MAX_PLAUSIBLE_TILT_DEG))


def split_near(prof, height):
    """{방위: 거리} → (차체 근접, 나머지). 근접은 지면 판정에서 빼야 한다."""
    lim = blind_radius(height)
    near, rest = {}, {}
    for b, r in prof.items():
        x = r * math.cos(math.radians(b))
        (near if x < lim else rest)[b] = r
    return near, rest


def in_window(bearing_deg, dist, min_range=PLANNER_MIN_RANGE):
    """그 반사가 planner 의 트랙창 안이라 '장애물' 로 잡히는가.

    min_range 미만은 planner 가 버리므로 여기서도 버린다(위 주석).
    """
    if dist < min_range:
        return False
    lat = dist * math.sin(math.radians(bearing_deg))
    fwd = dist * math.cos(math.radians(bearing_deg))
    return abs(lat) <= TRACK_HALF_WIDTH and 0 < fwd <= OBSTACLE_TRIGGER


def profile(frames, got):
    """프레임들 → {방위5도: 거리중앙값}, 트랙창 안 최근접.

    프레임 안에서는 최솟값(가장 가까운 반사), 프레임 사이에서는 중앙값을
    쓴다. 한 프레임 튄 것에 판정이 흔들리지 않게 하기 위함이다.
    """
    bins = {}
    for fr in frames:
        per = {}
        for b, r in fr:
            k = int(round(b / 5.0)) * 5
            per.setdefault(k, []).append(r)
        for k, v in per.items():
            bins.setdefault(k, []).append(min(v))
    prof = {k: statistics.median(v) for k, v in bins.items()
            if len(v) >= max(1, got // 2)}
    inwin = [d for k, d in prof.items() if in_window(k, d)]
    return prof, (min(inwin) if inwin else None)


def verdict(near, min_ok):
    """0 정상 · 1 경계 · 3 고장"""
    if near >= min_ok:
        return 0
    return 1 if near >= 4.0 else 3


# ---------------------------------------------------------------- 평면 기하
def to_xy(prof, max_range=10.0):
    """{방위: 거리} → [(전방 x, 횡 y, 방위)].  좌 +.

    max_range 밖은 버린다 — 멀리 잡힌 건물·나무가 적합을 망친다.
    """
    out = []
    for b, r in prof.items():
        if not (0 < r <= max_range):
            continue
        a = math.radians(b)
        x, y = r * math.cos(a), r * math.sin(a)
        if x <= 0:
            continue
        out.append((x, y, b))
    return out


def _lsq_line(pts):
    """x = x0 + k·y 최소제곱. 점이 모자라면 None."""
    n = len(pts)
    if n < 3:
        return None
    sy = sum(p[1] for p in pts)
    sx = sum(p[0] for p in pts)
    syy = sum(p[1] * p[1] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    den = n * syy - sy * sy
    if abs(den) < 1e-9:
        return None
    k = (n * sxy - sx * sy) / den
    x0 = (sx - k * sy) / n
    return x0, k


def fit_ground_line(pts):
    """전방거리를 횡거리로 회귀 — 지면이면 직선이 나온다.

    이상점(벽 모서리·기둥)이 섞이므로 한 번 맞추고 잔차 큰 것을 떨군 뒤
    다시 맞춘다. 2회면 충분하다 — 현장 도구라 정교함보다 예측가능성이 낫다.
    """
    if len(pts) < 3:
        return None
    work = list(pts)
    res = _lsq_line(work)
    for _ in range(2):
        if res is None:
            return None
        x0, k = res
        errs = [abs(p[0] - (x0 + k * p[1])) for p in work]
        med = statistics.median(errs)
        keep = [p for p, e in zip(work, errs) if e <= max(3.0 * med, 0.10)]
        if len(keep) < 3 or len(keep) == len(work):
            break
        work = keep
        res = _lsq_line(work)
    if res is None:
        return None
    x0, k = res
    errs = [p[0] - (x0 + k * p[1]) for p in work]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    ys = [p[1] for p in work]
    return {
        'x0': x0, 'k': k, 'rms': rms,
        'span': max(ys) - min(ys),
        'y_min': min(ys), 'y_max': max(ys),
        'n': len(work), 'n_total': len(pts),
    }


def best_fit(prof, max_range=10.0):
    """지면 직선을 **가장 잘 보이는 구간에서** 찾는다. (fit, 구간이름)

    ★ 왜 전체를 한 번에 맞추지 않나 (2026-09-17 현장)
      롤이 있으면 한쪽 지면만 가까이 들어오고, 반대쪽은 빔이 지면 위로
      지나가 **물체만** 잡힌다. 그 둘을 직선 하나로 맞추면 잔차가 터져서
      '물체' 로 오판한다(실제로 그랬다: 잔차 1.17m).
      구간을 몇 개 나눠 보고 **가장 깨끗한 평면**을 채택한다.
    """
    bands = [
        ('전체', lambda b: True),
        ('중앙 ±25°', lambda b: abs(b) <= 25),
        ('좌+중앙', lambda b: b >= -25),
        ('우+중앙', lambda b: b <= 25),
    ]
    best = None
    for name, keep in bands:
        sub = {b: r for b, r in prof.items() if keep(b)}
        f = fit_ground_line(to_xy(sub, max_range))
        if f is None or f['span'] < PLANE_MIN_SPAN:
            continue
        if best is None or f['rms'] < best[0]['rms'] - 1e-9:
            best = (f, name)
    if best is None:
        return fit_ground_line(to_xy(prof, max_range)), '전체'
    return best


def pitch_deg(x0, height):
    """지면 직선까지의 전방거리 → 하향각."""
    if x0 <= 0:
        return float('nan')
    return math.degrees(math.atan(height / x0))


def roll_deg(k, x0, height):
    """적합 기울기 → 롤각.  **+ 면 우측이 내려앉은 것이다.**

    ★ 2026-09-17 부호 정정 — 처음에 '+ = 좌측이 낮다' 로 적었는데 반대였다.
      현장 실측(우측 지면 1.85m · 좌측은 지면에 안 닿음)이 k=+1.415 였고,
      그 상태는 명백히 **우측이 낮은** 것이다. 검산:
        x = x0 + k·y (y 좌 +) 에서 k>0 이면 좌측(y>0)의 x 가 크다
        → 좌측 지면이 멀다 → 좌측 빔이 덜 숙였다 → **우측이 낮다**
      부호를 틀리면 심을 **반대쪽에** 넣어 더 나빠진다. 시험으로 박아 뒀다
      (tools/test_mount_geom.py).
    """
    if x0 <= 0:
        return float('nan')
    return math.degrees(math.atan(k * height / x0))


def side_tilt(prof, lo=10, hi=50):
    """좌우 대칭 비교 — 같은 |방위| 에서 어느 쪽 지면이 더 가까운가.

    직선 하나로 맞추는 것보다 이게 튼튼하다. 한쪽에만 지면이 보이고
    반대쪽은 물체만 보이는 상황(2026-09-17 현장)에서도 답이 나온다.
    반환: (쌍 목록, 우측이 더 가까운 평균 차이[m])  — + 면 우측이 낮다.
    """
    pairs = []
    for b in range(lo, hi + 1, 5):
        L, R = prof.get(b), prof.get(-b)
        if L is None or R is None:
            continue
        a = math.radians(b)
        xl, xr = L * math.cos(a), R * math.cos(a)
        pairs.append((b, xl, xr))
    if not pairs:
        return [], 0.0
    return pairs, statistics.median([xl - xr for _, xl, xr in pairs])


def classify(fit, pts=None):
    """'ground' | 'object' | 'none' + 사람이 읽을 사유.

    pts 를 주면 '점이 너무 적어 적합 실패' 를 더 정확히 가른다. 빈 들판에
    의자 하나면 방위 두어 칸만 잡혀 적합이 아예 안 되는데, 그걸 '판정 불가'
    로 내보내면 현장에서 아무것도 못 한다. **가까이 뭔가 있다는 사실 자체가
    답**이므로 '물체' 라고 말해 준다.
    """
    if fit is None:
        near = [p for p in (pts or []) if p[0] <= OBSTACLE_TRIGGER]
        if near:
            xs = [p[0] for p in near]
            return 'object', (f'가까운 반사가 방위 {len(near)}칸 뿐이다 '
                              f'(전방 {min(xs):.2f}~{max(xs):.2f}m) — 평면이 아니다')
        return 'none', '유효한 반사가 모자라다'
    if fit['rms'] > PLANE_MAX_RMS:
        return 'object', f'흩어져 있다 (잔차 {fit["rms"]:.2f}m > {PLANE_MAX_RMS})'
    if fit['span'] < PLANE_MIN_SPAN:
        return 'object', (f'횡 폭이 {fit["span"]:.2f}m 뿐이다 '
                          f'(평면이면 {PLANE_MIN_SPAN}m 이상 이어진다)')
    return 'ground', (f'횡 {fit["span"]:.1f}m 에 걸친 평면 '
                      f'(잔차 {fit["rms"]:.2f}m)')


def window_frames(frames):
    """트랙창 안에 점을 가진 프레임 비율 — 회피에 미치는 **실제** 영향.

    거리 숫자보다 이게 직접적이다. 이 비율이 높으면 planner 가 CLEAR 를
    못 받아 BLOCKED 로 주저앉는다(drive_0312 에서 79%).
    """
    if not frames:
        return 0.0
    hit = 0
    for fr in frames:
        if any(in_window(b, r) for b, r in fr):
            hit += 1
    return hit / len(frames)


def shim_advice(pitch, roll):
    """어디에 심을 넣을지 한 줄로."""
    if not math.isfinite(pitch):
        return ''
    msgs = []
    if pitch > 2.6:
        msgs.append('앞쪽을 들어올리게 심을 끼울 것')
    if math.isfinite(roll) and abs(roll) >= 0.3:
        # roll + = 우측이 낮다 (roll_deg 주석의 부호 정정 참고)
        side = '우측' if roll > 0 else '좌측'
        msgs.append(f'{side}이 {abs(roll):.1f}° 더 내려앉았다 — {side}을 올릴 것')
    return ' · '.join(msgs)


# ---------------------------------------------------------------- ROS 수집
class MountCheck(Node):
    def __init__(self, a):
        super().__init__('lidar_mount_check')
        self.a = a
        self.frames = []
        # BEST_EFFORT 구독자는 RELIABLE 발행자와도 붙는다(요구를 낮추는 쪽).
        # 두 번 걸면 같은 프레임을 두 번 세므로 하나만 건다.
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, a.topic, self.cb, qos)

    def cb(self, msg):
        if len(self.frames) >= self.a.frames:
            return
        yaw = math.radians(self.a.yaw_offset)
        out = []
        ang = msg.angle_min
        for r in msg.ranges:
            b = (ang + yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(b) <= math.pi / 2 and math.isfinite(r) and r > 0.05:
                out.append((math.degrees(b), float(r)))
            ang += msg.angle_increment
        if out:
            self.frames.append(out)


def collect(a):
    """스캔 프레임을 모은다. (frames, 오류문자열)"""
    n = MountCheck(a)
    t0 = n.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok() and len(n.frames) < a.frames:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.get_clock().now().nanoseconds * 1e-9 - t0 > a.timeout:
            break
    frames = list(n.frames)
    n.destroy_node()
    if not frames:
        return None, (f'스캔이 안 온다. {a.topic} 가 발행되는지 확인:\n'
                      f'    ros2 topic hz {a.topic}\n'
                      '    드라이버가 안 떴으면:\n'
                      '    ros2 launch lidar_clustering lidar_dual.launch.py rear:=false')
    return frames, None


def measure(a, label=''):
    """한 번 재고 결과를 dict 로. 화면에도 찍는다."""
    if label:
        print(f'\n── {label} ──')
    print(f' {a.frames}프레임 수집 중... (토픽 {a.topic})')
    frames, err = collect(a)
    if err:
        print(f'\n ❌ {err}')
        return None
    prof, inwin_min = profile(frames, len(frames))
    if not prof:
        print(' ❌ 유효한 반사가 없다 — 라이다 시야가 막혔는지 볼 것.')
        return None
    near, rest = split_near(prof, a.height)
    pts = to_xy(rest, a.max_range)
    fit, band = best_fit(rest, a.max_range)
    kind, why = classify(fit, pts)
    occ = window_frames(frames)
    pairs, asym = side_tilt(rest)
    print(f' {len(frames)}프레임 · 유효 방위 {len(prof)}개 · 트랙창 점유 {occ:.0%}')
    return {'frames': frames, 'prof': prof, 'inwin': inwin_min,
            'fit': fit, 'band': band, 'kind': kind, 'why': why, 'occ': occ,
            'pairs': pairs, 'asym': asym, 'near': near, 'rest': rest}


def show_table(m, a):
    print()
    print(' 방위별 최근접 (좌 + / 우 −)   전방 x = r·cosβ ← 지면이면 일정하다')
    print('  각도    거리    전방 x   창')
    print('  ' + '-' * 52)
    prof = m['prof']
    for k in sorted(prof, reverse=True):
        d = prof[k]
        if abs(k) > 60 and d > a.min_ok:
            continue
        x = d * math.cos(math.radians(k))
        bar = '█' * max(0, int(min(d, 8.0) * 4))
        print('  %+4d°  %5.2fm  %5.2fm  %-3s %s'
              % (k, d, x, '★' if in_window(k, d) else '', bar))


def report(m, a):
    """판정.

    ★ 2026-09-17 — 기준을 **트랙창 안 최근접**으로 바꿨다.
      처음엔 평면 적합의 x0 를 썼는데, 현장 데이터가 직선이 아니었다.
      기울어진 스캔면 ∩ 평평한 지면은 **반드시 직선**이므로, 휘었다는 건
      지면이 고르지 않거나 물체가 섞였다는 뜻이다. 그런 데서 평면을 억지로
      맞추면 잔차만 터지고 판정이 안 나온다.
      실제로 회피를 죽이는 양은 '창 안에 뭐가 얼마나 가까이 있나' 이고
      (drive_0312: 창 안 3.48m → BLOCKED 79%), 그건 적합 없이도 바로 잰다.
      평면 적합·좌우 대칭은 **원인 진단**(피치냐 롤이냐)에 쓴다.
    """
    fit, kind = m['fit'], m['kind']
    inwin, occ = m['inwin'], m['occ']
    pairs, asym = m.get('pairs') or [], m.get('asym', 0.0)

    near = m.get('near') or {}
    print()
    print('=' * 62)
    if near:
        lim = blind_radius(a.height)
        rs = sorted(near.items(), key=lambda kv: kv[0], reverse=True)
        print(' · 라이다 바로 옆(전방 %.2fm 안)에 반사가 있다 — 지면일 수 없다.'
              % lim)
        for b, r in rs[:8]:
            tag = '(플래너 무시: %.2fm 미만)' % PLANNER_MIN_RANGE \
                if r < PLANNER_MIN_RANGE else '★ 플래너가 본다'
            print('      %+4d°  %5.2fm  %s' % (b, r, tag))
        if all(r < PLANNER_MIN_RANGE for _, r in rs):
            print('    → 전부 플래너 최소거리(%.2fm) 미만이라 **회피에는 영향이 없다.**'
                  % PLANNER_MIN_RANGE)
            print('      다만 브래킷·심·케이블이 시야에 걸린 것이니 확인은 해 둘 것.')
        else:
            print('    → 브래킷·심·케이블·공구·사람 중 하나다. **치울 것.**')
        print('      지면 판정은 이 점들을 빼고 했다.')
        print()
    if inwin is None:
        print(' 트랙창      비어 있음 ✓   (전방 %.1fm · 횡 ±%.1fm 안)'
              % (OBSTACLE_TRIGGER, TRACK_HALF_WIDTH))
    else:
        print(' 트랙창 안 최근접  %.2f m   (프레임의 %.0f%% 에서 잡힌다)'
              % (inwin, occ * 100))
        print('   ← planner 가 이걸 장애물로 센다. 검증값: 창이 비어 있었다')

    if fit is not None and fit['x0'] <= 0:
        # 절편이 음수면 '차 뒤에서 만나는 평면' 이라 지면 해석이 불가능하다.
        # 옆 벽처럼 y 가 거의 일정한 선을 맞추면 이렇게 나온다. 안 찍는다.
        print()
        print(' 평면 적합   지면선을 못 찾았다 (옆면/벽이 잡힌 모양)')
        fit, pit = None, float('nan')
    if fit is not None:
        pit = pitch_deg(fit['x0'], a.height)
        print()
        print(' 평면 적합 (%s)   x = %.2f %+.3f·y   잔차 %.2fm'
              % (m.get('band', '전체'), fit['x0'], fit['k'], fit['rms']))
        print('   피치 %.2f°  (검증값 2.27° · 마운트 높이 %.2fm 가정)'
              % (pit, a.height))
        if fit['rms'] > PLANE_MAX_RMS:
            print('   ⚠ 잔차가 크다 — 지면이 고르지 않거나 물체가 섞였다.')
            print('     아래 좌우 대칭으로 판단할 것.')
    else:
        pit = float('nan')

    if pairs and len(pairs) < MIN_SIDE_PAIRS:
        print()
        print(' 좌우 대칭 — 쌍이 %d개뿐이라 판정 보류 (최소 %d개 필요)'
              % (len(pairs), MIN_SIDE_PAIRS))
        pairs, asym = [], 0.0
    if pairs:
        print()
        print(' 좌우 대칭 (같은 |방위| 의 전방거리)')
        print('   |각|    좌 x     우 x     차이')
        for b, xl, xr in pairs:
            print('   %3d°  %6.2fm  %6.2fm  %+6.2fm' % (b, xl, xr, xl - xr))
        if abs(asym) >= 0.3:
            low = '우측' if asym > 0 else '좌측'
            print('   → 중앙값 %+.2fm — **%s 지면이 더 가깝다 = %s이 낮다**'
                  % (asym, low, low))
        else:
            print('   → 좌우 차이 %+.2fm — 대칭이다' % asym)
    print('=' * 62)

    # ---- 무엇을 해야 하나 ----
    todo = []
    if abs(asym) >= 0.5:
        low = '우측' if asym > 0 else '좌측'
        todo.append('**%s을 올려라** (좌우 지면차 %.2fm — 롤이 주원인)'
                    % (low, abs(asym)))
    clean = fit is not None and fit['rms'] <= PLANE_MAX_RMS
    if clean and math.isfinite(pit) and pit > 2.6 and abs(asym) < 1.0:
        todo.append('앞쪽 전체를 들어올려라 (피치 %.1f° > 검증값 2.3°)' % pit)
    elif not clean and not todo:
        todo.append('평면 적합이 지저분하다 — 심을 더 넣기 전에 **앞을 비우고**'
                    ' 다시 잴 것 (지금 숫자로는 어디를 올릴지 못 정한다)')

    if inwin is None:
        print(' ✅ 정상 — 트랙창이 비어 있다. 그대로 주행할 것.')
        if fit is None or fit['rms'] > PLANE_MAX_RMS:
            print('    지면선이 안 보인다 = 빔이 %.0fm 안에서 지면을 안 때린다.'
                  % a.max_range)
            print('    그게 정상이다. (창 밖 반사는 옆 벽·구조물이다)')
        elif math.isfinite(pit):
            print('    지면선 %.2fm (피치 %.2f°)' % (fit['x0'], pit))
        return 0
    if kind == 'object' and not pairs:
        print(' ⚠ 창 안에 뭔가 있는데 평면이 아니다 — 앞에 놓인 물체로 보인다.')
        print('    전방 8m 를 비우고 다시 잴 것. (--confirm 으로 확정 가능)')
        return 2
    v = verdict(inwin, a.min_ok)
    if v == 0:
        print(' ✅ 정상 — 창 안 최근접 %.2fm. 그대로 주행할 것.' % inwin)
        return 0
    if v == 1:
        print(' ⚠ 경계 — 창 안 %.2fm. 심을 조여라.' % inwin)
    else:
        print(' ❌ 고장 — 창 안 %.2fm. 이대로 달리면 회피가 BLOCKED 로 죽는다.'
              % inwin)
        print('    (2026-09-16 실측: 창 안 3.48m → BLOCKED 79% · CLEAR 0%)')
    for t in todo:
        print('    → %s' % t)
    if not todo:
        print('    → 앞쪽을 들어올리게 심을 끼우고 다시 잴 것.')
    return 1 if v == 1 else 3


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/scan_front')
    p.add_argument('--frames', type=int, default=20)
    p.add_argument('--timeout', type=float, default=20.0)
    p.add_argument('--yaw-offset', type=float, default=180.0,
                   help='fg_yaw_offset_deg 와 같게')
    p.add_argument('--height', type=float, default=0.25,
                   help='라이다 마운트 높이[m] — 피치/롤 환산에 쓴다')
    p.add_argument('--min-ok', type=float, default=6.0,
                   help='합격 기준: 이 거리 안에 아무것도 없어야 한다')
    p.add_argument('--max-range', type=float, default=10.0,
                   help='평면 적합에 쓸 최대 거리[m]')
    p.add_argument('--confirm', action='store_true',
                   help='2회 측정 — 사이에 차를 앞으로 옮긴다. '
                        '지면이면 거리가 그대로, 벽이면 그만큼 줄어든다')
    p.add_argument('--move', type=float, default=2.0,
                   help='--confirm 에서 옮길 거리[m]')
    a = p.parse_args()

    print()
    print('=' * 62)
    print(' 라이다 마운트 점검')
    print('=' * 62)
    print(' ⚠ 전방 8m 안에 아무것도 없어야 한다. 있으면 그게 잡힌다.')

    rclpy.init()
    try:
        m1 = measure(a, '1차' if a.confirm else '')
        if m1 is None:
            return 2
        show_table(m1, a)
        rc = report(m1, a)

        if not a.confirm:
            if m1['kind'] == 'ground':
                print()
                print(' ※ 정면에 벽·펜스가 있으면 지면과 구분이 안 된다.')
                print('    확실히 하려면:  python3 tools/lidar_mount_check.py --confirm')
            return rc

        if m1['kind'] == 'none':
            return 2
        print()
        print('─' * 62)
        print(' 차를 **앞으로 %.1fm** 옮기고 Enter.  (밀거나 teleop)' % a.move)
        print('   지면이면 거리가 그대로다 (차체 고정)')
        print('   벽·물체면 %.1fm 줄어든다 (지도 고정)' % a.move)
        print('─' * 62)
        try:
            input(' 준비되면 Enter > ')
        except EOFError:
            print(' (입력 불가 — 2차 측정을 건너뛴다)')
            return rc
        m2 = measure(a, '2차')
        if m2 is None or m2['fit'] is None or m1['fit'] is None:
            return rc
        d1, d2 = m1['fit']['x0'], m2['fit']['x0']
        drop = d1 - d2
        print()
        print('=' * 62)
        print(' 1차 %.2fm → 2차 %.2fm   (변화 %+.2fm · 이동 %.1fm)'
              % (d1, d2, -drop, a.move))
        if abs(drop) < a.move * 0.4:
            print(' → **차체 고정 = 지면이다.** 위 판정이 맞다.')
            print('=' * 62)
            return rc
        if abs(drop - a.move) < a.move * 0.5:
            print(' → **지도 고정 = 앞에 있던 물체/벽이다.** 마운트는 무죄다.')
            print('    그 물체를 치우고 다시 잴 것.')
            print('=' * 62)
            return 2
        print(' → 애매하다. 더 크게(5m) 옮기거나 방향을 바꿔 다시 잴 것.')
        print('=' * 62)
        return rc
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

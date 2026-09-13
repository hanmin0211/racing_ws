#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_orient.py — 라이다 **마운트 방향**을 기준물 하나로 확정한다.

★ 왜 이게 첫 단계인가
  follow-gap 은 `fg_yaw_offset_deg`(기본 180) 로 스캔을 차량 좌표계(+x 전방)
  에 맞춘다. 이 값이 틀리면 **회피가 장애물 쪽으로 꺾는다** — 감점이 아니라
  이탈=탈락이다. 그런데 실내는 어질러져 있어서 스캔만 봐서는 어느 쪽이
  '차 앞' 인지 알 수 없다.

★ 방법 (기준물 하나, 차를 안 움직인다)
  1) 아무것도 놓지 않은 상태를 기준으로 저장  →  --baseline
  2) **차 정면 1~2m** 에 물체를 놓는다 (사람, 박스, 의자 아무거나)
  3) 다시 재서 기준 대비 **새로 생긴 것** 의 방위를 찾는다  →  --check
  차이를 보므로 주변이 복잡해도 걸린다.

사용:
  python3 tools/lidar_orient.py --baseline        # 비운 상태에서
  # ... 차 정면 1~2m 에 물체를 놓고 ...
  python3 tools/lidar_orient.py --check

판정:
  새 물체가 원본각 ~0°   → fg_yaw_offset_deg:=0
  새 물체가 원본각 ~180° → fg_yaw_offset_deg:=180 (기본값)
  그 외                  → 라이다가 비스듬히 달렸다. 그 각을 그대로 넣으면 된다
                           (offset = −(원본각), 부호는 아래 출력이 알려준다)
"""

import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# ★ 2026-09-13 — 앞뒤 라이다를 분리하면서(lidar_dual.launch.py) 전방 스캔이
#   /scan → **/scan_front** 로 바뀌었다. 이 도구는 계속 /scan 을 구독하고
#   있어서 그 뒤로 **아무것도 못 받았다**(조용히 타임아웃만 났다).
#   마운트를 바꿨을 때 제일 먼저 써야 하는 도구가 고장나 있던 셈이다.
DEFAULT_TOPIC = '/scan_front'

STORE = '/tmp/lidar_orient_baseline.json'


def grab(n_avg=10, timeout=20.0, topic=DEFAULT_TOPIC):
    """여러 스캔을 모아 각도 빈별 **중앙값** 거리를 만든다(순간 노이즈 제거)."""
    rclpy.init()
    node = Node('lidar_orient')
    buf = []
    meta = {}

    def cb(m):
        r = np.array(m.ranges, dtype=float)
        r[~np.isfinite(r)] = np.nan
        r[(r <= m.range_min) | (r >= m.range_max)] = np.nan
        buf.append(r)
        meta.setdefault('angle_min', m.angle_min)
        meta.setdefault('angle_increment', m.angle_increment)

    node.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < timeout and len(buf) < n_avg:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(buf) < 2:
        raise RuntimeError('스캔을 못 받았다 — sllidar 드라이버가 도는지 확인할 것')
    m = min(len(b) for b in buf)
    arr = np.vstack([b[:m] for b in buf])
    with np.errstate(invalid='ignore'):
        med = np.nanmedian(arr, axis=0)
    ang = meta['angle_min'] + np.arange(m) * meta['angle_increment']
    return ang, med, len(buf)


def find_moving(a, topic=DEFAULT_TOPIC):
    """시간에 따라 거리가 가장 많이 변한 방향 = 움직이는 사람.

    기준 스캔이 필요 없다. 주변이 복잡해도, 조명이 바뀌어도, **움직이는 것은
    하나뿐**이면 그것만 남는다. 정면에 선 사람이 좌우로 한두 걸음 흔들면 된다.
    """
    rclpy.init()
    node = Node('lidar_moving')
    buf = []
    meta = {}

    def cb(m):
        r = np.array(m.ranges, dtype=float)
        r[~np.isfinite(r)] = np.nan
        r[(r <= m.range_min) | (r >= m.range_max)] = np.nan
        buf.append(r)
        meta.setdefault('amin', m.angle_min)
        meta.setdefault('ainc', m.angle_increment)

    node.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)
    print(f'{a.moving:.0f}초간 측정한다 — **정면에 선 사람이 좌우로 흔들어** 주세요.')
    t0 = time.time()
    while time.time() - t0 < a.moving:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if len(buf) < 10:
        print(f'스캔이 너무 적다 ({len(buf)}개).')
        return 1
    m = min(len(b) for b in buf)
    arr = np.vstack([b[:m] for b in buf])
    ang = meta['amin'] + np.arange(m) * meta['ainc']

    # 각 방향 빈의 '있다/없다' 와 거리 변동을 같이 본다.
    with np.errstate(invalid='ignore'):
        near = arr < a.max_range
        hit_rate = np.nanmean(near.astype(float), axis=0)
        spread = np.nanmax(np.where(near, arr, np.nan), axis=0) \
            - np.nanmin(np.where(near, arr, np.nan), axis=0)
    spread = np.nan_to_num(spread, nan=0.0)
    # 한 번도 안 잡히거나 늘 잡히는 곳은 배경. 들락날락 + 거리변동이 큰 곳이 사람.
    score = spread * (hit_rate * (1.0 - hit_rate) * 4.0 + 0.25)
    score[np.isnan(hit_rate)] = 0.0

    k = int(np.argmax(score))
    # 주변 ±10 빈의 무게중심
    lo, hi = max(0, k - 10), min(m - 1, k + 10)
    sel = np.arange(lo, hi + 1)
    w = score[sel]
    if w.sum() <= 0:
        print('움직임을 못 찾았다. 더 크게 움직이거나 시간을 늘릴 것.')
        return 1
    bearing = math.degrees(float(np.sum(ang[sel] * w) / np.sum(w)))
    with np.errstate(invalid='ignore'):
        dist = float(np.nanmedian(np.where(near[:, sel], arr[:, sel], np.nan)))

    print('=' * 62)
    print(f'가장 많이 움직인 방향: 센서 원본각 {bearing:+.1f}° · '
          f'거리 약 {dist:.2f}m · 변동폭 {spread[k]:.2f}m ({len(buf)}스캔)')
    print('-' * 62)
    _verdict(bearing)
    return 0



def find_body(a):
  """차체 가려짐으로 마운트 방향을 잰다 — **사람도 기준물도 필요 없다.**

  ★ 왜 이게 제일 믿을 만한가 (2026-09-13 학교에서 확인)
    라이다를 차량 정면에 달면 **차체가 라이다 뒤를 넓게 가린다.** 그 구간은
    차에 고정된 강체라 주변이 바뀌어도 안 움직인다. 중심을 재면 그게 '뒤'
    이고, 반대가 정면이다.

    그날 --check 는 +119.1°, --moving 은 -149.7° 를 내놨다(20° 넘게 차이).
    둘 다 **가려짐의 가장자리**를 집은 것이었다 — 그 경계에서는 사람이 조금만
    움직여도 거리가 크게 튀어서 '변화량' 기준 두 방법이 나란히 속는다.
    반면 차체 중심은 전혀 다른 두 장면(벽 앞 / 열린 곳)에서 +10.0° 와 +7.05°
    로 재현됐고, 가려짐 범위가 중심 대비 -59°~+60° 로 **대칭**이었다.
    대칭성 자체가 '차량 중심선을 맞췄다' 는 증거다.

  ⚠ 라이다가 차체에 안 가리는 위치(범퍼 위 등)면 이 방법은 못 쓴다.
    차체 빈이 5% 미만이면 그렇게 알려주고 --moving 으로 보낸다.
  """
  ang, med, valid, n = _grab_valid(a.topic, secs=10.0, n_max=80)
  m = len(ang)
  body = (valid < a.body_valid) & (np.nan_to_num(med, nan=9.0) < a.body_range)
  frac = body.sum() / max(m, 1)
  print('=' * 62)
  print(f'차체 빈 {int(body.sum())}개 / {m} ({frac*100:.0f}%), {n}스캔')
  if frac < 0.05:
    print('-' * 62)
    print('❌ 차체 가려짐이 거의 없다 — 이 방법을 쓸 수 없다.')
    print('   라이다가 차체에 안 가리는 위치인 것 같다(범퍼 위 등).')
    print('   → python3 tools/lidar_orient.py --moving 8 을 쓸 것.')
    print('=' * 62)
    return 1
  th = np.radians(ang[body])
  c = math.degrees(math.atan2(float(np.sin(th).mean()),
                              float(np.cos(th).mean())))
  d = np.degrees(np.angle(np.exp(1j * (th - math.radians(c)))))
  front = ((c + 180.0 + 180.0) % 360.0) - 180.0
  off = -front
  print(f'차체 중심(=뒤) {c:+.2f}°   범위 {d.min():+.0f}~{d.max():+.0f}°'
        f'   비대칭 {abs(d.min()+d.max()):.1f}°')
  if abs(d.min() + d.max()) > 20.0:
    print('  ⚠ 가려짐이 좌우 비대칭이다 — 라이다가 차량 중심선에서 벗어났거나'
          ' 한쪽만 가린다. 값을 그대로 믿지 말고 --moving 으로 교차검증할 것.')
  print('-' * 62)
  print(f'→ 기하학적 정면 {front:+.2f}°')
  print(f'→ fg_yaw_offset_deg := {off:.1f}')
  print('-' * 62)
  corr = ((ang + off + 180.0) % 360.0) - 180.0
  print('보정 후 전방 시야 (여기가 깨끗해야 회피가 산다):')
  for lo, hi, name in [(-15, 15, '정면 ±15°'), (-30, 30, '정면 ±30°'),
                       (-60, -30, '좌 30~60°'), (30, 60, '우 30~60°')]:
    sel = (corr >= lo) & (corr < hi)
    if sel.sum() == 0:
      continue
    vr = float(valid[sel].mean()) * 100.0
    mark = '  ⚠ 가려져 있다' if vr < 80.0 else ''
    print(f'  {name:<11} 유효율 {vr:5.1f}%   '
          f'중앙거리 {np.nanmedian(med[sel]):.2f}m{mark}')
  print('=' * 62)
  return 0


def _grab_valid(topic, secs=10.0, n_max=80):
  """평균 거리와 함께 **유효율**(반사가 돌아온 비율)도 돌려준다."""
  buf, meta = [], {}

  def cb(msg):
    r = np.array(msg.ranges, dtype=float)
    r[(r <= msg.range_min) | (r >= msg.range_max) | ~np.isfinite(r)] = np.nan
    buf.append(r)
    meta.setdefault('a0', msg.angle_min)
    meta.setdefault('ai', msg.angle_increment)

  rclpy.init()
  node = rclpy.create_node('lidar_orient_body')
  node.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)
  t0 = time.time()
  while time.time() - t0 < secs and len(buf) < n_max:
    rclpy.spin_once(node, timeout_sec=0.2)
  node.destroy_node()
  rclpy.shutdown()
  if len(buf) < 5:
    raise RuntimeError(f'스캔을 못 받았다 ({len(buf)}개) — 드라이버 확인')
  m = min(len(b) for b in buf)
  arr = np.vstack([b[:m] for b in buf])
  with np.errstate(invalid='ignore'):
    med = np.nanmedian(arr, axis=0)
  valid = np.isfinite(arr).mean(axis=0)
  ang = np.degrees(meta['a0'] + np.arange(m) * meta['ai'])
  return ang, med, valid, len(buf)


def _verdict(bearing):
    cand = min((0.0, 180.0, -180.0), key=lambda c: abs(
        math.degrees(math.atan2(math.sin(math.radians(bearing - c)),
                                math.cos(math.radians(bearing - c))))))
    err = math.degrees(math.atan2(math.sin(math.radians(bearing - cand)),
                                  math.cos(math.radians(bearing - cand))))
    if abs(err) <= 25.0:
        need = 0.0 if cand == 0.0 else 180.0
        print(f'  → 정면이 원본각 {cand:+.0f}° 쪽이다 (오차 {err:+.1f}°)')
        print(f'  → fg_yaw_offset_deg := {need:.0f}')
        if need == 180.0:
            print('     (현재 기본값과 같다 — 그대로 쓰면 된다)')
        else:
            print('     ⚠ 기본값 180 과 다르다. 반드시 넘길 것: '
                  '-p fg_yaw_offset_deg:=0.0')
    else:
        print('  ⚠ 0°/180° 어느 쪽도 아니다 — 라이다가 비스듬히 달렸다.')
        print(f'  → fg_yaw_offset_deg := {-bearing:+.1f}')
    print('=' * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default=DEFAULT_TOPIC,
                    help=f'스캔 토픽 (기본 {DEFAULT_TOPIC}). '
                         '앞뒤 분리 전 런치를 쓰면 /scan')
    ap.add_argument('--baseline', action='store_true', help='빈 상태 저장')
    ap.add_argument('--check', action='store_true', help='물체 놓고 비교')
    ap.add_argument('--moving', type=float, default=0.0, metavar='SEC',
                    help='이 시간 동안 재서 **가장 많이 움직인 방향**을 찾는다. '
                         '정면에 선 사람이 좌우로 흔들면 그 사람이 잡힌다. '
                         '기준 저장이 필요 없고 주변이 복잡해도 걸린다')
    ap.add_argument('--min-drop', type=float, default=0.30,
                    help='이만큼 가까워진 빈을 "새로 생긴 것"으로 본다[m]')
    ap.add_argument('--body', action='store_true',
                    help='차체 가려짐으로 방향을 잰다 — 기준물도 사람도 '
                         '필요 없고 제일 재현성이 좋다. 라이다가 차체에 '
                         '가리는 위치(정면 마운트)일 때 쓴다')
    ap.add_argument('--body-valid', type=float, default=0.15, metavar='F',
                    help='유효율이 이보다 낮은 빈을 가려진 것으로 본다')
    ap.add_argument('--body-range', type=float, default=0.40, metavar='M',
                    help='가려진 빈에서 가끔 돌아오는 반사의 거리 상한')
    ap.add_argument('--min-range', type=float, default=0.30,
                    metavar='M',
                    help='이보다 가까운 반사는 무시 (기본 0.30 = 라이다 '
                         'min_range). 라이다가 차체를 보면 올릴 것')
    ap.add_argument('--min-points', type=int, default=5, metavar='N',
                    help='덩어리가 이 점수 미만이면 물체로 안 본다 (기본 5). '
                         '점 하나로 방향을 정하는 것을 막는다')
    ap.add_argument('--max-range', type=float, default=3.0,
                    help='이 거리 안의 변화만 본다[m]')
    a = ap.parse_args()

    if a.moving > 0.0:
        return find_moving(a, a.topic)

    if a.body:
        return find_body(a)

    ang, med, n = grab(topic=a.topic)
    if a.baseline or not a.check:
        json.dump({'ang': ang.tolist(), 'med': np.nan_to_num(med, nan=-1).tolist()},
                  open(STORE, 'w'))
        valid = int(np.isfinite(med).sum())
        print(f'기준 저장 ({n}스캔 평균, 유효 {valid}점) → {STORE}')
        print('이제 **차 정면 1~2m** 에 물체를 놓고:')
        print('  python3 tools/lidar_orient.py --check')
        return 0

    if not os.path.exists(STORE):
        print('기준이 없다. 먼저 --baseline 으로 빈 상태를 저장할 것.')
        return 1
    d = json.load(open(STORE))
    base = np.array(d['med'], dtype=float)
    base[base < 0] = np.nan
    m = min(len(base), len(med))
    base, cur, ang = base[:m], med[:m], ang[:m]

    # 기준보다 가까워진(= 뭔가 생긴) 빈
    drop = base - cur
    new = (np.isfinite(cur) & (cur < a.max_range) & (cur >= a.min_range)
           & (~np.isfinite(base) | (drop > a.min_drop)))
    if new.sum() < 3:
        print(f'새로 생긴 것을 못 찾았다 (변화 {int(new.sum())}점).')
        print('물체를 더 가까이(1~2m) 놓거나 --min-drop 을 낮출 것.')
        return 1

    # ★ 2026-09-13 — '가장 가까운 점' 으로 고르던 것을 **가장 큰 덩어리** 로 바꿨다.
    #   그날 학교에서 이 도구가 `원본각 -49.9° · 거리 0.16m · **폭 1점**` 을
    #   내놨다. 1~2m 에 놓은 물체가 아니라 라이다 바로 옆(16cm)의 잡음 점
    #   하나였다. 라이다를 범퍼 위에서 차량 정면으로 옮긴 뒤라 **차체가
    #   스캔에 들어오고** 있었다. 점 하나가 이기는 구조였던 것이다.
    #   이 값은 틀리면 회피가 장애물 쪽으로 꺾는 값이다 — 점 하나로 정하면 안 된다.
    idx = np.where(new)[0]

    # 연속 구간(덩어리)으로 나눈다
    runs = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i == prev + 1:
            prev = i
            continue
        runs.append((start, prev))
        start = prev = i
    runs.append((start, prev))

    def run_info(r):
        lo, hi = r
        sel = np.arange(lo, hi + 1)
        w = 1.0 / np.maximum(cur[sel], 0.05)
        b = math.degrees(float(np.sum(ang[sel] * w) / np.sum(w)))
        return {'lo': lo, 'hi': hi, 'n': hi - lo + 1,
                'bearing': b, 'dist': float(np.min(cur[sel]))}

    cands = sorted((run_info(r) for r in runs),
                   key=lambda c: (-c['n'], c['dist']))

    print('=' * 62)
    print('후보 덩어리 (큰 것부터):')
    for c in cands[:5]:
        mark = ''
        if c['n'] < a.min_points:
            mark = f"  ← {a.min_points}점 미만, 무시"
        print(f"   폭 {c['n']:3d}점  원본각 {c['bearing']:+7.1f}°  "
              f"거리 {c['dist']:5.2f}m{mark}")

    good = [c for c in cands if c['n'] >= a.min_points]
    if not good:
        print('-' * 62)
        print(f'❌ {a.min_points}점 이상인 덩어리가 없다 — 측정 실패다.')
        print('   이 결과로 fg_yaw_offset_deg 를 정하지 말 것.')
        print('   · 물체를 더 크게(사람이 서는 것이 제일 낫다) 1~2m 에 놓을 것')
        print('   · 라이다가 차체를 보고 있으면 --min-range 를 올릴 것'
              f' (지금 {a.min_range:.2f}m)')
        print('   · 그래도 안 되면 --moving 8 로 "사람이 정면에서 좌우로'
              ' 흔드는" 방식을 쓸 것 (기준 저장 불필요)')
        print('=' * 62)
        return 1

    best = good[0]
    bearing, dist = best['bearing'], best['dist']
    print('-' * 62)
    print(f"채택: 원본각 {bearing:+.1f}° · 거리 {dist:.2f}m · 폭 {best['n']}점")
    print('-' * 62)
    _verdict(bearing)
    print('확인: 이 값으로 cluster_plot_node 를 띄우고 tools/lidar_monitor.py 에서')
    print('      정면 물체의 corrected 각이 ~0° 로 나오는지 볼 것.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

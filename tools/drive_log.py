#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_log.py — 주행 1회를 CSV로 남기고 '직진 차선 밟기'의 원인을 판정한다.

왜 필요한가 (2026-08-22 인수인계 3절):
  커브 이탈은 잡았지만 **직진 구간에서 차선을 밟는 문제**가 남았다. 원인 후보는
  제어(추종 튜닝)와 위치추정(헤딩 오프셋 / RTK 품질)인데, **실차 로그가 없으면
  전부 추측**이다. 이 도구는 그 셋을 한 주행에서 동시에 기록해 구분해 준다.

판정 원리:
  · 직진 구간 횡오차의 **평균(부호 포함)이 크다** = 한쪽으로 치우쳐 달린다
      → 정적 편향. 제어가 아니라 위치추정/헤딩 오프셋 문제다.
        (제어가 원인이면 0 주위를 오가므로 평균은 작고 표준편차만 크다)
  · 헤딩 오프셋 오차 δ 는 직진에서 일정한 횡편차로 나타난다. 그래서
    **주행방향(궤적에서 계산) 대 보고된 yaw** 의 차이를 같이 잰다.
  · RTK 가 Float/Single 로 떨어진 구간의 횡오차를 따로 집계한다.

사용 (주행 런치와 별도 터미널, lap_timer 처럼):
  python3 tools/drive_log.py
  Ctrl-C 로 종료하면 요약이 나오고 CSV 경로를 알려준다.

  python3 tools/drive_log.py --replay logs/drive_....csv   # 저장된 로그 재분석
"""

import argparse
import csv
import math
import os
import time
from collections import deque

RTK_FIX = 'FIX'
RTK_FLOAT = 'FLOAT'
RTK_SINGLE = 'SINGLE'
RTK_NONE = '?'

# 직진 판정 기준 [m]. 최대 타각 18°(최소반경 2.42m)인 차라 20m 이상이면
# 조향이 거의 필요 없는 구간이다.
STRAIGHT_R = 20.0
# 곡률을 볼 때 앞뒤로 몇 점을 볼지. 웨이포인트 간격 0.5m 이므로 3 = 앞뒤 1.5m.
CURV_K = 3
# 주행방향을 계산할 최소 이동거리 [m]. 짧으면 GPS 노이즈가 각도로 증폭된다.
COURSE_MIN_D = 0.8

CSV_COLS = [
    't', 'x', 'y', 'yaw_deg', 'cte', 'path_i', 'path_r', 'seg',
    'head_err_deg', 'course_deg', 'course_err_deg', 'speed', 'steer_cmd',
    'lat', 'lon', 'h_acc', 'rtk', 'fix_ok', 'diff_soln', 'yaw_offset_deg',
]


def wrap(a):
  """[-pi, pi) 로 정규화."""
  return (a + math.pi) % (2 * math.pi) - math.pi


def circum_r(p1, p2, p3):
  """세 점의 외접원 반경. 거의 일직선이면 큰 값을 돌려준다."""
  a = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
  b = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
  c = math.hypot(p3[0] - p1[0], p3[1] - p1[1])
  s = (a + b + c) / 2.0
  ar2 = s * (s - a) * (s - b) * (s - c)
  if ar2 <= 1e-12:
    return 1e9
  return (a * b * c) / (4.0 * math.sqrt(ar2))


def seg_project(px, py, ax, ay, bx, by):
  """점을 선분 AB 에 투영. (부호있는 횡거리, 진행방향, 투영비 t) 반환.

  부호는 tracking_monitor_node 와 같다: **+ = 차가 경로 왼쪽**.
  꼭짓점 최근접만 쓰면 간격 0.5m 때문에 최대 25cm 의 종방향 오차가 그대로
  횡오차에 섞인다. 지금 쫓는 게 20~50cm 짜리 편차라 투영이 필요하다.
  """
  vx, vy = bx - ax, by - ay
  L2 = vx * vx + vy * vy
  if L2 < 1e-12:
    return math.hypot(px - ax, py - ay), 0.0, 0.0
  t = ((px - ax) * vx + (py - ay) * vy) / L2
  t = max(0.0, min(1.0, t))
  cx, cy = ax + vx * t, ay + vy * t
  pdir = math.atan2(vy, vx)
  # 경로점 → 차량 벡터의 왼쪽 성분
  dx, dy = px - cx, py - cy
  lateral = -dx * math.sin(pdir) + dy * math.cos(pdir)
  return lateral, pdir, t


# ────────────────────────────── 분석 ──────────────────────────────

def analyze(rows):
  """CSV 행(dict) 목록에서 요약을 출력한다. 실주행/재분석 공용."""
  import numpy as np

  def col(name, rows_):
    return np.array([float(r[name]) for r in rows_
                     if r.get(name) not in (None, '', 'nan')])

  print('\n' + '=' * 64)
  if not rows:
    print('데이터 없음 — /odometry/filtered 와 /global_path 가 필요하다.')
    print('=' * 64)
    return

  # 실제로 움직인 구간만 본다. 정지 중 샘플이 평균을 희석한다.
  moving = [r for r in rows if float(r['speed']) > 0.15]
  if not moving:
    print(f'샘플 {len(rows)}개 — 전부 정지 상태였다(속도 0.15m/s 미만).')
    print('=' * 64)
    return

  dist = 0.0
  prev = None
  for r in moving:
    xy = (float(r['x']), float(r['y']))
    if prev:
      d = math.hypot(xy[0] - prev[0], xy[1] - prev[1])
      if d < 1.0:            # 튐 방지
        dist += d
    prev = xy

  st = [r for r in moving if r['seg'] == 'straight']
  cv = [r for r in moving if r['seg'] == 'curve']

  print(f'주행 로그 분석 — 샘플 {len(moving)}개 / 이동 {dist:.1f}m')
  print('-' * 64)

  def cte_line(label, rs):
    if not rs:
      print(f'  {label}: 샘플 없음')
      return None
    e = col('cte', rs)
    print(f'  {label} ({len(rs)}샘플)')
    print(f'      평균(부호) {e.mean():+.3f} m   표준편차 {e.std():.3f} m')
    print(f'      평균(절대) {np.abs(e).mean():.3f} m   '
          f'최대 {np.abs(e).max():.3f} m   95% {np.percentile(np.abs(e), 95):.3f} m')
    return e

  print('[횡방향 오차]  (+ = 경로 왼쪽으로 벗어남)')
  e_st = cte_line('직진 구간', st)
  e_cv = cte_line('커브 구간', cv)

  # ── 핵심 판정: 직진에서 편향이냐 진동이냐 ──
  print('-' * 64)
  print('[판정] 직진 구간이 한쪽으로 치우쳤나?')
  if e_st is None or len(e_st) < 20:
    print('  직진 샘플이 부족하다(20개 미만). 한 바퀴를 더 돌 것.')
  else:
    bias, sd = e_st.mean(), e_st.std()
    side = '왼쪽' if bias > 0 else '오른쪽'
    print(f'  편향 {bias:+.3f} m ({side}),  진동폭(표준편차) {sd:.3f} m')
    if abs(bias) < 0.10:
      print('  → 편향 거의 없음. 직진 차선 밟기가 남아 있다면 원인은 편향이')
      print('     아니라 진동이다(제어 튜닝: lookahead / 조향 게인).')
    elif abs(bias) > sd:
      print(f'  → ★ 정적 편향이 지배적이다({side}으로 일정하게 치우쳐 달린다).')
      print('     제어가 아니라 위치추정 쪽이다 — 아래 헤딩/RTK 항목을 볼 것.')
    else:
      print('  → 편향과 진동이 섞여 있다. 아래 헤딩 오차부터 확인할 것.')
    if sd > 0.25:
      print(f'  ⚠ 진동폭 {sd:.2f} m 가 크다 — 제어 튜닝 여지도 있다.')

  # ── 헤딩: 보고된 yaw vs 실제 주행방향 ──
  print('-' * 64)
  print('[헤딩]  보고 yaw − 실제 주행방향  (yaw_offset 검증)')
  he = [r for r in st if r.get('course_err_deg') not in (None, '', 'nan')
        and abs(float(r['steer_cmd'])) < 5.0]
  if len(he) < 20:
    print('  직진·저조향 샘플 부족 — 판정 불가.')
  else:
    ce = np.array([float(r['course_err_deg']) for r in he])
    # 각도 평균은 wrap 을 고려해 벡터 평균으로
    m = math.degrees(math.atan2(np.sin(np.radians(ce)).mean(),
                                np.cos(np.radians(ce)).mean()))
    print(f'  평균 {m:+.2f}°   표준편차 {ce.std():.2f}°   ({len(he)}샘플)')
    if abs(m) < 1.0:
      print('  → yaw_offset 정상. 헤딩은 원인이 아니다.')
    else:
      print(f'  → ★ yaw 가 실제 주행방향과 {abs(m):.1f}° 어긋나 있다.')
      print('     헤딩 캘리브(10m 직진) 품질 문제다. 재캘리브 후 재측정할 것.')
      print(f'     참고: 1° 어긋나면 10m 진행에 {10*math.tan(math.radians(abs(m))):.2f}m 가 밀린다.')

  # ── 경로 대비 자세각(추종 상태) ──
  he2 = col('head_err_deg', st)
  if len(he2) >= 20:
    m2 = math.degrees(math.atan2(np.sin(np.radians(he2)).mean(),
                                 np.cos(np.radians(he2)).mean()))
    print(f'  (참고) 경로방향 대비 차체각 평균 {m2:+.2f}°')

  # ── RTK 품질 ──
  print('-' * 64)
  print('[RTK 품질]')
  klass = {}
  for r in moving:
    klass.setdefault(r['rtk'] or RTK_NONE, []).append(r)
  if set(klass) == {RTK_NONE}:
    print('  RTK 상태를 못 받았다 — /ubx_nav_hp_pos_llh 가 없었다.')
    print('  (ngii_rtk.launch.py 가 떠 있었는지 확인)')
  else:
    for k in (RTK_FIX, RTK_FLOAT, RTK_SINGLE, RTK_NONE):
      rs = klass.get(k)
      if not rs:
        continue
      pct = 100.0 * len(rs) / len(moving)
      e = col('cte', rs)
      ha = col('h_acc', rs)
      acc = f'   h_acc 평균 {ha.mean():.3f} m' if len(ha) else ''
      print(f'  {k:6s} {pct:5.1f}%   횡오차(절대) 평균 '
            f'{np.abs(e).mean():.3f} m{acc}')
    fix_pct = 100.0 * len(klass.get(RTK_FIX, [])) / len(moving)
    if fix_pct < 90.0:
      print(f'  → ★ RTK Fixed 가 {fix_pct:.0f}% 뿐이다. Float/Single 구간은')
      print('     위치오차가 수십 cm~m 라 어떤 제어로도 차선을 못 지킨다.')
      print('     NTRIP 연결·안테나 시야를 먼저 볼 것.')
    else:
      print(f'  → RTK Fixed {fix_pct:.0f}%. GPS 품질은 원인이 아니다.')

  print('=' * 64)


# ────────────────────────────── ROS 수집 ──────────────────────────────

def run_ros(out_path):
  import numpy as np
  import rclpy
  from geometry_msgs.msg import Twist
  from nav_msgs.msg import Odometry, Path
  from rclpy.executors import ExternalShutdownException
  from rclpy.node import Node
  from rclpy.qos import DurabilityPolicy, QoSProfile
  from sensor_msgs.msg import NavSatFix
  from std_msgs.msg import Float64

  try:
    from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavStatus
    HAVE_UBX = True
  except ImportError:                      # 드라이버 미설치 환경에서도 돌게
    HAVE_UBX = False

  class DriveLog(Node):

    def __init__(self):
      super().__init__('drive_log')
      self.wps = np.empty((0, 2))
      self.rows = []
      self.t0 = time.time()
      self.speed = 0.0
      self.steer = 0.0
      self.yaw_off = float('nan')
      self.fix = None
      self.h_acc = float('nan')
      self.rtk = RTK_NONE
      self.fix_ok = ''
      self.diff_soln = ''
      self.trail = deque(maxlen=200)
      self.cmd_seen = False

      latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
      self.create_subscription(Path, '/global_path', self.path_cb, latched)
      self.create_subscription(Float64, '/heading/yaw_offset',
                               self.off_cb, latched)
      self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
      self.create_subscription(Float64, '/current_speed', self.spd_cb, 10)
      self.create_subscription(Float64, '/steering_cmd', self.str_cb, 10)
      self.create_subscription(NavSatFix, '/fix', self.fix_cb,
                               rclpy.qos.qos_profile_sensor_data)
      self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
      if HAVE_UBX:
        self.create_subscription(UBXNavHPPosLLH, '/ubx_nav_hp_pos_llh',
                                 self.hp_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(UBXNavStatus, '/ubx_nav_status',
                                 self.st_cb, rclpy.qos.qos_profile_sensor_data)
      self.create_timer(2.0, self.report)
      self.get_logger().info(f'주행 로거 시작 — 저장: {out_path}')
      if not HAVE_UBX:
        self.get_logger().warn('ublox_ubx_msgs 없음 — RTK 품질은 /fix 공분산으로 대체')

    # ── 콜백 ──
    def path_cb(self, m):
      self.wps = (np.array([[p.pose.position.x, p.pose.position.y]
                            for p in m.poses])
                  if m.poses else np.empty((0, 2)))

    def off_cb(self, m):
      self.yaw_off = math.degrees(float(m.data))

    def spd_cb(self, m):
      self.speed = abs(float(m.data))

    def str_cb(self, m):
      self.steer = float(m.data)

    def cmd_cb(self, m):
      if not self.cmd_seen and m.linear.x > 0.05:
        self.cmd_seen = True
        self.get_logger().info('▶ 출발 감지')

    def fix_cb(self, m):
      self.fix = (m.latitude, m.longitude)
      if not HAVE_UBX:
        var = float(m.position_covariance[0])
        if var > 0.0:
          self.h_acc = math.sqrt(var)
          self.rtk = self.classify(self.h_acc)

    def hp_cb(self, m):
      # h_acc 는 '0.1mm' 단위 (mm scale 0.1) → m 변환은 ×1e-4
      self.h_acc = float(m.h_acc) * 1e-4
      self.rtk = self.classify(self.h_acc)

    def st_cb(self, m):
      self.fix_ok = int(bool(m.gps_fix_ok))
      self.diff_soln = int(bool(m.diff_soln))

    @staticmethod
    def classify(h):
      if h < 0.05:
        return RTK_FIX
      if h < 0.5:
        return RTK_FLOAT
      return RTK_SINGLE

    # ── 본체 ──
    def odom_cb(self, msg):
      if len(self.wps) < 2 * CURV_K + 1:
        return
      x = msg.pose.pose.position.x
      y = msg.pose.pose.position.y
      q = msg.pose.pose.orientation
      yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                       1 - 2 * (q.y * q.y + q.z * q.z))

      # 최근접 꼭짓점 → 양옆 선분에 투영해 더 정확한 횡오차를 얻는다
      d = np.hypot(self.wps[:, 0] - x, self.wps[:, 1] - y)
      i = int(np.argmin(d))
      best = None
      for a in (i - 1, i):
        b = a + 1
        if a < 0 or b >= len(self.wps):
          continue
        lat, pdir, _ = seg_project(x, y, self.wps[a, 0], self.wps[a, 1],
                                   self.wps[b, 0], self.wps[b, 1])
        if best is None or abs(lat) < abs(best[0]):
          best = (lat, pdir)
      if best is None:
        return
      cte, pdir = best

      lo, hi = max(0, i - CURV_K), min(len(self.wps) - 1, i + CURV_K)
      R = circum_r(self.wps[lo], self.wps[i], self.wps[hi])
      seg = 'straight' if R > STRAIGHT_R else 'curve'

      # 실제 주행방향: 궤적에서 COURSE_MIN_D 이상 떨어진 과거점과 비교
      self.trail.append((x, y))
      course = float('nan')
      course_err = float('nan')
      for px, py in reversed(self.trail):
        if math.hypot(x - px, y - py) >= COURSE_MIN_D:
          course = math.atan2(y - py, x - px)
          course_err = math.degrees(wrap(yaw - course))
          break

      self.rows.append({
          't': f'{time.time() - self.t0:.3f}',
          'x': f'{x:.3f}', 'y': f'{y:.3f}',
          'yaw_deg': f'{math.degrees(yaw):.2f}',
          'cte': f'{cte:.4f}',
          'path_i': i,
          'path_r': f'{min(R, 9999.0):.2f}',
          'seg': seg,
          'head_err_deg': f'{math.degrees(wrap(yaw - pdir)):.2f}',
          'course_deg': '' if math.isnan(course) else f'{math.degrees(course):.2f}',
          'course_err_deg': '' if math.isnan(course_err) else f'{course_err:.2f}',
          'speed': f'{self.speed:.3f}',
          'steer_cmd': f'{self.steer:.2f}',
          'lat': f'{self.fix[0]:.8f}' if self.fix else '',
          'lon': f'{self.fix[1]:.8f}' if self.fix else '',
          'h_acc': '' if math.isnan(self.h_acc) else f'{self.h_acc:.4f}',
          'rtk': self.rtk,
          'fix_ok': self.fix_ok,
          'diff_soln': self.diff_soln,
          'yaw_offset_deg': '' if math.isnan(self.yaw_off) else f'{self.yaw_off:.2f}',
      })

    def report(self):
      if not self.rows:
        self.get_logger().info(
            f'대기중 — 경로 {len(self.wps)}점 / '
            f'{"오도메트리 수신중" if self.rows else "오도메트리 대기"}')
        return
      r = self.rows[-1]
      self.get_logger().info(
          f'{r["seg"]:8s} 횡오차 {float(r["cte"]):+.2f}m | '
          f'속도 {r["speed"]}m/s | 조향 {float(r["steer_cmd"]):+.1f}° | '
          f'RTK {r["rtk"]} | 샘플 {len(self.rows)}')

    def save(self):
      if not self.rows:
        return
      os.makedirs(os.path.dirname(out_path), exist_ok=True)
      with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLS)
        w.writeheader()
        w.writerows(self.rows)

  rclpy.init()
  n = DriveLog()
  try:
    rclpy.spin(n)
  except (KeyboardInterrupt, ExternalShutdownException):
    # Ctrl-C 는 rclpy 신호처리기를 거치면 KeyboardInterrupt 가 아니라
    # ExternalShutdownException 으로 온다. 둘 다 잡아야 요약이 깨끗이 나온다.
    pass
  finally:
    n.save()
    analyze(n.rows)
    if n.rows:
      print(f'\nCSV 저장: {out_path}  ({len(n.rows)}행)')
      print('재분석: python3 tools/drive_log.py --replay ' + out_path)
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


def main():
  ap = argparse.ArgumentParser(description='주행 로그 기록 및 추종 원인 분석')
  ap.add_argument('--replay', help='저장된 CSV 를 다시 분석만 한다')
  ap.add_argument('--out', help='CSV 저장 경로(기본 logs/drive_<시각>.csv)')
  a = ap.parse_args()

  if a.replay:
    with open(a.replay) as f:
      analyze(list(csv.DictReader(f)))
    return

  out = a.out or os.path.join(
      os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
      'logs', time.strftime('drive_%Y%m%d_%H%M%S.csv'))
  run_ros(out)


if __name__ == '__main__':
  main()

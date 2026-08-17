#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gps_check.py — RTK 품질 실시간 점검. 웨이포인트 찍기 전/중에 띄워두는 용도.

1초마다 한 줄로:
  · 수신율(Hz)      — 10Hz 가 정상. 낮으면 USB/드라이버 문제
  · 수평 표준편차 σ — RTK 판정의 실질 기준
  · RTCM 수신율     — NTRIP 이 실제로 F9P 까지 닿는지
  · 위경도 / 로컬 x,y (웨이포인트와 같은 원점 기준)

★ RTK 판정을 status.status 로 하지 않는 이유
  지금 쓰는 ublox_nav_sat_fix_hp 드라이버는 RTK Fixed 에서도 status=1(SBAS)만
  내보내고 2(GBAS)를 주지 않는다. 2026-08-17 실측에서 공분산 2.9mm(명백한
  Fixed)인데 status=1 이었다. 그래서 공분산으로만 판정한다.

  σ ≤ 5cm    → FIXED  (웨이포인트 기록 가능)
  σ ≤ 50cm   → FLOAT  (기록하면 경로가 흔들린다)
  그 이상     → 단독측위 — NTRIP/안테나 확인

사용:
  python3 tools/gps_check.py
  python3 tools/gps_check.py --ros-args -p fixed_threshold:=0.03
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

try:
  from pyproj import Transformer
except ImportError:                                  # pyproj 없으면 위경도만
  Transformer = None


class GpsCheck(Node):

  def __init__(self):
    super().__init__('gps_check')
    self.declare_parameter('fix_topic', '/fix')
    self.declare_parameter('utm_epsg', 32652)
    self.declare_parameter('origin_x', 399848.522)
    self.declare_parameter('origin_y', 4092209.171)
    self.declare_parameter('fixed_threshold', 0.05)   # FIXED 판정 [m]
    self.declare_parameter('float_threshold', 0.50)   # FLOAT 판정 [m]

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.origin_x = float(g('origin_x'))
    self.origin_y = float(g('origin_y'))
    self.fixed_th = float(g('fixed_threshold'))
    self.float_th = float(g('float_threshold'))

    self.tf = None
    if Transformer is not None:
      self.tf = Transformer.from_crs('EPSG:4326', f'EPSG:{int(g("utm_epsg"))}',
                                     always_xy=True)

    self.n_fix = 0
    self.n_rtcm = 0
    self.last = None
    self.t_win = time.time()
    self.fixed_time = 0.0      # FIXED 상태로 누적된 시간 [s]
    self.total_time = 0.0

    self.create_subscription(NavSatFix, str(g('fix_topic')), self.fix_cb,
                             qos_profile_sensor_data)
    self._sub_rtcm()
    self.create_timer(1.0, self.report)
    print('GPS 점검 시작 — Ctrl-C 로 종료 (요약 출력)\n')

  def _sub_rtcm(self):
    """RTCM 메시지 타입은 환경마다 달라서 있으면 붙이고 없으면 조용히 넘어간다."""
    try:
      from rtcm_msgs.msg import Message as Rtcm
    except ImportError:
      try:
        from mavros_msgs.msg import RTCM as Rtcm
      except ImportError:
        return
    self.create_subscription(Rtcm, '/ntrip_client/rtcm',
                             lambda _m: setattr(self, 'n_rtcm', self.n_rtcm + 1),
                             10)

  def fix_cb(self, msg):
    if math.isnan(msg.latitude):
      return
    self.n_fix += 1
    h_std = math.sqrt(max(0.0, msg.position_covariance[0]
                          + msg.position_covariance[4]))
    xy = None
    if self.tf is not None:
      ux, uy = self.tf.transform(msg.longitude, msg.latitude)
      xy = (ux - self.origin_x, uy - self.origin_y)
    self.last = (msg.latitude, msg.longitude, msg.altitude, h_std,
                 msg.status.status, xy)

  def verdict(self, h_std):
    if h_std <= self.fixed_th:
      return 'FIXED ✅'
    if h_std <= self.float_th:
      return 'FLOAT ⚠'
    return '단독측위 ❌'

  def report(self):
    now = time.time()
    dt = now - self.t_win
    self.t_win = now
    hz = self.n_fix / dt if dt > 0 else 0.0
    rtcm_hz = self.n_rtcm / dt if dt > 0 else 0.0
    self.n_fix = self.n_rtcm = 0

    if self.last is None:
      print('  /fix 수신 없음 — ublox 드라이버 확인')
      return
    lat, lon, alt, h_std, status, xy = self.last
    v = self.verdict(h_std)
    self.total_time += dt
    if h_std <= self.fixed_th:
      self.fixed_time += dt
    loc = f'x={xy[0]:9.2f} y={xy[1]:9.2f}' if xy else '(pyproj 없음)'
    print(f'  {v:11s} σ={h_std * 100:6.1f}cm | {hz:4.1f}Hz | '
          f'RTCM {rtcm_hz:4.1f}Hz | {loc} | {lat:.7f},{lon:.7f} {alt:6.1f}m '
          f'| status={status}')

  def summary(self):
    print('\n' + '=' * 68)
    print('GPS 점검 요약')
    print('-' * 68)
    if self.last is None or self.total_time < 1.0:
      print('  데이터 부족')
      print('=' * 68)
      return
    pct = self.fixed_time / self.total_time * 100
    print(f'  관측 시간      : {self.total_time:.0f}s')
    print(f'  FIXED 유지율   : {pct:.1f}%')
    print(f'  마지막 수평 σ  : {self.last[3] * 100:.1f}cm  → {self.verdict(self.last[3])}')
    print('-' * 68)
    if pct >= 95:
      print('  ✅ 웨이포인트 기록 진행해도 좋다.')
    elif pct >= 60:
      print('  ⚠ FIXED 가 끊긴다. 기록하면 경로에 튐이 섞인다.')
      print('    하늘이 트인 곳인지, 안테나 케이블/지판이 정상인지 확인.')
    else:
      print('  ❌ RTK 가 거의 안 잡힌다. NTRIP 접속과 RTCM 수신부터 확인.')
      print('    (NTRIP 클라이언트가 중복 실행되면 401 로 RTCM 이 0 이 된다:')
      print('     bash tools/ros_cleanup.sh 후 재실행)')
    print('=' * 68)


def main():
  rclpy.init()
  node = GpsCheck()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.summary()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

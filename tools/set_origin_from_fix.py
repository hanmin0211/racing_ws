#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""set_origin_from_fix.py — 현장에서 RTK fix 로 로컬 원점을 세팅한다.

★ 왜 필요한가
  원점(config/site_origin.yaml)이 남의 장소로 남아 있으면 좌표가 150km 어긋난다
  (2026-08-17 충주 원점이 대구권에 남아 x=78019 로 찍힌 실제 사고). 장소가 바뀔
  때 이 값을 손으로 고치다 실수하는 걸 막으려고, 현장 fix 로 자동 계산한다.

★ 동작
  /fix 를 받아 σ≤threshold 인 양질 fix 를 N개 모아 위경도를 평균 → UTM52N 변환
  → 100m 단위로 내림(경로가 0~수백 m 에 들어오게) → 원점 후보 산출.
  기본은 **미리보기(dry-run)**. 값이 타당하면 --write 로 site_origin.yaml 에 쓴다.
  쓰기 전 기존 파일을 .bak 로 백업한다.

★ 순서 (현장, 예: 용인 9/19)
  1) bringup 없이 GPS 만 올려도 되고, 평소처럼 올려도 된다. /fix 만 나오면 됨.
  2) python3 tools/set_origin_from_fix.py            # 미리보기
  3) 값 확인(σ FIXED, 위경도가 용인인지) 후:
     python3 tools/set_origin_from_fix.py --write --site "용인 (2026-09-19)"
  4) 웨이포인트는 이 원점으로 **새로 기록**한다(옛 파일 재사용 안 함).

사용:
  python3 tools/set_origin_from_fix.py [--write] [--site 이름]
                                       [--samples 20] [--sigma 0.05]
                                       [--floor 100] [--fix-topic /fix]
"""

import argparse
import math
import os
import shutil
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

try:
  from pyproj import Transformer
except ImportError:
  print('pyproj 가 필요하다: pip install pyproj', file=sys.stderr)
  raise

SITE_YAML = '/home/han/racing_ws/config/site_origin.yaml'
UTM_EPSG = 32652  # 한국은 대부분 UTM 52N


def floor_to(v, step):
  """step 단위로 내림. step<=0 이면 그대로."""
  return math.floor(v / step) * step if step > 0 else v


def compute_origin(lat, lon, epsg=UTM_EPSG, floor=100.0):
  """위경도 → (origin_x, origin_y, utm_x, utm_y). 원점은 floor 단위 내림."""
  tf = Transformer.from_crs('EPSG:4326', f'EPSG:{int(epsg)}', always_xy=True)
  ux, uy = tf.transform(lon, lat)
  return floor_to(ux, floor), floor_to(uy, floor), ux, uy


class OriginSetter(Node):

  def __init__(self, args):
    super().__init__('set_origin_from_fix')
    self.args = args
    self.samples = []          # (lat, lon, sigma)
    self.n_seen = 0
    self.done = False
    self.create_subscription(NavSatFix, args.fix_topic, self.cb,
                             qos_profile_sensor_data)
    print(f'/fix 수신 대기 — σ≤{args.sigma * 100:.0f}cm 인 fix {args.samples}개 수집\n')
    self.create_timer(1.0, self.tick)
    self.t0 = time.time()

  def cb(self, msg):
    if self.done or math.isnan(msg.latitude):
      return
    self.n_seen += 1
    sigma = math.sqrt(max(0.0, msg.position_covariance[0]
                          + msg.position_covariance[4]))
    if sigma <= self.args.sigma:
      self.samples.append((msg.latitude, msg.longitude, sigma))

  def tick(self):
    if self.done:
      return
    n = len(self.samples)
    last_sig = self.samples[-1][2] * 100 if n else float('nan')
    print(f'  수집 {n:3d}/{self.args.samples}  (수신 {self.n_seen}, '
          f'최근 σ={last_sig:.1f}cm)')
    if n >= self.args.samples:
      self.finish()
    elif time.time() - self.t0 > self.args.timeout:
      print(f'\n⏱ {self.args.timeout:.0f}s 안에 양질 fix 를 {self.args.samples}개 '
            f'못 모았다. RTK(σ≤{self.args.sigma * 100:.0f}cm) 상태부터 확인.')
      self.done = True
      rclpy.shutdown()

  def finish(self):
    self.done = True
    lat = sum(s[0] for s in self.samples) / len(self.samples)
    lon = sum(s[1] for s in self.samples) / len(self.samples)
    sig = sum(s[2] for s in self.samples) / len(self.samples)
    ox, oy, ux, uy = compute_origin(lat, lon, UTM_EPSG, self.args.floor)

    print('\n' + '=' * 64)
    print('원점 후보')
    print('-' * 64)
    print(f'  평균 위경도 : {lat:.7f}, {lon:.7f}   (평균 σ={sig * 100:.1f}cm)')
    print(f'  UTM52N      : {ux:.2f}, {uy:.2f}')
    print(f'  원점(내림{self.args.floor:.0f}m) : origin_x={ox:.1f}  origin_y={oy:.1f}')
    print(f'  이 원점 기준 현재 위치 : x={ux - ox:.2f}, y={uy - oy:.2f}  '
          '(0~수백 m 면 정상)')
    print('=' * 64)

    if not self.args.write:
      print('\n미리보기다. 값이 맞으면 --write 를 붙여 실제로 써라:')
      print(f'  python3 tools/set_origin_from_fix.py --write '
            f'--site "{self.args.site}"')
    else:
      self.write_yaml(ox, oy, lat, lon)
    rclpy.shutdown()

  def write_yaml(self, ox, oy, lat, lon):
    if os.path.exists(SITE_YAML):
      bak = SITE_YAML + '.bak'
      shutil.copy2(SITE_YAML, bak)
      print(f'\n기존 원점 백업 → {bak}')
    body = (
        '# 로컬 좌표계 원점 — 이 파일이 유일한 정본.\n'
        '# set_origin_from_fix.py 가 현장 RTK fix 로 자동 생성했다.\n'
        f'# 생성 시각: {time.strftime("%Y-%m-%d %H:%M:%S")}\n'
        f'# 세팅 지점 위경도 ≈ {lat:.7f}, {lon:.7f}\n\n'
        f'site: "{self.args.site}"\n\n'
        f'utm_epsg: {UTM_EPSG}\n'
        f'origin_x: {ox:.1f}\n'
        f'origin_y: {oy:.1f}\n')
    with open(SITE_YAML, 'w') as f:
      f.write(body)
    print(f'✅ 원점 기록 완료 → {SITE_YAML}')
    print('   확인: python3 src/waypoint_follower/waypoint_follower/site_origin.py')
    print('   ⚠ 이제 웨이포인트를 이 원점으로 새로 기록할 것.')


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--write', action='store_true', help='실제로 yaml 에 쓴다')
  p.add_argument('--site', default='현장 (미지정)', help='site 이름 기록')
  p.add_argument('--samples', type=int, default=20)
  p.add_argument('--sigma', type=float, default=0.05, help='FIXED 판정 [m]')
  p.add_argument('--floor', type=float, default=100.0, help='원점 내림 단위 [m]')
  p.add_argument('--timeout', type=float, default=60.0)
  p.add_argument('--fix-topic', default='/fix')
  # ros2 run 스타일 --ros-args 무시
  args, _ = p.parse_known_args()

  rclpy.init()
  node = OriginSetter(args)
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    # 결과를 낸 뒤 스스로 shutdown 하므로 spin 이 이 예외로 빠져나온다.
    # 안 잡으면 정상 종료인데도 traceback 이 찍혀 결과를 가린다.
    pass
  finally:
    try:
      node.destroy_node()
    except Exception:  # noqa: BLE001
      pass
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

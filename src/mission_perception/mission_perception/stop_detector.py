#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stop_detector.py — 경로를 찍으면서 "차가 선 자리"를 자동으로 정지지점으로 기록한다.

웨이포인트를 찍는 주행 중에 정지선에서 잠깐 서기만 하면, 그 자리가
`~/stop_points.yaml` 로 저장된다. 엔터를 누를 필요도, 따로 한 바퀴 더 돌 필요도 없다.

    ros2 run waypoint_follower waypoint_recorder     # 터미널 A — 경로
    ros2 run mission_perception stop_detector        # 터미널 B — 정지지점 (이 노드)

★ 왜 따로 필요한가
  waypoint_recorder 는 **이동한 거리**로 점을 찍는다. 서 있는 동안은 점이 안 찍히므로
  기록된 경로 파일만 봐서는 어디서 얼마나 섰는지 알 수 없다. 시간을 보는 눈이
  따로 있어야 한다.

★ 판정
  hold_seconds(기본 3.0초) 이상 move_epsilon(기본 15cm) 안에 머물면 '정지'로 본다.
  대회 규정대로 5초 서면 넉넉히 잡힌다. 다시 움직이면 그 정지는 확정된다.

★ 출발 지점과 마지막 정지는 걸러야 한다
  출발 전에 서 있던 것과 기록을 끝내며 선 것도 '정지'다. 그래서
  · 출발 전 정지는 **차가 min_travel(기본 3m) 이상 움직인 뒤부터** 센다
  · 마지막 정지는 저장할 때 표시해 주고, --drop-last 로 뺄 수 있다
  화면에 전부 번호를 붙여 찍어주므로 현장에서 눈으로 확인하고 고르면 된다.

파라미터:
  hold_seconds   : 정지 판정 시간[s] (기본 3.0)
  move_epsilon   : 이 반경 안이면 안 움직인 것[m] (기본 0.15)
  min_travel     : 이만큼 주행한 뒤부터 정지를 센다[m] (기본 3.0)
  min_separation : 이보다 가까운 정지는 같은 지점으로 본다[m] (기본 2.0)
  drop_last      : 마지막 정지를 빼고 저장 (기본 false)
  output_file    : 기본 ~/stop_points.yaml
"""

import math
import os
import sys

import rclpy
import yaml
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from waypoint_follower.site_origin import declare_and_get


class StopDetector(Node):

  def __init__(self):
    super().__init__('stop_detector')

    self.declare_parameter('fix_topic', '/fix')
    epsg, self.origin_x, self.origin_y = declare_and_get(self)
    self.declare_parameter('hold_seconds', 3.0)
    self.declare_parameter('move_epsilon', 0.15)
    self.declare_parameter('min_travel', 3.0)
    self.declare_parameter('min_separation', 2.0)
    self.declare_parameter('drop_last', False)
    self.declare_parameter('output_file',
                           os.path.expanduser('~/stop_points.yaml'))

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.hold = float(g('hold_seconds'))
    self.eps = float(g('move_epsilon'))
    self.min_travel = float(g('min_travel'))
    self.min_sep = float(g('min_separation'))
    self.drop_last = bool(g('drop_last'))
    self.out = str(g('output_file'))

    self.tf = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}',
                                   always_xy=True)

    self.xy = None            # 현재 위치
    self.anchor = None        # 정지 후보의 기준점
    self.anchor_t = None      # 그 기준점에 머물기 시작한 시각
    self.traveled = 0.0       # 누적 주행거리
    self.last_xy = None
    self.in_stop = False      # 이미 이번 정지를 확정했는가
    self.stops = []           # [(x, y, 머문시간)]

    self.create_subscription(NavSatFix, str(g('fix_topic')), self.fix_cb,
                             qos_profile_sensor_data)
    self.create_timer(0.5, self.tick)

    self.get_logger().info(
        f'정지지점 자동 감지 시작 — {self.hold:.0f}초 이상 서 있으면 기록한다. '
        f'({self.min_travel:.0f}m 주행 후부터 판정)  → {self.out}')
    self.get_logger().info(
        '경로 기록과 같이 켜두고, 정지선에서 그냥 서 있기만 하면 된다. '
        '끝나면 Ctrl-C.')

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def fix_cb(self, msg: NavSatFix):
    if msg.latitude != msg.latitude or abs(msg.latitude) < 1e-9:
      return
    ux, uy = self.tf.transform(msg.longitude, msg.latitude)
    xy = (ux - self.origin_x, uy - self.origin_y)
    if self.last_xy is not None:
      self.traveled += math.hypot(xy[0] - self.last_xy[0],
                                  xy[1] - self.last_xy[1])
    self.last_xy = xy
    self.xy = xy

  def tick(self):
    if self.xy is None:
      self.get_logger().warn('GPS 미수신 — /fix 확인', throttle_duration_sec=5.0)
      return

    x, y = self.xy
    if self.anchor is None:
      self.anchor, self.anchor_t = (x, y), self.now()
      return

    d = math.hypot(x - self.anchor[0], y - self.anchor[1])
    if d > self.eps:
      # 움직였다 — 정지 후보 리셋
      if self.in_stop:
        self.get_logger().info('  ▶ 출발 — 정지 확정')
      self.anchor, self.anchor_t = (x, y), self.now()
      self.in_stop = False
      return

    held = self.now() - self.anchor_t
    if held < self.hold or self.in_stop:
      return

    # ---- 정지 확정 ----
    self.in_stop = True
    if self.traveled < self.min_travel:
      self.get_logger().info(
          f'정지 감지했지만 아직 {self.traveled:.1f}m 밖에 안 갔다 — '
          f'출발 전 대기로 보고 무시한다.')
      return
    near = [i for i, (sx, sy, _) in enumerate(self.stops)
            if math.hypot(sx - x, sy - y) < self.min_sep]
    if near:
      self.get_logger().info(
          f'정지 감지 — 이미 기록한 {near[0] + 1}번과 가까워 무시한다.')
      return

    self.stops.append((x, y, held))
    self.get_logger().info(
        f'■ 정지지점 {len(self.stops)}번 기록: ({x:.2f}, {y:.2f})  '
        f'{held:.1f}초 정지  [주행 {self.traveled:.1f}m 지점]')
    self.save()          # 즉시 저장 — 종료 방식에 상관없이 남게

  def save(self):
    if not self.stops:
      return
    pts = list(self.stops)
    dropped = None
    if self.drop_last and len(pts) > 1:
      dropped = pts.pop()
    data = {
        'origin_x': self.origin_x, 'origin_y': self.origin_y,
        'stop_points': [{'x': round(x, 3), 'y': round(y, 3)}
                        for (x, y, _) in pts],
        'detected': [{'x': round(x, 3), 'y': round(y, 3),
                      'held_s': round(h, 1)} for (x, y, h) in pts],
    }
    with open(self.out, 'w', encoding='utf-8') as f:
      yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    if dropped is not None:
      print(f'  (마지막 정지 ({dropped[0]:.2f},{dropped[1]:.2f}) 는 제외했다)',
            flush=True)

  def report(self):
    print('\n' + '=' * 56, flush=True)
    if not self.stops:
      print('정지지점을 하나도 못 잡았다.', flush=True)
      print(f'  · {self.hold:.0f}초 이상 서 있었는가?', flush=True)
      print(f'  · {self.min_travel:.0f}m 이상 주행한 뒤였는가?', flush=True)
      return
    print(f'정지지점 {len(self.stops)}개  → {self.out}', flush=True)
    for i, (x, y, h) in enumerate(self.stops, 1):
      tag = '  ← 마지막 (기록 종료 지점일 수 있다)' \
          if i == len(self.stops) else ''
      print(f'  {i}. ({x:8.2f}, {y:8.2f})  {h:4.1f}초{tag}', flush=True)
    print('\n마지막 것이 실제 정지선이 아니라 기록을 끝낸 자리라면:', flush=True)
    print('  파일에서 그 항목을 지우거나, -p drop_last:=true 로 다시 실행',
          flush=True)
    print('=' * 56, flush=True)


def main(args=None):
  rclpy.init(args=args)
  node = StopDetector()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
  finally:
    try:
      node.save()
      node.report()
    except Exception as e:  # noqa: BLE001
      print(f'⚠ 저장 실패: {e}', flush=True)
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

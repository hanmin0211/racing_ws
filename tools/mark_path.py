#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mark_path.py — 경로의 문제 지점을 RViz 마커로 띄운다.

현장에서 "어디가 문제냐"를 좌표 숫자로 읽는 건 느리다. RViz 에 바로 찍어서
그 자리로 걸어가 확인할 수 있게 한다.

  빨강 X 큰 구  : 되꺾임(cusp) — 진행방향이 crop_deg 이상 뒤집히는 곳.
                  차는 여기를 못 지나간다(pure_pursuit 은 전진 전용).
  주황 구       : 최소회전반경 미달 커브 — 제어가 완벽해도 이탈한다.
  흰 글자       : 진행거리 s [m]

사용:
  python3 tools/mark_path.py --waypoints ~/wp_yongin_resampled_0.5.yaml
  RViz 에서 Add → MarkerArray → Topic: /path_markers
"""

import argparse
import math

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

WHEELBASE = 0.785
MAX_STEER_DEG = 18.0


def load(path):
  d = yaml.safe_load(open(path, encoding='utf-8'))
  return [(float(q['x']), float(q['y'])) for q in d['waypoints']]


def heading(p, i):
  j = min(max(i, 0), len(p) - 2)
  return math.atan2(p[j + 1][1] - p[j][1], p[j + 1][0] - p[j][0])


def radii(p):
  out = {}
  for i in range(1, len(p) - 1):
    (x1, y1), (x2, y2), (x3, y3) = p[i - 1], p[i], p[i + 1]
    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    s = (a + b + c) / 2.0
    ar2 = s * (s - a) * (s - b) * (s - c)
    if ar2 > 1e-12:
      out[i] = (a * b * c) / (4.0 * math.sqrt(ar2))
  return out


class Marks(Node):

  def __init__(self, args):
    super().__init__('path_markers')
    self.pts = load(args.waypoints)
    self.args = args
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.pub = self.create_publisher(MarkerArray, '/path_markers', qos)
    self.msg = self.build()
    self.pub.publish(self.msg)
    self.create_timer(1.0, lambda: self.pub.publish(self.msg))

  def sphere(self, mid, x, y, rgb, scale, ns):
    m = Marker()
    m.header.frame_id = self.args.frame
    m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
    m.pose.position.x, m.pose.position.y = float(x), float(y)
    m.pose.position.z = 0.3
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 0.9
    return m

  def text(self, mid, x, y, s, ns, scale=1.2):
    m = Marker()
    m.header.frame_id = self.args.frame
    m.ns, m.id = ns, mid
    m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position.x, m.pose.position.y = float(x), float(y)
    m.pose.position.z = 1.2
    m.pose.orientation.w = 1.0
    m.scale.z = scale
    m.color.r = m.color.g = m.color.b = 1.0
    m.color.a = 0.95
    m.text = s
    return m

  def build(self):
    p, a = self.pts, self.args
    arr = MarkerArray()
    mid = 0
    R = radii(p)

    # 되꺾임
    cusps = [i for i in range(1, len(p) - 2)
             if abs((math.degrees(heading(p, i)) - math.degrees(heading(p, i - 1))
                     + 540) % 360 - 180) > a.cusp_deg]
    for i in cusps:
      arr.markers.append(self.sphere(mid, p[i][0], p[i][1], (0.85, 0.1, 0.1),
                                     2.0, 'cusp')); mid += 1
      arr.markers.append(self.text(mid, p[i][0], p[i][1],
                                   f'CUSP s={i * a.spacing:.0f}m',
                                   'cusp_txt', 1.6)); mid += 1

    # 못 도는 커브
    bad = [i for i, r in R.items() if r < a.min_radius]
    for i in bad:
      arr.markers.append(self.sphere(mid, p[i][0], p[i][1], (0.95, 0.6, 0.05),
                                     0.9, 'tight')); mid += 1

    # s 라벨
    step = int(round(a.label_every / a.spacing))
    for i in range(0, len(p), max(1, step)):
      arr.markers.append(self.text(mid, p[i][0], p[i][1],
                                   f'{i * a.spacing:.0f}', 's_txt')); mid += 1

    # 시작/종점
    arr.markers.append(self.sphere(mid, p[0][0], p[0][1], (0.1, 0.8, 0.3),
                                   2.0, 'ends')); mid += 1
    arr.markers.append(self.text(mid, p[0][0], p[0][1], 'START', 'ends_txt',
                                 2.0)); mid += 1
    arr.markers.append(self.sphere(mid, p[-1][0], p[-1][1], (0.6, 0.2, 0.8),
                                   2.0, 'ends')); mid += 1
    arr.markers.append(self.text(mid, p[-1][0], p[-1][1], 'END', 'ends_txt',
                                 2.0)); mid += 1

    self.get_logger().info(
        f'{len(p)}점 · 되꺾임 {len(cusps)}개 · 못 도는 커브 {len(bad)}개 '
        f'→ /path_markers ({len(arr.markers)} 마커)')
    for i in cusps:
      self.get_logger().warn(
          f'  CUSP s={i * a.spacing:.1f}m  ({p[i][0]:.2f}, {p[i][1]:.2f}) — '
          '차는 여기를 못 지나간다')
    return arr


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--waypoints', required=True)
  ap.add_argument('--frame', default='map')
  ap.add_argument('--spacing', type=float, default=0.5,
                  help='리샘플 간격[m] — s 계산용')
  ap.add_argument('--min-radius', type=float, default=2.42)
  ap.add_argument('--cusp-deg', type=float, default=60.0)
  ap.add_argument('--label-every', type=float, default=50.0)
  args, _ = ap.parse_known_args()

  rclpy.init()
  node = Marks(args)
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

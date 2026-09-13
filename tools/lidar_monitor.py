#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_monitor.py — 라이다 회피 실시간 콘솔 모니터 (현장 튜닝·마운트 확인용).

/scan 원본과 cluster_plot_node 의 결정 토픽을 함께 보여준다.

  · 전방(차량기준) 최근접 물체: 각도·거리 → 마운트 방향(yaw_offset) 확인용.
    실제로 '차 정면'에 물체를 놓았을 때 corrected 각이 ~0°면 yaw_offset 정상.
    ~180°면 라이다가 뒤집혀 있으니 fg_yaw_offset_deg 를 0 으로.
  · 좌/정면/우 섹터별 최근접 거리 → 슬라롬에서 어느 쪽이 뚫렸는지.
  · follow-gap 결정: mode / avoid_steer / obstacle_distance.

사용 (드라이버 + cluster_plot_node 실행 중):
  python3 tools/lidar_monitor.py
  python3 tools/lidar_monitor.py --yaw-offset 180   # 원본 scan 해석용 오프셋
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class Monitor(Node):

  def __init__(self, yaw_offset_deg, topic='/scan_front'):
    super().__init__('lidar_monitor')
    self.yaw = math.radians(yaw_offset_deg)
    self.mode = '-'
    self.steer = float('nan')
    self.obs = None
    self.create_subscription(LaserScan, topic, self.scan_cb,
                             qos_profile_sensor_data)
    self.create_subscription(String, '/lidar/mode',
                             lambda m: setattr(self, 'mode', m.data), 10)
    self.create_subscription(Float64, '/lidar/avoid_steer',
                             lambda m: setattr(self, 'steer', m.data), 10)
    self.create_subscription(Float64, '/obstacle_distance',
                             lambda m: setattr(self, 'obs', m.data), 10)
    self.create_timer(0.25, self.render)
    self.near = None       # (corrected_deg, dist)
    self.raw_near = None    # (raw_deg, dist)
    self.sectors = (None, None, None)  # 좌, 정면, 우 최근접 거리

  def scan_cb(self, msg):
    best = None; raw_best = None
    left = front = right = None
    a = msg.angle_min
    for d in msg.ranges:
      if math.isfinite(d) and msg.range_min < d < msg.range_max:
        corr = math.atan2(math.sin(a + self.yaw), math.cos(a + self.yaw))
        cdeg = math.degrees(corr)
        # 전방(±90°)만
        if abs(cdeg) <= 90:
          if best is None or d < best[1]:
            best = (cdeg, d); raw_best = (math.degrees(a), d)
          if cdeg > 20:      # 좌
            left = d if left is None else min(left, d)
          elif cdeg < -20:   # 우
            right = d if right is None else min(right, d)
          else:              # 정면
            front = d if front is None else min(front, d)
      a += msg.angle_increment
    self.near = best; self.raw_near = raw_best
    self.sectors = (left, front, right)

  def render(self):
    def f(v):
      return f'{v:.2f}m' if v is not None else '  -  '
    left, front, right = self.sectors
    st = self.steer
    st_s = 'NaN' if (st is None or math.isnan(st)) else f'{st:+.1f}°'
    near = self.near; raw = self.raw_near
    near_s = f'{near[0]:+.0f}° {near[1]:.2f}m' if near else '없음'
    raw_s = f'{raw[0]:+.0f}°' if raw else '-'
    obs_s = f'{self.obs:.2f}m' if self.obs is not None else '-'
    print(f'섹터[좌 {f(left)} | 정면 {f(front)} | 우 {f(right)}]  '
          f'최근접(차량기준) {near_s} (raw {raw_s})  ||  '
          f'mode={self.mode:14s} steer={st_s:>7s} obs={obs_s}')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--topic', default='/scan_front',
                  help='스캔 토픽 (기본 /scan_front — 앞뒤 분리 후)')
  ap.add_argument('--yaw-offset', type=float, default=180.0,
                  help='원본 scan 해석용 yaw offset[도] (cluster 노드와 맞출 것)')
  args = ap.parse_args()
  rclpy.init()
  n = Monitor(args.yaw_offset, args.topic)
  print('라이다 모니터 시작 — 차 정면에 물체를 놓고 "최근접(차량기준)" 각이')
  print('~0°면 마운트 정상. ~±180°면 fg_yaw_offset_deg 를 뒤집을 것. Ctrl-C 종료.\n')
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

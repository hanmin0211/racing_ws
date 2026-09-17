#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_record.py — 주행 중 데이터를 CSV 로 쌓는다. 나중에 지도로 그린다.

★ 왜 필요한가 (2026-09-14)
  회피가 '되긴 하는데 의자 사이로 안 간다' 를 로그 텍스트로 판정하려니
  한계가 있었다. cluster_plot_node 가 찍는 좌표는 **차량 기준**이고 프레임
  마다 튀어서, 의자가 실제로 어디 있었고 차가 어디로 갔는지를 못 본다.

  그래서 주행 중에
    · 차의 실제 궤적(지도 좌표)
    · 그 순간의 라이다 점들을 **지도 좌표로 변환**해서
    · 회피 모드·조향각·전압
  을 같이 쌓는다. 끝나면 tools/drive_plot.py 로 한 장에 그린다.

사용 (bringup 과 같이 띄운다):
  python3 tools/drive_record.py                    # /tmp/drive_HHMM.csv
  python3 tools/drive_record.py --out /tmp/a.csv
  python3 tools/drive_record.py --scan-every 5     # 스캔 저장 간격(프레임)
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Int32, String


def yaw_from_quat(q):
  s = 2.0 * (q.w * q.z + q.x * q.y)
  c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(s, c)


class Recorder(Node):

  def __init__(self, a):
    super().__init__('drive_record')
    self.a = a
    self.t0 = time.time()
    self.pose = None          # (x, y, yaw)
    self.v = 0.0
    self.mode = '-'
    self.avoid = float('nan')
    self.obs = float('nan')
    self.cmd_v = 0.0
    self.cmd_steer = 0.0
    self.steer_act = float('nan')
    self.vcc = 0
    self.scan_n = 0

    self.f = open(a.out, 'w', newline='')
    self.w = csv.writer(self.f)
    self.w.writerow(['t', 'x', 'y', 'yaw_deg', 'v',
                     'mode', 'avoid_steer_deg', 'obstacle_m',
                     'cmd_v', 'cmd_steer_deg', 'steer_actual_deg', 'vcc_mv'])
    # 라이다 점은 별도 파일 — 행이 훨씬 많다
    self.sf = open(os.path.splitext(a.out)[0] + '_scan.csv', 'w', newline='')
    self.sw = csv.writer(self.sf)
    self.sw.writerow(['t', 'x', 'y'])   # 지도 좌표로 변환해서 저장

    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
    self.create_subscription(String, '/lidar/mode',
                             lambda m: setattr(self, 'mode', m.data), 10)
    self.create_subscription(Float64, '/lidar/avoid_steer',
                             lambda m: setattr(self, 'avoid', m.data), 10)
    self.create_subscription(Float64, '/obstacle_distance',
                             lambda m: setattr(self, 'obs', m.data), 10)
    self.create_subscription(Float64, '/steering_angle',
                             lambda m: setattr(self, 'steer_act', m.data), 10)
    self.create_subscription(Int32, '/vcc_mv',
                             lambda m: setattr(self, 'vcc', m.data), 10)
    self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
    self.create_subscription(LaserScan, a.scan_topic, self.scan_cb,
                             qos_profile_sensor_data)
    self.create_timer(1.0 / a.rate, self.tick)
    self.rows = 0
    print(f'기록 시작 → {a.out}  (Ctrl-C 로 종료)')

  def odom_cb(self, m):
    p = m.pose.pose
    self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
    self.v = m.twist.twist.linear.x

  def cmd_cb(self, m):
    self.cmd_v = m.linear.x
    # ★ 2026-09-16 — angular.z 는 **이미 도** 단위다.
    #   vehicle_cmd_mux_node.py:18 'angular.z = 조향각 [도] ※ rad/s 아님!'
    #   math.degrees() 를 씌우면 57.3배로 부풀어 기록이 통째로 틀린다.
    self.cmd_steer = float(m.angular.z)

  def scan_cb(self, msg):
    """스캔을 **지도 좌표로** 바꿔 저장한다.

    차량 기준으로 두면 나중에 '의자가 어디 있었나' 를 못 본다. 차가 움직이며
    같은 물체를 계속 다른 좌표로 보기 때문이다. 지도 좌표로 바꿔 쌓으면
    여러 프레임이 같은 자리에 겹쳐서 장애물 윤곽이 드러난다.
    """
    self.scan_n += 1
    if self.pose is None or self.scan_n % self.a.scan_every:
      return
    x0, y0, yaw = self.pose
    # cluster_plot_node 와 같은 보정을 쓴다(차량 +x 전방).
    off = math.radians(self.a.yaw_offset)
    t = time.time() - self.t0
    a = msg.angle_min
    for r in msg.ranges:
      if math.isfinite(r) and msg.range_min < r < min(msg.range_max,
                                                      self.a.max_range):
        th = yaw + a + off
        self.sw.writerow([f'{t:.2f}',
                          f'{x0 + r * math.cos(th):.3f}',
                          f'{y0 + r * math.sin(th):.3f}'])
      a += msg.angle_increment

  def tick(self):
    if self.pose is None:
      return
    x, y, yaw = self.pose
    self.w.writerow([f'{time.time() - self.t0:.2f}',
                     f'{x:.3f}', f'{y:.3f}', f'{math.degrees(yaw):.1f}',
                     f'{self.v:.2f}', self.mode,
                     '' if math.isnan(self.avoid) else f'{self.avoid:.1f}',
                     '' if math.isnan(self.obs) else f'{self.obs:.2f}',
                     f'{self.cmd_v:.2f}', f'{self.cmd_steer:.1f}',
                     '' if math.isnan(self.steer_act) else f'{self.steer_act:.1f}',
                     self.vcc])
    self.rows += 1
    if self.rows % 100 == 0:
      self.f.flush()
      self.sf.flush()
      print(f'  {self.rows}행  x={x:.1f} y={y:.1f} v={self.v:.2f} '
            f'mode={self.mode}')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--out', default=f'/tmp/drive_{time.strftime("%H%M")}.csv')
  ap.add_argument('--rate', type=float, default=20.0)
  ap.add_argument('--scan-topic', default='/scan_front')
  ap.add_argument('--scan-every', type=int, default=3,
                  help='이 프레임마다 한 번 스캔을 저장 (기본 3)')
  ap.add_argument('--yaw-offset', type=float, default=173.0,
                  help='라이다 마운트 보정[도] — cluster 노드와 맞출 것')
  ap.add_argument('--max-range', type=float, default=6.0)
  a = ap.parse_args()

  rclpy.init()
  n = Recorder(a)
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  finally:
    n.f.close()
    n.sf.close()
    print(f'\n기록 종료: {a.out}  ({n.rows}행)')
    print(f'          {os.path.splitext(a.out)[0]}_scan.csv')
    print(f'그리기:  python3 tools/drive_plot.py {a.out}')
    try:
      if rclpy.ok():
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
      pass
  return 0


if __name__ == '__main__':
  sys.exit(main())

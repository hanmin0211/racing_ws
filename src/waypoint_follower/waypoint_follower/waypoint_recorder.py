#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_recorder.py
====================
GPS 웨이포인트 자동 기록 노드.

RTK GPS(/fix, 위경도)를 받아 UTM 52N으로 변환하고 기존 파이프라인과 동일한
원점(origin_x/y)을 빼서 로컬 x,y(m)로 만든 뒤, 차량이 point_spacing(기본 1m)
이동할 때마다 한 점씩 자동으로 기록한다. 다 찍고 차량을 멈추면(정지 감지)
자동으로 YAML 파일에 저장한다.

저장 형식(global_path_publisher가 읽는 형식과 동일):
  waypoints:
    - {x: <float>, y: <float>}
    - ...

저장 트리거(셋 다 지원):
  - 정지 감지: stop_seconds(기본 3s) 동안 안 움직이면 자동 저장
  - 서비스 ~/save (std_srvs/Trigger)
  - Ctrl-C 종료 시에도 저장

파라미터:
  fix_topic     : GPS 토픽 (기본 /fix)
  utm_epsg      : UTM 대역 EPSG — 기본값은 config/site_origin.yaml 에서 읽음
  origin_x/y    : 로컬좌표 원점 — 기본값은 config/site_origin.yaml 에서 읽음
                  (장소가 바뀌면 그 파일만 고치면 전 노드가 따라온다)
  point_spacing : 점 간격[m] (기본 1.0)
  stop_seconds  : 정지 판정 시간[s] (기본 3.0)
  require_rtk   : True면 RTK Fixed일 때만 기록 (기본 False, 아니면 경고만)
  output_file   : 저장 경로
"""

import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from waypoint_follower.site_origin import (declare_and_get, load_site_origin,
                                           origin_stamp)
from waypoint_follower.waypoint_resample import resample, save_waypoints

DEFAULT_OUT = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
               'waypoints_recorded.yaml')


class WaypointRecorder(Node):

  def __init__(self):
    super().__init__('waypoint_recorder')

    self.declare_parameter('fix_topic', '/fix')
    # 원점은 config/site_origin.yaml 이 정본. utm_epsg/origin_x/origin_y 파라미터를
    # 여기서 선언하고 읽는다(CLI -p 로 덮어쓰기 가능).
    epsg, self.origin_x, self.origin_y = declare_and_get(self)
    self.utm_epsg = epsg
    # 저장 파일에 남길 장소 이름 (site_origin.yaml 의 site:)
    self.site_name = load_site_origin(self.get_logger())[3]
    self.declare_parameter('point_spacing', 1.0)
    self.declare_parameter('stop_seconds', 3.0)
    self.declare_parameter('require_rtk', False)
    # ★ RTK 판정은 status.status 가 아니라 **공분산**으로 한다.
    # 지금 쓰는 ublox_nav_sat_fix_hp 드라이버는 RTK Fixed 에서도
    # status.status=1(SBAS) 만 내보내고 2(GBAS)를 절대 안 준다.
    # 그래서 status>=2 조건으로 걸면 require_rtk:=true 일 때 한 점도 기록되지
    # 않는다(2026-08-17 확인: 공분산 2.9mm 인데 status=1).
    # 공분산은 position_covariance_type=3(KNOWN)으로 항상 실려오고
    # Fixed(수 mm) / Float(수십 cm) 구분이 훨씬 확실하다.
    self.declare_parameter('max_h_std', 0.05)   # 수평 표준편차 상한 [m]
    self.declare_parameter('output_file', DEFAULT_OUT)
    # 저장 시 균일 간격 리샘플 파일도 함께 생성 (0이면 생성 안 함)
    self.declare_parameter('resample_spacing', 0.5)

    fix_topic = self.get_parameter('fix_topic').value
    self.spacing = float(self.get_parameter('point_spacing').value)
    self.stop_seconds = float(self.get_parameter('stop_seconds').value)
    self.require_rtk = bool(self.get_parameter('require_rtk').value)
    self.max_h_std = float(self.get_parameter('max_h_std').value)
    self.output_file = self.get_parameter('output_file').value
    self.resample_spacing = float(self.get_parameter('resample_spacing').value)

    self.tf = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}',
                                   always_xy=True)

    self.waypoints = []          # [(x, y), ...]
    self.last_xy = None          # 마지막으로 '기록한' 점
    self.last_pos = None         # 마지막으로 '본' 위치 (정지 판정용)
    self.last_motion_time = time.time()
    self.h_acc_m = None      # /ubx_nav_hp_pos_llh 수평정확도 [m]
    self.h_acc_t = 0.0
    self.saved = False
    self._warned_rtk = False

    # /fix(ublox)는 BEST_EFFORT 발행이므로 센서 QoS로 구독해야 받는다.
    # ★ RTK 품질 보조 입력 (2026-09-05 용인 현장에서 발견)
    #   지금 드라이버(ublox_dgnss)는 /fix 에 **공분산을 아예 안 채운다**
    #   (position_covariance 전부 0, covariance_type=0=UNKNOWN). 게다가 RTK
    #   Fixed 인데도 status.status=1(SBAS) 만 준다. 그래서 예전 판정 경로가
    #   둘 다 막혀 require_rtk:=true 로 두면 **한 점도 기록되지 않았다.**
    #   정확도는 /ubx_nav_hp_pos_llh 의 h_acc 에 살아 있다(0.1mm 단위).
    #   현장 실측: h_acc=141 → 1.41cm = RTK Fixed. 이걸 폴백으로 쓴다.
    try:
      from ublox_ubx_msgs.msg import UBXNavHPPosLLH
      self.create_subscription(UBXNavHPPosLLH, '/ubx_nav_hp_pos_llh',
                               self.hpacc_cb, qos_profile_sensor_data)
      self.get_logger().info('RTK 품질 보조: /ubx_nav_hp_pos_llh h_acc 사용')
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(
          f'/ubx_nav_hp_pos_llh 구독 불가({e}) — 공분산/status 만 본다')

    self.create_subscription(NavSatFix, fix_topic, self.fix_cb,
                             qos_profile_sensor_data)
    self.create_service(Trigger, '~/save', self.save_srv)
    self.create_timer(1.0, self.check_stop)

    self.get_logger().info(
        f'웨이포인트 레코더 시작: {fix_topic} 구독, {self.spacing:.1f}m마다 기록, '
        f'{self.stop_seconds:.0f}s 정지 시 자동저장 → {self.output_file}')
    self.get_logger().info(
        f'RTK 필터: {"ON — 수평 σ ≤ %.0fcm 인 점만 기록" % (self.max_h_std * 100)}'
        if self.require_rtk else 'RTK 필터: OFF — 모든 점 기록 (품질 무관)')

  def hpacc_cb(self, msg):
    """UBX HPPOSLLH 의 h_acc 는 0.1mm 단위 정수다."""
    try:
      self.h_acc_m = float(msg.h_acc) * 1e-4
      self.h_acc_t = time.time()
    except Exception:  # noqa: BLE001
      pass

  def fix_cb(self, msg: NavSatFix):
    if math.isnan(msg.latitude) or abs(msg.latitude) < 1e-9:
      return
    # RTK 품질 확인 — 공분산(수평 표준편차) 기준. 위 declare_parameter 주석 참고.
    h_std = math.sqrt(max(0.0, msg.position_covariance[0]
                          + msg.position_covariance[4]))
    # ★ 품질 출처를 순서대로 고른다 (2026-09-05 용인).
    #   ① /fix 공분산 — 채워져 있으면 제일 좋다
    #   ② /ubx_nav_hp_pos_llh 의 h_acc — 지금 드라이버는 여기에만 값이 있다
    #   ③ status.status — 이 드라이버는 Fixed 여도 1 만 주므로 최후수단
    src = 'cov'
    if msg.position_covariance_type == 0:
      if (self.h_acc_m is not None
          and time.time() - self.h_acc_t <= 2.0):
        h_std, src = self.h_acc_m, 'h_acc'
      else:
        src = 'status'
    if self.require_rtk:
      if src == 'status':
        # 공분산도 h_acc 도 못 쓸 때만. 이 드라이버에서는 사실상 항상 거부된다.
        ok = msg.status.status >= 2
      else:
        ok = h_std <= self.max_h_std
      if not ok:
        why = ('/fix 공분산·h_acc 둘 다 못 읽는다 — '
               'ubx_nav_hp_pos_llh 가 나오는지 확인할 것'
               if src == 'status'
               else f'수평 σ={h_std * 100:.1f}cm > {self.max_h_std * 100:.0f}cm')
        self.get_logger().warn(
            f'RTK 정밀도 미달 [{src}] ({why}) — 기록 보류',
            throttle_duration_sec=3.0)
        self._warned_rtk = True
        return
      if self._warned_rtk:
        self.get_logger().info(
            f'RTK 회복 [{src}] (수평 σ={h_std * 100:.1f}cm) — 기록 재개')
        self._warned_rtk = False

    utm_x, utm_y = self.tf.transform(msg.longitude, msg.latitude)
    x = utm_x - self.origin_x
    y = utm_y - self.origin_y

    # 정지 판정: 직전 본 위치에서 조금이라도 움직였으면 시간 갱신
    if self.last_pos is not None:
      if math.hypot(x - self.last_pos[0], y - self.last_pos[1]) > 0.05:
        self.last_motion_time = time.time()
    self.last_pos = (x, y)

    # 기록: 첫 점이거나 직전 기록점에서 spacing 이상 이동했을 때
    if self.last_xy is None or \
       math.hypot(x - self.last_xy[0], y - self.last_xy[1]) >= self.spacing:
      self.waypoints.append((x, y))
      self.last_xy = (x, y)
      self.saved = False
      self.get_logger().info(
          f'점 기록 #{len(self.waypoints)}: ({x:.2f}, {y:.2f})  '
          f'σ={h_std * 100:.1f}cm [{src}]')

  def check_stop(self):
    if (self.waypoints and not self.saved
        and time.time() - self.last_motion_time >= self.stop_seconds):
      self.get_logger().info(
          f'정지 감지 ({self.stop_seconds:.0f}s) — 자동 저장합니다.')
      self.save()

  def save_srv(self, request, response):
    ok = self.save()
    response.success = ok
    response.message = (f'{len(self.waypoints)}개 저장: {self.output_file}'
                        if ok else '저장 실패')
    return response

  def save(self):
    if not self.waypoints:
      self.get_logger().warn('기록된 점이 없어 저장하지 않음.')
      return False
    try:
      os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
      # ★ 원점을 반드시 함께 남긴다. 없으면 장소가 바뀐 뒤 이 파일이
      #   조용히 150km 어긋난 경로로 읽힌다(global_path_publisher 가 검증).
      stamp = origin_stamp(self.utm_epsg, self.origin_x, self.origin_y,
                           self.site_name)
      data = {'origin': stamp,
              'waypoints': [{'x': float(x), 'y': float(y)}
                            for x, y in self.waypoints]}
      with open(self.output_file, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False,
                       allow_unicode=True)
      self.saved = True
      self.get_logger().info(
          f'✅ 저장 완료: {len(self.waypoints)}개 점(원본) → {self.output_file}')

      # 균일 간격 리샘플 파일도 함께 생성
      if self.resample_spacing > 0 and len(self.waypoints) >= 2:
        base, ext = os.path.splitext(self.output_file)
        res_path = f'{base}_resampled_{self.resample_spacing:g}{ext}'
        res = resample(self.waypoints, self.resample_spacing)
        save_waypoints(res, res_path, origin=stamp)
        self.get_logger().info(
            f'✅ 리샘플 저장: {len(res)}개 점 @ {self.resample_spacing:g}m → '
            f'{res_path}')
      return True
    except Exception as e:  # noqa: BLE001
      self.get_logger().error(f'저장 실패: {e}')
      return False


def main(args=None):
  rclpy.init(args=args)
  node = WaypointRecorder()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    # Ctrl-C 로 끝내는 게 정상 사용법이다. 여기서 안 잡으면 traceback 이
    # 찍혀 '저장됐는지' 가 로그에 묻힌다.
    pass
  finally:
    # 종료 시에도 안전하게 저장 (컨텍스트가 이미 내려갔을 수 있으니 방어적으로)
    try:
      node.save()
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

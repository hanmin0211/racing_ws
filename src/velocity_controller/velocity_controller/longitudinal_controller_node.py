#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
longitudinal_controller_node.py — 미션 인지 종방향(속도) 플래너.

곡률·정지선·장애물·미션상태를 종합해 **목표 속도[m/s]** 를 정하고 `/target_speed`
로 발행한다. 가/감속 프로파일(슬루레이트)까지 여기서 처리해 부드럽게 만든다.

★ 설계 원칙: **PWM은 펌웨어가 소유한다.**
   펌웨어(henes_firmware)에 이미 속도 PID + FF + 안티와인드업 + 소프트스타트 +
   스톨가드가 있고 실측 검증됐다. ROS가 raw PWM을 직접 쏘면 두 제어기가 싸우고
   무엇보다 **스톨가드가 무력화되어 모터를 태운다**(과거 2회 소손 이력).
   따라서 이 노드는 '목표 속도'까지만 책임지고, 속도→PWM 변환은 펌웨어에 맡긴다.

미션 확장:
   MISSION_POLICY 에 항목만 추가하면 새 미션이 붙는다. 각 정책은
   (고정속도 or None, 설명) 형태. None이면 일반 주행 로직(곡률/정지선/장애물)을 탄다.

입력:
  /curvature           Float64  경로 곡률 κ (local_sliding_window)
  /current_speed       Float64  실측 속도 (serial_bridge, 엔코더 기반)
  /stop_line_distance  Float64  정지선까지 [m] (미구현 시 무시)
  /obstacle_distance   Float64  전방 장애물까지 [m] (serial_bridge 소나)
  /mission_state       String   DRIVE | STOP | SLALOM | REVERSE_PARK ...
출력:
  /target_speed        Float64  목표 속도[m/s] (음수=후진)
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

# 미션별 고정 속도 정책. None = 일반 주행 연산(곡률·정지선·장애물) 사용.
# 새 미션 추가는 여기에 한 줄이면 된다.
MISSION_POLICY = {
    'DRIVE': None,
    'STOP': 0.0,
    'SLALOM': 'v_slalom',        # 지그재그: 저속 고정
    'REVERSE_PARK': 'v_reverse',  # 후진 주차
    'ALIGN': 'v_slow',            # 초기 헤딩 정렬 직진
}


class LongitudinalController(Node):

  def __init__(self):
    super().__init__('longitudinal_controller')

    # ---- 속도 파라미터 ----
    # ⚠ 구동모터는 아직 벤치 미검증이라 기본값을 보수적으로 둔다.
    #    벤치 검증(FF/PID 재식별) 후 v_max를 단계적으로 올릴 것.
    self.declare_parameter('v_max', 1.0)
    # 0.5 → 0.25: 곡률 감속이 실제로 걸리려면 하한이 낮아야 한다.
    # (첫 실주행에서 코너 진입이 빨랐는데, v_min 0.5가 감속을 막고 있었다)
    self.declare_parameter('v_min', 0.25)     # 코너링 스톨 방지 최소
    self.declare_parameter('v_slow', 0.8)
    self.declare_parameter('v_slalom', 0.6)
    self.declare_parameter('v_reverse', -0.5)
    # ---- 감속 규칙 ----
    # 2.5 → 6.0: FF 미보정이라 실제 속도가 명령보다 높게 나오므로 감속을 강하게.
    # 예) κ=0.23(R=4.4m) 에서 2.5면 63%, 6.0이면 42%로 줄어든다.
    # v = v_max/(1+gain·|κ|). 무게중심이 높으면(배터리 뱅크·기둥) 코너에서
    # 더 일찍·더 많이 줄여야 기울지 않는다. 6.0 → 9.0 으로 상향.
    # 급코너(R=2.68m)에서 v_max 1.4 → 0.31 m/s 로 감속(횡가속 0.04 m/s²).
    self.declare_parameter('curvature_gain', 9.0)
    self.declare_parameter('max_accel', 1.0)        # m/s²
    self.declare_parameter('max_decel', 1.8)        # m/s²
    self.declare_parameter('stop_line_trigger', 19.0)
    self.declare_parameter('stop_line_decel', 1.0)  # 정지선 감속도
    self.declare_parameter('obstacle_trigger', 4.0)
    self.declare_parameter('obstacle_stop_dist', 0.8)
    # ---- 안전 ----
    self.declare_parameter('curvature_timeout', 0.5)  # 경로 끊김 감지
    self.declare_parameter('rate', 20.0)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.v_max = float(g('v_max'))
    self.v_min = float(g('v_min'))
    self.v_slow = float(g('v_slow'))
    self.v_slalom = float(g('v_slalom'))
    self.v_reverse = float(g('v_reverse'))
    self.curv_gain = float(g('curvature_gain'))
    self.max_accel = float(g('max_accel'))
    self.max_decel = float(g('max_decel'))
    self.stop_trigger = float(g('stop_line_trigger'))
    self.stop_decel = float(g('stop_line_decel'))
    self.obs_trigger = float(g('obstacle_trigger'))
    self.obs_stop = float(g('obstacle_stop_dist'))
    self.curv_timeout = float(g('curvature_timeout'))
    rate = float(g('rate'))
    self.dt = 1.0 / rate

    # ---- 상태 ----
    self.kappa = 0.0
    self.current_speed = 0.0
    self.stop_line_dist = 999.0
    self.obstacle_dist = 999.0
    self.mission = 'DRIVE'
    self.profiled_speed = 0.0        # 슬루레이트 적용된 목표
    self.last_curv_time = None       # 곡률 수신 시각(경로 살아있는지 판단)
    self.goal_reached = False        # 완주 래치 (한 번 서면 다시 안 달린다)

    self.pub = self.create_publisher(Float64, '/target_speed', 10)
    self.create_subscription(Float64, '/curvature', self.curvature_cb, 10)
    self.create_subscription(Float64, '/current_speed', self.speed_cb, 10)
    self.create_subscription(Float64, '/stop_line_distance', self.stop_cb, 10)
    self.create_subscription(Float64, '/obstacle_distance', self.obs_cb, 10)
    self.create_subscription(String, '/mission_state', self.mission_cb, 10)
    # ★ 완주 신호. 이게 없으면 경로 끝에서 local_sliding_window 가 /curvature
    # 발행을 멈추고, 여기서는 그걸 '경로 끊김'으로만 인식한다. 완주로 선 것과
    # 센서 고장으로 선 것이 로그상 구분되지 않아 대회 중 원인 판단이 늦어진다.
    self.create_subscription(Bool, '/goal_reached', self.goal_cb, 10)
    self.create_timer(self.dt, self.control_loop)

    self.get_logger().info(
        f'종방향 제어 시작: v_max={self.v_max} v_min={self.v_min} '
        f'(목표속도만 발행 — PWM은 펌웨어 PID가 담당)')

  # ---- 콜백 ----
  def curvature_cb(self, msg):
    self.kappa = abs(float(msg.data))
    self.last_curv_time = time.time()

  def speed_cb(self, msg):
    self.current_speed = abs(float(msg.data))

  def stop_cb(self, msg):
    self.stop_line_dist = float(msg.data)

  def obs_cb(self, msg):
    self.obstacle_dist = float(msg.data)

  def goal_cb(self, msg):
    """완주 신호. 한 번 True 면 래치한다(경로 끝에서 왔다갔다 하지 않도록)."""
    if bool(msg.data) and not self.goal_reached:
      self.goal_reached = True
      self.get_logger().info(
          '🏁 완주 신호 수신 — 감속 정지합니다 (고장 아님)')

  def mission_cb(self, msg):
    new = msg.data.strip().upper()
    if new != self.mission:
      self.get_logger().info(f'미션 전환: {self.mission} → {new}')
    self.mission = new

  # ---- 목표속도 결정 ----
  def decide_target(self):
    """미션 정책 → 고정속도, 또는 일반 주행 연산."""
    policy = MISSION_POLICY.get(self.mission, None)
    if policy is not None:
      if isinstance(policy, str):
        return float(getattr(self, policy))
      return float(policy)

    # ---- 일반 주행(DRIVE): 가장 보수적인 값 채택 ----
    v = self.v_max

    # A. 곡률 선제 감속 (코너 전 미리 줄임)
    v_curve = self.v_max / (1.0 + self.curv_gain * self.kappa)
    v = min(v, max(v_curve, self.v_min))

    # B. 정지선 비례 감속: v = √(2·a·d)
    if self.stop_line_dist < self.stop_trigger:
      v_stop = math.sqrt(2.0 * self.stop_decel * max(0.0, self.stop_line_dist))
      v = min(v, v_stop)
      if self.stop_line_dist <= 0.3:
        return 0.0

    # C. 전방 장애물 감속
    if self.obstacle_dist < self.obs_trigger:
      span = max(1e-3, self.obs_trigger - self.obs_stop)
      v_obs = self.v_slow * max(0.0, self.obstacle_dist - self.obs_stop) / span
      v = min(v, v_obs)
      if self.obstacle_dist <= self.obs_stop:
        return 0.0

    # 최소속도 하한은 '움직일 때만' 적용 (정지 판단은 위에서 이미 return)
    return max(v, self.v_min) if v > 0.05 else 0.0

  def control_loop(self):
    # 완주가 먼저다. 완주하면 local_sliding_window 가 /curvature 발행을 멈추므로
    # 아래 '끊김' 분기에 걸리는데, 그건 고장이 아니라 정상 종료다.
    # 순서를 바꾸면 대회 중 완주했는데 '경로 끊김' 경고만 보고 고장으로 오판한다.
    if self.goal_reached:
      # 급정거하지 않고 감속 프로파일로 세운다(정지지점 오버슈트 방지).
      self.profiled_speed = max(0.0, self.profiled_speed - self.max_decel * self.dt)
      self.publish(self.profiled_speed, profiled=False)
      if self.profiled_speed <= 1e-3:
        self.get_logger().info('🏁 완주 정지 완료', throttle_duration_sec=10.0)
      return
    # 안전: 경로(곡률)가 끊기면 정지. 단 수신 이력이 없으면(아직 시작 전) 정지 유지.
    if self.last_curv_time is None:
      self.publish(0.0, profiled=False)
      return
    if time.time() - self.last_curv_time > self.curv_timeout:
      self.get_logger().warn(
          '⚠ 경로(/curvature) 끊김 → 정지 — 완주 신호는 없었다. '
          'GPS/IMU 끊김이나 로컬경로 생성 실패를 의심할 것',
          throttle_duration_sec=2.0)
      self.profiled_speed = 0.0
      self.publish(0.0, profiled=False)
      return

    raw = self.decide_target()

    # 가/감속 프로파일 (슬루레이트) — 급가속/급제동 방지
    diff = raw - self.profiled_speed
    if diff > 0:
      self.profiled_speed += min(diff, self.max_accel * self.dt)
    elif diff < 0:
      self.profiled_speed -= min(-diff, self.max_decel * self.dt)

    self.publish(self.profiled_speed)

  def publish(self, v, profiled=True):
    if profiled:
      self.profiled_speed = v
    self.pub.publish(Float64(data=float(v)))


def main(args=None):
  rclpy.init(args=args)
  node = LongitudinalController()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node.pub.publish(Float64(data=0.0))
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

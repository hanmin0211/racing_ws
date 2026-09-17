#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_pure_pursuit_node.py
==========================
/local_path 기반 Pure Pursuit 횡방향 제어 (우리 파이프라인과 유기적으로 통합).

local_sliding_window_node가 만든 /local_path 는 이미 '차량 기준(base_link)'
좌표라, 좌표변환 없이 그 위에서 lookahead 점만 고르면 바로 조향각이 나온다.

  Ld    = clip(k_ld·v + min_ld, min_ld, max_ld)     # 속도비례 lookahead
  (x,y) = /local_path 위에서 차량으로부터 호길이 ~Ld 인 점  # 이미 차량기준
  ld    = √(x² + y²)
  κ_pp  = 2·y / ld²                                  # 그 점을 지나는 원호 곡률
  δ     = atan(L·κ_pp)                               # Ackermann 조향각 (L=축거)
  δ    = clip(δ, ±max_steer)

속도: 경로 곡률(/curvature)이 클수록 감속.
출력: /cmd_vel (Twist) — linear.x=속도[m/s], angular.z=조향각[deg]
      (velocity_controller/serial_bridge_node 가 이 포맷을 받음)

안전장치:
  - /local_path 가 timeout 이상 안 오면 → 정지
  - lookahead 점이 뒤(x<0)거나 이상하면 → 정지
  - 헤딩 미정렬 등으로 경로가 이상하면 조향 클램프로 방어
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class LocalPurePursuit(Node):

  def __init__(self):
    super().__init__('local_pure_pursuit')

    # ---- 차량 제원 ----
    self.declare_parameter('wheelbase', 0.785)          # HENES T870 브룬 실측 축거
    # 실사용 최대 타각. 물리한계 20°지만 포텐셔미터 포화(ADC 0)를 피해 18°로 제한.
    self.declare_parameter('max_steering_deg', 18.0)
    # ---- lookahead (속도비례) ----
    self.declare_parameter('k_ld', 0.6)
    self.declare_parameter('min_lookahead', 1.0)
    self.declare_parameter('max_lookahead', 4.0)
    # ---- 속도 ----
    self.declare_parameter('target_speed', 1.0)         # 저속부터
    self.declare_parameter('min_speed', 0.4)
    self.declare_parameter('curvature_speed_gain', 2.0)
    # ---- 토픽/안전 ----
    self.declare_parameter('local_path_topic', '/local_path')
    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('cmd_topic', '/cmd_vel')
    self.declare_parameter('control_rate', 20.0)
    self.declare_parameter('path_timeout', 0.5)         # 초; 이보다 오래 경로 없으면 정지
    # ★ 2026-09-16 — odom 속도 스파이크 방어.
    #   /odometry/filtered 의 twist 는 EKF 가 odom 프레임을 늦게/빠뜨려 받으면
    #   dt≈0 으로 나누며 튄다. drive_0337 실측: 1.08 → **61.90** → 0.99 m/s
    #   (2샘플, 랩당 3회). 위치는 멀쩡했다 — twist 필드만 튄다.
    #   여기서는 lookahead 가 ld=k_ld·v+min_ld 라 그 순간 max_lookahead 로 튀고,
    #   0.1초 동안 조향이 둔해진다. 클램프 덕에 파국은 아니지만 외란은 외란이다.
    #   이 값을 넘는 측정은 **버리고 직전 값을 유지한다.** 차량 최고속이
    #   1.5m/s 대이므로 3.0 이면 정상 주행을 절대 자르지 않는다.
    self.declare_parameter('max_plausible_speed', 3.0)
    # ---- 조향 슬루레이트 (부드러운 sweep) ----
    # 조향각이 1초에 이 각도 이상 못 바뀌게 제한. 급조향 방지 → 부드러운 추종 +
    # 조향모터 스톨/과부하 방지(안전). 실차에서 조향모터 속도에 맞춰 튜닝.
    self.declare_parameter('max_steer_rate_deg', 90.0)  # 도/초

    self.L = float(self.get_parameter('wheelbase').value)
    self.max_steer = math.radians(
        float(self.get_parameter('max_steering_deg').value))
    self.k_ld = float(self.get_parameter('k_ld').value)
    self.min_ld = float(self.get_parameter('min_lookahead').value)
    self.max_ld = float(self.get_parameter('max_lookahead').value)
    self.target_speed = float(self.get_parameter('target_speed').value)
    self.min_speed = float(self.get_parameter('min_speed').value)
    self.curv_gain = float(self.get_parameter('curvature_speed_gain').value)
    lp_topic = self.get_parameter('local_path_topic').value
    odom_topic = self.get_parameter('odom_topic').value
    cmd_topic = self.get_parameter('cmd_topic').value
    rate = float(self.get_parameter('control_rate').value)
    self.control_rate = rate
    self.path_timeout = float(self.get_parameter('path_timeout').value)
    self.max_steer_rate = float(self.get_parameter('max_steer_rate_deg').value)
    self.max_plausible_speed = float(
        self.get_parameter('max_plausible_speed').value)
    self.speed_rejects = 0

    self.path_pts = []          # [(x, y), ...] 차량기준
    self.last_path_time = None
    self.speed = 0.0
    self.curvature = 0.0
    self.prev_steer_deg = 0.0   # 슬루레이트 제한용 직전 조향각

    # 조향각은 항상 /steering_cmd(Float64, 도)로 낸다 — vehicle_cmd_mux가 이걸
    # 종방향(/target_speed)과 합쳐 최종 /cmd_vel을 만든다(단일 출구 원칙).
    # standalone:=true 면 예전처럼 /cmd_vel도 직접 발행(먹스 없이 조향만 시험할 때).
    self.declare_parameter('standalone', False)
    self.standalone = bool(self.get_parameter('standalone').value)
    self.steer_pub = self.create_publisher(Float64, '/steering_cmd', 10)
    self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
    self.create_subscription(Path, lp_topic, self.path_cb, 10)
    self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
    self.create_subscription(Float64, '/curvature', self.curv_cb, 10)
    # ★ 완주 신호. 경로 끝에서 local_sliding_window 가 /local_path 발행을 멈추므로
    # 아래 path_timeout 에 걸리는데, 그건 고장이 아니라 정상 종료다.
    # 이 구분이 없으면 대회 중 완주했는데 '경로 0.5s 끊김' 경고만 보인다.
    self.goal_reached = False
    self.create_subscription(Bool, '/goal_reached', self.goal_cb, 10)
    self.create_timer(1.0 / rate, self.control)

    self.get_logger().info(
        f'local_pure_pursuit 시작: L={self.L}m, max={math.degrees(self.max_steer):.0f}°, '
        f'target_speed={self.target_speed}m/s, {lp_topic}→{cmd_topic}')

  def path_cb(self, msg: Path):
    self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    self.last_path_time = self.get_clock().now()

  def odom_cb(self, msg: Odometry):
    # 말이 안 되는 속도는 버리고 직전 값을 유지한다(위 max_plausible_speed 주석).
    v = abs(float(msg.twist.twist.linear.x))
    if not math.isfinite(v) or v > self.max_plausible_speed:
      self.speed_rejects += 1
      self.get_logger().warn(
          f'odom 속도 {v:.1f}m/s — 비현실적이라 버린다 '
          f'(직전 {self.speed:.2f} 유지, 누적 {self.speed_rejects}회). '
          'EKF twist 스파이크로 보이며 위치는 보통 멀쩡하다.',
          throttle_duration_sec=5.0)
      return
    self.speed = v

  def curv_cb(self, msg: Float64):
    self.curvature = float(msg.data)

  def goal_cb(self, msg: Bool):
    if bool(msg.data) and not self.goal_reached:
      self.goal_reached = True
      self.get_logger().info('🏁 완주 신호 수신 — 조향 중앙 복귀 후 정지 (고장 아님)')

  def stop(self, reason=None):
    # 속도는 0, 조향은 슬루레이트로 서서히 중앙(0)으로 (급조향 없이 안전 정지)
    max_step = self.max_steer_rate / self.control_rate
    if self.prev_steer_deg > 0:
      self.prev_steer_deg = max(0.0, self.prev_steer_deg - max_step)
    else:
      self.prev_steer_deg = min(0.0, self.prev_steer_deg + max_step)
    self.steer_pub.publish(Float64(data=float(self.prev_steer_deg)))
    if self.standalone:
      cmd = Twist()
      cmd.angular.z = float(self.prev_steer_deg)
      self.cmd_pub.publish(cmd)
    if reason:
      # 완주는 정상 종료다 — WARN 으로 찍으면 현장에서 고장으로 오인한다.
      # ★ 2026-08-19 크래시 수정: info/warn 을 같은 줄에서 호출하면 rclpy 가
      # 'Logger severity cannot be changed between calls' 로 죽는다(throttle 은
      # 호출 위치로 캐싱하는데 같은 위치에서 심각도가 바뀌므로). 완주 때 실제로
      # 노드가 죽었다. 심각도별로 호출 위치(줄)를 분리한다.
      if reason.startswith('🏁'):
        self.get_logger().info(f'정지: {reason}', throttle_duration_sec=5.0)
      else:
        self.get_logger().warn(f'정지: {reason}', throttle_duration_sec=5.0)

  def find_lookahead(self, ld_target):
    """/local_path(차량기준) 위에서 차량으로부터 호길이 ld_target 인 점을 찾는다."""
    pts = self.path_pts
    if len(pts) < 2:
      return None
    s = 0.0
    for i in range(1, len(pts)):
      seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
      if s + seg >= ld_target:
        # 구간 내 선형보간
        t = (ld_target - s) / seg if seg > 1e-6 else 0.0
        x = pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0])
        y = pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1])
        return (x, y)
      s += seg
    return pts[-1]   # 경로가 짧으면 마지막 점

  def control(self):
    # 완주가 먼저다 — 완주하면 /local_path 가 끊기므로 아래 타임아웃에 걸린다.
    if self.goal_reached:
      self.stop('🏁 완주 (정상 종료)')
      return
    # 안전: 경로가 최근에 안 왔으면 정지
    if self.last_path_time is None:
      self.stop('경로 대기중')
      return
    dt = (self.get_clock().now() - self.last_path_time).nanoseconds * 1e-9
    if dt > self.path_timeout:
      self.stop(f'⚠ 경로 {dt:.1f}s 끊김 — 완주 신호 없음. GPS/IMU 확인')
      return
    if len(self.path_pts) < 2:
      self.stop('경로 점 부족')
      return

    # 1. 속도비례 lookahead
    ld = min(max(self.k_ld * self.speed + self.min_ld, self.min_ld), self.max_ld)

    # 2. lookahead 점 (이미 차량기준)
    la = self.find_lookahead(ld)
    if la is None:
      self.stop('lookahead 없음')
      return
    x_ld, y_ld = la
    ld_dist = math.hypot(x_ld, y_ld)

    # 3. 조향: 뒤(x<0)면 이상 → 정지(안전). 아니면 pure pursuit
    if x_ld <= 0.05 or ld_dist < 0.1:
      self.stop('lookahead가 차량 뒤/너무 가까움 (헤딩 이상 가능)')
      return
    kappa = 2.0 * y_ld / (ld_dist * ld_dist)
    delta = math.atan(self.L * kappa)
    delta = max(-self.max_steer, min(self.max_steer, delta))
    delta_deg = math.degrees(delta)

    # 4. 조향 슬루레이트 제한 (부드러운 sweep): 이번 사이클 최대 변화량 이내로
    max_step = self.max_steer_rate / self.control_rate
    delta_deg = max(self.prev_steer_deg - max_step,
                    min(self.prev_steer_deg + max_step, delta_deg))
    self.prev_steer_deg = delta_deg

    # 5. 속도: 경로 곡률 클수록 감속
    v = self.target_speed / (1.0 + self.curv_gain * abs(self.curvature))
    v = max(self.min_speed, min(self.target_speed, v))

    # 6. 조향각 발행 (도). 속도는 longitudinal_controller가 소유하므로 여기선
    #    참고용으로만 계산한다(standalone 모드에서만 /cmd_vel로 함께 나감).
    self.steer_pub.publish(Float64(data=float(delta_deg)))
    if self.standalone:
      cmd = Twist()
      cmd.linear.x = float(v)
      cmd.angular.z = float(delta_deg)
      self.cmd_pub.publish(cmd)


def main(args=None):
  rclpy.init(args=args)
  node = LocalPurePursuit()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node.stop()   # 종료 시 정지 명령
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_vehicle_node.py — 폐루프 가상 차량 (RViz 시뮬레이션용).

기존 sim_odom_publisher 와의 차이:
  · sim_odom_publisher : 가상 차량을 **전역 경로 위에 강제로 올려놓고** 진행시킨다.
                         위치가 항상 정답이라 추종 오차가 원리적으로 0이다.
                         로컬 경로가 그려지는지'만' 볼 수 있다.
  · 이 노드           : /local_path 를 구독해 pure pursuit 조향을 직접 계산하고,
                         자전거 모델로 움직인 결과를 /odometry/filtered 로 낸다.
                         차가 경로를 벗어나면 그 벗어난 위치에서 다음 로컬 경로가
                         만들어지므로 **실제 추종 성능이 RViz에 그대로 보인다.**

  local_sliding_window_node ──/local_path──▶ (이 노드) ──/odometry/filtered──┐
              ▲                                                              │
              └──────────────────────────────────────────────────────────────┘

조향식·슬루레이트·타각 한계는 local_pure_pursuit_node 와 동일하게 맞췄다.
파라미터를 바꾸면 그 결과가 바로 RViz에 나타난다.

발행:
  /odometry/filtered  : 가상 차량 위치·자세
  map → base_link TF
  /sim_trail          : 실제로 지나온 궤적 (경로와 겹쳐 보면 오차가 눈에 보임)
  /sim_cte            : 현재 횡방향 오차 [m]

사용:
  ros2 launch waypoint_follower sim_tracking.launch.py
  ros2 launch waypoint_follower sim_tracking.launch.py speed:=0.35 init_offset:=2.0
"""

import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

DEFAULT_WP = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
              'waypoints_recorded_resampled_0.5.yaml')


def yaw_to_quat(yaw):
  q = Quaternion()
  q.z = math.sin(yaw / 2.0)
  q.w = math.cos(yaw / 2.0)
  return q


class SimVehicle(Node):

  def __init__(self):
    super().__init__('sim_vehicle')
    self.declare_parameter('path_file', DEFAULT_WP)
    self.declare_parameter('rate', 20.0)
    # 차량 제원 — local_pure_pursuit_node 와 동일하게
    self.declare_parameter('wheelbase', 0.785)
    self.declare_parameter('max_steering_deg', 18.0)
    self.declare_parameter('max_steer_rate_deg', 90.0)
    # pure pursuit
    self.declare_parameter('k_ld', 0.6)
    self.declare_parameter('min_lookahead', 1.0)
    self.declare_parameter('max_lookahead', 4.0)
    # 속도 (longitudinal_controller 와 같은 곡률 감속식)
    self.declare_parameter('speed', 1.0)
    self.declare_parameter('min_speed', 0.4)
    self.declare_parameter('curvature_speed_gain', 2.0)
    # 출발 시 의도적 오차 — 복귀 성능을 눈으로 확인
    self.declare_parameter('init_offset', 0.0)
    self.declare_parameter('init_heading_err', 0.0)
    # 완주 후 자동 재시작 (계속 돌려보고 싶을 때)
    self.declare_parameter('restart_on_goal', True)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.L = float(g('wheelbase'))
    self.max_steer = math.radians(float(g('max_steering_deg')))
    self.rate = float(g('rate'))
    self.max_step = math.radians(float(g('max_steer_rate_deg'))) / self.rate
    self.k_ld = float(g('k_ld'))
    self.min_ld = float(g('min_lookahead'))
    self.max_ld = float(g('max_lookahead'))
    self.v_max = float(g('speed'))
    # min_speed 가 speed 보다 크면 곡률 감속이 통째로 무력화되고 목표속도도
    # 무시된다(clamp 순서상 min 이 이긴다). speed:=0.35 로 줬는데 0.40 으로
    # 달리는 일이 실제로 있었다 → 여기서 눌러준다.
    self.v_min = min(float(g('min_speed')), self.v_max)
    self.curv_gain = float(g('curvature_speed_gain'))
    self.restart = bool(g('restart_on_goal'))

    self.wps = self.load(str(g('path_file')))
    self.init_offset = float(g('init_offset'))
    self.init_head_err = float(g('init_heading_err'))
    self.reset_pose()

    self.delta = 0.0
    self.speed = 0.0
    self.path_pts = []
    self.curvature = 0.0
    self.trail = []
    self.goal = False
    self.halt = None
    self.restart_ticks = 0

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
    self.trail_pub = self.create_publisher(Path, '/sim_trail', latched)
    self.cte_pub = self.create_publisher(Float64, '/sim_cte', 10)
    self.tfb = TransformBroadcaster(self)
    self.create_subscription(Path, '/local_path', self.path_cb, 10)
    self.create_subscription(Float64, '/curvature', self.curv_cb, 10)
    self.create_timer(1.0 / self.rate, self.tick)
    self.create_timer(2.0, self.report)

    self.get_logger().info(
        f'가상 차량(폐루프) 시작: 축거 {self.L}m, 최대타각 '
        f'{math.degrees(self.max_steer):.0f}°, 목표속도 {self.v_max}m/s')
    if self.init_offset or self.init_head_err:
      self.get_logger().warn(
          f'의도적 초기 오차: 횡 {self.init_offset:+.1f}m, '
          f'헤딩 {self.init_head_err:+.0f}° — 복귀 과정을 RViz에서 확인하세요')

  def load(self, f):
    with open(f) as fh:
      d = yaml.safe_load(fh)
    raw = d.get('waypoints', d.get('poses'))
    return np.array([[float(p['x']), float(p['y'])] for p in raw])

  def reset_pose(self):
    self.x, self.y = float(self.wps[0][0]), float(self.wps[0][1])
    yaw0 = math.atan2(self.wps[1][1] - self.wps[0][1],
                      self.wps[1][0] - self.wps[0][0])
    self.yaw = yaw0 + math.radians(self.init_head_err)
    self.x += -math.sin(yaw0) * self.init_offset
    self.y += math.cos(yaw0) * self.init_offset
    self.travelled = 0.0

  def path_cb(self, msg):
    self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

  def curv_cb(self, msg):
    self.curvature = abs(float(msg.data))

  def cross_track(self):
    a, b = self.wps[:-1], self.wps[1:]
    ab = b - a
    ap = np.array([self.x, self.y]) - a
    den = np.einsum('ij,ij->i', ab, ab)
    t = np.clip(np.einsum('ij,ij->i', ap, ab) / np.maximum(den, 1e-12), 0, 1)
    pr = a + t[:, None] * ab
    return float(np.min(np.hypot(pr[:, 0] - self.x, pr[:, 1] - self.y)))

  def find_lookahead(self, ld):
    pts = self.path_pts
    if len(pts) < 2:
      return None
    s = 0.0
    for i in range(1, len(pts)):
      seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
      if s + seg >= ld:
        t = (ld - s) / seg if seg > 1e-6 else 0.0
        return (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
                pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))
      s += seg
    return pts[-1]

  def tick(self):
    dt = 1.0 / self.rate
    self.halt = None
    # 경로가 아직 없으면 제자리에서 자세만 발행 (로컬 경로 생성에 odom 이 필요)
    if self.goal:
      pass
    elif len(self.path_pts) < 2:
      self.halt = f'/local_path 점 부족 ({len(self.path_pts)}개)'
    else:
      v = self.v_max / (1.0 + self.curv_gain * self.curvature)
      v = max(self.v_min, min(self.v_max, v))
      ld = min(max(self.k_ld * v + self.min_ld, self.min_ld), self.max_ld)
      la = self.find_lookahead(ld)
      if la is None:
        self.halt = 'lookahead 계산 실패'
      else:
        xl, yl = la
        d = math.hypot(xl, yl)
        if xl <= 0.05 or d < 0.1:
          # 실차(local_pure_pursuit_node)에서도 이 조건이면 정지한다.
          # 정지하면 자세가 안 변해 조건이 영원히 안 풀리는 교착이 된다.
          self.halt = (f'lookahead가 차량 뒤/근접 (x={xl:.2f} y={yl:.2f} '
                       f'd={d:.2f}, ld목표={ld:.2f})')
        else:
          kap = 2.0 * yl / (d * d)
          tgt = max(-self.max_steer, min(self.max_steer, math.atan(self.L * kap)))
          self.delta = max(self.delta - self.max_step,
                           min(self.delta + self.max_step, tgt))
          self.x += v * math.cos(self.yaw) * dt
          self.y += v * math.sin(self.yaw) * dt
          self.yaw += v / self.L * math.tan(self.delta) * dt
          self.travelled += v * dt
          self.speed = v
          self.trail.append((self.x, self.y))

    if self.halt:
      self.speed = 0.0
      self.get_logger().warn(f'정지: {self.halt}', throttle_duration_sec=2.0)
    self.publish()

  def publish(self):
    now = self.get_clock().now().to_msg()
    o = Odometry()
    o.header.stamp = now
    o.header.frame_id = 'map'
    o.child_frame_id = 'base_link'
    o.pose.pose.position.x = self.x
    o.pose.pose.position.y = self.y
    o.pose.pose.orientation = yaw_to_quat(self.yaw)
    o.twist.twist.linear.x = self.speed
    self.odom_pub.publish(o)

    t = TransformStamped()
    t.header.stamp = now
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    t.transform.translation.x = self.x
    t.transform.translation.y = self.y
    t.transform.rotation = yaw_to_quat(self.yaw)
    self.tfb.sendTransform(t)

    if self.trail:
      pa = Path()
      pa.header.stamp = now
      pa.header.frame_id = 'map'
      for tx, ty in self.trail[::2]:      # 표시용으로 절반만
        ps = PoseStamped()
        ps.header = pa.header
        ps.pose.position.x = tx
        ps.pose.position.y = ty
        ps.pose.orientation.w = 1.0
        pa.poses.append(ps)
      self.trail_pub.publish(pa)
      self.cte_pub.publish(Float64(data=self.cross_track()))

  def report(self):
    if not self.trail:
      self.get_logger().info('대기중 — /local_path 필요', throttle_duration_sec=5.0)
      return
    cte = self.cross_track()
    # 완주 판정: 마지막 웨이포인트 근처
    rem = math.hypot(self.wps[-1][0] - self.x, self.wps[-1][1] - self.y)
    if rem < 1.0 and not self.goal:
      self.goal = True
      arr = np.array([self.cross_track_at(p) for p in self.trail])
      self.get_logger().info(
          f'🏁 완주 — 주행 {self.travelled:.1f}m, '
          f'횡오차 평균 {arr.mean():.3f}m / 최대 {arr.max():.3f}m')
      if self.restart:
        # 타이머를 새로 만들면 완주할 때마다 쌓이므로 카운터로 처리한다.
        self.restart_ticks = 2          # report 주기 2s × 2 = 약 4초 뒤
        self.get_logger().info('잠시 뒤 재시작 — 궤적을 확인하세요')
      return
    if self.goal and self.restart:
      self.restart_ticks -= 1
      if self.restart_ticks <= 0:
        self.do_restart()
      return
    self.get_logger().info(
        f'{self.travelled:6.1f}m | 횡오차 {cte:5.3f}m | '
        f'조향 {math.degrees(self.delta):+5.1f}° | 속도 {self.speed:.2f}m/s | '
        f'남은거리 {rem:.1f}m')

  def cross_track_at(self, pt):
    a, b = self.wps[:-1], self.wps[1:]
    ab = b - a
    ap = np.array(pt) - a
    den = np.einsum('ij,ij->i', ab, ab)
    t = np.clip(np.einsum('ij,ij->i', ap, ab) / np.maximum(den, 1e-12), 0, 1)
    pr = a + t[:, None] * ab
    return float(np.min(np.hypot(pr[:, 0] - pt[0], pr[:, 1] - pt[1])))

  def do_restart(self):
    self.reset_pose()
    self.trail = []
    self.delta = 0.0
    self.goal = False
    self.halt = None
    self.restart_ticks = 0
    self.get_logger().info('재시작')


def main(args=None):
  rclpy.init(args=args)
  n = SimVehicle()
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

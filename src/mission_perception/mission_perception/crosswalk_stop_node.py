#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
crosswalk_stop_node.py — 횡단보도 정지선 앞 정지 → 3초 대기 → 재출발.

    stop_point_recorder 로 찍은 정지지점 + /odometry/filtered
        └─→ 이 노드 ─→ /stop_line_distance ─→ longitudinal_controller
                                              (√(2ad) 감속·정지 로직이 이미 있다)

★ 제어 코드는 한 줄도 안 바꾼다
  longitudinal_controller 는 /stop_line_distance 만 보고 선다
  (19m 안에서 v=√(2·decel·d), 0.3m 이하면 v=0 — longitudinal_controller_node.py:167-172).
  없던 것은 '**정지 후 N초 뒤 재출발**' 하나뿐이라, 그 상태기계만 여기서 얹는다.

★ traffic_light_bridge 와 동시에 띄우지 말 것
  둘 다 /stop_line_distance 를 발행한다. 같이 켜면 서로 덮어써서 어느 쪽도 제대로
  동작하지 않는다. 기동 시 발행자 수를 세어 경고한다.

★ 64cm 규정
  정지선에서 64cm 안에 서야 한다. 이 노드는 '정지지점까지의 거리'를 그대로 내보내고,
  실제로 멈춘 순간의 오차를 로그로 남긴다(합격/불합격을 숫자로 찍는다).
  기록은 **차를 세우고 싶은 자세 그대로 세운 뒤** stop_point_recorder 로 찍는 것이
  가장 정확하다(안테나 위치 기준).
  ⚠ longitudinal_controller 는 0.3m 이하에서 하드정지하므로, 정지지점을 정지선 위에
    찍으면 차는 그보다 앞에서 선다. 그만큼 여유를 보고 stop_bias 로 보정한다.

상태기계
  IDLE     전방 approach_range 안에 정지지점 없음        → 999 (제약 없음)
  APPROACH 접근 중                                       → 남은 거리 발행(감속)
  HOLD     stop_enter_dist 안 → 0.0 발행(완전정지 강제)
           실제로 멈추면(|v|<stopped_speed) dwell 타이머 시작
  CLEARED  dwell 경과 → 999 발행 + 그 지점 영구 제외      → 다시 출발

파라미터
  stop_points      : [x1,y1, x2,y2, ...] map 좌표. points_file 보다 우선
  points_file      : stop_point_recorder 가 저장한 yaml 경로
  dwell            : 정지 유지 시간 [s] (기본 3.0)
  stop_enter_dist  : 이 거리 안에 들면 완전정지 명령 [m] (기본 0.5)
  stop_bias        : 정지지점을 이만큼 앞당겨 본다 [m] (기본 0.0)
  stop_tolerance   : 합격 판정 거리 [m] (기본 0.64 — 대회 규정)
"""

import math
import os

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

NO_CONSTRAINT = 999.0


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


class CrosswalkStop(Node):

  def __init__(self):
    super().__init__('crosswalk_stop')

    self.declare_parameter('stop_points', rclpy.Parameter.Type.DOUBLE_ARRAY)
    self.declare_parameter('points_file', '')
    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('dwell', 3.0)
    self.declare_parameter('approach_range', 25.0)
    self.declare_parameter('stop_enter_dist', 0.5)
    self.declare_parameter('stop_bias', 0.0)
    self.declare_parameter('stop_tolerance', 0.64)
    self.declare_parameter('stopped_speed', 0.05)
    # 멈추라고 했는데 안 멈추면(크립·엔코더 이상) 영원히 대기한다. 탈출구를 둔다.
    self.declare_parameter('hold_timeout', 8.0)
    self.declare_parameter('rate', 10.0)
    # ★ 미션 시퀀서 연동 — require_arm:=true 면 /crosswalk/arm 이 true 일 때만
    #   동작한다. 기본 false 라 시퀀서 없이 쓰던 런치는 그대로 돌아간다.
    self.declare_parameter('require_arm', False)
    self.declare_parameter('arm_topic', '/crosswalk/arm')

    # 타입만 선언한 파라미터는 **미지정이면 get_parameter 가 예외를 던진다**.
    # points_file 만 쓰는 게 정상 사용법이므로 없을 때 None 으로 받아야 한다.
    g = lambda n: self.get_parameter_or(  # noqa: E731
        n, rclpy.parameter.Parameter(n, value=None)).value
    self.dwell = float(g('dwell'))
    self.approach_range = float(g('approach_range'))
    self.enter_dist = float(g('stop_enter_dist'))
    self.bias = float(g('stop_bias'))
    self.tolerance = float(g('stop_tolerance'))
    self.stopped_v = float(g('stopped_speed'))
    self.hold_timeout = float(g('hold_timeout'))

    self.points = self.load_points(list(g('stop_points') or []),
                                   str(g('points_file')))
    self.pose = None
    self.speed = 0.0
    self.state = 'IDLE'
    self.active = None        # 현재 붙잡고 있는 지점 인덱스
    self.hold_since = None    # HOLD 진입 시각
    self.dwell_since = None   # 실제로 멈춘 시각
    self.cleared = set()
    self.last_log = None
    self.require_arm = bool(g('require_arm'))
    # require_arm 이 아니면 처음부터 켜진 것으로 본다.
    self.armed = not self.require_arm

    self.pub = self.create_publisher(Float64, '/stop_line_distance', 10)
    self.state_pub = self.create_publisher(String, '/crosswalk/state', 10)
    self.done_pub = self.create_publisher(Bool, '/crosswalk/done', 10)
    self.create_subscription(Odometry, str(g('odom_topic')), self.odom_cb, 10)
    self.create_subscription(Bool, str(g('arm_topic')), self.arm_cb, 10)
    self.create_timer(1.0 / float(g('rate')), self.tick)
    self.create_timer(2.0, self.check_conflict)

    if not self.points:
      self.get_logger().error(
          '정지지점이 없다 — 항상 999 를 발행한다. stop_point_recorder 로 찍은 뒤 '
          'points_file 또는 stop_points 로 줄 것.')
    else:
      self.get_logger().info(
          f'횡단보도 정지 시작: 지점 {len(self.points)}개, {self.dwell:.1f}s 대기 후 '
          f'재출발 (합격 {self.tolerance*100:.0f}cm)')

  def load_points(self, flat, path):
    if flat:
      if len(flat) % 2 != 0:
        self.get_logger().error('stop_points 는 [x1,y1,x2,y2,...] 짝수여야 한다')
        return []
      return [(float(flat[i]), float(flat[i + 1]))
              for i in range(0, len(flat), 2)]
    if not path:
      return []
    try:
      with open(os.path.expanduser(path), encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
      return [(float(p['x']), float(p['y'])) for p in d['stop_points']]
    except Exception as e:  # noqa: BLE001
      self.get_logger().error(f'{path} 를 읽지 못했다: {e}')
      return []

  def check_conflict(self):
    n = self.count_publishers('/stop_line_distance')
    if n > 1:
      self.get_logger().error(
          f'⚠ /stop_line_distance 발행자가 {n}개다 — traffic_light_bridge 가 같이 '
          f'떠 있으면 서로 덮어써서 둘 다 망가진다. 하나만 띄울 것.')

  def odom_cb(self, msg: Odometry):
    p = msg.pose.pose
    self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
    self.speed = float(msg.twist.twist.linear.x)

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def forward_distance(self):
    """가장 가까운 유효 정지지점까지의 진행방향 거리와 인덱스.

    지나쳤다고 바로 놓으면 안 된다 — 살짝 넘어간 채로 제약이 사라지면 3초를
    안 채우고 그냥 출발한다. dwell 을 다 채운 지점만 cleared 로 영구 제외한다.
    """
    if self.pose is None or not self.points:
      return None, None
    x, y, yaw = self.pose
    c, s = math.cos(yaw), math.sin(yaw)
    best, best_i = None, None
    for i, (px, py) in enumerate(self.points):
      if i in self.cleared:
        continue
      dx, dy = px - x, py - y
      fwd = dx * c + dy * s - self.bias      # 진행방향 성분
      if fwd < -2.0:
        continue                              # 확실히 지나침
      if math.hypot(dx, dy) > self.approach_range:
        continue
      if best is None or fwd < best:
        best, best_i = fwd, i
    return best, best_i

  def arm_cb(self, msg: Bool):
    """시퀀서의 arm 신호. 내려갈 때 **제약 해제를 반드시 한 번 쏜다.**

    ★ 왜 그냥 발행을 끊으면 안 되는가 (2026-09-04 확인)
      longitudinal_controller 는 `/stop_line_distance` 를 받으면 그 값을
      **타임아웃 없이 계속 들고 있는다**(`self.stop_line_dist`). HOLD 중
      0.0 을 쏘던 상태에서 조용히 발행만 멈추면 그 0.0 이 영원히 남아
      차가 그 자리에 굳는다. **1분 이상 정지는 감점이 아니라 탈락이다.**
      그래서 내려갈 때 999(제약 없음)를 한 번 쏘고 토픽을 놓는다.
    """
    want = bool(msg.data)
    if want == self.armed:
      return
    self.armed = want
    if want:
      self.reset('IDLE')
      self.get_logger().info('▶ ARM — 횡단보도 정지 감시 시작')
    else:
      self.emit(NO_CONSTRAINT, None)     # ★ 브레이크를 반드시 풀고 나간다
      self.reset('IDLE')
      self.get_logger().info('■ DISARM — 제약 해제(999) 발행 후 토픽을 놓는다')

  def tick(self):
    if not self.armed:
      # 발행하지 않는다 — 다른 미션이 같은 토픽을 쓸 수 있게 비워둔다.
      self.state_pub.publish(String(data='DISARMED'))
      return
    dist, idx = self.forward_distance()

    if dist is None:
      self.reset('IDLE')
      self.emit(NO_CONSTRAINT, '전방 정지지점 없음')
      return

    # 붙잡던 지점이 바뀌면 상태를 초기화한다(다음 횡단보도).
    if self.active is not None and idx != self.active:
      self.reset('APPROACH')
    self.active = idx

    if self.state in ('IDLE', 'APPROACH') and dist > self.enter_dist:
      self.state = 'APPROACH'
      self.emit(max(0.0, dist), f'접근 중 — {dist:.2f}m')
      return

    # ---- HOLD: 완전정지 명령 ----
    if self.state in ('IDLE', 'APPROACH'):
      self.state = 'HOLD'
      self.hold_since = self.now()
      self.dwell_since = None
      err = abs(dist)
      ok = '합격' if err <= self.tolerance else '⚠ 초과'
      self.get_logger().info(
          f'정지지점 {idx} 도달 — 정지선까지 {err*100:.0f}cm '
          f'(규정 {self.tolerance*100:.0f}cm: {ok})')

    if self.state == 'HOLD':
      self.emit(0.0, None)         # 0.0 → longitudinal 이 v=0 으로 하드정지
      held = self.now() - self.hold_since

      if abs(self.speed) > self.stopped_v:
        self.dwell_since = None    # 아직 구르는 중 — 타이머 시작 안 함
        if held > self.hold_timeout:
          self.get_logger().warn(
              f'{held:.1f}s 동안 멈추지 않는다 (v={self.speed:+.2f}m/s) — '
              f'그래도 통과시킨다. 브레이크/엔코더 확인할 것.')
          self.clear(idx)
        else:
          self.log_once(f'정지 중… v={self.speed:+.2f}m/s')
        return

      if self.dwell_since is None:
        self.dwell_since = self.now()
        stop_err = abs(dist)
        self.get_logger().info(
            f'■ 정지 완료 — 정지선까지 {stop_err*100:.0f}cm. '
            f'{self.dwell:.1f}초 대기 시작')
        return

      waited = self.now() - self.dwell_since
      if waited >= self.dwell:
        self.get_logger().info(f'▶ {waited:.1f}초 대기 완료 — 재출발')
        self.clear(idx)
      else:
        self.log_once(f'대기 {waited:.1f}/{self.dwell:.1f}s')
      return

    # CLEARED 이후 (같은 지점 재진입 방지용 안전망)
    self.emit(NO_CONSTRAINT, None)

  def clear(self, idx):
    self.cleared.add(idx)
    self.state = 'CLEARED'
    self.active = None
    self.hold_since = None
    self.dwell_since = None
    self.emit(NO_CONSTRAINT, None)
    self.done_pub.publish(Bool(data=True))

  def reset(self, state):
    self.state = state
    self.hold_since = None
    self.dwell_since = None

  def emit(self, dist, why):
    self.pub.publish(Float64(data=float(dist)))
    self.state_pub.publish(String(data=self.state))
    if why:
      self.log_once(why)

  def log_once(self, msg):
    if msg != self.last_log:
      self.get_logger().info(msg, throttle_duration_sec=1.0)
      self.last_log = msg


def main(args=None):
  rclpy.init(args=args)
  node = None
  try:
    node = CrosswalkStop()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if node is not None:
      node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

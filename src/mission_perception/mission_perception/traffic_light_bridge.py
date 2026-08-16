#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light_bridge.py — 신호등 상태 + GPS 정지지점 → /stop_line_distance

파이(Hailo-8)가 쏘는 `/traffic_light_state` 는 "빨강이다"까지만 알려주고
**어디서 서야 하는지**는 모른다. 반대로 `longitudinal_controller` 는
정지선까지의 **거리**만 있으면 알아서 선다(√(2ad) 감속 프로파일이 이미 있다).

이 노드가 그 사이를 잇는다.

    /traffic_light_state (파이)  ─┐
                                  ├─→ /stop_line_distance ─→ longitudinal_controller
    /odometry/filtered + 정지지점 ─┘                            (기존 감속 로직이 처리)

★ 왜 정지선을 카메라로 안 찾는가
  정지선·횡단보도는 지금 모델에 클래스가 없고, 새로 학습하려면 데이터 수집부터
  필요하다. 반면 정지 지점은 **GPS 경로에 미리 찍어두면** 된다. RTK 가 1.4cm
  라 위치 정확도는 카메라보다 낫고, 대회 당일 조명·그림자에도 흔들리지 않는다.
  제어 쪽 인터페이스가 '거리 숫자 하나'라서 이 방식이 그대로 들어맞는다.

★ 제어 쪽은 한 줄도 안 바꾼다
  longitudinal_controller 의 정지선 로직은 이미 완성돼 있었고 입력만 없었다.
  (stop_line_trigger 19m 안에 들어오면 v=√(2·decel·d), 0.3m 이하면 정지)

파라미터:
  state_topic        : 신호등 상태 토픽 (기본 /traffic_light_state)
                       COCO 검출기로 바꾸려면 /traffic_light_state_coco
  stop_points        : 정지 지점 [x1,y1, x2,y2, ...] map 좌표 [m]
  approach_range     : 이 거리 안에 들어와야 정지 지점으로 인정 (기본 25m)
  yellow_pass_dist   : 노랑일 때 이 거리보다 가까우면 그냥 통과 (기본 2.0m)
  on_none            : 신호 미검출 시 'stop' 또는 'pass' (기본 stop)
  none_timeout       : 미검출로 멈춘 채 이 시간이 지나면 통과 (기본 10s, 0=무한대기)
  state_timeout      : 상태 토픽이 이 시간 끊기면 미검출로 간주 (기본 1.0s)

출력:
  /stop_line_distance  Float64  정지지점까지 [m]. 제약 없으면 999.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String

NO_CONSTRAINT = 999.0


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


class TrafficLightBridge(Node):

  def __init__(self):
    super().__init__('traffic_light_bridge')

    self.declare_parameter('state_topic', '/traffic_light_state')
    self.declare_parameter('odom_topic', '/odometry/filtered')
    # 빈 리스트를 기본값으로 주면 BYTE_ARRAY 로 추론돼 DOUBLE_ARRAY 를 거부한다.
    # 값 대신 '타입'으로 선언하면(미설정 상태) 이 문제가 없다.
    self.declare_parameter('stop_points', rclpy.Parameter.Type.DOUBLE_ARRAY)
    self.declare_parameter('approach_range', 25.0)
    self.declare_parameter('yellow_pass_dist', 2.0)
    self.declare_parameter('on_none', 'stop')
    self.declare_parameter('none_timeout', 10.0)
    self.declare_parameter('state_timeout', 1.0)
    self.declare_parameter('rate', 10.0)
    # 정지 중 살짝 지나쳐도 놓지 않는 여유. 이만큼 넘어가도 계속 붙잡는다.
    self.declare_parameter('overshoot_margin', 1.5)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    state_topic = str(g('state_topic'))
    odom_topic = str(g('odom_topic'))
    self.approach_range = float(g('approach_range'))
    self.yellow_pass = float(g('yellow_pass_dist'))
    self.on_none = str(g('on_none')).lower()
    self.none_timeout = float(g('none_timeout'))
    self.state_timeout = float(g('state_timeout'))
    self.overshoot = float(g('overshoot_margin'))

    # 타입만 선언했으므로 미지정이면 None 이 온다
    flat = list(g('stop_points') or [])
    if len(flat) % 2 != 0:
      raise ValueError('stop_points 는 [x1,y1,x2,y2,...] 짝수 개여야 한다')
    self.stop_points = [(float(flat[i]), float(flat[i + 1]))
                        for i in range(0, len(flat), 2)]

    self.state = 'NONE'
    self.state_time = None
    self.pose = None          # (x, y, yaw)
    self.none_since = None    # 미검출로 붙잡힌 시각
    self.last_pub = None
    self.cleared = set()      # GREEN 으로 통과 확정된 정지지점 인덱스

    self.pub = self.create_publisher(Float64, '/stop_line_distance', 10)
    self.create_subscription(String, state_topic, self.state_cb, 10)
    self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
    self.create_timer(1.0 / float(g('rate')), self.tick)

    if not self.stop_points:
      self.get_logger().warn(
          '정지 지점이 하나도 없다 — 항상 999(제약 없음)를 발행한다. '
          'stop_points 파라미터로 [x1,y1,x2,y2,...] 를 줄 것.')
    self.get_logger().info(
        f'신호등 다리 시작: {state_topic} + 정지지점 {len(self.stop_points)}개 '
        f'→ /stop_line_distance (미검출 시 {self.on_none}, '
        f'{self.none_timeout:.0f}s 뒤 통과)')

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def state_cb(self, msg):
    self.state = msg.data.strip().upper()
    self.state_time = self.now()

  def odom_cb(self, msg):
    p = msg.pose.pose
    self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

  def forward_distance(self):
    """가장 가까운 유효 정지 지점까지의 진행방향 거리와 인덱스.

    ★ 지나쳤다고 바로 놓으면 안 된다.
      빨간불에 서다가 정지 지점을 조금 넘어가면, 그 지점이 '뒤'가 되면서
      제약이 사라지고 차가 그대로 출발한다 — 신호 위반이다. 그래서
      overshoot_margin 만큼 넘어가도 계속 붙잡고, **GREEN 으로 통과가
      확정된 지점만** cleared 에 넣어 영구히 제외한다.

    반환: (거리[m], 인덱스) 또는 (None, None)
    """
    if self.pose is None or not self.stop_points:
      return None, None
    x, y, yaw = self.pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    best, best_i = None, None
    for i, (px, py) in enumerate(self.stop_points):
      if i in self.cleared:
        continue                        # 이미 통과 확정
      dx, dy = px - x, py - y
      fwd = dx * cy + dy * sy          # 진행방향 성분
      if fwd < -self.overshoot:
        continue                        # 여유를 넘어 확실히 지나침
      if math.hypot(dx, dy) > self.approach_range:
        continue                        # 아직 멀다
      if best is None or fwd < best:
        best, best_i = fwd, i
    return best, best_i

  def decide(self, state, dist, stale, none_held):
    """발행할 거리를 정한다. 순수 함수라 오프라인 테스트가 된다.

    state     : 'RED'/'YELLOW'/'GREEN'/'NONE'
    dist      : 전방 정지지점까지 [m], 없으면 None
    stale     : 상태 토픽이 끊겼는가
    none_held : 미검출로 붙잡힌 시간 [s] (붙잡힌 적 없으면 0)
    반환      : (발행거리, 사유)
    """
    if dist is None:
      return NO_CONSTRAINT, '전방 정지지점 없음'

    eff = 'NONE' if stale else state

    if eff == 'GREEN':
      return NO_CONSTRAINT, f'GREEN — 통과 ({dist:.1f}m)'

    if eff == 'RED':
      return dist, f'RED — 정지 ({dist:.1f}m)'

    if eff == 'YELLOW':
      # 이미 코앞이면 급정거보다 통과가 안전하다
      if dist <= self.yellow_pass:
        return NO_CONSTRAINT, f'YELLOW — 너무 가까워 통과 ({dist:.1f}m)'
      return dist, f'YELLOW — 정지 ({dist:.1f}m)'

    # ---- 미검출(NONE) 또는 링크 끊김 ----
    why = '링크끊김' if stale else '미검출'
    if self.on_none != 'stop':
      return NO_CONSTRAINT, f'{why} — 정책상 통과'
    # 안전상 일단 선다. 다만 영원히 서 있으면 완주를 못 하므로 탈출구를 둔다.
    if self.none_timeout > 0.0 and none_held >= self.none_timeout:
      return NO_CONSTRAINT, (f'{why} — {none_held:.0f}s 대기 후 통과 '
                             f'(검출 실패로 판단)')
    return dist, f'{why} — 정지 ({dist:.1f}m, {none_held:.0f}s 대기)'

  def tick(self):
    stale = (self.state_time is None
             or (self.now() - self.state_time) > self.state_timeout)
    dist, idx = self.forward_distance()

    # GREEN 으로 실제로 지나간 지점만 통과 확정 처리한다.
    if (idx is not None and not stale and self.state == 'GREEN'
            and dist is not None and dist < 0.0):
      self.cleared.add(idx)
      self.get_logger().info(f'정지지점 {idx} 통과 확정 (GREEN)')
      dist, idx = self.forward_distance()

    # 지나친 뒤(음수)에도 붙잡고 있는 동안엔 0 으로 취급해 계속 세운다
    if dist is not None and dist < 0.0:
      dist = 0.0

    eff_none = stale or self.state not in ('RED', 'YELLOW', 'GREEN')

    # 미검출로 붙잡힌 시간 누적 (정지지점이 앞에 있을 때만)
    if eff_none and dist is not None:
      if self.none_since is None:
        self.none_since = self.now()
      held = self.now() - self.none_since
    else:
      self.none_since = None
      held = 0.0

    out, why = self.decide(self.state, dist, stale, held)
    self.pub.publish(Float64(data=float(out)))

    if why != self.last_pub:
      self.get_logger().info(why)
      self.last_pub = why


def main(args=None):
  rclpy.init(args=args)
  node = None
  try:
    node = TrafficLightBridge()
    rclpy.spin(node)
  except (KeyboardInterrupt, ValueError) as e:
    if isinstance(e, ValueError):
      print(f'[ERROR] {e}')
  finally:
    if node is not None:
      node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

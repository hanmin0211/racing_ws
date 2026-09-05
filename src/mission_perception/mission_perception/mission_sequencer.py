#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mission_sequencer.py — 코스 위 미션들을 순서대로 켜고 끄는 상위 상태기계.

★ 왜 필요한가 (미션 노드가 이미 각자 상태기계를 갖고 있는데도)

  1) **동시 활성이 곧 사고다.**
     parking_node 는 `/teleop/cmd_vel`, crosswalk_stop_node 와
     traffic_light_bridge 는 `/stop_line_distance` 를 쓴다. 둘 이상이 동시에
     켜지면 서로 덮어쓴다. 지금까지는 "mission:=true 와 crosswalk:=true 를
     같이 켜지 말 것" 이라는 **사람이 지키는 규칙**으로 막고 있었다.
     용인은 미션이 8개다 — 사람이 지킬 수 있는 규모가 아니다.
     여기서 **한 번에 하나만 arm** 한다.

  2) **미션 실패는 감점, 멈춤은 탈락.**
     탈락 조건은 이탈 / 1분 이상 정지 / DNF / 입력장치뿐이다. 미션이 걸려
     제자리에 서 있으면 감점이 아니라 **탈락**이다. 그래서 미션마다 타임아웃을
     두고, 넘으면 **강제로 disarm 하고 자율 주행으로 복귀**시킨다.
     "그 미션을 포기한다" 가 언제나 "거기서 멈춘다" 보다 낫다.

  3) **8분 예산.**
     남은 시간이 모자라면 감점-only 미션(critical: false)을 건너뛴다.
     완주가 먼저다.

★ 어떻게 켜는가 — arm 토픽
  미션 노드마다 `arm_topic` (std_msgs/Bool) 하나를 둔다. true 면 그 미션이
  자기 일을 하고, false 면 아무것도 발행하지 않는다.
  parking_node 는 이미 `/parking/start` 가 그 역할을 한다.

  ※ 시퀀서는 노드를 죽이거나 띄우지 않는다. 다 띄워놓고 arm 으로만 통제한다.
    런치 중에 프로세스를 죽이면 복구가 안 되고, 무엇보다 **시험이 불가능**해진다.

★ 코스 위 어디인지 — 진행거리 s
  `/global_path` 를 받아 누적 호길이를 만들고, 현재 위치의 최근접 점에서 s 를
  읽는다. 미션은 `s_enter ~ s_exit` 구간에서 arm 된다. 좌표 기반(point) 도 지원.

  왜 s 인가: 코스는 한 줄이고 미션은 그 위 특정 구간에 있다. 좌표만 보면
  경로가 자기 근처로 되돌아올 때 오작동한다(굴절·방향전환에서 실제로 생긴다).

설정: config/mission_plan.yaml (장소마다 이 파일만 갈아끼운다)
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped  # noqa: F401  (Path 안에 들어있다)
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

DEFAULT_PLAN = '/home/han/racing_ws/config/mission_plan.yaml'

# 미션 하나의 진행 상태
PENDING = 'PENDING'    # 아직 구간에 안 들어옴
ARMED = 'ARMED'        # 켜져 있음
DONE = 'DONE'          # 완료 신호 받음
TIMEOUT = 'TIMEOUT'    # 시간 초과 — 포기하고 자율 복귀
SKIPPED = 'SKIPPED'    # 시간예산 부족으로 건너뜀


class Mission:
  """계획 파일 한 항목 + 실행 중 상태."""

  def __init__(self, d):
    self.name = str(d['name'])
    self.arm_topic = str(d['arm_topic'])
    self.done_topic = d.get('done_topic')
    self.timeout = float(d.get('timeout_s', 30.0))
    self.expected_s = float(d.get('expected_s', self.timeout))
    self.critical = bool(d.get('critical', False))
    # ★ exclusive=False 는 '배경 기능(overlay)' 이다.
    #   라이다 회피처럼 차를 직접 몰지 않고 **조향을 덧씌우기만** 하는 것은
    #   구간 미션과 성격이 다르다. 하나만 켜는 규칙에 묶으면 라이다가 켜진
    #   동안 횡단보도·주차가 아예 못 켜진다. overlay 는:
    #     · 다른 미션을 막지 않고, 다른 미션에 막히지도 않는다
    #     · 구간 안에 있으면 켜고, 벗어나면 끈다 (done 신호가 없다)
    #     · 타임아웃이 없다 — 끝나는 게 아니라 구간을 벗어나는 것이다
    self.exclusive = bool(d.get('exclusive', True))
    # ★ 이 미션이 켜져 있는 동안 **강제로 꺼야 하는** 배경 기능들.
    #   규정(항목 7 돌발상황 급정지): "회피기동은 미션 성공으로 인정하지 않음".
    #   즉 더미 앞에서는 **반드시 서야** 하고, 라이다가 옆으로 돌아 나가면
    #   미션 실패(10점)다. 배치를 s 구간으로만 관리하면 계획 파일을 잘못 적었을
    #   때 조용히 회피해 버린다. 여기서 **명시적으로** 막는다.
    self.inhibits = [str(v) for v in (d.get('inhibits') or [])]
    self.select_topic = d.get('select_topic')     # 예: /parking/select
    self.select_value = d.get('select_value')

    t = d.get('trigger') or {}
    self.trig_type = str(t.get('type', 'course_s'))
    self.s_enter = float(t.get('s_enter', 0.0))
    self.s_exit = float(t.get('s_exit', 1e9))
    self.px = float(t.get('x', 0.0))
    self.py = float(t.get('y', 0.0))
    self.radius = float(t.get('radius', 3.0))

    self.state = PENDING
    self.armed_at = None
    self.finished_at = None

  def in_zone(self, s, x, y):
    if self.trig_type == 'point':
      return math.hypot(x - self.px, y - self.py) <= self.radius
    if s is None:
      return False
    return self.s_enter <= s <= self.s_exit

  def past_zone(self, s):
    """구간을 지나쳤는가 — arm 도 못 해보고 흘려보낸 경우를 잡는다."""
    if self.trig_type == 'point' or s is None:
      return False
    return s > self.s_exit


class MissionSequencer(Node):

  def __init__(self):
    super().__init__('mission_sequencer')

    self.declare_parameter('plan_file', DEFAULT_PLAN)
    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('rate', 10.0)
    # 8분. 초과하면 1분당 5점 감점이라 완주를 우선한다.
    self.declare_parameter('time_budget_s', 480.0)
    # 예산에서 이만큼은 완주용으로 남겨둔다(미션에 안 씀).
    self.declare_parameter('reserve_s', 60.0)
    # 자동 시작: 첫 odom 부터 시계를 돌린다. false 면 /mission/start 를 기다린다.
    self.declare_parameter('auto_start', True)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    plan_file = str(g('plan_file'))
    self.rate = float(g('rate'))
    self.budget = float(g('time_budget_s'))
    self.reserve = float(g('reserve_s'))
    self.auto_start = bool(g('auto_start'))

    self.missions = self.load_plan(plan_file)
    if not self.missions:
      self.get_logger().error(
          f'❌ 미션 계획이 비었다: {plan_file} — 아무것도 arm 하지 않는다.')

    # 경로(진행거리 계산용)
    self.path_xy = []
    self.path_s = []
    self.s_now = None
    self.pose = None
    self.t_start = None
    self.active = None          # 지금 arm 된 Mission (동시 하나만)
    self._last_inhibited = set()

    # 발행자 — 미션마다 arm 토픽 하나
    self.arm_pubs = {m.name: self.create_publisher(Bool, m.arm_topic, 10)
                     for m in self.missions}
    self.sel_pubs = {}
    for m in self.missions:
      if m.select_topic:
        from std_msgs.msg import Int32
        self.sel_pubs[m.name] = self.create_publisher(Int32, m.select_topic, 10)
    self.active_pub = self.create_publisher(String, '/mission/active', 10)
    self.state_pub = self.create_publisher(String, '/mission/state', 10)

    qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Path, '/global_path', self.path_cb, qos_tl)
    self.create_subscription(Odometry, str(g('odom_topic')), self.odom_cb, 10)
    self.create_subscription(Bool, '/mission/start', self.start_cb, 10)
    for m in self.missions:
      if m.done_topic:
        self.create_subscription(
            Bool, m.done_topic,
            (lambda mm: (lambda msg: self.done_cb(mm, msg)))(m), 10)

    self.create_timer(1.0 / self.rate, self.tick)
    self.get_logger().info(
        f'미션 시퀀서 시작 — {len(self.missions)}개, 예산 {self.budget:.0f}s '
        f'(예비 {self.reserve:.0f}s)')
    for m in self.missions:
      z = (f'점({m.px:.1f}, {m.py:.1f}) r{m.radius:.1f}'
           if m.trig_type == 'point' else f's {m.s_enter:.1f}~{m.s_exit:.1f}m')
      kind = '배경' if not m.exclusive else (
          '필수' if m.critical else '감점-only')
      tmo = '—' if not m.exclusive else f'{m.timeout:.0f}s'
      inh = f'  ⊘{",".join(m.inhibits)}' if m.inhibits else ''
      self.get_logger().info(
          f'  · {m.name:<16} {z:<24} 타임아웃 {tmo:<5} [{kind}]{inh}')

  # ------------------------------------------------------------------ 로드
  def load_plan(self, path):
    if not os.path.exists(path):
      self.get_logger().error(f'계획 파일 없음: {path}')
      return []
    try:
      d = yaml.safe_load(open(path, encoding='utf-8')) or {}
    except Exception as e:  # noqa: BLE001
      self.get_logger().error(f'계획 파일 로드 실패: {e}')
      return []
    out = []
    for item in (d.get('missions') or []):
      try:
        out.append(Mission(item))
      except Exception as e:  # noqa: BLE001
        self.get_logger().error(f'미션 항목 무시({e}): {item}')
    # s 순서대로 — 계획 파일 순서 실수를 흡수한다.
    out.sort(key=lambda m: (m.s_enter if m.trig_type != 'point' else 0.0))
    return out

  # ------------------------------------------------------------------ 입력
  def path_cb(self, msg: Path):
    xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    if len(xy) < 2:
      return
    s = [0.0]
    for i in range(1, len(xy)):
      s.append(s[-1] + math.hypot(xy[i][0] - xy[i - 1][0],
                                  xy[i][1] - xy[i - 1][1]))
    if len(xy) != len(self.path_xy):
      self.get_logger().info(
          f'전역 경로 수신: {len(xy)}점 {s[-1]:.1f}m — 진행거리 기준 확보')
    self.path_xy, self.path_s = xy, s

  def odom_cb(self, msg: Odometry):
    p = msg.pose.pose.position
    self.pose = (p.x, p.y)
    if self.t_start is None and self.auto_start:
      self.t_start = self.now()
      self.get_logger().info('▶ 주행 시계 시작')
    if self.path_xy:
      best, bi = float('inf'), 0
      for i, (ax, ay) in enumerate(self.path_xy):
        d = (p.x - ax) ** 2 + (p.y - ay) ** 2
        if d < best:
          best, bi = d, i
      self.s_now = self.path_s[bi]

  def start_cb(self, msg: Bool):
    if msg.data and self.t_start is None:
      self.t_start = self.now()
      self.get_logger().info('▶ 주행 시계 시작 (/mission/start)')

  def done_cb(self, m: Mission, msg: Bool):
    if m.state != ARMED:
      return
    # ★ done=false 도 '끝났다' 로 받는다. 실패해도 다음으로 가야 한다.
    #   미션 실패는 감점, 거기 머무르면 탈락이다.
    m.state = DONE
    m.finished_at = self.now()
    took = m.finished_at - (m.armed_at or m.finished_at)
    self.get_logger().info(
        f'✅ {m.name} 종료 ({"성공" if msg.data else "실패"}) {took:.1f}s')
    self.disarm(m)

  # ------------------------------------------------------------------ 제어
  def s_fmt(self):
    return f'{self.s_now:.1f}m' if self.s_now is not None else '?'

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def elapsed(self):
    return 0.0 if self.t_start is None else self.now() - self.t_start

  def remaining(self):
    return self.budget - self.elapsed()

  def arm(self, m: Mission):
    m.state = ARMED
    m.armed_at = self.now()
    self.active = m
    if m.name in self.sel_pubs and m.select_value is not None:
      from std_msgs.msg import Int32
      self.sel_pubs[m.name].publish(Int32(data=int(m.select_value)))
    self.get_logger().info(
        f'▶ {m.name} ARM (s={self.s_now if self.s_now is not None else -1:.1f}m, '
        f'경과 {self.elapsed():.0f}s, 남은 {self.remaining():.0f}s)')

  def disarm(self, m: Mission):
    self.arm_pubs[m.name].publish(Bool(data=False))
    if self.active is m:
      self.active = None

  def tick(self):
    # 지금 켜진 구간 미션이 금지하는 배경 기능들
    inhibited = set(self.active.inhibits) if self.active else set()
    if inhibited != self._last_inhibited:
      for n in inhibited - self._last_inhibited:
        self.get_logger().warn(
            f'⊘ {n} 강제 차단 — {self.active.name} 구간 (규정상 회피 금지)')
      for n in self._last_inhibited - inhibited:
        self.get_logger().info(f'⊙ {n} 차단 해제')
      self._last_inhibited = inhibited

    # arm 상태를 계속 재발행한다 — 미션 노드가 늦게 떠도 받게, 그리고
    # 토픽 한 번 놓쳐서 미션이 조용히 안 켜지는 일이 없게.
    for m in self.missions:
      on = (m.state == ARMED) and (m.name not in inhibited)
      self.arm_pubs[m.name].publish(Bool(data=on))

    self.active_pub.publish(
        String(data=self.active.name if self.active else 'NONE'))
    self.state_pub.publish(String(data=self.summary()))

    if self.pose is None:
      return
    x, y = self.pose

    # 0) overlay(배경 기능) — 구간에 들어오면 켜고 벗어나면 끈다.
    #    exclusive 슬롯을 쓰지 않으므로 다른 미션과 상관없이 동작한다.
    for m in self.missions:
      if m.exclusive or m.state in (DONE, SKIPPED):
        continue
      inside = m.in_zone(self.s_now, x, y)
      if inside and m.state == PENDING:
        m.state = ARMED
        m.armed_at = self.now()
        self.get_logger().info(f'▶ {m.name} ARM (배경, s={self.s_fmt()})')
      elif not inside and m.state == ARMED:
        m.state = DONE
        m.finished_at = self.now()
        self.get_logger().info(f'■ {m.name} 구간 이탈 — 끈다 (배경)')
        self.arm_pubs[m.name].publish(Bool(data=False))

    # 1) ARM 된 미션의 타임아웃 — 멈춰 있으면 탈락이므로 반드시 푼다.
    if self.active is not None:
      m = self.active
      if self.now() - m.armed_at > m.timeout:
        m.state = TIMEOUT
        m.finished_at = self.now()
        self.get_logger().warn(
            f'⏱ {m.name} 타임아웃 {m.timeout:.0f}s 초과 — 포기하고 자율 복귀한다. '
            '(미션 실패는 감점, 정지는 탈락)')
        self.disarm(m)
      return   # 하나 켜져 있는 동안은 다른 걸 켜지 않는다

    # 2) 새로 켤 미션 찾기
    for m in self.missions:
      if m.state != PENDING or not m.exclusive:
        continue
      if m.past_zone(self.s_now):
        m.state = SKIPPED
        self.get_logger().warn(f'⤳ {m.name} 구간을 지나쳤다 — 건너뜀')
        continue
      if not m.in_zone(self.s_now, x, y):
        continue
      # 시간예산 검사 — 감점-only 는 시간 없으면 버린다
      if (not m.critical
          and self.t_start is not None
          and self.remaining() - m.expected_s < self.reserve):
        m.state = SKIPPED
        self.get_logger().warn(
            f'⤳ {m.name} 건너뜀 — 남은 {self.remaining():.0f}s 로는 '
            f'예상 {m.expected_s:.0f}s + 예비 {self.reserve:.0f}s 를 못 맞춘다. '
            '완주가 먼저다.')
        continue
      self.arm(m)
      break

  def summary(self):
    parts = [f'{m.name}:{m.state}' for m in self.missions]
    return (f't={self.elapsed():.0f}/{self.budget:.0f}s '
            f's={self.s_now if self.s_now is not None else -1:.1f}m | '
            + ' '.join(parts))


def main(args=None):
  rclpy.init(args=args)
  node = MissionSequencer()
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

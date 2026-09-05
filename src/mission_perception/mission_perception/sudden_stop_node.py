#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sudden_stop_node.py — 돌발 급정지 미션 (더미 감지 → 완전정지 → 대기 → 재출발).

★ 규정 (HL FMA 2026 경기규정 항목 7 — 돌발상황에서 급정지, ★변경)
  · 가속구간에서 돌발상황 **1회** 발생 (고정 위치 아님)
  · 마킹된 기준선 통과 시 어린이 더미(인형)가 **좌 또는 우(랜덤)** 에서
    도로 중앙에 정지
  · 더미 중앙 정지 후 참가 차량은 **3초 이상 완전 정지** 요구
  · 기준선은 대회 당일 테이핑으로 표시
  · ※ **회피기동은 미션 성공으로 인정하지 않음**
  · ※ **3초 정지 후 더미를 치우거나 우회하여 주행 재개 가능**
  · 감점 10점 — 더미 앞 정지 후 3초 이상 완전 정지 못할 때 /
                회피기동(정지 없이 피해 주행)으로 미션 수행 시

  두 가지가 설계를 결정한다:
    ① "회피기동 불인정" → 이 구간에서 라이다 **조향 회피를 꺼야 한다.**
       계획 파일에서 `inhibits: [lidar_avoid]` 로 선언한다(시퀀서가 강제).
       피해서 지나가면 미션 실패다.
    ② "3초 후 우회 가능" → 아래 PASSING 단계가 규정상 허용된다.

  ⚠ 규정 최소는 **3초**다. 기본값 5초는 여유를 둔 값이고, 그만큼 8분 예산을
    더 쓴다. `dwell` 파라미터로 조절할 것.

★ 무엇이 이미 있고 무엇이 없었나
  라이다가 `/obstacle_distance` 를 내고, `longitudinal_controller` 가 그 값으로
  이미 감속·정지한다(0.8m 이하 → v=0). **서는 것까지는 공짜다.**
  없던 것은 딱 두 가지다:
    1) "완전히 섰다" 를 판정하고 **정해진 시간만큼 붙잡아 두는 것**
    2) **다시 출발시키는 것** — 이게 진짜 문제다(아래)

★ ★ 왜 '재출발' 이 어려운가
  더미가 아직 앞에 있으면 `/obstacle_distance` 가 계속 작게 나오고,
  longitudinal 은 계속 v=0 을 낸다. 즉 **가만히 두면 영원히 못 간다.**
  그런데 대회에서 **1분 이상 정지는 감점이 아니라 탈락**이다.

  그래서 단계적으로 빠져나온다:
    HOLD(5초) → WAIT_CLEAR(치워지길 기다림) → 그래도 막혀 있으면 PASSING
  PASSING 은 `/lidar/mute` 로 전방 감속을 잠깐 끄고 지나간다. 위험한 선택이라
  기본값을 바꿀 수 있게 두었고(`pass_when_blocked`), 로그를 크게 남긴다.
  **탈락(정지)보다 감점(접촉 위험 감수)이 낫다**는 대회 전략에 따른 것이다.

상태기계
  DISARMED   arm 대기 (require_arm 일 때만)
  IDLE       전방이 비어 있음                    → 999 (제약 없음)
  HOLD       장애물 근접 → 완전정지 명령          → 0.0
             실제로 멈추면(|v|<stopped_speed) dwell 타이머 시작
  WAIT_CLEAR dwell 경과. 더미가 치워지길 기다림    → 0.0
  PASSING    clear_timeout 초과 → 라이다 감속 mute 하고 통과 → 999
  CLEARED    통과 완료. 다시 걸리지 않는다         → 999 + done 발행

⚠ `/stop_line_distance` 를 crosswalk_stop_node 와 같이 쓴다.
  **동시에 arm 되면 서로 덮어쓴다** — mission_sequencer 가 하나만 켜는 것이 전제다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

NO_CONSTRAINT = 999.0


class SuddenStop(Node):

  def __init__(self):
    super().__init__('sudden_stop')

    self.declare_parameter('obstacle_topic', '/obstacle_distance')
    self.declare_parameter('odom_topic', '/odometry/filtered')
    self.declare_parameter('arm_topic', '/sudden_stop/arm')
    self.declare_parameter('require_arm', False)
    # 이 거리 안이면 '더미가 앞에 있다'. longitudinal 의 obstacle_stop_dist(0.8)
    # 보다 조금 크게 잡아, 하드정지가 걸린 상태를 확실히 인지한다.
    self.declare_parameter('trigger_dist', 1.2)
    # ★ 정지 유지 시간 [s] — 규정 최소 3.0. 기본 5.0 은 여유분이다.
    #   늘릴수록 안전하지만 8분 지정시간을 그만큼 쓴다.
    self.declare_parameter('dwell', 5.0)
    self.declare_parameter('stopped_speed', 0.05)
    # 이 거리보다 멀어지면 '치워졌다'
    self.declare_parameter('clear_dist', 3.0)
    # dwell 후 이만큼 더 기다려도 안 치워지면 통과를 시도한다
    self.declare_parameter('clear_timeout', 10.0)
    # 안 치워져도 통과할 것인가 (false 면 계속 기다린다 = 탈락 위험)
    self.declare_parameter('pass_when_blocked', True)
    # 통과하는 동안 라이다 전방 감속을 끄는 시간 [s]
    self.declare_parameter('pass_duration', 6.0)
    # 멈추라고 했는데 안 멈출 때의 탈출구
    self.declare_parameter('hold_timeout', 8.0)
    self.declare_parameter('rate', 10.0)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.trigger_dist = float(g('trigger_dist'))
    self.dwell = float(g('dwell'))
    self.stopped_v = float(g('stopped_speed'))
    self.clear_dist = float(g('clear_dist'))
    self.clear_timeout = float(g('clear_timeout'))
    self.pass_when_blocked = bool(g('pass_when_blocked'))
    self.pass_duration = float(g('pass_duration'))
    self.hold_timeout = float(g('hold_timeout'))
    self.require_arm = bool(g('require_arm'))

    self.armed = not self.require_arm
    self.obstacle = NO_CONSTRAINT
    self.speed = 0.0
    self.state = 'IDLE' if self.armed else 'DISARMED'
    self.hold_since = None      # HOLD 진입 시각
    self.dwell_since = None     # 실제로 멈춘 시각
    self.wait_since = None      # WAIT_CLEAR 진입 시각
    self.pass_since = None      # PASSING 진입 시각
    self.last_log = None

    self.pub = self.create_publisher(Float64, '/stop_line_distance', 10)
    self.mute_pub = self.create_publisher(Bool, '/lidar/mute', 10)
    self.state_pub = self.create_publisher(String, '/sudden_stop/state', 10)
    self.done_pub = self.create_publisher(Bool, '/sudden_stop/done', 10)

    self.create_subscription(Float64, str(g('obstacle_topic')),
                             self.obs_cb, 10)
    self.create_subscription(Odometry, str(g('odom_topic')), self.odom_cb, 10)
    self.create_subscription(Bool, str(g('arm_topic')), self.arm_cb, 10)
    self.create_timer(1.0 / float(g('rate')), self.tick)

    self.get_logger().info(
        f'돌발 급정지 — 근접 {self.trigger_dist:.1f}m 에서 정지, '
        f'**{self.dwell:.0f}초** 유지 후 재출발 '
        f'(치워짐 판정 {self.clear_dist:.1f}m, 대기상한 {self.clear_timeout:.0f}s)')
    if self.require_arm:
      self.get_logger().info('arm 대기 중 — 시퀀서가 켜야 동작한다')

  # ------------------------------------------------------------------ 입력
  def obs_cb(self, msg):
    self.obstacle = float(msg.data)

  def odom_cb(self, msg: Odometry):
    self.speed = float(msg.twist.twist.linear.x)

  def arm_cb(self, msg: Bool):
    want = bool(msg.data)
    if want == self.armed:
      return
    self.armed = want
    if want:
      self.reset('IDLE')
      self.get_logger().info('▶ ARM — 돌발 급정지 감시 시작')
    else:
      # ★ 브레이크와 mute 를 반드시 풀고 나간다.
      #   longitudinal 은 마지막 stop_line_distance 를 타임아웃 없이 들고 있다.
      self.emit(NO_CONSTRAINT)
      self.mute_pub.publish(Bool(data=False))
      self.reset('DISARMED')
      self.get_logger().info('■ DISARM — 제약 해제(999) + mute 해제')

  # ------------------------------------------------------------------ 보조
  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def reset(self, state):
    self.state = state
    self.hold_since = None
    self.dwell_since = None
    self.wait_since = None
    self.pass_since = None

  def emit(self, dist):
    self.pub.publish(Float64(data=float(dist)))

  def log_once(self, msg):
    if msg != self.last_log:
      self.get_logger().info(msg)
      self.last_log = msg

  # ------------------------------------------------------------------ 주기
  def tick(self):
    self.state_pub.publish(String(data=self.state))
    if not self.armed:
      return

    t = self.now()

    # ---- CLEARED: 끝. 다시 걸리지 않는다 ----
    if self.state == 'CLEARED':
      self.emit(NO_CONSTRAINT)
      self.done_pub.publish(Bool(data=True))
      return

    # ---- PASSING: 감속을 잠깐 끄고 지나간다 ----
    if self.state == 'PASSING':
      self.mute_pub.publish(Bool(data=True))
      self.emit(NO_CONSTRAINT)
      if t - self.pass_since >= self.pass_duration:
        self.mute_pub.publish(Bool(data=False))
        self.state = 'CLEARED'
        self.get_logger().info('통과 완료 — 라이다 감속 복구')
      return

    near = self.obstacle <= self.trigger_dist

    # ---- IDLE: 전방이 비어 있음 ----
    if self.state == 'IDLE':
      self.emit(NO_CONSTRAINT)
      if near:
        self.state = 'HOLD'
        self.hold_since = t
        self.dwell_since = None
        self.get_logger().info(
            f'⚠ 전방 장애물 {self.obstacle:.2f}m — 완전정지 후 '
            f'{self.dwell:.0f}초 대기한다')
      return

    # ---- HOLD: 완전정지 명령 + 정지 확인 후 dwell ----
    if self.state == 'HOLD':
      self.emit(0.0)
      if abs(self.speed) > self.stopped_v:
        self.dwell_since = None
        if t - self.hold_since > self.hold_timeout:
          self.get_logger().warn(
              f'{t - self.hold_since:.1f}s 동안 멈추지 않는다 '
              f'(v={self.speed:+.2f}m/s) — 그래도 진행시킨다. 브레이크 확인할 것.')
          self.state = 'WAIT_CLEAR'
          self.wait_since = t
        else:
          self.log_once(f'정지 중… v={self.speed:+.2f}m/s')
        return
      if self.dwell_since is None:
        self.dwell_since = t
        self.get_logger().info(
            f'완전정지 확인 — {self.dwell:.0f}초 유지 시작 '
            f'(장애물 {self.obstacle:.2f}m)')
      held = t - self.dwell_since
      if held < self.dwell:
        self.log_once(f'대기 {held:.1f}/{self.dwell:.0f}s')
        return
      self.state = 'WAIT_CLEAR'
      self.wait_since = t
      self.get_logger().info(f'{self.dwell:.0f}초 경과 — 재출발 조건 확인')
      return

    # ---- WAIT_CLEAR: 치워졌나? ----
    if self.state == 'WAIT_CLEAR':
      if self.obstacle >= self.clear_dist:
        self.state = 'CLEARED'
        self.get_logger().info(
            f'✅ 전방 확보({self.obstacle:.2f}m) — 재출발')
        return
      self.emit(0.0)       # 아직 막혀 있다 — 계속 정지
      waited = t - self.wait_since
      self.log_once(f'전방 아직 막힘({self.obstacle:.2f}m) — '
                    f'{waited:.1f}/{self.clear_timeout:.0f}s 대기')
      if waited < self.clear_timeout:
        return
      if not self.pass_when_blocked:
        self.get_logger().error(
            '⛔ 전방이 안 치워졌고 pass_when_blocked=false 다 — 계속 정지한다. '
            '**1분 넘으면 탈락이다.** E-stop 후 수동 개입할 것.')
        return
      self.state = 'PASSING'
      self.pass_since = t
      self.get_logger().warn(
          f'⚠ {self.clear_timeout:.0f}s 기다려도 안 치워졌다 — '
          f'라이다 전방 감속을 {self.pass_duration:.0f}초 끄고 통과한다. '
          '규정 항목 7: "3초 정지 후 더미를 치우거나 우회하여 주행 재개 가능". '
          '정지 유지는 1분 넘으면 탈락이다.')
      return


def main(args=None):
  rclpy.init(args=args)
  node = SuddenStop()
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

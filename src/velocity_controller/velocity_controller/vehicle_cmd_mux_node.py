#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_cmd_mux_node.py — 종방향(속도) + 횡방향(조향)을 합쳐 최종 /cmd_vel 생성.

자율주행 명령과 수동 teleop 명령을 한 곳에서 조정(arbitration)하고, 마지막
안전 클램프를 건 뒤 딱 하나의 /cmd_vel 만 내보낸다. 이렇게 단일 출구를 두면
여러 노드가 /cmd_vel 을 동시에 쏴서 서로 덮어쓰는 사고를 막을 수 있다.

우선순위:
  1) E-stop(/e_stop = true)      → 무조건 정지 (조향은 직진)
  2) teleop(/teleop/cmd_vel)     → 최근 수신 중이면 자율 명령을 덮어씀(사람 우선)
  3) 자율(/target_speed + /steering_cmd)
  4) 입력 끊김(워치독)           → 정지

/cmd_vel 규약 (스택 전체 공통):
  linear.x  = 목표 속도 [m/s]   (음수 = 후진)
  angular.z = 조향각   [도]     (좌 +, 우 −)   ※ rad/s 아님!

입력: /target_speed(Float64), /steering_cmd(Float64,도),
      /teleop/cmd_vel(Twist), /e_stop(Bool),
      /lidar/avoid_steer(Float64,도 · NaN=회피없음),
      /local_path(Path,차량기준 — 이탈 상한용), /lidar/arm(Bool — 유지시간 해제용)
출력: /cmd_vel(Twist)

※ /local_path 와 /lidar/arm 은 **없어도 된다.** 없으면 이탈 상한은 안 걸리고
  arm 은 켜진 것으로 본다 — 기존 구성이 그대로 동작한다.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64


class VehicleCmdMux(Node):

  def __init__(self):
    super().__init__('vehicle_cmd_mux')

    # 최종 안전 클램프 — 스택 어디서 뭐가 오든 여기서 마지막으로 잘린다.
    # 조향 18°: 물리한계 20°지만 포텐셔미터 포화(ADC 0) 회피 마진.
    self.declare_parameter('max_steer_deg', 18.0)
    self.declare_parameter('max_speed', 1.0)      # 구동 벤치검증 후 상향
    self.declare_parameter('min_speed', -0.6)     # 후진 한계
    self.declare_parameter('input_timeout', 0.5)  # 자율 입력 끊김 판정
    self.declare_parameter('teleop_timeout', 0.5)  # 이 시간 지나면 teleop 해제
    self.declare_parameter('rate', 20.0)
    # ★ 헤딩 캘리브 완료 전에는 자율(AUTO)을 막는다.
    # direct_localization 은 yaw_offset=0(=IMU 원시 yaw, 방향 의미 없음)으로도
    # odom 을 발행한다. 그 위에서 로컬경로·조향이 계산되므로, 캘리브 전에
    # control:=true 로 띄우면 **차가 엉뚱한 방향으로 스스로 출발한다.**
    # teleop 과 E-stop 은 막지 않는다 — 캘리브 10m 직진을 사람이 몰아야 하므로.
    self.declare_parameter('require_heading_calib', True)

    # ★ 2026-09-16 — 회피 override 유지시간 / 이탈 상한 (둘은 한 쌍이다)
    #
    # 왜 유지시간인가 (drive_0337 t78.0~78.3 실측):
    #   회피가 한 프레임이라도 AVOID 가 아니면 planner 는 NaN 을 내고, 먹스는
    #   **즉시** GPS 경로조향으로 돌아간다. 그 런에서 실제로 이렇게 찍혔다:
    #       t78.0  BLOCKED  회피각 nan  명령 +17.4   ← 경로조향(좌)
    #       t78.2  BLOCKED  회피각 nan  명령 +16.8
    #       t78.3  AVOID    회피각 -18.0 명령 -18.0  ← 0.3s 만에 35° 반전
    #   BLOCKED 은 '갭이 없다' 지 '장애물이 없다' 가 아니다. 그 0.15초 동안
    #   먹스는 **장애물 쪽으로** 조향을 냈다. 조향은 지연 0.35s 라 다행히
    #   따라가지 못했지만(실제각 −0.7~+1.3), 이건 운이지 설계가 아니다.
    #   유지시간은 이 빈틈을 마지막 회피각으로 메운다.
    #   ⚠ 라이다 자체가 죽으면(avoid_timeout) 유지하지 않는다 — 센서가 없는데
    #     옛 각을 물고 가는 것이 제일 위험하다.
    #   ⚠ DISARM 되면 즉시 푼다(/lidar/arm). 안 그러면 구간을 벗어나고도
    #     hold 만큼 회피각으로 달린다.
    #   기본 0.0 = 꺼짐. drive_0337 은 이 기능 없이 완주했다 — 검증된 동작을
    #   기본값으로 바꾸지 않는다. 실차에서 재보고 켤 것.
    self.declare_parameter('avoid_hold_s', 0.0)
    # 이탈 상한 — **유지시간과 반드시 같이** 쓴다. 유지시간만 늘리면 회피각을
    #   더 오래 물고 있으므로 이탈이 커진다. 경로에서 이만큼 벗어나면 회피를
    #   포기하고 경로조향으로 돌아온다.
    #   근거 (drive_0312 실측, 조향 최대 18°·최대 각속도 ~25°/s):
    #       이탈 0.67m 헤딩오차 19.2° → 회복 가능
    #       이탈 0.90m 헤딩오차 25.3° → ❌ 필요 25°/s vs 최대 25°/s (경계)
    #       이탈 1.87m 헤딩오차 46.8° → ❌ 필요 47°/s vs 최대 24°/s
    #   drive_0337 의 **성공한** 회피는 최대 0.72m / 17.6° 였다. 상한을 0.7m 에
    #   걸면 성공한 회피 한가운데서 끊긴다 — 그래서 0.9/25 의 경계 바로 아래인
    #   0.85m / 22° 를 권장값으로 둔다(성공 사례는 통과, 복귀불능은 차단).
    #   대가: 0.85m 이상 비켜야 하는 장애물은 못 피한다(접촉=감점).
    #   규정상 이탈·정지는 탈락, 접촉은 감점이므로 이 교환이 맞다.
    #   기본 0.0 = 꺼짐.
    self.declare_parameter('avoid_max_lateral_m', 0.0)
    self.declare_parameter('avoid_max_heading_deg', 0.0)
    # 상한에 걸린 뒤 이 비율 아래로 돌아와야 회피를 다시 허용한다(히스테리시스).
    # 없으면 상한 근처에서 회피↔경로가 매 프레임 번갈아 떨린다.
    self.declare_parameter('avoid_cap_release', 0.6)
    # 이탈은 /local_path(차량기준) 첫 점에서 읽는다. 별도 노드·odom 구독이
    # 필요 없다 — local_sliding_window_node 가 이미 10Hz 로 내고 있다.
    self.declare_parameter('local_path_topic', '/local_path')
    self.declare_parameter('local_path_timeout', 0.5)
    self.declare_parameter('avoid_arm_topic', '/lidar/arm')

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.max_steer = float(g('max_steer_deg'))
    self.max_speed = float(g('max_speed'))
    self.min_speed = float(g('min_speed'))
    self.input_timeout = float(g('input_timeout'))
    self.teleop_timeout = float(g('teleop_timeout'))
    rate = float(g('rate'))

    self.target_speed = 0.0
    self.steer_deg = 0.0
    self.speed_time = None
    self.steer_time = None
    # ★ 라이다 회피 조향 override (2026-08-19). AUTO 중 라이다가 유효한 회피각을
    # 주면(NaN 아님) GPS 경로 조향 대신 그 각으로 장애물을 피한다. 속도는 종방향이
    # /obstacle_distance 로 이미 안전하게 낮춘다. NaN 이면 평소대로 GPS 조향.
    self.avoid_steer = float('nan')
    self.avoid_time = None
    # 라이다 무신호 0.3s 면 회피각을 버리고 GPS 조향으로 복귀(옛 각을 물고 있지 않게).
    self.avoid_timeout = 0.3
    # ★ 유지시간·이탈상한 상태 (위 파라미터 주석 참고)
    self.avoid_hold_s = float(g('avoid_hold_s'))
    self.avoid_max_lat = float(g('avoid_max_lateral_m'))
    self.avoid_max_head = float(g('avoid_max_heading_deg'))
    self.avoid_cap_release = float(g('avoid_cap_release'))
    self.local_path_timeout = float(g('local_path_timeout'))
    self.last_avoid = float('nan')   # 마지막으로 받은 **유효한** 회피각
    self.last_avoid_t = None
    self.avoid_armed = True          # /lidar/arm 이 없으면 켜진 것으로 본다
    self.path_lat = 0.0              # 경로가 차량 기준 옆으로 벗어난 양 [m]
    self.path_head = 0.0             # 경로 방향의 차량기준 각 [도]
    self.path_time = None
    self.cap_tripped = False
    self._cap_logged = False
    self.teleop = None
    self.teleop_time = None
    self.estop = False
    self.last_mode = None
    self.require_calib = bool(g('require_heading_calib'))
    self.heading_ready = not self.require_calib

    self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.create_subscription(Float64, '/target_speed', self.speed_cb, 10)
    self.create_subscription(Float64, '/steering_cmd', self.steer_cb, 10)
    self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_cb, 10)
    self.create_subscription(Bool, '/e_stop', self.estop_cb, 10)
    self.create_subscription(Float64, '/lidar/avoid_steer', self.avoid_cb, 10)
    # 이탈 상한·유지시간 입력. 둘 다 없어도 동작한다(상한은 안 걸리고,
    # 유지시간은 arm 을 켜진 것으로 본다) — 기존 구성이 그대로 돈다.
    self.create_subscription(Path, str(g('local_path_topic')),
                             self.local_path_cb, 10)
    self.create_subscription(Bool, str(g('avoid_arm_topic')), self.arm_cb, 10)
    # heading_init 은 계산 후 종료하므로 latched(TRANSIENT_LOCAL)로 발행한다.
    # 늦게 뜬 먹스도 과거 값을 받아야 하므로 같은 QoS 로 구독한다.
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Float64, '/heading/yaw_offset',
                             self.heading_cb, latched)
    self.create_timer(1.0 / rate, self.tick)

    self.get_logger().info(
        f'명령 먹스 시작: 조향±{self.max_steer}° 속도 {self.min_speed}~'
        f'{self.max_speed}m/s (E-stop > teleop > 자율)')
    if self.require_calib:
      self.get_logger().warn(
          '헤딩 캘리브 대기 — /heading/yaw_offset 을 받기 전에는 자율(AUTO)을 '
          '거부한다. teleop 으로 10m 직진해 캘리브를 끝낼 것.')
    # ★ 켜졌다는 것을 눈에 보이게 찍는다 (cluster_plot_node 의 같은 주석 참고).
    #   꺼져 있으면 아무 말도 안 한다 — 기존 로그를 어지럽히지 않기 위해서다.
    if self.avoid_hold_s > 0.0:
      self.get_logger().warn(
          f'★ 회피 유지시간 {self.avoid_hold_s:.2f}s — BLOCKED/CLEAR 빈틈을 '
          '마지막 회피각으로 메운다. 라이다 무신호·DISARM 이면 즉시 푼다.')
    if self.avoid_max_lat > 0.0 or self.avoid_max_head > 0.0:
      self.get_logger().warn(
          f'★ 이탈 상한 — 이탈 {self.avoid_max_lat:.2f}m / '
          f'헤딩 {self.avoid_max_head:.0f}° 를 넘으면 회피를 포기하고 경로로 '
          f'돌아간다 (재개는 {self.avoid_cap_release:.0%} 아래로 복귀 시).')

  def heading_cb(self, msg):
    if not self.heading_ready:
      self.heading_ready = True
      self.get_logger().info(
          f'헤딩 캘리브 완료 ({math.degrees(float(msg.data)):.1f}°) — 자율 허용')

  def now(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def speed_cb(self, msg):
    self.target_speed = float(msg.data)
    self.speed_time = self.now()

  def steer_cb(self, msg):
    self.steer_deg = float(msg.data)
    self.steer_time = self.now()

  def teleop_cb(self, msg):
    self.teleop = msg
    self.teleop_time = self.now()

  def estop_cb(self, msg):
    if bool(msg.data) != self.estop:
      self.get_logger().warn(f'E-STOP {"작동" if msg.data else "해제"}')
    self.estop = bool(msg.data)

  def avoid_cb(self, msg):
    self.avoid_steer = float(msg.data)   # NaN = 회피 없음
    self.avoid_time = self.now()

  def local_path_cb(self, msg: Path):
    """/local_path 는 **차량 기준**(base_link) 이고 첫 점이 차량에 가장 가까운
    경로점이다(local_path_core.build_local_path: targets 가 s0 부터 시작).
    그래서 첫 점의 y 가 곧 '경로가 옆으로 얼마나 있나' = 이탈량이고,
    첫 두 점의 방향이 경로 헤딩의 차량기준 각이다. odom 도 전역경로도 필요 없다.
    """
    n = len(msg.poses)
    if n < 2:
      return
    p0 = msg.poses[0].pose.position
    p1 = msg.poses[1].pose.position
    self.path_lat = float(p0.y)
    self.path_head = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
    self.path_time = self.now()

  def arm_cb(self, msg):
    """회피 arm. 내려가면 유지시간·상한 상태를 **즉시** 버린다.

    안 그러면 시퀀서가 구간을 닫은 뒤에도 hold 만큼 옛 회피각으로 달린다.
    """
    want = bool(msg.data)
    if want == self.avoid_armed:
      return
    self.avoid_armed = want
    if not want:
      self.last_avoid_t = None
      self.cap_tripped = False
      self._cap_logged = False

  def fresh(self, t, timeout):
    return t is not None and (self.now() - t) <= timeout

  def avoid_angle(self):
    """이번 주기에 쓸 회피각. 없으면 None (= GPS 경로조향).

    유지시간이 메우는 것은 **BLOCKED/CLEAR 빈틈**이지 센서 부재가 아니다.
    라이다가 신선하지 않으면(avoid_timeout) 무조건 포기한다.
    """
    if not self.fresh(self.avoid_time, self.avoid_timeout):
      return None
    if not math.isnan(self.avoid_steer):
      self.last_avoid = self.avoid_steer
      self.last_avoid_t = self.now()
      return self.avoid_steer
    # NaN — planner 가 AVOID 가 아니다. 유지시간 안이면 마지막 각으로 메운다.
    if self.avoid_hold_s <= 0.0 or self.last_avoid_t is None:
      return None
    if not self.avoid_armed:
      return None
    if (self.now() - self.last_avoid_t) <= self.avoid_hold_s:
      return self.last_avoid
    return None

  def cap_reason(self):
    """이탈 상한에 걸렸으면 사유 문자열, 아니면 None.

    히스테리시스: 한 번 걸리면 상한의 avoid_cap_release 배 아래로 돌아와야 푼다.
    """
    if self.avoid_max_lat <= 0.0 and self.avoid_max_head <= 0.0:
      return None
    if not self.fresh(self.path_time, self.local_path_timeout):
      # 경로를 모르면 상한을 판단할 수 없다. **막지 않는다** — 여기서 회피를
      # 끄면 경로도 모르는 채 장애물로 직진하게 된다.
      return None
    lat, head = abs(self.path_lat), abs(self.path_head)
    lat_on = self.avoid_max_lat > 0.0
    head_on = self.avoid_max_head > 0.0
    if self.cap_tripped:
      r = self.avoid_cap_release
      lat_ok = (not lat_on) or lat < self.avoid_max_lat * r
      head_ok = (not head_on) or head < self.avoid_max_head * r
      if lat_ok and head_ok:
        self.cap_tripped = False
        self._cap_logged = False
        self.get_logger().info(
            f'회피 재개 — 이탈 {lat:.2f}m / 헤딩 {head:.1f}° 로 복귀')
        return None
      return f'이탈{lat:.2f}m·헤딩{head:.0f}°'
    over = []
    if lat_on and lat > self.avoid_max_lat:
      over.append(f'이탈 {lat:.2f}m>{self.avoid_max_lat:.2f}')
    if head_on and head > self.avoid_max_head:
      over.append(f'헤딩 {head:.1f}°>{self.avoid_max_head:.0f}')
    if not over:
      return None
    self.cap_tripped = True
    if not self._cap_logged:
      self._cap_logged = True
      self.get_logger().warn(
          '⚠ 이탈 상한 — 회피를 포기하고 경로로 돌아간다 (' +
          ', '.join(over) + ').\n'
          '   여기서 더 벌어지면 조향 최대치로도 못 돌아온다. '
          '접촉은 감점이지만 이탈은 탈락이다.')
    return '·'.join(over)

  def tick(self):
    v, s, mode = 0.0, 0.0, 'STOP'

    if self.estop:
      v, s, mode = 0.0, 0.0, 'E-STOP'
    elif self.fresh(self.teleop_time, self.teleop_timeout) and self.teleop:
      # 사람이 잡으면 사람이 우선 (teleop은 이미 도 단위로 발행)
      v, s, mode = self.teleop.linear.x, self.teleop.angular.z, 'TELEOP'
    elif not self.heading_ready:
      # 캘리브 전 자율 거부. 이유를 명시해야 현장에서 '왜 안 가지'로 헤매지 않는다.
      mode = 'STOP(헤딩 캘리브 전)'
    elif self.fresh(self.speed_time, self.input_timeout) and \
            self.fresh(self.steer_time, self.input_timeout):
      v, s, mode = self.target_speed, self.steer_deg, 'AUTO'
      # 라이다 회피: 유효한 회피각(NaN 아님)이 신선하면 GPS 조향을 덮어쓴다.
      # 속도(v)는 종방향이 /obstacle_distance 로 이미 낮췄으므로 그대로 둔다.
      # avoid_angle() 이 유지시간(빈틈 메우기)까지 판단한다.
      a = self.avoid_angle()
      if a is not None:
        reason = self.cap_reason()
        if reason is None:
          s, mode = a, 'AVOID(라이다)'
        else:
          # 상한 초과 — 회피를 버리고 경로조향(위에서 넣은 self.steer_deg)을 쓴다.
          # 모드 문자열에 숫자를 넣지 않는다 — 매 프레임 달라져 '모드 바뀜'
          # 로그가 도배된다. 구체적인 수치는 cap_reason() 이 한 번만 찍는다.
          mode = 'AUTO(회피중단·이탈상한)'
    else:
      # 자율 입력 중 하나라도 끊기면 정지(조향은 유지하지 않고 직진으로)
      mode = 'STOP(입력끊김)'

    # 최종 클램프
    v = max(self.min_speed, min(self.max_speed, float(v)))
    s = max(-self.max_steer, min(self.max_steer, float(s)))

    if mode != self.last_mode:
      self.get_logger().info(f'모드: {mode}')
      self.last_mode = mode

    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = s
    self.pub.publish(cmd)


def main(args=None):
  rclpy.init(args=args)
  node = VehicleCmdMux()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node.pub.publish(Twist())
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

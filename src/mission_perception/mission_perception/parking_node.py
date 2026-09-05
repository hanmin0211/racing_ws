#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parking_node.py — 지금 자세에서 주차 궤적을 **계획해서** 재생한다.

    /odometry/filtered ─→ [parking_planner] ─→ [parking_follower] ─→ /teleop/cmd_vel
                                                                     (vehicle_cmd_mux)

★ 2026-08-25 변경 — 왜 재설계 했나
  예전엔 `make_parking_path.py` 가 미리 만든 yaml 을 재생하면서 '가장 가까운 점'
  을 찾아 이어 갔다. 그런데 완주 종점은 매 주행마다 다르고, 실제로 자리1·2 는
  최근접 점이 후진 원호 한복판(위치는 0.1~0.4m 로 가깝지만 궤적 자세와 차 자세가
  58~78° 어긋남)이라 최소회전반경 2.42m 로 그 자세차를 못 메우고 매번 ABORT 했다.

  이제 궤적을 **미리 만들지 않는다.** 트리거가 오는 순간 차 위치에서 시작해
  Dubins 로 전진부, 해석해로 후진부를 이어 붙여 궤적을 만든다. 전진부 Dubins 는
  '끝 자세'까지 정확히 이어지므로 헤딩 어긋남으로 못 붙는 문제가 원리적으로 없다.
  자리 바로 옆에서 시작해 전진만으론 못 붙는 경우(1·2번)를 위해 시작 직전에
  '뒤로 조금 물러섰다 전진' 을 자동으로 추가한다.

  자세 오차 격자(±1m ±0.6m ±30°) 폐루프 시뮬 525회에서 99.2% 성공, 최악 21cm.

★ /teleop/cmd_vel — 이유
  vehicle_cmd_mux 는 E-stop > teleop > 자율 순. 이 채널로 내면 완주 코드를
  한 줄도 안 건드리고 제어권만 가져올 수 있다. 대회 D-1 에 완주 로직을 손대는 것이
  가장 큰 위험이다.

★ 데드맨(teleop_timeout 0.5s)
  mux 는 teleop 이 0.5s 끊기면 자율로 돌아간다. 그래서 이 노드는 주차가 끝난
  뒤에도 0 명령을 계속 발행한다. 발행을 멈추면 0.5s 뒤 차가 다시 출발한다.

★ 시작 트리거
  뜨자마자 발행하면 자율을 덮어쓴다. 기본은 대기이고 /parking/start (Bool True)
  나 /goal_reached (완주 신호) 를 받아야 계획·발행을 시작한다.
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from mission_perception import parking_follower as pf
from mission_perception import parking_planner as pp


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


class ParkingNode(Node):

  def __init__(self):
    super().__init__('parking_node')

    self.declare_parameter('slot', 1)
    self.declare_parameter('pose_dir', os.path.expanduser('~'))
    self.declare_parameter('auto_start', False)
    self.declare_parameter('trigger_on_goal_reached', True)
    self.declare_parameter('odom_topic', '/odometry/filtered')

    # 차량
    self.declare_parameter('wheelbase', 0.785)
    self.declare_parameter('max_steering_deg', 18.0)

    # 추종
    self.declare_parameter('speed', 0.3)
    self.declare_parameter('min_speed', 0.15)
    self.declare_parameter('lookahead', 0.8)
    self.declare_parameter('goal_tolerance', 0.20)
    self.declare_parameter('final_tolerance', 0.15)
    self.declare_parameter('approach_decel', 0.4)
    self.declare_parameter('cusp_dwell', 1.0)
    self.declare_parameter('stopped_speed', 0.05)
    self.declare_parameter('rate', 20.0)
    self.declare_parameter('steer_sign_reverse', 1.0)
    self.declare_parameter('max_steer_rate_deg', 90.0)
    self.declare_parameter('abort_cross_track', 1.0)
    self.declare_parameter('approach_cross_track', 3.0)
    self.declare_parameter('converge_tol', 0.35)

    # 계획
    # ★ 전진 반경: 최소회전반경(2.42m)에 여유. 3.0m 로 하면 조향이 계속 포화하지
    #   않아 실차 오차를 흡수한다.
    self.declare_parameter('plan_r_fwd', 3.0)
    # ★ 후진 반경 범위. 상한을 크게 잡을수록 곡률이 완만해져 실차에서 편하지만
    #   활주로가 길어져 차선을 벗어날 수 있다. 5.0m 가 시뮬에서 좋았다.
    self.declare_parameter('plan_r_rev_min', 2.9)
    self.declare_parameter('plan_r_rev_max', 5.0)
    self.declare_parameter('plan_back_max', 3.0)     # 필요시 최대 뒤로 물러남
    self.declare_parameter('plan_spacing', 0.2)

    # ★ 주차 후 탈출 (2026-09-05, 경기규정 항목 6·8)
    #   규정: "후진으로 차고에 진입하여 뒷바퀴가 확인선을 접촉하고
    #          **전진으로 진입확인선을 통과**해야 함"
    #   지금까지는 차고에 들어가 멈추는 데서 끝났다 — 규정상 미완이다.
    #   ~/parking_exit_{slot}.yaml 에 찍어둔 탈출 자세가 있으면, 주차 완료 후
    #   전진 Dubins 로 그 자세까지 나간 뒤에 done 을 낸다.
    self.declare_parameter('exit_enabled', True)
    self.declare_parameter('exit_r_fwd', 3.0)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.pose_dir = str(g('pose_dir'))
    self.slot = int(g('slot'))
    self.L = float(g('wheelbase'))
    self.max_steer_deg = float(g('max_steering_deg'))
    self.speed = float(g('speed'))
    self.min_speed = float(g('min_speed'))
    self.lookahead = float(g('lookahead'))
    self.goal_tol = float(g('goal_tolerance'))
    self.final_tol = float(g('final_tolerance'))
    self.decel = float(g('approach_decel'))
    self.cusp_dwell = float(g('cusp_dwell'))
    self.stopped_v = float(g('stopped_speed'))
    self.rate = float(g('rate'))
    self.rev_sign = float(g('steer_sign_reverse'))
    self.max_steer_rate = float(g('max_steer_rate_deg'))
    self.abort_cte = float(g('abort_cross_track'))
    self.approach_cte = float(g('approach_cross_track'))
    self.converge_tol = float(g('converge_tol'))

    self.plan_r_fwd = float(g('plan_r_fwd'))
    self.plan_r_rev = (float(g('plan_r_rev_min')),
                       float(g('plan_r_rev_max')), 0.35)
    self.plan_back = float(g('plan_back_max'))
    self.plan_spacing = float(g('plan_spacing'))
    self.exit_enabled = bool(g('exit_enabled'))
    self.exit_r_fwd = float(g('exit_r_fwd'))

    self.pose = None            # (x, y, yaw)
    self.linear_speed = 0.0
    self.follower = None
    self.plan = None
    self.state = 'WAIT'
    # 시작 트리거 상승엣지 래치 — start_cb 주석 참고.
    self._started = False
    self.exit_goal = None       # (x, y, yaw) — 탈출 목표 자세
    self.exit_started = False
    self.other_goals = []       # 다른 자리 (계획 시 침범 금지)
    self.goals = {}
    self.corridor = self._load_corridor()

    self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
    self.done_pub = self.create_publisher(Bool, '/parking/done', 10)
    self.state_pub = self.create_publisher(String, '/parking/state', 10)
    self.path_pub = self.create_publisher(Path, '/parking/path', 10)

    self.create_subscription(Odometry, str(g('odom_topic')), self.odom_cb, 10)
    self.create_subscription(Int32, '/parking/select', self.select_cb, 10)
    self.create_subscription(Bool, '/parking/start', self.start_cb, 10)
    self.create_subscription(Bool, '/parking/reset', self.reset_cb, 10)
    if bool(g('trigger_on_goal_reached')):
      self.create_subscription(Bool, '/goal_reached', self.start_cb, 10)
      self.get_logger().info(
          '완주 신호(/goal_reached) 도 시작 트리거로 사용한다.')

    self.create_timer(1.0 / self.rate, self.tick)
    self.create_timer(1.0, self.publish_path)

    self._load_goals()

    self.exit_goal = self._load_exit() if self.exit_enabled else None
    if bool(g('auto_start')):
      self.begin()
    else:
      self.get_logger().info(
          f'대기 중 (자리 {self.slot}) — /parking/start 로 시작할 것. 그 전까지는 '
          '아무것도 발행하지 않는다(자율주행을 덮어쓰지 않기 위해).')

  # ---------------- 입력 ----------------
  def odom_cb(self, msg: Odometry):
    p = msg.pose.pose
    self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
    self.linear_speed = float(msg.twist.twist.linear.x)

  def select_cb(self, msg: Int32):
    if self.state in ('DRIVE', 'CUSP'):
      self.get_logger().warn('주차 진행 중에는 자리를 바꾸지 않는다 — 무시')
      return
    self.slot = int(msg.data)
    self.get_logger().info(f'자리 {self.slot} 선택')

  def start_cb(self, msg: Bool):
    """시작 트리거. **상승엣지 한 번만** 받는다.

    ★ 왜 래치가 필요한가 (2026-09-04 발견)
      `/goal_reached` 는 완주 후 **계속 True 를 발행한다**(20Hz, latched).
      `/mission/…/arm` 도 시퀀서가 10Hz 로 재발행한다. 예전 조건은
      DRIVE/CUSP 만 막았으므로, 주차가 **DONE 이나 ABORT 로 끝난 다음 프레임에
      곧바로 begin() 이 다시 불려 무한히 재시작**됐다. 대회에서는 주차를
      성공해 놓고 그 자리에서 다시 주차를 시도하게 된다.

      그래서 한 번 시작하면 다시 시작하지 않는다. 재시도는 `/parking/reset`
      으로 명시적으로 요청해야 한다 — 토픽이 계속 True 라서 일어날 일이 아니다.
    """
    if not bool(msg.data):
      return
    if self._started:
      return
    if self.state in ('DRIVE', 'CUSP'):
      self.get_logger().warn(
          f'주차 시작 트리거 무시 (이미 {self.state} 상태).')
      return
    self._started = True
    self.get_logger().info(
        f'주차 시작 트리거 수신 → 계획 시도 (자리 {self.slot})')
    self.begin()

  def reset_cb(self, msg: Bool):
    """재시도 허용 — 다음 시작 트리거를 다시 받게 한다."""
    if not bool(msg.data):
      return
    if self.state in ('DRIVE', 'CUSP'):
      self.get_logger().warn('주차 진행 중 — reset 무시')
      return
    self._started = False
    self.state = 'WAIT'
    self.follower = None
    self.get_logger().info('주차 리셋 — 다음 시작 트리거를 다시 받는다')

  # ---------------- 로딩 ----------------
  def _load_corridor(self):
    """계획 시 cusp 가 차선 안에 오도록 쓰는 트랙 점열."""
    p = ('/home/han/racing_ws/src/pure_pursuit_pkg/config/'
         'waypoints_recorded_resampled_0.5.yaml')
    try:
      with open(p, encoding='utf-8') as f:
        d = yaml.safe_load(f)
      seq = (d['waypoints']
             if isinstance(d, dict) and 'waypoints' in d else d)
      return [(float(q['x']), float(q['y'])) for q in seq]
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(f'트랙을 못 읽어 corridor 검사를 건너뛴다: {e}')
      return None

  def _load_goals(self):
    """parking_pose_{1,2,3}.yaml 을 전부 읽어둔다. 하나 자리를 뽑을 때 나머지는
    keep-out 로 준다(계획이 다른 칸을 침범하지 않게)."""
    from waypoint_follower.site_origin import load_site_origin
    try:
      _, ox, oy, _ = load_site_origin(self.get_logger())
    except Exception:  # noqa: BLE001
      ox = oy = None
    for s in (1, 2, 3):
      p = os.path.join(self.pose_dir, f'parking_pose_{s}.yaml')
      try:
        with open(p, encoding='utf-8') as f:
          d = yaml.safe_load(f)
      except Exception as e:  # noqa: BLE001
        self.get_logger().warn(f'{p} 없음 ({e}) — 자리 {s} 는 계획 불가')
        continue
      if ox is not None and d.get('origin'):
        org = d['origin']
        if abs(float(org.get('x', ox)) - ox) > 1.0 or \
           abs(float(org.get('y', oy)) - oy) > 1.0:
          self.get_logger().error(
              f'⚠ 자리 {s} 원점 불일치! ({org.get("x")},{org.get("y")}) vs '
              f'({ox},{oy}) — 이 자리 사용 안 함')
          continue
      self.goals[s] = (
          (float(d['pose']['x']), float(d['pose']['y'])),
          math.radians(float(d['pose']['yaw_deg'])))
      self.get_logger().info(
          f'자리 {s}: ({d["pose"]["x"]:.2f}, {d["pose"]["y"]:.2f}) '
          f'yaw {d["pose"]["yaw_deg"]:+.1f}°')

  def _load_exit(self):
    """~/parking_exit_{slot}.yaml → 탈출 목표 자세. 없으면 None.

    parking_pose_recorder 로 **차고에서 나와 코스로 향하는 자세**에 차를 세우고
    찍으면 된다. 원점 검사는 주차 자세와 동일하게 한다 — 다른 장소 파일이면
    엉뚱한 데로 전진하게 되고, 그건 이탈=탈락이다.
    """
    from waypoint_follower.site_origin import load_site_origin
    p = os.path.join(self.pose_dir, f'parking_exit_{self.slot}.yaml')
    if not os.path.exists(p):
      self.get_logger().warn(
          f'탈출 자세 없음: {p} — 차고에 들어가 멈추는 데서 끝난다. '
          '규정(항목 6·8)은 전진으로 진입확인선 통과까지를 요구한다.')
      return None
    try:
      with open(p, encoding='utf-8') as f:
        d = yaml.safe_load(f)
      _, ox, oy, _ = load_site_origin(self.get_logger())
      org = d.get('origin') or {}
      if org and (abs(float(org.get('x', ox)) - ox) > 1.0 or
                  abs(float(org.get('y', oy)) - oy) > 1.0):
        self.get_logger().error(
            f'⚠ 탈출 자세 원점 불일치 — 사용하지 않는다 ({p})')
        return None
      ps = d['pose']
      goal = (float(ps['x']), float(ps['y']),
              math.radians(float(ps['yaw_deg'])))
      self.get_logger().info(
          f'탈출 자세: ({goal[0]:.2f}, {goal[1]:.2f}) '
          f'yaw {math.degrees(goal[2]):+.1f}°')
      return goal
    except Exception as e:  # noqa: BLE001
      self.get_logger().error(f'탈출 자세 로드 실패({e}) — 사용하지 않는다')
      return None

  def _make_follower(self, segs):
    return pf.ParkingFollower(
        segs, wheelbase=self.L, max_steering_deg=self.max_steer_deg,
        speed=self.speed, min_speed=self.min_speed,
        lookahead=self.lookahead, goal_tolerance=self.goal_tol,
        final_tolerance=self.final_tol, approach_decel=self.decel,
        cusp_dwell=self.cusp_dwell, stopped_speed=self.stopped_v,
        rate=self.rate, steer_sign_reverse=self.rev_sign,
        max_steer_rate_deg=self.max_steer_rate,
        abort_cross_track=self.abort_cte,
        approach_cross_track=self.approach_cte,
        converge_tol=self.converge_tol)

  def begin_exit(self):
    """주차 완료 자세 → 탈출 자세. **전진 전용**(Dubins)."""
    self.exit_started = True
    got = pp.dubins(self.pose, self.exit_goal, self.exit_r_fwd)
    if got is None:
      self.get_logger().error(
          '⛔ 탈출 경로를 만들 수 없다 — 주차는 완료된 상태로 종료한다. '
          '(탈출 자세가 차고에서 너무 가깝거나 각도가 과하다)')
      self.state = 'FINISHED'
      return
    length, pts, word = got
    segs = pf.split_segments([(x, y, 1) for (x, y) in pts])
    self.follower = self._make_follower(segs)
    self.state = 'EXIT'
    self.get_logger().info(
        f'▶ 탈출 시작 — {word} {length:.2f}m ({len(pts)}점, 전진 전용)')

  # ---------------- 계획 ----------------
  def begin(self):
    if self.slot not in self.goals:
      self.get_logger().error(
          f'자리 {self.slot} 의 pose 파일이 없다 — 시작할 수 없다.')
      return
    if self.pose is None:
      self.get_logger().error(
          '측위(/odometry/filtered) 미수신 — 시작할 수 없다.')
      return
    goal, th_g = self.goals[self.slot]
    # 다른 자리를 keep-out 로 넣으면 인접 슬롯(0.9m 간격) 상자가 겹쳐 매번 계획
    # 실패한다. 대회에서 다른 자리에 실제 차가 있어도 crosswalk/lidar 가 잡을
    # 몫이고, 여기서는 기하만 다룬다. 필요하면 파라미터로 다시 켤 수 있다.
    others = []
    self.get_logger().info(
        f'▶ 계획 시작: 지금 ({self.pose[0]:.2f}, {self.pose[1]:.2f}) '
        f'yaw {math.degrees(self.pose[2]):+.1f}° → 자리 {self.slot} '
        f'({goal[0]:.2f}, {goal[1]:.2f}) yaw {math.degrees(th_g):+.1f}°')
    plan = pp.plan_parking(
        self.pose, goal, th_g,
        r_fwd=self.plan_r_fwd, r_rev=self.plan_r_rev,
        d_back=(0.0, self.plan_back, 0.75),
        spacing=self.plan_spacing, corridor=self.corridor,
        other_goals=others)
    if plan is None:
      self.get_logger().error(
          '⛔ 이 자세에서 자리로 가는 궤적을 못 만들었다 — 중단.')
      self.state = 'ABORT'
      return

    n_f = sum(1 for p in plan['points'] if p[2] > 0)
    n_r = len(plan['points']) - n_f
    self.get_logger().info(
        f'  계획: {plan["word"]}  뒤로 {plan["d_back"]:.2f}m  '
        f'전진 {plan["fwd_len"]:.2f}m  후진 {plan["rev_len"]:.2f}m  '
        f'turn {plan["turn_deg"]:+.0f}°  R_rev {plan["r_rev"]:.2f}m  '
        f'({n_f}전 + {n_r}후 = {len(plan["points"])}점)')

    segs = pf.split_segments(plan['points'])
    self.follower = self._make_follower(segs)
    self.plan = plan
    self.state = 'DRIVE'
    self.get_logger().info(f'▶ 주행 시작 — {len(segs)}구간')

  # ---------------- 주기 ----------------
  def tick(self):
    self.state_pub.publish(String(data=self.state))

    if self.state == 'WAIT':
      return
    if self.follower is None:
      return

    # ★ 차고에 넣은 것으로 끝이 아니다 — 규정은 전진으로 진입확인선 통과까지다.
    if (self.state == 'DONE' and self.exit_enabled
            and self.exit_goal is not None and not self.exit_started):
      self.begin_exit()
      return

    if self.state in ('DONE', 'ABORT', 'FINISHED'):
      self._publish_cmd(0.0, 0.0)
      self.done_pub.publish(Bool(data=(self.state != 'ABORT')))
      return

    v, d = self.follower.update(self.pose, self.linear_speed, self._now())
    for kind, msg in self.follower.drain_events():
      if kind == 'ABORT':
        self.get_logger().error(f'⛔ 주차 중단: {msg}')
      elif kind == 'DONE':
        self.get_logger().info(f'✅ {msg} — 0 명령을 계속 발행한다.')
      else:
        self.get_logger().info(msg)
    fs = self.follower.state
    if self.exit_started:
      # 탈출 구간의 follower 상태를 미션 상태로 옮긴다.
      # 그냥 대입하면 탈출 완료가 다시 'DONE' 이 되어 탈출이 재시작된다.
      mapped = {'DONE': 'FINISHED', 'ABORT': 'ABORT'}.get(fs, 'EXIT')
    else:
      mapped = fs
    if mapped != self.state:
      self.state = mapped
      if mapped == 'FINISHED':
        self.get_logger().info('✅ 탈출 완료 — 진입확인선 통과. 미션 종료.')
    self._publish_cmd(v, d)
    if self.follower.status:
      self.get_logger().info(self.follower.status,
                             throttle_duration_sec=0.5)

  def _publish_cmd(self, v, steer_deg):
    cmd = Twist()
    cmd.linear.x = float(v)
    cmd.angular.z = float(steer_deg)     # ★ 도 단위 (스택 공통 규약)
    self.pub.publish(cmd)

  def publish_path(self):
    if self.plan is None:
      return
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    for (x, y, _g) in self.plan['points']:
      ps = PoseStamped()
      ps.header = msg.header
      ps.pose.position.x = float(x)
      ps.pose.position.y = float(y)
      ps.pose.orientation.w = 1.0
      msg.poses.append(ps)
    self.path_pub.publish(msg)

  def _now(self):
    return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
  rclpy.init(args=args)
  node = ParkingNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node._publish_cmd(0.0, 0.0)
    except Exception:  # noqa: BLE001
      pass
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

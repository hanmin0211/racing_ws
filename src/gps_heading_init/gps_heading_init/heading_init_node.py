#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
heading_init_node.py
====================
10m 직진 GPS-course 헤딩(yaw) 초기화 — 1회성(one-shot) 노드.

A9 IMU의 yaw는 자력계 오차로 맵 좌표계에 정렬돼 있지 않다. 차량이 직진할 때
GPS 이동방향(course)을 '진짜 헤딩'으로 삼아, 그 순간 IMU yaw와의 차이
(yaw_offset)를 계산한다.

  1. 첫 유효 GPS를 시작점으로 기록
  2. 직진 → 시작점에서 calib_distance(기본 10m) 도달 시:
        course     = atan2(Δnorth, Δeast)
        yaw_offset = normalize(course - imu_yaw)
  3. /heading/yaw_offset 을 래치(TRANSIENT_LOCAL)로 발행한 뒤 노드 자동 종료.

오프셋을 실제로 IMU에 적용하는 건 direct_localization_node가 한다. 그래서 이
노드는 값만 계산하고 빠져도 로컬라이제이션이 계속 돌아간다. 재캘리브가
필요하면 이 노드를 다시 실행하면 된다(이미 떠 있는 구독자가 새 값을 받음).

자동 직진(auto_drive) — 기본 꺼짐:
  IMU yaw는 재시작마다 리셋되므로 매 세션 10m 직진이 필요한데, 사람이 밀거나
  teleop으로 몰면 잘 휜다(현장 로그에서 편차 43°·157°로 2연속 거부됨).
  조향 0°를 유지하는 건 기계가 더 잘하므로, 차량이 스스로 직진하게 할 수 있다.

  명령은 /teleop/cmd_vel 로 낸다. vehicle_cmd_mux 우선순위가
  E-stop > teleop > 자율 이므로 (a) pure_pursuit의 '정지'를 덮어쓰고
  (b) mux의 teleop_timeout(0.5s)이 데드맨으로 동작해 이 노드가 죽으면
  0.5초 안에 자동 정지하며 (c) E-stop은 그대로 최상위로 남는다.
  → mux를 고치지 않고 얻는 성질들이다.

  ★ 직진 유지는 폐루프다. 헤딩의 **절대값**은 아직 모르지만(그걸 구하는 게
    이 노드의 목적) 출발 시점 대비 **변화량**은 IMU 로 정확히 알 수 있으므로,
    '출발할 때의 yaw 를 유지'하면 절대 헤딩을 몰라도 곧게 간다.
    조향 0° 만 주는 개루프면 STEER_CENTER 가 조금만 어긋나도 계속 휘어
    직진성 검증에서 거부된다(2026-08-18 현장: 우측 쏠림).

  ⚠ 그래도 **차가 향한 방향으로 간다**. 목적지를 아는 주행이 아니다.
    앞이 비어 있는지 확인하고, E-stop을 손에 쥔 채로 쓸 것.
  ⚠ wasd_teleop 등 다른 teleop과 동시에 쓰지 말 것 (같은 토픽을 두고 싸운다).

파라미터:
  fix_topic, imu_topic : 입력 (기본 /fix, handsfree/imu)
  calib_distance       : 직진 거리[m] (기본 10.0)
  restart_settle_sec   : 직진성 검증 실패 후 재시작 전 정지 대기[s] (기본 3.0)
  auto_drive           : 자동 직진 사용 (기본 False — 반드시 명시적으로 켤 것)
  auto_speed           : 자동 직진 속도[m/s] (기본 0.3)
  auto_countdown       : 출발 전 카운트다운[s] (기본 5.0)
  auto_timeout         : 출발 후 10m 미달 시 포기[s] (기본 90.0)
  auto_heading_gain    : 직진 유지 게인 [도/도] (기본 1.5)
  auto_max_correction  : 보정 조향각 상한[도] (기본 5.0)
  auto_steer_bias      : 조향 중앙 어긋남 임시 상쇄용 트림[도] (기본 0.0).
                         좌로 더 가게 하려면 +. 근본 해결은 STEER_CENTER 수정.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_sensor_data)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Int32

M_PER_DEG = 111320.0


def yaw_from_quat(q):
  siny = 2.0 * (q.w * q.z + q.x * q.y)
  cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny, cosy)


def normalize_angle(a):
  return math.atan2(math.sin(a), math.cos(a))


class HeadingInitNode(Node):

  def __init__(self):
    super().__init__('gps_heading_init')

    self.declare_parameter('fix_topic', '/fix')
    self.declare_parameter('imu_topic', 'handsfree/imu')
    self.declare_parameter('calib_distance', 10.0)
    # direct_localization의 invert_imu_yaw와 반드시 같은 값이어야 yaw_offset이 일관됨.
    self.declare_parameter('invert_imu_yaw', False)
    # 캘리브 중 허용할 최대 진행방향 편차[도]. 이보다 휘면 직진이 아니라고 보고 거부.
    self.declare_parameter('max_deviation_deg', 20.0)
    # 직진성 평가를 시작할 최소 현 길이[m]. 이보다 짧으면 현 방향 자체가
    # 노이즈라서 비교가 무의미하다(위 fix_callback 주석 참고).
    self.declare_parameter('min_chord_for_dev', 2.0)
    # 직진성 평가 시 최근 진행방향을 재는 구간 길이[m]. 길수록 GPS 튐값에 강하다.
    self.declare_parameter('seg_window_m', 1.5)
    # 실패 후 재시작: 차량이 이 시간만큼 멈춰 있어야 새 시작점을 잡는다.
    self.declare_parameter('restart_settle_sec', 3.0)
    self.declare_parameter('restart_settle_radius', 0.5)   # 이보다 움직이면 '이동 중'
    self.declare_parameter('restart_settle_timeout', 20.0)  # 정지 감지 실패 시 탈출
    # 자동 직진 — 기본 꺼짐. 켜면 차가 스스로 움직이므로 반드시 명시적 opt-in.
    self.declare_parameter('auto_drive', False)
    self.declare_parameter('auto_speed', 0.3)
    self.declare_parameter('auto_countdown', 5.0)
    self.declare_parameter('auto_timeout', 90.0)
    # 자동 직진 중 헤딩 유지 (절대 헤딩을 몰라도 '출발 시점 대비 변화'로 곧게 간다)
    self.declare_parameter('auto_heading_gain', 1.5)   # 도/도
    self.declare_parameter('auto_max_correction', 5.0)  # 보정 조향각 상한[도]
    # 조향 중앙이 틀어진 걸 임시 상쇄하는 수동 트림[도]. 좌로 더 가게 하려면 +.
    # 근본 해결은 STEER_CENTER 수정이고, 이건 현장 임시 대응용이다.
    self.declare_parameter('auto_steer_bias', 0.0)
    # 조향 중앙 역산용 (펌웨어와 같은 값이어야 카운트 환산이 맞다)
    # ★ 2026-08-24 정정: 옛 amap 보드 값(424 / 21.2)이 남아 있었다.
    #   이 보드 실측은 center 412 / 좌rail 936 / 우rail 0 이고, counts/도 는
    #   좌 26.2 · 우 20.6 으로 **비대칭**이다(henes_firmware.ino 참고).
    #   틀린 상수로 STEER_CENTER 를 권고하면 조향 중립이 통째로 어긋나므로
    #   펌웨어와 같은 값을 쓴다. (사고 런에서 '424 → 518 권장' 이 나온 원인)
    self.declare_parameter('wheelbase', 0.785)
    self.declare_parameter('steer_center', 412)
    self.declare_parameter('steer_cpd_left', 26.2)    # +각(좌) counts/도
    self.declare_parameter('steer_cpd_right', 20.6)   # -각(우) counts/도
    self.declare_parameter('steer_counts_per_deg', 23.4)   # 레거시 평균

    # ★ 게이트④ — 앞바퀴가 펴져 있지 않으면 캘리브를 시작하지 않는다.
    #   정지 상태에서 바퀴가 꺾여 있으면 (a) 펌웨어가 중앙으로 되돌리려다
    #   타이어 접지마찰에 막혀 STEER 스톨이 계속 나고 (b) 출발 초반이 곡선이라
    #   '10m 직진' 자체가 직진이 아니게 된다.
    #   2026-08-24 벤치에서 조향 ADC 567~643 (중립 412) = 좌로 5.9~8.8° 꺾인 채로
    #   캘리브를 돌리고 있었다. 스톨 임계(15카운트)의 10배가 넘는 오차였다.
    #   /steering_adc 가 안 올라오면(control:=false 등) 이 게이트는 건너뛴다.
    self.declare_parameter('require_wheels_straight', True)
    self.declare_parameter('max_steer_offset_deg', 2.0)

    # ★ 2026-08-24 충돌 사고 대응 — 측위 점프를 '직진'으로 오인한 사고.
    #
    #   로그(21:53): 자동직진 출발 명령은 08.245 에 나가는데, 07.669 에 이미
    #   '8.4/10m' 를 갔다고 보고됐다. 즉 **차가 서 있는 동안** 좌표가 0.38초에
    #   7.3m 튀었다. 그 시각은 NTRIP 첫 RTCM 수신(07.829) 직전 — 단독측위에서
    #   RTK Fixed 로 해가 스냅하며 생긴 점프다.
    #   그 점프 벡터를 진행방향으로 믿고 yaw_offset=-155.8° 를 확정했고,
    #   헤딩이 통째로 틀어진 채 AUTO 로 넘어가 우측으로 감겨 벽에 충돌했다.
    #
    #   기존 직진성 검사는 이걸 못 잡는다: 점프 후 '최근 구간'과 '전체 현'이
    #   둘 다 같은 점프 벡터라 편차가 안 생기고, 지속편차 1.5m 도 못 채운다.
    #   그래서 성질이 다른 게이트 셋을 각각 독립으로 건다.
    #
    #   ① RTK 수렴 전에는 시작조차 안 한다 (점프의 원인 자체를 제거)
    #   ② 물리적으로 불가능한 이동속도는 측위 점프로 보고 무효화
    #   ③ 물리적 최소 소요시간을 못 채운 '완료'는 거부
    self.declare_parameter('require_rtk', True)
    self.declare_parameter('max_h_std', 0.05)       # 수평 σ 상한[m]
    self.declare_parameter('max_jump_speed', 2.0)   # 이 속도 초과 이동 = 점프[m/s]
    # 완료 판정 시 최대 편차 하드 상한[도]. 지속편차 검사를 빠져나온 큰 peak 도
    # 여기서 막는다 (사고 때 36° 가 그대로 통과했다).
    self.declare_parameter('max_peak_dev_deg', 25.0)

    fix_topic = self.get_parameter('fix_topic').value
    imu_topic = self.get_parameter('imu_topic').value
    self.calib_distance = float(self.get_parameter('calib_distance').value)
    self.invert_imu_yaw = bool(self.get_parameter('invert_imu_yaw').value)
    self.max_deviation = math.radians(
        float(self.get_parameter('max_deviation_deg').value))
    # ★ 2026-08-19 대회 강건화 (발 부딪힘 등 일시적 방해 대응)
    # ① 편차가 이 거리 이상 '지속'돼야 거부한다. 순간 튐(발에 툭/조향 걸림)은
    #    곧 회복되면 시작→끝 직선 헤딩이 여전히 유효하므로 통과시킨다. 진짜
    #    곡선 주행만 지속 편차로 걸린다. (peak 만 보던 옛 방식은 0.3m 튐도 거부)
    self.declare_parameter('dev_sustain_m', 1.5)
    self.dev_sustain_m = float(self.get_parameter('dev_sustain_m').value)
    # ② 자동직진 실패 시 자동 재시도 횟수. 한 번 삐끗해도 되돌아와 다시 시도한다.
    #    이 횟수를 다 쓰면(연속 실패=기계 문제 의심) 그때 포기한다.
    self.declare_parameter('max_calib_retries', 3)
    self.max_calib_retries = int(self.get_parameter('max_calib_retries').value)
    self.calib_fail_count = 0
    # ★ 전방 벽/커브 보호: 재시도(자동직진)는 원래 시작점 이 반경 안으로 차를
    # 되돌려야만 다시 출발한다. 실패 지점(10m 앞)에서 또 전진하면 커브·벽으로
    # 돌진하므로, 사람이 차를 시작점으로 당겨올 때까지 출발을 보류한다.
    self.declare_parameter('retry_start_radius', 2.0)
    self.retry_start_radius = float(self.get_parameter('retry_start_radius').value)
    self.origin_lat = None       # 최초 시작점(재시도 복귀 기준)
    self.origin_lon = None
    self._last_reposition_log = 0.0
    self._dev_run_start = None    # 편차 구간 시작 dist
    self.max_dev_run = 0.0        # 가장 길게 지속된 편차 구간[m]
    self.min_chord_for_dev = float(
        self.get_parameter('min_chord_for_dev').value)
    self.seg_window = float(self.get_parameter('seg_window_m').value)
    self.settle_sec = float(self.get_parameter('restart_settle_sec').value)
    self.settle_radius = float(self.get_parameter('restart_settle_radius').value)
    self.settle_timeout = float(self.get_parameter('restart_settle_timeout').value)

    # 실패 후 재무장 대기 상태 (최초 1회차에는 적용하지 않는다 — 런치 직후엔
    # 차가 서 있는 게 정상이고, 괜히 3초를 더 기다리게 만들 이유가 없다)
    self.rearming = False
    self._settle_lat = None
    self._settle_lon = None
    self._settle_t = 0.0
    self._rearm_t0 = 0.0
    self._last_settle_log = 0.0

    self.auto_drive = bool(self.get_parameter('auto_drive').value)
    self.auto_speed = float(self.get_parameter('auto_speed').value)
    self.auto_countdown = float(self.get_parameter('auto_countdown').value)
    self.auto_timeout = float(self.get_parameter('auto_timeout').value)
    self.auto_heading_gain = float(self.get_parameter('auto_heading_gain').value)
    self.auto_max_corr = float(self.get_parameter('auto_max_correction').value)
    self.auto_steer_bias = float(self.get_parameter('auto_steer_bias').value)
    self.drive_yaw0 = None        # 자동 직진 시작 시점의 IMU yaw (기준)
    self.wheelbase = float(self.get_parameter('wheelbase').value)
    self.counts_per_deg = float(self.get_parameter('steer_counts_per_deg').value)
    self.steer_center = int(self.get_parameter('steer_center').value)
    self.cpd_left = float(self.get_parameter('steer_cpd_left').value)
    self.cpd_right = float(self.get_parameter('steer_cpd_right').value)
    self.require_straight = bool(
        self.get_parameter('require_wheels_straight').value)
    self.max_steer_off = float(
        self.get_parameter('max_steer_offset_deg').value)
    self.steer_adc = None        # 최근 조향 ADC
    self.steer_adc_t = 0.0
    self._last_straight_log = 0.0
    self.require_rtk = bool(self.get_parameter('require_rtk').value)
    self.max_h_std = float(self.get_parameter('max_h_std').value)
    self.max_jump_speed = float(self.get_parameter('max_jump_speed').value)
    self.max_peak_dev = math.radians(
        float(self.get_parameter('max_peak_dev_deg').value))
    self._last_fix = None        # (t, east, north) — 점프 검사용
    self._rtk_ok = False
    self._last_rtk_log = 0.0
    self.calib_t0 = None         # 시작점을 잡은 시각
    self.drive_started_t = None  # 실제로 굴러가기 시작한 시각(카운트다운 후)
    self.track = []               # 캘리브 구간 궤적 (조향 중앙 역산용)
    self.drive_t0 = None          # 카운트다운 시작 시각
    self.drive_aborted = False    # 실패/타임아웃 후에는 자동 재주행하지 않는다
    self.brake_until = None       # 이 시각까지 0을 쏴서 세운다
    self._last_cd_log = -1

    self.lat0 = None
    self.lon0 = None
    self.cos_lat0 = 1.0
    self.imu_yaw = None
    self.done_time = None
    self.prev_e = None
    self.prev_n = None
    self.max_dev = 0.0
    self.max_dev_at = 0.0

    # 오프셋은 래치(TRANSIENT_LOCAL)로 발행 — 이 노드가 종료해도 이미 구독 중인
    # direct_localization이 값을 받도록. (같은 이유로 course도 래치)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.off_pub = self.create_publisher(Float64, '/heading/yaw_offset', latched)
    self.course_pub = self.create_publisher(Float64, '/heading/gps_course',
                                            latched)

    # 자동 직진용. auto_drive가 꺼져 있으면 이 토픽에 아무것도 쓰지 않는다
    # (쓰면 mux가 teleop 모드로 붙잡혀 자율 주행이 막힌다).
    self.drive_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)

    self.create_subscription(NavSatFix, fix_topic, self.fix_cb,
                             qos_profile_sensor_data)
    self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
    self.create_subscription(Int32, '/steering_adc', self.steer_adc_cb, 10)
    self.create_timer(0.3, self.shutdown_check)
    if self.auto_drive:
      self.create_timer(0.1, self.drive_tick)

    self.get_logger().info(
        f'헤딩 초기화(1회성) 시작: {self.calib_distance:.0f}m 직진하면 '
        f'yaw_offset 계산 후 자동 종료 (fix={fix_topic}, imu={imu_topic})')
    if self.auto_drive:
      self.get_logger().warn(
          f'⚠ 자동 직진 켜짐: GPS 수신 후 {self.auto_countdown:.0f}초 뒤 '
          f'{self.auto_speed:.2f}m/s로 스스로 {self.calib_distance:.0f}m 전진한다. '
          f'앞을 비우고 E-stop을 손에 쥘 것.')

  def imu_cb(self, msg: Imu):
    y = yaw_from_quat(msg.orientation)
    self.imu_yaw = -y if self.invert_imu_yaw else y

  def steer_adc_cb(self, msg: Int32):
    self.steer_adc = int(msg.data)
    self.steer_adc_t = self.get_clock().now().nanoseconds * 1e-9

  def _steer_angle_now(self):
    """현재 조향각[도]. 펌웨어 steerADCToAngle 과 같은 식(방향별 기울기)."""
    if self.steer_adc is None:
      return None
    d = self.steer_adc - self.steer_center
    return d / self.cpd_left if d >= 0 else d / self.cpd_right

  def _wheels_straight(self) -> bool:
    """앞바퀴가 펴져 있는가. 텔레메트리가 없으면 판정하지 않고 통과시킨다.

    ★ 왜 통과시키나
      control:=false 로 측위만 볼 때나 아두이노가 안 붙었을 때는
      /steering_adc 가 아예 안 온다. 그 경우까지 막으면 캘리브가 영영
      시작되지 않는다. 게이트는 '알 수 있을 때만' 건다.
    """
    if not self.require_straight:
      return True
    t = self.get_clock().now().nanoseconds * 1e-9
    if self.steer_adc is None or t - self.steer_adc_t > 2.0:
      return True                       # 조향 텔레메트리 없음 — 판정 불가
    ang = self._steer_angle_now()
    if abs(ang) <= self.max_steer_off:
      return True
    if t - self._last_straight_log > 2.0:
      self._last_straight_log = t
      side = '좌' if ang > 0 else '우'
      self.get_logger().warn(
          f'앞바퀴가 {side}로 {abs(ang):.1f}° 꺾여 있다 '
          f'(ADC {self.steer_adc}, 중립 {self.steer_center}, '
          f'허용 ±{self.max_steer_off:.1f}°) — 출발하지 않는다. '
          f'손으로 앞바퀴를 정면에 맞출 것. '
          f'⚠ 이대로 가면 초반이 곡선이라 10m 직진이 직진이 아니다. '
          f'무시하려면 require_wheels_straight:=false.')
    return False

  def _settled(self, msg: NavSatFix) -> bool:
    """실패 후 재시작: 차량이 실제로 멈출 때까지 시작점 기록을 미룬다.

    실패 직후 곧바로 시작점을 잡으면, 운전자가 차를 되돌리는 그 동작이 다음
    시도의 앞구간으로 기록돼 또 '직진 아님'으로 거부된다. 현장에서 실제로
    이것 때문에 2회 연속 실패했다(1차 실패 110ms 뒤에 2차 시작점이 잡힘).
    """
    t = self.get_clock().now().nanoseconds * 1e-9
    if self._settle_lat is None:
      self._settle_lat, self._settle_lon = msg.latitude, msg.longitude
      self._settle_t = self._rearm_t0 = t
      return False

    cos_lat = math.cos(math.radians(self._settle_lat))
    de = (msg.longitude - self._settle_lon) * M_PER_DEG * cos_lat
    dn = (msg.latitude - self._settle_lat) * M_PER_DEG
    if math.hypot(de, dn) > self.settle_radius:
      # 아직 움직이는 중 — 기준점을 현재로 옮기고 정지 타이머를 리셋
      self._settle_lat, self._settle_lon = msg.latitude, msg.longitude
      self._settle_t = t

    still = t - self._settle_t
    if still >= self.settle_sec:
      return True

    # RTK가 나빠 위치가 계속 튀면 영원히 '정지'로 안 잡힌다. 그 경우 노드가
    # 조용히 멎어버리는 게 원래 버그보다 나쁘므로 탈출구를 둔다.
    if t - self._rearm_t0 >= self.settle_timeout:
      self.get_logger().warn(
          f'정지 감지 실패 ({self.settle_timeout:.0f}s 경과 — GPS 튐 가능). '
          f'그대로 시작점을 잡는다. 차량이 멈춰 있는지 눈으로 확인할 것.')
      return True

    if t - self._last_settle_log >= 2.0:
      self._last_settle_log = t
      self.get_logger().info(
          f'재시작 대기: 차량을 세우고 기다리세요 '
          f'({still:.0f}/{self.settle_sec:.0f}s)')
    return False

  def _steer_bias(self):
    """직진 궤적의 활꼴 높이로 조향 중앙(STEER_CENTER) 오차를 역산한다.

    자동 직진은 조향 0°를 명령하므로, 그래도 호를 그렸다면 그건 사람 손이
    아니라 기계의 계통 오차다(= 고칠 수 있다). 현 → 궤적 최대 수직거리 h와
    현 길이 c로 곡률을 구하고, 자전거 모델로 조향각 오차를 낸다.

        κ = 8h/c²          (원호 근사)
        δ = atan(κ·L)      (L = 축거)
        ΔADC = δ · counts_per_deg

    반환: (h[m], δ[도], 권장 STEER_CENTER). 좌로 휘면 δ>0.
    """
    if len(self.track) < 3:
      return None
    e0, n0 = self.track[0]
    e1, n1 = self.track[-1]
    cx, cy = e1 - e0, n1 - n0
    c = math.hypot(cx, cy)
    if c < 1.0:
      return None
    h = 0.0
    for (e, n) in self.track:
      # 외적 부호: 현(chord) 진행방향 기준 왼쪽이 +
      d = (cx * (n - n0) - cy * (e - e0)) / c
      if abs(d) > abs(h):
        h = d
    # ★ 원호는 곡률 중심의 반대쪽으로 부푼다. 좌회전이면 중심이 왼쪽이므로
    # 호는 현 기준 '오른쪽'으로 불룩하다 → 활꼴 높이 부호가 회전방향과 반대.
    # 그래서 뒤집어 회전방향 기준(좌가 +)으로 맞춘다.
    h = -h
    kappa = 8.0 * h / (c * c)
    delta_deg = math.degrees(math.atan(kappa * self.wheelbase))
    # +각도(좌) = ADC 증가. 좌로 휘었다면 실제 직진 ADC는 현재값보다 작다.
    # ★ counts/도 는 방향별로 다르다(좌 26.2 / 우 20.6). 평균값을 쓰면 권고치가
    #   한쪽으로 최대 12% 어긋난다 — 그대로 펌웨어에 넣으면 중립이 망가진다.
    cpd = self.cpd_left if delta_deg >= 0 else self.cpd_right
    new_center = self.steer_center - delta_deg * cpd
    return h, delta_deg, new_center

  def _rtk_ready(self, msg: NavSatFix) -> bool:
    """RTK 가 수렴했는가. 수렴 전에는 시작점조차 잡지 않는다.

    ★ 판정은 status.status 가 아니라 **공분산**으로 한다.
      지금 쓰는 ublox_nav_sat_fix_hp 드라이버는 RTK Fixed 에서도
      status.status=1(SBAS) 만 내보내고 2(GBAS)를 절대 안 준다.
      status>=2 로 걸면 영원히 통과 못 한다(waypoint_recorder 에서 확인된 사실).
      공분산은 Fixed(수 mm) / Float(수십 cm) 구분이 훨씬 확실하다.
    """
    if not self.require_rtk:
      return True
    h_std = math.sqrt(max(0.0, msg.position_covariance[0]
                          + msg.position_covariance[4]))
    if msg.position_covariance_type == 0:
      ok = msg.status.status >= 2          # 공분산을 못 믿는 드라이버 폴백
    else:
      ok = h_std <= self.max_h_std
    t = self.get_clock().now().nanoseconds * 1e-9
    if ok:
      if not self._rtk_ok:
        self._rtk_ok = True
        self.get_logger().info(
            f'RTK 수렴 (수평 σ={h_std * 100:.1f}cm) — 헤딩 캘리브를 시작한다.')
      return True
    self._rtk_ok = False
    if t - self._last_rtk_log > 2.0:
      self._last_rtk_log = t
      self.get_logger().warn(
          f'RTK 수렴 대기 중 (수평 σ={h_std * 100:.1f}cm > '
          f'{self.max_h_std * 100:.0f}cm) — 출발하지 않는다. '
          f'⚠ 여기서 출발하면 RTK 확정 순간의 좌표 점프를 직진으로 오인한다'
          f'(2026-08-24 충돌 원인). 정말 무시하려면 require_rtk:=false.')
    return False

  def _invalidate_calib(self, reason):
    """측위 이상으로 캘리브를 무효화한다.

    ★ 왜 _reject_and_retry 와 따로 두나
      저쪽은 '차가 휘었다'가 전제라 궤적 곡률에서 조향 트림을 학습한다.
      GPS 가 튄 경우 그 궤적은 차의 움직임이 아니므로, 거기서 트림을 뽑으면
      멀쩡한 조향에 엉뚱한 보정이 쌓인다. 그래서 트림은 손대지 않는다.
    """
    self.calib_fail_count += 1
    self.get_logger().error(f'❌ 헤딩 캘리브 무효: {reason}')
    self.lat0 = None
    self.prev_e = self.prev_n = None
    self.max_dev = 0.0
    self.max_dev_at = 0.0
    self.max_dev_run = 0.0
    self._dev_run_start = None
    self.drive_yaw0 = None
    self._last_fix = None
    self.calib_t0 = None
    self.drive_started_t = None
    self._last_log_m = -1
    self.rearming = True
    self._settle_lat = None
    self._last_settle_log = 0.0
    if self.auto_drive:
      if self.calib_fail_count < self.max_calib_retries:
        self.drive_t0 = None
        self.get_logger().warn(
            f'자동 재시도 {self.calib_fail_count}/{self.max_calib_retries} — '
            f'차를 시작점으로 되돌리면 자동 재출발(후진 안 함).')
        self._stop_driving(f'재시도 {self.calib_fail_count}')
      else:
        self.drive_aborted = True
        self.get_logger().error(
            '연속 실패 — 자동 재주행 중단. RTK/안테나 상태를 확인할 것.')
        self._stop_driving('측위 이상 연속 실패')

  def fix_cb(self, msg: NavSatFix):
    if self.done_time is not None:
      return
    if math.isnan(msg.latitude) or abs(msg.latitude) < 1e-9:
      return
    # ① RTK 게이트 — 수렴 전에는 시작점도 안 잡고 카운트다운도 안 돈다.
    if self.lat0 is None and not self._rtk_ready(msg):
      return
    # ④ 앞바퀴 정렬 게이트 — 꺾인 채로 출발하면 초반이 곡선이 된다.
    if self.lat0 is None and not self._wheels_straight():
      return
    if self.lat0 is None:
      if self.rearming and not self._settled(msg):
        return
      # ★ 재시도(자동직진)면 원점 근처로 되돌아왔을 때만 새 시작점을 잡는다.
      # 실패 지점(10m 앞)에서 그대로 출발하면 전방 커브·벽으로 돌진하기 때문.
      if (self.auto_drive and self.calib_fail_count > 0
              and self.origin_lat is not None):
        d = math.hypot(
            (msg.longitude - self.origin_lon) * M_PER_DEG
            * math.cos(math.radians(self.origin_lat)),
            (msg.latitude - self.origin_lat) * M_PER_DEG)
        if d > self.retry_start_radius:
          t = self.get_clock().now().nanoseconds * 1e-9
          if t - self._last_reposition_log > 2.0:
            self._last_reposition_log = t
            self.get_logger().warn(
                f'↩ 차를 시작점으로 되돌리세요 (현재 {d:.1f}m 앞 — 전방 벽/커브 '
                f'보호). {self.retry_start_radius:.0f}m 안으로 오면 자동 재출발.')
          return
      self.rearming = False
      self._settle_lat = None
      self.lat0, self.lon0 = msg.latitude, msg.longitude
      if self.origin_lat is None:       # 최초 시작점을 복귀 기준으로 저장
        self.origin_lat, self.origin_lon = msg.latitude, msg.longitude
      self.cos_lat0 = math.cos(math.radians(self.lat0))
      self.prev_e = self.prev_n = None
      self.max_dev = 0.0
      self.max_dev_at = 0.0
      self.track = [(0.0, 0.0)]
      self.calib_t0 = self.get_clock().now().nanoseconds * 1e-9
      self._last_fix = (self.calib_t0, 0.0, 0.0)
      self.get_logger().info(
          f'시작점 기록: ({self.lat0:.7f}, {self.lon0:.7f}). 직진 시작하세요.')
      if self.auto_drive and self.drive_t0 is None:
        self.drive_t0 = self.get_clock().now().nanoseconds * 1e-9
      return

    east = (msg.longitude - self.lon0) * M_PER_DEG * self.cos_lat0
    north = (msg.latitude - self.lat0) * M_PER_DEG
    dist = math.hypot(east, north)

    # ② 점프 게이트 — 연속한 두 fix 사이의 '함축 속도'가 물리적으로 불가능하면
    #    차가 움직인 게 아니라 측위 해가 튄 것이다.
    #    사고 당시 0.38초에 7.3m(=19m/s)가 그대로 적분됐다.
    t_now = self.get_clock().now().nanoseconds * 1e-9
    if self._last_fix is not None:
      lt, le, ln = self._last_fix
      dt = t_now - lt
      step = math.hypot(east - le, north - ln)
      if dt > 1e-3 and step / dt > self.max_jump_speed:
        v_imp = step / dt
        if self.drive_started_t is None:
          # 아직 굴러가기 전이다. 차는 가만히 있는데 좌표만 튄 것이므로
          # 실패로 세지 않고 **기준점만 다시 잡는다**(RTK 확정 직후 정상 동작).
          self.get_logger().warn(
              f'측위 점프 감지: {step:.1f}m/{dt:.2f}s = {v_imp:.1f}m/s '
              f'(출발 전) — 시작점을 다시 잡는다.')
          self.lat0 = None
          self._last_fix = None
          self.calib_t0 = None
          self.track = []
          self.drive_t0 = None       # 카운트다운도 다시
          self._last_log_m = -1
          return
        self._invalidate_calib(
            f'주행 중 측위 점프 {step:.1f}m/{dt:.2f}s = {v_imp:.1f}m/s '
            f'(상한 {self.max_jump_speed:.1f}m/s). '
            f'이 점프를 직진으로 적분하면 헤딩이 통째로 틀어진다.')
        return
    self._last_fix = (t_now, east, north)
    # 캘리브가 끝나지 않고 세션이 길어져도 무한정 쌓이지 않게 상한을 둔다.
    # 정상 캘리브(10m, ~5Hz)는 100점 안팎이라 걸릴 일이 없다.
    if len(self.track) < 2000:
      self.track.append((east, north))

    # ★ 직진성 검증: 시작점→현재점의 '직선 방향'을 헤딩으로 쓰기 때문에,
    # 캘리브 중 곡선으로 가거나 후진하면 그 직선이 실제 진행방향과 달라져
    # yaw_offset이 통째로 틀어진다(전 구간 경로 이탈로 이어짐).
    # 최근 구간의 진행방향과 전체 직선방향이 크게 다르면 캘리브를 거부한다.
    # ★ 직진성 평가는 '최근 구간 방향 vs 전체 현 방향' 으로 한다.
    #
    # 예전엔 최근 구간을 '직전 GPS 점 ~ 현재 점'(약 0.3m)으로 잡았는데,
    # 그 구간이 너무 짧아 **GPS 한 점만 튀어도 방향이 90° 넘게 꺾여** 멀쩡한
    # 직진이 거부됐다(2026-08-18 현장: 실제 궤적은 편차 1~2° 인데 '97°' 로 거부).
    # v=1.5m/s 로 빠르면 5Hz 샘플 간격이 0.3m 라 단일 튐값에 그대로 노출된다.
    #
    # 대책: 최근 seg_window_m(기본 1.5m) 전의 점과 비교해 구간을 길게 잡는다.
    # 구간이 길수록 점 하나의 튐이 방향에 주는 영향이 작아진다.
    if dist >= self.min_chord_for_dev:
      # track 뒤에서부터 현재로부터 seg_window_m 이상 떨어진 점을 찾는다
      ref = None
      for (pe, pn) in reversed(self.track[:-1]):
        if math.hypot(east - pe, north - pn) >= self.seg_window:
          ref = (pe, pn)
          break
      if ref is not None:
        seg_course = math.atan2(north - ref[1], east - ref[0])
        chord_course = math.atan2(north, east)
        dev = abs(normalize_angle(seg_course - chord_course))
        if dev > getattr(self, 'max_dev', 0.0):
          self.max_dev = dev
          self.max_dev_at = dist
        # ★ 지속편차: 편차가 임계 초과인 '연속 구간'의 길이를 추적한다.
        # 순간 튐은 짧게 끝나고(곧 회복), 진짜 곡선은 길게 이어진다.
        if dev > self.max_deviation:
          if self._dev_run_start is None:
            self._dev_run_start = dist
          self.max_dev_run = max(self.max_dev_run, dist - self._dev_run_start)
          # ★ 조기 감지(early abort): 지속편차가 확인되는 즉시 중단한다.
          # 10m 끝까지 가서 판정하면 벽/커브 코앞(9m)에서 서게 되지만, 여기서
          # 잡으면 ~3m 에서 멈춘다 — 벽에서 멀고 되돌리기도 쉽다.
          if self.max_dev_run > self.dev_sustain_m:
            self._reject_and_retry(dist)
            return
        else:
          self._dev_run_start = None

    if dist >= self.calib_distance:
      if self.imu_yaw is None:
        self.get_logger().warn('IMU yaw 미수신 — 헤딩 계산 보류.')
        return
      max_dev = getattr(self, 'max_dev', 0.0)

      # ③ 최소 소요시간 게이트 — 10m 를 물리적으로 가능한 시간보다 빨리
      #    '갔다'면 그건 주행이 아니라 측위 점프의 누적이다.
      #    자동직진은 속도제어를 받으므로 auto_speed 의 2배를 넘을 수 없다.
      #    (사고 때: 출발 3.4초 만에 10.1m '완주' → 여기서 걸린다)
      if self.auto_drive and self.drive_started_t is not None:
        v_cap = max(0.05, self.auto_speed * 2.0)
        t_ref = self.drive_started_t
      else:
        v_cap = self.max_jump_speed          # 사람이 미는 경우
        t_ref = self.calib_t0
      if t_ref is not None:
        need = self.calib_distance / v_cap
        took = self.get_clock().now().nanoseconds * 1e-9 - t_ref
        if took < need:
          self._invalidate_calib(
              f'{dist:.1f}m 를 {took:.1f}초 만에 갔다고 나온다 '
              f'(최소 {need:.1f}초 필요, 속도상한 {v_cap:.2f}m/s). '
              f'실제 주행이 아니라 측위 점프다.')
          return

      # ③-b peak 편차 하드 상한 — 지속편차(1.5m) 를 못 채운 큰 튐도 여기서 막는다.
      #     사고 때 peak 36° 가 아무 저항 없이 통과했다.
      if max_dev > self.max_peak_dev:
        self._invalidate_calib(
            f'최대 편차 {math.degrees(max_dev):.0f}° > '
            f'{math.degrees(self.max_peak_dev):.0f}° (@{self.max_dev_at:.1f}m). '
            f'직진으로 볼 수 없다.')
        return

      course = math.atan2(north, east)
      yaw_offset = normalize_angle(course - self.imu_yaw)
      # max_dev를 성공 시에도 남긴다: 자동 직진에서 이 값이 매번 한쪽으로
      # 크게 나오면 STEER_CENTER가 어긋났다는 신호다(무료 진단).
      self.get_logger().info(
          f'✅ 헤딩 초기화 완료: {dist:.1f}m 직진 (최대 편차 '
          f'{math.degrees(max_dev):.0f}°). '
          f'GPS course={math.degrees(course):.1f}°, '
          f'IMU yaw={math.degrees(self.imu_yaw):.1f}°, '
          f'→ yaw_offset={math.degrees(yaw_offset):.1f}°')
      # 래치 발행 (구독자에게 전파될 시간을 준 뒤 자동 종료)
      self.off_pub.publish(Float64(data=float(yaw_offset)))
      self.course_pub.publish(Float64(data=float(course)))
      self.done_time = self.get_clock().now().nanoseconds * 1e-9

      # 조향 중앙 진단: 자동 직진(조향 0° 명령)이었을 때만 의미가 있다.
      # 사람이 밀었으면 휘어짐이 사람 탓이라 STEER_CENTER를 못 물어본다.
      bias = self._steer_bias() if self.auto_drive else None
      if bias is not None:
        h, delta_deg, new_center = bias
        side = '좌' if h > 0 else '우'
        if abs(h) < 0.10:
          self.get_logger().info(
              f'조향 중앙 점검: 횡편차 {h:+.2f}m — STEER_CENTER '
              f'{self.steer_center} 양호 (보정 불필요)')
        else:
          self.get_logger().warn(
              f'조향 중앙 어긋남: 10m에서 {side}로 {abs(h):.2f}m 휘었다 '
              f'(조향각 오차 {delta_deg:+.2f}°). '
              f'STEER_CENTER {self.steer_center} → {new_center:.0f} 권장. '
              f'⚠ 부호는 첫 회에 눈으로 확인할 것 (휜 방향과 맞는지).')

      if self.auto_drive:
        self._stop_driving('캘리브 완료')
    else:
      if int(dist) != getattr(self, '_last_log_m', -1):
        self._last_log_m = int(dist)
        self.get_logger().info(f'직진 중... {dist:.1f}/{self.calib_distance:.0f}m')

  def _reject_and_retry(self, dist):
    """직진성 실패 처리 + 자동 재시도. 조기 감지·종점 어디서 불려도 동일.

    후진하지 않는다. 제동·정지 후 원점 근처로 되돌아오면(fix_cb 의 원점 게이트)
    다시 앞으로 간다. max_calib_retries 를 다 쓰면 계통 문제로 보고 중단한다.
    """
    max_dev = getattr(self, 'max_dev', 0.0)
    self.calib_fail_count += 1
    self.get_logger().error(
        f'❌ 직진이 아닙니다 (편차 {math.degrees(self.max_deviation):.0f}° 초과가 '
        f'{self.max_dev_run:.1f}m 지속 > {self.dev_sustain_m:.1f}m; '
        f'peak {math.degrees(max_dev):.0f}° @ {dist:.1f}m). 헤딩 캘리브 무효.')
    # ★ 적응형 조향 트림 (자동직진 '무조건 되게'의 핵심): 이번에 휜 곡률에서
    # 필요한 조향 보정을 역산해, 다음 시도에서 미리 상쇄한다. STEER_CENTER 가
    # 심하게 어긋나 폐루프(±5°)가 포화되는 경우에도, 재시도마다 트림이 쌓여
    # 곧아진다(수렴). 절반씩(0.6) 반영해 오버슈트를 막는다.
    if self.auto_drive:
      bias = self._steer_bias()
      if bias is not None:
        _, delta_deg, _ = bias    # 좌로 휘면 δ>0 → 우로(음수) 트림
        old = self.auto_steer_bias
        self.auto_steer_bias = max(-8.0, min(8.0, old - 0.6 * delta_deg))
        if abs(self.auto_steer_bias - old) > 0.05:
          self.get_logger().warn(
              f'적응형 조향 트림: {self.auto_steer_bias - old:+.1f}° 반영 → '
              f'누적 {self.auto_steer_bias:+.1f}° (다음 시도 더 곧게 간다)')
    self.lat0 = None
    self.prev_e = self.prev_n = None
    self.max_dev = 0.0
    self.max_dev_at = 0.0
    self.max_dev_run = 0.0
    self._dev_run_start = None
    self.drive_yaw0 = None
    self._last_fix = None
    self.calib_t0 = None
    self.drive_started_t = None
    self._last_log_m = -1
    self.rearming = True
    self._settle_lat = None
    self._last_settle_log = 0.0
    if self.auto_drive:
      if self.calib_fail_count < self.max_calib_retries:
        self.drive_t0 = None       # 카운트다운 재무장 (원점 복귀 후 재출발)
        self.get_logger().warn(
            f'자동 재시도 {self.calib_fail_count}/{self.max_calib_retries} — '
            f'차를 시작점으로 되돌리면 자동 재출발(후진 안 함).')
        self._stop_driving(f'재시도 {self.calib_fail_count}')
      else:
        self.drive_aborted = True
        self.get_logger().error(
            f'{self.calib_fail_count}회 연속 실패 — STEER_CENTER(424) 어긋남 등 '
            f'기계 문제 의심. 자동 재주행 중단. 수동으로 시도할 것.')
        self._stop_driving('직진성 검증 연속 실패')

  def _stop_driving(self, reason):
    """구동을 멈춘다. 1초간 0을 쏴서 세운 뒤 토픽을 놓는다.

    놓으면 mux의 teleop_timeout(0.5s)이 지나 자율(AUTO)로 자동 인계된다.
    """
    if self.brake_until is None:
      self.get_logger().info(f'자동 직진 종료: {reason}')
    self.brake_until = self.get_clock().now().nanoseconds * 1e-9 + 1.0

  def drive_tick(self):
    """자동 직진: 조향 0°로 곧게 전진. auto_drive일 때만 타이머가 붙는다."""
    t = self.get_clock().now().nanoseconds * 1e-9

    # 제동 구간: 0을 쏴서 세우고, 끝나면 토픽을 놓아 AUTO로 인계
    if self.brake_until is not None:
      if t < self.brake_until:
        self.drive_pub.publish(Twist())
      return

    if self.drive_aborted or self.done_time is not None:
      return
    if self.drive_t0 is None:      # 아직 시작점(=유효 GPS)이 없다
      return

    elapsed = t - self.drive_t0
    if elapsed < self.auto_countdown:
      left = int(self.auto_countdown - elapsed) + 1
      if left != self._last_cd_log:
        self._last_cd_log = left
        self.get_logger().warn(f'자동 직진 {left}초 전... (E-stop 준비)')
      self.drive_pub.publish(Twist())   # 카운트다운 중엔 정지 명령
      return

    if elapsed - self.auto_countdown > self.auto_timeout:
      self.drive_aborted = True
      self.get_logger().error(
          f'❌ 자동 직진 타임아웃: {self.auto_timeout:.0f}초 안에 '
          f'{self.calib_distance:.0f}m를 못 갔다. 구동 FF 부족이나 스톨 의심 — '
          f'수동으로 직진시키거나 auto_speed를 올릴 것.')
      self._stop_driving('타임아웃')
      return

    # ★ 헤딩 유지. 조향 0°만 주는 개루프면 STEER_CENTER 가 조금만 어긋나도
    # 차가 한쪽으로 계속 휘고, 그러면 직진성 검증에서 거부된다
    # (2026-08-18 현장: 우측으로 계속 쏠림).
    #
    # 헤딩의 **절대값**은 아직 모르지만(그걸 구하는 게 이 노드의 목적),
    # 출발 시점 대비 **변화량**은 IMU 만으로 정확히 알 수 있다. 그래서
    # '출발할 때의 yaw 를 유지'하는 폐루프를 걸면 절대 헤딩을 몰라도 곧게 간다.
    # straight_drive 가 같은 방식으로 30m 를 ±3° 로 유지한 실적이 있다.
    steer = 0.0
    if self.imu_yaw is not None:
      if self.drive_yaw0 is None:
        self.drive_yaw0 = self.imu_yaw
        self.get_logger().info(
            f'직진 기준 헤딩 고정: {math.degrees(self.drive_yaw0):.1f}° '
            f'(절대값은 무의미, 변화량만 사용)')
      err = math.degrees(normalize_angle(self.drive_yaw0 - self.imu_yaw))
      steer = max(-self.auto_max_corr,
                  min(self.auto_max_corr, self.auto_heading_gain * err))
    # 수동 트림(조향 중앙이 틀어진 걸 임시로 상쇄하고 싶을 때)
    steer += self.auto_steer_bias
    steer = max(-self.auto_max_corr - abs(self.auto_steer_bias),
                min(self.auto_max_corr + abs(self.auto_steer_bias), steer))

    if self.drive_started_t is None:
      # 카운트다운이 끝나고 실제로 구동 명령이 나가는 첫 순간.
      # 최소 소요시간 게이트(③)의 기준점이다.
      self.drive_started_t = t

    cmd = Twist()
    cmd.linear.x = self.auto_speed
    cmd.angular.z = float(steer)   # 조향각[도]
    self.drive_pub.publish(cmd)

  def shutdown_check(self):
    # 오프셋 발행 후 1초 지나면 자동 종료 (래치 전파 시간 확보)
    if self.done_time is not None:
      t = self.get_clock().now().nanoseconds * 1e-9
      # 제동이 안 끝났으면 기다린다. 먼저 죽어버리면 차가 굴러가는 채로
      # mux가 AUTO로 넘어간다.
      if self.brake_until is not None and t < self.brake_until:
        return
      if t - self.done_time > 1.0:
        self.get_logger().info('yaw_offset 발행 완료 → 노드 자동 종료.')
        rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)
  node = HeadingInitNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

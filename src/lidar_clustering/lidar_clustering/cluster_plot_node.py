import math

import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from lidar_clustering.dbscan_clusterer import DBSCANClusterer
from lidar_clustering.follow_gap_planner import FollowGapPlanner
from lidar_clustering.hungarian_tracker import HungarianTracker
from lidar_clustering.obstacle_features import add_obstacle_features
from lidar_clustering.scan_preprocessor import ScanPreprocessor

# 회피 조향 '없음' 센티넬. 먹스는 NaN 을 받으면 GPS 경로 조향을 쓴다.
NO_STEER = float('nan')

# ★ 제어팀 통합 (2026-08-19): 원본은 결과를 matplotlib 그래프/로그로만 표시했다.
# 차량에 물리려면 ROS 로 발행해야 하고, 헤드리스에서 그래프가 죽으면 안 된다.
#   /obstacle_distance : CLEAR→999(감속없음), AVOID/BLOCKED→front(종방향이 감속/정지),
#                        NO_SCAN→0(라이다 무신호 = 정지). 기존 종방향 장애물 로직 재활용.
#   /lidar/avoid_steer : AVOID→best_angle_deg(+좌/−우, 우리 조향 규약과 동일), 그 외 NaN.
#   /lidar/mode        : 진단용 문자열.
# enable_plot=False(기본)면 그래프를 아예 만들지 않아 헤드리스에서 안전하다.


class ClusterPlotNode(Node):
    """
    /scan
      -> preprocessing
      -> DBSCAN
      -> Hungarian tracking
      -> obstacle features

    In parallel:
    raw /scan
      -> Follow the Gap
      -> STRAIGHT / LEFT / RIGHT / STOP
    """

    def __init__(self):
        super().__init__('cluster_plot_node')

        self.preprocessor = ScanPreprocessor(
            min_detection_range=0.30,
            max_detection_range=8.0,
            roi_x_min=0.30,
            roi_x_max=16.0,
            roi_y_min=-8.0,
            roi_y_max=8.0,
            yaw_offset_deg=180.0,
        )

        self.clusterer = DBSCANClusterer(
            eps=0.25,
            min_samples=4,
            minimum_cluster_points=4,
        )

        self.tracker = HungarianTracker(
            gating_distance=1.60,
            max_missed_frames=25,
            display_missed_frames=5,
            max_tentative_missed_frames=4,
            min_confirmed_hits=2,
            position_smoothing_alpha=0.70,
            velocity_smoothing_alpha=0.35,
        )

        # ★ 대회 회피 미션(2026)에 맞춰 ROS 파라미터화 — 실트랙에서 리빌드 없이 튜닝.
        #
        # ★ 규정 (경기규정 항목 4 — S코스 장애물 회피, 신규)
        #   · 장애물은 **T870 차체(바퀴 없음) 2대**. 중앙선 없이 좌·우 자유 회피.
        #   · 배치는 좌→우 / 우→좌 **랜덤, 매 주행마다 변동 가능**.
        #     → 궤적을 미리 찍어둘 수 없다. 라이다가 그날 본 대로 피해야 한다.
        #   · 감점: 접촉 10점/회 · 미션 포기 15점 · 구간 내 차선이탈 최대 10점.
        #
        #   ⚠ 예전 이 주석에는 "이삿짐박스 900×500×600mm, 2.5m 간격" 이 적혀
        #     있었는데 **규정에 없는 값이었다**(출처 불명, 2026-09-05 대조).
        #     아래 기본값들도 그 잘못된 전제 위에서 잡힌 것이라 **현장 실측 후
        #     다시 잡아야 한다.** tools/lidar_slalom_test.py 로 검증할 것.
        #
        #   yaw_offset_deg      : 라이다 0°가 향하는 방향 보정(마운트 따라. 180=후방).
        #   obstacle_trigger    : 이 거리 안 장애물에 반응.
        #   planning_lookahead  : 갭 계획 전방거리.
        #   track_width         : 도로 폭. 이 밖 점은 무시. **실측 필요**
        #   vehicle_width       : 차폭 0.775. safety_margin: 여유.
        gp = lambda n, d: float(self.declare_parameter(n, d).value)
        self.follow_gap_planner = FollowGapPlanner(
            yaw_offset_deg=gp('fg_yaw_offset_deg', 180.0),
            front_fov_deg=gp('fg_front_fov_deg', 180.0),
            min_range=gp('fg_min_range', 0.30),
            max_range=gp('fg_max_range', 8.0),
            track_width=gp('fg_track_width', 2.7),
            planning_lookahead=gp('fg_planning_lookahead', 2.2),
            obstacle_trigger_distance=gp('fg_obstacle_trigger', 3.0),
            vehicle_width=gp('fg_vehicle_width', 0.775),
            safety_margin=gp('fg_safety_margin', 0.25),
            straight_deadband_deg=gp('fg_straight_deadband_deg', 5.0),
            min_gap_width_deg=gp('fg_min_gap_width_deg', 3.0),
            side_score_margin=gp('fg_side_score_margin', 0.20),
            # 갭 안에서 겨냥점 — 'nearest' 는 '필요한 만큼만 비켜간다'.
            # follow_gap_planner 의 self.aim 주석 참고.
            # 기본 'path' — 측정 근거는 follow_gap_planner 의 self.aim 주석.
            aim=str(self.declare_parameter('fg_aim', 'path').value),
            aim_margin_deg=gp('fg_aim_margin_deg', 2.0),
        )

        # 헤드리스 기본. 그래프가 필요할 때만(디버깅) enable_plot:=true.
        self.enable_plot = bool(
            self.declare_parameter('enable_plot', False).value)
        self.plotter = None
        if self.enable_plot:
            from lidar_clustering.lidar_plotter import LidarPlotter
            self.plotter = LidarPlotter(
                on_close=self.on_plot_close,
                on_reset=self.reset_system,
                x_min=0.0, x_max=16.0, y_min=-8.0, y_max=8.0,
            )

        # CLEAR 시 장애물 없음으로 보낼 거리(종방향 obstacle_trigger 4.0 보다 커야
        # 감속이 안 걸린다).
        self.clear_distance = float(
            self.declare_parameter('clear_distance', 999.0).value)

        # ★ 미션 시퀀서 연동 (2026-09-04)
        #
        #   라이다 출력은 성격이 다른 두 갈래다. **같이 끄면 안 된다.**
        #
        #     /obstacle_distance  종방향 감속·정지  = 안전 기능 → 항상 켠다
        #     /lidar/avoid_steer  조향 override    = 회피 기동 → 구간 한정
        #
        #   조향만 구간을 제한하는 이유: 회피는 조향을 통째로 뺏는다. S코스가
        #   아닌 곳에서 관중·표지물·연석에 반응해 틀면 그게 곧 **이탈=탈락**이다.
        #   반대로 감속까지 끄면 코스 어디서든 앞에 뭐가 있어도 안 서게 된다 —
        #   그건 더 위험하다. 그래서 감속은 언제나 살려둔다.
        #
        #   기본 false — 시퀀서 없이 쓰던 런치는 그대로 돌아간다.
        self.require_arm = bool(
            self.declare_parameter('require_arm_for_steer', False).value)
        self.armed = not self.require_arm

        # ★ /lidar/mute — 전방 감속을 잠깐 끈다 (2026-09-04)
        #   돌발 급정지 미션이 5초 대기 후에도 더미가 안 치워졌을 때, 이걸 켜서
        #   빠져나간다. **1분 이상 정지는 감점이 아니라 탈락**이기 때문이다.
        #   위험한 기능이라 켠 동안 계속 경고를 남긴다.
        self.muted = False

        # ★ 막힘 탈출 (2026-09-11) — **서 있는 것이 가장 비싼 실패다.**
        #
        #   BLOCKED 는 '트랙 안에 통과할 갭이 없다' 는 뜻이고, 그때
        #   obstacle_distance = front_distance 를 내보내 종방향이 선다.
        #   그런데 **차가 서면 스캔이 안 바뀌므로 계속 BLOCKED 다.** 탈출구가
        #   없어서 영원히 멈춰 있는다 — 규정상 **1분 이상 정지는 탈락**이다.
        #   (2026-09-11 tools/lidar_slalom_test.py 우→좌 배치에서 실제로 이렇게
        #    끝났다: 두 번째 장애물 정면에서 BLOCKED 로 고착)
        #
        #   그래서 일정 시간 막혀 있으면 회피를 포기하고 감속을 풀어 GPS 경로로
        #   빠져나간다. 장애물에 닿으면 감점(S코스 접촉 10점)이지만,
        #   거기 서 있으면 탈락이다. **감점이 언제나 탈락보다 낫다.**
        #   빠져나가는 방향이 GPS 경로라는 점도 중요하다 — 경로는 기록된 주행선
        #   이므로, 과회피로 차선을 벗어나 갇힌 경우에는 이게 곧 복귀다.
        #
        #   ★ 값 정하는 법 — **정당한 정지보다 길고, 탈락 문턱보다 훨씬 짧게.**
        #     아래보다 짧으면 미션을 스스로 망친다:
        #       돌발 급정지  규정 최소 3초 정지 (sudden_stop_node dwell 기본 5초)
        #       경사로       3초 이상 정지
        #       횡단보도     3초 대기
        #     위로는 탈락 문턱이 60초다. 그래서 8초로 둔다.
        #     (3초로 뒀다가 돌발 급정지의 3초 정지와 정면으로 겹치는 걸
        #      2026-09-12 에 확인했다 — 규정 시간을 채우기 전에 출발해 버린다)
        #     ※ 신호교차로 신호대기는 /stop_line_distance 로 서는 것이라
        #       이 타이머와 무관하다(라이다 전방거리로만 판정한다).
        #   0 으로 두면 비활성(예전 동작 그대로).
        self.blocked_escape_s = float(
            self.declare_parameter('blocked_escape_s', 8.0).value)
        #   라이다가 죽으면(NO_SCAN) obstacle_distance=0 이라 역시 영원히 선다.
        #   센서가 나가도 GPS 주행은 되므로, 더 오래 기다린 뒤 같은 처리를 한다.
        #   (lidar:=false 로 달리는 것과 같은 상태가 될 뿐이다)
        self.no_scan_escape_s = float(
            self.declare_parameter('no_scan_escape_s', 8.0).value)
        #   ★ 2026-09-12 — 고착은 BLOCKED 만이 아니다.
        #   AVOID 일 때도 obstacle_distance = front_distance 를 내보내므로,
        #   종방향이 obstacle_stop_dist(0.8m) 에서 **차를 세운다.** 라이다는
        #   조향으로 비켜가려 하는데 차가 안 나가니 스캔도 안 바뀐다 →
        #   장애물 앞에 붙어서 영원히 정지. (2026-09-12 tools/avoid_on_track.py
        #   로 학교 트랙에 장애물 3개를 놓았더니 400초에 7m 만 갔다)
        #   그래서 '전방 거리가 정지 문턱 안에 머무는 상태' 도 고착으로 본다.
        #   ★ 실제로 서는 거리는 obstacle_stop_dist(0.8) 가 아니라 **1.0m** 다.
        #     종방향이 v_obs = v_slow·(d−obs_stop)/span 을 쓰고, 그 값이 0.05
        #     이하면 0 으로 떨어뜨리기 때문이다:
        #       0.8·(d−0.8)/3.2 = 0.05  →  d = 1.0
        #     그래서 문턱을 1.0 보다 위(1.2)에 둬야 이 고착이 잡힌다.
        #     0.9 로 뒀다가 front=0.99 에서 안 걸리는 걸 실측으로 확인했다.
        self.stuck_distance = float(
            self.declare_parameter('stuck_distance', 1.2).value)
        self._stuck_since = None      # 정지 유발 상태가 시작된 시각
        self._escaping = False

        # ★ 회피각을 조향각으로 어떻게 옮길 것인가 (2026-09-12)
        #   follow-gap 이 내는 best_angle_deg 는 **갭 방향(방위각)** 이지
        #   조향각이 아니다. 그런데 먹스는 이 값을 그대로 조향각으로 쓴다
        #   (vehicle_cmd_mux_node: s = self.avoid_steer).
        #   자전거 모델에서 방위각 α 를 따라가는 조향각은
        #       δ = atan(2·L·sin α / Ld)
        #   이고, L=0.785 · Ld=2.2 에서 α=13° → δ=9.1° 다. 즉 그대로 쓰면
        #   **1.43배 과조향**이고, 이게 회피가 차선을 크게 벗어나는 원인 중 하나다.
        #     'bearing' = 예전 동작(그대로 사용)
        #     'pursuit' = 퓨어퍼슛 환산
        # ★ aim='path' 용 — 경로 추종이 원하는 조향 방향.
        #   local_pure_pursuit 가 /steering_cmd 로 낸다(도, 좌+/우−).
        #   이게 있어야 '경로로 돌아오는 쪽으로 비켜간다' 가 된다.
        #   못 받으면(타임아웃) 0 으로 두어 aim='nearest' 와 같게 동작한다.
        self.path_steer_deg = 0.0
        self.path_steer_time = 0.0
        self.path_steer_timeout = float(
            self.declare_parameter('path_steer_timeout', 0.5).value)
        self.create_subscription(
            Float64,
            str(self.declare_parameter('path_steer_topic',
                                       '/steering_cmd').value),
            self.path_steer_cb, 10)

        self.steer_mode = str(
            self.declare_parameter('steer_mode', 'bearing').value).lower()
        self.steer_wheelbase = float(
            self.declare_parameter('steer_wheelbase', 0.785).value)

        # 제어팀 통합 발행
        self.obstacle_pub = self.create_publisher(
            Float64, '/obstacle_distance', 10)
        self.avoid_steer_pub = self.create_publisher(
            Float64, '/lidar/avoid_steer', 10)
        self.mode_pub = self.create_publisher(String, '/lidar/mode', 10)

        # ★ 2026-09-12 — 스캔 토픽을 파라미터로 뺐다.
        #   이 차에는 라이다가 **앞뒤 두 대** 달려 있다(둘 다 CP2102).
        #   드라이버를 두 번 띄우면 기본값 그대로라 **둘 다 /scan 에 발행**하고,
        #   그러면 앞뒤 스캔이 섞여 회피가 **뒤쪽 물체에 반응**한다 = 이탈.
        #   전방 회피는 /scan_front 만 봐야 한다.
        #   (기본값은 /scan 으로 둔다 — 한 대만 띄우던 기존 사용법이 그대로 돈다)
        self.scan_topic = str(
            self.declare_parameter('scan_topic', '/scan').value)
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.declare_parameter('arm_topic', '/lidar/arm').value),
            self.arm_cb, 10)
        self.create_subscription(
            Bool,
            str(self.declare_parameter('mute_topic', '/lidar/mute').value),
            self.mute_cb, 10)

        # ★ 스캔 워치독 (2026-09-12)
        #   토픽 이름이 어긋나면(단일 /scan vs 앞뒤 분리 /scan_front) 이 노드는
        #   **아무 말 없이 라이다가 없는 것처럼** 동작한다. 회피도 전방 정지도
        #   조용히 사라진다 — 현장에서 가장 찾기 어려운 종류의 고장이다.
        #   구독한 토픽 이름을 박아 크게 알린다.
        self._last_scan_t = None
        self.create_timer(2.0, self._scan_watchdog)

        self.window_closed = False
        self.frame_count = 0
        self.draw_interval = 2
        self.obstacle_log_interval = 10
        self.status_log_interval = 20

        self.get_logger().info(
            f'LiDAR DBSCAN + Hungarian + Follow the Gap started. '
            f'(scan={self.scan_topic}, yaw_offset='
            f'{math.degrees(self.follow_gap_planner.yaw_offset):.0f}°, '
            f'aim={self.follow_gap_planner.aim})'
        )
        self.get_logger().info(
            'Vehicle coordinate: +X forward, +Y left, -Y right.'
        )

    def _scan_watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_scan_t is None:
            self.get_logger().error(
                f'❌ 스캔이 한 번도 안 왔다 — 구독 토픽: {self.scan_topic}\n'
                '   드라이버가 도는지, **토픽 이름이 맞는지** 확인할 것.\n'
                '   앞뒤 두 대면 /scan_front 다(lidar_dual.launch.py).\n'
                '   한 대만 띄웠으면 /scan 이다 → -p scan_topic:=/scan',
                throttle_duration_sec=5.0)
        elif now - self._last_scan_t > 3.0:
            self.get_logger().error(
                f'❌ 스캔이 {now - self._last_scan_t:.0f}초째 끊겼다 '
                f'({self.scan_topic}) — 전방 정지·회피가 모두 죽어 있다.',
                throttle_duration_sec=5.0)

    def path_steer_cb(self, msg):
        self.path_steer_deg = float(msg.data)
        self.path_steer_time = self.get_clock().now().nanoseconds * 1e-9

    def _path_target_deg(self):
        """신선한 /steering_cmd 만 쓴다. 끊기면 0(직진 기준)으로 폴백."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.path_steer_time > self.path_steer_timeout:
            return 0.0
        return self.path_steer_deg

    def _bearing_to_steer(self, deg):
        """갭 방위각[도] → 조향각[도]. steer_mode 에 따라 환산하거나 그대로."""
        if self.steer_mode != 'pursuit':
            return float(deg)
        a = math.radians(float(deg))
        ld = max(self.follow_gap_planner.planning_lookahead, 0.1)
        return math.degrees(
            math.atan(2.0 * self.steer_wheelbase * math.sin(a) / ld))

    def on_plot_close(self):
        if self.window_closed:
            return
        self.window_closed = True
        self.get_logger().info('Graph window closed. Stopping node.')
        if rclpy.ok():
            rclpy.shutdown()

    def reset_system(self):
        self.preprocessor.reset()
        self.tracker.reset()
        self.frame_count = 0
        self.get_logger().info('Preprocessor and tracker were reset.')

    def print_obstacle_information(self, tracked_clusters):
        if not tracked_clusters:
            self.get_logger().info('No confirmed obstacles.')
            return

        for obstacle in tracked_clusters:
            self.get_logger().info(
                f'ID={obstacle.get("track_id", -1)}, '
                f'X={obstacle.get("center_x", 0.0):.2f} m, '
                f'Y={obstacle.get("center_y", 0.0):.2f} m, '
                f'Width={obstacle.get("width", 0.0):.2f} m, '
                f'Depth={obstacle.get("depth", 0.0):.2f} m, '
                f'Distance={obstacle.get("center_distance", 0.0):.2f} m'
            )

    def _publish_decision(self, d):
        """FollowGap 결정 → 제어팀 토픽.

        obstacle_distance 규약(종방향 기존 로직이 이 값으로 감속/정지):
          CLEAR   → clear_distance(감속 없음)
          AVOID   → front_distance (근접 시 안전 감속·정지. 회피는 조향이 담당)
          BLOCKED → front_distance (갭 없음 → 근접해서 정지)
          NO_SCAN → 0.0 (라이다 무신호 = 이동 위험 → 정지)
        avoid_steer: AVOID 일 때만 best_angle_deg, 그 외엔 NaN(먹스는 GPS 조향 사용).
        """
        mode = d.mode
        if mode == 'CLEAR':
            obs = self.clear_distance
            steer = NO_STEER
        elif mode == 'NO_SCAN':
            obs = 0.0
            steer = NO_STEER
        elif mode == 'AVOID':
            obs = float(d.front_distance)
            steer = self._bearing_to_steer(d.best_angle_deg)
        else:  # BLOCKED
            obs = float(d.front_distance)
            steer = NO_STEER

        # ★ 막힘 탈출 — 위 주석(self.blocked_escape_s) 참고.
        #   고착 두 종류를 모두 본다:
        #     BLOCKED/NO_SCAN — 갭이 없다 / 스캔이 없다
        #     AVOID 인데 전방이 정지 문턱 안 — 비켜가려는데 차가 안 나간다
        limit = {'BLOCKED': self.blocked_escape_s,
                 'NO_SCAN': self.no_scan_escape_s}.get(mode, 0.0)
        if (limit <= 0.0 and mode == 'AVOID'
                and self.blocked_escape_s > 0.0
                and float(d.front_distance) <= self.stuck_distance):
            limit = self.blocked_escape_s
        if limit > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._stuck_since is None:
                self._stuck_since = now
            stuck_for = now - self._stuck_since
            if stuck_for >= limit:
                if not self._escaping:
                    self._escaping = True
                    self.get_logger().error(
                        f'⚠ {mode} 가 {stuck_for:.1f}s 지속 — 회피를 포기하고 '
                        '감속을 푼다(GPS 경로로 빠져나간다).\n'
                        '   장애물 접촉은 감점이지만 1분 이상 정지는 탈락이다.')
                obs = self.clear_distance
                # ★ AVOID 고착이면 **조향은 유지한다.** 갭은 보이는데 속도가
                #   묶여 못 나가는 상황이므로, 속도만 풀어 하던 회피를 마저
                #   시킨다. GPS 조향으로 되돌리면 장애물 쪽으로 되돌아간다.
                #   BLOCKED/NO_SCAN 은 갈 길이 안 보이는 것이라 GPS 로 맡긴다.
                if d.mode == 'AVOID':
                    steer = self._bearing_to_steer(d.best_angle_deg)
                else:
                    steer = NO_STEER
                mode = f'{mode}(ESCAPE)'
                self.get_logger().warn(
                    f'막힘 탈출 중 — 전방 감속 없음 ({stuck_for:.0f}s)',
                    throttle_duration_sec=1.0)
        else:
            self._stuck_since = None
            if self._escaping:
                self.get_logger().info('✅ 막힘 해소 — 정상 동작 복귀')
            self._escaping = False
        # ★ 감속(obstacle)은 언제나 내보낸다 — 안전 기능이다.
        #   조향 override 만 arm 구간에서만 내보낸다.
        if self.muted:
            obs = self.clear_distance      # '아무것도 없음' 으로 보고
            mode = f'{mode}(MUTE)'
            self.get_logger().warn('MUTE 중 — 전방 감속 없음',
                                   throttle_duration_sec=1.0)
        self.obstacle_pub.publish(Float64(data=obs))
        if not self.armed:
            steer = NO_STEER
            mode = f'{mode}(조향OFF)'
        self.avoid_steer_pub.publish(Float64(data=steer))
        self.mode_pub.publish(String(data=f'{mode}|{d.direction}'))

    def mute_cb(self, msg):
        """전방 감속 일시 해제. 조향 회피에는 영향을 주지 않는다."""
        want = bool(msg.data)
        if want == self.muted:
            return
        self.muted = want
        if want:
            self.get_logger().warn(
                '⚠ MUTE — 전방 장애물 감속을 끈다. 앞이 막혀 있어도 안 선다!')
        else:
            self.get_logger().info('MUTE 해제 — 전방 감속 복구')

    def arm_cb(self, msg):
        """arm 신호 — **조향 override 만** 켜고 끈다.

        감속(/obstacle_distance)은 안전 기능이라 arm 과 무관하게 항상 나간다.

        내려갈 때 NaN 을 한 번 쏘는 이유: 먹스는 마지막 avoid_steer 를
        타임아웃(0.3s) 동안 들고 있으므로, 조용히 멈추면 그 사이 옛 회피각으로
        조향한다. 명시적으로 '회피 없음' 을 알린다.
        """
        want = bool(msg.data)
        if want == self.armed:
            return
        self.armed = want
        if want:
            self.get_logger().info('▶ ARM — 조향 회피 시작 (감속은 원래 켜져 있다)')
        else:
            # 조향만 놓는다. /obstacle_distance 는 계속 내보낸다 —
            # 그걸 끊으면 코스 어디서든 앞을 막아도 안 서게 된다.
            self.avoid_steer_pub.publish(Float64(data=NO_STEER))
            self.get_logger().info('■ DISARM — 조향 회피만 끈다. 감속은 계속 동작')

    def scan_callback(self, msg):
        self._last_scan_t = self.get_clock().now().nanoseconds * 1e-9
        if self.window_closed:
            return

        self.frame_count += 1

        gap_decision = self.follow_gap_planner.plan(
            msg, target_deg=self._path_target_deg())

        # ★ 제어팀 통합: FollowGap 결정을 **가장 먼저** 발행한다.
        # follow_gap 은 원본 msg 로 독립 계산되므로 트래커 상태와 무관하다.
        # RPLidar A1 은 회전마다 점 개수가 달라(정상 특성) 아래 preprocessor 가
        # scan_size_changed 로 조기 return 하는데, 발행이 그 뒤에 있으면 회피/정지가
        # 전혀 안 나간다(2026-08-19 실측: 매 프레임 리셋되어 토픽 0개). 그래서 앞으로.
        self._publish_decision(gap_decision)

        result = self.preprocessor.process(msg)

        if result.scan_size_changed:
            self.tracker.reset()
            self.get_logger().warning(
                'LaserScan size changed. Tracker was reset.',
                throttle_duration_sec=5.0)
            return

        current_points = result.foreground_points
        clusters = self.clusterer.cluster(current_points)
        tracked_clusters = self.tracker.update(clusters)
        tracked_clusters = add_obstacle_features(tracked_clusters)

        if gap_decision.mode == 'NO_SCAN':
            graph_status = 'NO SCAN | STOP'
        elif gap_decision.mode == 'CLEAR':
            graph_status = 'CLEAR | STRAIGHT'
        elif gap_decision.mode == 'BLOCKED':
            graph_status = 'BLOCKED | STOP'
        else:
            graph_status = (
                f'AVOID | {gap_decision.direction} | '
                f'{gap_decision.front_distance:.2f} m'
            )

        if self.enable_plot and self.frame_count % self.draw_interval == 0:
            self.plotter.draw(
                current_points,
                tracked_clusters,
                status=graph_status,
            )

        if self.frame_count % self.obstacle_log_interval == 0:
            self.print_obstacle_information(tracked_clusters)
            self.get_logger().info(
                f'FollowGap: mode={gap_decision.mode}, '
                f'direction={gap_decision.direction}, '
                f'front={gap_decision.front_distance:.2f} m, '
                f'best_angle={gap_decision.best_angle_deg:.1f} deg'
            )

        if self.frame_count % self.status_log_interval == 0:
            self.get_logger().info(
                f'points={len(current_points)}, '
                f'clusters={len(clusters)}, '
                f'tracks={len(tracked_clusters)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ClusterPlotNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.plotter is not None:
            node.plotter.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

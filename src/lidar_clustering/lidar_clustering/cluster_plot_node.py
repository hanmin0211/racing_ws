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
        #   0 으로 두면 비활성(예전 동작 그대로).
        self.blocked_escape_s = float(
            self.declare_parameter('blocked_escape_s', 3.0).value)
        #   라이다가 죽으면(NO_SCAN) obstacle_distance=0 이라 역시 영원히 선다.
        #   센서가 나가도 GPS 주행은 되므로, 더 오래 기다린 뒤 같은 처리를 한다.
        #   (lidar:=false 로 달리는 것과 같은 상태가 될 뿐이다)
        self.no_scan_escape_s = float(
            self.declare_parameter('no_scan_escape_s', 5.0).value)
        self._stuck_since = None      # 정지 유발 모드가 시작된 시각
        self._escaping = False

        # 제어팀 통합 발행
        self.obstacle_pub = self.create_publisher(
            Float64, '/obstacle_distance', 10)
        self.avoid_steer_pub = self.create_publisher(
            Float64, '/lidar/avoid_steer', 10)
        self.mode_pub = self.create_publisher(String, '/lidar/mode', 10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
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

        self.window_closed = False
        self.frame_count = 0
        self.draw_interval = 2
        self.obstacle_log_interval = 10
        self.status_log_interval = 20

        self.get_logger().info(
            'LiDAR DBSCAN + Hungarian + Follow the Gap started.'
        )
        self.get_logger().info(
            'Vehicle coordinate: +X forward, +Y left, -Y right.'
        )

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
            steer = float(d.best_angle_deg)
        else:  # BLOCKED
            obs = float(d.front_distance)
            steer = NO_STEER

        # ★ 막힘 탈출 — 위 주석(self.blocked_escape_s) 참고.
        limit = {'BLOCKED': self.blocked_escape_s,
                 'NO_SCAN': self.no_scan_escape_s}.get(mode, 0.0)
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
                steer = NO_STEER
                mode = f'{mode}(ESCAPE)'
                self.get_logger().warn(
                    f'막힘 탈출 중 — 전방 감속 없음 ({stuck_for:.0f}s)',
                    throttle_duration_sec=1.0)
        else:
            self._stuck_since = None
        if mode.startswith(('CLEAR', 'AVOID')):
            # 길이 다시 보이면 즉시 정상 복귀
            if self._escaping:
                self.get_logger().info('✅ 막힘 해소 — 정상 동작 복귀')
            self._stuck_since = None
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
        if self.window_closed:
            return

        self.frame_count += 1

        gap_decision = self.follow_gap_planner.plan(msg)

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

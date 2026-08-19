import math

import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

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

        self.follow_gap_planner = FollowGapPlanner(
            yaw_offset_deg=180.0,
            front_fov_deg=180.0,
            min_range=0.30,
            max_range=8.0,
            track_width=3.0,
            planning_lookahead=3.0,
            obstacle_trigger_distance=3.0,
            vehicle_width=0.775,
            safety_margin=0.20,
            straight_deadband_deg=5.0,
            min_gap_width_deg=3.0,
            side_score_margin=0.20,
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
        self.obstacle_pub.publish(Float64(data=obs))
        self.avoid_steer_pub.publish(Float64(data=steer))
        self.mode_pub.publish(String(data=f'{mode}|{d.direction}'))

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

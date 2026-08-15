import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import yaml
import math

class LocalSlidingWindowNode(Node):
    def __init__(self):
        super().__init__('local_sliding_window_node')
        
        # 1. 윈도우 크기 설정 (100개 점)
        self.WINDOW_SIZE = 20
        self.global_waypoints = None

        # 2. 퍼블리셔 및 서브스크라이버 설정 (토픽 충돌 방지를 위해 /local_path 사용)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 3. 전역 경로 로드
        self.load_global_path()

    def load_global_path(self):
        # YAML/전역 경로 파일 로드 로직 (경로에 맞게 확인)
        try:
            path_file = '/home/han/racing_ws/src/pure_pursuit_pkg/config/waypoints_local_resampled_0.3.yaml'
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                raw_points = data.get('waypoints', data.get('poses', list(data.values())[0]))
                pts = []
                for pt in raw_points:
                    if isinstance(pt, dict):
                        pts.append([float(pt['x']), float(pt['y'])])
                    else:
                        pts.append([float(pt[0]), float(pt[1])])
                self.global_waypoints = np.array(pts)
                self.get_logger().info(f"✅ 전역 경로 로드 완료! 총 {len(self.global_waypoints)}개 정점 로드됨")
        except Exception as e:
            self.get_logger().error(f"전역 경로 로드 실패: {e}")

    def odom_callback(self, msg):
        if self.global_waypoints is None:
            return

        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        
        # 쿼터니언 -> 쿼터니언 변환 (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        car_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 1. 가장 가까운 전역 점 찾기
        dists = np.hypot(self.global_waypoints[:, 0] - car_x, self.global_waypoints[:, 1] - car_y)
        closest_idx = np.argmin(dists)

        # 2. 지정한 WINDOW_SIZE(100개) 만큼 전방 슬라이싱
        total_pts = len(self.global_waypoints)
        win_size = min(self.WINDOW_SIZE, total_pts)
        indices = [(closest_idx + i) % total_pts for i in range(win_size)]
        forward_global = self.global_waypoints[indices]

        # 3. base_link 기준 Local Path 생성 및 퍼블리시
        local_path_msg = Path()
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        local_path_msg.header.frame_id = 'base_link'

        poses_list = []
        for pt in forward_global:
            dx = float(pt[0]) - car_x
            dy = float(pt[1]) - car_y

            local_x =  dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
            local_y = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)

            pose = PoseStamped()
            pose.header = local_path_msg.header
            pose.pose.position.x = float(local_x)
            pose.pose.position.y = float(local_y)
            pose.pose.position.z = 0.0
            poses_list.append(pose)

        local_path_msg.poses = poses_list
        self.local_path_pub.publish(local_path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalSlidingWindowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

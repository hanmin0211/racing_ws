#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import os
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class LocalSlidingWindowNode(Node):
    def __init__(self):
        super().__init__('local_sliding_window_node')
        
        # 1. 실시간 차량 위치(Odometry) 구독
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # 2. 사람 중심(base_link) 전방 20개 지역 경로 발행
        self.local_path_pub = self.create_publisher(Path, '/vehicle_frame_path', 10)
        
        # 3. 전역 경로(Waypoints) 파일 로드 및 축 정렬 초기화
        self.global_waypoints = []
        self.load_and_align_global_path()

    def load_and_align_global_path(self):
        # 가지고 계신 yaml 파일 경로로 지정 (사용자 환경에 맞게 수정 가능)
        yaml_path = os.path.expanduser('~/racing_ws/src/pure_pursuit_pkg/config/waypoints_local_resampled_0.3.yaml')
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                # yaml 구조에 따라 'waypoints' 등의 키값을 파싱
                raw_points = data.get('waypoints', data) 
                self.global_waypoints = np.array([[pt['x'], pt['y']] for pt in raw_points])
            
            self.get_logger().info(f'전역 경로 로드 완료: 총 {len(self.global_waypoints)}개 정점')
            
            # [교수님 지시 ①] 초기 출발선 구간 데이터를 활용한 기준축 정렬
            # 0번 점과 10번 점을 이어 트랙 진행 방향을 새로운 순수 +X축으로 눕힘
            pt0 = self.global_waypoints[0]
            pt10 = self.global_waypoints[min(10, len(self.global_waypoints)-1)]
            self.origin_heading = np.arctan2(pt10[1] - pt0[1], pt10[0] - pt0[0])
            
            self.get_logger().info(f'트랙 기준축 설정 완료 (출발선 정렬 각도: {np.degrees(self.origin_heading):.2f}도)')
            
        except Exception as e:
            self.get_logger().error(f'전역 경로 파일 로드 실패: {str(e)}')

    def odom_callback(self, msg):
        if len(self.global_waypoints) == 0:
            return
            
        # 현재 차량의 글로벌 위치 추출
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        
        # 쿼터니언 방향 데이터를 오일러 Yaw 각도로 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        car_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # [교수님 지시 ②] 현재 차량 위치와 가장 가까운 전역 경로 인덱스 탐색
        distances = np.linalg.norm(self.global_waypoints - np.array([car_x, car_y]), axis=1)
        closest_idx = np.argmin(distances)
        
        # 차량이 이동하더라도 항상 전방 20개의 웨이포인트만 슬라이싱 유지 (Sliding Window)
        total_points = len(self.global_waypoints)
        if closest_idx + 20 <= total_points:
            forward_20_global = self.global_waypoints[closest_idx : closest_idx + 20]
        else:
            # 폐곡선(루프 트랙) 대비 인덱스 초과 시 처음 점들과 이어붙임 예외 처리
            remainder = (closest_idx + 20) - total_points
            forward_20_global = np.vstack((self.global_waypoints[closest_idx:], self.global_waypoints[0:remainder]))
            
        # [교수님 지시 ③] 20개 점들을 '지도(map) ➡️ 차량(base_link)' 사람 좌표계로 실시간 이원화 변환
        local_path_msg = Path()
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        local_path_msg.header.frame_id = 'base_link'  # 사람(차량) 기준 좌표축 명시
        
        for pt in forward_20_global:
            dx = pt[0] - car_x
            dy = pt[1] - car_y
            
            # 내 진행 방향이 무조건 정면 +X축, 왼쪽 90도가 +Y축이 되도록 직교 투영 회전 변환
            local_x = dx * np.cos(-car_yaw) - dy * np.sin(-car_yaw)
            local_y = dx * np.sin(-car_yaw) + dy * np.cos(-car_yaw)
            
            pose = PoseStamped()
            pose.pose.position.x = local_x
            pose.pose.position.y = local_y
            pose.pose.position.z = 0.0
            local_path_msg.poses.append(pose)
            
        # 최종 정렬된 20개 제어용 지역 경로 발행
        self.local_path_pub.publish(local_path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalSlidingWindowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

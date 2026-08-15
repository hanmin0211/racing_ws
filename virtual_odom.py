import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time
import yaml

class VirtualOdomNode(Node):
    def __init__(self):
        super().__init__('virtual_odom_publisher')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        yaml_path = '/home/han/racing_ws/build/pure_pursuit_pkg/config/waypoints_local_resampled_0.3.yaml'
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"YAML 파일 읽기 실패: {e}")
            self.waypoints = [{'x': 0.0, 'y': 0.0}]

        self.idx = 0
        self.timer = self.create_period = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"총 {len(self.waypoints)}개의 웨이포인트로 가상 주행 시작!")

    def timer_callback(self):
        if self.idx >= len(self.waypoints):
            self.idx = 0  # 경로 끝에 도달하면 반복 순환

        wp = self.waypoints[self.idx]
        x = float(wp['x'])
        y = float(wp['y'])
        now = self.get_clock().now().to_msg()

        # 1. /odom 토픽 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom_msg)

        # 2. odom -> base_link 동적 TF 발행 (이 부분이 핵심입니다!)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = VirtualOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

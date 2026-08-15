import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class DummyOdom(Node):
    def __init__(self):
        super().__init__('dummy_odom')
        self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        # 출발선 근처에서 조금씩 앞으로 이동하는 가짜 차량 위치
        msg.pose.pose.position.x = 0.0 + self.t * 0.1
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0  # Heading 0도

        self.pub.publish(msg)
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

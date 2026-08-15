import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class FakeOdom(Node):

    def __init__(self):
        super().__init__('fake_odom_publisher')

        self.pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_odom
        )

    def publish_odom(self):

        msg = Odometry()

        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        # 차량 위치
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # yaw = 0
        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0
        )

        self.pub.publish(msg)


def main():
    rclpy.init()

    node = FakeOdom()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

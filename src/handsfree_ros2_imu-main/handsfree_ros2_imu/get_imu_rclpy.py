#!/usr/bin/env python3
# coding=UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class IMUListener(Node):
    def __init__(self):
        super().__init__('get_imu')
        self.subscription = self.create_subscription(
            Imu,
            '/handsfree/imu',
            self.imu_callback,
            10  
        )
        self.subscription 

    def imu_callback(self, msg):
        r, p, y = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.get_logger().info(f"Roll = {r*180/3.1415926}, Pitch = {p*180/3.1415926}, Yaw = {y*180/3.1415926}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

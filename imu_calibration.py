#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuBiasCorrection(Node):
    def __init__(self):
        super().__init__('imu_bias_correction')

        # 캘리브레이션으로 측정한 바이어스 값 (필요시 여기만 수정하면 됩니다)
        self.gyro_bias = {
            'x': 0.000534,
            'y': -0.000229,
            'z': 0.000178
        }
        self.accel_bias = {
            'x': -0.180522,
            'y': 0.140988,
            'z': 0.0  # z는 중력이므로 보정하지 않음
        }

        self.sub = self.create_subscription(
            Imu, '/handsfree/imu', self.callback, 10
        )
        self.pub = self.create_publisher(
            Imu, '/handsfree/imu/corrected', 10
        )

        self.get_logger().info('IMU 바이어스 보정 노드 시작됨')
        self.get_logger().info(f'  자이로 바이어스: {self.gyro_bias}')
        self.get_logger().info(f'  가속도 바이어스: {self.accel_bias}')
        self.get_logger().info('구독: /handsfree/imu -> 발행: /handsfree/imu/corrected')

    def callback(self, msg: Imu):
        corrected = Imu()
        corrected.header = msg.header
        corrected.orientation = msg.orientation
        corrected.orientation_covariance = msg.orientation_covariance

        # 각속도 보정
        corrected.angular_velocity.x = msg.angular_velocity.x - self.gyro_bias['x']
        corrected.angular_velocity.y = msg.angular_velocity.y - self.gyro_bias['y']
        corrected.angular_velocity.z = msg.angular_velocity.z - self.gyro_bias['z']
        corrected.angular_velocity_covariance = msg.angular_velocity_covariance

        # 가속도 보정
        corrected.linear_acceleration.x = msg.linear_acceleration.x - self.accel_bias['x']
        corrected.linear_acceleration.y = msg.linear_acceleration.y - self.accel_bias['y']
        corrected.linear_acceleration.z = msg.linear_acceleration.z - self.accel_bias['z']
        corrected.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(corrected)

def main():
    rclpy.init()
    node = ImuBiasCorrection()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

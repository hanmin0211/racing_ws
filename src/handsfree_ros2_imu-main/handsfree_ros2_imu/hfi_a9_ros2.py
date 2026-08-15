#!/usr/bin/env python3
import serial
import struct
import math
import platform
import serial.tools.list_ports
import threading
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/HFRobotIMU'),
                ('baudrate', 921600),
                ('gra_normalization', True)
            ]
        )
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.gra_normalization = self.get_parameter('gra_normalization').value

        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'handsfree/mag', 10)
        
        self.key = 0
        self.buff = {}
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True]
        self.data_right_count = 0

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        try:
            self.hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=5.0)
            if self.hf_imu.is_open:
                self.get_logger().info(f"Serial port {port} opened successfully")
            else:
                self.hf_imu.open()
                self.get_logger().info(f"Serial port {port} opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            exit(1)

        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial(self):
        while rclpy.ok():
            try:
                if self.hf_imu.in_waiting > 0:
                    data = self.hf_imu.read_all()
                    for byte in data:
                        self.handle_serial_data(byte)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")
                exit(1)

    def check_sum(self, list_data, check_data):
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data_part = raw_data[i:i+4]
            data_bytes = bytes(data_part)
            ieee_val = struct.unpack('>f', data_bytes)[0]
            ieee_data.append(ieee_val)
        ieee_data.reverse()
        return ieee_data

    def handle_serial_data(self, raw_data):
        self.buff[self.key] = raw_data
        self.key += 1

        if self.buff.get(0, 0) != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return

        if self.key < 3:
            return

        if self.buff.get(1, 0) != 0x55:
            self.key = 0
            return

        data_length = self.buff.get(2, 0)
        if self.key < data_length + 5:
            return

        data_buff = list(self.buff.values())
        if data_length == 0x2c and self.pub_flag[0]:
            if self.check_sum(data_buff[2:47], data_buff[47:49]):
                processed_data = self.hex_to_ieee(data_buff[7:47])
                self.angularVelocity = processed_data[1:4]
                self.acceleration = processed_data[4:7]
                self.magnetometer = processed_data[7:10]
            else:
                self.get_logger().warn("IMU data checksum failed")
            self.pub_flag[0] = False
        elif data_length == 0x14 and self.pub_flag[1]:
            if self.check_sum(data_buff[2:23], data_buff[23:25]):
                processed_data = self.hex_to_ieee(data_buff[7:23])
                self.angle_degree = processed_data[1:4]
            else:
                self.get_logger().warn("Angle data checksum failed")
            self.pub_flag[1] = False
        else:
            self.get_logger().warn(f"Unhandled data type: 0x{data_length:02x}")
            self.buff = {}
            self.key = 0
            return

        self.buff = {}
        self.key = 0
        self.pub_flag = [True, True]

        roll = math.radians(self.angle_degree[0])
        pitch = math.radians(-self.angle_degree[1])
        yaw = math.radians(-self.angle_degree[2])

        q = quaternion_from_euler(roll, pitch, yaw)

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = "imu_link"
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.angular_velocity.x = self.angularVelocity[0]
        self.imu_msg.angular_velocity.y = self.angularVelocity[1]
        self.imu_msg.angular_velocity.z = self.angularVelocity[2]

        if self.gra_normalization:
            acc_k = math.sqrt(sum(x**2 for x in self.acceleration))
            acc_k = acc_k if acc_k != 0 else 1.0
            self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
            self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
            self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
        else:
            self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
            self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
            self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8

        self.mag_msg.header.stamp = self.imu_msg.header.stamp
        self.mag_msg.header.frame_id = "imu_link"
        self.mag_msg.magnetic_field.x = self.magnetometer[0]
        self.mag_msg.magnetic_field.y = self.magnetometer[1]
        self.mag_msg.magnetic_field.z = self.magnetometer[2]

        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
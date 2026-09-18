#!/usr/bin/env python3
import serial
import struct
import time
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
        # ★ 재연결용으로 보관 (2026-09-13) — 아래 read_serial 주석 참고.
        self.port = port
        self.baudrate = baudrate
        self.reconnect_count = 0
        # 이 시간 동안 유효한 바이트가 한 개도 없으면 파이프가 멈춘 것으로 본다.
        self.declare_parameter('stall_timeout', 1.5)
        self.stall_timeout = float(self.get_parameter('stall_timeout').value)
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
            # ★ exclusive=True → TIOCEXCL. 다른 프로세스가 이 포트를 여는 것을
            #   **커널이 막는다.** serial_bridge 가 /dev/arduino 에 쓰는 것과
            #   같은 방식이다(serial_bridge_node.py 의 _open_no_reset 주석).
            #
            # ★ 왜 넣었나 (2026-09-18 밤, 학교)
            #   런치를 Ctrl-C 로 내려도 이 노드가 **좀비로 살아남는 경우**가 있다.
            #   그러면 다음 런치의 IMU 노드가 같은 포트를 열고, 둘이 한 스트림을
            #   나눠 읽으며 서로의 프레임을 깨뜨린다. 증상:
            #       device reports readiness to read but returned no data
            #       (device disconnected or **multiple access on port**?)
            #   재연결이 반복되다 결국 노드가 exit 1 로 죽는다. 그날 세 세션이
            #   각각 122·127·235초 만에 이렇게 죽었고, 단일 인스턴스일 때는
            #   330초 넘게 멀쩡했다.
            #
            #   이게 왜 위험한가: IMU 가 죽으면 0.5초 뒤 serial_bridge 의
            #   _grade_pwm() 이 0 을 반환해 **경사 보상이 조용히 사라진다**
            #   (GRADE_FRESH_S). 경사로 한가운데서 이러면 그대로 못 올라간다.
            #   그리고 direct_localization 의 yaw 도 같이 얼어붙는다.
            #
            #   exclusive=True 면 **두 번째 노드가 즉시 열기에 실패**한다.
            #   2분 뒤 주행 중에 죽는 대신, 띄우는 순간 큰 소리로 실패한다.
            self.hf_imu = serial.Serial(port=port, baudrate=baudrate,
                                        timeout=5.0, exclusive=True)
            if not self.hf_imu.is_open:
                self.hf_imu.open()
            self.get_logger().info(f"Serial port {port} opened successfully")
        except Exception as e:
            self.get_logger().error(
                f'❌ IMU 시리얼 열기 실패: {port} — {e}\n'
                '   가장 흔한 원인은 **IMU 노드가 이미 떠 있는 것**이다.\n'
                '     ps -eo pid,etimes,args | grep [h]fi_a9_ros2\n'
                '   남아 있으면 정리하고 다시 띄울 것:\n'
                '     bash tools/ros_cleanup.sh\n'
                '   포트 자체가 없으면 udev 를 볼 것 (라이다와 같은 CP2102 라\n'
                '   시리얼 0001 로 구분한다 — /etc/udev/rules.d/99-imu.rules)')
            raise SystemExit(1)

        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial(self):
        """IMU 시리얼 읽기 — **끊겨도 스스로 복구한다.**

        ★ 왜 고쳤나 (2026-09-13 밤, 학교)
          주행 중 IMU 가 끊기면 direct_localization 이 위치 발행을 멈추고,
          로컬경로가 사라져 **차가 그 자리에 선다.** 대회에서 1분 이상
          정지는 탈락이다. 그날 밤 주행마다 다른 장치가 돌아가며 끊겼고,
          IMU 가 끊긴 판은 매번 거기서 끝났다.

          원래 코드에는 결함이 둘 있었다:
            ① `except: exit(1)` — 읽기 오류 한 번에 노드가 죽는다.
            ② **파이프가 멈추면 예외가 안 난다.** CP210x 가
               `urb stopped: -32` 로 멈추면 in_waiting 이 영원히 0 이고
               read 도 예외를 안 던진다. 노드는 "opened successfully" 를
               찍어 놓고 조용히 아무것도 안 읽는다 — 그날 그 상태로
               한참을 헤맸다. 겉으로는 멀쩡해 보여서 더 나쁘다.

          그래서 예외뿐 아니라 **무데이터 시간**으로도 끊김을 판정하고,
          포트를 닫았다 다시 연다. /dev/imu 는 udev 심볼릭 링크라 장치가
          다른 ttyUSB 로 재열거돼도 같은 경로로 다시 잡힌다.
          (serial_bridge 가 /dev/arduino 에 대해 하는 것과 같은 방식이다)
        """
        last_data = time.time()
        while rclpy.ok():
            try:
                if self.hf_imu.in_waiting > 0:
                    data = self.hf_imu.read_all()
                    if data:
                        last_data = time.time()
                        for byte in data:
                            self.handle_serial_data(byte)
                else:
                    # 바쁜 대기를 막는다(원래는 CPU 를 한 코어 다 먹었다).
                    time.sleep(0.002)

                if time.time() - last_data > self.stall_timeout:
                    self._reconnect(f'{self.stall_timeout:.1f}초간 데이터 없음')
                    last_data = time.time()
            except Exception as e:  # noqa: BLE001
                self._reconnect(f'읽기 예외: {e}')
                last_data = time.time()

    def _reconnect(self, why):
        """포트를 닫았다 다시 연다. 실패해도 죽지 않고 계속 재시도한다."""
        self.reconnect_count += 1
        self.get_logger().warn(
            f'⚠ IMU 시리얼 끊김 ({why}) — 재연결 시도 {self.reconnect_count}회: '
            f'{self.port}')
        try:
            self.hf_imu.close()
        except Exception:  # noqa: BLE001
            pass
        for _ in range(20):
            if not rclpy.ok():
                return
            time.sleep(0.3)
            try:
                # ★ 재연결에도 exclusive=True. 여기를 빠뜨리면 첫 열기에서만
                #   막히고, 한 번 끊긴 뒤에는 좀비가 다시 끼어들 수 있다.
                self.hf_imu = serial.Serial(
                    port=self.port, baudrate=self.baudrate, timeout=5.0,
                    exclusive=True)
                self.get_logger().info(
                    f'✅ IMU 시리얼 재연결 성공: {self.port} '
                    f'(총 {self.reconnect_count}회)')
                return
            except Exception:  # noqa: BLE001
                continue
        self.get_logger().error(
            f'❌ IMU 재연결 6초간 실패 — USB 를 뺐다 꽂을 것 ({self.port}). '
            '계속 시도한다.')

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
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_node.py
==============
USB 카메라(Logitech C920) → ROS 이미지 토픽.

★ 왜 image_tools/cam2image 를 안 쓰는가
  cam2image 는 FOURCC 를 설정할 수 없어서 OpenCV 기본 백엔드(GStreamer)로
  열리고, 1280x720 에서 **9 fps** 밖에 안 나온다(YUYV 무압축이라 USB 대역폭에
  막힘). V4L2 백엔드를 강제하고 MJPG 로 잡으면 같은 해상도에서 **30 fps** 다.
  2026-08-16 실측:

      기본 백엔드          1280x720   9.1 fps
      V4L2 + MJPG          1280x720  30.0 fps   ← 이 노드
      V4L2 + MJPG           640x480  30.0 fps

  이 한 줄(FOURCC) 차이가 3배다. 카메라를 바꿔도 이 설정은 유지할 것.

★ 발행 주기(publish_rate)를 캡처와 분리한 이유
  raw bgr8 1280x720@30 은 약 82MB/s 다. 이 노트북은 rviz2 만으로도 CPU 112% 를
  찍은 적이 있고(HANDOFF 함정 #5), 그때 토픽이 간헐 실패해 오진으로 이어졌다.
  차량이 0.4m/s 로 기므로 15fps 면 프레임당 2.7cm — 인지에 차고 넘친다.
  캡처는 30fps 로 최신 프레임을 유지하되 발행만 줄여 대역폭을 절반으로 깎는다.

파라미터:
  device            : 카메라 인덱스 (기본 2 = C920. 0은 노트북 내장)
  width, height     : 캡처 해상도 (기본 1280x720)
  fourcc            : 픽셀 포맷 (기본 MJPG — 이걸 바꾸면 fps 가 떨어진다)
  publish_rate      : 발행 Hz (기본 15.0)
  frame_id          : 이미지 frame_id (기본 camera)
  flip              : 상하좌우 반전 (마운트가 뒤집혔을 때, 기본 false)

출력:
  /camera/image_raw   sensor_msgs/Image (bgr8)
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):

  def __init__(self):
    super().__init__('camera_node')

    self.declare_parameter('device', 2)
    self.declare_parameter('width', 1280)
    self.declare_parameter('height', 720)
    self.declare_parameter('fourcc', 'MJPG')
    self.declare_parameter('publish_rate', 15.0)
    self.declare_parameter('frame_id', 'camera')
    self.declare_parameter('flip', False)

    g = lambda n: self.get_parameter(n).value  # noqa: E731
    self.device = int(g('device'))
    width, height = int(g('width')), int(g('height'))
    fourcc = str(g('fourcc'))
    rate = float(g('publish_rate'))
    self.frame_id = str(g('frame_id'))
    self.flip = bool(g('flip'))

    # ★ CAP_V4L2 를 명시하지 않으면 GStreamer 로 열려 fps 가 1/3 로 떨어진다.
    self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
    if not self.cap.isOpened():
      self.get_logger().error(
          f'카메라 /dev/video{self.device} 를 열 수 없다. '
          f'`ls /dev/video*` 로 인덱스 확인, 다른 프로세스가 잡고 있는지도 확인할 것.')
      raise RuntimeError('camera open failed')

    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # 버퍼를 1로 줄여 지연을 억제 (드라이버가 무시할 수도 있음)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok, frame = self.cap.read()
    if not ok or frame is None:
      raise RuntimeError('첫 프레임 수신 실패')
    # CAP_PROP_FRAME_* 는 거짓말을 하는 경우가 있어(2026-08-16 실측: 요청과
    # 무관하게 2304x1536 을 보고했다) 실제 프레임 shape 을 신뢰한다.
    h, w = frame.shape[:2]
    if (w, h) != (width, height):
      self.get_logger().warn(
          f'요청 {width}x{height} 이지만 실제 {w}x{h} 로 잡혔다. '
          f'카메라가 지원하는 조합인지 확인할 것.')

    self.bridge = CvBridge()
    self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
    self.timer = self.create_timer(1.0 / rate, self.tick)

    self.n_pub = 0
    self.n_fail = 0
    self.create_timer(5.0, self.report)

    self.get_logger().info(
        f'카메라 시작: /dev/video{self.device} {w}x{h} {fourcc} → '
        f'/camera/image_raw @{rate:.0f}Hz (frame_id={self.frame_id})')

  def tick(self):
    ok, frame = self.cap.read()
    if not ok or frame is None:
      self.n_fail += 1
      return
    if self.flip:
      frame = cv2.flip(frame, -1)
    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.frame_id
    self.pub.publish(msg)
    self.n_pub += 1

  def report(self):
    # 카메라가 조용히 죽는 경우(USB 빠짐 등)를 눈에 띄게 한다.
    if self.n_fail:
      self.get_logger().warn(
          f'프레임 수신 실패 {self.n_fail}회 / 발행 {self.n_pub}회 (최근 5초). '
          f'USB 연결·전원 확인.')
    self.n_pub = self.n_fail = 0

  def destroy_node(self):
    if hasattr(self, 'cap') and self.cap is not None:
      self.cap.release()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = None
  try:
    node = CameraNode()
    rclpy.spin(node)
  except (KeyboardInterrupt, RuntimeError):
    pass
  finally:
    if node is not None:
      node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vrs_ntrip_client.py
===================
NGII 실시간(VRS) NTRIP 클라이언트 — GGA 전송 지원 버전.

ublox_dgnss 패키지에 기본 포함된 ntrip_client_node 는 GGA(NMEA)를 캐스터로
보내지 못한다. 그런데 NGII VRS는 rover의 대략 위치를 GGA로 받아야 그 근처의
'가상 기준국' 보정신호(RTCM)를 내려준다. 그래서 이 노드가 그 역할을 한다.

  1. NGII 캐스터(RTS2.ngii.go.kr:2101)의 VRS 마운트포인트에 Basic 인증으로 접속
  2. rover 위치(GGA)를 주기적으로 캐스터로 전송
     - /fix(NavSatFix)가 들어오면 그 실측 위치를, 아직 없으면 파라미터의
       고정 위치(트랙 근방)를 사용
  3. 수신한 RTCM3 스트림을 프레임 단위로 잘라 rtcm_msgs/Message 로
     /ntrip_client/rtcm 에 발행 → ublox_dgnss_node 가 이를 F9P(USB)에 주입
     → F9P가 RTK Fixed(cm급)로 수렴

파라미터:
  host, port, mountpoint, username, password : NGII 접속 정보
  lat, lon, height : /fix 수신 전까지 쓸 고정 GGA 위치(트랙 근방)
  gga_interval     : GGA 전송 주기[s] (기본 1.0)
  rtcm_topic       : RTCM 발행 토픽 (기본 /ntrip_client/rtcm)
"""

import base64
import os
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtcm_msgs.msg import Message as RTCM
from sensor_msgs.msg import NavSatFix


def _nmea_checksum(body: str) -> int:
  c = 0
  for ch in body:
    c ^= ord(ch)
  return c


def build_gga(lat: float, lon: float, hgt: float) -> str:
  """주어진 위경도/높이로 $GPGGA 문장을 만든다 (VRS 캐스터 전송용)."""
  hhmmss = time.strftime('%H%M%S', time.gmtime()) + '.00'
  ns = 'N' if lat >= 0 else 'S'
  ew = 'E' if lon >= 0 else 'W'
  la, lo = abs(lat), abs(lon)
  lad, lod = int(la), int(lo)
  lam, lom = (la - lad) * 60.0, (lo - lod) * 60.0
  body = (f'GPGGA,{hhmmss},{lad:02d}{lam:09.6f},{ns},'
          f'{lod:03d}{lom:09.6f},{ew},1,12,1.0,{hgt:.1f},M,0.0,M,,')
  return f'${body}*{_nmea_checksum(body):02X}\r\n'


class VrsNtripClient(Node):

  def __init__(self):
    super().__init__('ngii_vrs_ntrip_client')

    self.declare_parameter('host', 'RTS2.ngii.go.kr')
    self.declare_parameter('port', 2101)
    self.declare_parameter('mountpoint', 'VRS-RTCM32')
    self.declare_parameter('username', os.environ.get('NGII_ID', ''))
    self.declare_parameter('password', os.environ.get('NGII_PW', ''))
    self.declare_parameter('lat', 36.9706)     # 트랙 근방 고정 위치(충주)
    self.declare_parameter('lon', 127.8748)
    self.declare_parameter('height', 100.0)
    self.declare_parameter('gga_interval', 1.0)
    self.declare_parameter('rtcm_topic', '/ntrip_client/rtcm')

    self.host = self.get_parameter('host').value
    self.port = int(self.get_parameter('port').value)
    self.mount = self.get_parameter('mountpoint').value
    self.user = self.get_parameter('username').value
    self.pw = self.get_parameter('password').value
    self.lat = float(self.get_parameter('lat').value)
    self.lon = float(self.get_parameter('lon').value)
    self.hgt = float(self.get_parameter('height').value)
    self.gga_interval = float(self.get_parameter('gga_interval').value)
    rtcm_topic = self.get_parameter('rtcm_topic').value

    self.have_fix = False
    self.sock = None
    self.running = True
    self.rtcm_count = 0

    self.pub = self.create_publisher(RTCM, rtcm_topic, 10)
    self.create_subscription(NavSatFix, '/fix', self.fix_cb,
                             qos_profile_sensor_data)
    self.create_timer(5.0, self.status_timer)

    self.get_logger().info(
        f'NGII VRS NTRIP 클라이언트 시작: {self.host}:{self.port}/{self.mount} '
        f'(id={self.user}) → {rtcm_topic}')

    self.thread = threading.Thread(target=self.run_loop, daemon=True)
    self.thread.start()

  def fix_cb(self, msg: NavSatFix):
    # /fix가 들어오면(단독측위여도) 그 실측 위치로 GGA 갱신
    if not math.isnan(msg.latitude) and abs(msg.latitude) > 1e-6:
      self.lat = msg.latitude
      self.lon = msg.longitude
      self.hgt = 0.0 if math.isnan(msg.altitude) else msg.altitude
      self.have_fix = True

  def status_timer(self):
    state = '연결됨' if self.sock else '재연결중'
    src = '실측' if self.have_fix else '고정'
    self.get_logger().info(
        f'[NTRIP {state}] RTCM {self.rtcm_count}프레임 발행, '
        f'GGA {src}({self.lat:.5f},{self.lon:.5f})')

  def connect(self):
    try:
      s = socket.create_connection((self.host, self.port), timeout=10)
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(f'캐스터 연결 실패: {e}')
      return None
    auth = base64.b64encode(f'{self.user}:{self.pw}'.encode()).decode()
    req = (f'GET /{self.mount} HTTP/1.1\r\n'
           f'Host: {self.host}:{self.port}\r\n'
           f'Ntrip-Version: Ntrip/2.0\r\n'
           f'User-Agent: NTRIP ngii_ros/1.0\r\n'
           f'Authorization: Basic {auth}\r\n'
           f'Connection: close\r\n\r\n')
    try:
      s.sendall(req.encode())
      s.settimeout(10)
      resp = b''
      while b'\r\n\r\n' not in resp and len(resp) < 4096:
        chunk = s.recv(1)
        if not chunk:
          break
        resp += chunk
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(f'응답 수신 실패: {e}')
      s.close()
      return None
    status_line = resp.split(b'\r\n', 1)[0].decode(errors='ignore')
    if '200' not in status_line:
      self.get_logger().error(f'NTRIP 인증/연결 실패: {status_line}')
      s.close()
      return None
    self.get_logger().info(f'NTRIP 연결 성공: {status_line}')
    return s

  def run_loop(self):
    buf = bytearray()
    last_gga = 0.0
    while self.running and rclpy.ok():
      if self.sock is None:
        self.sock = self.connect()
        if self.sock is None:
          time.sleep(3)
          continue
        self.sock.settimeout(1.0)
        self.send_gga()          # 접속 직후 GGA 즉시 1회
        last_gga = time.time()
        buf.clear()
      try:
        data = self.sock.recv(4096)
        if not data:
          raise ConnectionError('stream closed')
        buf.extend(data)
        self.extract_rtcm(buf)
      except socket.timeout:
        pass
      except Exception as e:  # noqa: BLE001
        self.get_logger().warn(f'스트림 끊김, 재연결: {e}')
        try:
          self.sock.close()
        except Exception:  # noqa: BLE001
          pass
        self.sock = None
        time.sleep(2)
        continue
      if time.time() - last_gga >= self.gga_interval:
        self.send_gga()
        last_gga = time.time()

  def send_gga(self):
    if self.sock is None:
      return
    try:
      self.sock.sendall(build_gga(self.lat, self.lon, self.hgt).encode())
    except Exception as e:  # noqa: BLE001
      self.get_logger().warn(f'GGA 전송 실패: {e}')

  def extract_rtcm(self, buf: bytearray):
    """RTCM3 프레임(0xD3, 길이 10bit, 페이로드, CRC 3byte)을 잘라 발행."""
    while len(buf) >= 3:
      if buf[0] != 0xD3:
        buf.pop(0)
        continue
      length = ((buf[1] & 0x03) << 8) | buf[2]
      frame_len = 3 + length + 3
      if len(buf) < frame_len:
        break
      frame = bytes(buf[:frame_len])
      del buf[:frame_len]
      msg = RTCM()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = 'gps'
      msg.message = list(frame)
      self.pub.publish(msg)
      self.rtcm_count += 1

  def destroy_node(self):
    self.running = False
    if self.sock:
      try:
        self.sock.close()
      except Exception:  # noqa: BLE001
        pass
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = VrsNtripClient()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

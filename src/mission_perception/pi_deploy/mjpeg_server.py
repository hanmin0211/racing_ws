#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mjpeg_server.py — 검출 결과를 **브라우저로** 본다 (파이에서 실행).

★ 왜 이게 필요한가 (2026-09-17)
  카메라 각도를 맞추려면 화면을 봐야 하는데, 카메라는 검출기 한 프로세스만
  열 수 있다. rqt_image_view 는 따로 설치해야 하고 원격 X 가 느리다.
  검출기가 이미 /traffic_light_debug/compressed 로 JPEG 를 쏘고 있으니,
  그걸 받아 HTTP 로 중계만 하면 브라우저에서 바로 보인다.

  ⚠ **검출기를 건드리지 않는다.** 이 노드는 순수 구독자다. 죽어도 주행·검출에
    아무 영향이 없다. 그래서 대회 당일에도 안심하고 띄울 수 있다.

전제:
  검출기를 publish_image 켜고 띄울 것.
      ros2 run traffic_light_detector coco_traffic_light_node \\
          --ros-args -p publish_image:=true

사용 (파이에서):
    python3 mjpeg_server.py                 # 0.0.0.0:8080
    python3 mjpeg_server.py --port 8081
    python3 mjpeg_server.py --host 192.168.99.8   # 직결 링크에만 연다

보기 (노트북 브라우저):
    http://192.168.99.8:8080/

  ⚠ 인증이 없다. 사설 직결 링크(192.168.99.0/24)용이다. 파이가 공용 와이파이에
    같이 붙어 있으면 --host 로 직결 IP 에만 묶을 것.
"""
import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

BOUNDARY = 'frame'
# 프레임이 이보다 오래되면 '검출기가 멈췄다' 로 본다 [s]
STALE_S = 3.0

_latest = {'jpeg': None, 't': 0.0, 'n': 0}
_lock = threading.Lock()

PAGE = """<!doctype html><meta charset="utf-8">
<title>traffic light debug</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif}
 header{padding:8px 12px;background:#1b1b1b;border-bottom:1px solid #333}
 img{display:block;max-width:100%;height:auto;margin:0 auto}
 code{color:#8bd}
</style>
<header>
 <b>traffic light debug</b> &nbsp;
 <code>/traffic_light_debug/compressed</code> &nbsp;
 <span id=s></span>
</header>
<img src="/stream" alt="stream">
<script>
setInterval(async()=>{
  try{
    const r = await fetch('/status'); const j = await r.json();
    document.getElementById('s').textContent =
      j.ok ? `프레임 ${j.n} · ${j.age.toFixed(1)}s 전` : '프레임 없음 — 검출기가 publish_image:=true 인지 확인';
  }catch(e){}
}, 1000);
</script>
""".encode('utf-8')


class Relay(Node):
  def __init__(self):
    super().__init__('traffic_light_mjpeg')
    self.create_subscription(
        CompressedImage, '/traffic_light_debug/compressed', self.cb, 2)
    self.get_logger().info(
        '구독: /traffic_light_debug/compressed — 검출기를 '
        'publish_image:=true 로 띄워야 프레임이 온다')

  def cb(self, msg):
    with _lock:
      _latest['jpeg'] = bytes(msg.data)
      _latest['t'] = time.time()
      _latest['n'] += 1


def snapshot():
  with _lock:
    return _latest['jpeg'], _latest['t'], _latest['n']


class Handler(BaseHTTPRequestHandler):
  protocol_version = 'HTTP/1.0'

  def log_message(self, *a):        # 접속 로그로 콘솔을 어지럽히지 않는다
    pass

  def _send(self, code, ctype, body=b'', extra=None):
    self.send_response(code)
    self.send_header('Content-Type', ctype)
    if body:
      self.send_header('Content-Length', str(len(body)))
    for k, v in (extra or {}).items():
      self.send_header(k, v)
    self.end_headers()
    if body:
      self.wfile.write(body)

  def do_GET(self):
    path = self.path.split('?')[0]
    if path in ('/', '/index.html'):
      self._send(200, 'text/html; charset=utf-8', PAGE)
      return
    if path == '/status':
      _, t, n = snapshot()
      age = time.time() - t if t else 9e9
      ok = 'true' if (t and age < STALE_S) else 'false'
      body = ('{"ok":%s,"n":%d,"age":%.2f}' % (ok, n, min(age, 9999))).encode()
      self._send(200, 'application/json', body,
                 {'Cache-Control': 'no-store'})
      return
    if path in ('/snapshot.jpg', '/snapshot'):
      jpeg, t, _ = snapshot()
      if not jpeg:
        self._send(503, 'text/plain; charset=utf-8',
                   '아직 프레임이 없다. 검출기를 publish_image:=true 로 '
                   '띄웠는지 확인할 것.'.encode())
        return
      self._send(200, 'image/jpeg', jpeg, {'Cache-Control': 'no-store'})
      return
    if path == '/stream':
      self.stream()
      return
    self._send(404, 'text/plain', b'not found')

  def stream(self):
    self.send_response(200)
    self.send_header('Age', '0')
    self.send_header('Cache-Control', 'no-cache, private')
    self.send_header('Pragma', 'no-cache')
    self.send_header('Content-Type',
                     f'multipart/x-mixed-replace; boundary={BOUNDARY}')
    self.end_headers()
    last_n = -1
    try:
      while True:
        jpeg, _, n = snapshot()
        if jpeg is None or n == last_n:
          # 새 프레임이 없으면 잠깐 쉰다 (같은 그림을 다시 보내지 않는다)
          time.sleep(0.02)
          continue
        last_n = n
        self.wfile.write(b'--' + BOUNDARY.encode() + b'\r\n')
        self.wfile.write(b'Content-Type: image/jpeg\r\n')
        self.wfile.write(
            b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n')
        self.wfile.write(jpeg)
        self.wfile.write(b'\r\n')
    except (BrokenPipeError, ConnectionResetError):
      pass          # 브라우저가 탭을 닫은 것 — 정상이다


def local_ips():
  ips = []
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('192.168.99.1', 1))
    ips.append(s.getsockname()[0])
    s.close()
  except OSError:
    pass
  return ips


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--host', default='0.0.0.0')
  p.add_argument('--port', type=int, default=8080)
  a = p.parse_args()

  rclpy.init()
  node = Relay()
  th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  th.start()

  srv = ThreadingHTTPServer((a.host, a.port), Handler)
  srv.daemon_threads = True
  print()
  print('=' * 58)
  print(' 검출 디버그 영상 — 브라우저로 열 것')
  for ip in local_ips() or [a.host]:
    print(f'   http://{ip}:{a.port}/')
  print('=' * 58)
  print(' 검출기가 publish_image:=true 여야 프레임이 온다.')
  print(' Ctrl-C 로 종료. (이 서버가 죽어도 검출·주행에는 영향 없다)')
  print()
  try:
    srv.serve_forever()
  except KeyboardInterrupt:
    print('\n종료')
  finally:
    srv.shutdown()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

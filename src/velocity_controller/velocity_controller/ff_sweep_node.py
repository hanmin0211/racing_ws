#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ff_sweep_node.py — 구동 FF(피드포워드) 재식별.

펌웨어의 개루프 모드(`PWM:x`)로 PWM을 **아주 천천히 올리며** 주행하고,
그때 실제로 나오는 속도를 기록해 다음 관계를 직접 측정한다.

    PWM = STATIC_FF + VELOCITY_FF_GAIN × v

지금 펌웨어의 STATIC_FF=35, VELOCITY_FF_GAIN=60 은 근거가 불분명하고, 실제로
무부하 벤치에서 5배 과다한 것이 확인됐다(0.3 명령 → 0.58~0.78 m/s 오버슈트).
FF가 틀리면 PID가 그걸 억지로 메우느라 게인이 비정상적으로 커진다.

★ PWM 램프를 느리게(기본 4 PWM/s) 올리므로 각 지점이 거의 정상상태(quasi-static)라
  개루프인데도 신뢰할 만한 (PWM, 속도) 쌍이 얻어진다.

안전:
  · 목표 속도(max_speed)에 도달하거나 PWM 상한에 닿으면 즉시 정지
  · 스톨가드·워치독은 펌웨어에서 그대로 동작
  · Ctrl-C 시 PWM 0 발행 후 결과 요약

사용 (개활지, 직선 30m 이상, 바퀴 접지):
  ros2 run velocity_controller ff_sweep
  ros2 run velocity_controller ff_sweep --ros-args -p max_speed:=1.2 -p ramp_rate:=3.0

전제: serial_bridge 가 떠 있어야 한다(그 노드가 시리얼을 소유).
      이 노드는 /drive_pwm_cmd 로 개루프 PWM을 요청한다.
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class FFSweep(Node):

  def __init__(self):
    super().__init__('ff_sweep')
    self.declare_parameter('ramp_rate', 4.0)     # PWM/초 — 느릴수록 정확
    # 펌웨어 MAX_OPENLOOP_PWM(230)과 맞춘다. 여기가 낮으면 쓰려는 속도 범위를
    # 다 못 덮고 결국 외삽하게 된다 — FF 가 10배 틀렸던 원인이 그 외삽이었다.
    self.declare_parameter('max_pwm', 230)
    self.declare_parameter('max_speed', 1.2)     # 이 속도 넘으면 종료 [m/s]
    self.declare_parameter('start_pwm', 10)
    self.declare_parameter('settle_speed', 0.05)  # 이 이하는 '정지'로 간주
    # ★ 지면 접지 검증 — 아래 '경위' 참고
    self.declare_parameter('require_rtk', True)
    self.declare_parameter('rtk_tolerance', 0.35)  # 엔코더 대비 허용 상대오차

    self.rate = float(self.get_parameter('ramp_rate').value)
    self.max_pwm = int(self.get_parameter('max_pwm').value)
    self.max_speed = float(self.get_parameter('max_speed').value)
    self.pwm = float(self.get_parameter('start_pwm').value)
    self.settle = float(self.get_parameter('settle_speed').value)
    self.require_rtk = bool(self.get_parameter('require_rtk').value)
    self.rtk_tol = float(self.get_parameter('rtk_tolerance').value)

    self.speed = 0.0
    self.samples = []          # (pwm, speed)
    self.done = False
    self.done_time = None
    self.abort_reason = None

    # RTK 교차검증 상태
    self.rtk_speed = 0.0
    self.last_xy = None
    self.last_odom_t = None
    self.enc_travel = 0.0      # 엔코더 적분 이동거리
    self.rtk_travel = 0.0      # RTK 적분 이동거리

    self.pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
    self.create_subscription(Float64, '/current_speed', self.speed_cb, 10)
    self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
    self.dt = 0.05
    self.create_timer(self.dt, self.tick)

    self.get_logger().info(
        f'FF 스윕 시작: PWM {self.pwm:.0f}부터 {self.rate:.1f}/초로 상승, '
        f'속도 {self.max_speed}m/s 또는 PWM {self.max_pwm} 도달 시 종료')
    if self.require_rtk:
      self.get_logger().warn(
          '지면 접지 검증 ON — RTK 이동거리가 엔코더와 '
          f'{self.rtk_tol * 100:.0f}% 이상 어긋나면 즉시 중단한다. '
          '(바퀴가 떠 있으면 FF가 10배 틀리게 나온다)')

  def speed_cb(self, msg):
    self.speed = abs(float(msg.data))
    self.enc_travel += self.speed * self.dt

  def odom_cb(self, msg):
    t = self.get_clock().now().nanoseconds * 1e-9
    xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    if self.last_xy is not None and self.last_odom_t is not None:
      d = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
      dt = t - self.last_odom_t
      if d > 0.01:             # RTK 지터 무시
        self.rtk_travel += d
      if dt > 1e-3:
        self.rtk_speed = d / dt
    self.last_xy = xy
    self.last_odom_t = t

  def check_grounded(self):
    """엔코더가 1m 이상 갔다고 할 때부터 RTK와 대조한다.

    2026-08-16: 바퀴가 접지되지 않은 상태로 스윕이 돌아 PWM 62에서 1.15 m/s
    라는 값이 기록됐고, 그걸로 뽑은 VELOCITY_FF_GAIN=20.7 을 펌웨어에 넣었다.
    실제 지면 기울기는 200 이상이라 차가 목표속도의 60% 밖에 못 냈다.
    개루프 스윕은 검증할 다른 수단이 없으므로 여기서 반드시 막아야 한다.
    """
    if not self.require_rtk or self.enc_travel < 1.0:
      return True
    if self.last_xy is None:
      self.abort_reason = ('/odometry/filtered 수신 없음 — RTK 없이는 접지를 '
                           '확인할 수 없다 (실내라면 require_rtk:=false)')
      return False
    ratio = self.rtk_travel / max(1e-6, self.enc_travel)
    if abs(ratio - 1.0) > self.rtk_tol:
      self.abort_reason = (
          f'엔코더 {self.enc_travel:.1f}m vs RTK {self.rtk_travel:.1f}m '
          f'(비 {ratio:.2f}) — 바퀴가 헛돌거나 위치가 안 잡힌다. '
          '차량을 지면에 내리고 다시 실행할 것')
      return False
    return True

  def tick(self):
    if self.done:
      self.pub.publish(Int32(data=0))
      # 정지 명령을 1초간 보낸 뒤 스스로 종료 → 요약이 자동 출력된다.
      # (예전엔 Ctrl-C 해야 결과가 나왔고, 남은 노드가 다음 실행과 충돌했다)
      if self.done_time is not None and (time.time() - self.done_time) > 1.0:
        raise SystemExit
      return

    if not self.check_grounded():
      self.get_logger().error(f'중단: {self.abort_reason}')
      self.done = True
      self.done_time = time.time()
      self.pub.publish(Int32(data=0))
      return

    self.pwm += self.rate * self.dt
    if self.pwm >= self.max_pwm or self.speed >= self.max_speed:
      self.get_logger().info(
          f'종료 조건 도달 (PWM {self.pwm:.0f}, 속도 {self.speed:.2f}m/s) → 정지')
      self.done = True
      self.done_time = time.time()
      self.pub.publish(Int32(data=0))
      return

    self.pub.publish(Int32(data=int(round(self.pwm))))
    # 실제로 움직이는 구간만 식별에 쓴다(정지 구간은 FF 직선과 무관)
    if self.speed > self.settle:
      self.samples.append((self.pwm, self.speed))

  def summary(self):
    print('\n' + '=' * 60)
    print('구동 FF 식별 결과')
    print('-' * 60)
    if self.abort_reason:
      print(f'  ❌ 접지 검증 실패로 중단 — 결과 폐기')
      print(f'     {self.abort_reason}')
      print('=' * 60)
      return
    print(f'  이동거리 대조 : 엔코더 {self.enc_travel:.1f}m / '
          f'RTK {self.rtk_travel:.1f}m')
    if len(self.samples) < 10:
      print(f'  샘플 부족({len(self.samples)}개) — 결과 없음')
      print('  차량이 실제로 움직였는지, /current_speed 가 오는지 확인')
      print('=' * 60)
      return

    import numpy as np
    a = np.array(self.samples)
    pwm, v = a[:, 0], a[:, 1]
    # PWM = STATIC_FF + GAIN * v  (최소자승 1차 피팅)
    gain, static_ff = np.polyfit(v, pwm, 1)
    pred = static_ff + gain * v
    rms = float(np.sqrt(np.mean((pwm - pred) ** 2)))

    print(f'  샘플 {len(a)}개, 속도 {v.min():.2f}~{v.max():.2f} m/s, '
          f'PWM {pwm.min():.0f}~{pwm.max():.0f}')
    print('-' * 60)
    print(f'  ★ STATIC_FF          = {static_ff:.1f}   (현재 40.5)')
    print(f'  ★ VELOCITY_FF_GAIN   = {gain:.1f}   (현재 20.7)')
    print(f'     피팅 잔차(RMS)     = {rms:.1f} PWM')
    if gain < 60.0:
      print()
      print('  ⚠ 기울기가 60 미만이다. 지면 주행이라면 150~300 이 정상 범위다.')
      print('    이렇게 작게 나오면 바퀴가 거의 무부하로 돌았다는 뜻 —')
      print('    이 값을 펌웨어에 넣지 말 것.')
    print()
    # 목표 속도별 필요 PWM → MAX_DRIVE_PWM 권장치
    for tgt in (0.5, 1.0, 1.5):
      need = static_ff + gain * tgt
      print(f'     {tgt:.1f} m/s 에 필요한 PWM ≈ {need:.0f}')
    need_max = static_ff + gain * self.max_speed
    rec = int(min(255, need_max * 1.4 + 20))
    print()
    print(f'  ★ MAX_DRIVE_PWM 권장 = {rec}  (최대속도 필요분 + PID 여유)')
    print('     현재 111. 이 값보다 낮으면 목표속도에 물리적으로 도달 불가.')
    print()
    print('  적용: henes_firmware.ino 의')
    print(f'     const float STATIC_FF = {static_ff:.1f}, '
          f'VELOCITY_FF_GAIN = {gain:.1f};')
    print(f'     #define MAX_DRIVE_PWM {rec}')
    print()
    # PID는 플랜트 기울기에 비례해야 한다. gain 이 PWM/(m/s) 이므로
    # kp 를 그 30% 정도로 두면 0.1m/s 오차에 gain*0.03 PWM 이 붙는다.
    print(f'  ★ PID 권장 (플랜트 기울기 {gain:.0f} 기준)')
    print(f'     velocity_kp = {gain * 0.30:.0f}, '
          f'velocity_ki = {gain * 0.40:.0f}, velocity_kd = {gain * 0.02:.1f}')
    print(f'     적분 클램프(ts constrain) = ±{max(0.5, 60.0 / max(1.0, gain * 0.40)):.1f}')
    print('     ※ kp/ki 는 플랜트 기울기에 비례해야 한다. 기울기가 200인데')
    print('       kp=8 이면 0.1m/s 오차에 0.8 PWM — 사실상 P가 없는 것과 같다.')
    print('=' * 60)


def main(args=None):
  rclpy.init(args=args)
  node = FFSweep()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      node.pub.publish(Int32(data=0))
    except Exception:  # noqa: BLE001
      pass
    node.summary()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

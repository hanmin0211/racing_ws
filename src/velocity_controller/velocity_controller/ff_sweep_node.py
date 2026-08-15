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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class FFSweep(Node):

  def __init__(self):
    super().__init__('ff_sweep')
    self.declare_parameter('ramp_rate', 4.0)     # PWM/초 — 느릴수록 정확
    self.declare_parameter('max_pwm', 140)
    self.declare_parameter('max_speed', 1.2)     # 이 속도 넘으면 종료 [m/s]
    self.declare_parameter('start_pwm', 10)
    self.declare_parameter('settle_speed', 0.05)  # 이 이하는 '정지'로 간주

    self.rate = float(self.get_parameter('ramp_rate').value)
    self.max_pwm = int(self.get_parameter('max_pwm').value)
    self.max_speed = float(self.get_parameter('max_speed').value)
    self.pwm = float(self.get_parameter('start_pwm').value)
    self.settle = float(self.get_parameter('settle_speed').value)

    self.speed = 0.0
    self.samples = []          # (pwm, speed)
    self.done = False

    self.pub = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
    self.create_subscription(Float64, '/current_speed', self.speed_cb, 10)
    self.dt = 0.05
    self.create_timer(self.dt, self.tick)

    self.get_logger().info(
        f'FF 스윕 시작: PWM {self.pwm:.0f}부터 {self.rate:.1f}/초로 상승, '
        f'속도 {self.max_speed}m/s 또는 PWM {self.max_pwm} 도달 시 종료')

  def speed_cb(self, msg):
    self.speed = abs(float(msg.data))

  def tick(self):
    if self.done:
      self.pub.publish(Int32(data=0))
      return

    self.pwm += self.rate * self.dt
    if self.pwm >= self.max_pwm or self.speed >= self.max_speed:
      self.get_logger().info(
          f'종료 조건 도달 (PWM {self.pwm:.0f}, 속도 {self.speed:.2f}m/s) → 정지')
      self.done = True
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
    print(f'  ★ STATIC_FF          = {static_ff:.1f}   (현재 35.0)')
    print(f'  ★ VELOCITY_FF_GAIN   = {gain:.1f}   (현재 60.0)')
    print(f'     피팅 잔차(RMS)     = {rms:.1f} PWM')
    print()
    # 목표 속도별 필요 PWM → MAX_DRIVE_PWM 권장치
    for tgt in (0.5, 1.0, 1.5):
      need = static_ff + gain * tgt
      print(f'     {tgt:.1f} m/s 에 필요한 PWM ≈ {need:.0f}')
    need_max = static_ff + gain * self.max_speed
    rec = int(min(255, need_max * 1.4 + 20))
    print()
    print(f'  ★ MAX_DRIVE_PWM 권장 = {rec}  (최대속도 필요분 + PID 여유)')
    print('     현재 80. 이 값보다 낮으면 목표속도에 물리적으로 도달 불가.')
    print()
    print('  적용: henes_firmware.ino 의')
    print(f'     const float STATIC_FF = {static_ff:.1f}, '
          f'VELOCITY_FF_GAIN = {gain:.1f};')
    print(f'     #define MAX_DRIVE_PWM {rec}')
    print('  ※ FF가 정확해지면 PID는 작은 오차만 보정하면 되므로')
    print('     velocity_kp/ki 도 낮춰야 한다(현재 30/24는 FF 오차를 메우던 값).')
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

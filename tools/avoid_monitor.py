#!/usr/bin/env python3
"""avoid_monitor.py — 회피가 '보고 있는지'를 차를 세운 채로 확인한다.

왜 필요한가
  2026-09-12 학교 주행에서 차가 의자를 그대로 박았다. 원인은 bringup 에
  lidar:=true 를 빼먹어 회피 노드가 **아예 안 떠 있었던** 것이었다.
  그런데 노드를 띄운 뒤에도 회피가 안 될 수 있는 경로가 여럿이다:
    · 장애물이 fg_track_width(2.7m) 마스크 밖 → 없는 셈 친다
    · fg_obstacle_trigger(3.0m) 밖 → 아직 회피 안 한다
    · require_arm_for_steer 인데 arm 이 안 왔다 → 조향 출력 0
    · 먹스가 avoid_steer 를 안 먹는다 → 판단은 맞는데 바퀴가 안 돈다
  어느 단계에서 끊겼는지 **달리기 전에** 눈으로 본다.

쓰는 법 (차를 세워 두고, 장애물을 앞 1.5~2.5m 에 놓는다)
  ros2 launch gps_localization bringup.launch.py control:=true lidar:=true \
      waypoints:=... max_speed:=0.7
  python3 tools/avoid_monitor.py
"""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def main():
  rclpy.init()
  n = rclpy.create_node('avoid_monitor')
  st = {'scan': None, 'obs': None, 'steer': None, 'mode': None,
        'cmd': None, 't_scan': 0.0,
        # ★ 2026-09-13 펌웨어를 구운 뒤 확인할 것들.
        #   조향 스톨 상수(DEADBAND 24 / RESUME 32 / STALL_ERR 24 / STALL_MS 400)
        #   를 올렸으므로 stall 이 사라졌는지, 그리고 **명령각을 실제로 따라가는지**
        #   가 핵심이다. 회피각이 모자라 보이는 게 각도 문제가 아니라 조향이
        #   못 따라가는 문제일 수 있는데, 그건 이 두 칸을 비교해야 구분된다.
        'act': None, 'stall': None, 'vmin': None, 'pwm': None}

  def on_scan(m):
    st['scan'] = m
    st['t_scan'] = time.time()

  n.create_subscription(LaserScan, '/scan_front', on_scan,
                        qos_profile_sensor_data)
  # ⚠ 타입 주의 — 둘 다 **Float64** 다. Float32 로 구독하면 아무것도 안 온다
  #   (hil_probe 에서 /steering_angle 로 똑같이 데였다). 소스에서 확인함:
  #   cluster_plot_node.py 의 obstacle_pub / avoid_steer_pub.
  n.create_subscription(Float64, '/obstacle_distance',
                        lambda m: st.update(obs=m.data), 10)
  n.create_subscription(Float64, '/lidar/avoid_steer',
                        lambda m: st.update(steer=m.data), 10)
  n.create_subscription(String, '/lidar/mode',
                        lambda m: st.update(mode=m.data), 10)
  n.create_subscription(Twist, '/cmd_vel',
                        lambda m: st.update(cmd=m), 10)
  n.create_subscription(Float64, '/steering_angle',
                        lambda m: st.update(act=m.data), 10)
  n.create_subscription(Bool, '/vehicle_stall',
                        lambda m: st.update(stall=m.data), 10)
  n.create_subscription(Int32, '/vcc_min_mv',
                        lambda m: st.update(vmin=m.data), 10)
  n.create_subscription(Int32, '/drive_pwm',
                        lambda m: st.update(pwm=m.data), 10)

  print('장애물을 차 앞 2~3m 에 놓고 보세요. Ctrl-C 로 종료.')
  print('  · mode 가 AVOID 로 바뀌고 회피각이 나오는가')
  print('  · 조향 명령→실제 가 따라붙는가 (차이가 크면 조향이 못 따라가는 것)')
  print('  · 스톨이 뜨지 않는가 (2026-09-13 펌웨어에서 고친 부분)')
  print('  · 주행 중 전압이 4300mV 아래로 떨어지지 않는가\n')
  print(f"{'스캔':>6} {'최근접':>7} {'장애물':>8} {'mode':>9} "
        f"{'회피각':>8} {'조향 명령→실제':>15} {'PWM':>5} {'전압':>7} {'스톨':>5}")
  print('-' * 96)
  try:
    while rclpy.ok():
      for _ in range(25):
        rclpy.spin_once(n, timeout_sec=0.02)

      age = time.time() - st['t_scan']
      scan_s = '끊김' if (st['scan'] is None or age > 1.0) else 'OK'
      near = '—'
      if st['scan'] is not None and age <= 1.0:
        r = [x for x in st['scan'].ranges
             if st['scan'].range_min < x < st['scan'].range_max]
        near = f'{min(r):.2f}m' if r else '없음'

      obs = '—' if st['obs'] is None else f'{st["obs"]:.2f}m'
      mode = (st['mode'] or '—').split('|')[0]
      steer = '—' if st['steer'] is None else (
          'NaN' if st['steer'] != st['steer'] else f'{st["steer"]:+.1f}°')
      # 조향 명령(먹스가 낸 각) → 실제(펌웨어 ADC 환산). 차이가 크면
      # 각도가 아니라 **조향이 못 따라가는** 문제다.
      c = '—' if st['cmd'] is None else f'{st["cmd"].angular.z:+5.1f}'
      act = '—' if st['act'] is None else f'{st["act"]:+5.1f}'
      pair = f'{c}→{act}'
      pwm = '—' if st['pwm'] is None else f'{st["pwm"]:d}'
      vmin = '—' if st['vmin'] is None else f'{st["vmin"]}mV'
      stall = '—' if st['stall'] is None else ('⚠STALL' if st['stall'] else 'ok')

      print(f'{scan_s:>6} {near:>7} {obs:>8} {mode:>9} {steer:>8} '
            f'{pair:>15} {pwm:>5} {vmin:>7} {stall:>5}')
  except KeyboardInterrupt:
    pass
  finally:
    # rclpy 는 SIGINT 에 RCLError 를 던진다(KeyboardInterrupt 가 아니다).
    try:
      n.destroy_node()
      rclpy.shutdown()
    except Exception:
      pass


if __name__ == '__main__':
  main()

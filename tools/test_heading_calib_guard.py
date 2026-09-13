#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_heading_calib_guard.py — 헤딩 캘리브의 측위-점프 안전장치를 검증한다.

2026-08-24 충돌 사고 재현 시험이다. 그날 일어난 일:

    21:53:03.18  시작점 기록 (단독측위 — RTCM 아직 안 옴)
    21:53:07.29  직진 중... 1.1/10m
    21:53:07.67  직진 중... 8.4/10m   ← 0.38초에 7.3m (=19m/s) 점프
    21:53:07.83  [NTRIP] 첫 RTCM 발행 ← 단독측위→RTK Fixed 스냅
    21:53:08.24  (자동직진 출발 명령은 여기서 나간다 — 점프가 먼저였다)
    21:53:11.67  ✅ 완료 10.1m (최대 편차 36°) → yaw_offset=-155.8°
                 → 헤딩이 통째로 틀어진 채 AUTO → 우측으로 감겨 벽에 충돌

즉 **차가 서 있는 동안 좌표만 튄 것**을 10m 직진으로 적분했다.

이 시험은 가짜 차량 + 가짜 GPS 로 같은 상황을 만들고, 게이트를 하나씩 꺼가며
**각 게이트가 단독으로 사고를 막는지** 확인한다.

  가짜차량 ──/fix, handsfree/imu──→ heading_init_node ──/teleop/cmd_vel──→ 가짜차량
                                          │
                                          └─→ /heading/yaw_offset (이게 나오면 '캘리브 확정')

  ① require_rtk      : RTK 수렴 전엔 시작조차 안 한다
  ② max_jump_speed   : 물리적으로 불가능한 이동은 점프로 보고 무효화
  ③ 최소 소요시간     : 10m 를 너무 빨리 '갔다'면 주행이 아니다

  python3 tools/test_heading_calib_guard.py
"""

import math
import os
import signal
import subprocess
import sys
import time

# ★ 실차 스택과 격리한다 (test_parking_node.py 와 같은 이유).
TEST_DOMAIN = os.environ.get('CALIB_TEST_DOMAIN', '78')
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN

import rclpy                                             # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from rclpy.node import Node                              # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,     # noqa: E402
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, NavSatFix               # noqa: E402
from std_msgs.msg import Float64                         # noqa: E402

M_PER_DEG = 111320.0
LAT0, LON0 = 35.8245203, 128.7540086     # 사고 당시 시작점
L = 0.785

# 시험을 빨리 끝내려고 실제(10m/0.3m/s=33s)보다 짧게 잡는다. 게이트 로직은
# 거리·속도에 비례하므로 축소해도 성질이 같다.
CALIB_D = 4.0
SPEED = 0.5
COUNTDOWN = 2.0


def quat_from_yaw(y):
  return (0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))


class FakeVehicle(Node):
  """자전거모델 차량 + 가짜 GPS. 측위 오차와 점프를 주입할 수 있다."""

  def __init__(self, jump_at, jump_m, rtk_at):
    super().__init__('fake_vehicle')
    self.x = 0.0          # 참값 [m] (동/북)
    self.y = 0.0
    self.yaw = 0.0        # 참 헤딩 [rad] — 동쪽
    self.imu_bias = math.radians(37.0)   # IMU 는 맵에 정렬돼 있지 않다
    self.v = 0.0
    self.steer = 0.0
    self.t0 = time.time()

    self.jump_at = jump_at      # 이 시각[s]에 측위 오차가 사라진다(=점프)
    self.jump_m = jump_m        # 점프 크기[m]
    self.rtk_at = rtk_at        # 이 시각[s]부터 공분산이 RTK Fixed 수준
    # 단독측위 오차 방향 — 사고 때 점프는 남서(-148.9°) 쪽이었다
    self.err_dir = math.radians(-148.9 + 180.0)

    self.fix_pub = self.create_publisher(NavSatFix, '/fix',
                                         qos_profile_sensor_data)
    self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 50)
    self.create_subscription(Twist, '/teleop/cmd_vel', self.cmd_cb, 10)
    self.create_timer(0.05, self.step)      # 20Hz 적분
    self.create_timer(0.2, self.pub_fix)    # 5Hz GPS
    self.create_timer(0.02, self.pub_imu)   # 50Hz IMU
    self.last_step = time.time()
    self.moved = 0.0

  def cmd_cb(self, msg):
    self.v = float(msg.linear.x)
    self.steer = math.radians(float(msg.angular.z))

  def step(self):
    now = time.time()
    dt = now - self.last_step
    self.last_step = now
    self.x += self.v * math.cos(self.yaw) * dt
    self.y += self.v * math.sin(self.yaw) * dt
    self.yaw += self.v * math.tan(self.steer) / L * dt
    self.moved += abs(self.v) * dt

  def elapsed(self):
    return time.time() - self.t0

  def pub_fix(self):
    t = self.elapsed()
    # 측위 오차: jump_at 전에는 err 만큼 밀려 있고, 그 순간 사라진다(=점프)
    if t < self.jump_at:
      ex = self.jump_m * math.cos(self.err_dir)
      ey = self.jump_m * math.sin(self.err_dir)
    else:
      ex = ey = 0.0
    m = NavSatFix()
    m.header.stamp = self.get_clock().now().to_msg()
    m.header.frame_id = 'gps'
    m.latitude = LAT0 + (self.y + ey) / M_PER_DEG
    m.longitude = LON0 + (self.x + ex) / (M_PER_DEG
                                          * math.cos(math.radians(LAT0)))
    m.status.status = 1        # 이 드라이버는 RTK Fixed 여도 1(SBAS) 만 준다
    m.position_covariance_type = 3
    s = 0.003 if t >= self.rtk_at else 1.2      # RTK Fixed 3mm / 단독 1.2m
    m.position_covariance = [s * s, 0.0, 0.0,
                             0.0, s * s, 0.0,
                             0.0, 0.0, s * s]
    self.fix_pub.publish(m)

  def pub_imu(self):
    m = Imu()
    m.header.stamp = self.get_clock().now().to_msg()
    m.header.frame_id = 'imu'
    qx, qy, qz, qw = quat_from_yaw(self.yaw - self.imu_bias)
    m.orientation.x, m.orientation.y = qx, qy
    m.orientation.z, m.orientation.w = qz, qw
    self.imu_pub.publish(m)


class OffsetWatch(Node):
  def __init__(self):
    super().__init__('offset_watch')
    self.got = None
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    self.create_subscription(Float64, '/heading/yaw_offset',
                             self.cb, latched)

  def cb(self, msg):
    self.got = float(msg.data)


_case_idx = [0]


def _kill_group(proc):
  """`ros2 run` 래퍼와 그 자식 노드를 **그룹째** 죽인다.

  terminate() 로는 노드가 남는다(위 Popen 주석 참고). 남은 노드는 다음
  테스트까지 오염시키므로 SIGTERM → 1초 → SIGKILL 로 확실히 정리한다.
  """
  try:
    pgid = os.getpgid(proc.pid)
  except (ProcessLookupError, PermissionError):
    return
  # ⚠ proc.wait() 는 **래퍼**가 죽은 것만 알려준다. heading_init_node 는
  #   SIGTERM 을 무시하므로 래퍼만 죽고 노드는 살아남는데, wait() 가 곧바로
  #   돌아와 '죽었다' 고 오인하게 된다. 그래서 wait() 결과와 무관하게
  #   **항상 SIGKILL 까지 보낸다.**
  try:
    os.killpg(pgid, signal.SIGTERM)
  except (ProcessLookupError, PermissionError):
    return
  try:
    proc.wait(timeout=1.0)
  except subprocess.TimeoutExpired:
    pass
  try:
    os.killpg(pgid, signal.SIGKILL)
  except (ProcessLookupError, PermissionError):
    pass
  try:
    proc.wait(timeout=3.0)
  except subprocess.TimeoutExpired:
    pass


def run(name, expect_ok, extra_params, jump_at, jump_m, rtk_at,
        timeout=45.0):
  print('=' * 72)
  print(f'▶ {name}')
  print('=' * 72)

  # ★ 케이스마다 다른 도메인을 쓴다.
  #   /heading/yaw_offset 은 **TRANSIENT_LOCAL(래치)** 토픽이라, 앞 케이스의
  #   노드가 하나라도 살아남아 있으면 그 값이 새 구독자에게 **즉시** 배달된다.
  #   그러면 차가 0.00m 움직였는데도 '캘리브 확정' 으로 읽혀 거부 케이스가
  #   전부 거짓 통과한다(2026-09-13 에 실제로 4케이스 모두 +37.0° 로 나왔다).
  #   프로세스 정리를 고쳐도, 도메인을 가르는 것이 마지막 방어선이다.
  domain = str(int(TEST_DOMAIN) + _case_idx[0])
  _case_idx[0] += 1
  os.environ['ROS_DOMAIN_ID'] = domain      # rclpy.init 이 이 값을 읽는다
  env = dict(os.environ, ROS_DOMAIN_ID=domain)
  params = [
      '-p', f'calib_distance:={CALIB_D}',
      '-p', 'auto_drive:=true',
      '-p', f'auto_speed:={SPEED}',
      '-p', f'auto_countdown:={COUNTDOWN}',
      '-p', 'max_calib_retries:=1',      # 실패하면 곧바로 포기 — 판정이 명확해진다
  ] + extra_params
  # ★ start_new_session — 프로세스 **그룹**으로 죽이기 위해서다.
  #   `ros2 run` 은 래퍼이고 실제 노드는 그 자식이다. proc.terminate() 는
  #   래퍼만 죽이고 노드는 살아남아 stdout 파이프를 계속 잡는다. 그러면
  #   아래 communicate() 가 영원히 블록되고, 테스트는 A 케이스에서 멈춘 채
  #   **노드를 도메인에 남긴다**. 남은 노드는 다음 케이스(그리고 다음 테스트)
  #   에 같은 토픽으로 끼어들어 엉뚱한 실패를 만든다.
  #   (2026-09-13: 실제로 이렇게 540초를 넘겼고, 남은 노드는 SIGTERM 을
  #    무시해 SIGKILL 이 필요했다.)
  proc = subprocess.Popen(
      ['ros2', 'run', 'gps_heading_init', 'heading_init_node',
       '--ros-args'] + params,
      env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
      text=True, bufsize=1, start_new_session=True)

  rclpy.init()
  veh = FakeVehicle(jump_at, jump_m, rtk_at)
  watch = OffsetWatch()
  ex = rclpy.executors.SingleThreadedExecutor()
  ex.add_node(veh)
  ex.add_node(watch)

  # 노드가 뭔가 발행하기 전에 이미 값이 잡히면 그건 **남의 값**이다.
  for _ in range(20):
    ex.spin_once(timeout_sec=0.05)
  if watch.got is not None:
    print(f'   ⚠ 시작 전에 이미 yaw_offset 이 잡혔다 (도메인 {domain}). '
          '이전 노드가 살아 있다 — 결과를 믿을 수 없다.')
    _kill_group(proc)
    veh.destroy_node(); watch.destroy_node(); rclpy.shutdown()
    return False

  t_end = time.time() + timeout
  while time.time() < t_end and watch.got is None and proc.poll() is None:
    ex.spin_once(timeout_sec=0.05)
  # 노드가 종료 직전 발행한 값을 놓치지 않도록 조금 더 돈다
  for _ in range(40):
    ex.spin_once(timeout_sec=0.05)

  moved = veh.moved
  veh.destroy_node()
  watch.destroy_node()
  rclpy.shutdown()
  _kill_group(proc)
  try:
    out = proc.communicate(timeout=8)[0]
  except subprocess.TimeoutExpired:
    # 그룹 SIGKILL 뒤에도 파이프가 안 닫히면 읽기를 포기한다.
    # 로그를 못 읽는 것보다 테스트가 멈추는 것이 나쁘다.
    out = ''

  for line in (out or '').splitlines():
    if any(k in line for k in ('✅', '❌', '무효', '점프', 'RTK', '완료',
                               '재시도', '거부')):
      print('   │ ' + line.strip()[:150])

  got = watch.got
  print(f'   실제 주행거리 {moved:.2f}m   yaw_offset '
        f'{"미발행" if got is None else f"{math.degrees(got):+.1f}°"}')

  if expect_ok:
    ok = got is not None
    # 캘리브가 확정됐다면 값이 맞아야 한다. 참 헤딩(동쪽=0°)과 IMU 바이어스
    # 37° 를 넣었으므로 yaw_offset 은 +37° 여야 한다.
    if ok:
      err = abs(math.degrees(math.atan2(math.sin(got - veh.imu_bias),
                                        math.cos(got - veh.imu_bias))))
      ok = err < 8.0
      print(f'   기대값 +37.0° 대비 오차 {err:.1f}°')
  else:
    ok = got is None

  print(f'   {"✅ PASS" if ok else "❌ FAIL"} — '
        f'기대: {"캘리브 확정" if expect_ok else "캘리브 거부"}\n')
  return ok


def main():
  results = []

  # A. 정상 — RTK 처음부터 좋고 점프 없음 → 캘리브가 확정돼야 한다.
  #    (게이트가 멀쩡한 캘리브까지 막아버리면 그게 더 큰 사고다)
  results.append(run(
      'A. 정상 주행 — 캘리브가 확정돼야 한다',
      expect_ok=True, extra_params=[],
      jump_at=0.0, jump_m=0.0, rtk_at=0.0))

  # B. 사고 재현 — RTK 가 5초 뒤 수렴하며 7.3m 점프.
  #    게이트① 이 점프 전 시작을 막으므로, 점프가 지나간 뒤 정상 캘리브된다.
  results.append(run(
      'B. 사고 재현 (RTK 늦게 수렴 + 7.3m 점프) — 게이트①이 막고 정상 확정',
      expect_ok=True, extra_params=[],
      jump_at=5.0, jump_m=7.3, rtk_at=5.0))

  # C. 게이트① 을 끄고 같은 점프 → 게이트② 가 잡아야 한다.
  results.append(run(
      'C. 게이트① OFF + 7.3m 점프 — 게이트②(점프속도)가 거부해야 한다',
      expect_ok=False, extra_params=['-p', 'require_rtk:=false'],
      jump_at=5.0, jump_m=7.3, rtk_at=5.0))

  # D. 게이트①② 를 끄고 같은 점프 → 게이트③ 이 잡아야 한다.
  #    (사고 당시 코드는 여기서 아무 저항 없이 통과했다)
  results.append(run(
      'D. 게이트①② OFF + 7.3m 점프 — 게이트③(최소시간)이 거부해야 한다',
      expect_ok=False,
      extra_params=['-p', 'require_rtk:=false', '-p', 'max_jump_speed:=999.0'],
      jump_at=5.0, jump_m=7.3, rtk_at=5.0))

  print('=' * 72)
  n_ok = sum(1 for r in results if r)
  print(f'결과: {n_ok}/{len(results)} 통과')
  print('=' * 72)
  return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
  sys.exit(main())

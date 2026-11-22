#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speed_pi_live.py — 실주행 중 속도 PI 가 목표를 따라가는지 **실시간으로** 판정한다.

★ 왜 (2026-10-09)
  주행이 끝난 뒤 그래프를 뽑아서야 '안 됐다' 를 알면 그 주행은 버린다.
  SIL 기준(RMS 0.08~0.12)을 현장에서 바로 대조해, 캘리브 직진 10m 안에
  PI 가 살아 있는지·목표를 따라가는지 알 수 있게 한다.

  보는 것: /target_speed(목표) · /current_speed_gps(PI 가 본 실측) · /cmd_vel
  2초마다 최근 5초의 평균오차·RMS 와 판정을 찍고, Ctrl-C 하면 전체 요약을 낸다.

  ⚠ /current_speed_gps 가 안 오면 PI 가 꺼져 있다 — 런치에 spd_kp 를 빠뜨렸거나
    ff_mode 가 ros 가 아니다. 시작 직후 그 경고를 낸다.

사용 (런치 뒤 새 터미널):
  python3 tools/speed_pi_live.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

OK_RMS, WARN_RMS = 0.15, 0.30      # SIL 최악(FLOAT 포함) 0.14 · 개루프 0.32
MEAS_LAG = 0.25                    # 위치미분 0.5s 창의 지연 — 명령을 이만큼 늦춰 비교
START_SKIP = 3.0                   # 정지→출발 가속 구간은 추종 오차가 아니다


class Live(Node):
  def __init__(self):
    super().__init__('speed_pi_live')
    self.tgt = None; self.cmd = 0.0; self.meas = None; self.meas_t = 0.0
    self.cmd_hist = []               # (t, cmd) — 지연 맞춤용
    self.go_since = None             # 명령이 0.3 을 넘은 시각(출발)
    self.buf = []                    # (t, target, meas)
    self.all = []
    self.t0 = time.time()
    self.create_subscription(Float64, '/target_speed', lambda m: setattr(self, 'tgt', m.data), 10)
    self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m.linear.x), 10)
    self.create_subscription(Float64, '/current_speed_gps', self.meas_cb, 10)
    self.create_timer(0.1, self.sample)
    self.create_timer(2.0, self.report)
    print('속도 PI 실시간 판정 — 기준: RMS < 0.15 양호 · < 0.30 주의 · 그 이상 불량')

  def meas_cb(self, m):
    self.meas = m.data; self.meas_t = time.time()

  def sample(self):
    now = time.time()
    self.cmd_hist.append((now, self.cmd))
    self.cmd_hist = [c for c in self.cmd_hist if now - c[0] <= 2.0]
    if self.cmd > 0.3:
      if self.go_since is None:
        self.go_since = now
    else:
      self.go_since = None
    # 실제로 달리라는 명령이 있고, 출발 가속이 끝났고, 측정이 신선할 때만 센다
    if self.go_since is not None and now - self.go_since > START_SKIP and \
        self.meas is not None and now - self.meas_t < 0.5:
      # 먹스 출력(캘리브 중에도 실제 명령)을 측정 지연만큼 늦춰서 비교한다
      past = [c for t, c in self.cmd_hist if t <= now - MEAS_LAG]
      ref = past[-1] if past else self.cmd
      self.buf.append((now, ref, self.meas)); self.all.append((ref, self.meas))
    self.buf = [b for b in self.buf if now - b[0] <= 5.0]

  def report(self):
    el = time.time() - self.t0
    if self.meas is None:
      if el > 15:
        print(f'[{el:5.0f}s] ❌ /current_speed_gps 가 안 온다 — 속도 PI 꺼짐 '
              '(spd_kp:=80.0 · ff_mode:=ros 확인) 또는 GPS 미수신')
      else:
        print(f'[{el:5.0f}s] 대기 — 위치 수신 대기 중')
      return
    if time.time() - self.meas_t > 1.0:
      print(f'[{el:5.0f}s] ⚠ 측정 끊김 — RTK FLOAT 이거나 위치가 안 온다 (그동안 FF 만)')
      return
    if len(self.buf) < 10:
      print(f'[{el:5.0f}s] 대기 — 명령 {self.cmd:.2f} · 실측 {self.meas:.2f}')
      return
    e = [m - r for _, r, m in self.buf]
    mean = sum(e) / len(e); rms = math.sqrt(sum(x * x for x in e) / len(e))
    tag = '✅ 양호' if rms < OK_RMS else ('⚠ 주의' if rms < WARN_RMS else '❌ 불량 — 멈추고 알려줄 것')
    print(f'[{el:5.0f}s] 명령 {self.buf[-1][1]:.2f} · 실측 {self.buf[-1][2]:.2f} · '
          f'최근5초 평균 {mean:+.2f} RMS {rms:.2f}  {tag}')

  def summary(self):
    if len(self.all) < 20:
      print('\n요약: 주행 표본이 부족하다'); return
    e = [m - r for r, m in self.all]
    mean = sum(e) / len(e); rms = math.sqrt(sum(x * x for x in e) / len(e))
    over = 100 * sum(abs(x) > 0.3 for x in e) / len(e)
    print(f'\n요약 ({len(e) / 10:.0f}초 주행): 평균 {mean:+.3f} · RMS {rms:.3f} · '
          f'|오차|>0.3 {over:.1f}%   (SIL 기준 RMS 0.08~0.12)')


def main():
  rclpy.init(); n = Live()
  try:
    rclpy.spin(n)
  except KeyboardInterrupt:
    pass
  n.summary()


if __name__ == '__main__':
  main()

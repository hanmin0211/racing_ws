#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heading_tracker.py — 주행 중 GPS 진행방향으로 IMU 헤딩 오프셋을 **계속** 고친다.

★ 왜 (2026-10-12)
  헤딩 오프셋은 출발 때 10 m 직진으로 **한 번만** 잡았다. 그런데 저가 IMU 의 yaw 는
  떠내려간다. 직선 헤딩 잔차의 기울기:
      10/7  −1.5 °/분     10/8  −3.7 °/분     10/12  **−21 °/분**
  10/12 은 캘리브 직후 −2.8° 가 랩 끝에 −28° 가 됐고, 차가 랩 내내 경로 오른쪽으로
  0.5~1.5 m 밀려 달렸다. 라이다 지도도 번졌다(drive_scan.csv).

방법 — 느린 성분만 GPS 로 (상보필터)
  IMU 는 짧은 시간의 회전은 정확하고 천천히 떠내려간다. GPS 진행방향은 떠내려가지
  않지만 직진·주행 중에만 맞는다. 그래서:
    측정   m = GPS 현(chord) 방향(창 win 초) − IMU yaw(창 **가운데** 시각)
    갱신   offset += (Δt / tau) · wrap(m − offset)        (tau 초 시정수)
  창 가운데 시각의 IMU 와 비교하므로 현 방향의 시간 지연이 없다.

  쓰지 않는 표본 (이유를 센다)
    slow    : 현 길이/시간 < min_speed  — 서 있거나 느리면 방향이 잡음이다
    turn    : 창 안 IMU 회전율 > max_rate — 커브에선 현 방향이 접선과 어긋난다
    rtk     : 수평 σ > max_sigma        — FLOAT 위치는 방향을 망친다
    outlier : |m − offset| > gate       — 후진·점프. 오차가 이만큼 커질 일은 없다
                                          (계속 고치고 있으니까)

이득 근거 (tools/heading_track_replay.py, 10/12 bag 재생)
  고정 오프셋   직선 잔차 평균 −14°
  tau 3 s      → 평균·RMS 는 그 도구 출력 참고. 드리프트 −0.35 °/s × 3 s ≈ 1° 지연오차.

ROS 의존 없음 — 노드와 재생 도구가 같은 코드를 쓴다(베끼면 갈라진다).
"""
import math
from collections import deque


def wrap(a):
  return math.atan2(math.sin(a), math.cos(a))


class HeadingTracker:

  def __init__(self, tau=3.0, win=0.6, min_speed=0.5, max_rate_dps=15.0,
               gate_deg=20.0, max_sigma=0.05):
    self.tau = tau
    self.win = win
    self.min_speed = min_speed
    self.max_rate = math.radians(max_rate_dps)
    self.gate = math.radians(gate_deg)
    self.max_sigma = max_sigma
    self.offset = None                 # rad. None = 아직 캘리브 전 (고치지 않는다)
    self._imu = deque()                # (t, 연속 yaw)
    self._imu_last = None
    self._fix = deque()                # (t, x, y)
    self._last_upd = None
    self.counts = {'ok': 0, 'slow': 0, 'turn': 0, 'rtk': 0, 'outlier': 0, 'nodata': 0}
    self.last_innov = 0.0

  def set_offset(self, offset):
    """캘리브 결과로 초기화(또는 재캘리브)."""
    self.offset = wrap(offset)
    self._last_upd = None

  def add_imu(self, t, raw):
    """IMU 원시 yaw(오프셋 전). 연속값으로 펼쳐 보관한다."""
    if self._imu_last is not None:
      raw = self._imu_last + wrap(raw - self._imu_last)
    self._imu_last = raw
    self._imu.append((t, raw))
    while self._imu and t - self._imu[0][0] > 3.0:
      self._imu.popleft()

  def _imu_at(self, t):
    q = self._imu
    if len(q) < 2:
      return None
    # GPS 표본이 도착한 순간엔 최신 IMU 가 몇 ms 묵어 있다(IMU 300 Hz). 끝에서
    # 50 ms 까지는 끝값을 쓴다 — 그 사이 회전은 커브 판정(max_rate) 아래라 작다.
    if t > q[-1][0]:
      return q[-1][1] if t - q[-1][0] < 0.05 else None
    if t < q[0][0]:
      return None
    lo, hi = 0, len(q) - 1
    while hi - lo > 1:
      mid = (lo + hi) // 2
      if q[mid][0] <= t:
        lo = mid
      else:
        hi = mid
    (t0, y0), (t1, y1) = q[lo], q[hi]
    return y0 if t1 <= t0 else y0 + (y1 - y0) * (t - t0) / (t1 - t0)

  def add_fix(self, t, x, y, sigma=None):
    """GPS 위치 한 표본. 고쳤으면 'ok', 아니면 버린 이유를 돌려준다."""
    self._fix.append((t, x, y))
    while len(self._fix) > 2 and t - self._fix[1][0] >= self.win:
      self._fix.popleft()
    if self.offset is None:
      return None
    t0, x0, y0 = self._fix[0]
    dt = t - t0
    if dt < self.win * 0.8 or dt > self.win * 2.5:
      return self._count('nodata')
    if sigma is not None and sigma > self.max_sigma:
      return self._count('rtk')
    dist = math.hypot(x - x0, y - y0)
    if dist / dt < self.min_speed:
      return self._count('slow')
    ya, yb, ym = self._imu_at(t0), self._imu_at(t), self._imu_at(0.5 * (t0 + t))
    if ya is None or yb is None or ym is None:
      return self._count('nodata')
    if abs(yb - ya) / dt > self.max_rate:
      return self._count('turn')
    meas = wrap(math.atan2(y - y0, x - x0) - ym)
    innov = wrap(meas - self.offset)
    if abs(innov) > self.gate:
      return self._count('outlier')
    step = 0.1 if self._last_upd is None else min(t - self._last_upd, 0.5)
    self.offset = wrap(self.offset + (step / self.tau) * innov)
    self._last_upd = t
    self.last_innov = innov
    return self._count('ok')

  def _count(self, k):
    self.counts[k] += 1
    return k

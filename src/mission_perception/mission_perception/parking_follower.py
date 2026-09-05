#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parking_follower.py — 주차 궤적 재생 상태기계 (ROS 없음).

`parking_node` 의 제어부를 그대로 떼어낸 것이다. ROS 에 안 묶여 있으므로
`tools/test_parking_plan.py` 가 자전거모델과 붙여 **수백 개 시작자세를 몇 초 만에**
폐루프로 돌려볼 수 있다. 노드는 이 클래스를 감싸기만 한다 — 즉 시험에서 검증되는
코드와 실차에서 도는 코드가 같은 코드다.

상태: DRIVE → (CUSP → DRIVE)* → DONE / ABORT
"""

import math


def yaw_wrap(a):
  return math.atan2(math.sin(a), math.cos(a))


def pursuit_steer(x_ld, y_ld, wheelbase, max_steer_rad):
  """차량기준 lookahead 점 → 조향각[rad]. 전진·후진 공통.

  원점에서 x축에 접하고 P(x_ld,y_ld) 를 지나는 원호의 곡률 κ = 2·y_ld/ld².
  P 가 앞이든 뒤든 성립한다. v<0 을 넣고 적분하면 δ>0 에서 후방-좌측으로 간다.
  """
  ld2 = x_ld * x_ld + y_ld * y_ld
  if ld2 < 1e-6:
    return 0.0
  kappa = 2.0 * y_ld / ld2
  delta = math.atan(wheelbase * kappa)
  return max(-max_steer_rad, min(max_steer_rad, delta))


def split_segments(points):
  """[(x,y,gear)] → 같은 기어끼리 묶은 구간 [(gear, [(x,y),...])].

  기어가 바뀌는 지점(cusp)에서 차는 반드시 완전히 멈춰야 하므로, 이전 구간의
  마지막 점을 다음 구간 첫 점으로 넣어 궤적이 끊기지 않게 한다.
  """
  segs = []
  cur_gear, cur = None, []
  for (x, y, g) in points:
    if cur_gear is None:
      cur_gear, cur = g, [(x, y)]
      continue
    if g != cur_gear:
      segs.append((cur_gear, cur))
      cur_gear, cur = g, [cur[-1], (x, y)]
    else:
      cur.append((x, y))
  if cur_gear is not None and len(cur) >= 2:
    segs.append((cur_gear, cur))
  return [s for s in segs if len(s[1]) >= 2]


class ParkingFollower(object):
  """구간별 pure pursuit + cusp 정지 + 이탈 감시."""

  def __init__(self, segs, *, wheelbase=0.785, max_steering_deg=18.0,
               speed=0.3, min_speed=0.15, lookahead=0.8,
               goal_tolerance=0.20, final_tolerance=0.15,
               approach_decel=0.4, cusp_dwell=1.0, stopped_speed=0.05,
               rate=20.0, steer_sign_reverse=1.0, max_steer_rate_deg=90.0,
               abort_cross_track=1.0, approach_cross_track=3.0,
               converge_tol=0.35, start_seg=0, start_cursor=0):
    self.segs = segs
    self.L = wheelbase
    self.max_steer = math.radians(max_steering_deg)
    self.v_mag = abs(speed)
    self.v_min = abs(min_speed)
    self.ld = lookahead
    self.goal_tol = goal_tolerance
    self.final_tol = final_tolerance
    self.decel = approach_decel
    self.cusp_dwell = cusp_dwell
    self.stopped_v = stopped_speed
    self.rate = rate
    self.rev_sign = steer_sign_reverse
    self.max_steer_step = max_steer_rate_deg / rate
    self.abort_cte = abort_cross_track
    self.approach_cte = approach_cross_track
    self.converge_tol = converge_tol

    self.seg_i = start_seg
    self.cursor = start_cursor
    self.state = 'DRIVE'
    self.converged = False
    self.cusp_since = None
    self.prev_steer_deg = 0.0
    self.cmd_v = 0.0
    self.reason = ''
    self.events = []          # (state, 설명) — 노드가 로그로 흘린다
    self.status = ''

  # ---------------- 기하 ----------------
  def to_vehicle(self, pose, px, py):
    x, y, yaw = pose
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c

  def advance_cursor(self, pose, pl):
    """앞쪽만 훑어 최근접 점으로 커서를 전진시킨다(되돌아가지 않는다)."""
    x, y = pose[0], pose[1]
    best_i, best_d = self.cursor, float('inf')
    for i in range(self.cursor, len(pl)):
      d = math.hypot(pl[i][0] - x, pl[i][1] - y)
      if d < best_d:
        best_d, best_i = d, i
      elif d > best_d + 2.0:
        break
    self.cursor = best_i
    return best_d

  def lookahead_point(self, pl):
    s = 0.0
    for i in range(self.cursor, len(pl) - 1):
      seg = math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1])
      if s + seg >= self.ld:
        t = (self.ld - s) / seg if seg > 1e-6 else 0.0
        return (pl[i][0] + t * (pl[i + 1][0] - pl[i][0]),
                pl[i][1] + t * (pl[i + 1][1] - pl[i][1]))
      s += seg
    return pl[-1]

  def remaining(self, pl):
    return sum(math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1])
               for i in range(self.cursor, len(pl) - 1))

  # ---------------- 주기 ----------------
  def update(self, pose, speed, now):
    """반환 (v[m/s], steer[deg]). 끝난 뒤에도 계속 불러도 된다(0 을 낸다)."""
    if self.state in ('DONE', 'ABORT'):
      return 0.0, 0.0
    if pose is None:
      self.status = '측위 끊김 — 정지'
      return 0.0, 0.0

    gear, pl = self.segs[self.seg_i]

    if self.state == 'CUSP':
      if abs(speed) > self.stopped_v:
        self.cusp_since = None
        return 0.0, 0.0
      if self.cusp_since is None:
        self.cusp_since = now
        return 0.0, 0.0
      if now - self.cusp_since >= self.cusp_dwell:
        self.seg_i += 1
        self.cursor = 0
        self.converged = False
        self.cusp_since = None
        self.prev_steer_deg = 0.0
        self.cmd_v = 0.0
        self.state = 'DRIVE'
        ng = self.segs[self.seg_i][0]
        self.events.append(
            ('SEG', f'구간 {self.seg_i} 시작 — {"전진" if ng > 0 else "후진"}'))
      return 0.0, 0.0

    # ---- DRIVE ----
    cte = self.advance_cursor(pose, pl)

    # 이탈 감시는 '따라가다 벗어나는 것'을 잡는 장치이지, 아직 한 번도 붙어보지
    # 못한 초기 접근을 막으려는 게 아니다. 붙기 전엔 느슨하게, 붙고 나면 조인다.
    if not self.converged and cte <= self.converge_tol:
      self.converged = True
      self.events.append(('CONV', f'궤적에 붙었다 (cte {cte:.2f}m)'))
    limit = self.abort_cte if self.converged else self.approach_cte
    if cte > limit:
      return self._abort(f'궤적에서 {cte:.2f}m 벗어남 (한계 {limit}m'
                         f'{"" if self.converged else ", 접근 중"})')

    rem = self.remaining(pl)
    end_d = math.hypot(pl[-1][0] - pose[0], pl[-1][1] - pose[1])
    is_last = (self.seg_i == len(self.segs) - 1)
    tol = self.final_tol if is_last else self.goal_tol

    if min(rem, end_d) <= tol:
      if is_last:
        self.state = 'DONE'
        self.cmd_v = 0.0
        self.events.append(('DONE', f'주차 완료 — 목표점까지 {end_d:.2f}m'))
      else:
        self.state = 'CUSP'
        self.cusp_since = None
        self.cmd_v = 0.0
        self.events.append(
            ('CUSP', f'구간 {self.seg_i} 끝 ({end_d:.2f}m) — 정지 후 기어 전환'))
      return 0.0, 0.0

    lx, ly = self.lookahead_point(pl)
    x_ld, y_ld = self.to_vehicle(pose, lx, ly)

    # 방향 정합성: 전진 구간의 목표점은 앞에, 후진 구간의 목표점은 뒤에.
    # 끝에 가까우면 lookahead 가 마지막 점으로 눌려 부호가 흔들리므로 안 본다.
    dir_tol = 0.0 if self.converged else self.ld * 0.75
    if rem > self.ld * 1.5:
      if gear > 0 and x_ld <= -dir_tol:
        return self._abort(f'전진 구간인데 목표점이 뒤 (x={x_ld:.2f}) — 헤딩 의심')
      if gear < 0 and x_ld >= dir_tol:
        return self._abort(f'후진 구간인데 목표점이 앞 (x={x_ld:.2f}) — 헤딩 의심')

    delta = pursuit_steer(x_ld, y_ld, self.L, self.max_steer)
    if gear < 0:
      delta *= self.rev_sign
    delta_deg = math.degrees(delta)
    delta_deg = max(self.prev_steer_deg - self.max_steer_step,
                    min(self.prev_steer_deg + self.max_steer_step, delta_deg))
    self.prev_steer_deg = delta_deg

    v_goal = min(self.v_mag, math.sqrt(2.0 * self.decel * max(0.0, rem - tol)))
    v_goal = max(self.v_min, v_goal) * (1.0 if gear > 0 else -1.0)
    step = self.decel / self.rate
    self.cmd_v = max(self.cmd_v - step, min(self.cmd_v + step, v_goal))

    self.status = (f'구간{self.seg_i}({"전" if gear > 0 else "후"}) '
                   f'남은 {rem:.2f}m  cte {cte:.2f}m  '
                   f'v {self.cmd_v:+.2f}  δ {delta_deg:+.1f}°')
    return self.cmd_v, delta_deg

  def _abort(self, why):
    self.state = 'ABORT'
    self.reason = why
    self.cmd_v = 0.0
    self.events.append(('ABORT', why))
    return 0.0, 0.0

  def drain_events(self):
    out, self.events = self.events, []
    return out

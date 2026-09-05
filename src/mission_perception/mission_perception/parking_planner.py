#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parking_planner.py — 차가 **실제로 서 있는 자세**에서 주차칸까지 궤적을 만든다.

  (현재 자세) ──전진 Dubins──→ (기어전환점 cusp) ──후진 원호+직선──→ (주차 완료 자세)

★ 왜 이 파일이 생겼나 (2026-08-25)
  기존 방식은 `make_parking_path.py` 가 **미리** 궤적을 만들어 yaml 로 저장하고,
  `parking_node` 가 재생할 때 '가장 가까운 점'을 찾아 거기서 이어 갔다.
  그런데 완주 종점은 매 주행마다 다르고, 실제로 자리1·2 에서는 최근접 점이
  **후진 원호 한복판**으로 잡혔다. 위치는 0.1~0.4m 로 가까웠지만 그 점에서의
  궤적 진행 자세와 차의 자세가 58~78° 어긋나 있었다. 최소회전반경 2.42m 로는
  그 자세차를 좁은 주차구역 안에서 메울 수 없어 매번 ABORT 했다.

  위치만 보고 궤적에 올라타려 한 것이 문제다. **자세(위치+헤딩)** 로 붙어야 한다.
  Dubins 경로는 임의의 두 자세를 곡률한계 안에서 정확히 잇는 최단 전진경로이고
  해가 항상 존재한다. 그래서 '어긋난 자세를 메운다' 가 아니라 '지금 자세에서
  출발하는 경로를 새로 만든다' 로 문제를 바꾼다.

★ 왜 후진부는 Dubins 가 아닌가
  후진 진입은 주차 완료 자세에서 거꾸로 쌓는 편이 정확하다(칸에 곧게 들어가는
  마지막 직선을 보장해야 한다). 그래서 후진부는 `make_parking_path.py` 와 같은
  방식(직선 d_str + 원호 R)으로 만들고, **cusp 자세를 자유변수로 두고 탐색**한다.
  cusp 가 정해지면 전진부는 Dubins 로 정확히 이어진다.

이 파일은 ROS 에 의존하지 않는다 — 노드와 시험 도구가 같은 코드를 쓴다.
"""

import math

WHEELBASE = 0.785
MAX_STEER_DEG = 18.0
R_MIN = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))   # 2.4157 m


def wrap(a):
  """[-pi, pi)"""
  return math.atan2(math.sin(a), math.cos(a))


def _mod2pi(t):
  return t - 2.0 * math.pi * math.floor(t / (2.0 * math.pi))


# ---------------------------------------------------------------- Dubins
#
# Shkel & Lumelsky 의 6개 단어(LSL RSR LSR RSL RLR LRL).
# ★ 공식을 그대로 믿지 않는다: 만든 (t,p,q) 로 **직접 적분해 끝자세를 확인**하고
#   목표 자세와 1mm/0.1° 안에서 안 맞으면 그 단어는 버린다. 부호 하나 틀린 공식이
#   '그럴듯하지만 엉뚱한 곳으로 가는 경로'를 내놓는 것이 이 판에서 제일 위험하다.

def _words(alpha, beta, d):
  sa, sb = math.sin(alpha), math.sin(beta)
  ca, cb = math.cos(alpha), math.cos(beta)
  c_ab = math.cos(alpha - beta)
  out = []

  p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb)
  if p_sq >= 0.0:
    th = math.atan2(cb - ca, d + sa - sb)
    out.append(('LSL', _mod2pi(th - alpha), math.sqrt(p_sq),
                _mod2pi(beta - th)))

  p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa)
  if p_sq >= 0.0:
    th = math.atan2(ca - cb, d - sa + sb)
    out.append(('RSR', _mod2pi(alpha - th), math.sqrt(p_sq),
                _mod2pi(th - beta)))

  p_sq = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb)
  if p_sq >= 0.0:
    p = math.sqrt(p_sq)
    th = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    out.append(('LSR', _mod2pi(th - alpha), p, _mod2pi(th - beta)))

  p_sq = -2.0 + d * d + 2.0 * c_ab - 2.0 * d * (sa + sb)
  if p_sq >= 0.0:
    p = math.sqrt(p_sq)
    th = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    out.append(('RSL', _mod2pi(alpha - th), p, _mod2pi(beta - th)))

  tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
  if abs(tmp) <= 1.0:
    p = _mod2pi(2.0 * math.pi - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    out.append(('RLR', t, p, _mod2pi(alpha - beta - t + p)))

  tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sb - sa)) / 8.0
  if abs(tmp) <= 1.0:
    p = _mod2pi(2.0 * math.pi - math.acos(tmp))
    t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
    out.append(('LRL', t, p, _mod2pi(beta - alpha - t + p)))

  return out


_TURN = {'L': 1.0, 'S': 0.0, 'R': -1.0}


def _run_word(pose, word, seg_lens, R, ds):
  """단어를 실제로 적분해 점열과 끝자세를 만든다. 반환 (points, end_pose)."""
  x, y, th = pose
  pts = [(x, y)]
  for ch, ln in zip(word, seg_lens):
    if ln <= 1e-9:
      continue
    k = _TURN[ch] / R if ch != 'S' else 0.0
    n = max(1, int(math.ceil(ln / ds)))
    step = ln / n
    for _ in range(n):
      if k == 0.0:
        x += step * math.cos(th)
        y += step * math.sin(th)
      else:
        th_new = th + k * step
        # 원호 중심을 지나는 정확한 적분 (직선 근사 누적오차를 없앤다)
        cx = x - math.sin(th) / k
        cy = y + math.cos(th) / k
        x = cx + math.sin(th_new) / k
        y = cy - math.cos(th_new) / k
        th = th_new
      pts.append((x, y))
  return pts, (x, y, th)


def dubins_length(start, goal, R):
  """샘플링 없이 최단 길이만. 탐색 1차 통과용(최종 경로는 dubins() 로 검증한다)."""
  dx, dy = goal[0] - start[0], goal[1] - start[1]
  d = math.hypot(dx, dy) / R
  th = math.atan2(dy, dx)
  alpha = _mod2pi(start[2] - th)
  beta = _mod2pi(goal[2] - th)
  best = None
  for (_w, t, p, q) in _words(alpha, beta, d):
    if t < 0 or p < 0 or q < 0:
      continue
    total = (t + p + q) * R
    if best is None or total < best:
      best = total
  return best


def dubins(start, goal, R, ds=0.15):
  """전진 전용 최단 경로. 반환 (length, points, word) 또는 None.

  start/goal 은 (x, y, yaw[rad]).
  """
  dx, dy = goal[0] - start[0], goal[1] - start[1]
  D = math.hypot(dx, dy)
  d = D / R
  th = math.atan2(dy, dx)
  alpha = _mod2pi(start[2] - th)
  beta = _mod2pi(goal[2] - th)

  best = None
  for (word, t, p, q) in _words(alpha, beta, d):
    if t < 0 or p < 0 or q < 0:
      continue
    lens = [t * R, p * R if word[1] == 'S' else p * R, q * R]
    total = sum(lens)
    if best is not None and total >= best[0]:
      continue
    pts, end = _run_word(start, word, lens, R, ds)
    # ★ 검증: 공식이 맞았는지 적분 결과로 확인한다.
    if math.hypot(end[0] - goal[0], end[1] - goal[1]) > 1e-3:
      continue
    if abs(wrap(end[2] - goal[2])) > math.radians(0.1):
      continue
    best = (total, pts, word)
  return best


# ------------------------------------------------------- 후진 진입부
def reverse_entry(goal, th_g, th_a, R, d_str, ds=0.15):
  """주차 완료 자세에서 거꾸로 쌓은 후진 경로.

    cusp(헤딩 th_a) ──후진 원호 R, Δθ=wrap(th_g−th_a)──→ P1 ──후진 직선 d_str──→ goal

  반환 (cusp_pose, points[cusp→goal 순서], Δθ).

  ★ 원호는 **해석해로** 만든다. 예전처럼 Euler 적분으로 쌓으면 ds=2cm 에서도
    끝점이 1.4cm 어긋나고, 그 오차가 그대로 cusp 위치 오차가 된다.
      θ(s) = th_a + Δθ·s/S,  k = sgn(Δθ)/R,  후진이므로 dP/ds = −(cosθ, sinθ)
      ⇒ P(θ) = cusp − R·sgn(Δθ)·(sinθ − sin th_a,  cos th_a − cos θ)
  """
  gx, gy = goal
  d_theta = wrap(th_g - th_a)
  p1 = (gx + d_str * math.cos(th_g), gy + d_str * math.sin(th_g))
  if abs(d_theta) < 1e-9:
    cusp = (p1[0], p1[1], th_a)
    pts = [p1]
  else:
    sgn = 1.0 if d_theta > 0 else -1.0
    cusp_xy = (p1[0] + R * sgn * (math.sin(th_g) - math.sin(th_a)),
               p1[1] + R * sgn * (math.cos(th_a) - math.cos(th_g)))
    cusp = (cusp_xy[0], cusp_xy[1], th_a)
    S = R * abs(d_theta)
    n = max(2, int(math.ceil(S / ds)))
    pts = []
    for k in range(n + 1):
      th = th_a + d_theta * (k / n)
      pts.append((cusp_xy[0] - R * sgn * (math.sin(th) - math.sin(th_a)),
                  cusp_xy[1] - R * sgn * (math.cos(th_a) - math.cos(th))))

  m = max(1, int(math.ceil(d_str / ds))) if d_str > 1e-6 else 0
  for k in range(1, m + 1):
    t = d_str * (k / m)
    pts.append((p1[0] - t * math.cos(th_g), p1[1] - t * math.sin(th_g)))
  return cusp, pts, d_theta


# ------------------------------------------------------- 키프아웃
def in_slot_box(p, goal, th_g, ahead=0.4, behind=1.6, half_w=0.9):
  """점 p 가 주차칸 상자 안인가.

  칸 입구는 +th_g 쪽이다(차가 그쪽에서 후진으로 들어온다). goal 을 원점으로
  +th_g 방향 ahead 까지, 반대쪽 behind 까지, 좌우 half_w 를 칸으로 본다.

  ★ ahead 를 크게 잡으면 안 된다. 이 시험장은 주차 목표점이 주행차선에서
    1.9m 밖에 안 떨어져 있어서, 상자를 2m 넘게 내밀면 **차선 자체가 금지구역**
    이 되어 어떤 경로도 못 만든다(2026-08-25 에 실제로 전 후보가 탈락했다).
    여기서 막고 싶은 것은 '칸 안으로 전진해 들어가는 것' 하나뿐이므로
    goal 뒤쪽(칸 안쪽)만 넉넉히 잡고 앞쪽은 짧게 둔다.
  """
  dx, dy = p[0] - goal[0], p[1] - goal[1]
  c, s = math.cos(th_g), math.sin(th_g)
  along = dx * c + dy * s
  lat = -dx * s + dy * c
  return (-behind <= along <= ahead) and abs(lat) <= half_w


# ------------------------------------------------------- 본체
def plan_parking(start, goal, th_g, *,
                 r_fwd=3.0,
                 r_rev=(2.9, 5.0, 0.35),
                 d_str=(0.3, 1.5, 0.4),
                 d_app=(0.0, 3.0, 1.0),
                 d_back=(0.0, 3.0, 0.75),
                 dtheta_deg=(30.0, 150.0, 5.0),
                 spacing=0.2,
                 corridor=None,
                 max_cusp_off=7.0,
                 other_goals=(),
                 min_forward=1.5,
                 max_forward=14.0,
                 back_penalty=1.6,
                 refine=True):
  """지금 자세 → 주차 완료 자세 궤적. 해가 없으면 None.

  구조 (최대 3구간, 기어전환점 2개):

    start ──후진직선 d_back──→ ──전진 Dubins(R=r_fwd)──→ 접근선 ──직선 d_app──→
    cusp ──후진 원호 R_rev──→ ──후진 직선 d_str──→ goal

  ★ 왜 맨 앞에 '후진 직선' 이 필요한가 (자리1·2 를 막고 있던 것)
    완주 종점에서 자리1 은 **0.7m 뒤 · 2.2m 오른쪽**, 자리2 는 바로 옆이다.
    후진 진입의 기어전환점(cusp)은 목표점에서 비스듬히 2.4m 앞·1.1m 옆에 생기는데,
    최소반경 3m 로 옆으로 1.1m 를 밀려면 직진거리 3.4m 가 필요하다. 2.4m 밖에
    없으니 전진만으로는 절대 못 붙는다 — Dubins 는 해가 항상 있으므로 대신
    **16m 짜리 큰 원**을 그려버렸다(실측).
    사람이 하듯 **먼저 조금 뒤로 물러서면** 활주로가 그만큼 늘어난다. 물러서는
    구간은 방금 지나온 트랙 위라 비어 있는 것이 확실하다.
    필요할 때만 쓰도록 길이에 벌점(back_penalty)을 준다.

  ★ 왜 이렇게까지 하나
    예전 방식은 '미리 만든 궤적에서 제일 가까운 점을 찾아 이어 가기' 였다.
    위치는 0.1~0.4m 로 가까워도 그 점의 궤적 자세와 차 자세가 58~78° 어긋나
    매번 ABORT 했다. 여기서는 전진부가 Dubins 라 **끝 자세까지 정확히** 맞으므로
    '헤딩이 어긋나 못 붙는' 문제가 원리적으로 없다.
  """
  dt_lo, dt_hi, dt_st = dtheta_deg
  n_dt = int(round((dt_hi - dt_lo) / dt_st)) + 1
  c0, s0 = math.cos(start[2]), math.sin(start[2])

  def frange(spec):
    lo, hi, st = spec
    out, v = [], lo
    while v <= hi + 1e-9:
      out.append(round(v, 4))
      v += st
    return out

  backs = frange(d_back)
  apps = frange(d_app)

  scored = []
  for sign in (+1.0, -1.0):
    for i in range(n_dt):
      th_a = wrap(th_g - sign * math.radians(dt_lo + i * dt_st))
      dth = wrap(th_g - th_a)
      if abs(dth) < 1e-9:
        continue
      sg = 1.0 if dth > 0 else -1.0
      ca, sa = math.cos(th_a), math.sin(th_a)
      for r in frange(r_rev):
        for ds_ in frange(d_str):
          p1x = goal[0] + ds_ * math.cos(th_g)
          p1y = goal[1] + ds_ * math.sin(th_g)
          cx = p1x + r * sg * (math.sin(th_g) - sa)
          cy = p1y + r * sg * (ca - math.cos(th_g))
          if corridor:
            d_corr = min(math.hypot(cx - a, cy - b) for (a, b) in corridor)
            if d_corr > max_cusp_off:
              continue
          else:
            d_corr = 0.0
          rev_len = r * abs(dth) + ds_
          for da in apps:
            prex, prey = cx - da * ca, cy - da * sa
            for db in backs:
              st_ = (start[0] - db * c0, start[1] - db * s0, start[2])
              dl = dubins_length(st_, (prex, prey, th_a), r_fwd)
              if dl is None:
                continue
              fwd = dl + da
              if fwd < min_forward or fwd > max_forward:
                continue
              cost = (fwd + 1.2 * rev_len + 0.6 * d_corr
                      - 0.25 * min(r, 4.5) + back_penalty * db)
              scored.append((cost, th_a, r, ds_, da, db))

  if not scored:
    return None
  scored.sort(key=lambda z: z[0])

  def build(th_a, r, ds_, da, db):
    cusp, rev_pts, dth = reverse_entry(goal, th_g, th_a, r, ds_, ds=spacing)
    pre = (cusp[0] - da * math.cos(th_a),
           cusp[1] - da * math.sin(th_a), th_a)
    st_ = (start[0] - db * c0, start[1] - db * s0, start[2])
    got = dubins(st_, pre, r_fwd, ds=spacing)
    if got is None:
      return None
    dl, fwd_pts, word = got
    fwd_pts = list(fwd_pts)
    n = int(math.ceil(da / spacing)) if da > 1e-6 else 0
    for k in range(1, n + 1):
      t = da * (k / n)
      fwd_pts.append((pre[0] + t * math.cos(th_a),
                      pre[1] + t * math.sin(th_a)))

    back_pts = []
    if db > 1e-6:
      m = int(math.ceil(db / spacing))
      back_pts = [(start[0] - db * (k / m) * c0,
                   start[1] - db * (k / m) * s0) for k in range(m + 1)]

    # 키프아웃: 어느 구간도 주차칸을 가로질러선 안 된다.
    # (내 칸은 전진으로 들어가면 안 되고, 남의 칸은 아예 밟으면 안 된다)
    for p_ in back_pts + fwd_pts:
      if in_slot_box(p_, goal, th_g):
        return None
    for p_ in back_pts + fwd_pts + rev_pts:
      for (g_, t_) in other_goals:
        if in_slot_box(p_, g_, t_):
          return None

    pts = [(x, y, -1) for (x, y) in back_pts]
    pts += [(x, y, 1) for (x, y) in (fwd_pts[1:] if back_pts else fwd_pts)]
    pts += [(x, y, -1) for (x, y) in rev_pts[1:]]
    return {
        'points': pts, 'cusp': cusp, 'th_a': th_a, 'r_rev': r,
        'd_str': ds_, 'd_app': da, 'd_back': db, 'fwd_len': dl + da,
        'rev_len': r * abs(dth) + ds_, 'turn_deg': math.degrees(dth),
        'word': word, 'r_fwd': r_fwd, 'goal': (goal[0], goal[1], th_g),
    }

  chosen = None
  for cand in scored[:120]:
    got = build(*cand[1:])
    if got is not None:
      chosen = (got, cand[1:])
      break
  if chosen is None:
    return None

  best, (th_a, r, ds_, da, db) = chosen
  if refine:
    def sc(z):
      return z['fwd_len'] + 1.2 * z['rev_len'] + back_penalty * z['d_back']
    score = sc(best)
    for dth_off in (-3.0, -1.5, 0.0, 1.5, 3.0):
      for dr in (-0.15, 0.0, 0.15):
        for dd in (-0.2, 0.0, 0.2):
          for dda in (-0.5, 0.0, 0.5):
            for ddb in (-0.35, 0.0, 0.35):
              r2, d2 = r + dr, ds_ + dd
              a2, b2 = da + dda, db + ddb
              if not (r_rev[0] <= r2 <= r_rev[1]):
                continue
              if d2 < d_str[0] or a2 < 0.0 or b2 < 0.0:
                continue
              c2 = build(wrap(th_a + math.radians(dth_off)), r2, d2, a2, b2)
              if c2 is None:
                continue
              if sc(c2) < score:
                best, score = c2, sc(c2)
  return best


def max_curvature_radius(pts):
  """세 점 외접원으로 최소 회전반경 추정 (실현 가능성 재확인용)."""
  worst = float('inf')
  for i in range(1, len(pts) - 1):
    (x1, y1), (x2, y2), (x3, y3) = pts[i - 1][:2], pts[i][:2], pts[i + 1][:2]
    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
    if area2 < 1e-12 or a * b * c < 1e-12:
      continue
    worst = min(worst, a * b * c / (2.0 * area2))
  return worst

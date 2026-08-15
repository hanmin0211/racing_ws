#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""local_path_core.py — 로컬 경로 생성의 순수 계산부 (ROS 비의존).

ROS 노드(local_sliding_window_node)와 오프라인 검증 하네스가 **같은 코드**를
쓰도록 계산 로직만 분리했다. 이렇게 해야 "하네스에선 통과했는데 실차에선
다르더라" 하는 괴리가 생기지 않는다.

핵심 설계 (2026-08-15 개정):
  · **호길이 매개변수 피팅**: 예전엔 y=f(x) 3차 피팅이었는데, 급코너에서 경로가
    차량 뒤로 말려 들어가면 x가 단조롭지 않아 **수학적으로 표현 불가**했다.
    (실측 하네스: 179지점 중 39지점=22%에서 전제 붕괴)
    → x(s), y(s) 를 각각 s(호길이)의 다항식으로 피팅한다. 어떤 곡률도 표현된다.
  · **열린 경로 지원**: 예전엔 윈도우를 `% total`로 순환시켜 닫힌 루프를 가정했다.
    실제 기록 경로는 시작-끝이 3.82m 벌어진 열린 경로라 이음매에서 경로가 튀었다.
    (하네스: 25회 불연속) → 열린/닫힌을 자동 판정해 열린 경로는 순환하지 않는다.
  · **전방 곡률**: 예전엔 차량 위치(s=0)의 곡률을 발행해 '코너에 이미 진입한 뒤'에야
    커졌다. 선제 감속이 불가능했다 → 전방 일정 거리 구간의 최대 |κ|를 쓴다.
"""

import math

import numpy as np


class LocalPathParams:
  """로컬 경로 생성 파라미터 묶음."""

  def __init__(self, n_back=5, n_forward=20, poly_order=3,
               lookahead_distance=10.0, point_spacing=0.5,
               curvature_preview=4.0, closed_path='auto',
               closed_gap_thresh=2.0, goal_tolerance=1.0):
    self.n_back = n_back
    self.n_forward = n_forward
    self.poly_order = poly_order
    self.lookahead_distance = lookahead_distance
    self.point_spacing = point_spacing
    # 곡률을 앞쪽 몇 m 구간에서 볼지 (선제 감속용)
    self.curvature_preview = curvature_preview
    # 'auto' | True | False
    self.closed_path = closed_path
    self.closed_gap_thresh = closed_gap_thresh
    # 열린 경로에서 끝까지 남은 거리가 이 값 이하면 완주로 본다[m]
    self.goal_tolerance = goal_tolerance


def is_closed_path(wps, thresh=2.0):
  """시작점과 끝점이 가까우면 닫힌 루프로 본다."""
  if len(wps) < 3:
    return False
  return float(np.hypot(wps[0, 0] - wps[-1, 0],
                        wps[0, 1] - wps[-1, 1])) < thresh


def closest_index(wps, cx, cy, prev_idx=None, search_span=40):
  """차량에서 가장 가까운 웨이포인트 인덱스.

  prev_idx가 주어지면 그 주변만 국소 탐색한다. 경로가 자기 근처를 지나가는
  구간(교차·인접 차선)에서 엉뚱한 곳으로 튀는 것을 막는다.
  """
  n = len(wps)
  if prev_idx is None:
    d = np.hypot(wps[:, 0] - cx, wps[:, 1] - cy)
    return int(np.argmin(d))
  lo = max(0, prev_idx - search_span // 4)
  hi = min(n, prev_idx + search_span)
  seg = wps[lo:hi]
  d = np.hypot(seg[:, 0] - cx, seg[:, 1] - cy)
  return int(lo + np.argmin(d))


def window_indices(total, ci, p, closed):
  """슬라이딩 윈도우 인덱스. 열린 경로면 순환하지 않고 끝에서 잘라낸다."""
  win = min(p.n_back + p.n_forward + 1, total)
  if closed:
    return [(ci - p.n_back + i) % total for i in range(win)]
  lo = max(0, ci - p.n_back)
  hi = min(total, lo + win)
  lo = max(0, hi - win)          # 끝에 붙었으면 뒤로 당겨 개수 유지
  return list(range(lo, hi))


def build_local_path(wps, cx, cy, cyaw, p=None, prev_idx=None, closed=None):
  """차량 기준(base_link) 로컬 경로를 만든다.

  반환 dict:
    points      : [(x, y), ...] 차량기준 미래점 (없으면 None)
    curvature   : 전방 구간 최대 |κ| 에 부호를 붙인 값
    closest     : 사용한 최근접 인덱스
    remaining   : 경로 끝까지 남은 점 개수 (열린 경로 완주 판정용)
    coeffs      : (cx_coeffs, cy_coeffs) 진단용
    ok          : 유효한 경로가 나왔는가
  """
  p = p or LocalPathParams()
  total = len(wps)
  out = {'points': None, 'curvature': 0.0, 'closest': 0,
         'remaining': total, 'coeffs': None, 'ok': False,
         'goal_reached': False, 'remaining_dist': float('inf')}
  if total < p.poly_order + 2:
    return out

  if closed is None:
    closed = (is_closed_path(wps, p.closed_gap_thresh)
              if p.closed_path == 'auto' else bool(p.closed_path))

  ci = closest_index(wps, cx, cy, prev_idx)
  out['closest'] = ci
  out['remaining'] = total if closed else (total - 1 - ci)

  # 열린 경로 완주 판정: 끝점까지 남은 거리가 goal_tolerance 이내면 도착.
  # (닫힌 루프는 끝이 없으므로 판정하지 않는다)
  if not closed:
    rem = float(np.sum(np.hypot(np.diff(wps[ci:, 0]), np.diff(wps[ci:, 1]))))
    rem += float(np.hypot(wps[ci, 0] - cx, wps[ci, 1] - cy))
    out['remaining_dist'] = rem
    if rem <= p.goal_tolerance:
      out['goal_reached'] = True
      return out

  idx = window_indices(total, ci, p, closed)
  w = wps[idx]
  if len(w) <= p.poly_order:
    return out

  # --- 차량 기준 좌표로 변환 (정면 = +x) ---
  dx = w[:, 0] - cx
  dy = w[:, 1] - cy
  c, s = math.cos(cyaw), math.sin(cyaw)
  xl = dx * c + dy * s
  yl = -dx * s + dy * c

  # --- 호길이 s 계산 (경로 순서 그대로. 정렬하지 않는다) ---
  seg = np.hypot(np.diff(xl), np.diff(yl))
  sv = np.concatenate(([0.0], np.cumsum(seg)))
  if sv[-1] < 1e-6:
    return out

  # 차량 현재 위치에 해당하는 s (윈도우 시작점 기준). 과거점 n_back 개 뒤에 있다.
  # 실제 차량은 x=0,y=0 이므로 그 지점에 가장 가까운 s를 원점으로 삼는다.
  s0 = float(sv[int(np.argmin(np.hypot(xl, yl)))])

  # --- x(s), y(s) 각각 다항식 피팅 ---
  # y=f(x) 와 달리 s는 항상 단조증가하므로 어떤 곡률에서도 성립한다.
  order = min(p.poly_order, len(sv) - 1)
  try:
    fx = np.polyfit(sv, xl, order)
    fy = np.polyfit(sv, yl, order)
  except Exception:  # noqa: BLE001
    return out
  px, py = np.poly1d(fx), np.poly1d(fy)
  dpx, dpy = np.polyder(px), np.polyder(py)
  ddpx, ddpy = np.polyder(px, 2), np.polyder(py, 2)

  def kappa(s_val):
    """매개변수 곡선의 곡률 κ = (x'y'' − y'x'') / (x'² + y'²)^1.5 (부호 포함)."""
    xp, yp = float(dpx(s_val)), float(dpy(s_val))
    xpp, ypp = float(ddpx(s_val)), float(ddpy(s_val))
    den = (xp * xp + yp * yp) ** 1.5
    if den < 1e-9:
      return 0.0
    return (xp * ypp - yp * xpp) / den

  # --- 전방 구간 최대 |κ| (선제 감속용) ---
  # ★ 곡률은 경로점 피팅과 **분리해서** 계산한다. 경로 생성용 피팅은 12m 넘는
  #   넓은 윈도우라 급코너가 뭉개져 곡률이 실제의 절반 이하로 나온다(하네스 실측).
  #   곡률만은 전방 preview 구간의 점들로 저차 다항식을 따로 맞춰 계산한다.
  #   (원시 3점 곡률은 0.5m 간격·cm 노이즈에서 κ 노이즈가 0.1대라 그대로는 못 쓴다)
  s_end = min(sv[-1], s0 + max(p.curvature_preview, 2 * p.point_spacing))
  sel = (sv >= s0 - 1e-9) & (sv <= s_end + 1e-9)
  curvature = 0.0
  if int(np.count_nonzero(sel)) >= 4:
    ss, sx, sy = sv[sel], xl[sel], yl[sel]
    k_order = min(2, len(ss) - 1)     # 짧은 구간엔 2차면 충분(원호 근사)
    try:
      cfx = np.poly1d(np.polyfit(ss, sx, k_order))
      cfy = np.poly1d(np.polyfit(ss, sy, k_order))
      d1x, d1y = np.polyder(cfx), np.polyder(cfy)
      d2x, d2y = np.polyder(cfx, 2), np.polyder(cfy, 2)
      ks = []
      for v in np.linspace(ss[0], ss[-1], 10):
        xp, yp = float(d1x(v)), float(d1y(v))
        xpp, ypp = float(d2x(v)), float(d2y(v))
        den = (xp * xp + yp * yp) ** 1.5
        if den > 1e-9:
          ks.append((xp * ypp - yp * xpp) / den)
      if ks:
        curvature = max(ks, key=abs)
    except Exception:  # noqa: BLE001
      curvature = kappa(s0)
  else:
    curvature = kappa(s0)
  out['curvature'] = float(curvature)

  # --- 미래점 샘플링: 차량 위치(s0)부터 전방으로 point_spacing 간격 ---
  n_pts = int(round(p.lookahead_distance / p.point_spacing))
  targets = s0 + np.arange(0, n_pts + 1) * p.point_spacing
  targets = targets[targets <= sv[-1]]
  if len(targets) < 2:
    return out

  pts = [(float(px(v)), float(py(v))) for v in targets]
  out['points'] = pts
  out['coeffs'] = (fx, fy)
  out['ok'] = True
  return out

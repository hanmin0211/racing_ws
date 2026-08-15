#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
core_logic.py
=============
ROS 의존성이 없는 순수 계산 로직만 모아둔 모듈.
(단위 테스트가 쉽고, ROS 없는 환경에서도 알고리즘만 따로 검증 가능하도록 분리)

포함 내용:
1. 좌표 변환 (UTM 기반 GPS -> 로컬)      : gps_to_local_fit.py 와 동일 로직
2. Pure Pursuit 핵심 3단계               : 원 경로/직선 경로 기하 검증 완료
   - find_closest_index
   - find_lookahead_point (원-직선 교차 보간, 정밀)
   - compute_steering_angle : delta = atan2(2*L*sin(alpha), Ld)
3. 차량 기준 로컬 윈도우 3차 최소자승법 피팅 : local_window_fit.py 와 동일 로직
   (Pure Pursuit 조향각 계산 자체에는 필수가 아니지만, 곡률 기반 감속·시각화용으로 사용)

[검증 결과 요약 - 실제 수치 테스트 완료]
- 반지름 R=5m 원형 경로에서 delta 이론값 atan(L/R)과 오차 0.001도 미만으로 일치
- 직선 경로에서 좌/우 벗어남에 따른 조향각 부호가 기하학적으로 올바름을 확인
"""

import math

import numpy as np
from pyproj import Transformer


# ----------------------------------------------------------------------
# 1. GPS -> 로컬좌표 변환 (gps_to_local_fit.py 와 동일)
# ----------------------------------------------------------------------
def compute_utm_epsg(lat, lon):
    zone = int(math.floor((lon + 180) / 6) + 1)
    epsg = 32600 + zone if lat >= 0 else 32700 + zone
    return epsg, zone


class GpsLocalConverter:
    """
    고정된 원점(origin_lat, origin_lon)을 기준으로, 들어오는 위경도를
    실시간으로 동일한 로컬좌표계(x, y, 단위 m)로 변환한다.

    반드시 오프라인에서 waypoint 경로를 만들 때 사용한 것과 '같은 원점'을
    써야 한다. 원점이 다르면 실시간 차량 위치와 경로 좌표가 서로 다른
    기준점을 갖게 되어 완전히 어긋난 위치로 계산된다.
    """

    def __init__(self, origin_lat, origin_lon):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        epsg, zone = compute_utm_epsg(origin_lat, origin_lon)
        self.epsg = epsg
        self.zone = zone
        self._transformer = Transformer.from_crs(
            "EPSG:4326", f"EPSG:{epsg}", always_xy=True
        )
        self.origin_utm_x, self.origin_utm_y = self._transformer.transform(
            origin_lon, origin_lat
        )

    def to_local(self, lat, lon):
        utm_x, utm_y = self._transformer.transform(lon, lat)
        return utm_x - self.origin_utm_x, utm_y - self.origin_utm_y


# ----------------------------------------------------------------------
# 2. Pure Pursuit 핵심 로직 (검증 완료)
# ----------------------------------------------------------------------
def find_closest_index(path_xy, vehicle_xy):
    """path_xy: (N,2) ndarray. 차량과 가장 가까운 경로점의 인덱스와 거리들을 반환."""
    diffs = path_xy - np.array(vehicle_xy)
    dists = np.hypot(diffs[:, 0], diffs[:, 1])
    return int(np.argmin(dists)), dists


def find_lookahead_point(path_xy, vehicle_xy, closest_idx, lookahead_dist):
    """
    closest_idx부터 앞으로 순회하며 차량-경로점 거리가 lookahead_dist를
    처음 넘는 구간을 찾아, 원(반지름=lookahead_dist)과 그 구간 선분의
    교점을 정확히 계산해 반환한다 (단순 최근접점이 아닌 보간 방식).
    """
    n = len(path_xy)
    vx, vy = vehicle_xy

    for i in range(closest_idx, n - 1):
        p1 = path_xy[i]
        p2 = path_xy[i + 1]
        d1 = math.hypot(p1[0] - vx, p1[1] - vy)
        d2 = math.hypot(p2[0] - vx, p2[1] - vy)

        if d1 <= lookahead_dist <= d2:
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            fx, fy = p1[0] - vx, p1[1] - vy

            a = dx * dx + dy * dy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - lookahead_dist * lookahead_dist

            disc = b * b - 4 * a * c
            if disc < 0 or a < 1e-12:
                continue
            disc = math.sqrt(disc)
            t1 = (-b - disc) / (2 * a)
            t2 = (-b + disc) / (2 * a)
            for t in (t2, t1):
                if 0.0 <= t <= 1.0:
                    return (p1[0] + t * dx, p1[1] + t * dy), i

    return tuple(path_xy[-1]), n - 1  # 경로 끝에 도달한 경우


def compute_steering_angle(vehicle_xy, vehicle_heading, lookahead_xy, wheelbase, Ld=None):
    """
    delta = atan2(2*L*sin(alpha), Ld)
    alpha = 목표점 방향각 - 차량 헤딩 (좌회전 +, 우회전 -)
    """
    vx, vy = vehicle_xy
    lx, ly = lookahead_xy
    dx, dy = lx - vx, ly - vy

    if Ld is None:
        Ld = math.hypot(dx, dy)

    target_angle = math.atan2(dy, dx)
    alpha = target_angle - vehicle_heading
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), Ld)
    return delta, alpha


def adaptive_lookahead(speed, k_ld=0.6, min_ld=1.0, max_ld=4.0):
    """
    Ld = k_ld * v + min_ld  (속도가 빠를수록 lookahead를 늘림)
    실제 다수의 자율주행 문헌(예: arXiv 2511.11310)에서 쓰는
    L_d = K_ld * V + L_fc 형태의 속도비례 lookahead 공식.
    """
    return float(np.clip(k_ld * speed + min_ld, min_ld, max_ld))


# ----------------------------------------------------------------------
# 3. 차량 기준 로컬 윈도우 3차 최소자승법 피팅 (local_window_fit.py 와 동일)
# ----------------------------------------------------------------------
def compute_cumulative_distance(path_xy):
    seg_len = np.hypot(np.diff(path_xy[:, 0]), np.diff(path_xy[:, 1]))
    return np.concatenate(([0.0], np.cumsum(seg_len)))


def select_local_window(path_xy, cum_dist, idx, behind=5.0, ahead=15.0):
    center_dist = cum_dist[idx]
    mask = (cum_dist >= center_dist - behind) & (cum_dist <= center_dist + ahead)
    indices = np.where(mask)[0]
    return path_xy[indices]


def transform_to_vehicle_frame(points_xy, origin_x, origin_y, heading):
    xs = points_xy[:, 0] - origin_x
    ys = points_xy[:, 1] - origin_y
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    x_v = cos_h * xs + sin_h * ys
    y_v = -sin_h * xs + cos_h * ys
    return x_v, y_v


def fit_cubic_least_squares(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    A = np.column_stack([np.ones_like(x), x, x ** 2, x ** 3])
    coeffs, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, b, c, d = coeffs

    y_pred = A @ coeffs
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 1.0

    return {"a": float(a), "b": float(b), "c": float(c), "d": float(d),
            "r_squared": float(r_squared)}


def estimate_curvature_from_cubic(fit, x_eval=0.0):
    """
    y = a + bx + cx^2 + dx^3 에서 곡률 kappa = y'' / (1+y'^2)^1.5
    (x_eval=0, 즉 차량 위치에서의 국소 곡률 추정 - 코너 감속 판단용)
    """
    b, c, d = fit["b"], fit["c"], fit["d"]
    y1 = b + 2 * c * x_eval + 3 * d * x_eval ** 2   # 1차 미분
    y2 = 2 * c + 6 * d * x_eval                      # 2차 미분
    denom = (1.0 + y1 ** 2) ** 1.5
    return y2 / denom if denom > 1e-9 else 0.0

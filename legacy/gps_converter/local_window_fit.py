#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_window_fit.py
=====================
전체 트랙을 통째로 3차 다항식으로 피팅하면(특히 폐루프/커브가 많은 트랙)
R^2가 매우 낮게 나온다. 실제 자율주행 경로추종 제어기(Pure Pursuit,
Stanley, MPC 등)는 트랙 전체를 한 번에 피팅하지 않고, 아래 방식을 쓴다:

    매 제어 주기마다
    1) 차량 현재 위치에서 가장 가까운 waypoint를 찾고
    2) 그 지점 기준 앞/뒤 일정 구간(window, 예: 뒤 5m ~ 앞 15m)만 잘라서
    3) 차량 진행방향(heading)을 x축으로 하는 "차량 기준 좌표계"로 회전/
       평행이동 변환한 뒤
    4) 그 좁은 구간에서만 y_local = a + b*x_local + c*x_local^2 + d*x_local^3
       를 최소자승법으로 피팅한다.

이렇게 하면 좁은 구간 안에서는 경로가 급격히 꺾이지 않는 한 x_local이
항상 단조증가하므로(차량이 진행하는 방향으로만 좌표가 커짐) 3차 다항식
피팅이 항상 잘 맞는다. 이는 Apollo, Autoware 계열 로컬 경로 스무딩,
전통적인 차선(lane) 모델 피팅(y = a0+a1*x+a2*x^2+a3*x^3)에서 공통적으로
쓰이는 방식이다.

[핵심 함수]
- estimate_heading_at_index : 인접 waypoint 간 방향벡터로 현재 지점의
  진행방향(heading)을 추정 (yaw 필드가 없을 때 사용하는 표준적인 방법)
- select_local_window        : 누적거리 기준으로 차량 앞/뒤 구간만 추출
- transform_to_vehicle_frame : 회전행렬로 전역좌표 -> 차량기준좌표 변환
- fit_cubic_least_squares    : 정규방정식 기반 최소자승법(SVD, lstsq)

[사용 예시]
  # 특정 인덱스(가상의 현재 차량 위치)에서 한 번만 피팅해서 확인
  python3 local_window_fit.py --input waypoints_local.yaml \
      --vehicle-index 50 --ahead 15 --behind 5

  # 트랙 전체를 처음부터 끝까지 시뮬레이션(매 지점마다 재피팅)하며
  # R^2 통계를 내서 이 방식이 전체 트랙에서 안정적으로 잘 맞는지 검증
  python3 local_window_fit.py --input waypoints_local.yaml \
      --demo --ahead 15 --behind 5
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml


# ----------------------------------------------------------------------
# 1. 로드
# ----------------------------------------------------------------------
def load_local_waypoints(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if isinstance(data, dict):
        waypoints = data.get("waypoints")
        if waypoints is None:
            raise ValueError("[오류] YAML에 'waypoints' 키가 없습니다.")
    elif isinstance(data, list):
        waypoints = data
    else:
        raise ValueError("[오류] 지원하지 않는 YAML 구조입니다.")

    for i, wp in enumerate(waypoints):
        if "x" not in wp or "y" not in wp:
            raise ValueError(f"[오류] {i}번째 waypoint에 x/y 가 없습니다: {wp}")
    return waypoints


# ----------------------------------------------------------------------
# 2. 누적거리 계산 (resample_waypoints.py 와 동일한 방식)
# ----------------------------------------------------------------------
def compute_cumulative_distance(waypoints):
    xs = np.array([wp["x"] for wp in waypoints], dtype=float)
    ys = np.array([wp["y"] for wp in waypoints], dtype=float)
    seg_len = np.hypot(np.diff(xs), np.diff(ys))
    return np.concatenate(([0.0], np.cumsum(seg_len)))


# ----------------------------------------------------------------------
# 3. 진행방향(heading) 추정
# ----------------------------------------------------------------------
def estimate_heading_at_index(waypoints, idx):
    """
    waypoint에 yaw 필드가 있으면 그대로 사용하고, 없으면 앞/뒤 인접
    waypoint를 잇는 벡터의 방향(atan2)으로 진행방향을 추정한다.
    이는 heading 센서가 없을 때 흔히 쓰는 표준적인 근사법이다.
    """
    wp = waypoints[idx]
    if "yaw" in wp:
        return float(wp["yaw"])

    n = len(waypoints)
    i_prev = max(idx - 1, 0)
    i_next = min(idx + 1, n - 1)
    dx = waypoints[i_next]["x"] - waypoints[i_prev]["x"]
    dy = waypoints[i_next]["y"] - waypoints[i_prev]["y"]
    return math.atan2(dy, dx)


# ----------------------------------------------------------------------
# 4. 로컬 윈도우 선택 (누적거리 기준 앞/뒤 구간)
# ----------------------------------------------------------------------
def select_local_window(waypoints, cum_dist, idx, behind=5.0, ahead=15.0):
    center_dist = cum_dist[idx]
    lo = center_dist - behind
    hi = center_dist + ahead
    mask = (cum_dist >= lo) & (cum_dist <= hi)
    indices = np.where(mask)[0]
    return [waypoints[i] for i in indices]


# ----------------------------------------------------------------------
# 5. 전역좌표 -> 차량 기준 좌표 변환
# ----------------------------------------------------------------------
def transform_to_vehicle_frame(local_wps, origin_x, origin_y, heading):
    """
    차량 위치(origin_x, origin_y)를 원점으로, 차량 진행방향(heading)을
    x축으로 하는 좌표계로 평행이동 + 회전 변환한다.

        [x_v]   [ cos(h)  sin(h)] [x - ox]
        [y_v] = [-sin(h)  cos(h)] [y - oy]

    (heading만큼 반대로 회전시켜 진행방향이 항상 +x축이 되도록 정렬)
    """
    xs = np.array([wp["x"] for wp in local_wps], dtype=float) - origin_x
    ys = np.array([wp["y"] for wp in local_wps], dtype=float) - origin_y

    cos_h, sin_h = math.cos(heading), math.sin(heading)
    x_v = cos_h * xs + sin_h * ys
    y_v = -sin_h * xs + cos_h * ys
    return x_v, y_v


# ----------------------------------------------------------------------
# 6. 3차 다항식 최소자승법 피팅 (resample/gps_to_local_fit 과 동일 원리)
# ----------------------------------------------------------------------
def fit_cubic_least_squares(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    A = np.column_stack([np.ones_like(x), x, x ** 2, x ** 3])
    coeffs, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, b, c, d = coeffs

    y_pred = A @ coeffs
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 1.0  # 점이 거의 안 흩어지면 완전 피팅으로 간주

    return {"a": float(a), "b": float(b), "c": float(c), "d": float(d),
            "r_squared": float(r_squared)}


# ----------------------------------------------------------------------
# 7. 특정 위치에서 한 번 피팅
# ----------------------------------------------------------------------
def fit_at_vehicle_index(waypoints, cum_dist, idx, behind, ahead):
    vehicle = waypoints[idx]
    heading = estimate_heading_at_index(waypoints, idx)
    window = select_local_window(waypoints, cum_dist, idx, behind, ahead)

    if len(window) < 4:
        return None  # 3차 다항식은 최소 4점 필요

    x_v, y_v = transform_to_vehicle_frame(window, vehicle["x"], vehicle["y"], heading)

    order = np.argsort(x_v)
    x_v, y_v = x_v[order], y_v[order]

    fit = fit_cubic_least_squares(x_v, y_v)
    fit["n_points"] = len(window)
    fit["heading_rad"] = heading
    return fit


def fit_at_vehicle_index_adaptive(waypoints, cum_dist, idx, behind, ahead,
                                   min_ahead=3.0, r2_target=0.9, shrink_steps=5):
    """
    기본 ahead 거리로 피팅했을 때 R^2가 r2_target 미만이면, 급커브 구간일
    가능성이 높다고 보고 ahead 거리를 단계적으로 줄여가며 재시도한다.
    (Pure Pursuit 등에서 곡률이 큰 구간일수록 lookahead distance를 줄이는
    적응형 lookahead 기법과 동일한 아이디어)

    r2_target에 도달하면 그 시점의 결과를 반환하고, 끝까지 못 미치면
    min_ahead 로 시도한 마지막 결과를 반환한다.
    """
    ahead_candidates = np.linspace(ahead, min_ahead, shrink_steps)
    best_fit = None
    used_ahead = ahead

    for a in ahead_candidates:
        fit = fit_at_vehicle_index(waypoints, cum_dist, idx, behind, float(a))
        if fit is None:
            continue
        best_fit = fit
        used_ahead = float(a)
        if fit["r_squared"] >= r2_target:
            break

    if best_fit is not None:
        best_fit["used_ahead"] = used_ahead
    return best_fit


# ----------------------------------------------------------------------
# 8. 메인
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="차량 기준 로컬 윈도우에서 3차 다항식을 최소자승법으로 "
                     "피팅합니다 (실제 경로추종 제어기 방식)."
    )
    parser.add_argument("--input", "-i", required=True,
                         help="입력 로컬좌표(x,y[,yaw]) waypoint YAML 파일")
    parser.add_argument("--vehicle-index", type=int, default=None,
                         help="차량 현재 위치로 간주할 waypoint 인덱스 "
                              "(생략 시 --demo 필요)")
    parser.add_argument("--behind", type=float, default=5.0,
                         help="차량 뒤쪽으로 포함할 거리 [m] (기본 5.0)")
    parser.add_argument("--ahead", type=float, default=15.0,
                         help="차량 앞쪽으로 포함할 거리 [m] (기본 15.0)")
    parser.add_argument("--demo", action="store_true",
                         help="트랙 전체를 처음부터 끝까지 시뮬레이션하며 "
                              "각 지점에서 재피팅, R^2 통계 출력")
    parser.add_argument("--demo-stride", type=int, default=5,
                         help="--demo 시 몇 개마다 한 번씩 피팅할지 (기본 5, "
                              "전체를 다 하면 느려질 수 있어 샘플링)")
    parser.add_argument("--adaptive", action="store_true",
                         help="R^2가 낮으면(급커브 등) --ahead 를 자동으로 "
                              "줄여가며 재시도하는 적응형 윈도우 사용")
    parser.add_argument("--r2-target", type=float, default=0.9,
                         help="--adaptive 사용 시 목표 R^2 (기본 0.9)")
    parser.add_argument("--min-ahead", type=float, default=3.0,
                         help="--adaptive 사용 시 줄일 수 있는 최소 ahead 거리 [m] (기본 3.0)")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        raise FileNotFoundError(f"[오류] 입력 파일을 찾을 수 없습니다: {args.input}")

    waypoints = load_local_waypoints(args.input)
    n = len(waypoints)
    cum_dist = compute_cumulative_distance(waypoints)
    print(f"[로드] '{args.input}' 에서 waypoint {n}개 로드 (총 길이 "
          f"{cum_dist[-1]:.2f} m)")

    if not args.demo and args.vehicle_index is None:
        raise ValueError("[오류] --vehicle-index 를 지정하거나 --demo 를 사용하세요.")

    if args.demo:
        r2_list = []
        low_r2_points = []  # (index, x, y, r2) - 문제 지점 진단용
        fail_count = 0
        for idx in range(0, n, args.demo_stride):
            if args.adaptive:
                fit = fit_at_vehicle_index_adaptive(
                    waypoints, cum_dist, idx, args.behind, args.ahead,
                    min_ahead=args.min_ahead, r2_target=args.r2_target)
            else:
                fit = fit_at_vehicle_index(waypoints, cum_dist, idx, args.behind, args.ahead)
            if fit is None:
                fail_count += 1
                continue
            r2_list.append(fit["r_squared"])
            if fit["r_squared"] < 0.9:
                low_r2_points.append((idx, waypoints[idx]["x"], waypoints[idx]["y"],
                                       fit["r_squared"], cum_dist[idx]))

        r2_arr = np.array(r2_list)
        print("\n========== 전체 트랙 로컬 윈도우 피팅 검증 (--demo) ==========")
        print(f"윈도우 설정               : 뒤 {args.behind} m / 앞 {args.ahead} m")
        print(f"검사한 지점 수            : {len(r2_arr)}  "
              f"(포인트 부족으로 스킵된 지점: {fail_count})")
        print(f"R^2 평균                  : {r2_arr.mean():.6f}")
        print(f"R^2 최소값                : {r2_arr.min():.6f}")
        print(f"R^2 최대값                : {r2_arr.max():.6f}")
        print(f"R^2 < 0.9 인 지점 수      : {int(np.sum(r2_arr < 0.9))} / {len(r2_arr)}")

        if low_r2_points:
            print(f"\n[진단] R^2 < 0.9 인 문제 지점 목록 (누적거리순 정렬, "
                  f"최대 20개 표시):")
            print(f"{'index':>6} {'누적거리(m)':>12} {'x':>10} {'y':>10} {'R^2':>10}")
            for idx, x, y, r2, s in sorted(low_r2_points, key=lambda t: t[3])[:20]:
                print(f"{idx:>6} {s:>12.2f} {x:>10.3f} {y:>10.3f} {r2:>10.4f}")
            print("  -> 위 지점들은 대개 급커브(회전 반경이 작은 구간)입니다.")
            print("     해당 구간만 --ahead 값을 줄여서(예: 5~8m) 다시 확인해보세요.")
        print("================================================================\n")

    if args.vehicle_index is not None:
        if not (0 <= args.vehicle_index < n):
            raise ValueError(f"[오류] --vehicle-index 는 0~{n-1} 범위여야 합니다.")

        fit = fit_at_vehicle_index(waypoints, cum_dist, args.vehicle_index,
                                    args.behind, args.ahead)
        if fit is None:
            raise ValueError("[오류] 윈도우 내 점이 4개 미만이라 피팅할 수 없습니다. "
                              "--ahead/--behind 값을 늘려보세요.")

        vehicle = waypoints[args.vehicle_index]
        print(f"\n========== 차량 위치(index={args.vehicle_index}) 로컬 피팅 결과 ==========")
        print(f"차량 위치(전역좌표)       : x={vehicle['x']:.3f}, y={vehicle['y']:.3f}")
        print(f"추정 진행방향(heading)    : {math.degrees(fit['heading_rad']):.2f} deg")
        print(f"윈도우 내 점 개수         : {fit['n_points']}  "
              f"(뒤 {args.behind} m / 앞 {args.ahead} m)")
        print(f"y_local = a + b*x + c*x^2 + d*x^3  (차량 기준 좌표계)")
        print(f"  a = {fit['a']:.6f}")
        print(f"  b = {fit['b']:.6f}")
        print(f"  c = {fit['c']:.6f}")
        print(f"  d = {fit['d']:.6f}")
        print(f"  R^2 = {fit['r_squared']:.6f}")
        print("========================================================================\n")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, ValueError) as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

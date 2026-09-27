#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
resample_waypoints.py
======================
자율주행 경로계획(Path Planning)에서 사용하는 Waypoint 파일을
등간격(Arc-length equal spacing)으로 재샘플링하는 스크립트.

여기서 사용하는 방법은 업계 및 자율주행 경진대회(국제대학생 EV 자율주행
경진대회, 미래형자동차 자율주행 경진대회 등)에서 실제로 쓰이는 표준 기법이며,
오픈소스 자율주행 스택인 Autoware의 waypoint resampling 로직과 동일한
원리(누적 호길이(cumulative arc-length) 기반 선형보간)를 사용한다.

    참고 (증명된 공개 자료):
    - Autoware.AI / Autoware.Universe: trajectory/path resample 모듈은
      누적 거리(cumulative distance)를 파라미터로 사용해 선형보간으로
      경로를 재샘플링한다. (github.com/autowarefoundation/autoware)
    - numpy.interp 기반 1차원 선형보간은 단조증가(monotonic) 하는
      누적거리 배열에 대해 수치적으로 안정적이며, 오버슈트가 없어
      경로 데이터 재샘플링에 표준적으로 쓰인다.

핵심 처리 순서
--------------
1. YAML에서 waypoint 목록 로드
2. 중복(또는 사실상 동일한 위치) waypoint 자동 제거
3. 누적 거리(cumulative distance) 계산
4. 지정한 간격(기본 0.5m, 변수로 조정 가능)으로 등간격 재샘플링
   - x, y, z 등 위치값: 선형보간 (np.interp)
   - yaw(heading): unwrap 후 선형보간 -> 각도 wrap-around 문제 방지
     (각도를 그대로 선형보간하면 -pi/+pi 경계에서 값이 튀는 문제가
      발생하므로, np.unwrap으로 연속화한 뒤 보간하고 다시 wrap한다.
      이는 로보틱스/내비게이션 분야에서 각도 보간 시 표준적으로
      쓰이는 방법이다.)
   - 그 외 부가 필드(속도 등 숫자형 필드)도 동일하게 선형보간
5. 결과를 YAML로 저장
6. 재샘플링 전/후 waypoint 개수 출력
7. 재샘플링 전/후 총 경로 길이를 비교하여 경로 길이가 보존되는지 검증

사용 예시
--------
    python3 resample_waypoints.py --input waypoints.yaml \
        --output waypoints_resampled.yaml --interval 0.5
"""

import argparse
import copy
import math
import sys

import numpy as np
import yaml


# ----------------------------------------------------------------------
# 1. 데이터 로드 / 저장
# ----------------------------------------------------------------------
def load_waypoints(path):
    """
    YAML 파일에서 waypoint 리스트를 로드한다.

    지원 포맷:
      (A) 최상위가 waypoint 리스트인 경우
          - [ {x: .., y: ..}, {x: .., y: ..}, ... ]
      (B) 'waypoints' 키 아래에 리스트가 있는 경우
          - waypoints: [ {x: .., y: ..}, ... ]

    각 waypoint 원소는 dict 형태(x, y 필수, z/yaw/velocity 등은 선택)여야 한다.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError(f"[오류] 파일이 비어 있습니다: {path}")

    if isinstance(data, dict):
        if "waypoints" not in data:
            raise ValueError(
                "[오류] YAML에 'waypoints' 키가 없습니다. "
                "waypoints: [ {x:.., y:..}, ... ] 형태인지 확인하세요."
            )
        waypoints = data["waypoints"]
    elif isinstance(data, list):
        waypoints = data
    else:
        raise ValueError("[오류] 지원하지 않는 YAML 최상위 구조입니다.")

    if not waypoints:
        raise ValueError("[오류] waypoint 리스트가 비어 있습니다.")

    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict) or "x" not in wp or "y" not in wp:
            raise ValueError(
                f"[오류] {i}번째 waypoint에 x, y 값이 없습니다: {wp}"
            )

    return waypoints


def save_waypoints(path, waypoints, wrap_in_key=True):
    """재샘플링된 waypoint 리스트를 YAML로 저장한다."""
    out = {"waypoints": waypoints} if wrap_in_key else waypoints
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(out, f, allow_unicode=True, sort_keys=False,
                   default_flow_style=False)


# ----------------------------------------------------------------------
# 2. 중복 waypoint 제거
# ----------------------------------------------------------------------
def remove_duplicate_waypoints(waypoints, dup_tol=1e-3):
    """
    인접한 waypoint 사이의 유클리드 거리가 dup_tol(m) 이하이면
    사실상 동일한 지점으로 간주하고 뒤쪽 점을 제거한다.

    dup_tol 기본값 1mm는 GPS/SLAM 기반 waypoint 로깅 시 흔히 발생하는
    '정지 상태에서 같은 좌표가 중복 기록되는' 문제를 제거하기 위한
    실무적 기준값이다. 필요 시 조정 가능하다.
    """
    cleaned = [waypoints[0]]
    removed = 0

    for wp in waypoints[1:]:
        prev = cleaned[-1]
        dist = math.hypot(wp["x"] - prev["x"], wp["y"] - prev["y"])
        if dist <= dup_tol:
            removed += 1
            continue
        cleaned.append(wp)

    if removed > 0:
        print(f"[중복 제거] 중복(또는 거의 동일 위치) waypoint {removed}개 제거됨 "
              f"(임계값 {dup_tol} m)")

    if len(cleaned) < 2:
        raise ValueError("[오류] 중복 제거 후 남은 waypoint가 2개 미만입니다. "
                          "재샘플링을 진행할 수 없습니다.")

    return cleaned


# ----------------------------------------------------------------------
# 3. 누적 거리 계산
# ----------------------------------------------------------------------
def compute_cumulative_distance(waypoints):
    """
    각 waypoint까지의 누적 거리(arc length) 배열을 계산한다.
    cum_dist[0] = 0.0, cum_dist[-1] = 전체 경로 길이
    """
    xs = np.array([wp["x"] for wp in waypoints], dtype=float)
    ys = np.array([wp["y"] for wp in waypoints], dtype=float)

    seg_len = np.hypot(np.diff(xs), np.diff(ys))
    cum_dist = np.concatenate(([0.0], np.cumsum(seg_len)))
    return cum_dist


def total_path_length(waypoints):
    """전체 경로 길이(누적 거리 마지막 값)를 반환한다."""
    return compute_cumulative_distance(waypoints)[-1]


# ----------------------------------------------------------------------
# 4. 등간격 재샘플링
# ----------------------------------------------------------------------
def resample_waypoints(waypoints, interval=0.5, include_last_point=True):
    """
    누적 거리 기준으로 waypoints를 `interval`(m) 간격으로 재샘플링한다.

    - x, y, z 등 위치 필드: np.interp 로 선형보간
    - yaw(heading) 필드: np.unwrap 으로 각도 연속화 후 선형보간, 이후 다시
      [-pi, pi] 범위로 wrap
    - 그 외 숫자형 필드(velocity 등): 선형보간
    - 문자열/기타 비수치 필드: 가장 가까운 원본 waypoint 값을 그대로 사용
    """
    cum_dist = compute_cumulative_distance(waypoints)
    path_len = cum_dist[-1]

    if interval <= 0:
        raise ValueError("[오류] interval 은 0보다 커야 합니다.")

    # 목표 재샘플링 지점(0, interval, 2*interval, ...)
    sample_dist = np.arange(0.0, path_len, interval)
    if include_last_point and (len(sample_dist) == 0 or
                                path_len - sample_dist[-1] > 1e-9):
        sample_dist = np.append(sample_dist, path_len)

    # 보간 대상 필드 자동 탐지 (x, y 는 항상 포함)
    all_keys = set()
    for wp in waypoints:
        all_keys.update(wp.keys())
    numeric_keys = []
    for k in all_keys:
        if all(isinstance(wp.get(k, 0.0), (int, float)) for wp in waypoints):
            numeric_keys.append(k)
    if "x" not in numeric_keys:
        numeric_keys.append("x")
    if "y" not in numeric_keys:
        numeric_keys.append("y")

    yaw_key = "yaw" if "yaw" in numeric_keys else None

    interp_values = {}
    for k in numeric_keys:
        if k == yaw_key:
            continue
        vals = np.array([wp.get(k, 0.0) for wp in waypoints], dtype=float)
        interp_values[k] = np.interp(sample_dist, cum_dist, vals)

    if yaw_key is not None:
        raw_yaw = np.array([wp[yaw_key] for wp in waypoints], dtype=float)
        unwrapped = np.unwrap(raw_yaw)
        interp_yaw = np.interp(sample_dist, cum_dist, unwrapped)
        # [-pi, pi] 범위로 재정규화
        interp_values[yaw_key] = (interp_yaw + math.pi) % (2 * math.pi) - math.pi

    # 비수치 필드는 가장 가까운 원본 인덱스 값을 사용
    non_numeric_keys = [k for k in all_keys if k not in numeric_keys]
    nearest_idx = np.searchsorted(cum_dist, sample_dist)
    nearest_idx = np.clip(nearest_idx, 0, len(waypoints) - 1)

    resampled = []
    for i in range(len(sample_dist)):
        wp = {}
        for k in numeric_keys:
            wp[k] = float(interp_values[k][i])
        for k in non_numeric_keys:
            wp[k] = waypoints[nearest_idx[i]].get(k)
        resampled.append(wp)

    return resampled


# ----------------------------------------------------------------------
# 5. 검증
# ----------------------------------------------------------------------
def verify_length_preserved(original_len, resampled_len, tol=1e-2, rel_tol=0.02):
    """
    재샘플링 전/후 총 경로 길이를 비교하여 보존 여부를 검증한다.

    판정 기준: 절대오차(tol) 또는 상대오차(rel_tol, 기본 2%) 중
    하나라도 만족하면 통과로 본다.

    [왜 완전히 동일하지 않을 수 있는가]
    원본 waypoint들을 잇는 폴리라인(직선 연결)의 길이는 각 구간의
    직선 거리(chord length)의 합이다. 재샘플링도 동일하게 두 점 사이를
    직선으로 잇기 때문에, 경로가 완전한 직선이면 길이가 정확히 보존된다.
    그러나 경로가 곡선이고 원본 waypoint 밀도가 촘촘한 경우, 원본
    폴리라인은 곡선을 더 잘게 쪼갠 직선들로 근사한 것이므로 실제 곡선
    길이(arc length)에 더 가깝다. 재샘플링 후에는 간격이 넓어지면서
    직선(chord)이 곡선의 코너를 가로질러 절단(corner-cutting)하게 되어
    길이가 항상 원본보다 '같거나 짧게' 나온다. 이는 곡선을 다각선으로
    근사할 때 발생하는 잘 알려진 수치적 특성(chord length <= arc length)
    이며, Autoware 등 실제 자율주행 스택의 path resample 모듈에서도
    동일하게 나타나는 정상적인 현상이다. 간격(interval)을 촘촘히
    할수록 이 차이는 줄어든다.
    """
    diff = abs(original_len - resampled_len)
    rel_diff = diff / original_len if original_len > 0 else 0.0
    preserved = (diff <= tol) or (rel_diff <= rel_tol)
    return preserved, diff, rel_diff


# ----------------------------------------------------------------------
# 6. 메인 실행부
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Waypoint YAML 파일을 등간격으로 재샘플링합니다."
    )
    parser.add_argument("--input", "-i", required=True,
                         help="입력 waypoint YAML 파일 경로")
    parser.add_argument("--output", "-o", required=True,
                         help="출력(재샘플링 결과) YAML 파일 경로")
    parser.add_argument("--interval", "-d", type=float, default=0.5,
                         help="재샘플링 간격 [m] (기본값: 0.5)")
    parser.add_argument("--dup-tol", type=float, default=1e-3,
                         help="중복 waypoint 판정 거리 임계값 [m] (기본값: 0.001)")
    parser.add_argument("--length-tol", type=float, default=1e-2,
                         help="경로 길이 보존 검증 절대 허용오차 [m] (기본값: 0.01)")
    parser.add_argument("--length-rel-tol", type=float, default=0.02,
                         help="경로 길이 보존 검증 상대 허용오차 [비율] "
                              "(기본값: 0.02 = 2%%, 곡선 경로에서 코너 절단 "
                              "효과를 감안한 값)")
    args = parser.parse_args()

    # 1) 로드
    raw_waypoints = load_waypoints(args.input)
    n_before = len(raw_waypoints)
    print(f"[로드] '{args.input}' 에서 waypoint {n_before}개 로드 완료")

    # 2) 중복 제거
    dedup_waypoints = remove_duplicate_waypoints(raw_waypoints, dup_tol=args.dup_tol)
    n_after_dedup = len(dedup_waypoints)

    # 3) 원본(중복 제거 후) 총 길이
    original_length = total_path_length(dedup_waypoints)

    # 4) 재샘플링
    resampled = resample_waypoints(dedup_waypoints, interval=args.interval)
    n_after_resample = len(resampled)

    # 5) 재샘플링 후 총 길이
    resampled_length = total_path_length(resampled)

    # 6) 저장
    save_waypoints(args.output, resampled)

    # 7) 결과 리포트
    print("\n========== 재샘플링 결과 요약 ==========")
    print(f"원본 waypoint 개수        : {n_before}")
    print(f"중복 제거 후 개수         : {n_after_dedup}")
    print(f"재샘플링 후 개수          : {n_after_resample}  (간격 {args.interval} m)")
    print(f"원본 총 경로 길이         : {original_length:.4f} m")
    print(f"재샘플링 후 총 경로 길이  : {resampled_length:.4f} m")

    preserved, diff, rel_diff = verify_length_preserved(
        original_length, resampled_length,
        tol=args.length_tol, rel_tol=args.length_rel_tol
    )
    status = "통과 (PASS)" if preserved else "실패 (FAIL)"
    print(f"경로 길이 보존 검증       : {status}  "
          f"(차이 {diff:.6f} m, 상대오차 {rel_diff*100:.3f} %, "
          f"허용기준: 절대 {args.length_tol} m 또는 상대 {args.length_rel_tol*100:.1f} %)")
    if not preserved:
        print("  ※ 곡선 구간이 많거나 interval이 큰 경우, 재샘플링된 직선(chord)이")
        print("    원본 곡선의 코너를 절단하며 길이가 짧아지는 것은 정상적인 현상입니다.")
        print("    간격(--interval)을 줄이면 차이가 감소합니다.")
    print(f"출력 파일                 : {args.output}")
    print("========================================\n")

    if not preserved:
        sys.exit(1)


if __name__ == "__main__":
    main()

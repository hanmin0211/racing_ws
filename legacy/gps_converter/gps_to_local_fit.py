#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_to_local_fit.py
=====================
위경도(latitude/longitude) waypoint 파일을 로컬 평면좌표(m)로 변환하고,
변환된 (x, y) 점들에 3차 다항식을 최소자승법(Least Squares)으로 피팅하여
계수 a, b, c, d 를 구하는 스크립트.

    y = a + b*x + c*x^2 + d*x^3

[사용한 방법 - 증명된 표준 기법]

1) 위경도 -> 평면좌표 변환: UTM(Universal Transverse Mercator) 투영
   - WGS84(EPSG:4326) 위경도를 해당 지역의 UTM Zone으로 투영 변환한다.
   - UTM은 국토지리정보원, Autoware/HD맵 파이프라인, 대부분의 자율주행
     경진대회에서 실제로 사용하는 표준 좌표계이다. 평면 위에서 거리/각도
     왜곡이 매우 작아(수 km 이내에서 오차 mm~cm 수준) 로컬 경로 계획에
     그대로 사용 가능하다.
   - 변환은 검증된 오픈소스 라이브러리 pyproj(PROJ 엔진 래퍼)를 사용한다.
   - UTM Zone은 경도로부터 표준 공식으로 자동 계산한다:
         zone = floor((lon + 180) / 6) + 1
     한국(위도 33~39N, 경도 124~132E)은 대부분 UTM 52N(EPSG:32652)에 속한다.
   - 변환 후, 첫 번째 waypoint를 원점(0, 0)으로 하는 상대좌표로 평행이동한다
     (기존 waypoints_local.yaml 과 동일한 관례).

2) 3차 다항식 최소자승법(Least Squares) 피팅
   - 설계행렬(design matrix) A = [1, x, x^2, x^3] 를 구성하고,
     y = a + b*x + c*x^2 + d*x^3 를 만족하는 계수 벡터 c = [a,b,c,d]^T 를
     정규방정식(normal equation) A^T A c = A^T y 의 해로 구한다.
   - 실제 계산은 수치적으로 더 안정적인 특이값분해(SVD) 기반의
     numpy.linalg.lstsq 를 사용한다. 이는 정규방정식을 직접 역행렬로
     푸는 것과 수학적으로 동일한 해를 주지만, 조건수가 나쁜 경우에도
     안정적이라 실무에서 표준적으로 쓰인다.
   - 이 방식은 Apollo, TuSimple 등 다수의 자율주행/차선인식 스택에서
     차선(lane) 또는 로컬 경로를 3차 다항식으로 모델링할 때 쓰는 것과
     동일한 수학적 절차이다.
   - 적합도는 결정계수 R^2 로 함께 보고한다.

주의사항
--------
- 이 피팅은 y 를 x 의 함수로 가정한다(각 x에 대해 y가 하나). 경로가
  급격히 꺾여 동일한 x 값에 서로 다른 y 값이 여러 번 나타나는 형태(예:
  회전 구간, 헤어핀)라면 3차 다항식 피팅 자체가 부적절할 수 있으므로,
  스크립트는 x의 단조성 여부를 검사해 경고를 출력한다.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml
from pyproj import Transformer


# ----------------------------------------------------------------------
# 1. 위경도 waypoint 로드
# ----------------------------------------------------------------------
def load_gps_waypoints(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if isinstance(data, dict):
        if "waypoints" not in data:
            raise ValueError("[오류] YAML에 'waypoints' 키가 없습니다.")
        waypoints = data["waypoints"]
    elif isinstance(data, list):
        waypoints = data
    else:
        raise ValueError("[오류] 지원하지 않는 YAML 구조입니다.")

    if not waypoints:
        raise ValueError("[오류] waypoint 리스트가 비어 있습니다.")

    for i, wp in enumerate(waypoints):
        if "latitude" not in wp or "longitude" not in wp:
            raise ValueError(
                f"[오류] {i}번째 waypoint에 latitude/longitude 가 없습니다: {wp}"
            )
    return waypoints


# ----------------------------------------------------------------------
# 2. UTM Zone 자동 계산 + 위경도 -> 로컬좌표 변환
# ----------------------------------------------------------------------
def compute_utm_epsg(lat, lon):
    """
    표준 공식으로 UTM Zone을 계산하고 해당 EPSG 코드를 반환한다.
        zone = floor((lon + 180) / 6) + 1
        북반구: EPSG = 32600 + zone,  남반구: EPSG = 32700 + zone
    """
    zone = int(math.floor((lon + 180) / 6) + 1)
    epsg = 32600 + zone if lat >= 0 else 32700 + zone
    return epsg, zone


def convert_gps_to_local(waypoints):
    """
    위경도 waypoint 리스트를 UTM 좌표로 변환한 뒤, 첫 번째 점을 원점(0,0)
    으로 하는 로컬 평면좌표로 평행이동하여 반환한다.
    """
    lat0 = waypoints[0]["latitude"]
    lon0 = waypoints[0]["longitude"]
    epsg, zone = compute_utm_epsg(lat0, lon0)

    transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)

    utm_xs, utm_ys = [], []
    for wp in waypoints:
        utm_x, utm_y = transformer.transform(wp["longitude"], wp["latitude"])
        utm_xs.append(utm_x)
        utm_ys.append(utm_y)

    origin_x, origin_y = utm_xs[0], utm_ys[0]

    local_waypoints = []
    for wp, ux, uy in zip(waypoints, utm_xs, utm_ys):
        entry = dict(wp)  # 원본 필드(고도 등) 보존
        entry["x"] = round(ux - origin_x, 4)
        entry["y"] = round(uy - origin_y, 4)
        local_waypoints.append(entry)

    return local_waypoints, {
        "epsg": epsg,
        "utm_zone": zone,
        "origin_lat": lat0,
        "origin_lon": lon0,
        "origin_utm_x": origin_x,
        "origin_utm_y": origin_y,
    }


# ----------------------------------------------------------------------
# 3. 3차 다항식 최소자승법 피팅
# ----------------------------------------------------------------------
def fit_cubic_least_squares(x, y):
    """
    y = a + b*x + c*x^2 + d*x^3 를 최소자승법으로 피팅한다.

    설계행렬 A = [1, x, x^2, x^3] 에 대해 A^T A c = A^T y 를 만족하는
    c = [a, b, c, d] 를 numpy.linalg.lstsq(SVD 기반, 수치적으로 안정)로 구한다.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    A = np.column_stack([np.ones_like(x), x, x ** 2, x ** 3])
    coeffs, residuals, rank, sv = np.linalg.lstsq(A, y, rcond=None)
    a, b, c, d = coeffs

    y_pred = A @ coeffs
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    return {"a": float(a), "b": float(b), "c": float(c), "d": float(d),
            "r_squared": float(r_squared)}


def check_x_monotonic(x):
    """x가 단조증가/단조감소가 아니면 3차 다항식 피팅(y=f(x))이
    부적절할 수 있음을 경고하기 위한 검사."""
    diffs = np.diff(x)
    is_monotonic = np.all(diffs >= 0) or np.all(diffs <= 0)
    return is_monotonic


# ----------------------------------------------------------------------
# 4. 저장
# ----------------------------------------------------------------------
def save_result(path, local_waypoints, meta, fit):
    out = {
        "utm_info": meta,
        "cubic_fit": fit,
        "waypoints": local_waypoints,
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(out, f, allow_unicode=True, sort_keys=False,
                   default_flow_style=False)


# ----------------------------------------------------------------------
# 5. 메인
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="위경도 waypoint를 로컬좌표로 변환하고 3차 다항식을 "
                    "최소자승법으로 피팅합니다."
    )
    parser.add_argument("--input", "-i", required=True,
                         help="입력 위경도 YAML 파일 (latitude/longitude 필드 필요)")
    parser.add_argument("--output", "-o", required=True,
                         help="출력 YAML 파일 (로컬좌표 + 피팅 계수 저장)")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        raise FileNotFoundError(f"[오류] 입력 파일을 찾을 수 없습니다: {args.input}")

    gps_waypoints = load_gps_waypoints(args.input)
    print(f"[로드] '{args.input}' 에서 위경도 waypoint {len(gps_waypoints)}개 로드 완료")

    local_waypoints, meta = convert_gps_to_local(gps_waypoints)
    print(f"[좌표변환] UTM Zone {meta['utm_zone']}N (EPSG:{meta['epsg']}) 사용")
    print(f"[좌표변환] 원점 = (lat {meta['origin_lat']:.8f}, "
          f"lon {meta['origin_lon']:.8f}) -> 로컬 (0.0, 0.0)")

    xs = [wp["x"] for wp in local_waypoints]
    ys = [wp["y"] for wp in local_waypoints]

    if not check_x_monotonic(xs):
        print("[경고] x 좌표가 단조증가/단조감소하지 않습니다. 경로가 급격히 "
              "꺾이는 구간(예: 회전, 헤어핀)이 있다면 y=f(x) 형태의 3차 "
              "다항식 피팅이 실제 경로 형상을 왜곡할 수 있습니다.")

    fit = fit_cubic_least_squares(xs, ys)

    print("\n========== 3차 다항식 최소자승법 피팅 결과 ==========")
    print(f"y = a + b*x + c*x^2 + d*x^3")
    print(f"  a = {fit['a']:.6f}")
    print(f"  b = {fit['b']:.6f}")
    print(f"  c = {fit['c']:.6f}")
    print(f"  d = {fit['d']:.6f}")
    print(f"  R^2 (결정계수) = {fit['r_squared']:.6f}  "
          f"(1.0에 가까울수록 경로가 3차 다항식으로 잘 설명됨)")
    print("=====================================================\n")

    save_result(args.output, local_waypoints, meta, fit)
    print(f"[저장] 로컬좌표 + 피팅 계수 -> '{args.output}'")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, ValueError) as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

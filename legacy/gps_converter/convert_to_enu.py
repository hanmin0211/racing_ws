import yaml
import pymap3d as pm
import math

# YAML 읽기
with open("/home/han/waypoints/all_waypoint.yaml", "r") as f:
    data = yaml.safe_load(f)

waypoints = data["waypoints"]

# 첫 번째 좌표를 기준점(Origin)으로 사용
origin_lat = waypoints[0]["latitude"]
origin_lon = waypoints[0]["longitude"]
origin_alt = 0.0

local_waypoints = []

# 1. 위도/경도 -> ENU(x,y)
for wp in waypoints:
    east, north, up = pm.geodetic2enu(
        wp["latitude"],
        wp["longitude"],
        0.0,
        origin_lat,
        origin_lon,
        origin_alt,
    )

    local_waypoints.append({
    "x": float(east),
    "y": float(north)
})

# 2. Heading(Yaw) 계산
for i in range(len(local_waypoints) - 1):
    dx = local_waypoints[i + 1]["x"] - local_waypoints[i]["x"]
    dy = local_waypoints[i + 1]["y"] - local_waypoints[i]["y"]

    yaw = math.atan2(dy, dx)

    local_waypoints[i]["yaw"] = float(round(yaw, 6))

# 마지막 점은 이전 yaw 사용
local_waypoints[-1]["yaw"] = local_waypoints[-2]["yaw"]

# 소수점 정리
for wp in local_waypoints:
    wp["x"] = round(float(wp["x"]), 3)
    wp["y"] = round(float(wp["y"]), 3)

# 저장
with open("waypoints_local.yaml", "w") as f:
    yaml.dump(
        {"waypoints": local_waypoints},
        f,
        sort_keys=False,
        default_flow_style=False
    )

print(f"변환 완료! 총 {len(local_waypoints)}개의 waypoint 저장")

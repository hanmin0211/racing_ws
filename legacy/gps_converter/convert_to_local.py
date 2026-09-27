import yaml
from pyproj import Transformer

# -------------------------------
# 파일 경로
# -------------------------------
INPUT_FILE = "/home/han/waypoints/all_waypoint.yaml"
OUTPUT_FILE = "/home/han/gps_converter/waypoints_local.yaml"

# -------------------------------
# GPS -> UTM52N
# -------------------------------
transformer = Transformer.from_crs(
    "EPSG:4326",
    "EPSG:32652",
    always_xy=True
)

# -------------------------------
# Waypoint 읽기
# -------------------------------
with open(INPUT_FILE, "r") as f:
    data = yaml.safe_load(f)

gps_waypoints = data["waypoints"]

print(f"읽은 Waypoint : {len(gps_waypoints)}")

# -------------------------------
# 첫 번째 Waypoint를 원점으로 사용
# -------------------------------
first = gps_waypoints[0]

origin_x, origin_y = transformer.transform(
    first["longitude"],
    first["latitude"]
)

print(f"Origin UTM : ({origin_x:.3f}, {origin_y:.3f})")

local_waypoints = []

# -------------------------------
# 변환
# -------------------------------
for wp in gps_waypoints:

    utm_x, utm_y = transformer.transform(
        wp["longitude"],
        wp["latitude"]
    )

    local_waypoints.append({
        "x": round(utm_x - origin_x, 3),
        "y": round(utm_y - origin_y, 3)
    })

# -------------------------------
# 저장
# -------------------------------
with open(OUTPUT_FILE, "w") as f:

    yaml.dump(
        {"waypoints": local_waypoints},
        f,
        default_flow_style=False,
        sort_keys=False
    )

print("===================================")
print("변환 완료")
print(f"저장 위치 : {OUTPUT_FILE}")
print(f"총 Waypoint : {len(local_waypoints)}")
print(f"첫 번째 : {local_waypoints[0]}")
print(f"마지막 : {local_waypoints[-1]}")
print("===================================")

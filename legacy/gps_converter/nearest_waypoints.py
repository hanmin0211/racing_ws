import yaml
import math

with open("waypoints_local.yaml","r") as f:
    data = yaml.safe_load(f)

waypoints = data["waypoints"]

print("총 waypoint :",len(waypoints))

car_x = 0.0
car_y = 0.0

min_dist = 999999
nearest_index = 0

for i, wp in enumerate(waypoints):

    dx = wp["x"] - car_x
    dy = wp["y"] - car_y

    dist = math.sqrt(dx*dx + dy*dy)

    if dist < min_dist:
        min_dist = dist
        nearest_index = i


# 가장 가까운 waypoint부터 20개 선택
local_path = waypoints[nearest_index : nearest_index + 20]

print("Local Path :", len(local_path))

with open("local_path.yaml", "w") as f:
    yaml.dump(
        {"waypoints": local_path},
        f,
        sort_keys=False,
        default_flow_style=False
    )

print("local_path.yaml 저장 완료")

print("Nearest Index :", nearest_index)
print("Distance :", round(min_dist,3))

print("Nearest Index :", nearest_index)
print("Distance :", round(min_dist,3))



import yaml
import math


def world_to_vehicle(wx, wy, car_x, car_y, car_yaw):
    """
    World(ENU) → Vehicle Frame
    """

    dx = wx - car_x
    dy = wy - car_y

    x_vehicle = math.cos(car_yaw) * dx + math.sin(car_yaw) * dy
    y_vehicle = -math.sin(car_yaw) * dx + math.cos(car_yaw) * dy

    return x_vehicle, y_vehicle


# -------------------------------
# ENU Waypoint 불러오기
# -------------------------------

with open("waypoints_local.yaml", "r") as f:
    data = yaml.safe_load(f)

waypoints = data["waypoints"]

print(f"총 {len(waypoints)}개의 Waypoint 읽음")


# -------------------------------
# 현재 차량 위치(테스트용)
# 실제로는 RTK GPS + IMU에서 들어옴
# -------------------------------

car_x = 0.0
car_y = 0.0
car_yaw = 0.0


vehicle_waypoints = []

for wp in waypoints:

    vx, vy = world_to_vehicle(
        wp["x"],
        wp["y"],
        car_x,
        car_y,
        car_yaw
    )

    # 차량 앞쪽 waypoint만 저장
    dist = math.sqrt(vx**2 + vy**2)

    if 0 <= vx <= 35 and dist <= 35:

        vehicle_waypoints.append({
            "x": round(vx, 3),
            "y": round(vy, 3)
        })


print(f"차량 앞쪽 Waypoint : {len(vehicle_waypoints)}개")


# -------------------------------
# 저장
# -------------------------------

with open("vehicle_waypoints.yaml", "w") as f:

    yaml.dump(
        {"waypoints": vehicle_waypoints},
        f,
        default_flow_style=False,
        sort_keys=False
    )

print("vehicle_waypoints.yaml 저장 완료")

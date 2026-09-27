import math

def world_to_vehicle(wx, wy, car_x, car_y, car_yaw):
    """
    World(ENU) → Vehicle Frame

    wx, wy : waypoint
    car_x, car_y : 현재 차량 위치
    car_yaw : IMU yaw (rad)
    """

    dx = wx - car_x
    dy = wy - car_y

    x_vehicle = math.cos(car_yaw)*dx + math.sin(car_yaw)*dy
    y_vehicle = -math.sin(car_yaw)*dx + math.cos(car_yaw)*dy

    return x_vehicle, y_vehicle


if __name__ == "__main__":

    # 테스트

    car_x = 20.0
    car_y = 10.0

    car_yaw = math.radians(30)

    wx = 25.0
    wy = 15.0

    xv, yv = world_to_vehicle(wx, wy, car_x, car_y, car_yaw)

    print(f"x = {xv:.3f}")
    print(f"y = {yv:.3f}")

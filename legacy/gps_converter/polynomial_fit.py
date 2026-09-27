import yaml
import numpy as np


# -------------------------------
# Vehicle Waypoint 읽기
# -------------------------------
with open("vehicle_waypoints.yaml", "r") as f:
    data = yaml.safe_load(f)

waypoints = data["waypoints"]

print(f"읽은 Waypoint : {len(waypoints)}개")


# -------------------------------
# x, y 분리
# -------------------------------
x = []
y = []

for wp in waypoints:
    x.append(wp["x"])
    y.append(wp["y"])

x = np.array(x)
y = np.array(y)


# -------------------------------
# 최소자승법(3차 다항식)
# y = ax³ + bx² + cx + d
# -------------------------------
coef = np.polyfit(x, y, 3)

a, b, c, d = coef

print("\n===== Polynomial =====")
print(f"a = {a:.10f}")
print(f"b = {b:.10f}")
print(f"c = {c:.10f}")
print(f"d = {d:.10f}")


# -------------------------------
# 계산된 y값
# -------------------------------
y_fit = np.polyval(coef, x)


# -------------------------------
# 오차(RMSE)
# -------------------------------
rmse = np.sqrt(np.mean((y - y_fit) ** 2))

print(f"\nRMSE = {rmse:.4f} m")

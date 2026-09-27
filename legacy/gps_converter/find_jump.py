import yaml
import math

with open("waypoints_local.yaml") as f:
    data = yaml.safe_load(f)

wps = data["waypoints"]

print("===== 10m 이상 떨어진 구간 =====")

for i in range(len(wps)-1):
    dx = wps[i+1]["x"] - wps[i]["x"]
    dy = wps[i+1]["y"] - wps[i]["y"]
    d = math.hypot(dx, dy)

    if d > 10:
        print(f"{i} -> {i+1} : {d:.3f} m")


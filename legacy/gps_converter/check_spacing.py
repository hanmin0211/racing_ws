import yaml
import math

with open("waypoints_local.yaml") as f:
    data = yaml.safe_load(f)

wps = data["waypoints"]

dist = []

for i in range(len(wps)-1):

    dx = wps[i+1]["x"] - wps[i]["x"]
    dy = wps[i+1]["y"] - wps[i]["y"]

    dist.append(math.hypot(dx, dy))

print("="*40)
print("Waypoint 개수 :", len(wps))
print("총 길이      :", round(sum(dist),2), "m")
print("평균 간격    :", round(sum(dist)/len(dist),3), "m")
print("최소 간격    :", round(min(dist),3), "m")
print("최대 간격    :", round(max(dist),3), "m")
print("="*40)


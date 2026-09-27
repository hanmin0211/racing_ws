import yaml
import matplotlib.pyplot as plt

with open("waypoints_local.yaml", "r") as f:
    data = yaml.safe_load(f)

x = [p["x"] for p in data["waypoints"]]
y = [p["y"] for p in data["waypoints"]]

plt.figure(figsize=(8,8))

plt.plot(x, y, '-b', linewidth=1)
plt.scatter(x[0], y[0], c='green', s=80, label="START")
plt.scatter(x[-1], y[-1], c='red', s=80, label="END")

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()

plt.show()


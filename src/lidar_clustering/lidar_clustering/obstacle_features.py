import math


def add_obstacle_features(tracked_clusters):
    output = []

    for item in tracked_clusters:
        obstacle = dict(item)
        points = obstacle.get('points', [])

        if not points:
            output.append(obstacle)
            continue

        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]

        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)

        center_x = float(obstacle.get('center_x', sum(xs) / len(xs)))
        center_y = float(obstacle.get('center_y', sum(ys) / len(ys)))

        obstacle.update({
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max,
            'width': y_max - y_min,
            'depth': x_max - x_min,
            'center_distance': math.hypot(center_x, center_y),
            'nearest_distance': min(math.hypot(x, y) for x, y in points),
            'front_distance': x_min,
        })

        output.append(obstacle)

    return output

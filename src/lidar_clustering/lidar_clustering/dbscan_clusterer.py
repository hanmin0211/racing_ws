import numpy as np
from sklearn.cluster import DBSCAN


class DBSCANClusterer:
    def __init__(
        self,
        eps=0.25,
        min_samples=4,
        minimum_cluster_points=4,
    ):
        self.eps = float(eps)
        self.min_samples = int(min_samples)
        self.minimum_cluster_points = int(minimum_cluster_points)

    def cluster(self, points):
        if len(points) < self.min_samples:
            return []

        point_array = np.asarray(points, dtype=float)

        labels = DBSCAN(
            eps=self.eps,
            min_samples=self.min_samples,
            metric='euclidean',
        ).fit_predict(point_array)

        clusters = []

        for label in sorted(set(labels)):
            if label == -1:
                continue

            cluster_array = point_array[labels == label]

            if len(cluster_array) < self.minimum_cluster_points:
                continue

            clusters.append([
                (float(point[0]), float(point[1]))
                for point in cluster_array
            ])

        return clusters

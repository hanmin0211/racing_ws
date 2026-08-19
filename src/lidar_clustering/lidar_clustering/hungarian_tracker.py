import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class HungarianTracker:
    """
    DBSCAN cluster center -> Hungarian data association -> Track ID.

    A lightweight constant-velocity prediction is used before association.
    This is NOT a Kalman filter.
    """

    def __init__(
        self,
        gating_distance=1.60,
        max_missed_frames=25,
        display_missed_frames=5,
        max_tentative_missed_frames=4,
        min_confirmed_hits=2,
        position_smoothing_alpha=0.70,
        velocity_smoothing_alpha=0.35,
    ):
        self.gating_distance = float(gating_distance)
        self.max_missed_frames = int(max_missed_frames)
        self.display_missed_frames = int(display_missed_frames)
        self.max_tentative_missed_frames = int(max_tentative_missed_frames)
        self.min_confirmed_hits = int(min_confirmed_hits)
        self.position_alpha = float(position_smoothing_alpha)
        self.velocity_alpha = float(velocity_smoothing_alpha)

        self.reset()

    def reset(self):
        self.tracks = {}
        self.next_internal_id = 1
        self.next_track_id = 1

    def _new_internal_id(self):
        value = self.next_internal_id
        self.next_internal_id += 1
        return value

    def _new_track_id(self):
        value = self.next_track_id
        self.next_track_id += 1
        return value

    @staticmethod
    def _make_detections(clusters):
        detections = []
        for cluster in clusters:
            if not cluster:
                continue

            xs = np.asarray([p[0] for p in cluster], dtype=float)
            ys = np.asarray([p[1] for p in cluster], dtype=float)

            detections.append({
                'points': cluster,
                'center_x': float(np.mean(xs)),
                'center_y': float(np.mean(ys)),
                'point_count': len(cluster),
            })
        return detections

    def _create_track(self, detection):
        internal_id = self._new_internal_id()

        confirmed = self.min_confirmed_hits <= 1
        track_id = self._new_track_id() if confirmed else None

        self.tracks[internal_id] = {
            'track_id': track_id,
            'center_x': detection['center_x'],
            'center_y': detection['center_y'],
            'raw_x': detection['center_x'],
            'raw_y': detection['center_y'],
            'velocity_x': 0.0,
            'velocity_y': 0.0,
            'last_points': detection['points'],
            'point_count': detection['point_count'],
            'hits': 1,
            'hit_streak': 1,
            'missed': 0,
            'confirmed': confirmed,
        }

    def update(self, clusters):
        detections = self._make_detections(clusters)
        internal_ids = list(self.tracks.keys())

        # Every existing track starts this frame as missed.
        for track in self.tracks.values():
            track['missed'] += 1
            track['hit_streak'] = 0

        matched_track_ids = set()
        matched_detection_ids = set()

        if internal_ids and detections:
            cost = np.full(
                (len(internal_ids), len(detections)),
                fill_value=1e6,
                dtype=float,
            )

            for row, internal_id in enumerate(internal_ids):
                track = self.tracks[internal_id]

                predicted_x = track['center_x'] + track['velocity_x']
                predicted_y = track['center_y'] + track['velocity_y']

                for col, detection in enumerate(detections):
                    d = math.hypot(
                        detection['center_x'] - predicted_x,
                        detection['center_y'] - predicted_y,
                    )
                    cost[row, col] = d

            rows, cols = linear_sum_assignment(cost)

            for row, col in zip(rows, cols):
                distance = float(cost[row, col])
                if distance > self.gating_distance:
                    continue

                internal_id = internal_ids[row]
                detection = detections[col]
                track = self.tracks[internal_id]

                old_x = track['center_x']
                old_y = track['center_y']
                raw_x = detection['center_x']
                raw_y = detection['center_y']

                measured_vx = raw_x - track['raw_x']
                measured_vy = raw_y - track['raw_y']

                va = self.velocity_alpha
                track['velocity_x'] = (
                    va * measured_vx
                    + (1.0 - va) * track['velocity_x']
                )
                track['velocity_y'] = (
                    va * measured_vy
                    + (1.0 - va) * track['velocity_y']
                )

                pa = self.position_alpha
                track['center_x'] = pa * raw_x + (1.0 - pa) * old_x
                track['center_y'] = pa * raw_y + (1.0 - pa) * old_y
                track['raw_x'] = raw_x
                track['raw_y'] = raw_y
                track['last_points'] = detection['points']
                track['point_count'] = detection['point_count']
                track['hits'] += 1
                track['hit_streak'] += 1
                track['missed'] = 0

                if (
                    not track['confirmed']
                    and track['hits'] >= self.min_confirmed_hits
                ):
                    track['confirmed'] = True
                    track['track_id'] = self._new_track_id()

                matched_track_ids.add(internal_id)
                matched_detection_ids.add(col)

        # Unmatched detections become tentative tracks.
        for index, detection in enumerate(detections):
            if index not in matched_detection_ids:
                self._create_track(detection)

        # Remove expired tracks.
        expired = []
        for internal_id, track in self.tracks.items():
            limit = (
                self.max_missed_frames
                if track['confirmed']
                else self.max_tentative_missed_frames
            )
            if track['missed'] > limit:
                expired.append(internal_id)

        for internal_id in expired:
            del self.tracks[internal_id]

        tracked = []
        for track in self.tracks.values():
            if (
                track['confirmed']
                and track['missed'] <= self.display_missed_frames
            ):
                tracked.append({
                    'track_id': track['track_id'],
                    'points': track['last_points'],
                    'center_x': track['center_x'],
                    'center_y': track['center_y'],
                    'velocity_x': track['velocity_x'],
                    'velocity_y': track['velocity_y'],
                    'hits': track['hits'],
                    'missed': track['missed'],
                })

        tracked.sort(key=lambda item: item['track_id'])
        return tracked

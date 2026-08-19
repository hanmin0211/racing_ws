import math
from dataclasses import dataclass

import numpy as np


@dataclass
class FollowGapDecision:
    mode: str
    direction: str
    best_angle_deg: float
    front_distance: float
    gap_start_deg: float
    gap_end_deg: float
    gap_width_deg: float
    best_clearance: float
    left_score: float
    right_score: float


class FollowGapPlanner:
    """
    Track-constrained Follow-the-Gap planner.

    + angle: LEFT
    - angle: RIGHT
    """

    def __init__(
        self,
        yaw_offset_deg=180.0,
        front_fov_deg=180.0,
        min_range=0.30,
        max_range=8.0,
        track_width=3.0,
        planning_lookahead=3.0,
        obstacle_trigger_distance=3.0,
        vehicle_width=0.775,
        safety_margin=0.20,
        straight_deadband_deg=5.0,
        min_gap_width_deg=3.0,
        side_score_margin=0.20,
    ):
        self.yaw_offset = math.radians(yaw_offset_deg)
        self.front_half_fov = math.radians(front_fov_deg / 2.0)
        self.min_range = float(min_range)
        self.max_range = float(max_range)

        self.track_width = float(track_width)
        self.track_half_width = self.track_width / 2.0
        self.planning_lookahead = float(planning_lookahead)

        self.vehicle_width = float(vehicle_width)
        self.vehicle_half_width = self.vehicle_width / 2.0
        self.safety_margin = float(safety_margin)

        self.max_center_y = (
            self.track_half_width
            - self.vehicle_half_width
            - self.safety_margin
        )

        if self.max_center_y <= 0.0:
            raise ValueError(
                'Track width is too narrow for vehicle width and safety margin.'
            )

        self.safety_radius = self.vehicle_half_width + self.safety_margin
        self.obstacle_trigger_distance = float(obstacle_trigger_distance)
        self.straight_deadband_deg = float(straight_deadband_deg)
        self.min_gap_width_deg = float(min_gap_width_deg)
        self.side_score_margin = float(side_score_margin)

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _prepare_front_scan(self, msg):
        scan_data = []
        angle = msg.angle_min
        valid_min_range = max(float(msg.range_min), self.min_range)

        for distance in msg.ranges:
            corrected_angle = self._normalize_angle(angle + self.yaw_offset)

            if abs(corrected_angle) <= self.front_half_fov:
                if math.isinf(distance):
                    clean_distance = self.max_range
                elif math.isnan(distance) or distance < valid_min_range:
                    clean_distance = 0.0
                elif distance > self.max_range:
                    clean_distance = self.max_range
                else:
                    clean_distance = float(distance)

                scan_data.append((corrected_angle, clean_distance))

            angle += msg.angle_increment

        scan_data.sort(key=lambda value: value[0])

        if not scan_data:
            return np.array([]), np.array([])

        return (
            np.asarray([a for a, _ in scan_data], dtype=float),
            np.asarray([r for _, r in scan_data], dtype=float),
        )

    def _create_track_mask(self, angles):
        target_y = self.planning_lookahead * np.tan(angles)
        forward_mask = np.cos(angles) > 0.05

        return (
            np.isfinite(target_y)
            & forward_mask
            & (np.abs(target_y) <= self.max_center_y)
        )

    def _find_track_obstacles(self, angles, ranges):
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        mask = (
            (ranges > 0.0)
            & (ranges <= self.obstacle_trigger_distance)
            & (xs > 0.0)
            & (np.abs(ys) <= self.track_half_width)
        )
        return np.where(mask)[0]

    def _calculate_side_scores(self, angles, ranges, track_mask):
        left = ranges[track_mask & (angles > math.radians(2.0))]
        right = ranges[track_mask & (angles < math.radians(-2.0))]

        left = left[left > 0.0]
        right = right[right > 0.0]

        return (
            float(np.mean(left)) if len(left) else 0.0,
            float(np.mean(right)) if len(right) else 0.0,
        )

    def _apply_safety_bubble(
        self,
        angles,
        ranges,
        track_mask,
        obstacle_indices,
    ):
        safe_ranges = ranges.copy()
        safe_ranges[~track_mask] = 0.0

        if len(obstacle_indices) == 0:
            return safe_ranges

        closest_index = obstacle_indices[
            np.argmin(ranges[obstacle_indices])
        ]

        obstacle_distance = float(ranges[closest_index])
        obstacle_angle = float(angles[closest_index])

        ratio = self.safety_radius / max(obstacle_distance, 0.05)
        ratio = min(ratio, 0.99)

        bubble_half_angle = math.asin(ratio)

        safe_ranges[
            np.abs(angles - obstacle_angle) <= bubble_half_angle
        ] = 0.0

        return safe_ranges

    def _find_largest_gap(self, angles, safe_ranges, original_ranges):
        free_mask = safe_ranges > 0.0
        gaps = []
        start = None

        for index, is_free in enumerate(free_mask):
            if is_free and start is None:
                start = index
            elif not is_free and start is not None:
                gaps.append((start, index - 1))
                start = None

        if start is not None:
            gaps.append((start, len(free_mask) - 1))

        if not gaps:
            return None

        best_gap = max(
            gaps,
            key=lambda gap: angles[gap[1]] - angles[gap[0]],
        )

        start, end = best_gap
        best_index = int((start + end) / 2)

        return {
            'gap_start_angle': angles[start],
            'gap_end_angle': angles[end],
            'gap_width': angles[end] - angles[start],
            'best_angle': angles[best_index],
            'best_clearance': float(np.max(original_ranges[start:end + 1])),
        }

    def plan(self, msg):
        angles, ranges = self._prepare_front_scan(msg)

        if len(ranges) == 0:
            return FollowGapDecision(
                'NO_SCAN', 'STOP',
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            )

        track_mask = self._create_track_mask(angles)

        left_score, right_score = self._calculate_side_scores(
            angles, ranges, track_mask
        )

        obstacle_indices = self._find_track_obstacles(angles, ranges)

        if len(obstacle_indices) == 0:
            return FollowGapDecision(
                'CLEAR', 'STRAIGHT',
                0.0, self.max_range,
                0.0, 0.0, 0.0, self.max_range,
                left_score, right_score
            )

        front_distance = float(np.min(ranges[obstacle_indices]))

        safe_ranges = self._apply_safety_bubble(
            angles, ranges, track_mask, obstacle_indices
        )

        gap = self._find_largest_gap(
            angles, safe_ranges, ranges
        )

        if gap is None:
            return FollowGapDecision(
                'BLOCKED', 'STOP',
                0.0, front_distance,
                0.0, 0.0, 0.0, 0.0,
                left_score, right_score
            )

        gap_width_deg = math.degrees(gap['gap_width'])

        if gap_width_deg < self.min_gap_width_deg:
            return FollowGapDecision(
                'BLOCKED', 'STOP',
                0.0, front_distance,
                math.degrees(gap['gap_start_angle']),
                math.degrees(gap['gap_end_angle']),
                gap_width_deg,
                gap['best_clearance'],
                left_score, right_score
            )

        best_angle_deg = math.degrees(gap['best_angle'])

        if best_angle_deg > self.straight_deadband_deg:
            direction = 'LEFT'
        elif best_angle_deg < -self.straight_deadband_deg:
            direction = 'RIGHT'
        elif left_score > right_score + self.side_score_margin:
            direction = 'LEFT'
        elif right_score > left_score + self.side_score_margin:
            direction = 'RIGHT'
        else:
            direction = 'STRAIGHT'

        return FollowGapDecision(
            'AVOID',
            direction,
            best_angle_deg,
            front_distance,
            math.degrees(gap['gap_start_angle']),
            math.degrees(gap['gap_end_angle']),
            gap_width_deg,
            float(gap['best_clearance']),
            left_score,
            right_score,
        )

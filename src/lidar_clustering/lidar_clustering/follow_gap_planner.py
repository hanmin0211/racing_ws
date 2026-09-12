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
        aim='center',
        aim_margin_deg=2.0,
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

        # ★ 갭 안에서 **어디를 겨냥할 것인가** (2026-09-12)
        #
        #   'center'  — 가장 넓은 빈 구간의 한가운데. Follow-the-Gap 의 원형이고
        #               복도 주행에는 맞다. 그런데 차선이 있는 코스에서는
        #               **필요한 것보다 훨씬 크게 꺾는다.** 실측: 장애물을
        #               안전여유까지 포함해 비켜가는 데 7.2° 면 되는데 13° 를 냈다
        #               (트랙 마스크 상한 17.9° 와 장애물 가장자리의 중점).
        #               그 결과 차선을 1~2.6m 벗어났다.
        #   'nearest' — 갭 안에서 **직진(0°)에 가장 가까운 각**. 안전 버블이 이미
        #               차폭+여유만큼 장애물에서 떼어 놓았으므로, 갭 안이면 어디든
        #               안전하다. 그중 가장 덜 꺾는 곳을 고르면 '필요한 만큼만
        #               비켜간다'가 된다. 경로를 모르는 플래너가 쓸 수 있는
        #               가장 좋은 근사다(차가 경로 위에 있으면 직진 ≒ 경로 추종).
        #
        #   aim_margin_deg: 'nearest' 에서 갭 가장자리에 딱 붙지 않게 안쪽으로
        #   더 밀어 넣는 여유각.
        #
        # ★★ 2026-09-12 측정 — **둘 다 답이 아니다.** (학교 트랙, 장애물 3개)
        #      center  : 접촉 없음(최소간격 0.11~0.16m) · 차선 1~2.6m **이탈**
        #      nearest : 차선이탈 0(최대 0.18~0.32m) · 시험한 **전 배치에서 접촉**
        #    여유각(2~16°)을 키워도, 안전여유(0.25~0.70m)를 키워도 nearest 의
        #    접촉이 안 없어졌다. 안전여유를 키우면 버블만 커지는 게 아니라
        #      max_center_y = track_half − veh_half − margin
        #    도 같이 줄어 **회피에 쓸 각도 창 자체가 좁아지기 때문**이다
        #    (margin 0.70 이면 |각| ≤ 6.8° 밖에 못 쓴다). 구조적 결합이다.
        #
        #    진짜 원인은 플래너가 **기준 경로를 모른다**는 것이다. 차량 좌표계
        #    에서만 판단하므로 '경로에서 얼마나 벗어났는가' 라는 개념이 없고,
        #    그래서 '경로 이탈을 최소화하면서 안전하게 비켜간다' 를 목적으로
        #    삼을 수가 없다. 그걸 넣는 것이 실제 수정이다.
        #    그전까지 기본값은 'center'(접촉 없음) 다.
        self.aim = str(aim)
        self.aim_margin_deg = float(aim_margin_deg)

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
        if self.aim == 'nearest':
            # 갭 안에서 직진(0°)에 가장 가까운 각. 단 가장자리에서
            # aim_margin_deg 만큼은 안쪽으로 들어간다.
            m = math.radians(self.aim_margin_deg)
            lo, hi = angles[start], angles[end]
            if hi - lo > 2.0 * m:
                lo, hi = lo + m, hi - m
            target = min(max(0.0, lo), hi)      # 0 을 [lo, hi] 로 클램프
            best_index = int(start + np.argmin(
                np.abs(angles[start:end + 1] - target)))
        else:
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

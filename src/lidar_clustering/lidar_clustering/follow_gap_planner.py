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
        # ★★ 2026-09-12 측정 (학교 트랙 88m, 장애물 3개, 9가지 배치)
        #    통과 = 완주 + 접촉 없음 + 차선(±0.96m) 안
        #      center  : 0/9 — 접촉은 없지만 차선을 1.0~2.9m 벗어난다
        #      nearest : 4/9 — 차선은 지키지만 촘촘하면 여전히 벗어난다
        #      path    : **7/9** — 여유 0.28m, 최대이탈 0.39~0.86m
        #    실패한 2건은 간격 4m(장애물 길이 1.3m 를 빼면 사이가 2.7m 뿐인
        #    극단적 배치)뿐이다. → 기본값 'path'.
        #
        #    왜 path 가 나은가: center 는 '빈 공간의 한가운데' 라는, 경로와
        #    아무 상관 없는 목표를 쫓아 과하게 꺾는다. nearest 는 덜 꺾지만
        #    기준이 '직진' 이라 장애물을 지난 뒤에도 **벗어난 채로 직진**한다.
        #    path 는 기준이 '경로가 원하는 방향' 이라 비켜가는 동안에도, 지난
        #    뒤에도 경로로 돌아온다.
        #
        #    ※ 이 측정은 _apply_safety_bubble 을 '장애물의 모든 점' 에 씌우도록
        #      고친 뒤의 값이다. 그 전에는 최근접 한 점에만 씌워서 갭 가장자리가
        #      곧 박스 모서리였고, nearest/path 가 전 배치에서 스쳤다.
        #   'path'    — 'nearest' 와 같되, 겨냥의 기준을 직진(0°)이 아니라
        #               **경로 추종이 원하는 방향**으로 삼는다. plan(target_deg=)
        #               으로 pure pursuit 의 조향 방향을 받는다. 이게 있어야
        #               '장애물을 비켜가되 **경로로 돌아오는 쪽**으로 비켜간다'
        #               가 된다. 0° 기준이면 장애물을 지난 뒤에도 벗어난 채로
        #               직진해 버린다.
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

        # ★ 2026-09-12 — 버블을 **장애물의 모든 점**에 씌운다.
        #
        #   예전에는 '가장 가까운 한 점' 에만 씌웠다. 그러면 길이 1.3m 짜리
        #   박스처럼 각도를 넓게 차지하는 장애물은 **나머지 부분이 보호되지
        #   않는다.** 최근접점 기준 버블(3m 거리에서 ±12.3°)이 박스 전체를
        #   덮지 못해, 갭의 가장자리가 곧 박스 모서리가 된다.
        #   그래서 갭 가장자리를 겨냥하면(aim='nearest') 여유가 0 이 되어
        #   시험한 전 배치에서 스쳤다. 갭 중앙을 겨냥할 때(aim='center')만
        #   우연히 멀찍이 돌아 나가 접촉을 면했던 것이다.
        #
        #   점마다 거리가 다르므로 버블 반각도 점마다 다르다(가까울수록 넓다).
        for index in obstacle_indices:
            distance = float(ranges[index])
            ratio = min(self.safety_radius / max(distance, 0.05), 0.99)
            half_angle = math.asin(ratio)
            safe_ranges[
                np.abs(angles - float(angles[index])) <= half_angle
            ] = 0.0

        return safe_ranges

    def _find_largest_gap(self, angles, safe_ranges, original_ranges,
                          target_angle=0.0):
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
        if self.aim in ('nearest', 'path'):
            # 갭 안에서 '가고 싶은 방향' 에 가장 가까운 각.
            #   nearest → 가고 싶은 방향 = 직진(0°)
            #   path    → 가고 싶은 방향 = 경로 추종이 원하는 방향(target_angle)
            # 갭 가장자리에서 aim_margin_deg 만큼은 안쪽으로 들어간다.
            want = target_angle if self.aim == 'path' else 0.0
            m = math.radians(self.aim_margin_deg)
            lo, hi = angles[start], angles[end]
            if hi - lo > 2.0 * m:
                lo, hi = lo + m, hi - m
            target = min(max(want, lo), hi)     # want 를 [lo, hi] 로 클램프
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

    def plan(self, msg, target_deg=None):
        """target_deg: 경로 추종이 원하는 조향 방향[도]. aim='path' 에서만 쓴다."""
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
            angles, safe_ranges, ranges,
            target_angle=math.radians(target_deg or 0.0)
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

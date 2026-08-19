import math
from dataclasses import dataclass

@dataclass
class PreprocessResult:
    foreground_points: list
    scan_size_changed: bool = False


class ScanPreprocessor:
    """
    LaserScan -> vehicle-frame XY points.

    Vehicle coordinates:
      +X: forward
      +Y: left
      -Y: right
    """

    def __init__(
        self,
        min_detection_range=0.30,
        max_detection_range=8.0,
        roi_x_min=0.30,
        roi_x_max=16.0,
        roi_y_min=-8.0,
        roi_y_max=8.0,
        yaw_offset_deg=180.0,
    ):
        self.min_detection_range = float(min_detection_range)
        self.max_detection_range = float(max_detection_range)
        self.roi_x_min = float(roi_x_min)
        self.roi_x_max = float(roi_x_max)
        self.roi_y_min = float(roi_y_min)
        self.roi_y_max = float(roi_y_max)
        self.yaw_offset = math.radians(float(yaw_offset_deg))
        self._last_scan_size = None

    def reset(self):
        self._last_scan_size = None

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def process(self, msg):
        scan_size = len(msg.ranges)
        size_changed = (
            self._last_scan_size is not None
            and scan_size != self._last_scan_size
        )
        self._last_scan_size = scan_size

        points = []
        angle = float(msg.angle_min)
        effective_min = max(
            float(msg.range_min),
            self.min_detection_range,
        )
        effective_max = min(
            float(msg.range_max),
            self.max_detection_range,
        )

        for distance in msg.ranges:
            if math.isfinite(distance) and effective_min <= distance <= effective_max:
                corrected = self._normalize_angle(angle + self.yaw_offset)

                x = float(distance) * math.cos(corrected)
                y = float(distance) * math.sin(corrected)

                if (
                    self.roi_x_min <= x <= self.roi_x_max
                    and self.roi_y_min <= y <= self.roi_y_max
                ):
                    points.append((x, y))

            angle += float(msg.angle_increment)

        return PreprocessResult(
            foreground_points=points,
            scan_size_changed=size_changed,
        )

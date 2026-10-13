"""Distance comparison for a ROS controller; no motor or ROS I/O.

Feed standard CAN 0x110 payloads and a monotonic receive time. Enable only
in the ROS task stage that owns chassis motion. Positive direction means
left (REP-103 +Y); the ROS chassis controller supplies speed/wheel mapping.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class _Sample:
    distance_mm: int
    status: int
    sequence: int
    received_at: float


class ArmUpperAlignment:
    LEFT_SENSOR_ID = 3
    RIGHT_SENSOR_ID = 4

    def __init__(self, *, start_difference_mm=50, stop_difference_mm=20,
                 max_age_s=0.2, max_pair_skew_s=0.1):
        if not 0 <= stop_difference_mm < start_difference_mm:
            raise ValueError('Require 0 <= stop difference < start difference')
        if not (math.isfinite(max_age_s) and math.isfinite(max_pair_skew_s)
                and 0 < max_pair_skew_s <= max_age_s):
            raise ValueError('Require 0 < max pair skew <= max age')
        self.start_difference_mm = start_difference_mm
        self.stop_difference_mm = stop_difference_mm
        self.max_age_s = max_age_s
        self.max_pair_skew_s = max_pair_skew_s
        self.enabled = False
        self._samples = {}
        self._direction = 0

    def set_enabled(self, enabled):
        """Entering/leaving the ROS stage discards the previous sample pair."""
        enabled = bool(enabled)
        if enabled != self.enabled:
            self._samples.clear()
            self._direction = 0
        self.enabled = enabled

    def update_can_frame(self, arbitration_id, data, received_at, *,
                         is_extended_id=False, is_remote_frame=False):
        """Accept IDs 3/4 only. Duplicate frames do not renew freshness."""
        if (is_extended_id or is_remote_frame or arbitration_id != 0x110
                or len(data) != 8 or data[0] != 1
                or data[1] not in (self.LEFT_SENSOR_ID, self.RIGHT_SENSOR_ID)):
            return False
        if not math.isfinite(received_at):
            raise ValueError('Use a finite monotonic receive time')
        previous = self._samples.get(data[1])
        if previous:
            if received_at < previous.received_at:
                return False
            if data[7] == previous.sequence:
                return False
        self._samples[data[1]] = _Sample(
            data[2] | (data[3] << 8), data[6], data[7], received_at)
        return True

    def lateral_direction(self, now):
        """Return +1=left, -1=right, 0=stop, per the shorter-side rule."""
        pair = [self._samples.get(i) for i in
                (self.LEFT_SENSOR_ID, self.RIGHT_SENSOR_ID)]
        if not self.enabled or not math.isfinite(now) or any(s is None for s in pair):
            self._direction = 0
            return 0
        left, right = pair
        if (any(s.status != 0x01
                or not 50 <= s.distance_mm <= 3000
                or not 0 <= now - s.received_at <= self.max_age_s for s in pair)
                or abs(left.received_at - right.received_at) > self.max_pair_skew_s):
            self._direction = 0
            return 0
        difference = right.distance_mm - left.distance_mm
        if abs(difference) <= self.stop_difference_mm:
            self._direction = 0
        elif difference >= self.start_difference_mm:
            self._direction = 1
        elif difference <= -self.start_difference_mm:
            self._direction = -1
        elif self._direction * difference <= 0:
            self._direction = 0
        return self._direction

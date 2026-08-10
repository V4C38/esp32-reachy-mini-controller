"""
Antenna idle animation and threshold body-follow yaw.

Body holds until |head_yaw - body_yaw| exceeds BODY_FOLLOW_THRESHOLD, then
catches up on the excess only. IMU / clutch rotation always drives the head;
body is irrelevant below that threshold. MAX_HEAD_YAW remains the hard neck
limit used by the movement clamp.

Antenna dualSine ported from lens-studio RobotDriver.ts.
Owns body_yaw and antennas only — never head pose axes.
"""

from __future__ import annotations

import math
import time

MAX_HEAD_YAW = 65.0 * math.pi / 180.0
# Start body catch-up before the hard neck limit so torso rotates sooner.
BODY_FOLLOW_THRESHOLD = 40.0 * math.pi / 180.0
MAX_BODY_YAW = 160.0 * math.pi / 180.0
MAX_HEAD_YAW_ABSOLUTE = math.pi

# Puppeteer-like liveliness defaults
DEFAULT_LIVELINESS = 1.25
DEFAULT_GAZE_RESPONSIVENESS = 1.2
DEFAULT_ANTENNA_ACTIVITY = 0.8
HEAD_MOVE_SPEED = 0.06
MAX_HEAD_DELTA_DEG = 2.0
ANTENNA_AMPLITUDE_DEG = 15.0


def dual_sine(t: float, freq_a: float, freq_b: float) -> float:
    return math.sin(t * freq_a) * 0.6 + math.sin(t * freq_b) * 0.4


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _dampen(delta: float, max_delta: float) -> float:
    return _clamp(delta, -max_delta, max_delta)


class Behavior:
    """Derives body_yaw and antennas from head yaw + time."""

    def __init__(
        self,
        liveliness: float = DEFAULT_LIVELINESS,
        gaze_responsiveness: float = DEFAULT_GAZE_RESPONSIVENESS,
        antenna_activity: float = DEFAULT_ANTENNA_ACTIVITY,
    ) -> None:
        self.liveliness = liveliness
        self.gaze_responsiveness = gaze_responsiveness
        self.antenna_activity = antenna_activity
        self.body_yaw = 0.0
        self.antenna_left = 0.0
        self.antenna_right = 0.0
        self._t0 = time.monotonic()

    def reset(self) -> None:
        self.body_yaw = 0.0
        self.antenna_left = 0.0
        self.antenna_right = 0.0
        self._t0 = time.monotonic()

    def update(self, head_yaw: float) -> tuple[float, list[float]]:
        """Advance one tick. Returns (body_yaw, [left, right] antennas)."""
        now = time.monotonic() - self._t0
        deg = math.pi / 180.0

        yaw_smoothing = HEAD_MOVE_SPEED * self.gaze_responsiveness
        max_yaw_delta = MAX_HEAD_DELTA_DEG * self.gaze_responsiveness * deg
        body_smoothing = yaw_smoothing * 0.7 * (0.3 + self.liveliness * 0.4)
        antenna_smoothing = yaw_smoothing * 1.5
        effective_ant_amp = ANTENNA_AMPLITUDE_DEG * self.antenna_activity * deg
        ant_speed = 0.5 + self.antenna_activity * 0.5

        # Body follows only when head exceeds the follow threshold
        rel_yaw = head_yaw - self.body_yaw
        if abs(rel_yaw) > BODY_FOLLOW_THRESHOLD:
            excess = abs(rel_yaw) - BODY_FOLLOW_THRESHOLD
            step = math.copysign(excess * body_smoothing * 8, rel_yaw)
            self.body_yaw += _dampen(step, max_yaw_delta)
            self.body_yaw = _clamp(self.body_yaw, -MAX_BODY_YAW, MAX_BODY_YAW)

        # Antennas
        desired_l = dual_sine(now * ant_speed, 1.3, 3.11) * effective_ant_amp
        desired_r = dual_sine(now * ant_speed, 1.7, 2.73) * effective_ant_amp
        self.antenna_left += (desired_l - self.antenna_left) * antenna_smoothing
        self.antenna_right += (desired_r - self.antenna_right) * antenna_smoothing

        return self.body_yaw, [self.antenna_left, self.antenna_right]


def clamp_head_yaw_for_body(head_yaw: float, body_yaw: float) -> float:
    """Optional helper: clamp absolute head yaw range after body follow."""
    max_range = min(MAX_BODY_YAW + MAX_HEAD_YAW, MAX_HEAD_YAW_ABSOLUTE)
    return _clamp(head_yaw, -max_range, max_range)

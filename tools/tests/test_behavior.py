"""Body-follow regimes and antenna bounds."""

from __future__ import annotations

import math

from esp32_motion_controller.behavior import (
    BODY_FOLLOW_THRESHOLD,
    MAX_BODY_YAW,
    Behavior,
    dual_sine,
)


def test_dual_sine_bounded():
    for t in [i * 0.1 for i in range(200)]:
        v = dual_sine(t, 1.3, 3.11)
        assert -1.0 <= v <= 1.0


def test_body_hold_below_threshold():
    b = Behavior(liveliness=1.0, gaze_responsiveness=1.0, antenna_activity=0.0)
    head = BODY_FOLLOW_THRESHOLD * 0.9
    for _ in range(50):
        body, _ = b.update(head)
        assert body == 0.0
    assert b.body_yaw == 0.0


def test_body_follow_excess_catchup():
    b = Behavior(liveliness=1.25, gaze_responsiveness=1.2, antenna_activity=0.0)
    head = BODY_FOLLOW_THRESHOLD + 0.4
    for _ in range(80):
        body, _ = b.update(head)
    # Body catches up on excess so delta settles near BODY_FOLLOW_THRESHOLD
    assert abs(head - b.body_yaw) <= BODY_FOLLOW_THRESHOLD + 0.05
    assert abs(head - b.body_yaw) >= BODY_FOLLOW_THRESHOLD - 0.05
    assert abs(b.body_yaw) <= MAX_BODY_YAW


def test_antennas_bounded():
    b = Behavior(antenna_activity=1.0)
    for _ in range(100):
        _, ants = b.update(0.0)
    amp = 15.0 * math.pi / 180.0 * 1.0
    assert abs(ants[0]) <= amp + 0.05
    assert abs(ants[1]) <= amp + 0.05

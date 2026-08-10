"""Stewart ellipsoid: rotation-first, translation-fits-remainder."""

from __future__ import annotations

from esp32_motion_controller.movement_handler import (
    ELLIPSOID_PITCH_MAX_RAD,
    ELLIPSOID_ROLL_MAX_RAD,
    ELLIPSOID_X_MAX,
    ELLIPSOID_Y_MAX,
    ELLIPSOID_Z_MAX,
    LIMIT_HEAD_BODY_YAW_DELTA_RAD,
    _clamp_pose_to_daemon_limits,
    _clamp_stewart_ellipsoid,
)


def test_inside_ellipsoid_unchanged():
    x, y, z, r, p = _clamp_stewart_ellipsoid(0.005, -0.005, 0.01, 0.1, -0.1)
    assert abs(x - 0.005) < 1e-9
    assert abs(y + 0.005) < 1e-9
    assert abs(z - 0.01) < 1e-9
    assert abs(r - 0.1) < 1e-9
    assert abs(p + 0.1) < 1e-9


def test_full_tilt_preserves_rotation_zeroes_translation():
    """At full roll+pitch the remaining budget is 0 — translation dies, tilt lives."""
    x, y, z, r, p = _clamp_stewart_ellipsoid(
        ELLIPSOID_X_MAX,
        ELLIPSOID_Y_MAX,
        ELLIPSOID_Z_MAX,
        ELLIPSOID_ROLL_MAX_RAD,
        ELLIPSOID_PITCH_MAX_RAD,
    )
    assert abs(x) < 1e-9
    assert abs(y) < 1e-9
    assert abs(z) < 1e-9
    assert abs(r - ELLIPSOID_ROLL_MAX_RAD) < 1e-9
    assert abs(p - ELLIPSOID_PITCH_MAX_RAD) < 1e-9


def test_translation_does_not_scale_rotation():
    """A large translation request must not shrink a modest pitch."""
    pitch_in = 0.5 * ELLIPSOID_PITCH_MAX_RAD
    x, y, z, r, p = _clamp_stewart_ellipsoid(
        ELLIPSOID_X_MAX,
        ELLIPSOID_Y_MAX,
        ELLIPSOID_Z_MAX,
        0.0,
        pitch_in,
    )
    assert abs(p - pitch_in) < 1e-9
    assert abs(r) < 1e-9
    # Translation must sit on the remaining budget surface.
    nx = x / ELLIPSOID_X_MAX
    ny = y / ELLIPSOID_Y_MAX
    nz = z / ELLIPSOID_Z_MAX
    np_ = p / ELLIPSOID_PITCH_MAX_RAD
    remaining = 1.0 - np_ * np_
    trans_sq = nx * nx + ny * ny + nz * nz
    assert abs(trans_sq - remaining) < 1e-6


def test_roll_pitch_hard_clamped_to_radii():
    x, y, z, r, p = _clamp_stewart_ellipsoid(
        0.0, 0.0, 0.0,
        2.0 * ELLIPSOID_ROLL_MAX_RAD,
        -2.0 * ELLIPSOID_PITCH_MAX_RAD,
    )
    assert abs(r - ELLIPSOID_ROLL_MAX_RAD) < 1e-9
    assert abs(p + ELLIPSOID_PITCH_MAX_RAD) < 1e-9
    assert abs(x) + abs(y) + abs(z) < 1e-9


def test_head_body_yaw_delta_enforced():
    pose = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 1.5}
    out, body = _clamp_pose_to_daemon_limits(pose, body_yaw=0.0)
    assert abs(out["yaw"] - LIMIT_HEAD_BODY_YAW_DELTA_RAD) < 1e-9
    assert body == 0.0

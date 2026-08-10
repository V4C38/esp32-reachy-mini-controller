"""Pure reducer tests — clutch, limits, stale, slew, trajectories."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from esp32_motion_controller.control import (
    BODY_FOLLOW_THRESHOLD,
    DEV_TO_HEAD,
    ELLIPSOID_PITCH_MAX_RAD,
    ELLIPSOID_ROLL_MAX_RAD,
    ELLIPSOID_X_MAX,
    ELLIPSOID_Y_MAX,
    ELLIPSOID_Z_MAX,
    LIMIT_HEAD_BODY_YAW_DELTA_RAD,
    STALE_PACKET_SEC,
    TRANSLATION_SCALE,
    ControlState,
    clamp_pose_to_daemon_limits,
    clamp_stewart_ellipsoid,
    dual_sine,
    force_disengage,
    initial_state,
    note_sample_receipt,
    quat_relative_rpy,
    remap_displacement,
    seed_from_pose,
    slew_limit,
    step,
    zero_pose,
)
from esp32_motion_controller.protocol import Sample

BASELINES = Path(__file__).resolve().parents[1] / "baselines" / "v1_trajectories.json"


def _wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def _sample(
    *,
    q=None,
    p=(0.0, 0.0, 0.0),
    engaged=True,
    gain=1.0,
    ready=True,
    seq=1,
    boot_id="boot",
) -> Sample:
    if q is None:
        q = _wxyz(0, 0, 0)
    return Sample(
        boot_id=boot_id,
        seq=seq,
        q=tuple(q),  # type: ignore[arg-type]
        p=tuple(p),  # type: ignore[arg-type]
        engaged=engaged,
        gain=gain,
        ready=ready,
    )


def test_rising_edge_zero_delta():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    s = _sample(q=_wxyz(0.1, -0.2, 0.3), seq=1)
    st = note_sample_receipt(st, 0.0)
    result = step(st, now=0.05, dt=0.05, sample=s, sample_is_fresh=True)
    assert abs(result.state.desired_pose["roll"]) < 1e-6
    assert abs(result.state.desired_pose["pitch"]) < 1e-6
    assert abs(result.state.desired_pose["yaw"]) < 1e-6


def test_device_axes_map_to_head():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = note_sample_receipt(st, 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_wxyz(0.1, 0, 0), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["pitch"] - 0.1) < 1e-5

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_wxyz(0, 0.1, 0), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["yaw"] - 0.1) < 1e-5

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0.1), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["roll"] - 0.1) < 1e-5


def test_gain_and_release_commit():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), gain=2.0, seq=1), sample_is_fresh=True
    ).state
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_wxyz(0.1, 0, 0), gain=2.0, seq=2),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["pitch"] - 0.2) < 1e-5
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_wxyz(0.1, 0, 0), gain=2.0, engaged=False, seq=3),
        sample_is_fresh=True,
    ).state
    assert abs(st.base_pose["pitch"] - 0.2) < 1e-5


def test_not_ready_cannot_engage():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st,
        now=0.05,
        dt=0.05,
        sample=_sample(q=_wxyz(0.2, 0, 0), ready=False, seq=1),
        sample_is_fresh=True,
    ).state
    assert not st.engaged
    assert st.desired_pose["pitch"] == 0.0


def test_displacement_remap():
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    d = remap_displacement(np.array([0.0, 0.01, 0.0]), q0, translation_gain=1.0)
    assert abs(d[2] - 0.01 * TRANSLATION_SCALE) < 1e-9
    d = remap_displacement(np.array([0.01, 0.0, 0.0]), q0, translation_gain=1.0)
    assert abs(d[1] - 0.01 * TRANSLATION_SCALE) < 1e-9
    d = remap_displacement(np.array([0.0, 0.0, 0.01]), q0, translation_gain=1.0)
    assert abs(d[0] - 0.01 * TRANSLATION_SCALE) < 1e-9


def test_similarity_large_angle():
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    angle = math.radians(45.0)
    q1 = np.asarray(_wxyz(angle, 0, 0))
    roll, pitch, yaw = quat_relative_rpy(q0, q1)
    assert abs(pitch - angle) < 1e-5
    assert abs(roll) < 1e-5
    assert abs(yaw) < 1e-5


def test_stale_force_disengage_commits():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_wxyz(0, 0.2, 0), seq=2), sample_is_fresh=True
    ).state
    st = note_sample_receipt(st, 0.10)
    assert st.engaged
    result = step(st, now=0.10 + STALE_PACKET_SEC + 0.01, dt=0.05, sample=None, sample_is_fresh=False)
    assert not result.state.engaged
    assert abs(result.state.base_pose["yaw"] - 0.2) < 1e-5


def test_ellipsoid_and_yaw_delta():
    x, y, z, r, p = clamp_stewart_ellipsoid(
        ELLIPSOID_X_MAX,
        ELLIPSOID_Y_MAX,
        ELLIPSOID_Z_MAX,
        ELLIPSOID_ROLL_MAX_RAD,
        ELLIPSOID_PITCH_MAX_RAD,
    )
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9
    pose = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 1.5}
    out, body = clamp_pose_to_daemon_limits(pose, 0.0)
    assert abs(out["yaw"] - LIMIT_HEAD_BODY_YAW_DELTA_RAD) < 1e-9
    assert body == 0.0


def test_body_hold_and_follow():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    # Keep antenna_activity but drive head yaw via desired by engaging.
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    # Manually set desired yaw below threshold through many idle ticks.
    st.desired_pose["yaw"] = BODY_FOLLOW_THRESHOLD * 0.9
    st.engaged = False
    st.was_engaged = False
    for i in range(40):
        st = step(st, now=0.1 + i * 0.05, dt=0.05, sample=None, sample_is_fresh=False).state
    assert abs(st.body_yaw) < 1e-6


def test_dual_sine_bounded():
    for t in [i * 0.1 for i in range(200)]:
        v = dual_sine(t, 1.3, 3.11)
        assert -1.0 <= v <= 1.0


def test_slew_limits_step():
    base = zero_pose()
    desired = zero_pose()
    desired["yaw"] = 1.0
    send, body = slew_limit(base, 0.0, desired, 0.0, 0.05)
    assert abs(send["yaw"] - 1.5 * 0.05) < 1e-9


def test_force_disengage():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_wxyz(0.1, 0, 0), seq=2), sample_is_fresh=True
    ).state
    st = force_disengage(st)
    assert not st.engaged
    assert abs(st.base_pose["pitch"] - 0.1) < 1e-5


def test_v1_trajectory_fixture_gain_and_axes():
    data = json.loads(BASELINES.read_text())
    gain_case = data["cases"]["gain"]["desired"]
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), gain=2.0, seq=1), sample_is_fresh=True
    ).state
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_wxyz(0.1, 0, 0), gain=2.0, seq=2),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["pitch"] - gain_case["pitch"]) < 1e-5

    ell = data["cases"]["ellipsoid"]["full_tilt"]
    got = list(
        clamp_stewart_ellipsoid(
            ELLIPSOID_X_MAX,
            ELLIPSOID_X_MAX,
            ELLIPSOID_X_MAX,
            ELLIPSOID_ROLL_MAX_RAD,
            ELLIPSOID_PITCH_MAX_RAD,
        )
    )
    assert got == pytest.approx(ell, abs=1e-9)

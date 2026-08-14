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
    DISENGAGED_PITCH,
    DISENGAGED_Z,
    ELLIPSOID_PITCH_MAX_RAD,
    ELLIPSOID_ROLL_MAX_RAD,
    FORWARD_GAIN,
    HOLD_TILT,
    HORIZONTAL_YAW_FULL_RAD,
    HORIZONTAL_YAW_GAIN_MAX,
    HORIZONTAL_YAW_GAIN_MIN,
    LIMIT_HEAD_BODY_YAW_DELTA_RAD,
    LIMIT_HEAD_YAW_RAD,
    LIMIT_BODY_YAW_RAD,
    LIMIT_HEAD_X_MAX,
    LIMIT_HEAD_Y_MIN,
    LIMIT_HEAD_Z_MAX,
    MAX_ANGULAR_VEL,
    MAX_DT_FOR_VEL_CLAMP,
    MAX_POS_VEL,
    SIDEWAYS_GAIN,
    STALE_PACKET_SEC,
    APPEAR_ANTENNA_SPEED_MULT,
    ControlState,
    advance_antennas,
    clamp_pose_to_daemon_limits,
    clamp_stewart_ellipsoid,
    disengaged_rest_pose,
    dual_sine,
    force_disengage,
    hold_reference,
    adaptive_horizontal_yaw,
    horizontal_yaw_gain,
    initial_state,
    note_sample_receipt,
    quat_relative_rpy,
    relative_head_rpy,
    seed_from_pose,
    slew_limit,
    speed_lock,
    step,
    pose_travel,
    zero_pose,
)
from esp32_motion_controller.protocol import Sample

BASELINES = Path(__file__).resolve().parents[1] / "baselines" / "v1_trajectories.json"


def _wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return (float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def _wxyz_from_rot(rot: R) -> tuple[float, float, float, float]:
    xyzw = rot.as_quat()
    return (float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2]))


def _hold_wxyz() -> tuple[float, float, float, float]:
    return _wxyz_from_rot(HOLD_TILT)


def _from_hold(rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> tuple[float, float, float, float]:
    """Hold pose composed with a device-frame rotation vector."""
    return _wxyz_from_rot(HOLD_TILT * R.from_rotvec([rx, ry, rz]))


def _sample(
    *,
    q=None,
    engaged=True,
    gain=1.0,
    ready=True,
    seq=1,
    boot_id="boot",
) -> Sample:
    if q is None:
        q = _hold_wxyz()
    return Sample(
        boot_id=boot_id,
        seq=seq,
        q=tuple(q),  # type: ignore[arg-type]
        engaged=engaged,
        gain=gain,
        ready=ready,
    )


def test_rising_edge_zero_delta():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    s = _sample(q=_hold_wxyz(), seq=1)
    st = note_sample_receipt(st, 0.0)
    result = step(st, now=0.05, dt=0.05, sample=s, sample_is_fresh=True)
    assert abs(result.state.desired_pose["roll"]) < 1e-6
    assert abs(result.state.desired_pose["pitch"]) < 1e-6
    assert abs(result.state.desired_pose["yaw"]) < 1e-6


def test_engage_at_tilted_pose_is_zero():
    """Engage snapshots the device pose: a tilted grab is still head-neutral."""
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = note_sample_receipt(st, 0.0)
    pose = step(
        st,
        now=0.05,
        dt=0.05,
        sample=_sample(q=_from_hold(0.2, 0.1, 0.05), seq=1),
        sample_is_fresh=True,
    ).state.desired_pose
    assert abs(pose["roll"]) < 1e-5
    assert abs(pose["pitch"]) < 1e-5
    assert abs(pose["yaw"]) < 1e-5
    assert abs(pose["x"]) < 1e-9 and abs(pose["y"]) < 1e-9 and abs(pose["z"]) < 1e-9


def test_device_axes_map_to_head():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = note_sample_receipt(st, 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0.1, 0, 0), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["pitch"] - 0.1 * FORWARD_GAIN) < 1e-5

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0, 0.1, 0), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["yaw"] - adaptive_horizontal_yaw(0.1)) < 1e-5

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    pose = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0, 0, 0.1), seq=2), sample_is_fresh=True
    ).state.desired_pose
    assert abs(pose["roll"] - 0.1 * SIDEWAYS_GAIN) < 1e-5


def test_hold_pose_gains_follow_device_axes():
    """USB-down hold pose: gravity is device +Y, so a horizontal turn is about Y.

    Identity euler 'yaw' is about Z (screen normal) — that is sideways roll, not
    the USB-down pan. Gains must follow the body axis, not the Euler name.
    """
    # Body Y → world Z (screen toward user, USB down).
    q_hold = np.asarray(_wxyz_from_rot(R.from_euler("xyz", [math.pi / 2, 0, 0])))
    angle = 0.1
    r_hold = R.from_quat([q_hold[1], q_hold[2], q_hold[3], q_hold[0]])

    def after_body_axis(axis: np.ndarray) -> np.ndarray:
        q_new = r_hold * R.from_rotvec(axis * angle)
        xyzw = q_new.as_quat()
        return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])

    roll, pitch, yaw = relative_head_rpy(q_hold, after_body_axis(np.array([0.0, 1.0, 0.0])))
    assert abs(yaw - adaptive_horizontal_yaw(angle)) < 1e-5
    assert abs(roll) < 1e-5 and abs(pitch) < 1e-5

    roll, pitch, yaw = relative_head_rpy(q_hold, after_body_axis(np.array([1.0, 0.0, 0.0])))
    assert abs(pitch - angle * FORWARD_GAIN) < 1e-5
    assert abs(roll) < 1e-5 and abs(yaw) < 1e-5

    roll, pitch, yaw = relative_head_rpy(q_hold, after_body_axis(np.array([0.0, 0.0, 1.0])))
    assert abs(roll - angle * SIDEWAYS_GAIN) < 1e-5
    assert abs(pitch) < 1e-5 and abs(yaw) < 1e-5

    # Identity-relative euler yaw (about Z) is the sideways axis, not horizontal.
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    roll, pitch, yaw = relative_head_rpy(q0, np.asarray(_wxyz(0, 0, angle)))
    assert abs(roll - angle * SIDEWAYS_GAIN) < 1e-5
    assert abs(yaw) < 1e-5


def test_adaptive_horizontal_yaw_curve():
    """Short pans stay near 1:1; quadratic ease-in to 2:1 by 50°."""
    five = math.radians(5.0)
    twenty_five = math.radians(25.0)
    fifty = math.radians(50.0)
    sixty = math.radians(60.0)
    assert adaptive_horizontal_yaw(0.0) == pytest.approx(0.0)
    assert horizontal_yaw_gain(0.0) == pytest.approx(HORIZONTAL_YAW_GAIN_MIN)
    assert adaptive_horizontal_yaw(five) == pytest.approx(five * 1.01)
    assert adaptive_horizontal_yaw(twenty_five) == pytest.approx(twenty_five * 1.25)
    assert abs(adaptive_horizontal_yaw(twenty_five)) < abs(twenty_five * 1.5)
    assert adaptive_horizontal_yaw(fifty) == pytest.approx(fifty * HORIZONTAL_YAW_GAIN_MAX)
    assert adaptive_horizontal_yaw(sixty) == pytest.approx(sixty * HORIZONTAL_YAW_GAIN_MAX)
    assert adaptive_horizontal_yaw(-fifty) == pytest.approx(-fifty * HORIZONTAL_YAW_GAIN_MAX)
    assert HORIZONTAL_YAW_FULL_RAD == pytest.approx(math.radians(50.0))


def test_adaptive_yaw_through_clutch():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    five = math.radians(5.0)
    pose = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0, five, 0), seq=2),
        sample_is_fresh=True,
    ).state.desired_pose
    assert pose["yaw"] == pytest.approx(adaptive_horizontal_yaw(five), abs=1e-4)
    assert math.degrees(pose["yaw"]) == pytest.approx(5.05, abs=0.05)

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    twenty_five = math.radians(25.0)
    pose = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0, twenty_five, 0), seq=2),
        sample_is_fresh=True,
    ).state.desired_pose
    assert pose["yaw"] == pytest.approx(adaptive_horizontal_yaw(twenty_five), abs=1e-4)
    assert math.degrees(pose["yaw"]) == pytest.approx(31.25, abs=0.05)


def test_pitch_roll_less_coupled_than_yaw():
    q_hold = np.asarray(_hold_wxyz())
    angle = 0.1
    r_hold = R.from_quat([q_hold[1], q_hold[2], q_hold[3], q_hold[0]])

    def after_body_axis(axis: np.ndarray) -> np.ndarray:
        q_new = r_hold * R.from_rotvec(axis * angle)
        xyzw = q_new.as_quat()
        return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])

    _, pitch, _ = relative_head_rpy(q_hold, after_body_axis(np.array([1.0, 0.0, 0.0])))
    _, _, yaw = relative_head_rpy(q_hold, after_body_axis(np.array([0.0, 1.0, 0.0])))
    roll, _, _ = relative_head_rpy(q_hold, after_body_axis(np.array([0.0, 0.0, 1.0])))
    assert abs(pitch - roll) < 1e-5
    assert abs(pitch - angle * FORWARD_GAIN) < 1e-5
    assert abs(yaw - adaptive_horizontal_yaw(angle)) < 1e-5
    assert abs(yaw) > abs(pitch)
    assert FORWARD_GAIN == SIDEWAYS_GAIN
    assert HORIZONTAL_YAW_GAIN_MAX > FORWARD_GAIN


def test_gain_and_release_commit():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), gain=2.0, seq=1), sample_is_fresh=True
    ).state
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0.1, 0, 0), gain=2.0, seq=2),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["pitch"] - 0.1 * FORWARD_GAIN * 2.0) < 1e-5
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_from_hold(0.1, 0, 0), gain=2.0, engaged=False, seq=3),
        sample_is_fresh=True,
    ).state
    assert abs(st.base_pose["pitch"] - 0.1 * FORWARD_GAIN * 2.0) < 1e-5


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
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0, 0.2, 0), seq=2), sample_is_fresh=True
    ).state
    st = note_sample_receipt(st, 0.10)
    assert st.engaged
    result = step(st, now=0.10 + STALE_PACKET_SEC + 0.01, dt=0.05, sample=None, sample_is_fresh=False, controller_present=False)
    assert not result.state.engaged
    assert abs(result.state.base_pose["yaw"] - adaptive_horizontal_yaw(0.2)) < 1e-5


def test_ellipsoid_and_yaw_delta():
    x, y, z, r, p = clamp_stewart_ellipsoid(
        0.10,
        -0.10,
        0.10,
        ELLIPSOID_ROLL_MAX_RAD * 2,
        ELLIPSOID_PITCH_MAX_RAD * 2,
    )
    assert x == pytest.approx(LIMIT_HEAD_X_MAX)
    assert y == pytest.approx(LIMIT_HEAD_Y_MIN)
    assert z == pytest.approx(LIMIT_HEAD_Z_MAX)
    assert r == pytest.approx(ELLIPSOID_ROLL_MAX_RAD)
    assert p == pytest.approx(ELLIPSOID_PITCH_MAX_RAD)
    pose = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 1.5}
    out, body = clamp_pose_to_daemon_limits(pose, 0.0)
    assert abs(out["yaw"] - LIMIT_HEAD_BODY_YAW_DELTA_RAD) < 1e-9
    assert body == 0.0


def test_body_hold_and_follow():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    # Keep antenna_activity but drive head yaw via desired by engaging.
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
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


def test_appear_antennas_run_faster_than_idle():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    dt = 0.05
    now = 0.2
    idle = advance_antennas(st, now, dt, speed_mult=1.0)
    boosted = advance_antennas(st, now, dt, speed_mult=APPEAR_ANTENNA_SPEED_MULT)
    assert abs(boosted.antenna_left) > abs(idle.antenna_left)
    assert abs(boosted.antenna_right) > abs(idle.antenna_right)


def _geodesic(a: dict[str, float], b: dict[str, float]) -> float:
    r0 = R.from_euler("xyz", [a["roll"], a["pitch"], a["yaw"]])
    r1 = R.from_euler("xyz", [b["roll"], b["pitch"], b["yaw"]])
    return float((r0.inv() * r1).magnitude())


def test_slew_limits_step():
    base = zero_pose()
    desired = zero_pose()
    desired["yaw"] = 1.0
    send, body = slew_limit(base, 0.0, desired, 0.0, 0.05)
    assert send["yaw"] == pytest.approx(MAX_ANGULAR_VEL * 0.05, abs=1e-6)
    assert body == 0.0


def test_speed_lock_combined_axis():
    base = zero_pose()
    desired = zero_pose()
    desired["roll"] = math.pi / 2
    desired["pitch"] = math.pi / 2
    send, _ = speed_lock(base, 0.0, desired, 0.0, 0.05)
    max_step = MAX_ANGULAR_VEL * MAX_DT_FOR_VEL_CLAMP
    assert abs(send["roll"] - base["roll"]) <= max_step + 1e-9
    assert abs(send["pitch"] - base["pitch"]) <= max_step + 1e-9
    assert send["roll"] == pytest.approx(max_step, abs=1e-6)
    assert send["pitch"] == pytest.approx(max_step, abs=1e-6)


def test_speed_lock_does_not_starve_pitch():
    base = zero_pose()
    base["yaw"] = 0.4
    desired = zero_pose()
    desired["yaw"] = 0.8
    desired["pitch"] = 0.2
    send, _ = speed_lock(base, 0.0, desired, 0.0, 0.05)
    step = MAX_ANGULAR_VEL * 0.05
    assert send["pitch"] == pytest.approx(step, abs=1e-6)
    assert send["yaw"] == pytest.approx(0.4 + step, abs=1e-6)


def test_speed_lock_body_yaw_wrap():
    base = zero_pose()
    # A 2π-equivalent command is not a wrap-around. Do not take the 0.10 rad short-arc.
    _, body = speed_lock(base, 1.0, base, 1.0 + 0.10 - 2.0 * math.pi, 0.05)
    short_arc = 1.0 + MAX_ANGULAR_VEL * 0.05
    assert body != pytest.approx(short_arc, abs=1e-3)
    assert body == pytest.approx(1.0 - MAX_ANGULAR_VEL * 0.05, abs=1e-6)


def test_speed_lock_150_request():
    base = zero_pose()
    desired = zero_pose()
    desired["yaw"] = LIMIT_HEAD_YAW_RAD
    send, _ = speed_lock(base, 0.0, desired, 0.0, 0.05)
    assert _geodesic(base, send) <= MAX_ANGULAR_VEL * 0.05 + 1e-9
    assert abs(send["yaw"]) <= MAX_ANGULAR_VEL * 0.05 + 1e-9


def test_speed_lock_translation():
    base = zero_pose()
    desired = zero_pose()
    desired["x"] = 0.20
    send, _ = speed_lock(base, 0.0, desired, 0.0, 0.05)
    dist = math.sqrt(sum(send[k] ** 2 for k in ("x", "y", "z")))
    assert dist <= MAX_POS_VEL * 0.05 + 1e-9


def test_speed_lock_dt_cap_blocks_stall_jump():
    base = zero_pose()
    desired = zero_pose()
    desired["yaw"] = LIMIT_HEAD_YAW_RAD
    send, _ = speed_lock(base, 0.0, desired, 0.0, 2.0)
    assert _geodesic(base, send) <= MAX_ANGULAR_VEL * MAX_DT_FOR_VEL_CLAMP + 1e-9


def test_ten_hz_jitter_stays_engaged():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_wxyz(0, 0, 0), seq=1), sample_is_fresh=True
    ).state
    st = note_sample_receipt(st, 0.05)
    # 10 Hz + jitter: 150 ms gap is still under 600 ms.
    st = note_sample_receipt(st, 0.20)
    result = step(st, now=0.20, dt=0.05, sample=_sample(seq=2), sample_is_fresh=True)
    assert result.state.engaged
    st = note_sample_receipt(result.state, 0.20)
    late = step(st, now=0.20 + STALE_PACKET_SEC + 0.01, dt=0.05, sample=None, sample_is_fresh=False, controller_present=False)
    assert not late.state.engaged


def test_force_disengage():
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0.1, 0, 0), seq=2), sample_is_fresh=True
    ).state
    st = force_disengage(st)
    assert not st.engaged
    assert abs(st.base_pose["pitch"] - 0.1 * FORWARD_GAIN) < 1e-5


def test_v1_trajectory_fixture_gain_and_axes():
    data = json.loads(BASELINES.read_text())
    gain_case = data["cases"]["gain"]["desired"]
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), gain=2.0, seq=1), sample_is_fresh=True
    ).state
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0.1, 0, 0), gain=2.0, seq=2),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["pitch"] - gain_case["pitch"]) < 1e-5


def test_disengaged_rest_pose_values():
    rest = disengaged_rest_pose()
    assert rest["z"] == pytest.approx(DISENGAGED_Z)
    assert rest["pitch"] == pytest.approx(DISENGAGED_PITCH)
    assert rest["x"] == 0.0 and rest["y"] == 0.0
    assert rest["roll"] == 0.0 and rest["yaw"] == 0.0
    assert DISENGAGED_Z == pytest.approx(-0.050)
    assert DISENGAGED_PITCH == pytest.approx(math.radians(5.0))


def test_idle_preserves_disengaged_rest_pose():
    rest = disengaged_rest_pose()
    st = seed_from_pose(
        initial_state(robot_available=True, now=0.0),
        rest,
        0.0,
        apply_ellipsoid=False,
    )
    result = step(
        st,
        now=0.05,
        dt=0.05,
        sample=_sample(engaged=False, seq=1),
        sample_is_fresh=True,
    )
    assert result.state.desired_pose["z"] == pytest.approx(DISENGAGED_Z)
    assert result.state.desired_pose["pitch"] == pytest.approx(DISENGAGED_PITCH)
    assert result.command is not None
    assert result.command.pose["z"] == pytest.approx(DISENGAGED_Z)
    assert result.command.pose["pitch"] == pytest.approx(DISENGAGED_PITCH)


def test_clutch_survives_brief_gap_without_force_disengage():
    now = 10.0
    st = seed_from_pose(initial_state(robot_available=True, now=now), zero_pose(), 0.0)
    st = step(
        st, now=now, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    st = note_sample_receipt(st, now)
    q_ref = st.q_ref.copy()
    st = step(
        st,
        now=now + 0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0.2, 0, 0), seq=2),
        sample_is_fresh=True,
    ).state
    st = note_sample_receipt(st, now + 0.10)
    assert st.engaged
    gap = step(st, now=now + 0.40, dt=0.05, sample=None, sample_is_fresh=False).state
    assert gap.engaged
    assert np.allclose(gap.q_ref, q_ref)
    resumed = step(
        gap,
        now=now + 0.45,
        dt=0.05,
        sample=_sample(q=_from_hold(0.3, 0, 0), seq=3),
        sample_is_fresh=True,
    ).state
    assert resumed.engaged
    assert np.allclose(resumed.q_ref, q_ref)
    assert abs(resumed.desired_pose["pitch"]) > abs(st.desired_pose["pitch"]) - 1e-9


def test_reengage_preserves_committed_pose():
    """Disengage then re-engage: tilt delta is zero, prior head pose stays in base."""
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0.2, 0, 0), seq=2), sample_is_fresh=True
    ).state
    first_pitch = st.desired_pose["pitch"]
    assert abs(first_pitch - 0.2 * FORWARD_GAIN) < 1e-5
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_from_hold(0.2, 0, 0), engaged=False, seq=3),
        sample_is_fresh=True,
    ).state
    st = step(
        st,
        now=0.20,
        dt=0.05,
        sample=_sample(q=_from_hold(0.2, 0, 0), seq=4),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["pitch"] - first_pitch) < 1e-5
    assert abs(st.desired_pose["pitch"] - st.base_pose["pitch"]) < 1e-5


def test_direct_engage_at_tilt_is_neutral():
    """Engaging already tilted does not move the head — that pose is IMU zero."""
    q_target = _from_hold(0.2, 0, 0)
    direct = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    direct = step(
        direct, now=0.05, dt=0.05, sample=_sample(q=q_target, seq=1), sample_is_fresh=True
    ).state
    assert abs(direct.desired_pose["pitch"]) < 1e-5
    assert abs(direct.desired_pose["roll"]) < 1e-5
    assert abs(direct.desired_pose["yaw"]) < 1e-5


def test_disengaged_pan_does_not_change_head_yaw():
    """Heading is relative: panning while clutched, then re-engaging, keeps yaw."""
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=1), sample_is_fresh=True
    ).state
    st = step(
        st, now=0.10, dt=0.05, sample=_sample(q=_from_hold(0, 0.15, 0), seq=2), sample_is_fresh=True
    ).state
    committed_yaw = st.desired_pose["yaw"]
    assert abs(committed_yaw - adaptive_horizontal_yaw(0.15)) < 1e-5
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 0.15, 0), engaged=False, seq=3),
        sample_is_fresh=True,
    ).state
    st = step(
        st,
        now=0.20,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 0.35, 0), seq=4),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["yaw"] - committed_yaw) < 1e-5


def test_heading_fallback_when_screen_is_vertical():
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    q_ref = hold_reference(identity)
    assert np.all(np.isfinite(q_ref))
    assert abs(float(np.linalg.norm(q_ref)) - 1.0) < 1e-9

    # Screen straight down: Rx(180) from identity, or hold then +90° about X.
    q_down = np.asarray(_from_hold(math.pi / 2.0, 0.0, 0.0))
    q_ref_down = hold_reference(q_down)
    assert np.all(np.isfinite(q_ref_down))
    assert abs(float(np.linalg.norm(q_ref_down)) - 1.0) < 1e-9

    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = step(
        st, now=0.05, dt=0.05, sample=_sample(q=tuple(identity), seq=1), sample_is_fresh=True
    ).state
    assert st.engaged
    assert np.all(np.isfinite([st.desired_pose[k] for k in ("roll", "pitch", "yaw")]))


def _engage_hold(now: float = 0.05, seq: int = 1) -> ControlState:
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = note_sample_receipt(st, 0.0)
    return step(
        st, now=now, dt=0.05, sample=_sample(q=_hold_wxyz(), seq=seq), sample_is_fresh=True
    ).state


def test_device_pan_past_180_does_not_invert_yaw():
    """Rotvec would flip at 180° of device pan; unwrapped heading must not."""
    st = _engage_hold()
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0, math.radians(179.0), 0), seq=2),
        sample_is_fresh=True,
    ).state
    yaw_179 = st.desired_pose["yaw"]
    unwrap_179 = st.yaw_unwrapped
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_from_hold(0, math.radians(181.0), 0), seq=3),
        sample_is_fresh=True,
    ).state
    assert unwrap_179 > 0
    assert st.yaw_unwrapped == pytest.approx(unwrap_179, abs=1e-5)
    assert yaw_179 > 0
    assert st.desired_pose["yaw"] > 0
    assert st.desired_pose["yaw"] * yaw_179 > 0


def test_yaw_holds_at_stop_then_returns_the_same_way():
    st = _engage_hold()
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 2.0, 0), seq=2),
        sample_is_fresh=True,
    ).state
    unwrap_at_stop = st.yaw_unwrapped
    stop = st.desired_pose["yaw"]
    assert stop > 0
    assert stop == pytest.approx(
        st.body_yaw + LIMIT_HEAD_BODY_YAW_DELTA_RAD, abs=1e-3
    )
    assert adaptive_horizontal_yaw(unwrap_at_stop) > stop

    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 2.5, 0), seq=3),
        sample_is_fresh=True,
    ).state
    assert st.yaw_unwrapped == pytest.approx(unwrap_at_stop, abs=1e-5)
    assert st.desired_pose["yaw"] > 0
    assert st.desired_pose["yaw"] == pytest.approx(
        st.body_yaw + LIMIT_HEAD_BODY_YAW_DELTA_RAD, abs=1e-3
    )

    st = step(
        st,
        now=0.20,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 0.3, 0), seq=4),
        sample_is_fresh=True,
    ).state
    assert st.yaw_unwrapped == pytest.approx(0.3, abs=1e-4)
    assert st.desired_pose["yaw"] > 0
    assert st.desired_pose["yaw"] < stop
    assert abs(st.desired_pose["yaw"] - adaptive_horizontal_yaw(0.3)) < 1e-4


def test_speed_lock_holds_sign_flip_at_stop():
    base = zero_pose()
    base["yaw"] = LIMIT_HEAD_YAW_RAD
    body_base = LIMIT_HEAD_YAW_RAD - LIMIT_HEAD_BODY_YAW_DELTA_RAD
    desired = zero_pose()
    desired["yaw"] = -LIMIT_HEAD_YAW_RAD
    send, _ = speed_lock(base, body_base, desired, body_base, 0.05)
    assert send["yaw"] == pytest.approx(LIMIT_HEAD_YAW_RAD, abs=1e-6)
    assert send["yaw"] > 0

    _, body = speed_lock(base, LIMIT_BODY_YAW_RAD, base, -LIMIT_BODY_YAW_RAD, 0.05)
    assert body == pytest.approx(LIMIT_BODY_YAW_RAD, abs=1e-6)


def test_pose_travel_detects_pi_wrap():
    a = zero_pose()
    a["yaw"] = math.pi
    b = zero_pose()
    b["yaw"] = -math.pi
    _, ang = pose_travel(a, b, 0.0, 0.0)
    assert ang > math.radians(20.0)


def test_body_follow_does_not_chase_opposite_stop():
    st = _engage_hold()
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0, 2.0, 0), seq=2),
        sample_is_fresh=True,
    ).state
    assert st.desired_pose["yaw"] > 0
    for i in range(80):
        st = step(
            st,
            now=0.15 + i * 0.05,
            dt=0.05,
            sample=_sample(q=_from_hold(0, 2.5, 0), seq=3 + i),
            sample_is_fresh=True,
        ).state
    assert st.desired_pose["yaw"] > 0
    assert st.body_yaw >= 0.0
    assert st.desired_pose["yaw"] <= LIMIT_HEAD_YAW_RAD + 1e-6
    assert st.body_yaw < LIMIT_BODY_YAW_RAD + 1e-6


def test_absolute_yaw_stop_holds_and_does_not_flip():
    from dataclasses import replace as dc_replace

    st = _engage_hold()
    st = dc_replace(st, body_yaw=LIMIT_BODY_YAW_RAD)
    now = 0.10
    seq = 2
    for deg in (60, 120, 180, 240, 270):
        q = _wxyz_from_rot(HOLD_TILT * R.from_euler("y", math.radians(deg)))
        st = step(
            st,
            now=now,
            dt=0.05,
            sample=_sample(q=q, seq=seq),
            sample_is_fresh=True,
        ).state
        now += 0.05
        seq += 1
    assert st.desired_pose["yaw"] == pytest.approx(LIMIT_HEAD_YAW_RAD, abs=1e-3)
    unwrap = st.yaw_unwrapped
    q = _wxyz_from_rot(HOLD_TILT * R.from_euler("y", math.radians(300)))
    st = step(
        st,
        now=now,
        dt=0.05,
        sample=_sample(q=q, seq=seq),
        sample_is_fresh=True,
    ).state
    assert st.yaw_unwrapped == pytest.approx(unwrap, abs=1e-5)
    assert st.desired_pose["yaw"] > 0
    assert st.desired_pose["yaw"] == pytest.approx(LIMIT_HEAD_YAW_RAD, abs=1e-3)


def test_steep_pitch_pan_follows_heading_sign():
    st = _engage_hold()
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_from_hold(0.5, 0.2, 0), seq=2),
        sample_is_fresh=True,
    ).state
    assert st.yaw_unwrapped > 0
    assert st.desired_pose["yaw"] > 0


def test_on_side_roll_freezes_heading():
    """Right-edge heading is unusable when the board is rolled onto its side."""
    st = _engage_hold()
    r_side = HOLD_TILT * R.from_euler("z", math.pi / 2.0)
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_wxyz_from_rot(r_side), seq=2),
        sample_is_fresh=True,
    ).state
    yaw_side = st.desired_pose["yaw"]
    unwrap_side = st.yaw_unwrapped
    r_side_pan = r_side * R.from_rotvec([0.0, 0.15, 0.0])
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_wxyz_from_rot(r_side_pan), seq=3),
        sample_is_fresh=True,
    ).state
    assert abs(st.yaw_unwrapped - unwrap_side) < 1e-6
    assert abs(st.desired_pose["yaw"] - yaw_side) < 1e-4


def test_screen_to_floor_nod_still_allows_pan():
    """A 90° nod leaves the right edge horizontal, so world-up pan must unwrap."""
    st = _engage_hold()
    r_down = HOLD_TILT * R.from_euler("x", math.pi / 2.0)
    st = step(
        st,
        now=0.10,
        dt=0.05,
        sample=_sample(q=_wxyz_from_rot(r_down), seq=2),
        sample_is_fresh=True,
    ).state
    yaw_down = st.desired_pose["yaw"]
    r_down_pan = R.from_euler("z", math.radians(20.0)) * r_down
    st = step(
        st,
        now=0.15,
        dt=0.05,
        sample=_sample(q=_wxyz_from_rot(r_down_pan), seq=3),
        sample_is_fresh=True,
    ).state
    assert st.desired_pose["yaw"] != pytest.approx(yaw_down, abs=1e-3)
    assert abs(st.yaw_unwrapped) > 0


def test_level_nod_does_not_yaw():
    st = _engage_hold()
    for i, deg in enumerate(range(10, 51, 10), start=2):
        q = _wxyz_from_rot(HOLD_TILT * R.from_euler("x", math.radians(deg)))
        st = step(
            st,
            now=0.05 * i,
            dt=0.05,
            sample=_sample(q=q, seq=i),
            sample_is_fresh=True,
        ).state
    assert abs(st.desired_pose["yaw"]) < 1e-4
    assert abs(st.body_yaw) < 1e-6
    assert st.desired_pose["pitch"] == pytest.approx(ELLIPSOID_PITCH_MAX_RAD, abs=1e-3)


def test_rolled_engage_nod_does_not_yaw():
    """USB not perfectly down: nodding about board X must not leak into yaw."""
    eng = HOLD_TILT * R.from_euler("z", math.radians(12.0))
    st = seed_from_pose(initial_state(robot_available=True, now=0.0), zero_pose(), 0.0)
    st = note_sample_receipt(st, 0.0)
    st = step(
        st,
        now=0.05,
        dt=0.05,
        sample=_sample(q=_wxyz_from_rot(eng), seq=1),
        sample_is_fresh=True,
    ).state
    assert abs(st.desired_pose["yaw"]) < 1e-4
    for i, deg in enumerate(range(10, 51, 10), start=2):
        q = _wxyz_from_rot(eng * R.from_euler("x", math.radians(deg)))
        st = step(
            st,
            now=0.05 * i,
            dt=0.05,
            sample=_sample(q=q, seq=i),
            sample_is_fresh=True,
        ).state
    assert abs(st.desired_pose["yaw"]) < math.radians(1.0)
    assert abs(st.body_yaw) < 1e-6
    assert st.desired_pose["pitch"] > math.radians(10.0)


def _assert_yaw_workspace(pose: dict[str, float], body: float) -> None:
    assert abs(pose["yaw"]) <= LIMIT_HEAD_YAW_RAD + 1e-9
    assert abs(body) <= LIMIT_BODY_YAW_RAD + 1e-9
    assert abs(pose["yaw"] - body) <= LIMIT_HEAD_BODY_YAW_DELTA_RAD + 1e-9


def test_head_yaw_never_exceeds_150():
    pose, body = clamp_pose_to_daemon_limits(
        {**zero_pose(), "yaw": math.pi}, 0.0, apply_ellipsoid=False
    )
    assert abs(pose["yaw"]) <= LIMIT_HEAD_YAW_RAD + 1e-9
    _assert_yaw_workspace(pose, body)

    st = _engage_hold()
    now = 0.10
    seq = 2
    for deg in range(30, 361, 30):
        q = _wxyz_from_rot(HOLD_TILT * R.from_euler("y", math.radians(deg)))
        st = step(
            st,
            now=now,
            dt=0.05,
            sample=_sample(q=q, seq=seq),
            sample_is_fresh=True,
        ).state
        now += 0.05
        seq += 1
        _assert_yaw_workspace(st.desired_pose, st.body_yaw)
    assert abs(st.desired_pose["yaw"]) <= LIMIT_HEAD_YAW_RAD + 1e-6


def test_clamp_and_speed_lock_keep_workspace_invariants():
    cases = [
        ({**zero_pose(), "yaw": math.pi}, 0.0),
        ({**zero_pose(), "yaw": math.pi}, -LIMIT_BODY_YAW_RAD),
        ({**zero_pose(), "yaw": -math.pi}, LIMIT_BODY_YAW_RAD),
        ({**zero_pose(), "yaw": LIMIT_HEAD_YAW_RAD}, -LIMIT_BODY_YAW_RAD),
        ({**zero_pose(), "yaw": -LIMIT_HEAD_YAW_RAD}, LIMIT_BODY_YAW_RAD),
    ]
    for pose, body in cases:
        out_pose, out_body = clamp_pose_to_daemon_limits(
            pose, body, apply_ellipsoid=False
        )
        _assert_yaw_workspace(out_pose, out_body)

        send, send_body = speed_lock(
            zero_pose(), 0.0, pose, body, 0.05, apply_ellipsoid=False
        )
        _assert_yaw_workspace(send, send_body)

        send, send_body = speed_lock(
            pose, body, {**zero_pose(), "yaw": -pose["yaw"]}, -body, 0.05,
            apply_ellipsoid=False,
        )
        _assert_yaw_workspace(send, send_body)


def test_continuous_pan_past_stop_does_not_reverse_yaw():
    """Heading keeps tracking at a stop so a one-direction pan cannot alias."""
    st = _engage_hold()
    now = 0.10
    seq = 2
    yaws: list[float] = []
    for deg in range(10, 401, 10):
        q = _wxyz_from_rot(HOLD_TILT * R.from_euler("y", math.radians(deg)))
        st = step(
            st,
            now=now,
            dt=0.05,
            sample=_sample(q=q, seq=seq),
            sample_is_fresh=True,
        ).state
        yaws.append(st.desired_pose["yaw"])
        now += 0.05
        seq += 1
    assert all(y >= -1e-6 for y in yaws)
    assert max(yaws) <= LIMIT_HEAD_YAW_RAD + 1e-6
    at_stop = [
        i
        for i, y in enumerate(yaws)
        if y >= LIMIT_HEAD_BODY_YAW_DELTA_RAD - 1e-3
    ]
    assert at_stop
    after = yaws[at_stop[0] :]
    assert all(y > 0 for y in after)
    assert min(after) >= LIMIT_HEAD_BODY_YAW_DELTA_RAD - math.radians(5.0)



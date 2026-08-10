"""
Pure control reducer for protocol v2.

Owns clutch mapping, body follow, antenna phase, workspace projection, smoothing,
stale release, and slew limiting. No I/O.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from esp32_motion_controller.protocol import Sample

POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")
ANGULAR_AXES = ("roll", "pitch", "yaw")
POSITIONAL_AXES = ("x", "y", "z")

DEV_TO_HEAD = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

TRANSLATION_SCALE = 0.30
TRANSLATION_GAIN_DEFAULT = 1.0

STALE_PACKET_SEC = 0.300
CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

# Match v1 feel: POSE_ALPHA=0.12 at 33 ms ≈ tau ~0.26 s
POSE_TAU_SEC = 0.255
ANTENNA_TAU_SEC = 0.39

MAX_ANGULAR_VEL = 1.5  # rad/s
MAX_POS_VEL = 0.05  # m/s
MAX_DT_FOR_VEL_CLAMP = 0.05

LIMIT_BODY_YAW_RAD = 160.0 * math.pi / 180.0
LIMIT_HEAD_YAW_RAD = math.pi
LIMIT_HEAD_BODY_YAW_DELTA_RAD = 65.0 * math.pi / 180.0

LIMIT_HEAD_X_MIN = -0.020
LIMIT_HEAD_X_MAX = 0.020
LIMIT_HEAD_Y_MIN = -0.020
LIMIT_HEAD_Y_MAX = 0.020
LIMIT_HEAD_Z_MIN = 0.0
LIMIT_HEAD_Z_MAX = 0.025

ELLIPSOID_X_MAX = 0.015
ELLIPSOID_Y_MAX = 0.015
ELLIPSOID_Z_MAX = 0.018
ELLIPSOID_ROLL_MAX_RAD = 25.0 * math.pi / 180.0
ELLIPSOID_PITCH_MAX_RAD = 25.0 * math.pi / 180.0

MAX_HEAD_YAW = 65.0 * math.pi / 180.0
BODY_FOLLOW_THRESHOLD = 40.0 * math.pi / 180.0
MAX_BODY_YAW = 160.0 * math.pi / 180.0

DEFAULT_LIVELINESS = 1.25
DEFAULT_GAZE_RESPONSIVENESS = 1.2
DEFAULT_ANTENNA_ACTIVITY = 0.8
HEAD_MOVE_SPEED = 0.06
MAX_HEAD_DELTA_DEG = 2.0
ANTENNA_AMPLITUDE_DEG = 15.0

IK_FAIL_RETRACT_TARGET_ALPHA = 0.06
IK_FAIL_CONSECUTIVE_THRESHOLD = 3

Mode = str  # idle | engaged | resetting | fault


def zero_pose() -> dict[str, float]:
    return {k: 0.0 for k in POSE_AXES}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _finite_quat(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(arr)) or np.linalg.norm(arr) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return arr / np.linalg.norm(arr)


def _finite_vec3(v: Sequence[float]) -> np.ndarray:
    arr = np.asarray(v, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(arr)):
        return np.zeros(3, dtype=np.float64)
    return arr


def _wxyz_to_rotation(q: np.ndarray) -> R:
    return R.from_quat([q[1], q[2], q[3], q[0]])


def quat_relative_rpy(q_ref: np.ndarray, q_device: np.ndarray) -> tuple[float, float, float]:
    r_ref = _wxyz_to_rotation(q_ref)
    r_dev = _wxyz_to_rotation(q_device)
    r_rel_dev = r_ref.inv() * r_dev
    m = R.from_matrix(DEV_TO_HEAD)
    r_head = m * r_rel_dev * m.inv()
    roll, pitch, yaw = r_head.as_euler("xyz", degrees=False)
    return float(roll), float(pitch), float(yaw)


def remap_displacement(
    p_world_delta: np.ndarray,
    q_ref: np.ndarray,
    *,
    translation_gain: float = TRANSLATION_GAIN_DEFAULT,
) -> np.ndarray:
    r_ref = _wxyz_to_rotation(q_ref)
    disp_ref = r_ref.inv().apply(p_world_delta)
    disp_head = DEV_TO_HEAD @ disp_ref
    return disp_head * TRANSLATION_SCALE * float(translation_gain)


def dual_sine(t: float, freq_a: float, freq_b: float) -> float:
    return math.sin(t * freq_a) * 0.6 + math.sin(t * freq_b) * 0.4


def clamp_stewart_ellipsoid(
    x: float, y: float, z: float, roll: float, pitch: float,
) -> tuple[float, float, float, float, float]:
    z_clamped = _clamp(z, LIMIT_HEAD_Z_MIN, LIMIT_HEAD_Z_MAX)
    x_clamped = _clamp(x, LIMIT_HEAD_X_MIN, LIMIT_HEAD_X_MAX)
    y_clamped = _clamp(y, LIMIT_HEAD_Y_MIN, LIMIT_HEAD_Y_MAX)

    roll_c = _clamp(roll, -ELLIPSOID_ROLL_MAX_RAD, ELLIPSOID_ROLL_MAX_RAD)
    pitch_c = _clamp(pitch, -ELLIPSOID_PITCH_MAX_RAD, ELLIPSOID_PITCH_MAX_RAD)

    nr = roll_c / ELLIPSOID_ROLL_MAX_RAD if ELLIPSOID_ROLL_MAX_RAD > 0 else 0.0
    np_ = pitch_c / ELLIPSOID_PITCH_MAX_RAD if ELLIPSOID_PITCH_MAX_RAD > 0 else 0.0
    remaining = max(0.0, 1.0 - (nr * nr + np_ * np_))

    nx = x_clamped / ELLIPSOID_X_MAX if ELLIPSOID_X_MAX > 0 else 0.0
    ny = y_clamped / ELLIPSOID_Y_MAX if ELLIPSOID_Y_MAX > 0 else 0.0
    nz = z_clamped / ELLIPSOID_Z_MAX if ELLIPSOID_Z_MAX > 0 else 0.0
    trans_sq = nx * nx + ny * ny + nz * nz

    if trans_sq <= remaining or trans_sq <= 1e-12:
        return (x_clamped, y_clamped, z_clamped, roll_c, pitch_c)

    scale = math.sqrt(remaining / trans_sq)
    return (
        x_clamped * scale,
        y_clamped * scale,
        z_clamped * scale,
        roll_c,
        pitch_c,
    )


def clamp_pose_to_daemon_limits(
    pose: dict[str, float], body_yaw: float
) -> tuple[dict[str, float], float]:
    out_pose = dict(pose)
    cx, cy, cz, cr, cp = clamp_stewart_ellipsoid(
        pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"],
    )
    out_pose["x"] = cx
    out_pose["y"] = cy
    out_pose["z"] = cz
    out_pose["roll"] = cr
    out_pose["pitch"] = cp

    body_yaw_clamped = _clamp(body_yaw, -LIMIT_BODY_YAW_RAD, LIMIT_BODY_YAW_RAD)
    out_pose["yaw"] = _clamp(pose["yaw"], -LIMIT_HEAD_YAW_RAD, LIMIT_HEAD_YAW_RAD)
    delta = out_pose["yaw"] - body_yaw_clamped
    if delta > LIMIT_HEAD_BODY_YAW_DELTA_RAD:
        out_pose["yaw"] = body_yaw_clamped + LIMIT_HEAD_BODY_YAW_DELTA_RAD
    elif delta < -LIMIT_HEAD_BODY_YAW_DELTA_RAD:
        out_pose["yaw"] = body_yaw_clamped - LIMIT_HEAD_BODY_YAW_DELTA_RAD
    return out_pose, body_yaw_clamped


def _alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return 1.0 - math.exp(-max(dt, 0.0) / tau)


def slew_limit(
    baseline: dict[str, float],
    baseline_body: float,
    desired: dict[str, float],
    desired_body: float,
    dt: float,
) -> tuple[dict[str, float], float]:
    dt_c = min(max(dt, 0.0), MAX_DT_FOR_VEL_CLAMP)
    max_d_ang = MAX_ANGULAR_VEL * dt_c
    max_d_pos = MAX_POS_VEL * dt_c
    send = {}
    for axis in ANGULAR_AXES:
        delta = desired[axis] - baseline[axis]
        send[axis] = baseline[axis] + _clamp(delta, -max_d_ang, max_d_ang)
    for axis in POSITIONAL_AXES:
        delta = desired[axis] - baseline[axis]
        send[axis] = baseline[axis] + _clamp(delta, -max_d_pos, max_d_pos)
    body_delta = desired_body - baseline_body
    send_body = baseline_body + _clamp(body_delta, -max_d_ang, max_d_ang)
    return clamp_pose_to_daemon_limits(send, send_body)


@dataclass
class ControlState:
    mode: Mode = "idle"
    robot_available: bool = True
    error: str | None = None

    engaged: bool = False
    was_engaged: bool = False
    gain: float = 1.0
    ready: bool = False
    translation_gain: float = TRANSLATION_GAIN_DEFAULT

    q_ref: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    p_ref: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_pose: dict[str, float] = field(default_factory=zero_pose)
    desired_pose: dict[str, float] = field(default_factory=zero_pose)

    body_yaw: float = 0.0
    antenna_left: float = 0.0
    antenna_right: float = 0.0
    behavior_t0: float = 0.0

    smooth_pose: dict[str, float] = field(default_factory=zero_pose)
    smooth_body_yaw: float = 0.0
    smooth_antennas: list[float] = field(default_factory=lambda: [0.0, 0.0])

    baseline_pose: dict[str, float] = field(default_factory=zero_pose)
    baseline_body_yaw: float = 0.0
    baseline_antennas: list[float] = field(default_factory=lambda: [0.0, 0.0])

    last_sample_time: float = 0.0
    have_sample: bool = False
    sends_frozen: bool = True
    seeded: bool = False
    consecutive_ik_failures: int = 0

    liveliness: float = DEFAULT_LIVELINESS
    gaze_responsiveness: float = DEFAULT_GAZE_RESPONSIVENESS
    antenna_activity: float = DEFAULT_ANTENNA_ACTIVITY


@dataclass(frozen=True, slots=True)
class Command:
    pose: dict[str, float]
    body_yaw: float
    antennas: tuple[float, float]


@dataclass(frozen=True, slots=True)
class StepResult:
    state: ControlState
    command: Command | None
    mode_changed: bool = False
    host_error: str | None = None


def initial_state(*, robot_available: bool, now: float) -> ControlState:
    return ControlState(
        robot_available=robot_available,
        behavior_t0=now,
        sends_frozen=not robot_available,
        seeded=not robot_available,
        mode="idle",
    )


def seed_from_pose(
    state: ControlState,
    pose: dict[str, float],
    body_yaw: float,
    *,
    antennas: Sequence[float] | None = None,
) -> ControlState:
    ants = list(antennas) if antennas is not None else [0.0, 0.0]
    ants = (ants + [0.0, 0.0])[:2]
    pose_c, body_c = clamp_pose_to_daemon_limits(
        {k: float(pose.get(k, 0.0)) for k in POSE_AXES}, float(body_yaw)
    )
    return replace(
        state,
        base_pose=dict(pose_c),
        desired_pose=dict(pose_c),
        smooth_pose=dict(pose_c),
        baseline_pose=dict(pose_c),
        body_yaw=body_c,
        smooth_body_yaw=body_c,
        baseline_body_yaw=body_c,
        antenna_left=float(ants[0]),
        antenna_right=float(ants[1]),
        smooth_antennas=[float(ants[0]), float(ants[1])],
        baseline_antennas=[float(ants[0]), float(ants[1])],
        seeded=True,
        sends_frozen=False,
        consecutive_ik_failures=0,
        engaged=False,
        was_engaged=False,
        mode="idle" if state.mode != "resetting" else state.mode,
        error=None,
    )


def rebase_neutral(state: ControlState, *, measured_baseline: dict[str, float] | None,
                   measured_body: float | None) -> ControlState:
    """After reset completion: targets at neutral; baseline from measured pose if provided."""
    neutral = zero_pose()
    if measured_baseline is None or measured_body is None:
        return replace(
            state,
            base_pose=dict(neutral),
            desired_pose=dict(neutral),
            smooth_pose=dict(neutral),
            body_yaw=0.0,
            smooth_body_yaw=0.0,
            antenna_left=0.0,
            antenna_right=0.0,
            smooth_antennas=[0.0, 0.0],
            engaged=False,
            was_engaged=False,
            q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
            p_ref=np.zeros(3),
            sends_frozen=True,
            mode="fault",
            error="robot pose unread after reset",
            behavior_t0=state.behavior_t0,
        )
    pose_c, body_c = clamp_pose_to_daemon_limits(measured_baseline, measured_body)
    return replace(
        state,
        base_pose=dict(neutral),
        desired_pose=dict(neutral),
        smooth_pose=dict(neutral),
        baseline_pose=dict(pose_c),
        body_yaw=0.0,
        smooth_body_yaw=0.0,
        baseline_body_yaw=body_c,
        antenna_left=0.0,
        antenna_right=0.0,
        smooth_antennas=[0.0, 0.0],
        baseline_antennas=[0.0, 0.0],
        engaged=False,
        was_engaged=False,
        q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
        p_ref=np.zeros(3),
        sends_frozen=False,
        seeded=True,
        mode="idle",
        error=None,
        consecutive_ik_failures=0,
    )


def begin_reset(state: ControlState, now: float) -> ControlState:
    st = force_disengage(state)
    return replace(
        st,
        mode="resetting",
        error=None,
        base_pose=zero_pose(),
        desired_pose=zero_pose(),
        body_yaw=0.0,
        antenna_left=0.0,
        antenna_right=0.0,
        was_engaged=False,
        engaged=False,
        q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
        p_ref=np.zeros(3),
        behavior_t0=now,
    )


def force_disengage(state: ControlState) -> ControlState:
    if state.engaged:
        return replace(
            state,
            base_pose=dict(state.desired_pose),
            engaged=False,
            was_engaged=False,
            mode="idle" if state.mode == "engaged" else state.mode,
        )
    return replace(state, engaged=False, was_engaged=False)


def mark_sdk_success(state: ControlState, command: Command) -> ControlState:
    return replace(
        state,
        baseline_pose=dict(command.pose),
        baseline_body_yaw=command.body_yaw,
        baseline_antennas=list(command.antennas),
        consecutive_ik_failures=0,
        sends_frozen=False,
    )


def mark_sdk_failure(state: ControlState) -> ControlState:
    fails = state.consecutive_ik_failures + 1
    desired = dict(state.desired_pose)
    body = state.body_yaw
    if fails >= IK_FAIL_CONSECUTIVE_THRESHOLD:
        alpha = IK_FAIL_RETRACT_TARGET_ALPHA
        for ax in POSE_AXES:
            desired[ax] *= 1.0 - alpha
        body *= 1.0 - alpha
    return replace(
        state,
        consecutive_ik_failures=fails,
        desired_pose=desired,
        body_yaw=body,
        sends_frozen=True,
        mode="fault",
        error="set_target failed",
    )


def mark_pose_unread(state: ControlState) -> ControlState:
    return replace(
        state,
        sends_frozen=True,
        mode="fault",
        error="robot pose unread",
    )


def _update_clutch(state: ControlState, sample: Sample, *, allow_engage: bool) -> ControlState:
    gain = _clamp(float(sample.gain), 0.1, 3.0)
    q_dev = _finite_quat(sample.q)
    p_dev = _finite_vec3(sample.p)
    want = bool(sample.engaged) and bool(sample.ready) and allow_engage
    rising = want and not state.was_engaged
    falling = (not want) and state.was_engaged

    q_ref = state.q_ref
    p_ref = state.p_ref
    base = dict(state.base_pose)
    desired = dict(state.desired_pose)

    if rising:
        q_ref = q_dev.copy()
        p_ref = p_dev.copy()

    if want:
        roll, pitch, yaw = quat_relative_rpy(q_ref, q_dev)
        disp = remap_displacement(
            p_dev - p_ref,
            q_ref,
            translation_gain=state.translation_gain * gain,
        )
        desired = {
            "x": base["x"] + float(disp[0]),
            "y": base["y"] + float(disp[1]),
            "z": base["z"] + float(disp[2]),
            "roll": base["roll"] + gain * roll,
            "pitch": base["pitch"] + gain * pitch,
            "yaw": base["yaw"] + gain * yaw,
        }
    elif falling:
        base = dict(desired)

    return replace(
        state,
        ready=bool(sample.ready),
        gain=gain,
        q_ref=q_ref,
        p_ref=p_ref,
        base_pose=base,
        desired_pose=desired,
        engaged=want,
        was_engaged=want,
    )


def _advance_behavior(state: ControlState, head_yaw: float, now: float, dt: float) -> ControlState:
    # Normalize behavior update against the legacy ~33 ms tick using dt scaling.
    tick_scale = dt / 0.033 if dt > 0 else 1.0
    t = now - state.behavior_t0
    deg = math.pi / 180.0

    yaw_smoothing = HEAD_MOVE_SPEED * state.gaze_responsiveness * tick_scale
    max_yaw_delta = MAX_HEAD_DELTA_DEG * state.gaze_responsiveness * deg * tick_scale
    body_smoothing = yaw_smoothing * 0.7 * (0.3 + state.liveliness * 0.4)
    antenna_smoothing = yaw_smoothing * 1.5
    effective_ant_amp = ANTENNA_AMPLITUDE_DEG * state.antenna_activity * deg
    ant_speed = 0.5 + state.antenna_activity * 0.5

    body_yaw = state.body_yaw
    rel_yaw = head_yaw - body_yaw
    if abs(rel_yaw) > BODY_FOLLOW_THRESHOLD:
        excess = abs(rel_yaw) - BODY_FOLLOW_THRESHOLD
        step = math.copysign(excess * body_smoothing * 8, rel_yaw)
        body_yaw += _clamp(step, -max_yaw_delta, max_yaw_delta)
        body_yaw = _clamp(body_yaw, -MAX_BODY_YAW, MAX_BODY_YAW)

    desired_l = dual_sine(t * ant_speed, 1.3, 3.11) * effective_ant_amp
    desired_r = dual_sine(t * ant_speed, 1.7, 2.73) * effective_ant_amp
    ant_l = state.antenna_left + (desired_l - state.antenna_left) * antenna_smoothing
    ant_r = state.antenna_right + (desired_r - state.antenna_right) * antenna_smoothing

    return replace(
        state,
        body_yaw=body_yaw,
        antenna_left=ant_l,
        antenna_right=ant_r,
    )


def step(
    state: ControlState,
    *,
    now: float,
    dt: float,
    sample: Sample | None,
    sample_is_fresh: bool,
) -> StepResult:
    """Advance one control tick.

    `sample` is the latest validated sample (may be None before first packet).
    `sample_is_fresh` is True only when a new sample arrived since the previous tick.
    """
    prev_mode = state.mode
    st = state

    if st.mode == "resetting":
        # No streaming commands while reset owns the robot.
        return StepResult(state=st, command=None, mode_changed=False)

    # Stale detection uses host receipt time stamped on the sample mailbox.
    if st.have_sample and st.last_sample_time > 0:
        if now - st.last_sample_time > STALE_PACKET_SEC and st.engaged:
            st = force_disengage(st)

    if sample is not None and sample_is_fresh and st.mode != "resetting":
        allow_engage = st.mode != "fault" and not st.sends_frozen
        st = _update_clutch(st, sample, allow_engage=allow_engage)
        st = replace(st, have_sample=True)

    if st.mode not in {"resetting"}:
        st = _advance_behavior(st, st.desired_pose["yaw"], now, dt)

    target_pose, target_body = clamp_pose_to_daemon_limits(st.desired_pose, st.body_yaw)
    target_ants = [st.antenna_left, st.antenna_right]

    # Elapsed-time-normalized smoothing (replaces fixed 30 Hz POSE_ALPHA).
    a_pose = _alpha(dt, POSE_TAU_SEC)
    a_ant = _alpha(dt, ANTENNA_TAU_SEC)
    smooth = {
        k: st.smooth_pose[k] + a_pose * (target_pose[k] - st.smooth_pose[k])
        for k in POSE_AXES
    }
    smooth_body = st.smooth_body_yaw + a_pose * (target_body - st.smooth_body_yaw)
    smooth_ants = [
        st.smooth_antennas[i] + a_ant * (target_ants[i] - st.smooth_antennas[i])
        for i in range(2)
    ]

    new_mode: Mode = st.mode
    if st.mode not in {"resetting", "fault"}:
        new_mode = "engaged" if st.engaged else "idle"

    st = replace(
        st,
        desired_pose=dict(target_pose),
        body_yaw=target_body,
        smooth_pose=smooth,
        smooth_body_yaw=smooth_body,
        smooth_antennas=smooth_ants,
        mode=new_mode,
    )

    if st.sends_frozen or not st.seeded:
        return StepResult(
            state=st,
            command=None,
            mode_changed=(st.mode != prev_mode),
        )

    send_pose, send_body = slew_limit(
        st.baseline_pose,
        st.baseline_body_yaw,
        smooth,
        smooth_body,
        dt,
    )
    command = Command(
        pose=send_pose,
        body_yaw=send_body,
        antennas=(smooth_ants[0], smooth_ants[1]),
    )
    return StepResult(
        state=st,
        command=command,
        mode_changed=(st.mode != prev_mode),
    )


def note_sample_receipt(state: ControlState, receipt_time: float) -> ControlState:
    return replace(state, last_sample_time=receipt_time, have_sample=True)

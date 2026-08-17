"""
Pure control reducer for protocol v4.

Owns clutch mapping, body follow, antenna phase, workspace projection, smoothing,
stale release, and slew limiting. No I/O.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from esp32_motion_controller.protocol import GAIN_MAX, GAIN_MIN, Sample

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

STALE_PACKET_SEC = 0.600
CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

# Match v1 feel: POSE_ALPHA=0.12 at 33 ms ≈ tau ~0.26 s
POSE_TAU_SEC = 0.255
ANTENNA_TAU_SEC = 0.39

# Hard SDK-boundary speed lock. Per-axis linear rotation / Euclidean xyz.
MAX_ANGULAR_VEL = 1.5  # rad/s (~86 deg/s), matching Spectacles
MAX_POS_VEL = 0.030  # 30 mm/s
MAX_DT_FOR_VEL_CLAMP = 0.05
# Appear/disappear slews (BOOT engage/clutch) run faster than streaming.
ANIM_VEL_MULT = 2.5

LIMIT_BODY_YAW_RAD = 160.0 * math.pi / 180.0
LIMIT_HEAD_YAW_RAD = math.radians(150.0)  # 30° of margin from the ±180° IK wrap
LIMIT_HEAD_BODY_YAW_DELTA_RAD = 65.0 * math.pi / 180.0

LIMIT_HEAD_X_MIN = -0.020
LIMIT_HEAD_X_MAX = 0.020
LIMIT_HEAD_Y_MIN = -0.020
LIMIT_HEAD_Y_MAX = 0.020
LIMIT_HEAD_Z_MIN = 0.0
LIMIT_HEAD_Z_MAX = 0.025

ELLIPSOID_ROLL_MAX_RAD = 25.0 * math.pi / 180.0
ELLIPSOID_PITCH_MAX_RAD = 25.0 * math.pi / 180.0

# Nominal hold pose: screen toward user, USB + buttons down → device +Y is up.
# Matches IMU_MAP_* and imu_gravity_sane() in firmware/main/config.h.
HOLD_TILT = R.from_euler("x", math.pi / 2.0)

# Pitch/roll: this much board tilt from the engage snapshot spans the head's ±25°.
PITCH_ROLL_WINDOW_RAD = math.radians(40.0)
PITCH_ROLL_SCALE = ELLIPSOID_PITCH_MAX_RAD / PITCH_ROLL_WINDOW_RAD  # 25/40 = 0.625

# Device-frame rotation gains, applied to the clutch-relative rotation vector
# *before* DEV_TO_HEAD. Body axes after IMU_MAP / firmware ui.c:
#   X = tip top toward user (forward tilt)
#   Y = USB-down in-place turn (horizontal pan) — 1:1
#   Z = raise right edge (sideways roll)
FORWARD_GAIN = PITCH_ROLL_SCALE  # device X → head pitch (engage-relative)
SIDEWAYS_GAIN = PITCH_ROLL_SCALE  # device Z → head roll (engage-relative)

HEADING_MIN_HORIZ = 0.20  # device +X too vertical (board on its side) to give a heading
YAW_STOP_EPS = 1e-4
# Sign-cross hold: only when already past the neck stop, not near centre.
YAW_SIGN_HOLD_ABS = LIMIT_HEAD_BODY_YAW_DELTA_RAD

MAX_HEAD_YAW = 65.0 * math.pi / 180.0
BODY_FOLLOW_THRESHOLD = 40.0 * math.pi / 180.0
MAX_BODY_YAW = 160.0 * math.pi / 180.0

DEFAULT_LIVELINESS = 1.25
DEFAULT_GAZE_RESPONSIVENESS = 1.2
DEFAULT_ANTENNA_ACTIVITY = 0.8
HEAD_MOVE_SPEED = 0.06
MAX_HEAD_DELTA_DEG = 2.0
ANTENNA_AMPLITUDE_DEG = 15.0
APPEAR_ANTENNA_SPEED_MULT = 2.0

IK_FAIL_RETRACT_TARGET_ALPHA = 0.06
IK_FAIL_CONSECUTIVE_THRESHOLD = 3

# Disengaged rest: 5 cm down from reset, slight nod.
DISENGAGED_Z = -0.050
DISENGAGED_PITCH = math.radians(5.0)

NEAR_POSE_EPS_M = 0.005
NEAR_POSE_EPS_RAD = math.radians(3.0)

Mode = str  # idle | engaged | resetting | fault


def zero_pose() -> dict[str, float]:
    return {k: 0.0 for k in POSE_AXES}


def disengaged_rest_pose() -> dict[str, float]:
    pose = zero_pose()
    pose["z"] = DISENGAGED_Z
    pose["pitch"] = DISENGAGED_PITCH
    return pose


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_delta(from_ang: float, to_ang: float) -> float:
    """Shortest signed delta from `from_ang` to `to_ang`, wrapping at ±pi."""
    d = (to_ang - from_ang + math.pi) % (2.0 * math.pi) - math.pi
    return d


def _finite_quat(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(arr)) or np.linalg.norm(arr) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return arr / np.linalg.norm(arr)


def _wxyz_to_rotation(q: np.ndarray) -> R:
    return R.from_quat([q[1], q[2], q[3], q[0]])


def _rotation_to_wxyz(r: R) -> np.ndarray:
    x, y, z, w = r.as_quat()
    return np.array([w, x, y, z], dtype=np.float64)


def _right_edge_xy(r: R) -> tuple[float, float]:
    """World-XY of device +X (right edge). Nod about +X leaves this vector still."""
    v = r.apply(np.array([1.0, 0.0, 0.0]))
    return float(v[0]), float(v[1])


def _heading(r: R) -> float:
    """Rotation about world up from the board's right-edge azimuth.

    Nod (device +X) does not move this vector, so pitch cannot leak into yaw.
    """
    x, y = _right_edge_xy(r)
    return math.atan2(y, x)


def _heading_stable(r: R, heading_last: float | None) -> float:
    """Heading from the right edge; freeze if the board is rolled onto its side."""
    x, y = _right_edge_xy(r)
    if math.hypot(x, y) < HEADING_MIN_HORIZ:
        if heading_last is not None:
            return float(heading_last)
        return math.atan2(y, x)
    return math.atan2(y, x)


def hold_reference(q_device: np.ndarray) -> np.ndarray:
    """Clutch reference with the nominal hold tilt at the device's heading.

    Gravity pins tilt absolutely; heading is unobservable without a
    magnetometer, so it is latched from the device at engage.
    """
    r_dev = _wxyz_to_rotation(q_device)
    dh = _wrap_delta(_heading(HOLD_TILT), _heading(r_dev))
    return _rotation_to_wxyz(R.from_euler("z", dh) * HOLD_TILT)


def _relative_device_rotation(q_ref: np.ndarray, q_device: np.ndarray) -> R:
    r_ref = _wxyz_to_rotation(q_ref)
    r_dev = _wxyz_to_rotation(q_device)
    return r_ref.inv() * r_dev


def _device_rotation_to_head_rpy(r_rel_dev: R) -> tuple[float, float, float]:
    m = R.from_matrix(DEV_TO_HEAD)
    r_head = m * r_rel_dev * m.inv()
    roll, pitch, yaw = r_head.as_euler("xyz", degrees=False)
    return float(roll), float(pitch), float(yaw)


def quat_relative_rpy(q_ref: np.ndarray, q_device: np.ndarray) -> tuple[float, float, float]:
    r_rel_dev = _relative_device_rotation(q_ref, q_device)
    return _device_rotation_to_head_rpy(r_rel_dev)


def scale_device_rotation(r_rel_dev: R) -> R:
    """Scale rotation about each ESP32 body axis (tilt window; yaw is 1:1)."""
    rv = r_rel_dev.as_rotvec()
    rv[0] *= FORWARD_GAIN
    rv[2] *= SIDEWAYS_GAIN
    return R.from_rotvec(rv)


def relative_head_rpy(q_ref: np.ndarray, q_device: np.ndarray) -> tuple[float, float, float]:
    """Device-to-head rpy with per-axis physical-motion gains."""
    r_rel_dev = scale_device_rotation(_relative_device_rotation(q_ref, q_device))
    return _device_rotation_to_head_rpy(r_rel_dev)


def tilt_head_rp(
    q_ref: np.ndarray,
    q_device: np.ndarray,
    heading_dev: float,
) -> tuple[float, float]:
    """Roll/pitch from relative rotation after stripping world-up heading."""
    r_ref = _wxyz_to_rotation(q_ref)
    r_dev = _wxyz_to_rotation(q_device)
    r_ref_unyaw = R.from_euler("z", -_heading(r_ref)) * r_ref
    r_dev_unyaw = R.from_euler("z", -heading_dev) * r_dev
    r_tilt = r_ref_unyaw.inv() * r_dev_unyaw
    roll, pitch, _ = _device_rotation_to_head_rpy(scale_device_rotation(r_tilt))
    return float(roll), float(pitch)


def dual_sine(t: float, freq_a: float, freq_b: float) -> float:
    return math.sin(t * freq_a) * 0.6 + math.sin(t * freq_b) * 0.4


def clamp_stewart_ellipsoid(
    x: float, y: float, z: float, roll: float, pitch: float,
) -> tuple[float, float, float, float, float]:
    return (
        _clamp(x, LIMIT_HEAD_X_MIN, LIMIT_HEAD_X_MAX),
        _clamp(y, LIMIT_HEAD_Y_MIN, LIMIT_HEAD_Y_MAX),
        _clamp(z, LIMIT_HEAD_Z_MIN, LIMIT_HEAD_Z_MAX),
        _clamp(roll, -ELLIPSOID_ROLL_MAX_RAD, ELLIPSOID_ROLL_MAX_RAD),
        _clamp(pitch, -ELLIPSOID_PITCH_MAX_RAD, ELLIPSOID_PITCH_MAX_RAD),
    )


def clamp_pose_to_daemon_limits(
    pose: dict[str, float],
    body_yaw: float,
    *,
    apply_ellipsoid: bool = True,
) -> tuple[dict[str, float], float]:
    out_pose = dict(pose)
    if apply_ellipsoid:
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


def _yaw_positive_stop(body_yaw: float) -> float:
    body_c = _clamp(body_yaw, -LIMIT_BODY_YAW_RAD, LIMIT_BODY_YAW_RAD)
    return min(LIMIT_HEAD_YAW_RAD, body_c + LIMIT_HEAD_BODY_YAW_DELTA_RAD)


def _yaw_negative_stop(body_yaw: float) -> float:
    body_c = _clamp(body_yaw, -LIMIT_BODY_YAW_RAD, LIMIT_BODY_YAW_RAD)
    return max(-LIMIT_HEAD_YAW_RAD, body_c - LIMIT_HEAD_BODY_YAW_DELTA_RAD)


def _yaw_at_positive_stop(yaw: float, body_yaw: float) -> bool:
    return yaw >= _yaw_positive_stop(body_yaw) - YAW_STOP_EPS


def _yaw_at_negative_stop(yaw: float, body_yaw: float) -> bool:
    return yaw <= _yaw_negative_stop(body_yaw) + YAW_STOP_EPS


def _hold_sign_cross(baseline: float, desired: float, zone: float) -> float:
    """Hold `baseline` if a limit-zone command would chase the opposite sign."""
    if abs(baseline) >= zone - YAW_STOP_EPS and baseline * desired < 0.0:
        return baseline
    return desired


def _alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return 1.0 - math.exp(-max(dt, 0.0) / tau)


def _pose_rotation(pose: dict[str, float]) -> R:
    return R.from_euler(
        "xyz", [pose["roll"], pose["pitch"], pose["yaw"]], degrees=False
    )


def speed_lock(
    baseline: dict[str, float],
    baseline_body: float,
    desired: dict[str, float],
    desired_body: float,
    dt: float,
    *,
    apply_ellipsoid: bool = True,
    max_ang_vel: float = MAX_ANGULAR_VEL,
    max_pos_vel: float = MAX_POS_VEL,
) -> tuple[dict[str, float], float]:
    """Cap one streaming command against the last delivered pose.

    Roll, pitch, yaw, and body yaw each get an independent angular-velocity
    budget (linear deltas — yaw is a bounded axis, not a wrap). Positional
    slews (appear/disappear/reset) use Euclidean distance. `dt` is capped so
    stalls/reconnects cannot accumulate permission for a snap.
    """
    dt_c = min(max(dt, 0.0), MAX_DT_FOR_VEL_CLAMP)
    max_d_ang = max_ang_vel * dt_c
    max_d_pos = max_pos_vel * dt_c
    send = {k: float(desired[k]) for k in POSE_AXES}

    dp = np.array([desired[k] - baseline[k] for k in POSITIONAL_AXES], dtype=np.float64)
    dist = float(np.linalg.norm(dp))
    if dist > max_d_pos and dist > 0.0:
        scale = max_d_pos / dist
        for i, axis in enumerate(POSITIONAL_AXES):
            send[axis] = baseline[axis] + float(dp[i]) * scale
    else:
        for axis in POSITIONAL_AXES:
            send[axis] = desired[axis]

    hold_yaw = _hold_sign_cross(baseline["yaw"], desired["yaw"], YAW_SIGN_HOLD_ABS)
    hold_body = _hold_sign_cross(baseline_body, desired_body, LIMIT_BODY_YAW_RAD)

    for axis in ("roll", "pitch"):
        delta = desired[axis] - baseline[axis]
        if abs(delta) > max_d_ang:
            send[axis] = baseline[axis] + math.copysign(max_d_ang, delta)
        else:
            send[axis] = desired[axis]
    yaw_delta = hold_yaw - baseline["yaw"]
    if abs(yaw_delta) > max_d_ang:
        send["yaw"] = baseline["yaw"] + math.copysign(max_d_ang, yaw_delta)
    else:
        send["yaw"] = hold_yaw
    body_delta = hold_body - baseline_body
    if abs(body_delta) > max_d_ang:
        body_delta = math.copysign(max_d_ang, body_delta)
    send_body = baseline_body + body_delta
    return clamp_pose_to_daemon_limits(
        send, send_body, apply_ellipsoid=apply_ellipsoid
    )


def slew_limit(
    baseline: dict[str, float],
    baseline_body: float,
    desired: dict[str, float],
    desired_body: float,
    dt: float,
    *,
    apply_ellipsoid: bool = True,
) -> tuple[dict[str, float], float]:
    return speed_lock(
        baseline,
        baseline_body,
        desired,
        desired_body,
        dt,
        apply_ellipsoid=apply_ellipsoid,
    )


def pose_travel(
    from_pose: dict[str, float],
    to_pose: dict[str, float],
    from_body: float,
    to_body: float,
) -> tuple[float, float]:
    """Return (euclidean_m, geodesic_plus_body_rad) between two poses."""
    dp = np.array(
        [to_pose[k] - from_pose[k] for k in POSITIONAL_AXES], dtype=np.float64
    )
    dist = float(np.linalg.norm(dp))
    rel = _pose_rotation(from_pose).inv() * _pose_rotation(to_pose)
    # Geodesic treats +π and −π as identical; linear yaw catches a wrap attempt.
    ang = max(float(rel.magnitude()), abs(to_pose["yaw"] - from_pose["yaw"]))
    ang += abs(_wrap_delta(from_body, to_body))
    return dist, ang


def pose_near(
    from_pose: dict[str, float],
    to_pose: dict[str, float],
    from_body: float = 0.0,
    to_body: float = 0.0,
) -> bool:
    dist, ang = pose_travel(from_pose, to_pose, from_body, to_body)
    return dist < NEAR_POSE_EPS_M and ang < NEAR_POSE_EPS_RAD


@dataclass
class ControlState:
    mode: Mode = "idle"
    robot_available: bool = True
    error: str | None = None

    engaged: bool = False
    was_engaged: bool = False
    gain: float = 1.0
    ready: bool = False

    q_ref: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    heading_last: float | None = None
    yaw_unwrapped: float = 0.0
    heading_unwrapped: float = 0.0
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
    apply_ellipsoid: bool = True,
) -> ControlState:
    ants = list(antennas) if antennas is not None else [0.0, 0.0]
    ants = (ants + [0.0, 0.0])[:2]
    pose_c, body_c = clamp_pose_to_daemon_limits(
        {k: float(pose.get(k, 0.0)) for k in POSE_AXES},
        float(body_yaw),
        apply_ellipsoid=apply_ellipsoid,
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
        heading_last=None,
        yaw_unwrapped=0.0,
        heading_unwrapped=0.0,
        mode="idle" if state.mode != "resetting" else state.mode,
        error=None,
    )


def rebase_neutral(state: ControlState, *, measured_baseline: dict[str, float] | None,
                   measured_body: float | None) -> ControlState:
    """After appear / settings reset: targets at neutral; baseline from measured pose."""
    return rebase_to_pose(
        state,
        target=zero_pose(),
        target_body=0.0,
        measured_baseline=measured_baseline,
        measured_body=measured_body,
    )


def rebase_to_pose(
    state: ControlState,
    *,
    target: dict[str, float],
    target_body: float,
    measured_baseline: dict[str, float] | None,
    measured_body: float | None,
) -> ControlState:
    """After exclusive goto: hold `target`; baseline from measured pose if provided."""
    target_c, body_c = clamp_pose_to_daemon_limits(
        {k: float(target.get(k, 0.0)) for k in POSE_AXES},
        float(target_body),
        apply_ellipsoid=False,
    )
    if measured_baseline is None or measured_body is None:
        return replace(
            state,
            base_pose=dict(target_c),
            desired_pose=dict(target_c),
            smooth_pose=dict(target_c),
            body_yaw=body_c,
            smooth_body_yaw=body_c,
            antenna_left=0.0,
            antenna_right=0.0,
            smooth_antennas=[0.0, 0.0],
            engaged=False,
            was_engaged=False,
            heading_last=None,
            yaw_unwrapped=0.0,
            heading_unwrapped=0.0,
            q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
            sends_frozen=True,
            mode="fault",
            error="robot pose unread after reset",
            behavior_t0=state.behavior_t0,
        )
    pose_c, meas_body = clamp_pose_to_daemon_limits(
        measured_baseline, measured_body, apply_ellipsoid=False
    )
    return replace(
        state,
        base_pose=dict(target_c),
        desired_pose=dict(target_c),
        smooth_pose=dict(target_c),
        baseline_pose=dict(pose_c),
        body_yaw=body_c,
        smooth_body_yaw=body_c,
        baseline_body_yaw=meas_body,
        antenna_left=0.0,
        antenna_right=0.0,
        smooth_antennas=[0.0, 0.0],
        baseline_antennas=[0.0, 0.0],
        engaged=False,
        was_engaged=False,
        heading_last=None,
        yaw_unwrapped=0.0,
        heading_unwrapped=0.0,
        q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
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
        heading_last=None,
        yaw_unwrapped=0.0,
        heading_unwrapped=0.0,
        q_ref=np.array([1.0, 0.0, 0.0, 0.0]),
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
    gain = _clamp(float(sample.gain), GAIN_MIN, GAIN_MAX)
    q_dev = _finite_quat(sample.q)
    want = bool(sample.engaged) and bool(sample.ready) and allow_engage
    rising = want and not state.was_engaged
    falling = (not want) and state.was_engaged

    q_ref = state.q_ref
    heading_last = state.heading_last
    yaw_unwrapped = state.yaw_unwrapped
    heading_unwrapped = state.heading_unwrapped
    base = dict(state.base_pose)
    desired = dict(state.desired_pose)

    if rising:
        # Snapshot the device attitude as IMU zero: roll/pitch/yaw are
        # relative to this grab. Prior head pose stays in base_pose; x/y/z
        # hold at that base while engaged.
        q_ref = q_dev.copy()
        r_dev = _wxyz_to_rotation(q_dev)
        heading_last = _heading_stable(r_dev, None)
        yaw_unwrapped = 0.0
        heading_unwrapped = 0.0

    if want:
        r_dev = _wxyz_to_rotation(q_dev)
        h = _heading_stable(r_dev, heading_last)
        d = _wrap_delta(heading_last, h) if heading_last is not None else 0.0
        heading_unwrapped = heading_unwrapped + d
        heading_last = h
        yaw = heading_unwrapped
        roll, pitch = tilt_head_rp(q_ref, q_dev, h)
        desired = {
            "x": base["x"],
            "y": base["y"],
            "z": base["z"],
            "roll": base["roll"] + gain * roll,
            "pitch": base["pitch"] + gain * pitch,
            "yaw": base["yaw"] + gain * yaw,
        }
        prev_at_pos = _yaw_at_positive_stop(state.desired_pose["yaw"], state.body_yaw)
        prev_at_neg = _yaw_at_negative_stop(state.desired_pose["yaw"], state.body_yaw)
        if heading_unwrapped > yaw_unwrapped and prev_at_pos:
            desired["yaw"] = base["yaw"] + gain * yaw_unwrapped
        elif heading_unwrapped < yaw_unwrapped and prev_at_neg:
            desired["yaw"] = base["yaw"] + gain * yaw_unwrapped
        else:
            yaw_unwrapped = heading_unwrapped
    elif falling:
        base = dict(desired)

    return replace(
        state,
        ready=bool(sample.ready),
        gain=gain,
        q_ref=q_ref,
        heading_last=heading_last,
        yaw_unwrapped=yaw_unwrapped,
        heading_unwrapped=heading_unwrapped,
        base_pose=base,
        desired_pose=desired,
        engaged=want,
        was_engaged=want,
    )


def advance_antennas(
    state: ControlState,
    now: float,
    dt: float,
    *,
    speed_mult: float = 1.0,
) -> ControlState:
    """Advance the idle antenna wiggle toward its dual-sine target."""
    tick_scale = dt / 0.033 if dt > 0 else 1.0
    t = now - state.behavior_t0
    deg = math.pi / 180.0

    yaw_smoothing = HEAD_MOVE_SPEED * state.gaze_responsiveness * tick_scale
    antenna_smoothing = yaw_smoothing * 1.5
    effective_ant_amp = ANTENNA_AMPLITUDE_DEG * state.antenna_activity * deg
    ant_speed = (0.5 + state.antenna_activity * 0.5) * speed_mult

    desired_l = dual_sine(t * ant_speed, 1.3, 3.11) * effective_ant_amp
    desired_r = dual_sine(t * ant_speed, 1.7, 2.73) * effective_ant_amp
    ant_l = state.antenna_left + (desired_l - state.antenna_left) * antenna_smoothing
    ant_r = state.antenna_right + (desired_r - state.antenna_right) * antenna_smoothing

    return replace(state, antenna_left=ant_l, antenna_right=ant_r)


def _advance_behavior(state: ControlState, head_yaw: float, now: float, dt: float) -> ControlState:
    # Normalize behavior update against the legacy ~33 ms tick using dt scaling.
    tick_scale = dt / 0.033 if dt > 0 else 1.0
    deg = math.pi / 180.0

    yaw_smoothing = HEAD_MOVE_SPEED * state.gaze_responsiveness * tick_scale
    max_yaw_delta = MAX_HEAD_DELTA_DEG * state.gaze_responsiveness * deg * tick_scale
    body_smoothing = yaw_smoothing * 0.7 * (0.3 + state.liveliness * 0.4)

    body_yaw = state.body_yaw
    rel_yaw = head_yaw - body_yaw
    if abs(rel_yaw) > BODY_FOLLOW_THRESHOLD:
        excess = abs(rel_yaw) - BODY_FOLLOW_THRESHOLD
        step = math.copysign(excess * body_smoothing * 8, rel_yaw)
        body_yaw += _clamp(step, -max_yaw_delta, max_yaw_delta)
        body_yaw = _clamp(body_yaw, -MAX_BODY_YAW, MAX_BODY_YAW)

    st = advance_antennas(state, now, dt)
    return replace(st, body_yaw=body_yaw)


def step(
    state: ControlState,
    *,
    now: float,
    dt: float,
    sample: Sample | None,
    sample_is_fresh: bool,
    controller_present: bool = True,
) -> StepResult:
    """Advance one control tick.

    `sample` is the latest validated sample (may be None before first packet).
    `sample_is_fresh` is True only when a new sample arrived since the previous tick.
    Age-based stale release runs only when the controller socket is down.
    """
    prev_mode = state.mode
    st = state

    if st.mode == "resetting":
        # No streaming commands while reset owns the robot.
        return StepResult(state=st, command=None, mode_changed=False)

    if (
        not controller_present
        and st.have_sample
        and st.last_sample_time > 0
        and now - st.last_sample_time > STALE_PACKET_SEC
        and st.engaged
    ):
        st = force_disengage(st)

    if sample is not None and sample_is_fresh and st.mode != "resetting":
        allow_engage = st.mode != "fault" and not st.sends_frozen
        st = _update_clutch(st, sample, allow_engage=allow_engage)
        st = replace(st, have_sample=True)

    apply_ellipsoid = bool(st.engaged)
    # Clamp before body follow so the neck never chases unclamped clutch yaw.
    target_pose, _ = clamp_pose_to_daemon_limits(
        st.desired_pose, st.body_yaw, apply_ellipsoid=apply_ellipsoid
    )
    if st.mode not in {"resetting"}:
        st = _advance_behavior(st, target_pose["yaw"], now, dt)
    target_pose, target_body = clamp_pose_to_daemon_limits(
        st.desired_pose, st.body_yaw, apply_ellipsoid=apply_ellipsoid
    )
    target_ants = [st.antenna_left, st.antenna_right]

    # Elapsed-time-normalized smoothing (replaces fixed 30 Hz POSE_ALPHA).
    # Engaged streaming skips pose/body EMA so a new tilt is applied this tick;
    # the per-axis speed lock is the snap guard.
    a_pose = 1.0 if st.engaged else _alpha(dt, POSE_TAU_SEC)
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

    send_pose, send_body = speed_lock(
        st.baseline_pose,
        st.baseline_body_yaw,
        smooth,
        smooth_body,
        dt,
        apply_ellipsoid=apply_ellipsoid,
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

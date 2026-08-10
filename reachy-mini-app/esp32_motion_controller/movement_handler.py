"""
Movement state and robot commands: target/current LERP and set_target rate limiting.

Ported from spectacles_reachy_mini.movement_handler with a five-axis Stewart
ellipsoid that couples x/y translation into the workspace check.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
import uuid as uuid_mod
from typing import Any

import numpy as np
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)

POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")
ANGULAR_AXES = ("roll", "pitch", "yaw")
POSITIONAL_AXES = ("x", "y", "z")

POSE_ALPHA = 0.12
ANTENNA_ALPHA = 0.08
# Hard safety gate (same numbers as spectacles_reachy_mini). set_target on the
# daemon is instantaneous — these caps on consecutive sends are what keep the
# head from whipping when bookkeeping and the robot disagree.
MAX_ANGULAR_VEL = 1.5  # rad/s
MAX_POS_VEL = 0.05  # m/s
LOOP_INTERVAL = 0.033
DEFAULT_SEND_RATE_HZ = 20.0
SEND_RATE_HZ_MIN = 5.0
SEND_RATE_HZ_MAX = 50.0
MAX_DT_FOR_VEL_CLAMP = 0.05  # never allow a single step larger than 50 ms worth
RESYNC_FROM_ROBOT_SEC = 2.0
# Measured vs bookkeeping: within this, soft-refresh _prev_sent; beyond, hard seed.
RESYNC_EPS_POS_M = 0.003
RESYNC_EPS_ANG_RAD = 0.05

LIMIT_BODY_YAW_RAD = 160.0 * math.pi / 180.0
LIMIT_HEAD_YAW_RAD = math.pi
LIMIT_HEAD_BODY_YAW_DELTA_RAD = 65.0 * math.pi / 180.0

# Opened conservatively for controller translation (±20 mm).
LIMIT_HEAD_X_MIN = -0.020
LIMIT_HEAD_X_MAX = 0.020
LIMIT_HEAD_Y_MIN = -0.020
LIMIT_HEAD_Y_MAX = 0.020
LIMIT_HEAD_Z_MIN = 0.0
LIMIT_HEAD_Z_MAX = 0.025

# Stewart workspace: motor_arm≈0.04 m, rod≈0.085 m → pitch/roll ≈ ±25°, Z ≈ ±0.03 m.
# Rotation is clamped to its own radii first; translation then fits the remainder
# so junk/large translation cannot scale a real tilt down.
ELLIPSOID_X_MAX = 0.015
ELLIPSOID_Y_MAX = 0.015
ELLIPSOID_Z_MAX = 0.018
ELLIPSOID_ROLL_MAX_RAD = 25.0 * math.pi / 180.0
ELLIPSOID_PITCH_MAX_RAD = 25.0 * math.pi / 180.0

IK_FAIL_RETRACT_TARGET_ALPHA = 0.06
IK_FAIL_CONSECUTIVE_THRESHOLD = 3


def _zero_pose() -> dict[str, float]:
    return {k: 0.0 for k in POSE_AXES}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _parse_send_rate_hz(value: float | None) -> float:
    if value is None:
        return 1.0 / DEFAULT_SEND_RATE_HZ
    rate = max(SEND_RATE_HZ_MIN, min(SEND_RATE_HZ_MAX, value))
    return 1.0 / rate


def _clamp_stewart_ellipsoid(
    x: float, y: float, z: float, roll: float, pitch: float,
) -> tuple[float, float, float, float, float]:
    """Clamp rotation first, then fit translation into the remaining budget.

    Budget: (x/X)^2 + (y/Y)^2 + (z/Z)^2 + (roll/R)^2 + (pitch/P)^2 <= 1.
    Roll/pitch are hard-clamped to their radii (never scaled by translation).
    Translation is then projected onto whatever budget remains.
    """
    z_clamped = _clamp(z, LIMIT_HEAD_Z_MIN, LIMIT_HEAD_Z_MAX)
    x_clamped = _clamp(x, LIMIT_HEAD_X_MIN, LIMIT_HEAD_X_MAX)
    y_clamped = _clamp(y, LIMIT_HEAD_Y_MIN, LIMIT_HEAD_Y_MAX)

    roll_c = _clamp(roll, -ELLIPSOID_ROLL_MAX_RAD, ELLIPSOID_ROLL_MAX_RAD)
    pitch_c = _clamp(pitch, -ELLIPSOID_PITCH_MAX_RAD, ELLIPSOID_PITCH_MAX_RAD)

    nr = roll_c / ELLIPSOID_ROLL_MAX_RAD if ELLIPSOID_ROLL_MAX_RAD > 0 else 0.0
    np_ = pitch_c / ELLIPSOID_PITCH_MAX_RAD if ELLIPSOID_PITCH_MAX_RAD > 0 else 0.0
    rot_budget = nr * nr + np_ * np_
    # Tiny epsilon so a full-scale tilt leaves a sliver for translation=0.
    remaining = max(0.0, 1.0 - rot_budget)

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


def _clamp_pose_to_daemon_limits(
    pose: dict[str, float], body_yaw: float
) -> tuple[dict[str, float], float]:
    out_pose = dict(pose)
    cx, cy, cz, cr, cp = _clamp_stewart_ellipsoid(
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
    return (out_pose, body_yaw_clamped)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


class MovementHandler:
    """Owns all movement state and SDK interaction."""

    def __init__(
        self,
        reachy_mini: ReachyMini | None,
        send_rate_hz: float | None = None,
    ) -> None:
        self.mini = reachy_mini
        self._send_min_interval = _parse_send_rate_hz(send_rate_hz)
        logger.info(
            "MovementHandler send rate: %.1f Hz (interval %.3f s)",
            1.0 / self._send_min_interval,
            self._send_min_interval,
        )

        self._target_pose: dict[str, float] = _zero_pose()
        self._target_body_yaw: float = 0.0
        self._target_antennas: list[float] = [0.0, 0.0]

        self._current_pose: dict[str, float] = _zero_pose()
        self._current_body_yaw: float = 0.0
        self._current_antennas: list[float] = [0.0, 0.0]

        self._prev_sent_pose: dict[str, float] = _zero_pose()
        self._prev_sent_body_yaw: float = 0.0

        self._active_gotos: dict[str, bool] = {}
        self._apply_task: asyncio.Task[None] | None = None
        self._send_future: asyncio.Future[Any] | None = None
        self._last_send_time: float = 0.0
        self._send_count: int = 0
        self._send_seq: int = 0
        self._last_applied_seq: int = 0
        self._consecutive_ik_failures: int = 0
        self._last_resync_time: float = 0.0
        self._seeded: bool = False
        self._sends_frozen: bool = False

    @property
    def current_pose(self) -> dict[str, float]:
        return dict(self._current_pose)

    @property
    def target_pose(self) -> dict[str, float]:
        return dict(self._target_pose)

    def set_target(
        self,
        pose: dict[str, float],
        body_yaw: float | None = None,
        antennas: list[float] | None = None,
    ) -> None:
        merged = {}
        for k in POSE_AXES:
            v = pose.get(k, self._target_pose[k])
            merged[k] = (
                v
                if isinstance(v, (int, float)) and math.isfinite(v)
                else self._target_pose[k]
            )
        by = (
            body_yaw
            if body_yaw is not None
            and isinstance(body_yaw, (int, float))
            and math.isfinite(body_yaw)
            else self._target_body_yaw
        )
        self._target_pose, self._target_body_yaw = _clamp_pose_to_daemon_limits(
            merged, by
        )
        if antennas is not None:
            self._target_antennas = [
                a
                if isinstance(a, (int, float)) and math.isfinite(a)
                else (self._target_antennas[i] if i < len(self._target_antennas) else 0.0)
                for i, a in enumerate(antennas)
            ]
            self._target_antennas = (self._target_antennas + [0.0, 0.0])[:2]

    def goto(
        self,
        pose: dict[str, float],
        body_yaw: float = 0.0,
        antennas: list[float] | None = None,
        duration: float = 0.5,
        interpolation: str = "minjerk",
    ) -> str:
        move_uuid = str(uuid_mod.uuid4())
        self._active_gotos[move_uuid] = True

        start_pose = dict(self._current_pose)
        start_body_yaw = self._current_body_yaw
        start_antennas = list(self._current_antennas)
        end_pose = {k: pose.get(k, 0.0) for k in POSE_AXES}
        end_antennas = list(antennas) if antennas else [0.0, 0.0]

        asyncio.create_task(
            self._run_goto(
                move_uuid,
                start_pose, end_pose,
                start_body_yaw, body_yaw,
                start_antennas, end_antennas,
                duration, interpolation,
            )
        )
        return move_uuid

    def stop_move(self, move_uuid: str) -> bool:
        if move_uuid in self._active_gotos:
            self._active_gotos[move_uuid] = False
            return True
        return False

    def rebase_to_neutral(self) -> None:
        """Re-sync clamp reference after a reset goto; keep targets at neutral.

        Refresh `_prev_sent` from the robot so the velocity gate is honest, but
        do not copy the measured pose into targets — that would undo the goto.
        On read failure freeze sends; never invent a zero clamp reference.
        """
        if not self._seed_from_robot(update_target=False):
            logger.error("rebase_to_neutral: robot pose unread — freezing sends")
            self._sends_frozen = True
            return
        for axis in POSE_AXES:
            self._target_pose[axis] = 0.0
            self._current_pose[axis] = 0.0
        self._target_body_yaw = 0.0
        self._current_body_yaw = 0.0
        self._target_antennas = [0.0, 0.0]
        self._current_antennas = [0.0, 0.0]

    def _read_robot_pose(self) -> tuple[dict[str, float], float] | None:
        if self.mini is None:
            return None
        try:
            head = np.asarray(self.mini.get_current_head_pose(), dtype=np.float64)
            joints, _ = self.mini.get_current_joint_positions()
            body_yaw = float(joints[0])
        except Exception as exc:
            logger.warning("Could not read robot pose: %s", exc)
            return None
        rpy = Rotation.from_matrix(head[:3, :3]).as_euler("xyz")
        pose = {
            "x": float(head[0, 3]),
            "y": float(head[1, 3]),
            "z": float(head[2, 3]),
            "roll": float(rpy[0]),
            "pitch": float(rpy[1]),
            "yaw": float(rpy[2]),
        }
        return pose, body_yaw

    @staticmethod
    def _pose_gap(
        a: dict[str, float], a_yaw: float, b: dict[str, float], b_yaw: float
    ) -> tuple[float, float]:
        pos = math.sqrt(
            (a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2 + (a["z"] - b["z"]) ** 2
        )
        ang = max(
            abs(a["roll"] - b["roll"]),
            abs(a["pitch"] - b["pitch"]),
            abs(a["yaw"] - b["yaw"]),
            abs(a_yaw - b_yaw),
        )
        return pos, ang

    def _apply_seed_read(
        self,
        read: tuple[dict[str, float], float] | None,
        *,
        update_target: bool = True,
    ) -> bool:
        """Apply a pose read to bookkeeping. Returns False when read is None."""
        if read is None:
            self._sends_frozen = True
            return False
        pose, body_yaw = read
        self._prev_sent_pose = dict(pose)
        self._prev_sent_body_yaw = body_yaw
        if update_target:
            self._target_pose = dict(pose)
            self._current_pose = dict(pose)
            self._target_body_yaw = body_yaw
            self._current_body_yaw = body_yaw
            logger.info(
                "Seeded movement state from robot: pose=%s body_yaw=%.3f",
                {k: round(v, 4) for k, v in pose.items()},
                body_yaw,
            )
        self._last_resync_time = time.monotonic()
        self._seeded = True
        self._sends_frozen = False
        return True

    def _seed_from_robot(self, *, update_target: bool = True) -> bool:
        """Initialize pose bookkeeping from the robot's measured pose.

        Returns False when no robot is attached or the readback failed.
        On failure, bookkeeping is left untouched and sends freeze.
        """
        if self.mini is None:
            # log-only / no robot: treat as seeded at current bookkeeping
            self._seeded = True
            self._sends_frozen = False
            return True
        return self._apply_seed_read(self._read_robot_pose(), update_target=update_target)

    async def _periodic_resync(self) -> None:
        """Refresh clamp reference only when the robot is near bookkeeping.

        Pose reads go through the default executor so a USB round trip cannot
        block the asyncio event loop (and therefore WebSocket pongs).
        """
        if self.mini is None:
            self._last_resync_time = time.monotonic()
            return
        loop = asyncio.get_running_loop()
        read = await loop.run_in_executor(None, self._read_robot_pose)
        if read is None:
            self._sends_frozen = True
            return
        pose, body_yaw = read
        pos_gap, ang_gap = self._pose_gap(
            pose, body_yaw, self._prev_sent_pose, self._prev_sent_body_yaw
        )
        self._last_resync_time = time.monotonic()
        if pos_gap <= RESYNC_EPS_POS_M and ang_gap <= RESYNC_EPS_ANG_RAD:
            self._prev_sent_pose = dict(pose)
            self._prev_sent_body_yaw = body_yaw
            self._sends_frozen = False
            return
        # Hard desync: full seed, then resume under the velocity gate.
        logger.warning(
            "Pose desync pos=%.4f m ang=%.3f rad — full resync",
            pos_gap,
            ang_gap,
        )
        self._prev_sent_pose = dict(pose)
        self._prev_sent_body_yaw = body_yaw
        self._target_pose = dict(pose)
        self._current_pose = dict(pose)
        self._target_body_yaw = body_yaw
        self._current_body_yaw = body_yaw
        self._sends_frozen = False

    def resync_from_robot(self) -> bool:
        """Public: re-align velocity clamp with the robot after a reconnect."""
        return self._seed_from_robot(update_target=True)

    async def resync_from_robot_async(self, *, update_target: bool = True) -> bool:
        """Async resync — pose read runs in the executor so the event loop stays free."""
        if self.mini is None:
            self._seeded = True
            self._sends_frozen = False
            return True
        loop = asyncio.get_running_loop()
        read = await loop.run_in_executor(None, self._read_robot_pose)
        return self._apply_seed_read(read, update_target=update_target)

    def start(self) -> None:
        if self._apply_task is None or self._apply_task.done():
            if not self._seed_from_robot():
                logger.error("start: robot pose unread — sends frozen until resync")
            self._apply_task = asyncio.ensure_future(self._apply_loop())

    def stop(self) -> None:
        if self._apply_task is not None and not self._apply_task.done():
            self._apply_task.cancel()
            self._apply_task = None
        for uid in list(self._active_gotos):
            self._active_gotos[uid] = False
        self._active_gotos.clear()

    async def _apply_loop(self) -> None:
        loop = asyncio.get_running_loop()
        try:
            while True:
                now = time.monotonic()

                for axis in POSE_AXES:
                    self._current_pose[axis] += POSE_ALPHA * (
                        self._target_pose[axis] - self._current_pose[axis]
                    )
                self._current_body_yaw += POSE_ALPHA * (
                    self._target_body_yaw - self._current_body_yaw
                )
                for i in range(min(len(self._current_antennas), len(self._target_antennas))):
                    self._current_antennas[i] += ANTENNA_ALPHA * (
                        self._target_antennas[i] - self._current_antennas[i]
                    )

                interval_ok = (
                    self._last_send_time == 0
                    or (now - self._last_send_time) >= self._send_min_interval
                )
                # Serialize set_target: never dual in-flight (out-of-order
                # done-callbacks corrupt the velocity-clamp reference).
                previous_done = self._send_future is None or self._send_future.done()
                can_send = (
                    interval_ok
                    and previous_done
                    and not self._sends_frozen
                    and (self._seeded or self.mini is None)
                )

                if can_send and self.mini is not None:
                    # Hard resync overwrites targets — never fight an active goto
                    # (reset minjerk owns the trajectory exclusively).
                    if not self._active_gotos and (
                        self._last_resync_time == 0.0
                        or (now - self._last_resync_time) >= RESYNC_FROM_ROBOT_SEC
                    ):
                        await self._periodic_resync()
                        if self._sends_frozen:
                            await asyncio.sleep(LOOP_INTERVAL)
                            continue

                    dt_since_send = (
                        now - self._last_send_time
                        if self._last_send_time > 0
                        else LOOP_INTERVAL
                    )
                    dt_clamped = min(dt_since_send, MAX_DT_FOR_VEL_CLAMP)
                    max_d_ang = MAX_ANGULAR_VEL * dt_clamped
                    max_d_pos = MAX_POS_VEL * dt_clamped

                    send_pose: dict[str, float] = {}
                    for axis in ANGULAR_AXES:
                        delta = self._current_pose[axis] - self._prev_sent_pose[axis]
                        send_pose[axis] = self._prev_sent_pose[axis] + _clamp(
                            delta, -max_d_ang, max_d_ang
                        )
                    for axis in POSITIONAL_AXES:
                        delta = self._current_pose[axis] - self._prev_sent_pose[axis]
                        send_pose[axis] = self._prev_sent_pose[axis] + _clamp(
                            delta, -max_d_pos, max_d_pos
                        )
                    body_yaw_delta = self._current_body_yaw - self._prev_sent_body_yaw
                    send_body_yaw = self._prev_sent_body_yaw + _clamp(
                        body_yaw_delta, -max_d_ang, max_d_ang
                    )

                    head = create_head_pose(
                        x=send_pose["x"],
                        y=send_pose["y"],
                        z=send_pose["z"],
                        roll=send_pose["roll"],
                        pitch=send_pose["pitch"],
                        yaw=send_pose["yaw"],
                        degrees=False,
                    )
                    antennas_arr = np.array(self._current_antennas, dtype=np.float64)
                    self._send_seq += 1
                    send_seq = self._send_seq
                    sent_pose = dict(send_pose)
                    sent_body_yaw = send_body_yaw

                    def _do_set_target(
                        h=head, b=send_body_yaw, a=antennas_arr.copy()
                    ) -> None:
                        try:
                            self.mini.set_target(head=h, body_yaw=b, antennas=a)
                        except Exception as exc:
                            logger.warning("set_target failed: %s", exc)
                            raise

                    self._send_future = loop.run_in_executor(None, _do_set_target)

                    def _on_send_done(fut: asyncio.Future[Any]) -> None:
                        if send_seq < self._last_applied_seq:
                            return
                        if fut.exception() is None:
                            self._prev_sent_pose = sent_pose
                            self._prev_sent_body_yaw = sent_body_yaw
                            self._last_applied_seq = send_seq
                            self._consecutive_ik_failures = 0
                        else:
                            self._consecutive_ik_failures += 1
                            if self._consecutive_ik_failures >= IK_FAIL_CONSECUTIVE_THRESHOLD:
                                # Pull only the target back toward neutral. Never
                                # move _prev_sent — that is the clamp reference.
                                alpha_t = IK_FAIL_RETRACT_TARGET_ALPHA
                                for ax in POSE_AXES:
                                    self._target_pose[ax] *= 1.0 - alpha_t
                                self._target_body_yaw *= 1.0 - alpha_t

                    self._send_future.add_done_callback(_on_send_done)
                    self._last_send_time = now
                    self._send_count += 1
                elif can_send and self.mini is None:
                    self._prev_sent_pose = dict(self._current_pose)
                    self._prev_sent_body_yaw = self._current_body_yaw
                    self._last_send_time = now

                await asyncio.sleep(LOOP_INTERVAL)
        except asyncio.CancelledError:
            pass

    async def _run_goto(
        self,
        move_uuid: str,
        start_pose: dict[str, float],
        end_pose: dict[str, float],
        start_body_yaw: float,
        end_body_yaw: float,
        start_antennas: list[float],
        end_antennas: list[float],
        duration: float,
        interpolation: str,
    ) -> None:
        t0 = time.monotonic()
        while self._active_gotos.get(move_uuid, False):
            elapsed = time.monotonic() - t0
            t = min(elapsed / max(duration, 0.001), 1.0)
            s = self._ease(t, interpolation)
            lerped = {k: _lerp(start_pose[k], end_pose[k], s) for k in POSE_AXES}
            by = _lerp(start_body_yaw, end_body_yaw, s)
            self._target_pose, self._target_body_yaw = _clamp_pose_to_daemon_limits(
                lerped, by
            )
            self._target_antennas = [
                _lerp(start_antennas[i], end_antennas[i], s)
                for i in range(min(len(start_antennas), len(end_antennas)))
            ]
            if t >= 1.0:
                break
            await asyncio.sleep(LOOP_INTERVAL)
        self._active_gotos.pop(move_uuid, None)

    @staticmethod
    def _ease(t: float, mode: str) -> float:
        if mode == "minjerk":
            return t * t * t * (10 + t * (-15 + t * 6))
        if mode == "ease":
            if t < 0.5:
                return 4 * t * t * t
            return 1 - ((-2 * t + 2) ** 3) / 2
        if mode == "cartoon":
            c = 1.70158
            c3 = c + 1
            return 1 + c3 * ((t - 1) ** 3) + c * ((t - 1) ** 2)
        return t

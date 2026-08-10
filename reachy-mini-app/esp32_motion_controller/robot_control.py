"""
Single fixed-rate robot command owner for protocol v2.

Consumes the latest sample from SessionHub, runs the pure reducer, and performs
at most one SDK call in flight. Owns reset goto completion and pose seeding.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from esp32_motion_controller.control import (
    CONTROL_DT,
    CONTROL_HZ,
    Command,
    ControlState,
    begin_reset,
    force_disengage,
    initial_state,
    mark_pose_unread,
    mark_sdk_failure,
    mark_sdk_success,
    note_sample_receipt,
    rebase_neutral,
    seed_from_pose,
    step,
    zero_pose,
)
from esp32_motion_controller.session import SessionHub

logger = logging.getLogger(__name__)

RESET_DURATION_SEC = 1.5
IDLE_RECONCILE_SEC = 2.0
RESYNC_EPS_POS_M = 0.003
RESYNC_EPS_ANG_RAD = 0.05


class RobotGateway:
    """Thin SDK adapter — all blocking calls run in an executor by RobotControl."""

    def __init__(self, reachy_mini: Any | None, *, log_only: bool = False) -> None:
        self.mini = reachy_mini
        self.log_only = log_only

    def read_pose(self) -> tuple[dict[str, float], float] | None:
        if self.mini is None:
            return zero_pose(), 0.0
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

    def set_target(self, command: Command) -> None:
        if self.mini is None or self.log_only:
            return
        from reachy_mini.utils import create_head_pose

        head = create_head_pose(
            x=command.pose["x"],
            y=command.pose["y"],
            z=command.pose["z"],
            roll=command.pose["roll"],
            pitch=command.pose["pitch"],
            yaw=command.pose["yaw"],
            degrees=False,
        )
        antennas = np.array(command.antennas, dtype=np.float64)
        self.mini.set_target(head=head, body_yaw=command.body_yaw, antennas=antennas)

    def goto_neutral(self, duration: float = RESET_DURATION_SEC) -> None:
        if self.mini is None or self.log_only:
            time.sleep(min(duration, 0.05))
            return
        from reachy_mini.utils import create_head_pose

        head = create_head_pose(
            x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, degrees=False
        )
        self.mini.goto_target(
            head=head,
            body_yaw=0.0,
            antennas=[0.0, 0.0],
            duration=duration,
            method="minjerk",
        )


class RobotControl:
    def __init__(
        self,
        session: SessionHub,
        gateway: RobotGateway,
        *,
        robot_available: bool,
        log_only: bool = False,
        hz: float = CONTROL_HZ,
    ) -> None:
        self.session = session
        self.gateway = gateway
        self.robot_available = robot_available
        self.log_only = log_only
        self.dt = 1.0 / hz
        self._task: asyncio.Task[None] | None = None
        self._state = initial_state(robot_available=robot_available or log_only, now=time.monotonic())
        self._last_seen_seq: int | None = None
        self._last_reconcile = 0.0
        self._sdk_lock = asyncio.Lock()
        self._started = False

    @property
    def state(self) -> ControlState:
        return self._state

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._loop(), name="robot_control")
            self._started = True

    async def stop(self) -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._task = None
        self._started = False

    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        last = time.monotonic()
        try:
            # Initial seed attempt.
            await self._seed(update_host=True)
            while True:
                t0 = time.monotonic()
                dt = min(t0 - last, 0.05)
                if dt <= 0:
                    dt = self.dt
                last = t0

                await self._tick(loop, t0, dt)

                elapsed = time.monotonic() - t0
                await asyncio.sleep(max(0.0, self.dt - elapsed))
        except asyncio.CancelledError:
            pass

    async def _tick(self, loop: asyncio.AbstractEventLoop, now: float, dt: float) -> None:
        # Handle pending reset first — exclusive robot ownership.
        pending = await self.session.take_pending_reset()
        if pending is not None:
            await self._run_reset(loop, pending, now)
            return

        latest = await self.session.take_latest_sample()
        sample = None
        sample_is_fresh = False
        if latest is not None:
            sample = latest.sample
            if self._last_seen_seq != sample.seq:
                sample_is_fresh = True
                self._last_seen_seq = sample.seq
            self._state = note_sample_receipt(self._state, latest.receipt_time)

        # Idle reconciliation when disengaged and not frozen.
        if (
            self._state.mode == "idle"
            and not self._state.engaged
            and not self._state.sends_frozen
            and (now - self._last_reconcile) >= IDLE_RECONCILE_SEC
        ):
            await self._reconcile_idle(loop, now)

        prev_mode = self._state.mode
        result = step(
            self._state,
            now=now,
            dt=dt,
            sample=sample,
            sample_is_fresh=sample_is_fresh,
        )
        self._state = result.state

        if result.mode_changed or self._state.mode != prev_mode:
            await self.session.push_host_state(
                mode=self._state.mode,
                robot=self.robot_available or self.log_only,
                error=self._state.error,
                clear_error=self._state.error is None,
            )

        if result.command is None:
            return

        if self.log_only:
            if sample_is_fresh:
                logger.info(
                    "command engaged=%s pose=%s body=%.3f",
                    self._state.engaged,
                    {k: round(result.command.pose[k], 4) for k in result.command.pose},
                    result.command.body_yaw,
                )
            self._state = mark_sdk_success(self._state, result.command)
            return

        async with self._sdk_lock:
            try:
                await loop.run_in_executor(None, self.gateway.set_target, result.command)
            except Exception as exc:
                logger.warning("set_target failed: %s", exc)
                self._state = mark_sdk_failure(self._state)
                await self.session.push_host_state(
                    mode=self._state.mode,
                    error=self._state.error,
                )
                return
            self._state = mark_sdk_success(self._state, result.command)

    async def _seed(self, *, update_host: bool) -> bool:
        loop = asyncio.get_running_loop()
        read = await loop.run_in_executor(None, self.gateway.read_pose)
        if read is None:
            self._state = mark_pose_unread(self._state)
            if update_host:
                await self.session.push_host_state(mode="fault", error=self._state.error)
            return False
        pose, body = read
        self._state = seed_from_pose(self._state, pose, body)
        self._last_reconcile = time.monotonic()
        if update_host:
            await self.session.push_host_state(mode="idle", clear_error=True)
        return True

    async def _reconcile_idle(
        self, loop: asyncio.AbstractEventLoop, now: float
    ) -> None:
        self._last_reconcile = now
        if self.gateway.mini is None:
            return
        read = await loop.run_in_executor(None, self.gateway.read_pose)
        if read is None:
            self._state = mark_pose_unread(self._state)
            await self.session.push_host_state(mode="fault", error=self._state.error)
            return
        pose, body = read
        prev = self._state.baseline_pose
        pos = (
            (pose["x"] - prev["x"]) ** 2
            + (pose["y"] - prev["y"]) ** 2
            + (pose["z"] - prev["z"]) ** 2
        ) ** 0.5
        ang = max(
            abs(pose["roll"] - prev["roll"]),
            abs(pose["pitch"] - prev["pitch"]),
            abs(pose["yaw"] - prev["yaw"]),
            abs(body - self._state.baseline_body_yaw),
        )
        if pos <= RESYNC_EPS_POS_M and ang <= RESYNC_EPS_ANG_RAD:
            self._state = mark_sdk_success(
                self._state,
                Command(pose=pose, body_yaw=body, antennas=tuple(self._state.baseline_antennas[:2])),
            )
            return
        logger.warning("Pose desync pos=%.4f m ang=%.3f rad — idle resync", pos, ang)
        self._state = seed_from_pose(self._state, pose, body)

    async def _run_reset(
        self, loop: asyncio.AbstractEventLoop, reset, now: float
    ) -> None:
        if not self.robot_available and not self.log_only:
            await self.session.complete_reset(
                boot_id=reset.boot_id,
                op_id=reset.op_id,
                status="failed",
                message="Robot not available",
            )
            return

        self._state = begin_reset(self._state, now)
        await self.session.push_host_state(mode="resetting", clear_error=True)

        # Seed from measured pose before goto.
        read = await loop.run_in_executor(None, self.gateway.read_pose)
        if read is None and not self.log_only:
            self._state = mark_pose_unread(self._state)
            await self.session.complete_reset(
                boot_id=reset.boot_id,
                op_id=reset.op_id,
                status="failed",
                message="Robot pose unread",
            )
            return
        if read is not None:
            pose, body = read
            self._state = seed_from_pose(self._state, pose, body)
            self._state = begin_reset(self._state, now)

        async with self._sdk_lock:
            try:
                await loop.run_in_executor(None, self.gateway.goto_neutral, RESET_DURATION_SEC)
            except Exception as exc:
                logger.error("goto_target failed: %s", exc)
                self._state = mark_pose_unread(self._state)
                await self.session.complete_reset(
                    boot_id=reset.boot_id,
                    op_id=reset.op_id,
                    status="failed",
                    message=str(exc),
                )
                return

        measured = await loop.run_in_executor(None, self.gateway.read_pose)
        if measured is None and not self.log_only:
            self._state = rebase_neutral(self._state, measured_baseline=None, measured_body=None)
            await self.session.complete_reset(
                boot_id=reset.boot_id,
                op_id=reset.op_id,
                status="failed",
                message="Robot pose unread after reset",
            )
            return

        if measured is None:
            measured = (zero_pose(), 0.0)
        pose, body = measured
        self._state = rebase_neutral(
            self._state, measured_baseline=pose, measured_body=body
        )
        self._last_reconcile = time.monotonic()
        await self.session.complete_reset(
            boot_id=reset.boot_id,
            op_id=reset.op_id,
            status="completed",
        )

    async def on_controller_connected(self) -> None:
        """Reseed when a new controller session is admitted.

        Do not push host_state here — the hello handler sends the first snapshot
        so the device always sees hello before host_state.
        """
        self._state = force_disengage(self._state)
        self._last_seen_seq = None
        ok = await self._seed(update_host=False)
        if not ok and self.log_only:
            self._state = seed_from_pose(self._state, zero_pose(), 0.0)

    async def on_controller_disconnected(self) -> None:
        self._state = force_disengage(self._state)
        if self._state.mode not in {"resetting"}:
            await self.session.push_host_state(mode="idle")

"""
Single fixed-rate robot command owner for protocol v4.

Consumes the latest sample from SessionHub, runs the pure reducer, and performs
at most one SDK call in flight. Every outbound pose is capped per tick.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import replace
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from esp32_motion_controller.control import (
    ANIM_VEL_MULT,
    APPEAR_ANTENNA_SPEED_MULT,
    CONTROL_HZ,
    MAX_ANGULAR_VEL,
    MAX_POS_VEL,
    STALE_PACKET_SEC,
    Command,
    ControlState,
    advance_antennas,
    begin_reset,
    disengaged_rest_pose,
    force_disengage,
    initial_state,
    mark_pose_unread,
    mark_sdk_failure,
    mark_sdk_success,
    note_sample_receipt,
    pose_near,
    pose_travel,
    seed_from_pose,
    speed_lock,
    step,
    zero_pose,
)
from esp32_motion_controller.session import SessionHub

logger = logging.getLogger(__name__)

IDLE_RECONCILE_SEC = 2.0
RESYNC_EPS_POS_M = 0.003
RESYNC_EPS_ANG_RAD = 0.05
# Incoming jumps larger than this are dropped (hold last sent). Smaller
# deltas are speed-locked to one tick of MAX_*_VEL.
CULL_POS_M = 0.020
CULL_ANG_RAD = math.radians(20.0)


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
        self._sent_pose: dict[str, float] | None = None
        self._sent_body: float = 0.0
        self._sent_at: float = 0.0
        self._posture: str = "unknown"
        self._boot_id: str | None = None
        self._button_engaged: bool = False
        self._suppress_button_edge: bool = False
        self._anim: str | None = None
        self._anim_pose = zero_pose()
        self._anim_body = 0.0
        self._pending_reset = None

    def _remember_sent(self, pose: dict[str, float], body: float, now: float) -> None:
        self._sent_pose = dict(pose)
        self._sent_body = float(body)
        self._sent_at = now

    def _guard_command(self, command: Command, now: float) -> Command:
        """Cap (or drop) an incoming pose before it reaches the robot.

        Appear/disappear slews are speed-locked (faster than streaming).
        Streaming jumps larger than CULL_* are discarded so a stall cannot
        accumulate into a snap.
        """
        del now
        ref_pose = (
            dict(self._sent_pose)
            if self._sent_pose is not None
            else dict(self._state.baseline_pose)
        )
        ref_body = (
            float(self._sent_body)
            if self._sent_pose is not None
            else float(self._state.baseline_body_yaw)
        )
        if self._anim is None:
            dist, ang = pose_travel(
                ref_pose, command.pose, ref_body, command.body_yaw
            )
            if dist > CULL_POS_M or ang > CULL_ANG_RAD:
                logger.warning(
                    "cull jump pos=%.3f m ang=%.1f deg (hold last)",
                    dist,
                    math.degrees(ang),
                )
                return Command(
                    pose=ref_pose,
                    body_yaw=ref_body,
                    antennas=command.antennas,
                )
        anim_slew = self._anim in {"appear", "disappear"}
        pose, body = speed_lock(
            ref_pose,
            ref_body,
            command.pose,
            command.body_yaw,
            self.dt,
            apply_ellipsoid=self._state.engaged and self._anim is None,
            max_ang_vel=MAX_ANGULAR_VEL * ANIM_VEL_MULT if anim_slew else MAX_ANGULAR_VEL,
            max_pos_vel=MAX_POS_VEL * ANIM_VEL_MULT if anim_slew else MAX_POS_VEL,
        )
        return Command(pose=pose, body_yaw=body, antennas=command.antennas)

    def _begin_anim(self, kind: str, pose: dict[str, float], body: float) -> None:
        self._anim = kind
        self._anim_pose = dict(pose)
        self._anim_body = float(body)
        if kind in {"disappear", "reset"}:
            self._state = force_disengage(self._state)
        elif kind == "appear":
            now = time.monotonic()
            self._state = replace(
                self._state,
                behavior_t0=now,
                antenna_left=0.0,
                antenna_right=0.0,
                smooth_antennas=[0.0, 0.0],
            )

    async def _tick_anim(
        self, loop: asyncio.AbstractEventLoop, now: float, dt: float
    ) -> None:
        antennas = (0.0, 0.0)
        if self._anim == "appear":
            self._state = advance_antennas(
                self._state, now, dt, speed_mult=APPEAR_ANTENNA_SPEED_MULT
            )
            antennas = (self._state.antenna_left, self._state.antenna_right)

        command = Command(
            pose=dict(self._anim_pose),
            body_yaw=self._anim_body,
            antennas=antennas,
        )
        sent = await self._send_command(loop, now, command, log_sample=False)
        if sent is None:
            return
        if not pose_near(sent.pose, self._anim_pose, sent.body_yaw, self._anim_body):
            return
        kind = self._anim
        self._anim = None
        if kind == "appear":
            self._posture = "neutral"
            self._state = replace(
                self._state,
                desired_pose=zero_pose(),
                base_pose=zero_pose(),
                body_yaw=0.0,
                smooth_antennas=[
                    self._state.antenna_left,
                    self._state.antenna_right,
                ],
            )
        elif kind == "disappear":
            self._posture = "ducked"
            rest = disengaged_rest_pose()
            self._state = replace(
                self._state,
                desired_pose=dict(rest),
                base_pose=dict(rest),
                body_yaw=0.0,
                engaged=False,
            )
        elif kind == "reset":
            self._posture = "neutral"
            self._state = replace(self._state, mode="idle")
            reset = self._pending_reset
            self._pending_reset = None
            if reset is not None:
                await self.session.complete_reset(
                    boot_id=reset.boot_id,
                    op_id=reset.op_id,
                    status="completed",
                )
            self._begin_anim("disappear", disengaged_rest_pose(), 0.0)
            await self.session.push_host_state(mode="idle", clear_error=True)

    async def _send_command(
        self,
        loop: asyncio.AbstractEventLoop,
        now: float,
        command: Command,
        *,
        log_sample: bool,
    ) -> Command | None:
        command = self._guard_command(command, now)
        if self.log_only:
            if log_sample:
                logger.info(
                    "command engaged=%s pose=%s body=%.3f",
                    self._state.engaged,
                    {k: round(command.pose[k], 4) for k in command.pose},
                    command.body_yaw,
                )
            self._state = mark_sdk_success(self._state, command)
            self._remember_sent(command.pose, command.body_yaw, now)
            return command

        async with self._sdk_lock:
            t_sdk = time.monotonic()
            try:
                await loop.run_in_executor(None, self.gateway.set_target, command)
            except Exception as exc:
                logger.warning("set_target failed: %s", exc)
                self._state = mark_sdk_failure(self._state)
                await self.session.push_host_state(
                    mode=self._state.mode,
                    error=self._state.error,
                )
                return None
            sdk_ms = (time.monotonic() - t_sdk) * 1000.0
            self.session.note_sdk_duration(sdk_ms)
            if sdk_ms > 50.0:
                logger.warning("set_target slow %.0f ms", sdk_ms)
            self._state = mark_sdk_success(self._state, command)
            self._remember_sent(command.pose, command.body_yaw, now)
            return command

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
            last = time.monotonic()
            while True:
                t0 = time.monotonic()
                dt = min(t0 - last, 0.05)
                if dt <= 0:
                    dt = self.dt
                lag_ms = (t0 - last) * 1000.0
                if last > 0.0 and lag_ms > 80.0:
                    logger.warning("control tick lag %.0f ms", lag_ms)
                    self.session.note_tick_lag(lag_ms)
                last = t0

                await self._tick(loop, t0, dt)

                elapsed = time.monotonic() - t0
                await asyncio.sleep(max(0.0, self.dt - elapsed))
        except asyncio.CancelledError:
            pass

    def _want_engaged(self, sample) -> bool:
        if sample is None:
            return False
        if self._state.mode == "fault" or self._state.sends_frozen:
            return False
        return bool(sample.engaged) and bool(sample.ready)

    async def _tick(self, loop: asyncio.AbstractEventLoop, now: float, dt: float) -> None:
        pending = await self.session.take_pending_reset()
        if pending is not None:
            self._pending_reset = pending
            self._state = begin_reset(self._state, now)
            self._begin_anim("reset", zero_pose(), 0.0)
            await self.session.push_host_state(mode="resetting", clear_error=True)

        edge = self.session.poll_presence_edge()
        if edge == "absent":
            await self.on_controller_absent()

        latest = await self.session.take_latest_sample()
        sample = None
        sample_is_fresh = False
        if latest is not None:
            sample = latest.sample
            if self._last_seen_seq != sample.seq:
                sample_is_fresh = True
                self._last_seen_seq = sample.seq
            self._state = note_sample_receipt(self._state, latest.receipt_time)

        currently = self._state.engaged
        stale = (
            currently
            and not self.session.controller_present
            and self._state.have_sample
            and self._state.last_sample_time > 0.0
            and (now - self._state.last_sample_time) > STALE_PACKET_SEC
        )
        if stale:
            logger.warning(
                "stale disengage controller_absent age=%.0f ms (limit=%.0f ms)",
                (now - self._state.last_sample_time) * 1000.0,
                STALE_PACKET_SEC * 1000.0,
            )
            self._state = force_disengage(self._state)

        if sample_is_fresh:
            want = self._want_engaged(sample)
            if self._state.mode == "resetting" or self._anim == "reset":
                self._button_engaged = False
            elif self._suppress_button_edge:
                self._button_engaged = want
                self._suppress_button_edge = False
            elif want and not self._button_engaged:
                self._button_engaged = True
                if self._posture == "ducked":
                    self._begin_anim("appear", zero_pose(), 0.0)
            elif not want and self._button_engaged:
                self._button_engaged = False
                self._begin_anim("disappear", disengaged_rest_pose(), 0.0)

        if self._anim is not None:
            await self._tick_anim(loop, now, dt)
            return

        # Idle reconciliation when disengaged and not frozen.
        if (
            self._state.mode == "idle"
            and not self._state.engaged
            and not self._button_engaged
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
            controller_present=self.session.controller_present,
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
        await self._send_command(loop, now, result.command, log_sample=sample_is_fresh)

    async def _seed(self, *, update_host: bool) -> bool:
        loop = asyncio.get_running_loop()
        read = await loop.run_in_executor(None, self.gateway.read_pose)
        if read is None:
            self._state = mark_pose_unread(self._state)
            if update_host:
                await self.session.push_host_state(mode="fault", error=self._state.error)
            return False
        pose, body = read
        self._state = seed_from_pose(self._state, pose, body, apply_ellipsoid=False)
        self._remember_sent(self._state.baseline_pose, self._state.baseline_body_yaw, time.monotonic())
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
            self._remember_sent(pose, body, now)
            return
        logger.warning("Pose desync pos=%.4f m ang=%.3f rad — idle resync", pos, ang)
        self._state = seed_from_pose(
            self._state, pose, body, apply_ellipsoid=False
        )
        self._remember_sent(pose, body, now)

    async def on_controller_connected(self) -> None:
        """Reseed clutch from the measured pose. Tests and first-hello use this."""
        await self._reseed_controller()

    async def on_controller_hello(self, hello) -> None:
        """Keep clutch on first hello and any same-boot reconnect.

        Appear/disappear are button edges only — a socket gap must not duck
        or rise. Reseed only when the device actually rebooted.
        """
        prev = self._boot_id
        first = prev is None
        same = prev is not None and hello.boot_id == prev
        self._boot_id = hello.boot_id
        keep = first or (
            same
            and not self._state.sends_frozen
            and self._state.mode not in {"fault"}
        )
        if keep:
            logger.info(
                "hello keep clutch boot_id=%s first=%s same_boot=%s engaged=%s button=%s posture=%s",
                hello.boot_id,
                first,
                same,
                self._state.engaged,
                self._button_engaged,
                self._posture,
            )
            return
        logger.info(
            "hello reseed boot_id=%s prev=%s mode=%s",
            hello.boot_id,
            prev,
            self._state.mode,
        )
        await self._reseed_controller()

    async def _reseed_controller(self) -> None:
        """Do not push host_state here — hello already sent the snapshot."""
        self._state = force_disengage(self._state)
        self._last_seen_seq = None
        self._posture = "unknown"
        self._button_engaged = False
        self._suppress_button_edge = False
        ok = await self._seed(update_host=False)
        if not ok and self.log_only:
            self._state = seed_from_pose(self._state, zero_pose(), 0.0)
            self._remember_sent(zero_pose(), 0.0, time.monotonic())

    async def on_controller_absent(self) -> None:
        logger.info(
            "controller absent engaged=%s mode=%s",
            self._state.engaged,
            self._state.mode,
        )

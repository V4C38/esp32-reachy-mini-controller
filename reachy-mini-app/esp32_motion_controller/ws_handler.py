"""
WebSocket router for the ESP32 motion controller.

Handles controller_state / reset / status, the 300 ms stale-packet watchdog,
reset interlock (busy ignores engage; rebases clutch on goto completion),
and single-controller admission.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any

from fastapi import WebSocket
from starlette.websockets import WebSocketState

from esp32_motion_controller.behavior import Behavior
from esp32_motion_controller.controller_state import ControllerState
from esp32_motion_controller.movement_handler import MovementHandler

logger = logging.getLogger(__name__)

STALE_PACKET_SEC = 0.300
BEHAVIOR_TICK_SEC = 0.033
RESET_DURATION_SEC = 1.5
# Keep the apply loop across short reconnect blips; stop after this idle.
IDLE_STOP_SEC = 5.0


class WebSocketHandler:
    def __init__(
        self,
        movement: MovementHandler,
        controller: ControllerState,
        behavior: Behavior,
        *,
        robot_available: bool = True,
        log_only: bool = False,
    ) -> None:
        self.movement = movement
        self.controller = controller
        self.behavior = behavior
        self.robot_available = robot_available
        self.log_only = log_only
        self.busy = False
        self._last_packet_time: float = 0.0
        self._watchdog_task: asyncio.Task[None] | None = None
        self._behavior_task: asyncio.Task[None] | None = None
        self._idle_stop_task: asyncio.Task[None] | None = None
        self._active_ws: WebSocket | None = None

    async def on_connect(self, websocket: WebSocket) -> bool:
        """Admit a single controller. Stale sockets are replaced, not rejected.

        The ESP client auto-reconnects after WiFi blips. If we still hold the
        previous WebSocket object, a hard reject (1008) makes the board flap
        forever and the face stays closed.

        Movement keeps running across reconnects — stop/start would re-zero
        velocity-clamp bookkeeping and let the next set_target snap the head.
        """
        if self._idle_stop_task is not None and not self._idle_stop_task.done():
            self._idle_stop_task.cancel()
            self._idle_stop_task = None

        if self._active_ws is not None:
            old = self._active_ws
            alive = old.client_state == WebSocketState.CONNECTED
            if alive:
                logger.warning("Replacing active controller connection")
            self._detach_controller(stop_movement=False)
            if alive:
                try:
                    await old.close(code=1000)
                except Exception:
                    pass

        await websocket.accept()
        self._active_ws = websocket
        self.movement.start()
        if self.movement.resync_from_robot():
            # Clutch base matches the physical head so idle/engage cannot yank home.
            self.controller.set_base_pose(self.movement.current_pose)
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())
        self._behavior_task = asyncio.create_task(self._behavior_loop())
        logger.info("Controller connected")
        return True

    def _detach_controller(self, *, stop_movement: bool) -> None:
        self.controller.force_disengage()
        if stop_movement:
            self.movement.stop()
        for task in (self._watchdog_task, self._behavior_task):
            if task is not None and not task.done():
                task.cancel()
        self._watchdog_task = None
        self._behavior_task = None
        self._active_ws = None

    def cleanup(self, websocket: WebSocket | None = None) -> None:
        """Drop the active controller. Ignore stale sockets that already lost the slot."""
        if websocket is not None and self._active_ws is not websocket:
            return
        if self._active_ws is None and websocket is None:
            return
        # Keep the apply loop alive on a blip so velocity clamping stays
        # continuous; schedule a stop if nothing reconnects.
        self._detach_controller(stop_movement=False)
        if self._idle_stop_task is None or self._idle_stop_task.done():
            self._idle_stop_task = asyncio.create_task(self._idle_stop_after_grace())
        logger.info("Controller disconnected")

    async def _idle_stop_after_grace(self) -> None:
        try:
            await asyncio.sleep(IDLE_STOP_SEC)
            if self._active_ws is None:
                logger.info("No controller for %.1fs — stopping movement loop", IDLE_STOP_SEC)
                self.movement.stop()
        except asyncio.CancelledError:
            pass

    def shutdown(self) -> None:
        """App teardown: always stop movement."""
        if self._idle_stop_task is not None and not self._idle_stop_task.done():
            self._idle_stop_task.cancel()
            self._idle_stop_task = None
        self._detach_controller(stop_movement=True)

    async def handle_message(self, websocket: WebSocket, raw: str) -> None:
        try:
            msg: dict[str, Any] = json.loads(raw)
        except json.JSONDecodeError as exc:
            await self._send_error(websocket, "parse", f"Invalid JSON: {exc}")
            return

        msg_type = msg.get("type")
        request_id = msg.get("_id")
        handler = {
            "controller_state": self._handle_controller_state,
            "reset": self._handle_reset,
            "status": self._handle_status,
        }.get(msg_type)

        if handler is None:
            await self._send_error(
                websocket,
                msg_type or "unknown",
                f"Unknown message type: {msg_type}",
                request_id,
            )
            return

        try:
            response = await handler(msg)
        except Exception as exc:
            logger.error("Handler %s failed: %s", msg_type, exc)
            await self._send_error(websocket, msg_type, str(exc), request_id)
            return

        if request_id is None or response is None:
            return
        response["_id"] = request_id
        await websocket.send_json(response)

    async def _handle_controller_state(self, msg: dict[str, Any]) -> dict[str, Any] | None:
        self._last_packet_time = time.monotonic()
        q = msg.get("q", [1, 0, 0, 0])
        p = msg.get("p", [0, 0, 0])
        engaged = bool(msg.get("engaged", False))
        gain = float(msg.get("gain", 1.0))
        ready = bool(msg.get("ready", False))

        desired = self.controller.update(
            q=q,
            p=p,
            engaged=engaged,
            gain=gain,
            ready=ready,
            allow_engage=not self.busy,
        )

        # Reset goto owns the robot target exclusively — streaming set_target
        # here fights the minjerk path and looks like a whip when rebase snaps.
        if self.busy:
            return None

        # Behavior owns body_yaw + antennas from head yaw
        body_yaw, antennas = self.behavior.update(desired["yaw"])
        self.movement.set_target(desired, body_yaw=body_yaw, antennas=antennas)

        if self.log_only:
            logger.info(
                "controller engaged=%s gain=%.2f ready=%s pose=%s body_yaw=%.3f",
                self.controller.engaged,
                self.controller.gain,
                ready,
                {k: round(desired[k], 4) for k in desired},
                body_yaw,
            )
        return None

    async def _handle_reset(self, _msg: dict[str, Any]) -> dict[str, Any]:
        if self.busy:
            return {"type": "reset_result", "success": False, "message": "Reset already in progress"}
        if not self.robot_available and not self.log_only:
            return {"type": "reset_result", "success": False, "message": "Robot not available"}

        self.busy = True
        self.controller.force_disengage()
        # Zero clutch immediately so idle/behavior cannot re-target the old pose
        # while the goto is running.
        self.controller.rebase_neutral()
        self.behavior.reset()

        # Seed from the measured pose so goto starts where the head actually is
        # (and clears a prior send-freeze). Fail closed when the robot is real.
        if not await self.movement.resync_from_robot_async(update_target=True):
            if not self.log_only:
                self.busy = False
                return {
                    "type": "reset_result",
                    "success": False,
                    "message": "Robot pose unread",
                }

        neutral = {k: 0.0 for k in ("x", "y", "z", "roll", "pitch", "yaw")}
        move_uuid = self.movement.goto(
            neutral, body_yaw=0.0, antennas=[0.0, 0.0],
            duration=RESET_DURATION_SEC, interpolation="minjerk",
        )
        asyncio.create_task(self._finish_reset(move_uuid))
        return {"type": "reset_result", "success": True, "uuid": move_uuid}

    async def _finish_reset(self, move_uuid: str) -> None:
        try:
            await asyncio.sleep(RESET_DURATION_SEC + 0.05)
        finally:
            self.movement.rebase_to_neutral()
            self.controller.rebase_neutral()
            self.behavior.reset()
            self.busy = False
            logger.info("Reset complete; clutch rebased to neutral (uuid=%s)", move_uuid)

    async def _handle_status(self, _msg: dict[str, Any]) -> dict[str, Any]:
        return {
            "type": "status_result",
            "connected": True,
            "robot": self.robot_available,
            "busy": self.busy,
        }

    async def _watchdog_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(0.05)
                if self._last_packet_time <= 0:
                    continue
                if time.monotonic() - self._last_packet_time > STALE_PACKET_SEC:
                    if self.controller.engaged:
                        logger.warning("Stale controller_state; forcing disengage")
                        self.controller.force_disengage()
                        # freeze: keep last desired as movement target, no further updates
        except asyncio.CancelledError:
            pass

    async def _behavior_loop(self) -> None:
        """Keep antennas / body-follow alive while idle (not engaged)."""
        try:
            while True:
                await asyncio.sleep(BEHAVIOR_TICK_SEC)
                if self.controller.engaged or self.busy:
                    continue
                pose = self.controller.desired_pose
                body_yaw, antennas = self.behavior.update(pose["yaw"])
                self.movement.set_target(pose, body_yaw=body_yaw, antennas=antennas)
        except asyncio.CancelledError:
            pass

    async def _send_error(
        self,
        websocket: WebSocket,
        request_type: str,
        message: str,
        request_id: Any = None,
    ) -> None:
        response: dict[str, Any] = {
            "type": "error",
            "request_type": request_type,
            "message": message,
        }
        if request_id is not None:
            response["_id"] = request_id
        await websocket.send_json(response)

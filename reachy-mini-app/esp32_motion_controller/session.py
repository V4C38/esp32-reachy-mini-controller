"""
WebSocket session ingress for protocol v2.

Validates frames, keeps only the latest sample, manages session generation and
reset operation mailbox. Never calls the robot SDK.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable

from fastapi import WebSocket
from starlette.websockets import WebSocketState

from esp32_motion_controller.protocol import (
    Hello,
    ProtocolError,
    Reset,
    Sample,
    encode_error,
    encode_hello_response,
    encode_host_state,
    encode_reset_result,
    parse_frame,
)

logger = logging.getLogger(__name__)

SendFn = Callable[[dict[str, Any]], Awaitable[None]]


@dataclass
class LatestSample:
    sample: Sample
    receipt_time: float
    generation: int


@dataclass
class ResetRecord:
    boot_id: str
    op_id: int
    status: str  # accepted | completed | failed
    message: str | None = None


@dataclass
class SessionMailbox:
    """Shared state between the WS receiver and the control loop."""

    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    generation: int = 0
    boot_id: str | None = None
    active_ws: WebSocket | None = None
    latest: LatestSample | None = None
    last_seq: int | None = None
    pending_reset: Reset | None = None
    reset_cache: dict[tuple[str, int], ResetRecord] = field(default_factory=dict)
    host_mode: str = "idle"
    host_robot: bool = True
    host_error: str | None = None
    controller_present: bool = False


class SessionHub:
    def __init__(self, *, robot_available: bool) -> None:
        self.mailbox = SessionMailbox(host_robot=robot_available)
        self._send_lock = asyncio.Lock()

    @property
    def generation(self) -> int:
        return self.mailbox.generation

    async def on_connect(self, websocket: WebSocket) -> int:
        mb = self.mailbox
        async with mb.lock:
            old = mb.active_ws
            mb.generation += 1
            gen = mb.generation
            mb.active_ws = websocket
            mb.latest = None
            mb.last_seq = None
            mb.pending_reset = None
            mb.boot_id = None
            mb.controller_present = True
            mb.host_mode = "idle"
            mb.host_error = None

        if old is not None and old is not websocket:
            alive = old.client_state == WebSocketState.CONNECTED
            if alive:
                logger.warning("Replacing active controller connection")
                try:
                    await old.close(code=1000)
                except Exception:
                    pass

        await websocket.accept()
        logger.info("Controller socket accepted generation=%d", gen)
        return gen

    async def cleanup(self, websocket: WebSocket, generation: int) -> None:
        mb = self.mailbox
        async with mb.lock:
            if mb.active_ws is not websocket or mb.generation != generation:
                return
            mb.active_ws = None
            mb.controller_present = False
            mb.latest = None
            # Keep reset_cache and boot_id so reconnect can retrieve outcomes.
        logger.info("Controller disconnected generation=%d", generation)

    async def handle_message(self, websocket: WebSocket, generation: int, raw: str) -> None:
        try:
            msg = parse_frame(raw)
        except ProtocolError as exc:
            await self._send(websocket, generation, encode_error(exc.request_type, str(exc)))
            if exc.request_type == "hello":
                try:
                    await websocket.close(code=1002)
                except Exception:
                    pass
            return

        if isinstance(msg, Hello):
            await self._handle_hello(websocket, generation, msg)
            return
        if isinstance(msg, Sample):
            await self._handle_sample(websocket, generation, msg)
            return
        if isinstance(msg, Reset):
            await self._handle_reset(websocket, generation, msg)
            return

    async def _handle_hello(self, websocket: WebSocket, generation: int, hello: Hello) -> None:
        mb = self.mailbox
        async with mb.lock:
            if mb.generation != generation or mb.active_ws is not websocket:
                return
            mb.boot_id = hello.boot_id
            robot = mb.host_robot
            mode = mb.host_mode
            err = mb.host_error
        await self._send(websocket, generation, encode_hello_response(generation))
        await self._send(
            websocket,
            generation,
            encode_host_state(robot=robot, mode=mode, error=err),
        )
        logger.info("Hello ok boot_id=%s generation=%d", hello.boot_id, generation)

    async def _handle_sample(self, websocket: WebSocket, generation: int, sample: Sample) -> None:
        mb = self.mailbox
        now = time.monotonic()
        async with mb.lock:
            if mb.generation != generation or mb.active_ws is not websocket:
                return
            if mb.boot_id is None:
                # Implicit bind if device skipped hello (should not happen).
                mb.boot_id = sample.boot_id
            if sample.boot_id != mb.boot_id:
                logger.warning("sample boot_id mismatch; dropping")
                return
            if mb.last_seq is not None:
                # Duplicate
                if sample.seq == mb.last_seq:
                    return
                # Out-of-order (small backward jump) — drop unless wrap.
                if sample.seq < mb.last_seq:
                    # uint32 wrap: large backward jump
                    if mb.last_seq - sample.seq < 2**31:
                        return
            mb.last_seq = sample.seq
            mb.latest = LatestSample(sample=sample, receipt_time=now, generation=generation)

    async def _handle_reset(self, websocket: WebSocket, generation: int, reset: Reset) -> None:
        mb = self.mailbox
        async with mb.lock:
            if mb.generation != generation or mb.active_ws is not websocket:
                return
            if mb.boot_id is None:
                mb.boot_id = reset.boot_id
            if reset.boot_id != mb.boot_id:
                await self._send(
                    websocket,
                    generation,
                    encode_reset_result(
                        boot_id=reset.boot_id,
                        op_id=reset.op_id,
                        status="failed",
                        message="boot_id mismatch",
                    ),
                )
                return
            cached = mb.reset_cache.get((reset.boot_id, reset.op_id))
            if cached is not None:
                result = encode_reset_result(
                    boot_id=cached.boot_id,
                    op_id=cached.op_id,
                    status=cached.status,
                    message=cached.message,
                )
                pending = None
            elif mb.host_mode == "resetting":
                # Another reset already running with a different op_id.
                result = encode_reset_result(
                    boot_id=reset.boot_id,
                    op_id=reset.op_id,
                    status="failed",
                    message="Reset already in progress",
                )
                pending = None
            else:
                mb.pending_reset = reset
                mb.reset_cache[(reset.boot_id, reset.op_id)] = ResetRecord(
                    boot_id=reset.boot_id,
                    op_id=reset.op_id,
                    status="accepted",
                )
                mb.host_mode = "resetting"
                result = encode_reset_result(
                    boot_id=reset.boot_id,
                    op_id=reset.op_id,
                    status="accepted",
                )
                pending = reset

        await self._send(websocket, generation, result)
        if pending is not None:
            await self.push_host_state(mode="resetting")

    async def take_latest_sample(self) -> LatestSample | None:
        mb = self.mailbox
        async with mb.lock:
            return mb.latest

    async def take_pending_reset(self) -> Reset | None:
        mb = self.mailbox
        async with mb.lock:
            reset = mb.pending_reset
            mb.pending_reset = None
            return reset

    async def complete_reset(
        self,
        *,
        boot_id: str,
        op_id: int,
        status: str,
        message: str | None = None,
    ) -> None:
        mb = self.mailbox
        async with mb.lock:
            mb.reset_cache[(boot_id, op_id)] = ResetRecord(
                boot_id=boot_id, op_id=op_id, status=status, message=message
            )
            if status in {"completed", "failed"} and mb.host_mode == "resetting":
                mb.host_mode = "idle" if status == "completed" else "fault"
                mb.host_error = message if status == "failed" else None
            ws = mb.active_ws
            gen = mb.generation
            mode = mb.host_mode
            robot = mb.host_robot
            err = mb.host_error
        if ws is not None:
            await self._send(
                ws,
                gen,
                encode_reset_result(
                    boot_id=boot_id, op_id=op_id, status=status, message=message
                ),
            )
            await self._send(
                ws,
                gen,
                encode_host_state(robot=robot, mode=mode, error=err),
            )

    async def push_host_state(
        self,
        *,
        mode: str | None = None,
        robot: bool | None = None,
        error: str | None = None,
        clear_error: bool = False,
    ) -> None:
        mb = self.mailbox
        async with mb.lock:
            if mode is not None:
                mb.host_mode = mode
            if robot is not None:
                mb.host_robot = robot
            if clear_error:
                mb.host_error = None
            elif error is not None:
                mb.host_error = error
            ws = mb.active_ws
            gen = mb.generation
            payload = encode_host_state(
                robot=mb.host_robot, mode=mb.host_mode, error=mb.host_error
            )
        if ws is not None:
            await self._send(ws, gen, payload)

    async def snapshot_status(self) -> dict[str, Any]:
        mb = self.mailbox
        async with mb.lock:
            return {
                "robot": mb.host_robot,
                "busy": mb.host_mode == "resetting",
                "mode": mb.host_mode,
                "connected": mb.controller_present,
                "error": mb.host_error,
            }

    async def _send(
        self, websocket: WebSocket, generation: int, payload: dict[str, Any]
    ) -> None:
        async with self._send_lock:
            mb = self.mailbox
            async with mb.lock:
                if mb.generation != generation or mb.active_ws is not websocket:
                    return
            if websocket.client_state != WebSocketState.CONNECTED:
                return
            try:
                await websocket.send_json(payload)
            except Exception as exc:
                logger.warning("WS send failed: %s", exc)

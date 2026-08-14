"""
UDP session ingress for protocol v4.

Validates datagrams, keeps only the latest sample, tracks the peer by
address + last_rx. Never calls the robot SDK.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable

from esp32_motion_controller.protocol import (
    LINK_STALE_SEC,
    PROTOCOL_VERSION,
    Hello,
    LinkDiag,
    ProtocolError,
    Reset,
    Sample,
    encode_state_reply,
    parse_frame,
)

logger = logging.getLogger(__name__)

OnBootFn = Callable[[Hello], Awaitable[None]]


@dataclass
class LatestSample:
    sample: Sample
    receipt_time: float


@dataclass
class ResetRecord:
    boot_id: str
    op_id: int
    status: str  # accepted | completed | failed
    message: str | None = None


@dataclass
class SessionMailbox:
    """Shared state between the UDP receiver and the control loop."""

    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    boot_id: str | None = None
    peer: tuple[str, int] | None = None
    last_rx: float = 0.0
    latest: LatestSample | None = None
    last_seq: int | None = None
    pending_reset: Reset | None = None
    reset_cache: dict[tuple[str, int], ResetRecord] = field(default_factory=dict)
    host_mode: str = "idle"
    host_robot: bool = True
    host_error: str | None = None
    last_diag: LinkDiag | None = None
    seq_skips: int = 0
    sample_gaps: int = 0
    presents: int = 0
    absents: int = 0


class SessionHub:
    def __init__(self, *, robot_available: bool) -> None:
        self.mailbox = SessionMailbox(host_robot=robot_available)
        self._transport: Any = None
        self._last_sample_receipt: float = 0.0
        self._present: bool = False
        self.max_tick_lag_ms: float = 0.0
        self.last_sdk_ms: float = 0.0
        self.on_hello: OnBootFn | None = None

    def bind_transport(self, transport: Any) -> None:
        self._transport = transport

    @property
    def controller_present(self) -> bool:
        last = self.mailbox.last_rx
        if last <= 0.0:
            return False
        return (time.monotonic() - last) < LINK_STALE_SEC

    def note_tick_lag(self, lag_ms: float) -> None:
        if lag_ms > self.max_tick_lag_ms:
            self.max_tick_lag_ms = lag_ms

    def note_sdk_duration(self, duration_ms: float) -> None:
        self.last_sdk_ms = duration_ms

    def poll_presence_edge(self) -> str | None:
        """Return 'present' / 'absent' on a liveness edge, else None."""
        now = self.controller_present
        prev = self._present
        if now == prev:
            return None
        self._present = now
        if now:
            self.mailbox.presents += 1
            return "present"
        self.mailbox.absents += 1
        return "absent"

    async def handle_datagram(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            msg = parse_frame(data)
        except ProtocolError as exc:
            logger.warning("drop datagram from %s: %s", addr, exc)
            return

        if isinstance(msg, Hello):
            await self._handle_hello(msg, addr)
            return
        if isinstance(msg, Sample):
            await self._handle_sample(msg, addr)
            return

    def _touch_peer(
        self, boot_id: str, addr: tuple[str, int], *, hello: Hello | None
    ) -> bool:
        """Update peer/boot under lock. Returns True when boot_id changed."""
        mb = self.mailbox
        prev_boot = mb.boot_id
        mb.peer = addr
        mb.last_rx = time.monotonic()
        if prev_boot is not None and boot_id != prev_boot:
            mb.last_seq = None
            mb.latest = None
        mb.boot_id = boot_id
        if hello is not None and hello.diag is not None:
            mb.last_diag = hello.diag
        return prev_boot != boot_id

    async def _maybe_reseed(self, boot_id: str, hello: Hello | None) -> None:
        if self.on_hello is None:
            return
        payload = hello or Hello(protocol_version=PROTOCOL_VERSION, boot_id=boot_id, device="esp32")
        await self.on_hello(payload)

    def _reset_fields(self, boot_id: str, op: int | None) -> tuple[int | None, str | None]:
        mb = self.mailbox
        if op is None:
            return None, None
        cached = mb.reset_cache.get((boot_id, op))
        if cached is not None:
            return cached.op_id, cached.status
        if mb.host_mode == "resetting":
            pending = mb.pending_reset
            if pending is not None and pending.op_id == op and pending.boot_id == boot_id:
                return op, "accepted"
            mb.reset_cache[(boot_id, op)] = ResetRecord(
                boot_id=boot_id,
                op_id=op,
                status="failed",
                message="Reset already in progress",
            )
            return op, "failed"
        mb.pending_reset = Reset(boot_id=boot_id, op_id=op)
        mb.reset_cache[(boot_id, op)] = ResetRecord(
            boot_id=boot_id, op_id=op, status="accepted"
        )
        mb.host_mode = "resetting"
        return op, "accepted"

    async def _handle_hello(self, hello: Hello, addr: tuple[str, int]) -> None:
        mb = self.mailbox
        async with mb.lock:
            changed = self._touch_peer(hello.boot_id, addr, hello=hello)
            robot = mb.host_robot
            mode = mb.host_mode
            err = mb.host_error
        diag = hello.diag.as_dict() if hello.diag is not None else None
        logger.info("hello boot_id=%s diag=%s", hello.boot_id, diag)
        self._send(
            addr,
            encode_state_reply(robot=robot, mode=mode, error=err),
        )
        if changed:
            await self._maybe_reseed(hello.boot_id, hello)

    async def _handle_sample(self, sample: Sample, addr: tuple[str, int]) -> None:
        mb = self.mailbox
        now = time.monotonic()
        async with mb.lock:
            changed = self._touch_peer(sample.boot_id, addr, hello=None)
            if mb.last_seq is not None:
                if sample.seq == mb.last_seq:
                    op_ack, op_status = self._reset_fields(sample.boot_id, sample.op)
                    payload = encode_state_reply(
                        robot=mb.host_robot,
                        mode=mb.host_mode,
                        error=mb.host_error,
                        op_ack=op_ack,
                        op_status=op_status,
                    )
                    self._send(addr, payload)
                    return
                if sample.seq < mb.last_seq:
                    if mb.last_seq - sample.seq < 2**31:
                        return
                elif sample.seq > mb.last_seq + 1:
                    mb.seq_skips += 1
                    logger.warning(
                        "seq skip last=%s now=%s gap=%s",
                        mb.last_seq,
                        sample.seq,
                        sample.seq - mb.last_seq,
                    )
            mb.last_seq = sample.seq
            mb.latest = LatestSample(sample=sample, receipt_time=now)
            op_ack, op_status = self._reset_fields(sample.boot_id, sample.op)
            payload = encode_state_reply(
                robot=mb.host_robot,
                mode=mb.host_mode,
                error=mb.host_error,
                op_ack=op_ack,
                op_status=op_status,
            )

        if self._last_sample_receipt > 0.0:
            gap = now - self._last_sample_receipt
            if gap > 0.250:
                mb.sample_gaps += 1
                logger.warning("sample gap %.0f ms seq=%s", gap * 1000.0, sample.seq)
        self._last_sample_receipt = now
        self._send(addr, payload)
        if changed:
            await self._maybe_reseed(sample.boot_id, None)

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

    async def snapshot_status(self) -> dict[str, Any]:
        mb = self.mailbox
        async with mb.lock:
            last_rx = mb.last_rx
            present = last_rx > 0.0 and (time.monotonic() - last_rx) < LINK_STALE_SEC
            age_ms = (time.monotonic() - last_rx) * 1000.0 if last_rx > 0.0 else None
            return {
                "robot": mb.host_robot,
                "busy": mb.host_mode == "resetting",
                "mode": mb.host_mode,
                "connected": present,
                "error": mb.host_error,
                "boot_id": mb.boot_id,
                "peer": f"{mb.peer[0]}:{mb.peer[1]}" if mb.peer else None,
                "presents": mb.presents,
                "absents": mb.absents,
                "last_seq": mb.last_seq,
                "seq_skips": mb.seq_skips,
                "sample_gaps": mb.sample_gaps,
                "last_rx_age_ms": None if age_ms is None else round(age_ms, 1),
                "last_diag": mb.last_diag.as_dict() if mb.last_diag is not None else None,
                "max_tick_lag_ms": round(self.max_tick_lag_ms, 1),
                "last_sdk_ms": round(self.last_sdk_ms, 1),
            }

    def _send(self, addr: tuple[str, int], payload: dict[str, Any]) -> None:
        transport = self._transport
        if transport is None:
            return
        try:
            transport.sendto(json.dumps(payload, separators=(",", ":")).encode(), addr)
        except Exception as exc:
            logger.warning("UDP send failed: %s", exc)

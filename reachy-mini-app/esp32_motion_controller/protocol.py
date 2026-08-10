"""
Protocol v2 parsing — dependency-light, no SDK imports.

Rejects oversize frames, wrong types, non-finite numbers, and unsupported versions.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

PROTOCOL_VERSION = 2
MAX_FRAME_BYTES = 512
GAIN_MIN = 0.1
GAIN_MAX = 3.0


class ProtocolError(ValueError):
    def __init__(self, message: str, *, request_type: str = "unknown") -> None:
        super().__init__(message)
        self.request_type = request_type


@dataclass(frozen=True, slots=True)
class Hello:
    protocol_version: int
    boot_id: str
    device: str


@dataclass(frozen=True, slots=True)
class Sample:
    boot_id: str
    seq: int
    q: tuple[float, float, float, float]
    p: tuple[float, float, float]
    engaged: bool
    gain: float
    ready: bool


@dataclass(frozen=True, slots=True)
class Reset:
    boot_id: str
    op_id: int


def _require_dict(msg: Any, request_type: str) -> dict[str, Any]:
    if not isinstance(msg, dict):
        raise ProtocolError("JSON root must be an object", request_type=request_type)
    return msg


def _require_str(msg: dict[str, Any], key: str, request_type: str) -> str:
    val = msg.get(key)
    if not isinstance(val, str) or not val:
        raise ProtocolError(f"{key} must be a non-empty string", request_type=request_type)
    return val


def _require_int(msg: dict[str, Any], key: str, request_type: str) -> int:
    val = msg.get(key)
    if isinstance(val, bool) or not isinstance(val, int):
        raise ProtocolError(f"{key} must be an integer", request_type=request_type)
    return int(val)


def _require_bool(msg: dict[str, Any], key: str, request_type: str) -> bool:
    val = msg.get(key)
    if not isinstance(val, bool):
        raise ProtocolError(f"{key} must be a boolean", request_type=request_type)
    return val


def _require_finite_float(msg: dict[str, Any], key: str, request_type: str) -> float:
    val = msg.get(key)
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise ProtocolError(f"{key} must be a number", request_type=request_type)
    out = float(val)
    if not math.isfinite(out):
        raise ProtocolError(f"{key} must be finite", request_type=request_type)
    return out


def _require_vec(msg: dict[str, Any], key: str, n: int, request_type: str) -> tuple[float, ...]:
    val = msg.get(key)
    if not isinstance(val, list) or len(val) != n:
        raise ProtocolError(f"{key} must be an array of length {n}", request_type=request_type)
    out: list[float] = []
    for i, item in enumerate(val):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ProtocolError(f"{key}[{i}] must be a number", request_type=request_type)
        f = float(item)
        if not math.isfinite(f):
            raise ProtocolError(f"{key}[{i}] must be finite", request_type=request_type)
        out.append(f)
    return tuple(out)


def parse_frame(raw: str | bytes) -> Hello | Sample | Reset:
    if isinstance(raw, bytes):
        if len(raw) > MAX_FRAME_BYTES:
            raise ProtocolError("frame exceeds size limit", request_type="parse")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError(f"invalid UTF-8: {exc}", request_type="parse") from exc
    else:
        if len(raw.encode("utf-8")) > MAX_FRAME_BYTES:
            raise ProtocolError("frame exceeds size limit", request_type="parse")
        text = raw

    try:
        msg = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc}", request_type="parse") from exc

    msg = _require_dict(msg, "parse")
    msg_type = msg.get("type")
    if not isinstance(msg_type, str):
        raise ProtocolError("missing type", request_type="parse")

    if msg_type == "hello":
        return parse_hello(msg)
    if msg_type == "sample":
        return parse_sample(msg)
    if msg_type == "reset":
        return parse_reset(msg)
    raise ProtocolError(f"unknown message type: {msg_type}", request_type=msg_type)


def parse_hello(msg: dict[str, Any]) -> Hello:
    request_type = "hello"
    version = _require_int(msg, "protocol_version", request_type)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol_version: {version}",
            request_type=request_type,
        )
    return Hello(
        protocol_version=version,
        boot_id=_require_str(msg, "boot_id", request_type),
        device=_require_str(msg, "device", request_type) if "device" in msg else "esp32",
    )


def parse_sample(msg: dict[str, Any]) -> Sample:
    request_type = "sample"
    q = _require_vec(msg, "q", 4, request_type)
    p = _require_vec(msg, "p", 3, request_type)
    gain = _require_finite_float(msg, "gain", request_type)
    if gain < GAIN_MIN or gain > GAIN_MAX:
        raise ProtocolError(
            f"gain out of range [{GAIN_MIN}, {GAIN_MAX}]",
            request_type=request_type,
        )
    seq = _require_int(msg, "seq", request_type)
    if seq < 0:
        raise ProtocolError("seq must be non-negative", request_type=request_type)
    return Sample(
        boot_id=_require_str(msg, "boot_id", request_type),
        seq=seq,
        q=(q[0], q[1], q[2], q[3]),
        p=(p[0], p[1], p[2]),
        engaged=_require_bool(msg, "engaged", request_type),
        gain=gain,
        ready=_require_bool(msg, "ready", request_type),
    )


def parse_reset(msg: dict[str, Any]) -> Reset:
    request_type = "reset"
    op_id = _require_int(msg, "op_id", request_type)
    if op_id < 0:
        raise ProtocolError("op_id must be non-negative", request_type=request_type)
    return Reset(
        boot_id=_require_str(msg, "boot_id", request_type),
        op_id=op_id,
    )


def encode_hello_response(session_id: int) -> dict[str, Any]:
    return {
        "type": "hello",
        "protocol_version": PROTOCOL_VERSION,
        "session_id": int(session_id),
    }


def encode_host_state(
    *,
    robot: bool,
    mode: str,
    error: str | None = None,
) -> dict[str, Any]:
    return {
        "type": "host_state",
        "robot": bool(robot),
        "mode": mode,
        "error": error,
    }


def encode_reset_result(
    *,
    boot_id: str,
    op_id: int,
    status: str,
    message: str | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "type": "reset_result",
        "boot_id": boot_id,
        "op_id": int(op_id),
        "status": status,
    }
    if message is not None:
        out["message"] = message
    return out


def encode_error(request_type: str, message: str) -> dict[str, Any]:
    return {
        "type": "error",
        "request_type": request_type,
        "message": message,
    }

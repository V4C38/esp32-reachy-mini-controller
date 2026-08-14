"""
Protocol v4 parsing — dependency-light, no SDK imports.

UDP datagrams. Rejects oversize frames, wrong types, non-finite numbers,
and unsupported versions.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

PROTOCOL_VERSION = 4
MAX_FRAME_BYTES = 512
GAIN_MIN = 0.1
GAIN_MAX = 2.0
LINK_STALE_SEC = 1.0
HELLO_PERIOD_SEC = 2.0


class ProtocolError(ValueError):
    def __init__(self, message: str, *, request_type: str = "unknown") -> None:
        super().__init__(message)
        self.request_type = request_type


@dataclass(frozen=True, slots=True)
class LinkDiag:
    rst: int = 0
    wifi_n: int = 0
    wifi_r: int = 0
    rssi: int = 0
    wifi_up: int = 0
    down_ms: int = 0
    send_ok: int = 0
    send_fail: int = 0
    send_ms: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "rst": self.rst,
            "wifi_n": self.wifi_n,
            "wifi_r": self.wifi_r,
            "rssi": self.rssi,
            "wifi_up": self.wifi_up,
            "down_ms": self.down_ms,
            "send_ok": self.send_ok,
            "send_fail": self.send_fail,
            "send_ms": self.send_ms,
        }


@dataclass(frozen=True, slots=True)
class Hello:
    protocol_version: int
    boot_id: str
    device: str
    diag: LinkDiag | None = None


@dataclass(frozen=True, slots=True)
class Sample:
    boot_id: str
    seq: int
    q: tuple[float, float, float, float]
    engaged: bool
    gain: float
    ready: bool
    op: int | None = None


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


def parse_frame(raw: str | bytes) -> Hello | Sample:
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
    version = msg.get("pv")
    if not isinstance(version, int) or isinstance(version, bool):
        raise ProtocolError("missing pv", request_type="parse")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol_version: {version}",
            request_type="hello" if msg.get("type") == "hello" else "parse",
        )

    if msg.get("type") == "hello":
        return parse_hello(msg)
    return parse_sample(msg)


def _optional_int(msg: dict[str, Any], key: str) -> int:
    val = msg.get(key)
    if isinstance(val, bool) or not isinstance(val, int):
        return 0
    return int(val)


def parse_link_diag(raw: Any) -> LinkDiag | None:
    if not isinstance(raw, dict):
        return None
    return LinkDiag(
        rst=_optional_int(raw, "rst"),
        wifi_n=_optional_int(raw, "wifi_n"),
        wifi_r=_optional_int(raw, "wifi_r"),
        rssi=_optional_int(raw, "rssi"),
        wifi_up=_optional_int(raw, "wifi_up"),
        down_ms=_optional_int(raw, "down_ms"),
        send_ok=_optional_int(raw, "send_ok"),
        send_fail=_optional_int(raw, "send_fail"),
        send_ms=_optional_int(raw, "send_ms"),
    )


def parse_hello(msg: dict[str, Any]) -> Hello:
    request_type = "hello"
    return Hello(
        protocol_version=PROTOCOL_VERSION,
        boot_id=_require_str(msg, "boot_id", request_type),
        device=_require_str(msg, "device", request_type) if "device" in msg else "esp32",
        diag=parse_link_diag(msg.get("diag")),
    )


def parse_sample(msg: dict[str, Any]) -> Sample:
    request_type = "sample"
    q = _require_vec(msg, "q", 4, request_type)
    gain = _require_finite_float(msg, "gain", request_type)
    if gain < GAIN_MIN or gain > GAIN_MAX:
        raise ProtocolError(
            f"gain out of range [{GAIN_MIN}, {GAIN_MAX}]",
            request_type=request_type,
        )
    seq = _require_int(msg, "seq", request_type)
    if seq < 0:
        raise ProtocolError("seq must be non-negative", request_type=request_type)
    op = None
    if "op" in msg:
        op_val = _require_int(msg, "op", request_type)
        if op_val < 0:
            raise ProtocolError("op must be non-negative", request_type=request_type)
        if op_val > 0:
            op = op_val
    return Sample(
        boot_id=_require_str(msg, "boot_id", request_type),
        seq=seq,
        q=(q[0], q[1], q[2], q[3]),
        engaged=_require_bool(msg, "engaged", request_type),
        gain=gain,
        ready=_require_bool(msg, "ready", request_type),
        op=op,
    )


def encode_state_reply(
    *,
    robot: bool,
    mode: str,
    error: str | None = None,
    op_ack: int | None = None,
    op_status: str | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "pv": PROTOCOL_VERSION,
        "robot": bool(robot),
        "mode": mode,
        "error": error,
    }
    if op_ack is not None:
        out["op_ack"] = int(op_ack)
        out["op_status"] = op_status
    return out

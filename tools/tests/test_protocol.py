"""Protocol v2 parsing and encoding tests."""

from __future__ import annotations

import json

import pytest

from esp32_motion_controller.protocol import (
    MAX_FRAME_BYTES,
    PROTOCOL_VERSION,
    ProtocolError,
    encode_error,
    encode_hello_response,
    encode_host_state,
    encode_reset_result,
    parse_frame,
)


def test_parse_hello_ok():
    msg = parse_frame(
        json.dumps(
            {
                "type": "hello",
                "protocol_version": 2,
                "boot_id": "abc",
                "device": "esp32-reachy-ctl",
            }
        )
    )
    assert msg.protocol_version == PROTOCOL_VERSION
    assert msg.boot_id == "abc"


def test_parse_hello_rejects_v1():
    with pytest.raises(ProtocolError, match="unsupported"):
        parse_frame(json.dumps({"type": "hello", "protocol_version": 1, "boot_id": "x"}))


def test_parse_sample_ok():
    sample = parse_frame(
        json.dumps(
            {
                "type": "sample",
                "boot_id": "abc",
                "seq": 3,
                "q": [1, 0, 0, 0],
                "p": [0, 0, 0],
                "engaged": True,
                "gain": 1.0,
                "ready": True,
            }
        )
    )
    assert sample.seq == 3
    assert sample.engaged is True


def test_parse_sample_rejects_nan():
    with pytest.raises(ProtocolError):
        parse_frame(
            json.dumps(
                {
                    "type": "sample",
                    "boot_id": "abc",
                    "seq": 1,
                    "q": [1, 0, 0, float("nan")],
                    "p": [0, 0, 0],
                    "engaged": False,
                    "gain": 1.0,
                    "ready": True,
                }
            )
        )


def test_parse_sample_rejects_gain_range():
    with pytest.raises(ProtocolError, match="gain"):
        parse_frame(
            json.dumps(
                {
                    "type": "sample",
                    "boot_id": "abc",
                    "seq": 1,
                    "q": [1, 0, 0, 0],
                    "p": [0, 0, 0],
                    "engaged": False,
                    "gain": 9.0,
                    "ready": True,
                }
            )
        )


def test_oversize_frame_rejected():
    raw = "{" + ("a" * (MAX_FRAME_BYTES + 10)) + "}"
    with pytest.raises(ProtocolError, match="size"):
        parse_frame(raw)


def test_reset_and_encoders():
    reset = parse_frame(json.dumps({"type": "reset", "boot_id": "b", "op_id": 7}))
    assert reset.op_id == 7
    assert encode_hello_response(3)["session_id"] == 3
    assert encode_host_state(robot=True, mode="idle")["mode"] == "idle"
    assert encode_reset_result(boot_id="b", op_id=7, status="completed")["status"] == "completed"
    assert encode_error("reset", "nope")["message"] == "nope"

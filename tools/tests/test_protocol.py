"""Protocol v4 parsing and encoding tests."""

from __future__ import annotations

import json

import pytest

from esp32_motion_controller.protocol import (
    MAX_FRAME_BYTES,
    PROTOCOL_VERSION,
    ProtocolError,
    encode_state_reply,
    parse_frame,
)


def test_parse_hello_ok():
    msg = parse_frame(
        json.dumps(
            {
                "pv": 4,
                "type": "hello",
                "boot_id": "abc",
                "device": "esp32-reachy-ctl",
            }
        )
    )
    assert msg.protocol_version == PROTOCOL_VERSION
    assert msg.boot_id == "abc"
    assert msg.diag is None


def test_parse_hello_diag():
    msg = parse_frame(
        json.dumps(
            {
                "pv": 4,
                "type": "hello",
                "boot_id": "abc",
                "device": "esp32-reachy-ctl",
                "diag": {
                    "rst": 1,
                    "wifi_n": 2,
                    "wifi_r": 201,
                    "rssi": -61,
                    "wifi_up": 1,
                    "down_ms": 180,
                    "send_ok": 40,
                    "send_fail": 1,
                    "send_ms": 4,
                },
            }
        )
    )
    assert msg.diag is not None
    assert msg.diag.wifi_r == 201
    assert msg.diag.down_ms == 180


def test_parse_hello_rejects_v3():
    with pytest.raises(ProtocolError, match="unsupported"):
        parse_frame(json.dumps({"pv": 3, "type": "hello", "boot_id": "x"}))


def test_parse_hello_rejects_v2():
    with pytest.raises(ProtocolError, match="unsupported"):
        parse_frame(json.dumps({"pv": 2, "type": "hello", "boot_id": "x"}))


def test_parse_sample_ok():
    sample = parse_frame(
        json.dumps(
            {
                "pv": 4,
                "boot_id": "abc",
                "seq": 3,
                "q": [1, 0, 0, 0],
                "engaged": True,
                "gain": 1.0,
                "ready": True,
                "op": 3,
            }
        )
    )
    assert sample.seq == 3
    assert sample.engaged is True
    assert sample.op == 3


def test_parse_sample_omits_op():
    sample = parse_frame(
        json.dumps(
            {
                "pv": 4,
                "boot_id": "abc",
                "seq": 1,
                "q": [1, 0, 0, 0],
                "engaged": False,
                "gain": 1.0,
                "ready": True,
            }
        )
    )
    assert sample.op is None


def test_parse_sample_rejects_nan():
    with pytest.raises(ProtocolError):
        parse_frame(
            json.dumps(
                {
                    "pv": 4,
                    "boot_id": "abc",
                    "seq": 1,
                    "q": [1, 0, 0, float("nan")],
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
                    "pv": 4,
                    "boot_id": "abc",
                    "seq": 1,
                    "q": [1, 0, 0, 0],
                    "engaged": False,
                    "gain": 9.0,
                    "ready": True,
                }
            )
        )
    with pytest.raises(ProtocolError, match="gain"):
        parse_frame(
            json.dumps(
                {
                    "pv": 4,
                    "boot_id": "abc",
                    "seq": 1,
                    "q": [1, 0, 0, 0],
                    "engaged": False,
                    "gain": 2.1,
                    "ready": True,
                }
            )
        )


def test_parse_sample_accepts_gain_max():
    sample = parse_frame(
        json.dumps(
            {
                "pv": 4,
                "boot_id": "abc",
                "seq": 1,
                "q": [1, 0, 0, 0],
                "engaged": False,
                "gain": 2.0,
                "ready": True,
            }
        )
    )
    assert sample.gain == 2.0


def test_oversize_frame_rejected():
    raw = "{" + ("a" * (MAX_FRAME_BYTES + 10)) + "}"
    with pytest.raises(ProtocolError, match="size"):
        parse_frame(raw)


def test_state_reply_encoder():
    plain = encode_state_reply(robot=True, mode="idle")
    assert plain["pv"] == PROTOCOL_VERSION
    assert plain["mode"] == "idle"
    assert "op_ack" not in plain
    with_op = encode_state_reply(
        robot=True, mode="resetting", op_ack=7, op_status="accepted"
    )
    assert with_op["op_ack"] == 7
    assert with_op["op_status"] == "accepted"

"""Session mailbox, reset idempotency, and robot control fault tests."""

from __future__ import annotations

import asyncio
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from esp32_motion_controller.protocol import Reset
from esp32_motion_controller.robot_control import RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub


def _pose_matrix(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


class FakeWebSocket:
    def __init__(self) -> None:
        self.client_state = type("S", (), {"name": "CONNECTED"})()
        from starlette.websockets import WebSocketState

        self.client_state = WebSocketState.CONNECTED
        self.sent: list[dict] = []

    async def accept(self) -> None:
        return None

    async def send_json(self, payload: dict) -> None:
        self.sent.append(payload)

    async def close(self, code: int = 1000) -> None:
        from starlette.websockets import WebSocketState

        self.client_state = WebSocketState.DISCONNECTED


@pytest.mark.asyncio
async def test_hello_and_latest_sample():
    hub = SessionHub(robot_available=True)
    ws = FakeWebSocket()
    gen = await hub.on_connect(ws)  # type: ignore[arg-type]
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )
    assert any(m.get("type") == "hello" for m in ws.sent)
    assert any(m.get("type") == "host_state" for m in ws.sent)

    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"sample","boot_id":"b1","seq":1,"q":[1,0,0,0],"p":[0,0,0],'
        '"engaged":true,"gain":1.0,"ready":true}',
    )
    latest = await hub.take_latest_sample()
    assert latest is not None
    assert latest.sample.seq == 1

    # Duplicate dropped
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"sample","boot_id":"b1","seq":1,"q":[1,0,0,0],"p":[0,0,0],'
        '"engaged":true,"gain":1.0,"ready":true}',
    )
    latest2 = await hub.take_latest_sample()
    assert latest2 is not None
    assert latest2.sample.seq == 1


@pytest.mark.asyncio
async def test_reset_idempotent_cache():
    hub = SessionHub(robot_available=True)
    ws = FakeWebSocket()
    gen = await hub.on_connect(ws)  # type: ignore[arg-type]
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"reset","boot_id":"b1","op_id":3}',
    )
    pending = await hub.take_pending_reset()
    assert pending is not None and pending.op_id == 3
    await hub.complete_reset(boot_id="b1", op_id=3, status="completed")

    ws2 = FakeWebSocket()
    gen2 = await hub.on_connect(ws2)  # type: ignore[arg-type]
    await hub.handle_message(
        ws2,  # type: ignore[arg-type]
        gen2,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )
    await hub.handle_message(
        ws2,  # type: ignore[arg-type]
        gen2,
        '{"type":"reset","boot_id":"b1","op_id":3}',
    )
    # Cached completed — must not enqueue another pending reset.
    assert await hub.take_pending_reset() is None
    assert any(
        m.get("type") == "reset_result" and m.get("status") == "completed" for m in ws2.sent
    )


@pytest.mark.asyncio
async def test_robot_control_reset_and_seed():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix(yaw=0.4, pitch=0.1)
    mini.get_current_joint_positions.return_value = ([0.05] + [0.0] * 6, [0, 0])
    mini.set_target = MagicMock()
    mini.goto_target = MagicMock()

    hub = SessionHub(robot_available=True)
    gw = RobotGateway(mini, log_only=False)
    control = RobotControl(hub, gw, robot_available=True, log_only=False)
    await control.on_controller_connected()
    assert control.state.seeded
    assert control.state.baseline_pose["yaw"] == pytest.approx(0.4, abs=1e-5)

    # Queue a reset and run it through the private helper.
    await hub.mailbox.lock.acquire()
    hub.mailbox.pending_reset = Reset(boot_id="b1", op_id=1)
    hub.mailbox.host_mode = "resetting"
    hub.mailbox.lock.release()

    loop = asyncio.get_running_loop()
    await control._run_reset(loop, Reset(boot_id="b1", op_id=1), time.monotonic())
    mini.goto_target.assert_called()
    assert control.state.mode == "idle"
    assert control.state.desired_pose["yaw"] == pytest.approx(0.0)


@pytest.mark.asyncio
async def test_seed_failure_freezes():
    mini = MagicMock()
    mini.get_current_head_pose.side_effect = RuntimeError("offline")
    mini.get_current_joint_positions.side_effect = RuntimeError("offline")
    hub = SessionHub(robot_available=True)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    ok = await control._seed(update_host=True)
    assert ok is False
    assert control.state.sends_frozen is True


@pytest.mark.asyncio
async def test_set_target_failure_does_not_advance_baseline():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target.side_effect = RuntimeError("ik")

    hub = SessionHub(robot_available=True)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    await control.on_controller_connected()
    baseline = dict(control.state.baseline_pose)

    ws = FakeWebSocket()
    gen = await hub.on_connect(ws)  # type: ignore[arg-type]
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"sample","boot_id":"b1","seq":1,"q":[1,0,0,0],"p":[0,0,0],'
        '"engaged":true,"gain":1.0,"ready":true}',
    )
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"sample","boot_id":"b1","seq":2,"q":[0.995,0.1,0,0],"p":[0,0,0],'
        '"engaged":true,"gain":1.0,"ready":true}',
    )

    loop = asyncio.get_running_loop()
    # Drive a few ticks; failures should freeze without inventing baseline.
    for _ in range(5):
        await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.baseline_pose["yaw"] == pytest.approx(baseline["yaw"], abs=1e-5)
    await control.stop()

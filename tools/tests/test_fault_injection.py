"""Fault-injection: SDK latency, gaps, and no dual in-flight commands."""

from __future__ import annotations

import asyncio
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from esp32_motion_controller.robot_control import RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub


def _pose_matrix(yaw=0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [0.0, 0.0, yaw]).as_matrix()
    return m


class CountingGateway(RobotGateway):
    def __init__(self, mini, delay_s: float = 0.0) -> None:
        super().__init__(mini, log_only=False)
        self.delay_s = delay_s
        self.in_flight = 0
        self.max_in_flight = 0
        self.calls = 0

    def set_target(self, command) -> None:  # type: ignore[no-untyped-def]
        self.in_flight += 1
        self.max_in_flight = max(self.max_in_flight, self.in_flight)
        self.calls += 1
        try:
            if self.delay_s:
                time.sleep(self.delay_s)
            super().set_target(command)
        finally:
            self.in_flight -= 1


class FakeWebSocket:
    def __init__(self) -> None:
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
async def test_sdk_latency_single_inflight():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    gw = CountingGateway(mini, delay_s=0.08)
    control = RobotControl(hub, gw, robot_available=True, log_only=False, hz=20.0)
    await control.on_controller_connected()

    ws = FakeWebSocket()
    gen = await hub.on_connect(ws)  # type: ignore[arg-type]
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )

    loop = asyncio.get_running_loop()
    # Feed samples faster than delayed SDK can complete.
    for seq in range(1, 12):
        await hub.handle_message(
            ws,  # type: ignore[arg-type]
            gen,
            '{"type":"sample","boot_id":"b1","seq":%d,"q":[1,0,0,0],"p":[0,0,0],'
            '"engaged":true,"gain":1.0,"ready":true}' % seq,
        )
        await control._tick(loop, time.monotonic(), 0.05)

    assert gw.max_in_flight <= 1
    assert gw.calls >= 1
    await control.stop()


@pytest.mark.asyncio
async def test_latest_sample_wins_under_burst():
    hub = SessionHub(robot_available=True)
    ws = FakeWebSocket()
    gen = await hub.on_connect(ws)  # type: ignore[arg-type]
    await hub.handle_message(
        ws,  # type: ignore[arg-type]
        gen,
        '{"type":"hello","protocol_version":2,"boot_id":"b1","device":"esp32"}',
    )
    for seq in range(1, 6):
        await hub.handle_message(
            ws,  # type: ignore[arg-type]
            gen,
            '{"type":"sample","boot_id":"b1","seq":%d,"q":[1,0,0,0],"p":[0,0,%.3f],'
            '"engaged":false,"gain":1.0,"ready":true}' % (seq, seq * 0.001),
        )
    latest = await hub.take_latest_sample()
    assert latest is not None
    assert latest.sample.seq == 5
    assert latest.sample.p[2] == pytest.approx(0.005)

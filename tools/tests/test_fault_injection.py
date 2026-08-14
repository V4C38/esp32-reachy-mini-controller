"""Fault-injection: SDK latency, gaps, speed lock, and no dual in-flight commands."""

from __future__ import annotations

import asyncio
import json
import math
import time
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from esp32_motion_controller.control import MAX_ANGULAR_VEL, MAX_DT_FOR_VEL_CLAMP, MAX_POS_VEL
from esp32_motion_controller.robot_control import RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub

PEER = ("127.0.0.1", 40000)


def _pose_matrix(yaw=0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [0.0, 0.0, yaw]).as_matrix()
    return m


class RecordingGateway(RobotGateway):
    def __init__(self, mini, delay_s: float = 0.0) -> None:
        super().__init__(mini, log_only=False)
        self.delay_s = delay_s
        self.in_flight = 0
        self.max_in_flight = 0
        self.calls = 0
        self.commands: list[object] = []
        self.times: list[float] = []

    def set_target(self, command) -> None:  # type: ignore[no-untyped-def]
        self.in_flight += 1
        self.max_in_flight = max(self.max_in_flight, self.in_flight)
        self.calls += 1
        self.commands.append(command)
        self.times.append(time.monotonic())
        try:
            if self.delay_s:
                time.sleep(self.delay_s)
            super().set_target(command)
        finally:
            self.in_flight -= 1


class FakeTransport:
    def __init__(self) -> None:
        self.sent: list[dict] = []

    def sendto(self, data: bytes, addr) -> None:
        self.sent.append(json.loads(data.decode()))


def _quat_wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def _sample_json(seq: int, q=None, engaged: bool = True) -> str:
    return json.dumps(
        {
            "pv": 4,
            "boot_id": "b1",
            "seq": seq,
            "q": q or [1, 0, 0, 0],
            "engaged": engaged,
            "gain": 1.0,
            "ready": True,
        }
    )


async def _hello(hub: SessionHub) -> None:
    hub.bind_transport(FakeTransport())
    await hub.handle_datagram(
        json.dumps({"pv": 4, "type": "hello", "boot_id": "b1", "device": "esp32"}).encode(),
        PEER,
    )


async def _ingest(hub: SessionHub, raw: str) -> None:
    await hub.handle_datagram(raw.encode(), PEER)


def _assert_command_speeds(gw: RecordingGateway) -> None:
    for i in range(1, len(gw.commands)):
        prev = gw.commands[i - 1]
        cur = gw.commands[i]
        max_d_ang = MAX_ANGULAR_VEL * MAX_DT_FOR_VEL_CLAMP
        for axis in ("roll", "pitch", "yaw"):
            delta = cur.pose[axis] - prev.pose[axis]
            if axis == "yaw":
                delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
            assert abs(delta) <= max_d_ang + 1e-6
        pos = math.sqrt(
            sum((cur.pose[k] - prev.pose[k]) ** 2 for k in ("x", "y", "z"))
        )
        assert pos <= MAX_POS_VEL * MAX_DT_FOR_VEL_CLAMP + 1e-6


@pytest.mark.asyncio
async def test_sdk_latency_single_inflight():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    gw = RecordingGateway(mini, delay_s=0.08)
    control = RobotControl(hub, gw, robot_available=True, log_only=False, hz=20.0)
    await control.on_controller_connected()
    await _hello(hub)

    loop = asyncio.get_running_loop()
    for seq in range(1, 12):
        await _ingest(hub, _sample_json(seq))
        await control._tick(loop, time.monotonic(), 0.05)

    assert gw.max_in_flight <= 1
    assert gw.calls >= 1
    await control.stop()


@pytest.mark.asyncio
async def test_latest_sample_wins_under_burst():
    hub = SessionHub(robot_available=True)
    await _hello(hub)
    for seq in range(1, 6):
        await _ingest(
            hub,
            _sample_json(seq, engaged=False),
        )
    latest = await hub.take_latest_sample()
    assert latest is not None
    assert latest.sample.seq == 5


@pytest.mark.asyncio
async def test_output_guard_caps_180_and_combined_axis():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    gw = RecordingGateway(mini)
    control = RobotControl(hub, gw, robot_available=True, log_only=False, hz=20.0)
    await control.on_controller_connected()
    await _hello(hub)
    loop = asyncio.get_running_loop()
    now = time.monotonic()

    await _ingest(hub, _sample_json(1))
    await control._tick(loop, now, 0.05)

    q = _quat_wxyz(math.pi / 2, math.pi / 2, math.pi)
    await _ingest(hub, _sample_json(2, q=q))
    await control._tick(loop, now + 0.05, 0.05)
    await _ingest(hub, _sample_json(3, q=q))
    await control._tick(loop, now + 0.10, 0.05)
    assert gw.calls >= 1
    _assert_command_speeds(gw)
    await control.stop()


@pytest.mark.asyncio
async def test_output_guard_survives_reseed_and_delayed_sdk():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    gw = RecordingGateway(mini, delay_s=0.12)
    control = RobotControl(hub, gw, robot_available=True, log_only=False, hz=20.0)
    await control.on_controller_connected()
    await _hello(hub)
    loop = asyncio.get_running_loop()
    now = time.monotonic()

    await _ingest(hub, _sample_json(1))
    await control._tick(loop, now, 0.05)

    await control.on_controller_connected()
    await _hello(hub)
    q = _quat_wxyz(0.0, 0.0, math.pi)
    await _ingest(hub, _sample_json(3, q=q))
    await control._tick(loop, time.monotonic(), 0.05)
    await _ingest(hub, _sample_json(4, q=q))
    await control._tick(loop, time.monotonic(), 0.05)
    _assert_command_speeds(gw)
    await control.stop()

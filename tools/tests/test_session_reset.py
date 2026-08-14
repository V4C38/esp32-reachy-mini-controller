"""Session mailbox, reset idempotency, and robot control fault tests."""

from __future__ import annotations

import asyncio
import json
import math
import time
from dataclasses import replace
from unittest.mock import MagicMock

import numpy as np
import pytest

from esp32_motion_controller.control import (
    DISENGAGED_PITCH,
    DISENGAGED_Z,
    MAX_ANGULAR_VEL,
    MAX_POS_VEL,
    STALE_PACKET_SEC,
    Command,
    disengaged_rest_pose,
    force_disengage,
    zero_pose,
)
from esp32_motion_controller.protocol import PROTOCOL_VERSION, Hello, Reset
from esp32_motion_controller.robot_control import CULL_ANG_RAD, CULL_POS_M, RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub

PEER = ("127.0.0.1", 40000)


def _pose_matrix(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


class FakeTransport:
    def __init__(self) -> None:
        self.sent: list[dict] = []

    def sendto(self, data: bytes, addr) -> None:
        self.sent.append(json.loads(data.decode()))


def _bind(hub: SessionHub) -> FakeTransport:
    transport = FakeTransport()
    hub.bind_transport(transport)
    return transport


def _hello_json(boot_id: str = "b1") -> str:
    return json.dumps({"pv": 4, "type": "hello", "boot_id": boot_id, "device": "esp32"})


def _sample_json(
    seq: int,
    *,
    boot_id: str = "b1",
    engaged: bool = True,
    q=None,
    op: int | None = None,
) -> str:
    msg = {
        "pv": 4,
        "boot_id": boot_id,
        "seq": seq,
        "q": q or [1, 0, 0, 0],
        "engaged": engaged,
        "gain": 1.0,
        "ready": True,
    }
    if op is not None:
        msg["op"] = op
    return json.dumps(msg)


async def _ingest(hub: SessionHub, raw: str, addr=PEER) -> None:
    await hub.handle_datagram(raw.encode(), addr)


@pytest.mark.asyncio
async def test_hello_and_latest_sample():
    hub = SessionHub(robot_available=True)
    _bind(hub)
    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))
    latest = await hub.take_latest_sample()
    assert latest is not None
    assert latest.sample.seq == 1

    await _ingest(hub, _sample_json(4))
    snap = await hub.snapshot_status()
    assert snap["seq_skips"] == 1
    assert snap["last_seq"] == 4
    assert snap["connected"] is True

    await _ingest(hub, _sample_json(4))
    latest2 = await hub.take_latest_sample()
    assert latest2 is not None
    assert latest2.sample.seq == 4


@pytest.mark.asyncio
async def test_reorder_and_loss():
    hub = SessionHub(robot_available=True)
    _bind(hub)
    await _ingest(hub, _sample_json(1))
    await _ingest(hub, _sample_json(3))
    snap = await hub.snapshot_status()
    assert snap["seq_skips"] == 1
    latest = await hub.take_latest_sample()
    assert latest is not None and latest.sample.seq == 3

    await _ingest(hub, _sample_json(2))
    latest2 = await hub.take_latest_sample()
    assert latest2 is not None and latest2.sample.seq == 3


@pytest.mark.asyncio
async def test_reset_idempotent_cache():
    hub = SessionHub(robot_available=True)
    transport = _bind(hub)
    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1, op=3))
    pending = await hub.take_pending_reset()
    assert pending is not None and pending.op_id == 3
    await hub.complete_reset(boot_id="b1", op_id=3, status="completed")

    transport.sent.clear()
    await _ingest(hub, _sample_json(2, op=3))
    assert await hub.take_pending_reset() is None
    assert any(m.get("op_ack") == 3 and m.get("op_status") == "completed" for m in transport.sent)


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

    await hub.mailbox.lock.acquire()
    hub.mailbox.pending_reset = Reset(boot_id="b1", op_id=1)
    hub.mailbox.host_mode = "resetting"
    hub.mailbox.lock.release()

    loop = asyncio.get_running_loop()
    start_z = control._sent_pose["z"] if control._sent_pose else 0.0
    await control._tick(loop, time.monotonic(), 0.05)
    mini.goto_target.assert_not_called()
    assert mini.set_target.call_count >= 1
    assert control._anim == "reset"
    dz = abs(control._sent_pose["z"] - start_z)
    assert dz <= MAX_POS_VEL * control.dt + 1e-6

    for _ in range(120):
        await control._tick(loop, time.monotonic(), 0.05)
        if control._posture == "ducked" and control._anim is None:
            break
    assert control._posture == "ducked"
    assert control.state.mode == "idle"
    assert control.state.desired_pose["z"] == pytest.approx(DISENGAGED_Z)
    assert control.state.desired_pose["pitch"] == pytest.approx(DISENGAGED_PITCH)
    mini.goto_target.assert_not_called()
    await control.stop()


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
    _bind(hub)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    await control.on_controller_connected()
    baseline = dict(control.state.baseline_pose)

    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))
    await _ingest(hub, _sample_json(2, q=[0.995, 0.1, 0, 0]))

    loop = asyncio.get_running_loop()
    for _ in range(5):
        await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.baseline_pose["yaw"] == pytest.approx(baseline["yaw"], abs=1e-5)
    await control.stop()


@pytest.mark.asyncio
async def test_appear_disappear_speed_locked_set_target():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix(
        z=DISENGAGED_Z, pitch=DISENGAGED_PITCH
    )
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()
    mini.goto_target = MagicMock()

    hub = SessionHub(robot_available=True)
    _bind(hub)
    gw = RobotGateway(mini, log_only=False)
    control = RobotControl(hub, gw, robot_available=True, log_only=False)
    await control.on_controller_connected()
    control._posture = "ducked"
    control._remember_sent(disengaged_rest_pose(), 0.0, time.monotonic())

    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))

    loop = asyncio.get_running_loop()
    t = 1000.0
    await control._tick(loop, t, 0.05)
    mini.goto_target.assert_not_called()
    assert mini.set_target.call_count == 1
    assert control._anim == "appear"
    dz = abs(control._sent_pose["z"] - DISENGAGED_Z)
    assert dz <= MAX_POS_VEL * control.dt + 1e-6
    assert control._posture == "ducked"

    control._state = replace(control._state, behavior_t0=t)
    for i in range(10):
        await control._tick(loop, t + 0.05 * (i + 1), 0.05)
    antenna_calls = [
        c.kwargs["antennas"]
        for c in mini.set_target.call_args_list
        if "antennas" in c.kwargs
    ]
    assert any(abs(float(a[0])) + abs(float(a[1])) > 0.01 for a in antenna_calls)

    for _ in range(80):
        await control._tick(loop, time.monotonic(), 0.05)
        if control._posture == "neutral" and control._anim is None:
            break
    assert control._posture == "neutral"
    assert control._sent_pose["z"] == pytest.approx(0.0, abs=0.006)

    await _ingest(hub, _sample_json(2))
    await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.engaged

    await _ingest(hub, _sample_json(3, engaged=False))
    await control._tick(loop, time.monotonic(), 0.05)
    mini.goto_target.assert_not_called()
    assert control._anim == "disappear"
    for _ in range(80):
        await control._tick(loop, time.monotonic(), 0.05)
        if control._posture == "ducked" and control._anim is None:
            break
    assert control._posture == "ducked"
    assert control.state.desired_pose["z"] == pytest.approx(DISENGAGED_Z)
    assert control.state.mode == "idle"
    await control.stop()


@pytest.mark.asyncio
async def test_sample_gap_while_connected_does_not_disappear():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])

    hub = SessionHub(robot_available=True)
    _bind(hub)
    gw = RobotGateway(mini, log_only=False)
    control = RobotControl(hub, gw, robot_available=True, log_only=False)
    await control.on_controller_connected()
    control._posture = "neutral"
    control._remember_sent(zero_pose(), 0.0, time.monotonic())

    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))
    loop = asyncio.get_running_loop()
    t0 = time.monotonic()
    await control._tick(loop, t0, 0.05)
    assert control.state.engaged

    await control._tick(loop, t0 + STALE_PACKET_SEC + 0.05, 0.05)
    assert control._anim is None
    assert control.state.engaged
    assert control._posture == "neutral"
    await control.stop()


def _hello_msg(boot_id: str = "b1") -> Hello:
    return Hello(protocol_version=PROTOCOL_VERSION, boot_id=boot_id, device="esp32")


@pytest.mark.asyncio
async def test_same_boot_reconnect_grace_keeps_clutch():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    _bind(hub)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    hub.on_hello = control.on_controller_hello
    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))
    loop = asyncio.get_running_loop()
    t0 = time.monotonic()
    control._posture = "neutral"
    await control._tick(loop, t0, 0.05)
    assert control.state.engaged
    q_ref = control.state.q_ref.copy()
    desired = dict(control.state.desired_pose)

    await control.on_controller_absent()
    assert control.state.engaged
    assert np.allclose(control.state.q_ref, q_ref)

    await _ingest(hub, _hello_json())
    assert control.state.engaged
    assert np.allclose(control.state.q_ref, q_ref)
    assert control.state.desired_pose["yaw"] == pytest.approx(desired["yaw"])
    assert control._posture == "neutral"

    await _ingest(hub, _sample_json(2, q=[0.995, 0.1, 0, 0]))
    await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.engaged
    assert np.allclose(control.state.q_ref, q_ref)
    await control.stop()


@pytest.mark.asyncio
async def test_same_boot_grace_keeps_clutch_while_disengaged():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])

    hub = SessionHub(robot_available=True)
    _bind(hub)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    hub.on_hello = control.on_controller_hello
    await _ingest(hub, _hello_json())
    await _ingest(hub, _sample_json(1))
    loop = asyncio.get_running_loop()
    control._posture = "neutral"
    await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.engaged
    q_ref = control.state.q_ref.copy()
    control._state = force_disengage(control._state)
    assert not control.state.engaged

    await control.on_controller_hello(_hello_msg("b1"))
    assert np.allclose(control.state.q_ref, q_ref)
    assert control._posture == "neutral"
    await control.stop()


@pytest.mark.asyncio
async def test_unknown_posture_engages_without_appear():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()
    mini.goto_target = MagicMock()

    hub = SessionHub(robot_available=True)
    transport = _bind(hub)
    gw = RobotGateway(mini, log_only=False)
    control = RobotControl(hub, gw, robot_available=True, log_only=False)
    hub.on_hello = control.on_controller_hello
    await control.on_controller_connected()
    assert control._posture == "unknown"

    pose_reads = mini.get_current_head_pose.call_count
    await _ingest(hub, _hello_json())
    assert control._posture == "unknown"
    assert mini.get_current_head_pose.call_count == pose_reads
    assert any(m.get("pv") == PROTOCOL_VERSION for m in transport.sent)

    await _ingest(hub, _sample_json(1))
    loop = asyncio.get_running_loop()
    await control._tick(loop, time.monotonic(), 0.05)
    mini.goto_target.assert_not_called()
    assert control._anim is None
    assert control.state.engaged
    assert control._posture == "unknown"
    await control.stop()


@pytest.mark.asyncio
async def test_new_boot_reconnect_reseeds():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])

    hub = SessionHub(robot_available=True)
    _bind(hub)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    hub.on_hello = control.on_controller_hello
    await _ingest(hub, _hello_json("old"))
    await _ingest(hub, _sample_json(1, boot_id="old"))
    loop = asyncio.get_running_loop()
    control._posture = "neutral"
    await control._tick(loop, time.monotonic(), 0.05)
    assert control.state.engaged

    await control.on_controller_hello(_hello_msg("new"))
    assert not control.state.engaged
    assert control._boot_id == "new"
    await control.stop()


@pytest.mark.asyncio
async def test_same_boot_hello_keeps_clutch_after_socket_gap():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    mini.set_target = MagicMock()

    hub = SessionHub(robot_available=True)
    _bind(hub)
    gw = RobotGateway(mini, log_only=False)
    control = RobotControl(hub, gw, robot_available=True, log_only=False)
    hub.on_hello = control.on_controller_hello
    await control.on_controller_hello(_hello_msg("b1"))
    loop = asyncio.get_running_loop()
    await _ingest(hub, _sample_json(1))
    control._posture = "neutral"
    t0 = time.monotonic()
    await control._tick(loop, t0, 0.05)
    assert control.state.engaged
    q_ref = control.state.q_ref.copy()

    hub.mailbox.last_rx = 0.0
    hub._present = True
    await control._tick(loop, t0 + STALE_PACKET_SEC + 0.05, 0.05)
    assert control._anim is None
    assert not control.state.engaged
    assert control._posture == "neutral"
    assert control._button_engaged

    await control.on_controller_hello(_hello_msg("b1"))
    assert np.allclose(control.state.q_ref, q_ref)
    assert control._posture == "neutral"

    await _ingest(hub, _sample_json(2))
    await control._tick(loop, time.monotonic(), 0.05)
    assert control._anim is None
    assert control.state.engaged
    await control.stop()


@pytest.mark.asyncio
async def test_guard_culls_large_jump_and_caps_small_move():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix()
    mini.get_current_joint_positions.return_value = ([0.0] * 7, [0, 0])
    hub = SessionHub(robot_available=True)
    control = RobotControl(
        hub, RobotGateway(mini), robot_available=True, log_only=False
    )
    await control.on_controller_connected()
    control._remember_sent(zero_pose(), 0.0, time.monotonic())

    huge = Command(
        pose={**zero_pose(), "yaw": math.pi, "z": 0.20},
        body_yaw=0.0,
        antennas=(0.0, 0.0),
    )
    held = control._guard_command(huge, time.monotonic())
    assert held.pose["yaw"] == pytest.approx(0.0)
    assert held.pose["z"] == pytest.approx(0.0)
    dist, ang = (
        abs(held.pose["z"]),
        abs(held.pose["yaw"]),
    )
    assert dist < CULL_POS_M and ang < CULL_ANG_RAD

    modest = Command(
        pose={**zero_pose(), "yaw": math.radians(8.0)},
        body_yaw=0.0,
        antennas=(0.0, 0.0),
    )
    capped = control._guard_command(modest, time.monotonic())
    assert 0.0 < abs(capped.pose["yaw"]) <= MAX_ANGULAR_VEL * control.dt + 1e-9
    await control.stop()

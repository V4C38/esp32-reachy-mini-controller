"""Watchdog / reset interlock state-machine tests (no asyncio server)."""

from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

import numpy as np
import pytest

from esp32_motion_controller.behavior import Behavior
from esp32_motion_controller.controller_state import ControllerState
from esp32_motion_controller.movement_handler import MovementHandler
from esp32_motion_controller.ws_handler import WebSocketHandler


def _pose_matrix(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


@pytest.mark.asyncio
async def test_reset_rebases_clutch():
    movement = MovementHandler(None, send_rate_hz=20)
    controller = ControllerState()
    behavior = Behavior()
    handler = WebSocketHandler(
        movement, controller, behavior, robot_available=True, log_only=True
    )
    movement.start()

    # Engage and rotate via update.
    # Device rot about +Y → head yaw under DEV_TO_HEAD.
    q0 = [1, 0, 0, 0]
    controller.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    from scipy.spatial.transform import Rotation as R
    q = R.from_euler("xyz", [0, 0.3, 0]).as_quat()
    q_wxyz = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
    controller.update(q=q_wxyz, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    controller.update(q=q_wxyz, p=[0, 0, 0], engaged=False, gain=1.0, ready=True)
    assert abs(controller.base_pose["yaw"] - 0.3) < 1e-5

    # Seed movement at the clutched pose so reset has somewhere to come from
    movement.set_target(controller.desired_pose, body_yaw=0.0, antennas=[0.0, 0.0])
    for _ in range(5):
        await asyncio.sleep(0.04)

    result = await handler._handle_reset({})
    assert result["success"] is True
    assert handler.busy is True
    assert controller.base_pose["yaw"] == 0.0

    await asyncio.sleep(0.12)  # let minjerk goto start toward neutral
    yaw_mid = abs(movement.target_pose["yaw"])
    assert yaw_mid < 0.28

    # Streaming controller_state must not yank the target back to the old pose
    await handler._handle_controller_state(
        {
            "q": q_wxyz,
            "p": [0, 0, 0],
            "engaged": True,
            "gain": 1.0,
            "ready": True,
        }
    )
    assert not controller.engaged
    assert abs(movement.target_pose["yaw"]) <= yaw_mid + 1e-6

    await asyncio.sleep(1.6)
    assert handler.busy is False
    assert controller.base_pose["yaw"] == 0.0
    assert controller.desired_pose["yaw"] == 0.0
    movement.stop()


@pytest.mark.asyncio
async def test_reset_resyncs_then_drives_target_to_zero():
    mini = MagicMock()
    # Robot is at a non-zero pose; bookkeeping starts elsewhere.
    mini.get_current_head_pose.return_value = _pose_matrix(yaw=0.4, pitch=0.1)
    mini.get_current_joint_positions.return_value = ([0.05, 0, 0, 0, 0, 0, 0], [0, 0])
    mini.set_target = MagicMock()

    movement = MovementHandler(mini, send_rate_hz=20)
    controller = ControllerState()
    behavior = Behavior()
    handler = WebSocketHandler(
        movement, controller, behavior, robot_available=True, log_only=False
    )
    movement.start()
    # Stale zero bookkeeping + freeze — reset must resync before goto.
    movement._sends_frozen = True
    movement._current_pose = {
        "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0
    }
    movement._target_pose = dict(movement._current_pose)
    movement._prev_sent_pose = dict(movement._current_pose)

    result = await handler._handle_reset({})
    assert result["success"] is True
    assert movement._sends_frozen is False
    # Goto starts from the measured pose (resync), not the stale zeros.
    assert movement._prev_sent_pose["yaw"] == pytest.approx(0.4, abs=1e-5)

    await asyncio.sleep(0.7)
    assert abs(movement.target_pose["yaw"]) < 0.25
    assert abs(movement.target_pose["pitch"]) < 0.08

    await asyncio.sleep(1.1)
    assert handler.busy is False
    assert movement.target_pose["yaw"] == pytest.approx(0.0)
    assert movement.target_pose["pitch"] == pytest.approx(0.0)
    movement.stop()


@pytest.mark.asyncio
async def test_reset_fails_closed_when_pose_unread():
    mini = MagicMock()
    mini.get_current_head_pose.side_effect = RuntimeError("offline")
    mini.get_current_joint_positions.side_effect = RuntimeError("offline")

    movement = MovementHandler(mini, send_rate_hz=20)
    controller = ControllerState()
    behavior = Behavior()
    handler = WebSocketHandler(
        movement, controller, behavior, robot_available=True, log_only=False
    )

    result = await handler._handle_reset({})
    assert result["success"] is False
    assert "unread" in result["message"].lower()
    assert handler.busy is False


def test_force_disengage_commits():
    cs = ControllerState()
    from scipy.spatial.transform import Rotation as R
    # Device rot about +X → head pitch under DEV_TO_HEAD.
    q = R.from_euler("xyz", [0.1, 0, 0]).as_quat()
    q0 = [1, 0, 0, 0]
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    q1 = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
    cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    cs.force_disengage()
    assert not cs.engaged
    assert abs(cs.base_pose["pitch"] - 0.1) < 1e-5


@pytest.mark.asyncio
async def test_cleanup_keeps_movement_running():
    movement = MovementHandler(None, send_rate_hz=20)
    controller = ControllerState()
    behavior = Behavior()
    handler = WebSocketHandler(
        movement, controller, behavior, robot_available=False, log_only=True
    )
    movement.start()
    task = movement._apply_task
    assert task is not None and not task.done()

    # Simulate a blip: detach without a real websocket object.
    handler._active_ws = object()  # type: ignore[assignment]
    handler.cleanup()
    assert movement._apply_task is task
    assert not task.done()

    handler.shutdown()
    await asyncio.sleep(0.05)
    assert movement._apply_task is None


"""Velocity-gate / seed-from-robot safety tests (no real robot)."""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from esp32_motion_controller.movement_handler import MovementHandler


def _pose_matrix(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def test_seed_failure_does_not_zero_prev_sent():
    mini = MagicMock()
    mini.get_current_head_pose.side_effect = RuntimeError("offline")
    mini.get_current_joint_positions.side_effect = RuntimeError("offline")

    mh = MovementHandler(mini, send_rate_hz=20)
    mh._prev_sent_pose = {"x": 0.01, "y": 0.0, "z": 0.01, "roll": 0.2, "pitch": 0.1, "yaw": 0.3}
    mh._prev_sent_body_yaw = 0.15
    mh._seeded = True

    assert mh._seed_from_robot() is False
    assert mh._sends_frozen is True
    assert mh._prev_sent_pose["yaw"] == pytest.approx(0.3)
    assert mh._prev_sent_body_yaw == pytest.approx(0.15)


def test_rebase_failure_freezes_instead_of_zeroing():
    mini = MagicMock()
    mini.get_current_head_pose.side_effect = RuntimeError("offline")
    mini.get_current_joint_positions.side_effect = RuntimeError("offline")

    mh = MovementHandler(mini, send_rate_hz=20)
    mh._prev_sent_pose["pitch"] = 0.25
    mh._target_pose["pitch"] = 0.25
    mh.rebase_to_neutral()
    assert mh._sends_frozen is True
    assert mh._prev_sent_pose["pitch"] == pytest.approx(0.25)
    # Failure must not invent a zero target either.
    assert mh._target_pose["pitch"] == pytest.approx(0.25)


def test_rebase_keeps_targets_at_neutral():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix(pitch=0.2, yaw=0.3)
    mini.get_current_joint_positions.return_value = ([0.1, 0, 0, 0, 0, 0, 0], [0, 0])

    mh = MovementHandler(mini, send_rate_hz=20)
    mh._target_pose["yaw"] = 0.05
    mh._current_pose["yaw"] = 0.05
    mh.rebase_to_neutral()
    assert mh._sends_frozen is False
    # Clamp reference follows the robot…
    assert mh._prev_sent_pose["yaw"] == pytest.approx(0.3, abs=1e-5)
    assert mh._prev_sent_body_yaw == pytest.approx(0.1, abs=1e-5)
    # …but targets stay at neutral so the goto is not undone.
    assert mh._target_pose["yaw"] == pytest.approx(0.0)
    assert mh._current_pose["yaw"] == pytest.approx(0.0)
    assert mh._target_pose["pitch"] == pytest.approx(0.0)
    assert mh._target_body_yaw == pytest.approx(0.0)


def test_seed_success_clears_freeze():
    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix(pitch=0.12, yaw=0.05)
    mini.get_current_joint_positions.return_value = ([0.02, 0, 0, 0, 0, 0, 0], [0, 0])

    mh = MovementHandler(mini, send_rate_hz=20)
    mh._sends_frozen = True
    assert mh.resync_from_robot() is True
    assert mh._sends_frozen is False
    assert mh._prev_sent_pose["pitch"] == pytest.approx(0.12, abs=1e-5)
    assert mh._current_pose["pitch"] == pytest.approx(0.12, abs=1e-5)


@pytest.mark.asyncio
async def test_soft_resync_within_eps_only_updates_prev():
    mini = MagicMock()
    mh = MovementHandler(mini, send_rate_hz=20)
    mh._prev_sent_pose = _zero = {
        "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0
    }
    mh._target_pose = dict(_zero)
    mh._target_pose["yaw"] = 0.4
    mh._current_pose = dict(mh._target_pose)
    mh._prev_sent_body_yaw = 0.0

    # Measured pose is nearly at prev_sent — soft refresh only.
    mini.get_current_head_pose.return_value = _pose_matrix(yaw=0.01)
    mini.get_current_joint_positions.return_value = ([0.0] + [0.0] * 6, [0, 0])
    await mh._periodic_resync()
    assert mh._prev_sent_pose["yaw"] == pytest.approx(0.01, abs=1e-5)
    # Target left alone (still chasing 0.4)
    assert mh._target_pose["yaw"] == pytest.approx(0.4)


@pytest.mark.asyncio
async def test_hard_resync_skipped_while_goto_active():
    """Apply loop must not call periodic resync while a goto owns the target."""
    import asyncio
    from unittest.mock import AsyncMock, patch

    mini = MagicMock()
    mini.get_current_head_pose.return_value = _pose_matrix(yaw=0.5)
    mini.get_current_joint_positions.return_value = ([0.0] + [0.0] * 6, [0, 0])
    mini.set_target = MagicMock()

    mh = MovementHandler(mini, send_rate_hz=20)
    mh.start()
    mh._active_gotos["reset-uuid"] = True
    mh._last_resync_time = 0.0  # force the interval check to fire
    mh._target_pose["yaw"] = 0.4
    mh._current_pose["yaw"] = 0.4
    mh._prev_sent_pose["yaw"] = 0.4

    with patch.object(mh, "_periodic_resync", new_callable=AsyncMock) as resync:
        await asyncio.sleep(0.15)
        resync.assert_not_called()
        assert mh._target_pose["yaw"] == pytest.approx(0.4)

    mh._active_gotos.clear()
    mh._last_resync_time = 0.0
    with patch.object(mh, "_periodic_resync", new_callable=AsyncMock) as resync:
        await asyncio.sleep(0.15)
        resync.assert_called()
    mh.stop()

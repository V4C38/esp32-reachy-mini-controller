"""Clutch rising-edge, device→head mapping, and commit-on-release tests."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

from esp32_motion_controller.controller_state import (
    DEV_TO_HEAD,
    TRANSLATION_SCALE,
    ControllerState,
    quat_relative_rpy,
    remap_displacement,
)


def _wxyz_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    """Device-frame extrinsic xyz Euler → wire-format wxyz quaternion."""
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()  # xyzw
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def test_rising_edge_captures_reference_zero_delta():
    cs = ControllerState()
    q0 = _wxyz_from_euler(0.1, -0.2, 0.3)
    pose = cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["roll"]) < 1e-6
    assert abs(pose["pitch"]) < 1e-6
    assert abs(pose["yaw"]) < 1e-6


def test_device_pitch_becomes_head_pitch():
    """Tip the top of the board toward you (rot about device +X) → head pitch nose-down."""
    cs = ControllerState()
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    q1 = _wxyz_from_euler(0.1, 0, 0)  # +rot about device X
    pose = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["pitch"] - 0.1) < 1e-5
    assert abs(pose["roll"]) < 1e-5
    assert abs(pose["yaw"]) < 1e-5


def test_device_yaw_becomes_head_yaw():
    """Turn the screen toward your right (rot about device +Y) → head yaw same way."""
    cs = ControllerState()
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    q1 = _wxyz_from_euler(0, 0.1, 0)  # +rot about device Y
    pose = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["yaw"] - 0.1) < 1e-5
    assert abs(pose["roll"]) < 1e-5
    assert abs(pose["pitch"]) < 1e-5


def test_device_roll_becomes_head_roll():
    """Raise the screen's right edge (rot about device +Z) → head roll same way."""
    cs = ControllerState()
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    q1 = _wxyz_from_euler(0, 0, 0.1)  # +rot about device Z
    pose = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["roll"] - 0.1) < 1e-5
    assert abs(pose["pitch"]) < 1e-5
    assert abs(pose["yaw"]) < 1e-5


def test_relative_rotation_applies_gain():
    cs = ControllerState()
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=2.0, ready=True)
    # Device pitch → head pitch, then × gain
    q1 = _wxyz_from_euler(0.1, 0, 0)
    pose = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=2.0, ready=True)
    assert abs(pose["pitch"] - 0.2) < 1e-5


def test_large_angle_similarity_transform():
    """45° tip about device X must land as 45° head pitch (not a cyclic permute)."""
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    angle = math.radians(45.0)
    q1 = np.asarray(_wxyz_from_euler(angle, 0, 0))
    roll, pitch, yaw = quat_relative_rpy(q0, q1)
    assert abs(pitch - angle) < 1e-5
    assert abs(roll) < 1e-5
    assert abs(yaw) < 1e-5

    # Combined tip + turn at large angle: similarity, not component-wise assign.
    q2 = np.asarray(_wxyz_from_euler(angle, angle, 0))
    roll, pitch, yaw = quat_relative_rpy(q0, q2)
    # Exact values from the similarity transform; just check axes aren't swapped.
    r_dev = R.from_euler("xyz", [angle, angle, 0])
    m = R.from_matrix(DEV_TO_HEAD)
    expected = (m * r_dev * m.inv()).as_euler("xyz")
    assert abs(roll - expected[0]) < 1e-6
    assert abs(pitch - expected[1]) < 1e-6
    assert abs(yaw - expected[2]) < 1e-6


def test_release_commits_pose_into_base():
    cs = ControllerState()
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    # Device yaw (about Y) → head yaw
    q1 = _wxyz_from_euler(0, 0.25, 0)
    mid = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(mid["yaw"] - 0.25) < 1e-5
    held = cs.update(q=q1, p=[0, 0, 0], engaged=False, gain=1.0, ready=True)
    assert abs(held["yaw"] - 0.25) < 1e-5
    again = cs.update(q=q1, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(again["yaw"] - 0.25) < 1e-5


def test_not_ready_cannot_engage():
    cs = ControllerState()
    q0 = _wxyz_from_euler(0.2, 0, 0)
    pose = cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=False)
    assert pose["pitch"] == 0.0
    assert not cs.engaged


def test_displacement_remap_face_forward():
    """Screen = face: device +Z (toward user) → head +x; +X (right) → head +y; +Y → +z."""
    cs = ControllerState(translation_gain=1.0)
    q0 = _wxyz_from_euler(0, 0, 0)
    cs.update(q=q0, p=[0, 0, 0], engaged=True, gain=1.0, ready=True)

    # device +Y (lift) → head +z
    pose = cs.update(q=q0, p=[0, 0.01, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["z"] - 0.01 * TRANSLATION_SCALE) < 1e-9

    # device +X (right) → head +y (screen's left is robot left when facing you)
    pose = cs.update(q=q0, p=[0.01, 0, 0], engaged=True, gain=1.0, ready=True)
    assert abs(pose["y"] - 0.01 * TRANSLATION_SCALE) < 1e-9

    # device +Z (toward user / out of screen) → head +x (forward)
    pose = cs.update(q=q0, p=[0, 0, 0.01], engaged=True, gain=1.0, ready=True)
    assert abs(pose["x"] - 0.01 * TRANSLATION_SCALE) < 1e-9


def test_world_displacement_rotated_into_ref():
    """p is world-frame; a yawed engage reference must cancel before DEV_TO_HEAD."""
    # Engage with device yawed 90° about world Z (= device Y when upright... but
    # q encodes body→world. A 90° yaw about device Y maps world +X onto device -Z
    # etc. Use identity q_ref and a world delta, then a rotated q_ref that makes
    # the same physical device-frame move produce the same head delta.
    q_id = np.array([1.0, 0.0, 0.0, 0.0])
    p_world = np.array([0.0, 0.01, 0.0])  # world +Y
    d0 = remap_displacement(p_world, q_id, translation_gain=1.0)

    # Rotate the reference 90° about Z: world +Y becomes device -X in the ref.
    q_yaw = np.asarray(_wxyz_from_euler(0, 0, math.pi / 2))
    # For the same *device-frame* lift (+Y_ref), the world vector under q_yaw is
    # r_yaw.apply([0, 0.01, 0]).
    r_yaw = R.from_quat([q_yaw[1], q_yaw[2], q_yaw[3], q_yaw[0]])
    p_world_yawed = r_yaw.apply([0.0, 0.01, 0.0])
    d1 = remap_displacement(p_world_yawed, q_yaw, translation_gain=1.0)
    np.testing.assert_allclose(d0, d1, atol=1e-9)
    # And that common value is head +z.
    assert abs(d0[2] - 0.01 * TRANSLATION_SCALE) < 1e-9
    assert abs(d0[0]) < 1e-9
    assert abs(d0[1]) < 1e-9


def test_quat_relative_identity():
    q = np.array([1.0, 0, 0, 0])
    r, p, y = quat_relative_rpy(q, q)
    assert abs(r) + abs(p) + abs(y) < 1e-9

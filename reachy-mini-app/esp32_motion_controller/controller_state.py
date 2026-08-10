"""
Clutch mapping: relative orientation + displacement from the ESP32 controller.

Owns the engage rising-edge reference and the desired head pose (x/y/z/rpy).
Does not write body_yaw or antennas.

Frames
------
Device (after IMU_MAP_*): X right, Y up, Z out of screen (screen = Reachy face).
Head:                     x forward, y left, z up.

DEV_TO_HEAD maps device vectors onto head vectors (screen↔face, up↔up):
  head_x = device_z, head_y = device_x, head_z = device_y

`p` arrives in the gravity-aligned world frame. On engage we rotate the
world-frame delta into the clutch reference (device frame at engage), then
apply DEV_TO_HEAD.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")

# Device → head: face with face, up with up (det = +1).
# head_x = dev_z (face), head_y = dev_x (screen's left), head_z = dev_y (up)
DEV_TO_HEAD = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# Hand travel (metres) → head travel: ~50 mm of hand maps onto ~15 mm of head
# at translation_gain = 1.0, so a comfortable push fills most of the workspace.
TRANSLATION_SCALE = 0.30  # base m_head / m_hand before the UI gain
TRANSLATION_GAIN_DEFAULT = 1.0


def _finite_quat(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(arr)) or np.linalg.norm(arr) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return arr / np.linalg.norm(arr)


def _finite_vec3(v: Sequence[float]) -> np.ndarray:
    arr = np.asarray(v, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(arr)):
        return np.zeros(3, dtype=np.float64)
    return arr


def _wxyz_to_rotation(q: np.ndarray) -> R:
    # scipy uses [x, y, z, w]; wire format is [w, x, y, z]
    return R.from_quat([q[1], q[2], q[3], q[0]])


def quat_relative_rpy(q_ref: np.ndarray, q_device: np.ndarray) -> tuple[float, float, float]:
    """Return head (roll, pitch, yaw) of the relative device rotation.

    Applies the similarity transform M * R_rel_dev * M^{-1} so that a
    physical tip/turn/roll of the board becomes the matching head RPY.
    Euler order is extrinsic xyz, matching create_head_pose.
    """
    r_ref = _wxyz_to_rotation(q_ref)
    r_dev = _wxyz_to_rotation(q_device)
    r_rel_dev = r_ref.inv() * r_dev
    m = R.from_matrix(DEV_TO_HEAD)
    r_head = m * r_rel_dev * m.inv()
    roll, pitch, yaw = r_head.as_euler("xyz", degrees=False)
    return float(roll), float(pitch), float(yaw)


def remap_displacement(
    p_world_delta: np.ndarray,
    q_ref: np.ndarray,
    *,
    translation_gain: float = TRANSLATION_GAIN_DEFAULT,
) -> np.ndarray:
    """Map world-frame displacement delta to head-frame metres.

    Rotates into the engage reference (device frame at clutch), then applies
    DEV_TO_HEAD, then scales by TRANSLATION_SCALE * translation_gain.
    """
    r_ref = _wxyz_to_rotation(q_ref)
    disp_ref = r_ref.inv().apply(p_world_delta)
    disp_head = DEV_TO_HEAD @ disp_ref
    return disp_head * TRANSLATION_SCALE * float(translation_gain)


class ControllerState:
    """Clutch state machine for one ESP32 controller."""

    def __init__(
        self,
        *,
        translation_gain: float = TRANSLATION_GAIN_DEFAULT,
    ) -> None:
        self.engaged = False
        self.gain = 1.0
        self.translation_gain = float(translation_gain)
        self.ready = False
        self._q_ref = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._p_ref = np.zeros(3, dtype=np.float64)
        self._base_pose = {k: 0.0 for k in POSE_AXES}
        self._desired = {k: 0.0 for k in POSE_AXES}
        self._was_engaged = False

    @property
    def desired_pose(self) -> dict[str, float]:
        return dict(self._desired)

    @property
    def base_pose(self) -> dict[str, float]:
        return dict(self._base_pose)

    def set_base_pose(self, pose: dict[str, float]) -> None:
        self._base_pose = {k: float(pose.get(k, 0.0)) for k in POSE_AXES}
        if not self.engaged:
            self._desired = dict(self._base_pose)

    def rebase_neutral(self) -> None:
        self._base_pose = {k: 0.0 for k in POSE_AXES}
        self._desired = {k: 0.0 for k in POSE_AXES}
        self.engaged = False
        self._was_engaged = False
        self._q_ref = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._p_ref = np.zeros(3, dtype=np.float64)

    def update(
        self,
        *,
        q: Sequence[float],
        p: Sequence[float],
        engaged: bool,
        gain: float,
        ready: bool,
        allow_engage: bool = True,
    ) -> dict[str, float]:
        """Ingest one controller_state packet. Returns desired head pose."""
        self.ready = bool(ready)
        self.gain = float(gain) if math.isfinite(float(gain)) else self.gain
        self.gain = max(0.1, min(3.0, self.gain))

        q_dev = _finite_quat(q)
        p_dev = _finite_vec3(p)

        want_engage = bool(engaged) and self.ready and allow_engage
        rising = want_engage and not self._was_engaged
        falling = (not want_engage) and self._was_engaged

        if rising:
            self._q_ref = q_dev.copy()
            self._p_ref = p_dev.copy()
            # base already holds the committed pose from last release / reset

        self.engaged = want_engage
        self._was_engaged = want_engage

        if self.engaged:
            roll, pitch, yaw = quat_relative_rpy(self._q_ref, q_dev)
            # Translation uses its own gain (and TRANSLATION_SCALE); rotation
            # uses the UI gain. Both multiply the UI gain so the slider still
            # scales the whole motion feel.
            disp = remap_displacement(
                p_dev - self._p_ref,
                self._q_ref,
                translation_gain=self.translation_gain * self.gain,
            )
            self._desired = {
                "x": self._base_pose["x"] + float(disp[0]),
                "y": self._base_pose["y"] + float(disp[1]),
                "z": self._base_pose["z"] + float(disp[2]),
                "roll": self._base_pose["roll"] + self.gain * roll,
                "pitch": self._base_pose["pitch"] + self.gain * pitch,
                "yaw": self._base_pose["yaw"] + self.gain * yaw,
            }
        elif falling:
            # Commit both rotation and translation into the base on release
            self._base_pose = dict(self._desired)
        # else idle: hold last desired / base

        return dict(self._desired)

    def force_disengage(self) -> None:
        if self.engaged:
            self._base_pose = dict(self._desired)
        self.engaged = False
        self._was_engaged = False

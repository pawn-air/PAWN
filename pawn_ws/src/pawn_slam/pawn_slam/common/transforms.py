"""SE(3) poses, quaternions and the fixed frames of PAWN.

Conventions (REP-103):
* ``base_link`` is FLU: x forward, y left, z up, meters.
* ``camera_optical_frame``: x right, y down, z forward (the OpenCV camera frame).
* Quaternions are ``(x, y, z, w)``, the order of ``geometry_msgs/Quaternion``.
* ``Pose(R, t)`` named ``T_a_b`` maps points from frame b to frame a: ``p_a = R @ p_b + t``.
  Composition chains frames: ``T_a_c = T_a_b @ T_b_c``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Columns are the optical axes written in FLU: x_opt = -y_flu (right), y_opt = -z_flu (down), z_opt = x_flu (forward).
R_BASE_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


# ----------------------------------------------------------------- quaternions
def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return q / n


def quat_to_rot(q) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R) -> np.ndarray:
    """Rotation matrix -> unit quaternion with w >= 0 (Shepperd: stable for every angle, 180 deg included)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = quat_normalize(q)
    return -q if q[3] < 0 else q


def quat_slerp(q0, q1, s: float) -> np.ndarray:
    """Spherical interpolation, s in [0, 1], along the shorter arc."""
    q0, q1 = quat_normalize(q0), quat_normalize(q1)
    d = float(np.dot(q0, q1))
    if d < 0:                      # q and -q are the same rotation: take the short way
        q1, d = -q1, -d
    if d > 0.9995:                 # nearly identical: lerp avoids dividing by sin(~0)
        return quat_normalize(q0 + s * (q1 - q0))
    th = np.arccos(d)
    return (np.sin((1 - s) * th) * q0 + np.sin(s * th) * q1) / np.sin(th)


# ------------------------------------------------------------------ rotations
def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Radians, ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll). In FLU, +pitch is nose down, +yaw is to the left."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def tello_attitude_to_rot(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Tello state ``roll``/``pitch``/``yaw`` (degrees) -> rotation of base_link (FLU) in a gravity-aligned frame.

    UNVERIFIED: assumes the Tello reports aircraft angles (roll + = right side down, pitch + = nose up,
    yaw + = clockwise seen from above). In FLU that is roll +, pitch -, yaw -. Check against a recorded
    flight (tilt nose up, roll right, turn right) before relying on it.
    """
    return rot_from_rpy(np.radians(roll_deg), -np.radians(pitch_deg), -np.radians(yaw_deg))


# ----------------------------------------------------------------------- pose
@dataclass(frozen=True, eq=False)
class Pose:
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        object.__setattr__(self, "R", np.asarray(self.R, dtype=float).reshape(3, 3))
        object.__setattr__(self, "t", np.asarray(self.t, dtype=float).reshape(3))

    @classmethod
    def identity(cls) -> Pose:
        return cls()

    @classmethod
    def from_quat(cls, t, q) -> Pose:
        return cls(quat_to_rot(q), t)

    @classmethod
    def from_matrix(cls, M) -> Pose:
        M = np.asarray(M, dtype=float)
        return cls(M[:3, :3], M[:3, 3])

    @property
    def quat(self) -> np.ndarray:
        return rot_to_quat(self.R)

    @property
    def matrix(self) -> np.ndarray:
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = self.R, self.t
        return M

    def inverse(self) -> Pose:
        Rt = self.R.T
        return Pose(Rt, -Rt @ self.t)

    def __matmul__(self, other: Pose) -> Pose:
        return Pose(self.R @ other.R, self.R @ other.t + self.t)

    def apply(self, points) -> np.ndarray:
        """Transform one point (3,) or many (N, 3)."""
        return np.asarray(points, dtype=float) @ self.R.T + self.t

    def __repr__(self) -> str:
        return f"Pose(t={np.round(self.t, 4).tolist()}, q={np.round(self.quat, 4).tolist()})"


def interpolate(a: Pose, b: Pose, s: float) -> Pose:
    """Pose between a (s=0) and b (s=1): position linear, rotation slerp."""
    return Pose(quat_to_rot(quat_slerp(a.quat, b.quat, s)), (1 - s) * a.t + s * b.t)


def base_to_optical(pitch_down_deg: float = 0.0, t=(0.0, 0.0, 0.0)) -> Pose:
    """T_base_optical: camera mounted at ``t`` (FLU, m), tilted down by ``pitch_down_deg``.

    Tello mount values (tilt, offset) are not measured yet: they come with the calibration.
    """
    return Pose(rot_from_rpy(0.0, np.radians(pitch_down_deg), 0.0) @ R_BASE_OPTICAL, t)

"""Pure-numpy rigid-transform utilities (no ROS, no scipy dependency).

Conventions used everywhere in this package
-------------------------------------------
* A "transform" / "pose" is a 4x4 homogeneous matrix ``T`` such that a point
  expressed in frame B maps to frame A via ``p_A = A_T_B @ p_B``.  We name
  matrices ``A_T_B`` = "B expressed in A" = "maps B -> A".
* Quaternions follow the **ROS order (x, y, z, w)** in every public function of
  this module.  scipy uses (x, y, z, w) as well, but ``transforms3d`` and the
  low-level OpenCV/tf math often use (w, x, y, z) -- do NOT mix them.  Convert
  at the boundary with :func:`quaternion_ros_to_wxyz` / :func:`quaternion_wxyz_to_ros`.
* Units are meters for translation and radians for any angle argument.

Only numpy is required so this module is trivially unit-testable off-robot.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

_EPS = 1e-12


# --------------------------------------------------------------------------- #
# Quaternion helpers (ROS order x, y, z, w)
# --------------------------------------------------------------------------- #
def normalize_quaternion(q: Sequence[float]) -> np.ndarray:
    """Return a unit quaternion (x, y, z, w).  Falls back to identity on ~0 norm."""
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n < _EPS:
        return np.array([0.0, 0.0, 0.0, 1.0])
    q = q / n
    # Canonicalize sign so w >= 0 (q and -q are the same rotation).
    if q[3] < 0.0:
        q = -q
    return q


def quaternion_ros_to_wxyz(q_xyzw: Sequence[float]) -> np.ndarray:
    """(x, y, z, w) -> (w, x, y, z)."""
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z], dtype=float)


def quaternion_wxyz_to_ros(q_wxyz: Sequence[float]) -> np.ndarray:
    """(w, x, y, z) -> (x, y, z, w)."""
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w], dtype=float)


def quaternion_to_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    """ROS quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = normalize_quaternion(q_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ])


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> ROS quaternion (x, y, z, w), w >= 0.

    Uses the numerically stable branch method (Shepperd / Shuster).
    """
    R = np.asarray(R, dtype=float)[:3, :3]
    trace = np.trace(R)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion([x, y, z, w])


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis XYZ (roll about x, pitch about y, yaw about z) -> 3x3.

    Matches ``R.from_euler('xyz', ...)`` used by the Piper driver.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """3x3 -> (roll, pitch, yaw) fixed-axis XYZ, radians. GUI display only."""
    R = np.asarray(R, dtype=float)[:3, :3]
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 1.0 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def quaternion_to_euler(q_xyzw: Sequence[float]) -> Tuple[float, float, float]:
    return matrix_to_euler(quaternion_to_matrix(q_xyzw))


# --------------------------------------------------------------------------- #
# Homogeneous transform construction / decomposition
# --------------------------------------------------------------------------- #
def make_transform(R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)[:3, :3]
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def transform_from_quaternion(t: Sequence[float], q_xyzw: Sequence[float]) -> np.ndarray:
    return make_transform(quaternion_to_matrix(q_xyzw), t)


def decompose_transform(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """4x4 -> (R 3x3, t 3,)."""
    T = np.asarray(T, dtype=float)
    return T[:3, :3].copy(), T[:3, 3].copy()


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform (transpose rotation, rotate translation)."""
    R, t = decompose_transform(T)
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def compose_transform(*transforms: np.ndarray) -> np.ndarray:
    """Left-to-right matrix product: compose_transform(A_T_B, B_T_C) == A_T_C."""
    out = np.eye(4)
    for T in transforms:
        out = out @ np.asarray(T, dtype=float)
    return out


# --------------------------------------------------------------------------- #
# ROS message <-> matrix (imported lazily so the math core stays ROS-free)
# --------------------------------------------------------------------------- #
def pose_msg_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose (or Transform) -> 4x4."""
    if hasattr(pose, "position"):
        p, o = pose.position, pose.orientation
        t = [p.x, p.y, p.z]
    else:  # Transform
        p, o = pose.translation, pose.rotation
        t = [p.x, p.y, p.z]
    return transform_from_quaternion(t, [o.x, o.y, o.z, o.w])


def matrix_to_pose_msg(T: np.ndarray):
    """4x4 -> geometry_msgs/Pose."""
    from geometry_msgs.msg import Pose  # local import; ROS only needed at runtime
    R, t = decompose_transform(T)
    q = matrix_to_quaternion(R)
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = float(t[0]), float(t[1]), float(t[2])
    msg.orientation.x = float(q[0])
    msg.orientation.y = float(q[1])
    msg.orientation.z = float(q[2])
    msg.orientation.w = float(q[3])
    return msg


def matrix_to_transform_msg(T: np.ndarray):
    """4x4 -> geometry_msgs/Transform."""
    from geometry_msgs.msg import Transform
    R, t = decompose_transform(T)
    q = matrix_to_quaternion(R)
    msg = Transform()
    msg.translation.x, msg.translation.y, msg.translation.z = float(t[0]), float(t[1]), float(t[2])
    msg.rotation.x = float(q[0])
    msg.rotation.y = float(q[1])
    msg.rotation.z = float(q[2])
    msg.rotation.w = float(q[3])
    return msg


# --------------------------------------------------------------------------- #
# Geometric metrics / averaging
# --------------------------------------------------------------------------- #
def rotation_angle(R: np.ndarray) -> float:
    """Rotation magnitude of a 3x3 matrix, radians in [0, pi]."""
    R = np.asarray(R, dtype=float)[:3, :3]
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.acos(cos_theta)


def rotation_angle_between(Ta: np.ndarray, Tb: np.ndarray) -> float:
    """Angle (radians) between the rotation parts of two transforms."""
    Ra, _ = decompose_transform(Ta)
    Rb, _ = decompose_transform(Tb)
    return rotation_angle(Ra.T @ Rb)


def translation_distance(Ta: np.ndarray, Tb: np.ndarray) -> float:
    _, ta = decompose_transform(Ta)
    _, tb = decompose_transform(Tb)
    return float(np.linalg.norm(ta - tb))


def quaternion_average(quats_xyzw: Iterable[Sequence[float]]) -> np.ndarray:
    """Markley average of unit quaternions (ROS order in and out).

    Robust to sign ambiguity; returns the dominant eigenvector of the
    accumulated outer-product matrix.
    """
    A = np.zeros((4, 4))
    n = 0
    for q in quats_xyzw:
        qn = normalize_quaternion(q).reshape(4, 1)
        A += qn @ qn.T
        n += 1
    if n == 0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    A /= n
    eigvals, eigvecs = np.linalg.eigh(A)
    q = eigvecs[:, np.argmax(eigvals)]
    return normalize_quaternion(q)


def average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Average a list of 4x4 transforms: mean translation + quaternion average."""
    transforms = list(transforms)
    if not transforms:
        raise ValueError("average_transforms: empty input")
    ts = np.array([decompose_transform(T)[1] for T in transforms])
    quats = [matrix_to_quaternion(decompose_transform(T)[0]) for T in transforms]
    R = quaternion_to_matrix(quaternion_average(quats))
    return make_transform(R, ts.mean(axis=0))


def is_valid_rotation(R: np.ndarray, tol: float = 1e-4) -> bool:
    """True if R is (numerically) a proper rotation: orthonormal, det ~ +1."""
    R = np.asarray(R, dtype=float)[:3, :3]
    if not np.all(np.isfinite(R)):
        return False
    if abs(np.linalg.det(R) - 1.0) > tol:
        return False
    return np.allclose(R.T @ R, np.eye(3), atol=tol)

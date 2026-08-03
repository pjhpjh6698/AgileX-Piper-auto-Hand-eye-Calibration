#!/usr/bin/env python3
"""Forward kinematics and reachability against the ARM'S OWN model.

The arm's controller solves IK with the modified-DH table shipped in pyAgxArm.
A URDF (used by RViz, MoveIt and the Gazebo rig) does not have to agree with it
about where the flange frame sits, and when it does not, Cartesian poses tuned
in simulation are silently unreachable on hardware -- the arm accepts the
command, finds no solution, and does not move.

So anything that needs to answer "can the arm actually go there?" must ask the
arm's model, not the URDF. That is what this module is for.

Validated against ground truth: FK here reproduces the pose the arm reports for
its own joint angles to 0.00 mm / 0.00 deg. See test_agx_kinematics.py.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:
    from pyAgxArm.api.constants import (ROBOT_JOINT_LIMIT_PRESET_RAD,
                                        ROBOT_MDH_PRESET)
    _HAVE_SDK = True
except ImportError:  # allow import without the SDK; callers check HAVE_SDK
    ROBOT_MDH_PRESET = {}
    ROBOT_JOINT_LIMIT_PRESET_RAD = {}
    _HAVE_SDK = False

HAVE_SDK = _HAVE_SDK
ROBOT = "piper"

MDH = ROBOT_MDH_PRESET.get(ROBOT, ())
JOINT_LIMITS: List[Tuple[float, float]] = [
    tuple(ROBOT_JOINT_LIMIT_PRESET_RAD.get(ROBOT, {}).get(f"joint{i}", (-math.pi, math.pi)))
    for i in range(1, 7)
]


def fk(joint_angles: Sequence[float]) -> np.ndarray:
    """base_T_flange (4x4) for the given joint angles [rad].

    Standard modified-DH product; the table already carries each link's
    theta offset.
    """
    T = np.eye(4)
    for (d, a, alpha, theta_offset), theta in zip(MDH, joint_angles):
        ct, st = math.cos(theta + theta_offset), math.sin(theta + theta_offset)
        ca, sa = math.cos(alpha), math.sin(alpha)
        T = T @ np.array([
            [ct,      -st,       0.0,  a],
            [st * ca,  ct * ca, -sa,  -sa * d],
            [st * sa,  ct * sa,  ca,   ca * d],
            [0.0,      0.0,      0.0,  1.0],
        ])
    return T


def pose_to_xyz_rpy(T: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    """(x, y, z, roll, pitch, yaw) with the fixed-axis XYZ convention the arm uses."""
    x, y, z = T[:3, 3]
    # R = Rz @ Ry @ Rx
    pitch = math.asin(max(-1.0, min(1.0, -T[2, 0])))
    if abs(math.cos(pitch)) < 1e-8:            # gimbal lock
        roll, yaw = math.atan2(-T[1, 2], T[1, 1]), 0.0
    else:
        roll = math.atan2(T[2, 1], T[2, 2])
        yaw = math.atan2(T[1, 0], T[0, 0])
    return float(x), float(y), float(z), float(roll), float(pitch), float(yaw)


def ik(target_T: np.ndarray, seed: Sequence[float],
       pos_tol: float = 0.003, rot_tol_deg: float = 2.0) -> Optional[np.ndarray]:
    """Joint angles reaching ``target_T``, or None if no solution converges.

    Numeric least-squares against the arm's own MDH model, seeded from a known
    configuration and bounded by the joint limits -- intended for targets NEAR
    the seed (the calibration lattice), not for global IK. A pose the solver
    cannot reach within tolerance returns None instead of a best-effort guess,
    because a "close" pose that the arm's controller then refuses is exactly
    the silent-no-motion failure this module exists to prevent.
    """
    try:
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
    except ImportError:
        return None

    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])
    tp = target_T[:3, 3]
    tR = target_T[:3, :3]

    def resid(q):
        T = fk(q)
        dp = T[:3, 3] - tp
        rv = Rotation.from_matrix(T[:3, :3].T @ tR).as_rotvec()
        return np.concatenate([dp, 0.2 * rv])   # ~0.2 m per rad: balanced units

    q0 = np.clip(np.asarray(seed, dtype=float), lo + 1e-6, hi - 1e-6)
    r = least_squares(resid, q0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12,
                      max_nfev=400)
    T = fk(r.x)
    e_pos = float(np.linalg.norm(T[:3, 3] - tp))
    e_rot = math.degrees(float(np.linalg.norm(
        Rotation.from_matrix(T[:3, :3].T @ tR).as_rotvec())))
    if e_pos > pos_tol or e_rot > rot_tol_deg:
        return None
    return r.x


def within_joint_limits(joint_angles: Sequence[float], margin_deg: float = 0.0) -> bool:
    m = math.radians(margin_deg)
    return all(lo + m <= q <= hi - m
               for q, (lo, hi) in zip(joint_angles, JOINT_LIMITS))


def read_joint_angles(can_port: str = "can0",
                      timeout: float = 5.0) -> Optional[np.ndarray]:
    """Current joint angles [rad] straight from the arm. Read-only.

    Returns None if the arm cannot be reached, so callers can print a useful
    message instead of dying on a traceback.
    """
    import time
    try:
        from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
    except ImportError:
        return None
    arm = None
    try:
        cfg = create_agx_arm_config(robot=ArmModel.PIPER, comm="can", channel=can_port)
        arm = AgxArmFactory.create_arm(cfg)
        arm.connect()
        deadline = time.time() + timeout
        while time.time() < deadline:
            js = arm.get_joint_angles()
            if js is not None and js.hz > 0:
                return np.array(list(js.msg), dtype=float)
            time.sleep(0.05)
        return None
    except Exception:  # noqa: BLE001 - socketcan errors are not one type
        return None
    finally:
        if arm is not None:
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001
                pass

"""Eye-in-Hand calibration solver (pure Python, ROS-free).

Wraps ``cv2.calibrateHandEye`` with explicit, documented transform directions
and input validation so a wrong-way transform cannot silently corrupt a result.

OpenCV mapping (Eye-in-Hand)
----------------------------
``cv2.calibrateHandEye`` is called as::

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,   # base_T_gripper   (list per sample)
        R_target2cam,  t_target2cam,      # camera_T_target  (list per sample)
        method)

* ``R_gripper2base``/``t_gripper2base`` == our **base_T_gripper** (gripper in base).
* ``R_target2cam``/``t_target2cam``   == our **camera_T_target** (target in camera).
* returned ``cam2gripper``             == our **gripper_T_camera** (camera in gripper),
  which is exactly the static TF we publish: parent=gripper, child=camera.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence

import numpy as np

from . import transform_utils as tu

# Imported lazily inside solve() so importing this module never requires OpenCV
# (keeps the pure-math unit tests importable even if cv2 is unavailable).

METHODS = ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS")
DEFAULT_METHOD = "PARK"
DEFAULT_MIN_SAMPLES = 10


def _cv_method(name: str):
    import cv2
    table = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    key = (name or DEFAULT_METHOD).upper()
    if key not in table:
        raise ValueError(f"Unknown calibration method '{name}'. Choose from {METHODS}.")
    return key, table[key]


@dataclass
class SolveResult:
    gripper_T_camera: np.ndarray                 # 4x4 result
    method: str
    sample_count: int
    motion: Dict[str, float] = field(default_factory=dict)  # diversity metrics
    warnings: List[str] = field(default_factory=list)

    @property
    def translation(self) -> np.ndarray:
        return tu.decompose_transform(self.gripper_T_camera)[1]

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        return tu.matrix_to_quaternion(self.gripper_T_camera)


def check_inputs(base_T_gripper: Sequence[np.ndarray],
                 camera_T_target: Sequence[np.ndarray],
                 min_samples: int = DEFAULT_MIN_SAMPLES) -> List[str]:
    """Validate solver inputs. Returns a list of fatal error strings (empty = ok)."""
    errors: List[str] = []
    n = len(base_T_gripper)
    if n != len(camera_T_target):
        errors.append(
            f"Sample count mismatch: {n} base_T_gripper vs "
            f"{len(camera_T_target)} camera_T_target.")
        return errors
    if n < min_samples:
        errors.append(f"Not enough samples: {n} < required {min_samples}.")
    for i, (bg, ct) in enumerate(zip(base_T_gripper, camera_T_target)):
        for label, T in (("base_T_gripper", bg), ("camera_T_target", ct)):
            T = np.asarray(T, dtype=float)
            if T.shape != (4, 4):
                errors.append(f"Sample {i} {label}: shape {T.shape} != (4,4).")
                continue
            if not np.all(np.isfinite(T)):
                errors.append(f"Sample {i} {label}: contains NaN/Inf.")
                continue
            R, t = tu.decompose_transform(T)
            if not tu.is_valid_rotation(R, tol=1e-3):
                errors.append(f"Sample {i} {label}: rotation not orthonormal (det="
                              f"{np.linalg.det(R):.4f}).")
            if np.linalg.norm(t) > 10.0:
                errors.append(f"Sample {i} {label}: |t|={np.linalg.norm(t):.2f} m "
                              "unreasonably large (expected meters).")
    return errors


def motion_diversity(base_T_gripper: Sequence[np.ndarray]) -> Dict[str, float]:
    """Rotation/translation spread across the gripper motions (for diagnostics).

    Reports the max pairwise rotation angle (deg) and translation spread (m).
    Hand-eye needs rotation about >= 2 distinct axes; a tiny value here means
    the poses are essentially the same orientation and the solve is ill-posed.
    """
    n = len(base_T_gripper)
    max_rot = 0.0
    max_trans = 0.0
    axes = []
    for i in range(n):
        for j in range(i + 1, n):
            Ri = tu.decompose_transform(base_T_gripper[i])[0]
            Rj = tu.decompose_transform(base_T_gripper[j])[0]
            ang = tu.rotation_angle(Ri.T @ Rj)
            max_rot = max(max_rot, ang)
            max_trans = max(max_trans, tu.translation_distance(
                base_T_gripper[i], base_T_gripper[j]))
    # rough count of distinct rotation axes via relative rotations to sample 0
    if n >= 2:
        R0 = tu.decompose_transform(base_T_gripper[0])[0]
        for k in range(1, n):
            Rk = tu.decompose_transform(base_T_gripper[k])[0]
            Rrel = R0.T @ Rk
            ang = tu.rotation_angle(Rrel)
            if ang > np.deg2rad(5.0):
                # axis of rotation from skew part
                axis = np.array([Rrel[2, 1] - Rrel[1, 2],
                                 Rrel[0, 2] - Rrel[2, 0],
                                 Rrel[1, 0] - Rrel[0, 1]])
                if np.linalg.norm(axis) > 1e-6:
                    axes.append(axis / np.linalg.norm(axis))
    distinct_axes = _count_distinct_axes(axes)
    return {
        "max_rotation_deg": float(np.rad2deg(max_rot)),
        "max_translation_m": float(max_trans),
        "distinct_rotation_axes": float(distinct_axes),
    }


def _count_distinct_axes(axes: List[np.ndarray], angle_tol_deg: float = 15.0) -> int:
    reps: List[np.ndarray] = []
    tol = np.cos(np.deg2rad(angle_tol_deg))
    for a in axes:
        if not any(abs(float(np.dot(a, r))) > tol for r in reps):
            reps.append(a)
    return len(reps)


def solve(base_T_gripper: Sequence[np.ndarray],
          camera_T_target: Sequence[np.ndarray],
          method: str = DEFAULT_METHOD,
          min_samples: int = DEFAULT_MIN_SAMPLES,
          strict: bool = True) -> SolveResult:
    """Compute gripper_T_camera (Eye-in-Hand).

    :param base_T_gripper: list of 4x4 base_T_gripper, one per sample.
    :param camera_T_target: list of 4x4 camera_T_target, one per sample.
    :raises ValueError: if inputs fail validation (when ``strict``).
    """
    import cv2

    errors = check_inputs(base_T_gripper, camera_T_target, min_samples)
    if errors and strict:
        raise ValueError("Calibration input validation failed:\n  - "
                         + "\n  - ".join(errors))

    method_key, cv_flag = _cv_method(method)

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for bg, ct in zip(base_T_gripper, camera_T_target):
        Rg, tg = tu.decompose_transform(bg)
        Rc, tc = tu.decompose_transform(ct)
        R_g2b.append(Rg)
        t_g2b.append(tg.reshape(3, 1))
        R_t2c.append(Rc)
        t_t2c.append(tc.reshape(3, 1))

    R_c2g, t_c2g = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=cv_flag)

    gripper_T_camera = tu.make_transform(np.asarray(R_c2g), np.asarray(t_c2g).reshape(3))

    warnings: List[str] = list(errors) if not strict else []
    motion = motion_diversity(base_T_gripper)
    if motion["distinct_rotation_axes"] < 2:
        warnings.append("Low rotation diversity: fewer than 2 distinct rotation axes "
                        "detected; result may be unreliable.")
    if motion["max_rotation_deg"] < 20.0:
        warnings.append(f"Small max rotation ({motion['max_rotation_deg']:.1f} deg) "
                        "between poses; add more orientation variety.")

    return SolveResult(
        gripper_T_camera=gripper_T_camera,
        method=method_key,
        sample_count=len(base_T_gripper),
        motion=motion,
        warnings=warnings,
    )

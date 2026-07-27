"""Validation of a hand-eye result via the closed-loop target consistency check.

Idea
----
The target is physically fixed in the robot base frame, so for every sample::

    base_T_target_i = base_T_gripper_i @ gripper_T_camera @ camera_T_target_i

must yield (nearly) the same transform for all i.  The spread of these
``base_T_target_i`` estimates is a direct, unit-meaningful measure of
calibration quality (translation in meters, rotation in degrees).

This module is ROS-free and pure numpy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import transform_utils as tu


@dataclass
class ValidationResult:
    sample_count: int
    translation_rms_m: float
    translation_max_m: float
    rotation_rms_deg: float
    rotation_max_deg: float
    per_sample_translation_m: List[float] = field(default_factory=list)
    per_sample_rotation_deg: List[float] = field(default_factory=list)
    outlier_indices: List[int] = field(default_factory=list)
    base_T_target_mean: Optional[np.ndarray] = None
    status: str = "UNKNOWN"       # SUCCESS | WARNING | FAILED
    messages: List[str] = field(default_factory=list)


def compute_base_T_target(base_T_gripper: Sequence[np.ndarray],
                          gripper_T_camera: np.ndarray,
                          camera_T_target: Sequence[np.ndarray]) -> List[np.ndarray]:
    out = []
    for bg, ct in zip(base_T_gripper, camera_T_target):
        out.append(tu.compose_transform(bg, gripper_T_camera, ct))
    return out


def _residuals(estimates: Sequence[np.ndarray],
               reference: np.ndarray) -> Tuple[List[float], List[float]]:
    trans, rot = [], []
    for T in estimates:
        trans.append(tu.translation_distance(T, reference))
        rot.append(float(np.rad2deg(tu.rotation_angle_between(reference, T))))
    return trans, rot


def validate(base_T_gripper: Sequence[np.ndarray],
             gripper_T_camera: np.ndarray,
             camera_T_target: Sequence[np.ndarray],
             max_translation_rms_m: float = 0.01,
             max_rotation_rms_deg: float = 1.0) -> ValidationResult:
    """Closed-loop validation.  Thresholds decide SUCCESS/WARNING/FAILED."""
    estimates = compute_base_T_target(base_T_gripper, gripper_T_camera, camera_T_target)
    mean = tu.average_transforms(estimates)
    trans, rot = _residuals(estimates, mean)

    n = len(estimates)
    t_rms = float(np.sqrt(np.mean(np.square(trans)))) if n else 0.0
    r_rms = float(np.sqrt(np.mean(np.square(rot)))) if n else 0.0
    t_max = float(np.max(trans)) if n else 0.0
    r_max = float(np.max(rot)) if n else 0.0

    # outliers = samples > 3 sigma on either metric
    outliers = _flag_outliers(trans, rot)

    messages: List[str] = []
    status = "SUCCESS"
    if t_rms > max_translation_rms_m:
        status = "WARNING"
        messages.append(f"translation RMS {t_rms*1000:.2f} mm exceeds "
                        f"{max_translation_rms_m*1000:.2f} mm.")
    if r_rms > max_rotation_rms_deg:
        status = "WARNING"
        messages.append(f"rotation RMS {r_rms:.3f} deg exceeds "
                        f"{max_rotation_rms_deg:.3f} deg.")
    # Gross failure if it's an order of magnitude beyond threshold.
    if t_rms > 5.0 * max_translation_rms_m or r_rms > 5.0 * max_rotation_rms_deg:
        status = "FAILED"

    return ValidationResult(
        sample_count=n,
        translation_rms_m=t_rms,
        translation_max_m=t_max,
        rotation_rms_deg=r_rms,
        rotation_max_deg=r_max,
        per_sample_translation_m=trans,
        per_sample_rotation_deg=rot,
        outlier_indices=outliers,
        base_T_target_mean=mean,
        status=status,
        messages=messages,
    )


def _flag_outliers(trans: Sequence[float], rot: Sequence[float],
                   sigma: float = 3.0) -> List[int]:
    idx = set()
    for values in (trans, rot):
        v = np.asarray(values, dtype=float)
        if len(v) < 4:
            continue
        mu, sd = float(np.mean(v)), float(np.std(v))
        if sd < 1e-12:
            continue
        for i, x in enumerate(v):
            if abs(x - mu) > sigma * sd:
                idx.add(i)
    return sorted(idx)


def remove_outliers(base_T_gripper: List[np.ndarray],
                    camera_T_target: List[np.ndarray],
                    solve_fn,
                    max_translation_rms_m: float = 0.01,
                    max_rotation_rms_deg: float = 1.0,
                    max_removals: int = 3,
                    min_samples: int = 8):
    """Iteratively drop the single worst sample and re-solve.

    ``solve_fn(base_list, camera_list) -> gripper_T_camera (4x4)``.
    Bounded by ``max_removals`` and ``min_samples`` (never removes below it).
    Returns (best_gripper_T_camera, kept_base, kept_camera, removed_indices, history).
    """
    base = list(base_T_gripper)
    cam = list(camera_T_target)
    original_index = list(range(len(base)))
    removed: List[int] = []
    history = []

    gtc = solve_fn(base, cam)
    result = validate(base, gtc, cam, max_translation_rms_m, max_rotation_rms_deg)
    history.append(("initial", result.translation_rms_m, result.rotation_rms_deg))

    for _ in range(max_removals):
        if result.status == "SUCCESS" or len(base) <= min_samples:
            break
        # worst sample = largest combined normalized residual
        tnorm = np.asarray(result.per_sample_translation_m) / max(max_translation_rms_m, 1e-9)
        rnorm = np.asarray(result.per_sample_rotation_deg) / max(max_rotation_rms_deg, 1e-9)
        worst = int(np.argmax(tnorm + rnorm))
        removed.append(original_index.pop(worst))
        base.pop(worst)
        cam.pop(worst)
        gtc = solve_fn(base, cam)
        result = validate(base, gtc, cam, max_translation_rms_m, max_rotation_rms_deg)
        history.append((f"removed_orig_idx_{removed[-1]}",
                        result.translation_rms_m, result.rotation_rms_deg))

    return gtc, base, cam, removed, history

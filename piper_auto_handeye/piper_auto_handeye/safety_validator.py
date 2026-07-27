"""Motion safety checks for the Piper control adapter (ROS-free, pure numpy).

Every physical move must pass ``SafetyValidator.check_goal`` first. The checks
are intentionally conservative and reject on any doubt (NaN, out-of-bounds,
oversized step). The adapter refuses to send a command when this returns a
non-empty reason list.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np


@dataclass
class SafetyLimits:
    workspace_min: Sequence[float]
    workspace_max: Sequence[float]
    max_step_distance: float
    max_speed_fraction: float


class SafetyValidator:
    def __init__(self, limits: SafetyLimits):
        self.limits = limits

    def check_position(self, position: Sequence[float]) -> List[str]:
        reasons: List[str] = []
        p = np.asarray(position, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            reasons.append(f"target position not finite: {position}")
            return reasons
        lo = np.asarray(self.limits.workspace_min, dtype=float)
        hi = np.asarray(self.limits.workspace_max, dtype=float)
        for i, ax in enumerate("xyz"):
            if p[i] < lo[i] or p[i] > hi[i]:
                reasons.append(
                    f"{ax}={p[i]:.3f} m outside workspace "
                    f"[{lo[i]:.3f}, {hi[i]:.3f}]")
        return reasons

    def check_goal(self,
                   target_position: Sequence[float],
                   target_quaternion_xyzw: Optional[Sequence[float]],
                   current_position: Optional[Sequence[float]],
                   speed_fraction: float) -> List[str]:
        """Return a list of reasons the goal is unsafe. Empty list == safe."""
        reasons = list(self.check_position(target_position))

        if target_quaternion_xyzw is not None:
            q = np.asarray(target_quaternion_xyzw, dtype=float)
            if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-6:
                reasons.append(f"target orientation invalid: {target_quaternion_xyzw}")

        if not np.isfinite(speed_fraction) or speed_fraction <= 0:
            reasons.append(f"speed fraction must be > 0, got {speed_fraction}")
        elif speed_fraction > self.limits.max_speed_fraction + 1e-9:
            reasons.append(
                f"speed fraction {speed_fraction:.2f} exceeds max "
                f"{self.limits.max_speed_fraction:.2f}")

        if current_position is not None:
            c = np.asarray(current_position, dtype=float)
            p = np.asarray(target_position, dtype=float)
            if np.all(np.isfinite(c)) and np.all(np.isfinite(p)):
                step = float(np.linalg.norm(p - c))
                if step > self.limits.max_step_distance:
                    reasons.append(
                        f"single-step distance {step:.3f} m exceeds max "
                        f"{self.limits.max_step_distance:.3f} m")
        return reasons

    def clamp_speed(self, speed_fraction: float, default: float) -> float:
        if not np.isfinite(speed_fraction) or speed_fraction <= 0:
            speed_fraction = default
        return float(min(speed_fraction, self.limits.max_speed_fraction))

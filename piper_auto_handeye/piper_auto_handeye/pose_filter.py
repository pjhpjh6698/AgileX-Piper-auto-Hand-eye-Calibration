"""Temporal filtering of marker poses over consecutive frames (ROS-free).

Keeps a sliding window of recent 4x4 camera_T_target poses and produces a
filtered pose (median translation + quaternion average) plus a stability score.
Rejects sudden jumps so a single bad detection cannot poison a sample.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from . import transform_utils as tu


class PoseFilter:
    def __init__(self,
                 window: int = 5,
                 max_translation_jump_m: float = 0.05,
                 max_rotation_jump_deg: float = 15.0):
        self.window = max(1, int(window))
        self.max_translation_jump_m = float(max_translation_jump_m)
        self.max_rotation_jump_deg = float(max_rotation_jump_deg)
        self._buf: Deque[np.ndarray] = deque(maxlen=self.window)

    def reset(self) -> None:
        self._buf.clear()

    def add(self, T: np.ndarray) -> Tuple[Optional[np.ndarray], bool]:
        """Add a raw pose. Returns (filtered_pose_or_None, accepted).

        A pose is rejected (and not stored) if it jumps too far from the current
        filtered estimate; the filtered estimate is otherwise returned.
        """
        T = np.asarray(T, dtype=float)
        if self._buf:
            ref = self._filtered()
            dt = tu.translation_distance(ref, T)
            dr = np.rad2deg(tu.rotation_angle_between(ref, T))
            if dt > self.max_translation_jump_m or dr > self.max_rotation_jump_deg:
                return self._filtered(), False
        self._buf.append(T)
        return self._filtered(), True

    def _filtered(self) -> Optional[np.ndarray]:
        if not self._buf:
            return None
        transs = np.array([tu.decompose_transform(T)[1] for T in self._buf])
        med_t = np.median(transs, axis=0)
        quats = [tu.matrix_to_quaternion(tu.decompose_transform(T)[0]) for T in self._buf]
        R = tu.quaternion_to_matrix(tu.quaternion_average(quats))
        return tu.make_transform(R, med_t)

    def filtered(self) -> Optional[np.ndarray]:
        return self._filtered()

    def stability_score(self) -> float:
        """0..1; 1 = perfectly stable window. Based on translation & rotation spread."""
        if len(self._buf) < 2:
            return 0.0
        ref = self._filtered()
        t_dev = [tu.translation_distance(ref, T) for T in self._buf]
        r_dev = [np.rad2deg(tu.rotation_angle_between(ref, T)) for T in self._buf]
        t_score = max(0.0, 1.0 - float(np.mean(t_dev)) / self.max_translation_jump_m)
        r_score = max(0.0, 1.0 - float(np.mean(r_dev)) / self.max_rotation_jump_deg)
        return float(min(t_score, r_score))

    @property
    def count(self) -> int:
        return len(self._buf)

    def is_full(self) -> bool:
        return len(self._buf) >= self.window

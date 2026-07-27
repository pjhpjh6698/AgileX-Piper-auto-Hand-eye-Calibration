"""Read/write calibration results and raw samples as YAML (ROS-free).

Result file schema matches the README/spec. Default output directory is the
user's ROS state dir (~/.ros/piper_auto_handeye), never inside the package.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import yaml

from . import transform_utils as tu


def default_output_dir() -> str:
    return os.path.expanduser("~/.ros/piper_auto_handeye")


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def build_result_dict(gripper_T_camera: np.ndarray,
                      parent_frame: str,
                      child_frame: str,
                      method: str,
                      timestamp: str,
                      validation: Dict,
                      source: Dict,
                      calibration_type: str = "eye_in_hand") -> Dict:
    R, t = tu.decompose_transform(gripper_T_camera)
    q = tu.matrix_to_quaternion(R)
    return {
        "calibration": {
            "type": calibration_type,
            "parent_frame": parent_frame,
            "child_frame": child_frame,
            "method": method,
            "timestamp": timestamp,
            "translation": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
            "quaternion": {"x": float(q[0]), "y": float(q[1]),
                           "z": float(q[2]), "w": float(q[3])},
            "matrix": [[float(v) for v in row] for row in gripper_T_camera],
            "validation": validation,
            "source": source,
        }
    }


def save_result(result_dict: Dict, path: str) -> str:
    ensure_dir(os.path.dirname(os.path.abspath(path)))
    with open(path, "w") as f:
        yaml.safe_dump(result_dict, f, default_flow_style=False, sort_keys=False)
    return path


def load_result(path: str) -> Dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def result_to_transform(result_dict: Dict):
    """Return (gripper_T_camera 4x4, parent_frame, child_frame) from a loaded dict."""
    cal = result_dict["calibration"]
    t = cal["translation"]
    q = cal["quaternion"]
    T = tu.transform_from_quaternion([t["x"], t["y"], t["z"]],
                                     [q["x"], q["y"], q["z"], q["w"]])
    return T, cal["parent_frame"], cal["child_frame"]


def save_samples(samples: List[Dict], path: str) -> str:
    """Persist raw sample metadata (poses as flat lists + rejection reasons)."""
    ensure_dir(os.path.dirname(os.path.abspath(path)))
    with open(path, "w") as f:
        yaml.safe_dump({"samples": samples}, f, default_flow_style=False, sort_keys=False)
    return path


def sample_to_dict(pose_index: int,
                   base_T_gripper: np.ndarray,
                   camera_T_target: np.ndarray,
                   metadata: Dict) -> Dict:
    def flat(T):
        return [[float(v) for v in row] for row in T]
    d = {
        "pose_index": int(pose_index),
        "base_T_gripper": flat(base_T_gripper),
        "camera_T_target": flat(camera_T_target),
    }
    d.update(metadata)
    return d

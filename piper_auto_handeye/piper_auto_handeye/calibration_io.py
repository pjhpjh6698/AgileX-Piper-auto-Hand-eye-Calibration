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


# One well-known file, overwritten on every save, in the easy_handeye2 spirit:
# just the frames and the transform -- what a consumer actually needs.
RESULT_FILENAME = "handeye_calibration.yaml"


def default_result_path() -> str:
    return os.path.join(default_output_dir(), RESULT_FILENAME)


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
    """Minimal, easy_handeye2-style result: the frames and the transform.

    Deliberately no 4x4 matrix, no sample dump, no metadata blob -- one small
    file a consumer can read at a glance. The two RMS numbers stay because they
    answer the only follow-up question that matters: can this be trusted?
    """
    R, t = tu.decompose_transform(gripper_T_camera)
    q = tu.matrix_to_quaternion(R)
    return {
        "parameters": {
            "calibration_type": calibration_type,
            "robot_effector_frame": parent_frame,
            "tracking_camera_frame": child_frame,
            "method": method,
            "timestamp": timestamp,
            "samples": int(validation.get("sample_count", 0)),
            "translation_rms_m": float(validation.get("translation_rms_m", 0.0)),
            "rotation_rms_deg": float(validation.get("rotation_rms_deg", 0.0)),
        },
        "transform": {
            "translation": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
            "rotation": {"x": float(q[0]), "y": float(q[1]),
                         "z": float(q[2]), "w": float(q[3])},
        },
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
    """Return (gripper_T_camera 4x4, parent_frame, child_frame) from a loaded dict.

    Reads both formats: the current minimal one (parameters/transform) and the
    legacy verbose one (calibration/...), so results saved before the format
    change keep loading.
    """
    if "parameters" in result_dict and "transform" in result_dict:
        prm = result_dict["parameters"]
        tr = result_dict["transform"]
        t = tr["translation"]
        q = tr["rotation"]
        T = tu.transform_from_quaternion([t["x"], t["y"], t["z"]],
                                         [q["x"], q["y"], q["z"], q["w"]])
        return T, prm["robot_effector_frame"], prm["tracking_camera_frame"]
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

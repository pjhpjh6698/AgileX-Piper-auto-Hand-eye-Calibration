"""Hardware-free unit tests for the calibration math core.

Run with:  pytest -q src/autoCali/piper_auto_handeye/test/test_calibration_math.py

These tests import the pure-python modules directly (no ROS, no rclpy).
The synthetic hand-eye recovery test uses the exact relation:

    base_T_target = base_T_gripper @ gripper_T_camera @ camera_T_target
    =>  camera_T_target = inv(gripper_T_camera) @ inv(base_T_gripper) @ base_T_target
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from piper_auto_handeye import transform_utils as tu           # noqa: E402
from piper_auto_handeye import calibration_solver as solver     # noqa: E402
from piper_auto_handeye import calibration_validator as validator  # noqa: E402
from piper_auto_handeye.pose_filter import PoseFilter           # noqa: E402

cv2 = pytest.importorskip("cv2", reason="OpenCV required for solver tests")


# --------------------------------------------------------------------------- #
# deterministic rotation helper (no scipy)
# --------------------------------------------------------------------------- #
def rot(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = np.cos(angle), np.sin(angle), 1 - np.cos(angle)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def T(axis, angle, t):
    return tu.make_transform(rot(axis, angle), t)


# --------------------------------------------------------------------------- #
# transform_utils
# --------------------------------------------------------------------------- #
def test_quaternion_matrix_roundtrip():
    for axis, ang in [([1, 0, 0], 0.3), ([0, 1, 0], -1.2), ([1, 1, 1], 2.0)]:
        R = rot(axis, ang)
        q = tu.matrix_to_quaternion(R)
        R2 = tu.quaternion_to_matrix(q)
        assert np.allclose(R, R2, atol=1e-9)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9


def test_quaternion_order_is_ros_xyzw():
    # 90 deg about z -> q = (0,0,sin45,cos45) in (x,y,z,w)
    q = tu.matrix_to_quaternion(rot([0, 0, 1], np.pi / 2))
    assert np.allclose(q, [0, 0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-9)


def test_wxyz_conversion():
    q_xyzw = tu.normalize_quaternion([0.1, 0.2, 0.3, 0.9])
    q_wxyz = tu.quaternion_ros_to_wxyz(q_xyzw)
    assert np.allclose(q_wxyz, [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    assert np.allclose(tu.quaternion_wxyz_to_ros(q_wxyz), q_xyzw)


def test_invert_and_compose():
    A = T([1, 0, 0], 0.5, [0.1, 0.2, 0.3])
    assert np.allclose(tu.compose_transform(A, tu.invert_transform(A)), np.eye(4), atol=1e-12)
    B = T([0, 1, 0], -0.4, [0.05, -0.1, 0.2])
    assert np.allclose(tu.compose_transform(A, B),  A @ B, atol=1e-12)


def test_pose_msg_roundtrip():
    pytest.importorskip("geometry_msgs", reason="ROS geometry_msgs not sourced")
    A = T([1, 1, 0], 0.7, [0.3, -0.2, 0.9])
    msg = tu.matrix_to_pose_msg(A)
    A2 = tu.pose_msg_to_matrix(msg)
    assert np.allclose(A, A2, atol=1e-9)


def test_average_transforms():
    base = T([0, 0, 1], 0.2, [1.0, 2.0, 3.0])
    perturbed = [tu.compose_transform(base, T([1, 0, 0], d, [0, 0, 0]))
                 for d in (-0.01, 0.0, 0.01)]
    avg = tu.average_transforms(perturbed)
    assert tu.translation_distance(avg, base) < 1e-6
    assert np.rad2deg(tu.rotation_angle_between(avg, base)) < 0.5


# --------------------------------------------------------------------------- #
# synthetic hand-eye recovery
# --------------------------------------------------------------------------- #
def _make_dataset(gripper_T_camera, base_T_target, base_T_grippers, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    cam_T_target = []
    for bg in base_T_grippers:
        ct = tu.compose_transform(
            tu.invert_transform(gripper_T_camera),
            tu.invert_transform(bg),
            base_T_target,
        )
        if noise > 0:
            dt = rng.normal(0, noise, 3)
            dang = rng.normal(0, noise * 5, 3)  # rad-ish small
            ct = tu.compose_transform(ct, tu.make_transform(
                rot([1, 1, 1], np.linalg.norm(dang)), dt))
        cam_T_target.append(ct)
    return cam_T_target


def _pose_set():
    gtc = T([0.3, 0.6, 0.2], 0.9, [0.05, -0.03, 0.08])       # ground-truth gripper_T_camera
    btt = T([0, 0, 1], 0.4, [0.6, 0.1, 0.2])                 # fixed target in base
    # diverse gripper poses: rotate about multiple axes + translate
    base_T_grippers = []
    angles = [(-0.4, 0.3, 0.1), (0.5, -0.2, 0.3), (0.1, 0.6, -0.3),
              (-0.3, -0.4, 0.5), (0.6, 0.2, -0.2), (0.2, -0.5, 0.4),
              (-0.5, 0.4, 0.2), (0.3, 0.3, 0.3), (-0.2, 0.5, -0.4),
              (0.4, -0.3, -0.2), (0.1, -0.6, 0.3), (-0.4, 0.2, 0.5)]
    for i, (a, b, c) in enumerate(angles):
        R = rot([1, 0, 0], a) @ rot([0, 1, 0], b) @ rot([0, 0, 1], c)
        t = [0.3 + 0.05 * np.cos(i), 0.02 * i - 0.1, 0.35 + 0.03 * np.sin(i)]
        base_T_grippers.append(tu.make_transform(R, t))
    return gtc, btt, base_T_grippers


@pytest.mark.parametrize("method", solver.METHODS)
def test_synthetic_recovery_noise_free(method):
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs, noise=0.0)
    res = solver.solve(bgs, cam, method=method, min_samples=8)
    err_t = tu.translation_distance(res.gripper_T_camera, gtc)
    err_r = np.rad2deg(tu.rotation_angle_between(res.gripper_T_camera, gtc))
    assert err_t < 1e-4, f"{method}: translation error {err_t}"
    assert err_r < 0.05, f"{method}: rotation error {err_r}"


def test_synthetic_recovery_with_noise():
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs, noise=0.002, seed=3)
    res = solver.solve(bgs, cam, method="PARK", min_samples=8)
    err_t = tu.translation_distance(res.gripper_T_camera, gtc)
    err_r = np.rad2deg(tu.rotation_angle_between(res.gripper_T_camera, gtc))
    assert err_t < 0.02, f"translation error {err_t}"
    assert err_r < 3.0, f"rotation error {err_r}"


def test_validation_consistency_noise_free():
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs, noise=0.0)
    vr = validator.validate(bgs, gtc, cam)
    assert vr.translation_rms_m < 1e-6
    assert vr.rotation_rms_deg < 1e-4
    assert vr.status == "SUCCESS"


# --------------------------------------------------------------------------- #
# input guards
# --------------------------------------------------------------------------- #
def test_solver_rejects_mismatched_lengths():
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs)
    with pytest.raises(ValueError):
        solver.solve(bgs, cam[:-1], method="TSAI")


def test_solver_rejects_too_few_samples():
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs)
    with pytest.raises(ValueError):
        solver.solve(bgs[:3], cam[:3], method="TSAI", min_samples=10)


def test_solver_rejects_nan():
    gtc, btt, bgs = _pose_set()
    cam = _make_dataset(gtc, btt, bgs)
    bad = bgs[:]
    bad[0] = bad[0].copy()
    bad[0][0, 3] = np.nan
    with pytest.raises(ValueError):
        solver.solve(bad, cam, method="TSAI", min_samples=8)


def test_bad_rotation_detected():
    bad = np.eye(4)
    bad[:3, :3] = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 2]])  # not a rotation
    assert not tu.is_valid_rotation(bad[:3, :3])


# --------------------------------------------------------------------------- #
# pose filter
# --------------------------------------------------------------------------- #
def test_pose_filter_rejects_jump():
    f = PoseFilter(window=5, max_translation_jump_m=0.05, max_rotation_jump_deg=15)
    base = tu.make_transform(np.eye(3), [0.1, 0.1, 0.5])
    for _ in range(5):
        _, ok = f.add(base)
        assert ok
    jump = tu.make_transform(np.eye(3), [0.5, 0.1, 0.5])  # 0.4 m jump
    _, ok = f.add(jump)
    assert not ok
    assert f.stability_score() > 0.9


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))

#!/usr/bin/env python3
"""Tests for the marker-recovery geometry.

These cover the part of the occlusion workaround that can be wrong silently:
if a recovery nudge produced a non-rigid transform, or moved the wrist so far
that the pose no longer resembles the one the pose set was designed around, the
calibration would still "succeed" and quietly return a worse result.

Pure numpy -- no ROS node is constructed, so this runs in CI without hardware.
"""

import math

import numpy as np
import pytest

from piper_auto_handeye import transform_utils as tu
from piper_auto_handeye.handeye_calibration_node import HandeyeCalibrationNode


TRANS_STEP = 0.05
ROT_STEP_DEG = 12.0


class _Stub:
    """Just enough of the node to exercise the geometry helpers."""

    recovery_trans_step = TRANS_STEP
    recovery_rot_step_deg = ROT_STEP_DEG

    _recovery_offsets = HandeyeCalibrationNode._recovery_offsets
    _apply_offset = staticmethod(HandeyeCalibrationNode._apply_offset)


def _nominal():
    """`home_center` from config/calibration_poses.yaml."""
    return tu.make_transform(tu.euler_to_matrix(-3.14, 0.0, 0.0), [0.35, 0.0, 0.35])


def _recovered():
    stub = _Stub()
    return [(name, stub._apply_offset(_nominal(), off))
            for off, name in stub._recovery_offsets()]


def test_offsets_are_non_empty_and_named():
    offsets = _Stub()._recovery_offsets()
    assert len(offsets) >= 4, "too few recovery options to work around occlusion"
    for off, name in offsets:
        assert len(off) == 6, "offset must be (dx,dy,dz,droll,dpitch,dyaw)"
        assert name, "every recovery step needs a label for the operator log"


def test_recovery_poses_are_rigid_transforms():
    for name, T in _recovered():
        R, _ = tu.decompose_transform(T)
        assert tu.is_valid_rotation(R), f"'{name}' produced a non-orthonormal rotation"
        assert T.shape == (4, 4)
        assert np.allclose(T[3], [0, 0, 0, 1])


def test_recovery_stays_near_the_nominal_pose():
    """A nudge must not become a different pose.

    The pose set is chosen for orientation diversity; if recovery swung the
    wrist far enough to change which orientation the pose represents, it would
    defeat the point of having a designed pose set.
    """
    nominal = _nominal()
    for name, T in _recovered():
        d_pos = tu.translation_distance(nominal, T)
        d_rot = math.degrees(tu.rotation_angle_between(nominal, T))
        assert d_pos <= 3 * TRANS_STEP + 1e-9, f"'{name}' translated {d_pos:.3f} m"
        assert d_rot <= 2 * ROT_STEP_DEG + 1e-6, f"'{name}' rotated {d_rot:.1f} deg"


def test_recovery_poses_are_distinct():
    """Repeating the same nudge would waste a retry slot on a known-bad view."""
    poses = _recovered()
    for i, (name_a, a) in enumerate(poses):
        for name_b, b in poses[i + 1:]:
            same = (tu.translation_distance(a, b) < 1e-6
                    and math.degrees(tu.rotation_angle_between(a, b)) < 1e-6)
            assert not same, f"'{name_a}' and '{name_b}' are the same pose"


def test_first_recovery_only_backs_off():
    """The cheapest fix is tried first: more standoff, same orientation.

    Widening the field of view usually clears the gripper on its own, and it
    preserves the orientation the pose was chosen for, so it must come before
    any rotation.
    """
    stub = _Stub()
    (dx, dy, dz, droll, dpitch, dyaw), name = stub._recovery_offsets()[0]
    assert (droll, dpitch, dyaw) == (0.0, 0.0, 0.0), "first step must not rotate"
    assert dz > 0.0, "first step must increase standoff"
    assert (dx, dy) == (0.0, 0.0)


def test_zero_offset_is_identity():
    nominal = _nominal()
    same = _Stub._apply_offset(nominal, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert np.allclose(nominal, same, atol=1e-12)


@pytest.mark.parametrize("step", [0.0, 0.02, 0.05, 0.15])
def test_translation_step_scales(step):
    stub = _Stub()
    stub.recovery_trans_step = step
    (_, _, dz, _, _, _), _ = stub._recovery_offsets()[0]
    assert dz == pytest.approx(step)

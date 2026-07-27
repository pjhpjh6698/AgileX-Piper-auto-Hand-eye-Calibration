#!/usr/bin/env python3
"""Ground-truth test for the arm-model forward kinematics.

Every reachability claim this workspace makes rests on `agx_kinematics.fk`
matching what the arm's own controller computes. If it drifts, unreachable
poses start looking reachable (or the reverse) and the failure is silent, so
pin it against a real measurement.

The reference below was captured live from the Piper on can_follower: the joint
angles and the flange pose the arm reported for them at the same instant.
"""

import math

import numpy as np
import pytest

from piper_auto_handeye import agx_kinematics as ak

pytestmark = pytest.mark.skipif(not ak.HAVE_SDK,
                                reason="pyAgxArm not available")

# Captured from the arm on 2026-07-27.
JOINTS_RAD = [-0.05895722213236845, 0.23664919327791117, -0.16477653468078465,
              -0.02556907354171693, 0.43865360090373484, -0.12402309664671705]
FLANGE_XYZ_RPY = [0.067686, -0.004985, 0.215192,
                  -2.817066132473968, 1.1227354012229123, -2.855219029922564]


def test_fk_matches_the_arms_own_report():
    T = ak.fk(JOINTS_RAD)
    x, y, z, roll, pitch, yaw = ak.pose_to_xyz_rpy(T)

    assert x == pytest.approx(FLANGE_XYZ_RPY[0], abs=1e-4)
    assert y == pytest.approx(FLANGE_XYZ_RPY[1], abs=1e-4)
    assert z == pytest.approx(FLANGE_XYZ_RPY[2], abs=1e-4)
    assert roll == pytest.approx(FLANGE_XYZ_RPY[3], abs=1e-3)
    assert pitch == pytest.approx(FLANGE_XYZ_RPY[4], abs=1e-3)
    assert yaw == pytest.approx(FLANGE_XYZ_RPY[5], abs=1e-3)


def test_fk_returns_a_rigid_transform():
    T = ak.fk(JOINTS_RAD)
    R = T[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)
    assert np.allclose(T[3], [0, 0, 0, 1])


def test_zero_configuration_is_folded_over_the_base():
    """All-zero joints park the Piper folded up close to its own base.

    Pins the home configuration so a re-vendored MDH table with different theta
    offsets cannot shift every pose in the workspace unnoticed.
    """
    T = ak.fk([0.0] * 6)
    x, y, z = T[:3, 3]
    assert x == pytest.approx(0.0561, abs=1e-3)
    assert y == pytest.approx(0.0, abs=1e-3)
    assert z == pytest.approx(0.2133, abs=1e-3)
    # Well inside the reach envelope -- it is a folded pose, not an extended one.
    assert math.hypot(x, y) < 0.10


def test_joint_limits_loaded():
    assert len(ak.JOINT_LIMITS) == 6
    for lo, hi in ak.JOINT_LIMITS:
        assert lo < hi
    # Piper's shoulder cannot go negative; a sign flip here would silently
    # double the apparent workspace.
    assert ak.JOINT_LIMITS[1][0] == pytest.approx(0.0, abs=1e-6)


def test_within_joint_limits():
    assert ak.within_joint_limits(JOINTS_RAD)
    over = list(JOINTS_RAD)
    over[0] = ak.JOINT_LIMITS[0][1] + 0.1
    assert not ak.within_joint_limits(over)
    # margin excludes poses sitting on a stop
    at_limit = list(JOINTS_RAD)
    at_limit[0] = ak.JOINT_LIMITS[0][1]
    assert ak.within_joint_limits(at_limit)
    assert not ak.within_joint_limits(at_limit, margin_deg=3.0)


def test_rpy_round_trip():
    for q in ([0.1, 0.5, -0.4, 0.2, 0.3, -0.2], JOINTS_RAD, [0.0] * 6):
        T = ak.fk(q)
        _, _, _, roll, pitch, yaw = ak.pose_to_xyz_rpy(T)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        assert np.allclose(Rz @ Ry @ Rx, T[:3, :3], atol=1e-9)

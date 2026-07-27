#!/usr/bin/env python3
"""Tests for the ㄹ (marker-aiming lattice) calibration sweep.

Pins the specified motion -- left-to-right passes that repeat after a -y row
shift, every stop aiming at the same target point -- and the data-collection
principles it encodes: big rotation differences, small translations, redundant
poses per stop, and every pose executable on the arm's own kinematics.
"""

import math

import numpy as np
import pytest

from piper_auto_handeye import agx_kinematics as ak
from piper_auto_handeye import transform_utils as tu
from piper_auto_handeye.handeye_calibration_node import HandeyeCalibrationNode

pytestmark = pytest.mark.skipif(not ak.HAVE_SDK, reason="pyAgxArm not available")

HOME = [-0.0580, 1.1419, -1.0121, 0.0680, 1.1519, -0.1193]


class _Stub:
    home_joints = HOME
    serp_rows = 3
    serp_cols = 4
    serp_col_step = 0.035
    serp_row_step = 0.03
    serp_twist_deg = 25.0
    serp_orients = 2
    aim_distance = 0.25
    gen_down_cone = 45.0

    _generate_pose_set = HandeyeCalibrationNode._generate_pose_set

    class _L:
        def info(self, *_):
            pass

        def warn(self, *_):
            pass

    def get_logger(self):
        return self._L()


@pytest.fixture(scope="module")
def poses():
    entries = _Stub()._generate_pose_set()
    assert entries is not None, "sweep generation fell back -- lattice unreachable"
    return [np.array(v) for kind, v in entries if kind == "joints"]


def _aim_point():
    T0 = ak.fk(HOME)
    return T0[:3, 3] + T0[:3, :3][:, 2] * _Stub.aim_distance


def test_enough_poses_survive(poses):
    # 3 rows x 4 cols x 2 twists = 24 targets; allow some IK drops
    assert len(poses) >= 16


def test_every_pose_is_executable(poses):
    for q in poses:
        assert ak.within_joint_limits(q, margin_deg=2.0)


def test_every_pose_aims_at_the_marker(poses):
    """The flange axis at every stop must pass close to the shared aim point."""
    aim = _aim_point()
    for q in poses:
        T = ak.fk(q)
        p, z = T[:3, 3], T[:3, :3][:, 2]
        v = aim - p
        dist = np.linalg.norm(v)
        # perpendicular miss distance of the viewing ray from the aim point
        miss = np.linalg.norm(np.cross(z, v))
        assert miss < 0.02, f"viewing ray misses the aim point by {miss * 1000:.0f} mm"
        assert 0.15 < dist < 0.45


def test_same_stop_pairs_are_pure_rotation(poses):
    """Redundant poses at one stop: twisted apart, barely translated."""
    Ts = [ak.fk(q) for q in poses]
    pairs = 0
    for a, b in zip(Ts[0::2], Ts[1::2]):
        if tu.translation_distance(a, b) < 0.005:      # same lattice stop
            rot = math.degrees(tu.rotation_angle_between(a, b))
            assert rot > 30.0, f"same-stop twist only {rot:.1f} deg"
            pairs += 1
    assert pairs >= 6, "too few same-stop redundant pairs survived"


def _rows(poses):
    """Group poses into passes by their (quantised) y position, in emit order.

    Rows cannot be sliced by fixed size: a stop the IK genuinely cannot reach
    is dropped, so passes may have different lengths.
    """
    out, cur, cur_y = [], [], None
    for q in poses:
        T = ak.fk(q)
        y = round(T[1, 3] / _Stub.serp_row_step * 2) / 2  # quantise to half-steps
        if cur_y is None or y == cur_y:
            cur.append(T)
        else:
            out.append(cur)
            cur = [T]
        cur_y = y
    if cur:
        out.append(cur)
    return out


def test_rows_advance_along_minus_y(poses):
    """Pass k+1 sits a row_step further along -y than pass k."""
    rows = _rows(poses)
    assert len(rows) == _Stub.serp_rows
    means = [np.mean([T[1, 3] for T in row]) for row in rows]
    for a, b in zip(means, means[1:]):
        assert b < a - 0.5 * _Stub.serp_row_step, \
            f"rows did not advance along -y: {np.round(means, 3)}"


def test_columns_sweep_the_same_direction_every_row(poses):
    """Each pass goes left-to-right (increasing x); passes repeat, not zigzag."""
    for r, row in enumerate(_rows(poses)):
        stops = []
        for T in row:                      # collapse same-stop twist twins
            if not stops or abs(T[0, 3] - stops[-1]) > 1e-6:
                stops.append(T[0, 3])
        assert len(stops) >= 3
        assert all(b > a for a, b in zip(stops, stops[1:])), \
            f"row {r} does not sweep left-to-right: {np.round(stops, 3)}"
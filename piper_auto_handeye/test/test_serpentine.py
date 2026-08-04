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
    serp_cols = 5
    serp_col_step = 0.03
    serp_row_step = 0.03
    serp_dist_offsets = [0.0, -0.03, 0.03]
    serp_roll_deg = [0.0, -25.0, 25.0, -12.0, 12.0]
    serp_max_rolls = 5
    aim_distance = 0.25
    gen_down_cone = 55.0

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


def _configs(poses):
    """Group poses by their joints 1-5, in emit order."""
    out = {}
    for q in poses:
        out.setdefault(tuple(np.round(np.asarray(q)[:5], 6)), []).append(q)
    return out


def test_enough_poses_survive(poses):
    # rows x cols x standoffs stops, each up to serp_max_rolls poses;
    # allow generous IK/limit drops, but keep a margin over target_samples (30)
    assert len(poses) >= 40


def test_shots_per_arm_configuration_are_capped(poses):
    """No more than serp_max_rolls poses may share joints 1-5.

    Poses that share an arm configuration also share its viewpoint, so a marker
    seen badly there yields that many outliers at once. The solver may only
    discard a bounded number before failing the run, so an uncapped pile of
    joint-6 rolls can fail a calibration whose other samples were all good.
    """
    worst = max(len(v) for v in _configs(poses).values())
    assert worst <= _Stub.serp_max_rolls, (
        f"{worst} poses share one joints-1-5 configuration, "
        f"cap is {_Stub.serp_max_rolls}")


def test_a_run_sees_several_arm_configurations(poses):
    """The samples a run actually consumes must not come from a few stops."""
    for budget in (10, 20, 30):
        n = len(_configs(poses[:budget]))
        assert n >= budget / _Stub.serp_max_rolls, \
            f"only {n} arm configurations in the first {budget} poses"


def test_joints_other_than_six_actually_move(poses):
    """Joints 1-5 must vary across stops, not just joint 6.

    Rolling joint 6 spins the camera in place; it adds rotation but no new
    viewpoint. Without real motion in the other joints every sample looks at
    the marker from nearly the same place and the run leans on one geometry.
    """
    configs = np.degrees(np.array([list(k) for k in _configs(poses)]))
    assert len(configs) >= 6
    spans = configs.max(axis=0) - configs.min(axis=0)
    assert spans[:4].min() > 15.0, f"joints 1-4 barely move: {np.round(spans, 1)}"
    assert spans.sum() > 100.0, f"total joint1-5 travel only {spans.sum():.0f} deg"


def test_joint6_rolls_are_pure_rotation(poses):
    """Poses sharing an arm configuration rotate without moving the flange.

    Rolling the last joint spins the camera about its own axis, so the marker
    stays in frame. If these ever start translating, they have stopped being
    the cheap rotation diversity they exist to provide.
    """
    pairs = 0
    for qs in _configs(poses).values():
        for a, b in zip(qs, qs[1:]):
            Ta, Tb = ak.fk(a), ak.fk(b)
            assert tu.translation_distance(Ta, Tb) < 1e-6, "joint6 roll moved the flange"
            assert math.degrees(tu.rotation_angle_between(Ta, Tb)) > 5.0
            pairs += 1
    assert pairs >= 20, f"only {pairs} joint6-only pairs"


def test_configured_joint6_rolls_are_present(poses):
    """Every configured roll actually appears, measured from its stop."""
    for qs in _configs(poses).values():
        if len(qs) < len(_Stub.serp_roll_deg):
            continue                      # a roll hit a joint limit here
        rolls = sorted(round(math.degrees(np.asarray(q)[5] - np.asarray(qs[0])[5]), 1)
                       for q in qs)
        want = sorted(round(r - _Stub.serp_roll_deg[0], 1)
                      for r in _Stub.serp_roll_deg)
        assert rolls == want, f"rolls {rolls} != configured {want}"
        return
    pytest.fail("no stop kept a full set of rolls")


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


def _first_pass(poses):
    """The opening ㄹ traversal: one pose per stop, in lattice order.

    Poses are emitted roll-major -- a full ㄹ pass at each wrist roll -- so the
    lattice geometry lives in the first len(configs) poses. Later passes repeat
    the same path with the wrist rolled, and checking those too would only
    re-test the same stops.
    """
    return [ak.fk(q) for q in poses[:len(_configs(poses))]]


def _rows(Ts):
    """Group one pass into rows by (quantised) y, in emit order.

    Rows cannot be sliced by fixed size: a stop the IK genuinely cannot reach
    is dropped, so passes may have different lengths.
    """
    out, cur, cur_y = [], [], None
    for T in Ts:
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
    rows = _rows(_first_pass(poses))
    assert len(rows) == _Stub.serp_rows
    means = [np.mean([T[1, 3] for T in row]) for row in rows]
    for a, b in zip(means, means[1:]):
        assert b < a - 0.5 * _Stub.serp_row_step, (
            f"rows did not advance along -y: {np.round(means, 3)}")


def test_columns_sweep_the_same_direction_every_row(poses):
    """Each pass goes left-to-right (increasing x); passes repeat, not zigzag."""
    for r, row in enumerate(_rows(_first_pass(poses))):
        stops = [T[0, 3] for T in row]
        # the last pass can be short: unreachable stops are dropped, not faked
        assert len(stops) >= 2
        assert all(b > a for a, b in zip(stops, stops[1:])), (
            f"row {r} does not sweep left-to-right: {np.round(stops, 3)}")


def test_early_samples_are_spread_over_the_lattice(poses):
    """A short run must still see the whole ㄹ path, not a corner of it.

    This is what roll-major emission buys: stop-major order would spend the
    first 30 samples on 30/serp_max_rolls stops and leave the rest of the
    lattice unvisited, because a run ends the moment target_samples is reached.
    """
    stops_total = len(_configs(poses))
    for budget in (10, 20, 30):
        seen = len(_configs(poses[:budget]))
        assert seen >= min(budget, stops_total), (
            f"first {budget} poses touch only {seen} of {stops_total} stops")

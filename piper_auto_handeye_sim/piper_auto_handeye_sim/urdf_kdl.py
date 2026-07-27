"""Build a PyKDL chain straight from a URDF string (stdlib XML + PyKDL).

``kdl_parser_py`` is not packaged for Humble, so this module does the small
subset of the job we actually need: walk the fixed/revolute joints between two
links and turn them into KDL segments.

Only what a serial arm needs is supported:
  * fixed and revolute/continuous joints
  * ``origin`` xyz/rpy and ``axis`` xyz

Prismatic joints and mimic joints are rejected loudly rather than silently
producing a wrong chain -- a wrong chain here would corrupt every pose the
calibration ever sees.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import PyKDL as kdl


def _parse_triplet(text: str | None, default=(0.0, 0.0, 0.0)):
    if not text:
        return default
    parts = [float(v) for v in text.replace(",", " ").split()]
    if len(parts) != 3:
        raise ValueError(f"expected 3 numbers, got {text!r}")
    return tuple(parts)


class UrdfChainBuilder:
    """Parses a URDF once, then builds KDL chains between arbitrary links."""

    def __init__(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        if root.tag != "robot":
            raise ValueError(f"root element is <{root.tag}>, expected <robot>")

        # child_link -> (joint_name, parent_link, type, origin_xyz, origin_rpy, axis)
        self._parent_of: Dict[str, Tuple] = {}
        for joint in root.findall("joint"):
            jname = joint.get("name", "")
            jtype = joint.get("type", "")
            parent_el = joint.find("parent")
            child_el = joint.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.get("link")
            child = child_el.get("link")
            origin = joint.find("origin")
            xyz = _parse_triplet(origin.get("xyz") if origin is not None else None)
            rpy = _parse_triplet(origin.get("rpy") if origin is not None else None)
            axis_el = joint.find("axis")
            axis = _parse_triplet(axis_el.get("xyz") if axis_el is not None else None,
                                  default=(1.0, 0.0, 0.0))
            self._parent_of[child] = (jname, parent, jtype, xyz, rpy, axis)

        self.links = {link.get("name") for link in root.findall("link")}

    # ------------------------------------------------------------------ #
    def _path(self, base: str, tip: str) -> List[str]:
        """Link names from base (exclusive) down to tip (inclusive)."""
        if tip not in self.links:
            raise ValueError(f"tip link '{tip}' not in URDF")
        if base not in self.links:
            raise ValueError(f"base link '{base}' not in URDF")
        chain: List[str] = []
        cur = tip
        while cur != base:
            if cur not in self._parent_of:
                raise ValueError(
                    f"no path from '{base}' to '{tip}': walked up to '{cur}' "
                    "which has no parent joint")
            chain.append(cur)
            cur = self._parent_of[cur][1]
        chain.reverse()
        return chain

    def build_chain(self, base: str, tip: str) -> Tuple[kdl.Chain, List[str]]:
        """Return (chain, movable_joint_names) from base to tip."""
        chain = kdl.Chain()
        joint_names: List[str] = []

        for link in self._path(base, tip):
            jname, _parent, jtype, xyz, rpy, axis = self._parent_of[link]
            frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))

            # URDF and KDL compose a joint in OPPOSITE orders:
            #   URDF : parent -> (origin transform) -> (rotate about axis) -> child
            #   KDL  : Segment(joint, f_tip) = joint.pose(q) * f_tip
            # So a KDL joint built with the raw axis at the raw origin would
            # rotate BEFORE the origin transform and put every downstream link
            # in the wrong place. kdl_parser handles this by expressing the
            # joint in the parent frame: its reference point becomes origin.p
            # and its axis is rotated by origin.M, while f_tip stays the origin
            # transform. Getting this wrong is silent -- FK still returns a
            # plausible pose, just not the robot's.
            if jtype == "fixed":
                joint = kdl.Joint(jname, kdl.Joint.Fixed)
            elif jtype in ("revolute", "continuous"):
                joint = kdl.Joint(jname, frame.p, frame.M * kdl.Vector(*axis),
                                  kdl.Joint.RotAxis)
                joint_names.append(jname)
            else:
                raise ValueError(
                    f"joint '{jname}' has unsupported type '{jtype}'; this "
                    "builder handles fixed/revolute/continuous only")

            chain.addSegment(kdl.Segment(link, joint, frame))

        return chain, joint_names

    def joint_limits(self, joint_names: List[str], urdf_xml: str
                     ) -> Tuple[List[float], List[float]]:
        """Lower/upper limits for the named joints (continuous -> +/- 2*pi)."""
        root = ET.fromstring(urdf_xml)
        lower, upper = [], []
        by_name = {j.get("name"): j for j in root.findall("joint")}
        for name in joint_names:
            j = by_name.get(name)
            lim = j.find("limit") if j is not None else None
            if lim is None or lim.get("lower") is None:
                lower.append(-6.283185)
                upper.append(6.283185)
            else:
                lower.append(float(lim.get("lower")))
                upper.append(float(lim.get("upper")))
        return lower, upper


def solve_ik(ik_solver, fk_solver, target: kdl.Frame, q_seed: kdl.JntArray,
             lower, upper, attempts: int = 25, rng=None,
             pos_tol: float = 0.005, rot_tol_deg: float = 1.0):
    """Pose IK with random restarts, returning (q, ok).

    A single LMA call from one seed fails often on a 6-DoF arm even when the
    pose is perfectly reachable -- it is a local optimiser and the arm has
    multiple branches (elbow up/down, wrist flipped). Retrying from random
    configurations inside the joint limits turns "unreachable" into "found" for
    the large majority of genuinely reachable targets.

    Success is judged by FK on the returned joints, not by the solver's return
    code: LMA can report success while sitting outside the joint limits, and it
    can report failure having landed close enough anyway.
    """
    import math
    import numpy as np

    if rng is None:
        rng = np.random.default_rng(0)
    n = q_seed.rows()
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    best = None
    best_cost = float("inf")

    for attempt in range(max(1, attempts)):
        if attempt == 0:
            seed = q_seed
        else:
            seed = kdl.JntArray(n)
            draw = rng.uniform(lower, upper)
            for i in range(n):
                seed[i] = float(draw[i])

        q_out = kdl.JntArray(n)
        ik_solver.CartToJnt(seed, target, q_out)

        if any(q_out[i] < lower[i] - 1e-6 or q_out[i] > upper[i] + 1e-6
               for i in range(n)):
            continue

        reached = kdl.Frame()
        if fk_solver.JntToCart(q_out, reached) < 0:
            continue
        d_pos = (reached.p - target.p).Norm()
        d_rot = abs((reached.M.Inverse() * target.M).GetRotAngle()[0])
        cost = d_pos + 0.05 * d_rot
        if cost < best_cost:
            best_cost = cost
            best = (q_out, d_pos, math.degrees(d_rot))
        if d_pos <= pos_tol and math.degrees(d_rot) <= rot_tol_deg:
            return q_out, True

    if best is None:
        return kdl.JntArray(n), False
    q_out, d_pos, d_rot_deg = best
    return q_out, (d_pos <= pos_tol and d_rot_deg <= rot_tol_deg)


def frame_to_matrix(f: kdl.Frame):
    """PyKDL Frame -> 4x4 numpy array."""
    import numpy as np
    T = np.eye(4)
    for r in range(3):
        for c in range(3):
            T[r, c] = f.M[r, c]
        T[r, 3] = f.p[r]
    return T


def matrix_to_frame(T) -> kdl.Frame:
    """4x4 numpy array -> PyKDL Frame."""
    rot = kdl.Rotation(T[0, 0], T[0, 1], T[0, 2],
                       T[1, 0], T[1, 1], T[1, 2],
                       T[2, 0], T[2, 1], T[2, 2])
    return kdl.Frame(rot, kdl.Vector(float(T[0, 3]), float(T[1, 3]), float(T[2, 3])))

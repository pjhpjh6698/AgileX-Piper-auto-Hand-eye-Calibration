#!/usr/bin/env python3
"""Generate calibration poses FROM the arm's actually reachable set.

Hand-writing Cartesian poses for a 6-DoF arm does not work: most orientations
you would like at a given point simply are not achievable. This samples joint
space, keeps configurations whose camera actually sees the marker, then picks a
diverse subset and writes them out as base_T_gripper poses.

Because every pose comes from a real FK solution, reachability is guaranteed by
construction -- no IK guessing, no "UNREACHABLE" surprises in Gazebo.

Selection criteria per pose:
  * flange position inside a comfortable forward box
  * marker inside the image with a border margin
  * viewing incidence on the marker below a limit (glancing views are noisy)
  * depth within a sane range

Diversity (what hand-eye actually needs) is enforced by greedily requiring each
new pose to differ from all accepted ones by a minimum rotation angle.
"""

import argparse
import math
import os
import sys

import numpy as np
import PyKDL as kdl

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from piper_auto_handeye_sim.urdf_kdl import (  # noqa: E402
    UrdfChainBuilder, frame_to_matrix)

# camera mount == ground truth (keep in sync with the xacro)
CAM_XYZ = (-0.042, 0.0, 0.045)
CAM_RPY = (0.0, -1.4708, 0.09)
IMG_W, IMG_H, HFOV = 640, 480, 1.211
MARKER_LEN = 0.07
TABLE_Z = 0.003


def rpy_T(r, p, y, xyz=(0, 0, 0)):
    T = frame_to_matrix(kdl.Frame(kdl.Rotation.RPY(r, p, y), kdl.Vector(*xyz)))
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--samples", type=int, default=400000)
    ap.add_argument("--n-poses", type=int, default=14)
    ap.add_argument("--min-rot-deg", type=float, default=8.0)
    ap.add_argument("--home-height", type=float, default=0.20,
                    help="start pose height above the marker [m]")
    ap.add_argument("--home-tilt-deg", type=float, default=6.0,
                    help="max tool tilt off vertical for the start pose [deg]")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    urdf_xml = open(args.urdf).read()
    b = UrdfChainBuilder(urdf_xml)
    chain, joints = b.build_chain("base_link", "link6")
    lo, hi = b.joint_limits(joints, urdf_xml)
    fk = kdl.ChainFkSolverPos_recursive(chain)

    link6_T_cam = rpy_T(*CAM_RPY, xyz=CAM_XYZ) @ rpy_T(-math.pi / 2, 0, -math.pi / 2)
    fx = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
    cx, cy = IMG_W / 2.0, IMG_H / 2.0

    rng = np.random.default_rng(7)
    Q = rng.uniform(np.array(lo), np.array(hi), size=(args.samples, 6))

    q = kdl.JntArray(6)
    f = kdl.Frame()
    cand = []
    home_cand = []          # straight-down configs, for the start pose
    for k in range(args.samples):
        for i in range(6):
            q[i] = Q[k, i]
        fk.JntToCart(q, f)
        px, py, pz = f.p[0], f.p[1], f.p[2]

        # Straight-down candidates: link6's +Z (the tool axis) pointing at the
        # floor, hovering roughly home_height above it. These define where the
        # marker goes -- see the home-pose block below for why this order.
        tool_z = f.M[2, 2]
        if (tool_z < -math.cos(math.radians(args.home_tilt_deg))
                and abs(pz - (TABLE_Z + args.home_height)) < 0.03
                and px > 0.15):
            home_cand.append((frame_to_matrix(f), abs(tool_z)))

        # comfortable forward box, flange above the table
        if not (0.18 < px < 0.42 and abs(py) < 0.18 and 0.22 < pz < 0.46):
            continue
        base_T_l6 = frame_to_matrix(f)
        base_T_cam = base_T_l6 @ link6_T_cam
        origin = base_T_cam[:3, 3]
        view = base_T_cam[:3, 2]          # optical +Z is the view direction
        if view[2] > -0.35:               # must look downward at the table
            continue
        t = (TABLE_Z - origin[2]) / view[2]
        if not (0.15 < t < 0.55):
            continue
        hit = origin + t * view
        incid = math.degrees(math.acos(min(1.0, abs(float(view[2])))))
        if incid > 55.0:
            continue
        cand.append((base_T_l6, hit, t, incid, Q[k].copy()))

    if not cand:
        raise SystemExit("no candidate poses; loosen the filters")

    print(f"candidates            : {len(cand)}")

    # ---- home pose FIRST, then put the marker under it --------------------
    #
    # The obvious order (pick the marker spot, then find a pose above it) does
    # not work: the arm can only hold the tool vertical in a narrow band, so a
    # marker placed by any other rule usually has no straight-down pose over it.
    # Choosing a reachable vertical pose first and dropping the marker directly
    # beneath GUARANTEES the requested start pose exists.
    if not home_cand:
        raise SystemExit(
            f"no reachable pose with the tool within {args.home_tilt_deg} deg of "
            f"vertical at {args.home_height*100:.0f} cm; try --home-height or "
            "--home-tilt-deg")
    # Pick the home pose whose marker spot the OTHER poses can actually see.
    # Choosing the most-vertical candidate instead puts the marker wherever that
    # one pose happens to be -- often tucked near the base, where barely any of
    # the calibration poses look. Coverage matters far more than the last degree
    # of verticality here.
    h_pre = MARKER_LEN / 2.0
    corners_pre = [np.array([-h_pre, h_pre, 0, 1.0]), np.array([h_pre, h_pre, 0, 1.0]),
                   np.array([h_pre, -h_pre, 0, 1.0]), np.array([-h_pre, -h_pre, 0, 1.0])]

    def marker_under(T_home):
        """Marker spot for a home pose: under the CAMERA, not under link6.

        The camera sits ~6 cm off the tool axis. Centring the marker on link6
        would push it to the image border from only 20 cm up (the offset is a
        large fraction of the field of view at that range), so the start pose
        would not actually show what it is supposed to show. Dropping the marker
        under the optical centre keeps the tool vertical AND the marker centred.
        """
        cam = (T_home @ link6_T_cam)[:3, 3]
        return np.array([cam[0], cam[1], TABLE_Z])

    def coverage(T_home):
        m = np.eye(4)
        m[:3, 3] = marker_under(T_home)
        n = 0
        for base_T_l6, _hit, _t, _incid, _qv in cand:
            cTm = np.linalg.inv(base_T_l6 @ link6_T_cam) @ m
            uvs, ok = [], True
            for c in corners_pre:
                pc = cTm @ c
                if pc[2] <= 1e-6:
                    ok = False
                    break
                uvs.append((fx * pc[0] / pc[2] + cx, fx * pc[1] / pc[2] + cy))
            if not ok:
                continue
            if min(min(min(u, IMG_W - u) for u, _ in uvs),
                   min(min(v, IMG_H - v) for _, v in uvs)) >= 30:
                n += 1
        return n

    scored = [(coverage(T), vert, T) for T, vert in home_cand]
    best_n, _, home_T = max(scored, key=lambda s: (s[0], s[1]))
    marker = marker_under(home_T)
    print(f"home candidates       : {len(home_cand)} "
          f"(best sees {best_n} other poses)")
    print(f"home pose (link6)     : [{home_T[0,3]:.3f}, {home_T[1,3]:.3f}, "
          f"{home_T[2,3]:.3f}], tool {math.degrees(math.acos(min(1.0,abs(home_T[2,2])))):.1f} deg off vertical")
    print(f"marker pose (below it): [{marker[0]:.3f}, {marker[1]:.3f}, {marker[2]:.3f}]")

    # keep only poses that see the marker AT the chosen spot, in frame
    h = MARKER_LEN / 2.0
    corners = [np.array([-h, h, 0, 1.0]), np.array([h, h, 0, 1.0]),
               np.array([h, -h, 0, 1.0]), np.array([-h, -h, 0, 1.0])]
    base_T_marker = np.eye(4)
    base_T_marker[:3, 3] = marker

    good = []
    for base_T_l6, _hit, _t, incid, qv in cand:
        cam_T_marker = np.linalg.inv(base_T_l6 @ link6_T_cam) @ base_T_marker
        uv = []
        ok = True
        for c in corners:
            pc = cam_T_marker @ c
            if pc[2] <= 1e-6:
                ok = False
                break
            uv.append((fx * pc[0] / pc[2] + cx, fx * pc[1] / pc[2] + cy))
        if not ok:
            continue
        margin = min(min(min(u, IMG_W - u) for u, _ in uv),
                     min(min(v, IMG_H - v) for _, v in uv))
        if margin < 30:
            continue
        good.append((base_T_l6, incid, margin, qv))

    print(f"see the marker in view: {len(good)}")
    if len(good) < args.n_poses:
        raise SystemExit("not enough usable poses; adjust the filters")

    # greedy diversity: each accepted pose must rotate away from all others
    def rot_angle(A, B):
        R = A[:3, :3].T @ B[:3, :3]
        return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))

    rng.shuffle(good)
    chosen = []
    for cand_pose in good:
        if all(rot_angle(cand_pose[0], c[0]) >= args.min_rot_deg for c in chosen):
            chosen.append(cand_pose)
        if len(chosen) >= args.n_poses:
            break
    print(f"chosen (>= {args.min_rot_deg} deg apart): {len(chosen)}")

    # ---- home pose: straight down over the marker, first in the sequence ----
    #
    # Starting from a known marker-centred pose means the very first sample
    # cannot fail for "marker not visible", and it gives the operator an
    # unambiguous visual check that the rig is set up correctly before the arm
    # starts wandering through the rest of the poses.
    #
    # The tool axis is link6's +Z, so pointing the end-effector down means
    # rotating its +Z onto world -Z: that is R = Rx(pi), i.e. rpy = (pi, 0, yaw).
    # Yaw about the vertical is free, so try a spread and keep the first that
    # is reachable AND actually sees the marker.
    # home_T was picked before the marker was placed, so it is reachable by
    # construction. Report how the marker lands in its view as a sanity check:
    # the camera sits ~6 cm off the tool axis, so the marker is deliberately
    # off-centre here rather than dead centre.
    cam_T_marker = np.linalg.inv(home_T @ link6_T_cam) @ base_T_marker
    uv = []
    for c in corners:
        pc = cam_T_marker @ c
        uv.append((fx * pc[0] / pc[2] + cx, fx * pc[1] / pc[2] + cy))
    home_margin = min(min(min(u, IMG_W - u) for u, _ in uv),
                      min(min(v, IMG_H - v) for _, v in uv))
    centre = cam_T_marker @ np.array([0, 0, 0, 1.0])
    print(f"home view             : depth {centre[2]:.3f} m, "
          f"marker at ({fx*centre[0]/centre[2]+cx:.0f}, {fx*centre[1]/centre[2]+cy:.0f}) px, "
          f"margin {home_margin:.0f} px")
    if home_margin < 10:
        print("WARNING: marker sits at the image border from the home pose")
    # The home pose can also turn up in `chosen` (it came from the same sampled
    # set), which would waste a slot on a duplicate and quietly cost one unit of
    # the rotation diversity hand-eye depends on. Drop anything too close to it.
    def rot_from_home(T):
        R = home_T[:3, :3].T @ T[:3, :3]
        return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))

    chosen = [c for c in chosen if rot_from_home(c[0]) >= args.min_rot_deg]
    chosen = [(home_T, 0.0, home_margin, None)] + chosen[:args.n_poses - 1]

    lines = [
        "# Calibration poses for the GAZEBO Piper.",
        "#",
        "# GENERATED by scripts/generate_poses.py from the arm's actually reachable",
        "# set: every pose below is a real FK solution inside the joint limits whose",
        "# camera sees the marker with margin. Do not hand-edit the numbers -- rerun",
        "# the generator if the camera mount or marker position changes.",
        "#",
        f"# marker assumed at [{marker[0]:.3f}, {marker[1]:.3f}, {marker[2]:.3f}] "
        "(see worlds/handeye_calibration.world)",
        "/**:",
        "  ros__parameters:",
        "    poses:",
    ]
    for i, (T, incid, margin, _q) in enumerate(chosen):
        r, p, y = kdl.Rotation(*T[:3, :3].flatten()).GetRPY()
        t = T[:3, 3]
        lines.append(f"      - name: {'home_over_marker' if i == 0 else f'pose_{i:02d}'}")
        lines.append(f"        position: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
        lines.append(f"        rpy: [{r:.4f}, {p:.4f}, {y:.4f}]")
        lines.append(f"        # incidence {incid:.1f} deg, image margin {margin:.0f} px")

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "config", "sim_calibration_poses.yaml")
    out = os.path.normpath(out)
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

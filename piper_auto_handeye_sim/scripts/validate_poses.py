#!/usr/bin/env python3
"""Offline check of calibration poses: reachability, aim, and marker visibility.

Runs the same kinematics Gazebo will, but without Gazebo. For each candidate
pose it reports:

  IK      - does a solution exist inside the real joint limits?
  depth   - marker distance along the camera's optical +Z (must be positive)
  u,v     - where the marker centre lands in the image
  margin  - closest marker corner to the image border, in pixels
  incid   - angle between the marker normal and the viewing ray

Catching an unreachable or out-of-view pose here takes a second; catching it in
Gazebo takes a full launch cycle and looks like a mysterious "0 samples".

Usage:
    python3 validate_poses.py
    python3 validate_poses.py --poses ../config/sim_calibration_poses.yaml
"""

import argparse
import math
import os
import sys

import numpy as np
import PyKDL as kdl
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from piper_auto_handeye_sim.urdf_kdl import (  # noqa: E402
    UrdfChainBuilder, frame_to_matrix, matrix_to_frame, solve_ik)

# --- must match urdf/piper_handeye_gazebo.xacro ---------------------------- #
CAM_XYZ = (-0.042, 0.0, 0.045)
CAM_RPY = (0.0, -1.4708, 0.09)
IMG_W, IMG_H, HFOV = 640, 480, 1.211
# --- must match worlds/handeye_calibration.world --------------------------- #
MARKER_XYZ = (0.351, 0.044, 0.003)
MARKER_RPY = (0.0, 0.0, 0.0)      # lying flat, face normal = +Z
MARKER_LEN = 0.07


def rpy_matrix(r, p, y):
    return frame_to_matrix(kdl.Frame(kdl.Rotation.RPY(r, p, y), kdl.Vector(0, 0, 0)))


def make_T(xyz, rpy):
    T = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--poses", default=os.path.join(
        here, "..", "config", "sim_calibration_poses.yaml"))
    args = ap.parse_args()

    urdf_path = args.urdf
    if urdf_path is None:
        from ament_index_python.packages import get_package_share_directory
        urdf_path = os.path.join(get_package_share_directory("piper_description"),
                                 "urdf", "piper_no_gripper_description.urdf")
    with open(urdf_path) as f:
        urdf_xml = f.read()

    builder = UrdfChainBuilder(urdf_xml)
    chain, joints = builder.build_chain("base_link", "link6")
    lower, upper = builder.joint_limits(joints, urdf_xml)
    fk = kdl.ChainFkSolverPos_recursive(chain)
    ik = kdl.ChainIkSolverPos_LMA(chain, 1e-7, 1000)

    # camera extrinsics (the ground truth) and intrinsics
    link6_T_cam = make_T(CAM_XYZ, CAM_RPY) @ rpy_matrix(-math.pi / 2, 0, -math.pi / 2)
    fx = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
    fy = fx
    cx, cy = IMG_W / 2.0, IMG_H / 2.0

    print(f"URDF        : {urdf_path}")
    print(f"chain       : {len(joints)} joints {joints}")
    print(f"fx=fy       : {fx:.2f} px   (hfov {math.degrees(HFOV):.1f} deg)")
    t = link6_T_cam[:3, 3]
    print(f"GROUND TRUTH gripper_T_camera t = "
          f"[{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}] m")
    print()

    base_T_marker = make_T(MARKER_XYZ, MARKER_RPY)
    h = MARKER_LEN / 2.0
    corners_m = [np.array([-h, h, 0, 1.0]), np.array([h, h, 0, 1.0]),
                 np.array([h, -h, 0, 1.0]), np.array([-h, -h, 0, 1.0])]

    with open(args.poses) as f:
        data = yaml.safe_load(f)
    poses = data.get("/**", {}).get("ros__parameters", data).get("poses", [])

    hdr = (f"{'#':>2} {'name':<14} {'IK':<4} {'depth':>7} {'u':>7} {'v':>7} "
           f"{'margin':>7} {'incid':>6}  verdict")
    print(hdr)
    print("-" * len(hdr))

    rng = np.random.default_rng(3)
    q_seed = kdl.JntArray(len(joints))
    for i, val in enumerate((0.0, 1.10, -0.90, 0.0, 0.50, 0.0)[:len(joints)]):
        q_seed[i] = val

    ok_count = 0
    for idx, item in enumerate(poses):
        name = item.get("name", f"pose{idx}")
        target = make_T(item["position"], item["rpy"])

        q, ik_ok = solve_ik(ik, fk, matrix_to_frame(target), q_seed,
                            lower, upper, attempts=60, rng=rng)
        reached = kdl.Frame()
        fk.JntToCart(q, reached)
        base_T_link6 = frame_to_matrix(reached)
        pos_err = np.linalg.norm(base_T_link6[:3, 3] - target[:3, 3])

        if not ik_ok:
            print(f"{idx:>2} {name:<14} {'FAIL':<4} {'-':>7} {'-':>7} {'-':>7} "
                  f"{'-':>7} {'-':>6}  UNREACHABLE")
            continue

        base_T_cam = base_T_link6 @ link6_T_cam
        cam_T_marker = np.linalg.inv(base_T_cam) @ base_T_marker

        uv = []
        for c in corners_m:
            pc = cam_T_marker @ c
            if pc[2] <= 1e-6:
                uv = None
                break
            uv.append((fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy))
        centre = cam_T_marker @ np.array([0, 0, 0, 1.0])
        depth = centre[2]

        if uv is None or depth <= 0:
            print(f"{idx:>2} {name:<14} {'ok':<4} {depth:>7.3f} {'-':>7} {'-':>7} "
                  f"{'-':>7} {'-':>6}  BEHIND CAMERA")
            continue

        u = fx * centre[0] / depth + cx
        v = fy * centre[1] / depth + cy
        margin = min(min(min(a, IMG_W - a) for a, _ in uv),
                     min(min(b, IMG_H - b) for _, b in uv))

        # incidence: angle between marker normal (+Z of marker) and view ray
        normal_cam = (np.linalg.inv(base_T_cam) @ base_T_marker)[:3, 2]
        ray = centre[:3] / np.linalg.norm(centre[:3])
        incid = math.degrees(math.acos(abs(float(np.dot(normal_cam, ray)))))

        if margin < 5:
            verdict = "OUT OF FRAME"
        elif incid > 65:
            verdict = "TOO OBLIQUE"
        elif depth < 0.12:
            verdict = "TOO CLOSE"
        else:
            verdict = "OK"
            ok_count += 1

        print(f"{idx:>2} {name:<14} {'ok':<4} {depth:>7.3f} {u:>7.1f} {v:>7.1f} "
              f"{margin:>7.1f} {incid:>6.1f}  {verdict}")

    print("-" * len(hdr))
    print(f"{ok_count}/{len(poses)} poses usable "
          f"(hand-eye wants >= 10, with rotation about >= 2 axes)")


if __name__ == "__main__":
    main()

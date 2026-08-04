#!/usr/bin/env python3
"""Hand-eye calibration manager (state machine + RunCalibration action).

Orchestrates the full Eye-in-Hand procedure. Talks to other nodes ONLY through
ROS interfaces:
  subscribes  marker_detection (MarkerDetection)   camera_T_target
              robot_state       (RobotState)        base_T_gripper
  uses action move_to_calibration_pose (MoveToCalibrationPose)
  provides    run_calibration   (RunCalibration action)
              add_manual_sample / reset_calibration / save_calibration /
              load_calibration  (services)
  publishes   calibration_status (CalibrationStatus)

Sample pairs are always stored as (base_T_gripper, camera_T_target). The solver
returns gripper_T_camera. Collection happens ONLY while the robot is stopped.
"""

import datetime
import math
import os
import threading
import time
from collections import deque
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from auto_handeye_interfaces.action import RunCalibration, MoveToCalibrationPose
from auto_handeye_interfaces.msg import CalibrationStatus, MarkerDetection, RobotState
from auto_handeye_interfaces.srv import (ResetCalibration, SaveCalibration,
                                         LoadCalibration, AddManualSample)
from std_srvs.srv import Trigger

from . import transform_utils as tu
from . import agx_kinematics as ak
from . import calibration_solver as solver
from . import calibration_validator as validator
from . import calibration_io as cio


# state machine states
IDLE = "IDLE"
CHECKING_SYSTEM = "CHECKING_SYSTEM"
MOVING = "MOVING"
SETTLING = "SETTLING"
WAITING_FOR_MARKER = "WAITING_FOR_MARKER"
COLLECTING = "COLLECTING"
SAMPLE_ACCEPTED = "SAMPLE_ACCEPTED"
SAMPLE_REJECTED = "SAMPLE_REJECTED"
RECOVERING = "RECOVERING"
HOMING = "HOMING"
ALIGNING = "ALIGNING"
SOLVING = "SOLVING"
VALIDATING = "VALIDATING"
SUCCESS = "SUCCESS"
WARNING = "WARNING"
PAUSED = "PAUSED"
CANCELED = "CANCELED"
FAILED = "FAILED"


# Why a collection attempt at one pose ended. The distinction drives what we do
# next, so keep them separate -- "the gripper is in the way" and "the detector is
# jittery" call for opposite responses.
OUTCOME_OK = "ok"                    # sample stored
OUTCOME_OCCLUDED = "occluded"        # marker NEVER seen -> out of view / blocked: MOVE
OUTCOME_UNSTABLE = "unstable"        # marker seen but too few good frames: RETRY IN PLACE
OUTCOME_DUPLICATE = "duplicate"      # good sample, but orientation too close to an existing one
OUTCOME_NO_ROBOT = "no_robot"        # lost the robot pose stream
OUTCOME_ABORT = "abort"              # cancelled / shutting down


class Sample:
    def __init__(self, pose_index, base_T_gripper, camera_T_target, meta):
        self.pose_index = pose_index
        self.base_T_gripper = base_T_gripper
        self.camera_T_target = camera_T_target
        self.meta = meta


class HandeyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")
        p = self.declare_parameter
        self.method = p("calibration_method", "PARK").value
        self.min_samples = int(p("minimum_samples", 10).value)
        self.target_samples = int(p("target_samples", 15).value)
        self.base_frame = p("base_frame", "base_link").value
        self.gripper_frame = p("gripper_frame", "link6").value
        self.camera_frame = p("camera_frame", "camera_color_optical_frame").value
        self.target_frame = p("target_frame", "calibration_target").value
        self.settle_time = float(p("settle_time", 1.0).value)
        self.obs_per_pose = int(p("observations_per_pose", 10).value)
        self.marker_timeout = float(p("marker_timeout", 5.0).value)
        self.marker_stability_min = float(p("marker_stability_min", 0.6).value)
        self.max_dt = float(p("maximum_pose_time_difference", 0.2).value)
        self.min_rot_diff_deg = float(p("minimum_rotation_difference_deg", 5.0).value)
        self.on_marker_timeout = p("on_marker_timeout", "skip").value
        self.max_t_rms = float(p("maximum_translation_rms_m", 0.01).value)
        self.max_r_rms = float(p("maximum_rotation_rms_deg", 1.0).value)
        self.enable_outlier = bool(p("enable_outlier_removal", True).value)
        self.max_removals = int(p("max_outlier_removals", 3).value)
        # Floor on the budget as a fraction of the sample count, so the cap
        # tracks target_samples instead of being tuned for one run length.
        self.outlier_fraction = float(p("outlier_removal_fraction", 0.2).value)
        out_dir = p("output_directory", "").value
        self.output_dir = out_dir if out_dir else cio.default_output_dir()
        # Path to calibration_poses.yaml (loaded directly; robust vs ROS param
        # nested-list flattening). Empty => use built-in placeholder poses.
        self.calibration_poses_file = p("calibration_poses_file", "").value

        # Rest pose: where the arm parks when a run ends or is reset. Leaving
        # the arm wherever the last calibration pose happened to be is a real
        # hazard on hardware -- it can sit stretched out over the workspace.
        # Empty list disables the behaviour.
        self.rest_position = list(p("rest_position", [0.0, 0.0, 0.0]).value)
        self.rest_rpy = list(p("rest_rpy", [0.0, 0.0, 0.0]).value)
        self.return_to_rest = bool(p("return_to_rest", True).value)

        # --- marker recovery -------------------------------------------------
        # On an eye-in-hand rig the gripper sits between the camera and the
        # world, so at some calibration poses it simply covers the marker. The
        # old behaviour was to skip that pose, which quietly thins out exactly
        # the orientations that make the solve well-conditioned. Instead, nudge
        # the wrist until the marker comes back into view and sample there --
        # the sample is still valid, because what the solver needs is the
        # (base_T_gripper, camera_T_target) PAIR, not any particular pose.
        self.recovery_enabled = bool(p("marker_recovery_enabled", True).value)
        self.recovery_max_attempts = int(p("marker_recovery_max_attempts", 5).value)
        self.recovery_trans_step = float(p("marker_recovery_translation_step", 0.05).value)
        self.recovery_rot_step_deg = float(p("marker_recovery_rotation_step_deg", 12.0).value)
        self.recovery_joint_step_deg = float(p("marker_recovery_joint_step_deg", 8.0).value)
        # Retry in place while the marker IS visible but the detector is noisy.
        self.retry_in_place = int(p("marker_retry_in_place", 3).value)
        self.retry_timeout_growth = float(p("marker_retry_timeout_growth", 1.5).value)
        # A pose only counts as "not occluded" once we have seen this many
        # detected frames; one lucky frame is not evidence of visibility.
        self.recovery_min_sightings = int(p("marker_recovery_min_sightings", 3).value)

        # --- down-looking home + marker alignment ---------------------------
        # The marker lies flat on the table, so the useful working posture is
        # the camera looking DOWN. home_joints is a joint configuration whose
        # flange axis points straight down (found against the arm's own
        # kinematics; tilt 0.01 deg). On startup, and again at every run start,
        # the arm goes here first; then it hunts for the marker with small
        # wrist sweeps if needed, and centres itself vertically above it.
        self.home_joints = list(p("home_joints",
                                  [-0.0580, 1.1419, -1.0121, 0.0680,
                                   1.1519, -0.1193]).value)
        self.startup_home = bool(p("startup_home", True).value)
        # ㄹ sweep geometry -- see _generate_pose_set
        self.serp_rows = int(p("serp_rows", 3).value)
        self.serp_cols = int(p("serp_cols", 4).value)
        self.serp_col_step = float(p("serp_col_step", 0.06).value)    # m along x
        self.serp_row_step = float(p("serp_row_step", 0.05).value)    # m along -y
        # Standoff offsets cycled across stops: viewing the marker from a
        # different distance is what actually swings joints 2/3/5, which sliding
        # sideways at a fixed radius barely does.
        self.serp_dist_offsets = list(p("serp_dist_offsets",
                                        [0.0, -0.06, 0.06]).value)
        # Joint-6 rolls taken at each stop. These share joints 1-5 exactly, so
        # they are cheap rotation diversity but NOT viewpoint diversity -- and
        # every one of them fails together when a viewpoint is bad. Capped.
        self.serp_roll_deg = list(p("serp_roll_deg",
                                    [0.0, -25.0, 25.0, -12.0, 12.0]).value)
        self.serp_max_rolls = int(p("serp_max_rolls_per_stop", 5).value)
        # distance from the home flange, along its viewing axis, to the point
        # every stop aims at (~= camera-to-marker distance at home)
        self.aim_distance = float(p("aim_distance", 0.25).value)
        # Build the serpentine set at run start; calibration_poses.yaml is the
        # fallback when this is off or the pattern yields too few poses.
        self.auto_generate = bool(p("auto_generate_poses", True).value)
        self.gen_down_cone = float(p("gen_down_cone_deg", 40.0).value)

        # runtime state
        self._lock = threading.Lock()
        self._samples: List[Sample] = []
        self._state = IDLE
        self._pause = threading.Event()
        self._cancel = threading.Event()
        self._last_result = None            # SolveResult
        self._last_validation = None
        self._marker_buf = deque(maxlen=200)   # (stamp_sec, MarkerDetection)
        self._robot_buf = deque(maxlen=400)    # (stamp_sec, RobotState)
        self._latest_marker: Optional[MarkerDetection] = None
        self._latest_robot: Optional[RobotState] = None
        self._live = None                # running estimate for the GUI panel

        cb = ReentrantCallbackGroup()
        self.create_subscription(MarkerDetection, "marker_detection",
                                 self._marker_cb, 10, callback_group=cb)
        self.create_subscription(RobotState, "robot_state",
                                 self._robot_cb, 10, callback_group=cb)
        self.status_pub = self.create_publisher(CalibrationStatus, "calibration_status", 10)
        self.create_timer(0.2, self._publish_status)

        self._move_client = ActionClient(self, MoveToCalibrationPose,
                                         "move_to_calibration_pose", callback_group=cb)
        # Reset must be able to un-latch the control node's STOP flag, or its
        # homing move (and every move after) is silently rejected.
        self._clear_stop_cli = self.create_client(Trigger, "clear_stop",
                                                  callback_group=cb)

        self._run_server = ActionServer(
            self, RunCalibration, "run_calibration",
            execute_callback=self._execute_run,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=cb)

        self.create_service(ResetCalibration, "reset_calibration", self._srv_reset, callback_group=cb)
        self.create_service(SaveCalibration, "save_calibration", self._srv_save, callback_group=cb)
        self.create_service(LoadCalibration, "load_calibration", self._srv_load, callback_group=cb)
        self.create_service(AddManualSample, "add_manual_sample", self._srv_add_manual, callback_group=cb)
        self.create_service(Trigger, "pause_calibration", self._srv_pause, callback_group=cb)
        self.create_service(Trigger, "resume_calibration", self._srv_resume, callback_group=cb)

        cio.ensure_dir(self.output_dir)

        # At launch the arm makes ONE plain move to the taught home pose --
        # no marker search, no alignment (that routine was removed). A Start
        # pressed meanwhile takes over cleanly via the abort flag + lock.
        self._startup_abort = threading.Event()
        self._motion_lock = threading.Lock()   # startup/reset vs action runs
        if self.startup_home:
            threading.Thread(target=self._startup_home, daemon=True,
                             name="startup_home").start()

        self.get_logger().info(
            f"handeye_calibration_node up | method={self.method} "
            f"target_samples={self.target_samples} out={self.output_dir}")

    # ------------------------------------------------------------------ #
    def _marker_cb(self, msg: MarkerDetection):
        with self._lock:
            self._latest_marker = msg
            if msg.detected:
                self._marker_buf.append((self._stamp_sec(msg.header), msg))

    def _robot_cb(self, msg: RobotState):
        with self._lock:
            self._latest_robot = msg
            self._robot_buf.append((self._stamp_sec(msg.header), msg))

    @staticmethod
    def _stamp_sec(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _set_state(self, s, msg=""):
        with self._lock:
            self._state = s
        if msg:
            self.get_logger().info(f"[{s}] {msg}")

    # ------------------------------------------------------------------ #
    def _publish_status(self):
        with self._lock:
            state = self._state
            n = len(self._samples)
            val = self._last_validation
        st = CalibrationStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.state = state
        st.current_sample_count = n
        st.target_sample_count = self.target_samples
        st.current_pose_index = getattr(self, "_current_pose_index", -1)
        st.progress = float(min(1.0, n / max(1, self.target_samples)))
        st.message = getattr(self, "_status_message", "")
        if val is not None:
            st.translation_rms = float(val.translation_rms_m)
            st.rotation_rms_deg = float(val.rotation_rms_deg)
        with self._lock:
            live = self._live
        if live is not None:
            st.live_valid = True
            st.live_sample_count = int(live["n"])
            st.live_translation_rms = live["t_rms"]
            st.live_rotation_rms_deg = live["r_rms"]
            R, t = tu.decompose_transform(live["T"])
            q = tu.matrix_to_quaternion(R)
            st.live_gripper_to_camera.translation.x = float(t[0])
            st.live_gripper_to_camera.translation.y = float(t[1])
            st.live_gripper_to_camera.translation.z = float(t[2])
            st.live_gripper_to_camera.rotation.x = float(q[0])
            st.live_gripper_to_camera.rotation.y = float(q[1])
            st.live_gripper_to_camera.rotation.z = float(q[2])
            st.live_gripper_to_camera.rotation.w = float(q[3])
        self.status_pub.publish(st)

    # ------------------------------------------------------------------ #
    # RunCalibration action
    # ------------------------------------------------------------------ #
    def _on_cancel(self, goal_handle):
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _execute_run(self, goal_handle):
        g = goal_handle.request
        self._cancel.clear()
        self._pause.clear()
        # take over from a still-running startup routine
        self._startup_abort.set()
        self._motion_lock.acquire()
        target_n = g.target_sample_count if g.target_sample_count > 0 else self.target_samples
        method = g.calibration_method if g.calibration_method else self.method
        settle = g.settle_time if g.settle_time > 0 else self.settle_time
        obs = g.observations_per_pose if g.observations_per_pose > 0 else self.obs_per_pose

        result = RunCalibration.Result()
        try:
            return self._execute_run_locked(goal_handle, g, result,
                                            target_n, method, settle, obs)
        finally:
            self._motion_lock.release()

    def _execute_run_locked(self, goal_handle, g, result,
                            target_n, method, settle, obs):
        # ---- system check ----
        self._set_state(CHECKING_SYSTEM, "verifying camera/marker/robot streams")
        ok, why = self._system_check()
        if not ok:
            return self._finish_run(goal_handle, result, FAILED, method, why)

        if g.auto_move:
            # Reproducible start: go to the taught down-looking home. No marker
            # search, no alignment -- place the marker where the camera looks.
            self._set_state(HOMING, "moving to the taught home pose")
            if not self._send_move(-1, ("joints", self.home_joints)):
                self.get_logger().warn(
                    "homing move rejected/failed; collecting from the current pose")
            ok = self._auto_collect(goal_handle, target_n, settle, obs)
        else:
            ok = self._manual_collect(goal_handle, target_n)

        if self._cancel.is_set():
            return self._finish_run(goal_handle, result, CANCELED, method, "canceled by user")

        with self._lock:
            n = len(self._samples)
        if getattr(self, "_dry_run_detected", False):
            ok = sum(1 for _, good in getattr(self, "_pose_validation", []) if good)
            total = len(getattr(self, "_pose_validation", []))
            return self._finish_run(
                goal_handle, result, FAILED, method,
                f"DRY-RUN: {ok}/{total} poses validated, no samples collected "
                "(robot did not move). Relaunch with dry_run:=false for a real calibration.")
        if n < self.min_samples:
            return self._finish_run(goal_handle, result, FAILED, method,
                                    f"only {n} samples (< min {self.min_samples})")

        # ---- solve ----
        self._set_state(SOLVING, f"solving with {method} on {n} samples")
        try:
            solve_result, val = self._solve_and_validate(method)
        except Exception as exc:  # noqa: BLE001
            return self._finish_run(goal_handle, result, FAILED, method, f"solve failed: {exc}")

        self._last_result = solve_result
        self._last_validation = val

        final_state = SUCCESS if val.status == "SUCCESS" else (
            WARNING if val.status == "WARNING" else FAILED)

        saved_path = ""
        if g.save_on_success and final_state in (SUCCESS, WARNING):
            saved_path = self._save_result(solve_result, val, method, g.output_path)
        elif final_state == FAILED:
            # A failed run is the one you most need to look at, and it used to be
            # the only one that left nothing behind -- so the numbers that
            # explain the failure died with the process. Write the samples (not
            # the result: it is not a calibration anyone should load).
            try:
                self._save_samples_only(method)
            except Exception as exc:      # noqa: BLE001 - never mask the failure
                self.get_logger().warn(f"could not save failed-run samples: {exc}")

        result.gripper_to_camera = tu.matrix_to_transform_msg(solve_result.gripper_T_camera)
        result.sample_count = solve_result.sample_count
        result.translation_rms_m = val.translation_rms_m
        result.translation_max_m = val.translation_max_m
        result.rotation_rms_deg = val.rotation_rms_deg
        result.rotation_max_deg = val.rotation_max_deg
        result.saved_path = saved_path
        return self._finish_run(goal_handle, result, final_state, method,
                                "; ".join(val.messages) or "ok")

    def _startup_home(self):
        """One plain move to home_joints once the robot is up. Nothing else."""
        deadline = time.time() + 30.0
        while time.time() < deadline and not self._startup_abort.is_set():
            with self._lock:
                r = self._latest_robot
            if r is not None and r.connected:
                break
            time.sleep(0.3)
        else:
            if not self._startup_abort.is_set():
                self.get_logger().warn("startup: robot never connected; not homing")
            return
        if not self._motion_lock.acquire(timeout=1.0):
            return
        try:
            if self._startup_abort.is_set():
                return
            self._set_state(HOMING, "startup: moving to the taught home pose")
            if not self._send_move(-1, ("joints", list(self.home_joints))):
                self.get_logger().warn("startup homing move rejected/failed")
            self._set_state(IDLE, "ready")
        finally:
            self._motion_lock.release()

    def _generate_pose_set(self):
        """Build the ㄹ sweep: marker-aiming stops on a lattice.

        The motion, as specified: starting around the home (rest) pose, step
        LEFT and take a pose looking at the marker; step right and look again;
        continue right past home, one more step right -- then shift a little
        along base -y and repeat the same left-to-right pass. Every stop AIMS
        the flange axis at the same target point, so the whole sweep orbits
        the marker the way a hand-held calibration is filmed.

        This satisfies the data-collection principles by construction:
          * rotation difference is MAXIMISED -- stops at different lattice
            points aim from different directions, and at each stop a second
            pose is taken twisted about the aiming axis (pure rotation, marker
            stays centred);
          * translation is MINIMISED -- the lattice steps are a few cm;
          * REDUNDANT poses are welcomed -- the duplicate-orientation gate is
            off (minimum_rotation_difference_deg: 0).

        Where the marker is taken to be: the point the HOME pose looks at,
        aim_distance along its flange axis. Every target is converted to
        joints with the arm's own kinematics (IK seeded from the previous
        stop); stops the arm cannot reach are dropped, not approximated.
        """
        if not ak.HAVE_SDK:
            return None
        home = np.array(self.home_joints, dtype=float)
        T0 = ak.fk(home)
        p0, R0 = T0[:3, 3], T0[:3, :3]
        aim = p0 + R0[:, 2] * self.aim_distance      # where home is looking

        rows, cols = self.serp_rows, self.serp_cols
        col_off = [(c - (cols - 1) / 2.0) * self.serp_col_step for c in range(cols)]
        dists = list(self.serp_dist_offsets) or [0.0]
        # Rolls are applied straight to joint 6, so every roll at a stop shares
        # joints 1-5 by construction. The cap is the point: shots that share an
        # arm configuration also share its viewpoint, so a marker seen badly
        # there produces that many correlated outliers at once -- and the
        # solver only gets to discard max_outlier_removals of them before the
        # whole run is failed on samples that were never independent.
        rolls = list(self.serp_roll_deg)[:max(1, self.serp_max_rolls)]
        if 0.0 not in rolls:
            rolls = [0.0] + rolls[:max(0, self.serp_max_rolls - 1)]

        cone_cos = math.cos(math.radians(self.gen_down_cone))

        def aim_rotation(pos):
            """Rotation whose z-axis looks from ``pos`` at the aim point."""
            z_t = aim - pos
            nz = np.linalg.norm(z_t)
            if nz < 1e-6:
                return None
            z_t = z_t / nz
            if -z_t[2] < cone_cos:
                return None
            x_t = R0[:, 0] - np.dot(R0[:, 0], z_t) * z_t
            nx = np.linalg.norm(x_t)
            if nx < 1e-6:
                return None
            x_t = x_t / nx
            return np.column_stack([x_t, np.cross(z_t, x_t), z_t])

        # Near-vertical aims can exceed the wrist-pitch limit (j5 tops out at
        # 70 deg), and a single IK seed can stall in a local minimum. Two
        # remedies per stop: alternate elbow-posture seeds, and -- if the stop
        # is still unreachable -- lower it a little so the aim gets more
        # oblique. The ㄹ path survives; only that stop's height changes.
        elbow_alt = [np.zeros(6),
                     np.array([0.0, 0.25, -0.35, 0.0, -0.20, 0.0]),
                     np.array([0.0, -0.20, 0.25, 0.0, 0.15, 0.0])]

        def solve_stop(pos, q_prev):
            for drop in (0.0, 0.03, 0.06):
                p_d = pos - np.array([0.0, 0.0, drop])
                R_aim = aim_rotation(p_d)
                if R_aim is None:
                    continue
                T_t = tu.make_transform(R_aim, p_d)
                for base_seed in (q_prev, home):
                    for d in elbow_alt:
                        q = ak.ik(T_t, np.asarray(base_seed) + d)
                        if q is not None and ak.within_joint_limits(q, margin_deg=3.0):
                            return q
            return None

        stop_qs, dropped = [], 0
        q_seed = home
        for r_i in range(rows):
            y_off = -r_i * self.serp_row_step        # rows advance along base -y
            for c_i, x_off in enumerate(col_off):    # same left->right every row
                # Cycle the standoff so CONSECUTIVE stops differ in reach, not
                # only in sideways position. Sliding across a fixed sphere moves
                # joints 1 and 4 and leaves 2/3/5 almost still; changing how far
                # the arm must reach is what exercises them.
                d_off = dists[(r_i * cols + c_i) % len(dists)]
                pos = p0 + np.array([x_off, y_off, 0.0]) + R0[:, 2] * d_off
                q = solve_stop(pos, q_seed)
                if q is None:
                    dropped += 1
                    continue
                q_seed = q                           # chain: next IK starts nearby
                stop_qs.append(q)

        # Emit ROLL-MAJOR: one ㄹ pass at each wrist roll, rather than all the
        # rolls of one stop before moving on. A run stops at target_samples, so
        # the emit order decides what it actually collects -- stop-major spends
        # the whole budget on target_samples/len(rolls) viewpoints, while this
        # covers every stop in the first pass and only then adds rolls. Same
        # poses, same ㄹ path, walked once per roll.
        poses = []
        for roll in rolls:
            for q in stop_qs:
                q6 = np.array(q, dtype=float)
                q6[5] += math.radians(roll)
                if ak.within_joint_limits(q6, margin_deg=3.0):
                    poses.append(("joints", [float(v) for v in q6]))
                else:
                    dropped += 1
        stops = len(stop_qs)

        if len(poses) < 6:
            self.get_logger().warn(
                f"marker-aiming sweep produced only {len(poses)} reachable poses "
                f"({dropped} dropped); falling back to the pose file")
            return None
        self.get_logger().info(
            f"ㄹ sweep: {rows} rows x {cols} cols x {len(dists)} standoffs -> "
            f"{stops} arm configurations x up to {len(rolls)} joint6 rolls = "
            f"{len(poses)} poses aiming at ({aim[0]:.3f}, {aim[1]:.3f}, {aim[2]:.3f})"
            f" ({dropped} dropped)")
        return poses

    def _go_to_rest(self, why=""):
        """Park the arm at the configured rest pose. Best-effort, never raises.

        Called at the end of every run and on reset, so it must not be able to
        turn a successful calibration into a failure: a rest move that is
        rejected or times out is logged and swallowed.
        """
        if not self.return_to_rest:
            return False
        try:
            if any(self.rest_position):
                entry = ("pose", tu.make_transform(
                    tu.euler_to_matrix(*self.rest_rpy), self.rest_position))
                where = f"rest pose {self.rest_position}"
            else:
                # No explicit rest configured: home is the natural park. Ending
                # a run stretched out mid-air over the table is what this avoids.
                entry = ("joints", list(self.home_joints))
                where = "taught home pose"
            self.get_logger().info(f"returning to {where}"
                                   + (f" ({why})" if why else ""))
            self._current_pose_index = -1
            ok = self._send_move(-1, entry)
            if not ok:
                self.get_logger().warn("rest move was rejected or timed out")
            return ok
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"rest move failed: {exc}")
            return False

    def _finish_run(self, goal_handle, result, state, method, message):
        self._set_state(state, message)
        self._status_message = message
        result.state = state
        result.success = state in (SUCCESS, WARNING)
        result.message = message
        self._publish_run_feedback(goal_handle, state, message)

        # Park before reporting the outcome. Do it for cancels and failures too:
        # those are exactly the cases where the arm is left somewhere awkward.
        # _go_to_rest never raises, so it cannot change the result.
        if state != PAUSED:
            self._go_to_rest(why=f"run finished: {state}")

        if state == CANCELED:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _publish_run_feedback(self, goal_handle, state, message):
        with self._lock:
            n = len(self._samples)
        fb = RunCalibration.Feedback()
        fb.state = state
        fb.current_sample_count = n
        fb.target_sample_count = self.target_samples
        fb.current_pose_index = getattr(self, "_current_pose_index", -1)
        fb.progress = float(min(1.0, n / max(1, self.target_samples)))
        fb.message = message
        try:
            goal_handle.publish_feedback(fb)
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    def _system_check(self):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._lock:
                m = self._latest_marker
                r = self._latest_robot
            if r is not None and r.connected and m is not None:
                return True, "ok"
            time.sleep(0.1)
        with self._lock:
            r = self._latest_robot
            m = self._latest_marker
        if r is None:
            return False, "no robot_state received"
        if not r.connected:
            return False, "robot not connected"
        if m is None:
            return False, "no marker_detection received"
        return False, "system not ready"

    # ------------------------------------------------------------------ #
    def _auto_collect(self, goal_handle, target_n, settle, obs):
        poses = self._generate_pose_set() if self.auto_generate else None
        if poses is None:
            poses = self._load_calibration_poses()
        if not poses:
            self._set_state(FAILED, "no calibration poses configured")
            return False
        self.get_logger().info(f"auto collect: {len(poses)} poses, target {target_n} samples")
        self._dry_run_detected = False
        self._pose_validation = []          # (idx, ok) for the dry-run report
        for idx, entry in enumerate(poses):
            if self._cancel.is_set() or not rclpy.ok():
                return False
            self._wait_if_paused()
            with self._lock:
                if len(self._samples) >= target_n:
                    break
            self._current_pose_index = idx
            self._set_state(MOVING, f"moving to pose #{idx}")
            moved = self._send_move(idx, entry)
            self._pose_validation.append((idx, moved))
            if not moved:
                self.get_logger().warn(f"pose #{idx}: move failed/rejected, skipping")
                continue
            # In dry_run the robot never physically moves, so every sample would
            # share the same base_T_gripper (rejected as "0.0 deg"). Validate the
            # poses instead of collecting meaningless duplicates.
            if self._dry_run_detected:
                self.get_logger().info(f"pose #{idx}: DRY-RUN validated (no motion, no sample)")
                continue
            self._set_state(SETTLING, f"settling {settle:.1f}s at pose #{idx}")
            time.sleep(settle)
            self._collect_with_recovery(idx, entry, obs, settle)

        if self._dry_run_detected:
            self._report_dry_run()
            return False
        with self._lock:
            return len(self._samples) >= self.min_samples

    def _report_dry_run(self):
        ok = [i for i, good in self._pose_validation if good]
        bad = [i for i, good in self._pose_validation if not good]
        self.get_logger().warn("=" * 62)
        self.get_logger().warn(
            f"DRY-RUN COMPLETE: {len(ok)}/{len(self._pose_validation)} poses passed "
            "safety validation. No samples collected (the robot never moved).")
        if bad:
            self.get_logger().warn(f"REJECTED poses: {bad} -- fix these before going live.")
        else:
            self.get_logger().warn("All poses passed. To run a REAL calibration, relaunch with:")
            self.get_logger().warn(
                "  ros2 launch piper_auto_handeye real_calibration.launch.py dry_run:=false")
        self.get_logger().warn("=" * 62)

    def _send_move(self, idx, entry):
        """Send one calibration pose. ``entry`` is ("joints", [rad]) or ("pose", T)."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move action server unavailable")
            return False
        kind, value = entry
        goal = MoveToCalibrationPose.Goal()
        goal.pose_index = -1
        if kind == "joints":
            goal.target_joints = [float(v) for v in value]
        else:
            goal.target_pose = tu.matrix_to_pose_msg(value)
        goal.speed = 0.0
        goal.timeout = 0.0
        goal.dry_run = False  # honor the control node's master dry_run switch
        send_future = self._move_client.send_goal_async(goal)
        # NOTE: do NOT spin_until_future_complete here -- we are already inside an
        # executor callback. Poll instead; other executor threads service the future
        # (ReentrantCallbackGroup + MultiThreadedExecutor).
        deadline = time.time() + 10.0
        while not send_future.done() and time.time() < deadline:
            if self._cancel.is_set():
                return False
            time.sleep(0.02)
        gh = send_future.result() if send_future.done() else None
        if gh is None or not gh.accepted:
            self.get_logger().warn(
                f"move #{idx}: goal {'not accepted' if gh else 'send timed out'}")
            return False
        res_future = gh.get_result_async()
        deadline = time.time() + 60.0
        while not res_future.done() and time.time() < deadline:
            if self._cancel.is_set():
                gh.cancel_goal_async()
                return False
            time.sleep(0.05)
        if not res_future.done():
            self.get_logger().warn(f"move #{idx}: no result after 60s")
            return False
        res = res_future.result().result
        if not res.success:
            self.get_logger().warn(f"move #{idx} failed: {res.error_message}")
        # The control node reports dry_run moves as successful-but-not-commanded.
        # Detect it so we don't collect physically identical samples.
        if "dry_run" in (res.error_message or ""):
            self._dry_run_detected = True
        return res.success

    def _collect_at_pose(self, idx, obs, timeout=None, label="", stability_min=None):
        """One collection attempt at wherever the arm currently is.

        Returns one of the OUTCOME_* constants. The caller uses it to decide
        between moving the wrist (occluded) and simply trying again (unstable).

        ``stability_min`` overrides the acceptance gate: retries pass a lower
        threshold so a marker that is VISIBLE but jittery eventually yields a
        sample instead of stalling the run in WAITING_FOR_MARKER forever.
        """
        timeout = self.marker_timeout if timeout is None else timeout
        stability_min = (self.marker_stability_min
                         if stability_min is None else stability_min)
        tag = f"pose #{idx}{label}"
        self._set_state(WAITING_FOR_MARKER,
                        f"{tag}: waiting for stable marker ({timeout:.1f}s)")
        deadline = time.time() + timeout
        cam_poses = []
        sightings = 0                      # frames where the marker was found at all
        reason = "marker_timeout"
        last_stamp = None
        while time.time() < deadline and len(cam_poses) < obs:
            if self._cancel.is_set() or not rclpy.ok():
                return OUTCOME_ABORT
            robot_T, marker_T, dt, mdet, rstate = self._get_synced_pair()
            # This loop polls faster than the camera produces frames, so without
            # a stamp check the same detection is consumed several times. That
            # matters: averaging a frame with a copy of itself cancels no noise,
            # so observations_per_pose would buy far less accuracy than its name
            # promises, and 'sightings' would overcount what the detector saw.
            stamp = self._stamp_sec(mdet.header) if mdet is not None else None
            if stamp is not None and stamp == last_stamp:
                time.sleep(0.005)
                continue
            last_stamp = stamp
            # Count every frame the detector actually found the marker in, even
            # if we then reject it for quality. That is what separates "the
            # gripper is covering it" from "the detection is noisy".
            if mdet is not None and mdet.detected:
                sightings += 1
            if robot_T is None or marker_T is None:
                time.sleep(0.02)
                continue
            if rstate.moving:
                reason = "robot_moving"
                time.sleep(0.02)
                continue
            if dt > self.max_dt:
                reason = f"time_diff_{dt:.3f}s"
                time.sleep(0.02)
                continue
            if mdet.stability_score < stability_min:
                reason = f"low_stability_{mdet.stability_score:.2f}<{stability_min:.2f}"
                time.sleep(0.02)
                continue
            cam_poses.append(marker_T)
            self._set_state(COLLECTING, f"{tag}: {len(cam_poses)}/{obs} frames")
            time.sleep(0.02)

        if len(cam_poses) < max(1, obs // 2):
            occluded = sightings < self.recovery_min_sightings
            self._set_state(
                SAMPLE_REJECTED,
                f"{tag}: {reason} ({len(cam_poses)} good / {sightings} seen) -> "
                f"{'OCCLUDED, will reposition' if occluded else 'unstable, will retry'}")
            return OUTCOME_OCCLUDED if occluded else OUTCOME_UNSTABLE

        # representative camera_T_target = average of collected frames
        camera_T_target = tu.average_transforms(cam_poses)
        robot_T, _, dt, mdet, rstate = self._get_synced_pair()
        if robot_T is None:
            self._set_state(SAMPLE_REJECTED, f"{tag}: lost robot pose")
            return OUTCOME_NO_ROBOT
        added = self._try_add_sample(idx, robot_T, camera_T_target,
                                     reproj=mdet.reprojection_error if mdet else 0.0,
                                     stability=mdet.stability_score if mdet else 0.0,
                                     dt=dt)
        return OUTCOME_OK if added else OUTCOME_DUPLICATE

    # ------------------------------------------------------------------ #
    # marker recovery
    # ------------------------------------------------------------------ #
    def _recovery_offsets(self):
        """Wrist nudges to try, in order, when the marker cannot be seen.

        Ordered cheapest-and-most-likely first. Each entry is
        (dx, dy, dz, droll, dpitch, dyaw) applied to the nominal pose in the
        BASE frame; translations in m, rotations in rad.

        The reasoning, for an eye-in-hand camera looking past a gripper:
          1. back off  -- more standoff widens the field of view, so the marker
                          clears the fingers. Fixes the common case and barely
                          changes the orientation the pose was chosen for.
          2. yaw       -- swings the fingers sideways out of the line of sight.
          3. pitch     -- looks over or under the gripper.
          4. lift+yaw  -- combination for stubborn poses.
        Rotations stay small so the recovered pose keeps most of the
        orientation diversity the original pose set was designed for.
        """
        t = self.recovery_trans_step
        r = np.radians(self.recovery_rot_step_deg)
        return [
            ((0.0, 0.0,  t,   0.0,  0.0,  0.0), "back off"),
            ((0.0, 0.0,  t,   0.0,  0.0,  r),   "back off + yaw+"),
            ((0.0, 0.0,  t,   0.0,  0.0, -r),   "back off + yaw-"),
            ((0.0, 0.0,  2 * t, 0.0, r,   0.0), "further + pitch+"),
            ((0.0, 0.0,  2 * t, 0.0, -r,  0.0), "further + pitch-"),
            (( -t, 0.0,  2 * t, 0.0, 0.0, r),   "back + up + yaw+"),
        ]

    @staticmethod
    def _apply_offset(pose_T, offset):
        """Nominal pose + (translation in base frame, rotation in tool frame)."""
        dx, dy, dz, droll, dpitch, dyaw = offset
        R, t = tu.decompose_transform(pose_T)
        # Rotate about the tool's own axes so the nudge stays intuitive
        # regardless of how the wrist happens to be oriented.
        R_new = R @ tu.euler_to_matrix(droll, dpitch, dyaw)
        return tu.make_transform(R_new, t + np.array([dx, dy, dz], dtype=float))

    def _recovery_joint_offsets(self):
        """Joint nudges to try when the marker is hidden, in order.

        For a joint-space pose this is strictly better than nudging in Cartesian
        space: the wrist joints (4-6) swing the camera without displacing the
        arm's body at all, so the marker can be uncovered with almost no risk of
        hitting anything, and there is no IK step that could fail.

        Each entry is a list of per-joint deltas in degrees.
        """
        w = self.recovery_joint_step_deg
        return [
            ([0, 0, 0, 0, +w, 0], "wrist pitch+"),
            ([0, 0, 0, 0, -w, 0], "wrist pitch-"),
            ([0, 0, 0, 0, 0, +w * 1.5], "wrist yaw+"),
            ([0, 0, 0, 0, 0, -w * 1.5], "wrist yaw-"),
            ([0, 0, 0, +w, +w, 0], "wrist roll+pitch"),
            ([0, -w * 0.5, +w * 0.5, 0, +w, 0], "retract + wrist pitch"),
        ]

    @staticmethod
    def _apply_joint_offset(joints, offset_deg):
        return [q + math.radians(d) for q, d in zip(joints, offset_deg)]

    def _recovery_entries(self, entry):
        """Repositioned versions of ``entry``, cheapest-and-most-likely first."""
        kind, value = entry
        if kind == "joints":
            return [(("joints", self._apply_joint_offset(value, off)), f" [{name}]")
                    for off, name in self._recovery_joint_offsets()]
        return [(("pose", self._apply_offset(value, off)), f" [{name}]")
                for off, name in self._recovery_offsets()]

    def _collect_with_recovery(self, idx, entry, obs, settle):
        """Collect a sample at pose #idx, working around gripper occlusion.

        Two different failures, two different responses:

          * marker never seen      -> the gripper (or the arm's own body) is in
                                      the way. Reposition the wrist and retry.
          * marker seen but noisy  -> stay put and keep trying with a longer
                                      window; the detector just needs more time
                                      to settle.

        Returns True if a sample was stored.
        """
        attempts = [(None, "")]
        if self.recovery_enabled:
            attempts += self._recovery_entries(entry)[:self.recovery_max_attempts]

        for attempt_i, (recovery_entry, name) in enumerate(attempts):
            if self._cancel.is_set() or not rclpy.ok():
                return False
            self._wait_if_paused()

            if recovery_entry is not None:
                self._set_state(RECOVERING,
                                f"pose #{idx}: marker not visible, repositioning{name}")
                if not self._send_move(idx, recovery_entry):
                    # A rejected nudge is usually a workspace-bound hit; the next
                    # offset may still be reachable, so keep going rather than
                    # abandoning the pose.
                    self.get_logger().warn(
                        f"pose #{idx}: recovery move{name} rejected, trying the next one")
                    continue
                time.sleep(settle)

            # Persist while the marker is visible-but-jittery: each retry gets a
            # longer window AND a lower stability bar. A marker that stays
            # visible but noisy therefore converges to an accepted sample
            # instead of stalling; if even the relaxed retries fail, the outer
            # loop repositions the wrist -- a different viewing angle usually
            # fixes a persistently unstable detection.
            timeout = self.marker_timeout
            stab = self.marker_stability_min
            for retry in range(1 + max(0, self.retry_in_place)):
                suffix = name if retry == 0 else f"{name} retry {retry}"
                outcome = self._collect_at_pose(idx, obs, timeout, suffix,
                                                stability_min=stab)

                if outcome == OUTCOME_OK:
                    if recovery_entry is not None:
                        self.get_logger().info(
                            f"pose #{idx}: recovered{name} -- sample collected")
                    return True
                if outcome in (OUTCOME_ABORT, OUTCOME_NO_ROBOT):
                    return False
                if outcome == OUTCOME_DUPLICATE:
                    # The marker was seen fine; this pose just does not add new
                    # information. Repositioning would not change that.
                    return False
                if outcome == OUTCOME_OCCLUDED:
                    break            # waiting longer will not uncover the marker
                timeout *= self.retry_timeout_growth   # OUTCOME_UNSTABLE
                stab = max(0.35, stab * 0.8)           # relax the acceptance gate

        self.get_logger().warn(
            f"pose #{idx}: no usable marker view after {len(attempts)} position(s) "
            f"-> skipping this pose")
        return False

    def _get_synced_pair(self):
        """Return (base_T_gripper, camera_T_target, dt, marker_msg, robot_msg).

        Picks the robot pose whose timestamp is closest to the latest marker.
        """
        with self._lock:
            marker = self._latest_marker
            robots = list(self._robot_buf)
        if marker is None or not marker.detected or not robots:
            return None, None, 999.0, marker, None
        m_stamp = self._stamp_sec(marker.header)
        best = min(robots, key=lambda r: abs(r[0] - m_stamp))
        dt = abs(best[0] - m_stamp)
        robot_msg = best[1]
        base_T_gripper = tu.pose_msg_to_matrix(robot_msg.tcp_pose)
        camera_T_target = tu.pose_msg_to_matrix(marker.pose)
        return base_T_gripper, camera_T_target, dt, marker, robot_msg

    def _try_add_sample(self, idx, base_T_gripper, camera_T_target,
                        reproj, stability, dt):
        # reject near-duplicate orientation vs existing samples
        with self._lock:
            for s in self._samples:
                ang = np.degrees(tu.rotation_angle_between(s.base_T_gripper, base_T_gripper))
                if ang < self.min_rot_diff_deg:
                    self._state = SAMPLE_REJECTED
                    self.get_logger().info(
                        f"pose #{idx}: rejected, orientation too close to existing "
                        f"({ang:.1f} deg < {self.min_rot_diff_deg} deg)")
                    return False
            meta = {
                "timestamp": datetime.datetime.now().isoformat(),
                "reprojection_error": float(reproj),
                "marker_stability": float(stability),
                "robot_marker_dt": float(dt),
                "accepted": True,
            }
            self._samples.append(Sample(idx, base_T_gripper, camera_T_target, meta))
            n = len(self._samples)
        live = self._live_solve_summary()
        self._set_state(SAMPLE_ACCEPTED, f"pose #{idx}: sample {n} accepted "
                        f"(reproj={reproj:.2f}px stab={stability:.2f} dt={dt:.3f}s)"
                        + (f" | live: {live}" if live else ""))
        return True

    def _live_solve_summary(self):
        """Solve with the samples so far and return a one-line summary.

        Runs after every accepted sample so the log (and the GUI status line)
        shows the calibration CONVERGING in real time -- the translation
        settling and the RMS shrinking is the best live indicator that the run
        is healthy, and a wildly jumping result flags a bad setup immediately.
        Failures are swallowed: this is telemetry, never control flow.
        """
        with self._lock:
            base = [s.base_T_gripper for s in self._samples]
            cam = [s.camera_T_target for s in self._samples]
        if len(base) < 4:              # AX=XB needs >=3 motions to mean anything
            return ""
        try:
            res = solver.solve(base, cam, method=self.method,
                               min_samples=3, strict=False)
            val = validator.validate(base, res.gripper_T_camera, cam,
                                     self.max_t_rms, self.max_r_rms)
            t = res.gripper_T_camera[:3, 3] * 1000.0
            rpy = np.degrees(tu.matrix_to_euler(res.gripper_T_camera[:3, :3]))
            line = (f"t=[{t[0]:.1f} {t[1]:.1f} {t[2]:.1f}]mm "
                    f"rpy=[{rpy[0]:.1f} {rpy[1]:.1f} {rpy[2]:.1f}]deg "
                    f"rms={val.translation_rms_m * 1000:.1f}mm/"
                    f"{val.rotation_rms_deg:.2f}deg")
            self.get_logger().info(f"LIVE {self.method} n={len(base)}: {line}")
            # Hand the running estimate to the GUI so the Result panel updates
            # after every pose instead of staying blank until the run ends.
            with self._lock:
                self._live = {
                    "T": res.gripper_T_camera.copy(),
                    "n": len(base),
                    "t_rms": float(val.translation_rms_m),
                    "r_rms": float(val.rotation_rms_deg),
                }
            return line
        except Exception as exc:  # noqa: BLE001 - degenerate sample sets throw
            self.get_logger().debug(f"live solve skipped: {exc}")
            return ""

    # ------------------------------------------------------------------ #
    def _manual_collect(self, goal_handle, target_n):
        self._set_state(WAITING_FOR_MARKER,
                        "manual mode: move robot, then call add_manual_sample")
        while not self._cancel.is_set() and rclpy.ok():
            with self._lock:
                n = len(self._samples)
            self._publish_run_feedback(goal_handle, self._state,
                                       f"manual: {n}/{target_n} samples")
            if n >= target_n:
                return True
            time.sleep(0.3)
        return False

    def _add_current_pair(self):
        """Shared by manual service: validate + add the current synced pair."""
        robot_T, marker_T, dt, mdet, rstate = self._get_synced_pair()
        if robot_T is None or marker_T is None:
            return False, "no synced robot/marker pair available", None
        if rstate is not None and rstate.moving:
            return False, "robot is moving", None
        if not mdet.detected:
            return False, "marker not detected", None
        if dt > self.max_dt:
            return False, f"robot/marker time diff {dt:.3f}s > {self.max_dt}s", None
        if mdet.stability_score < self.marker_stability_min:
            return False, f"marker stability {mdet.stability_score:.2f} too low", None
        added = self._try_add_sample(-1, robot_T, marker_T,
                                     mdet.reprojection_error, mdet.stability_score, dt)
        with self._lock:
            n = len(self._samples)
        if not added:
            return False, "rejected (near-duplicate orientation)", n
        return True, "sample added", n

    # ------------------------------------------------------------------ #
    def _solve_one(self, base, cam, method):
        """Solve + validate with a single method, outlier removal included."""
        def solve_fn(b, c):
            return solver.solve(b, c, method=method,
                                min_samples=self.min_samples,
                                strict=False).gripper_T_camera

        res = solver.solve(base, cam, method=method, min_samples=self.min_samples)
        val = validator.validate(base, res.gripper_T_camera, cam,
                                 self.max_t_rms, self.max_r_rms)

        if self.enable_outlier and val.status != "SUCCESS" and len(base) > self.min_samples:
            # Scale the budget with the run. A fixed 3 was fine for a 10-sample
            # run and far too tight for 30: the sweep takes several shots at one
            # arm configuration, so a single badly-seen viewpoint contributes
            # that many outliers AT ONCE. Three removals could not clear one bad
            # viewpoint, and the run failed on 26 good samples.
            budget = max(self.max_removals,
                         int(self.outlier_fraction * len(base)))
            gtc, kb, kc, removed, hist = validator.remove_outliers(
                base, cam, solve_fn, self.max_t_rms, self.max_r_rms,
                budget, max(self.min_samples, 8))
            if removed:
                self.get_logger().info(
                    f"{method}: removed {len(removed)} outlier sample(s): {removed}")
                res = solver.solve(kb, kc, method=method,
                                   min_samples=self.min_samples, strict=False)
                val = validator.validate(kb, res.gripper_T_camera, kc,
                                         self.max_t_rms, self.max_r_rms)
        return res, val

    @staticmethod
    def _score(val):
        """Rank methods by a single number: normalised translation + rotation RMS.

        Neither RMS alone is enough -- a method can win on millimetres while
        being clearly worse in degrees. Dividing each by its own threshold puts
        them on a common scale before adding.
        """
        return (val.translation_rms_m / 0.01) + (val.rotation_rms_deg / 1.0)

    def _solve_and_validate(self, method):
        """Run EVERY method, report each as it finishes, keep the best.

        Picking an algorithm up front is guesswork: which of Tsai/Park/Horaud/
        Andreff/Daniilidis wins depends on the noise and pose distribution of
        the particular data set. They all run on the SAME samples in well under
        a second, so there is no reason not to try them all and let the
        closed-loop residual decide.

        ``method`` is still honoured as a tie-break preference and as the
        fallback if every method fails.
        """
        with self._lock:
            base = [s.base_T_gripper for s in self._samples]
            cam = [s.camera_T_target for s in self._samples]

        results = {}
        self._method_table = []          # published in CalibrationStatus.message
        for m in solver.METHODS:
            try:
                res, val = self._solve_one(base, cam, m)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"{m}: failed ({exc})")
                self._method_table.append((m, None, None, "FAILED"))
                self._publish_method_progress()
                continue
            results[m] = (res, val)
            self._method_table.append((m, val.translation_rms_m,
                                       val.rotation_rms_deg, val.status))
            self.get_logger().info(
                f"  {m:<11} t_rms={val.translation_rms_m*1000:8.3f} mm  "
                f"r_rms={val.rotation_rms_deg:8.4f} deg  "
                f"n={val.sample_count:2d}  {val.status}")
            self._publish_method_progress()

        if not results:
            raise RuntimeError("every calibration method failed")

        best_m = min(results, key=lambda m: (self._score(results[m][1]),
                                             m != method))
        res, val = results[best_m]
        self._last_method = best_m

        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"BEST METHOD: {best_m}  (requested '{method}' was "
            f"{'confirmed' if best_m == method else 'beaten'})")
        self.get_logger().info(
            f"RESULT gripper_T_camera t={res.translation} "
            f"| t_rms={val.translation_rms_m*1000:.3f}mm "
            f"r_rms={val.rotation_rms_deg:.4f}deg | status={val.status}")
        self.get_logger().info("=" * 62)
        for w in res.warnings:
            self.get_logger().warn(f"solver: {w}")
        return res, val

    def _publish_method_progress(self):
        """Push the per-method table out on calibration_status as it fills in."""
        parts = []
        for m, t_rms, r_rms, status in self._method_table:
            if t_rms is None:
                parts.append(f"{m}=FAILED")
            else:
                parts.append(f"{m}={t_rms*1000:.2f}mm/{r_rms:.3f}deg")
        self._status_message = "SOLVING | " + "  ".join(parts)
        self._publish_status()

    def _save_result(self, solve_result, val, method, output_path):
        # ONE well-known file, overwritten on every save. Timestamped copies
        # and sample dumps used to pile up here; now the newest result is
        # always the same path, in the minimal easy_handeye2-style format.
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        path = output_path if output_path else os.path.join(
            self.output_dir, cio.RESULT_FILENAME)
        validation = {
            "sample_count": val.sample_count,
            "translation_rms_m": round(val.translation_rms_m, 6),
            "rotation_rms_deg": round(val.rotation_rms_deg, 5),
        }
        d = cio.build_result_dict(
            solve_result.gripper_T_camera, self.gripper_frame, self.camera_frame,
            method, ts, validation, {})
        cio.save_result(d, path)
        self.get_logger().info(f"saved calibration -> {path} (overwritten)")
        return path

    def _save_samples_only(self, method):
        """Dump the raw samples of a FAILED run, without a result file.

        Deliberately timestamped rather than overwriting: failures are compared
        against each other, and one that clobbered the previous one would hide
        exactly the pattern you are looking for.
        """
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        path = os.path.join(self.output_dir, f"failed_{method.lower()}_{ts}_samples.yaml")
        with self._lock:
            rows = [cio.sample_to_dict(s.pose_index, s.base_T_gripper,
                                       s.camera_T_target, s.meta)
                    for s in self._samples]
        cio.save_samples(rows, path)
        self.get_logger().info(
            f"run FAILED -- wrote {len(rows)} raw samples to {path} "
            f"(no result file: a failed solve is not a calibration to load)")
        return path

    def _load_calibration_poses(self):
        """Load Cartesian calibration poses from calibration_poses.yaml.

        Reads the file directly (ROS 2 flattens nested list-of-dict params
        unreliably). Falls back to built-in placeholder poses if no file.
        """
        path = self.calibration_poses_file
        if not path or not os.path.exists(path):
            self.get_logger().warn(
                f"calibration_poses_file not found ('{path}'); using built-in placeholders")
            return self._builtin_poses()
        try:
            import yaml
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            # tolerate both {'/**': {'ros__parameters': {'poses': [...]}}} and {'poses': [...]}
            node_params = data.get("/**", {}).get("ros__parameters", data)
            raw = node_params.get("poses", [])
            poses = []
            n_joint = 0
            for item in raw:
                # A `joints:` entry is a joint-space target [rad]. Prefer it:
                # it needs no IK, so it cannot be silently unreachable the way a
                # hand-written Cartesian pose can. Cartesian entries still work.
                if item.get("joints"):
                    poses.append(("joints", [float(v) for v in item["joints"]]))
                    n_joint += 1
                else:
                    poses.append(("pose", tu.make_transform(
                        tu.euler_to_matrix(*item["rpy"]), item["position"])))
            if not poses:
                raise ValueError("no poses in file")
            kind = (f"{n_joint} joint-space, {len(poses) - n_joint} Cartesian"
                    if n_joint else "all Cartesian")
            self.get_logger().info(
                f"loaded {len(poses)} calibration poses from {path} ({kind})")
            if n_joint == 0:
                self.get_logger().warn(
                    "all poses are Cartesian -- if the arm accepts moves but never "
                    "budges, they are probably unreachable in its own kinematics. "
                    "Generate a joint-space set with: "
                    "ros2 run piper_auto_handeye generate_poses")
            return poses
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to load poses ({exc}); using built-in placeholders")
            return self._builtin_poses()

    def _builtin_poses(self):
        """Fallback placeholder poses if none provided (mock testing)."""
        base = [
            ([0.35, 0.0, 0.35], [-3.14, 0.0, 0.0]),
            ([0.33, 0.10, 0.35], [-3.14, 0.20, 0.0]),
            ([0.33, -0.10, 0.35], [-3.14, -0.20, 0.0]),
            ([0.38, 0.0, 0.40], [-2.95, 0.0, 0.15]),
            ([0.32, 0.0, 0.30], [3.05, 0.0, -0.15]),
            ([0.35, 0.08, 0.36], [-3.14, 0.10, 0.25]),
            ([0.35, -0.08, 0.36], [-3.14, -0.10, -0.25]),
            ([0.42, 0.0, 0.38], [-3.10, 0.05, 0.05]),
            ([0.30, 0.12, 0.33], [-3.14, 0.18, 0.20]),
            ([0.30, -0.12, 0.33], [-3.14, -0.18, -0.20]),
            ([0.37, 0.10, 0.42], [-2.98, 0.15, 0.18]),
            ([0.37, -0.10, 0.42], [-2.98, -0.15, -0.18]),
        ]
        return [("pose", tu.make_transform(tu.euler_to_matrix(*rpy), pos))
                for pos, rpy in base]

    def request_shutdown(self):
        """Ask any in-flight calibration to unwind promptly (used on Ctrl+C)."""
        self._cancel.set()
        self._pause.clear()

    def _wait_if_paused(self):
        while self._pause.is_set() and not self._cancel.is_set():
            self._set_state(PAUSED, "paused")
            time.sleep(0.2)

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #
    def _srv_reset(self, req, resp):
        # Reset = STOP NOW, forget everything, go back to the start pose.
        # Order matters: cancel first so a running move aborts, then wait for
        # the run to actually release the motion lock, then home.
        self._cancel.set()
        self.get_logger().warn("reset: cancelling any running motion")
        got_lock = self._motion_lock.acquire(timeout=10.0)

        with self._lock:
            self._samples.clear()
            self._live = None
            self._state = IDLE
            self._last_result = None
            self._last_validation = None
        self.get_logger().info("reset_calibration: cleared all samples")

        if not got_lock:
            self.get_logger().warn(
                "reset: a move is still winding down; homing will contend with it")

        moved = False
        try:
            self._cancel.clear()      # allow the homing move itself
            # A latched STOP (GUI stop button) silently rejects every move,
            # including this homing -- clear it first, best effort.
            if self._clear_stop_cli.wait_for_service(timeout_sec=1.0):
                fut = self._clear_stop_cli.call_async(Trigger.Request())
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 2.0:
                    time.sleep(0.02)
            self._set_state(HOMING, "reset: returning to the taught home pose")
            moved = self._send_move(-1, ("joints", self.home_joints))
            if not moved:
                self.get_logger().warn(
                    "reset: homing move rejected/failed -- check the control "
                    "node log for the safety reason")
            self._set_state(IDLE, "reset complete")
        finally:
            if got_lock:
                self._motion_lock.release()

        resp.success = True
        resp.message = ("reset: stopped, cleared, returned to home" if moved
                        else "reset: stopped and cleared (homing move failed)")
        return resp

    def _srv_pause(self, req, resp):
        self._pause.set()
        resp.success = True
        resp.message = "calibration paused (will hold before the next pose)"
        self.get_logger().warn("PAUSE requested")
        return resp

    def _srv_resume(self, req, resp):
        self._pause.clear()
        resp.success = True
        resp.message = "calibration resumed"
        self.get_logger().info("RESUME requested")
        return resp

    def _srv_add_manual(self, req, resp):
        ok, msg, n = self._add_current_pair()
        resp.success = ok
        resp.message = msg
        resp.sample_count = n if n is not None else len(self._samples)
        return resp

    def _srv_save(self, req, resp):
        if self._last_result is None or self._last_validation is None:
            resp.success = False
            resp.message = "no calibration to save (run a calibration first)"
            resp.saved_path = ""
            return resp
        path = self._save_result(self._last_result, self._last_validation,
                                 self._last_result.method, req.path)
        resp.success = True
        resp.message = "saved"
        resp.saved_path = path
        return resp

    def _srv_load(self, req, resp):
        path = req.path if req.path else self._latest_result_path()
        if not path or not os.path.exists(path):
            resp.success = False
            resp.message = f"file not found: {path}"
            return resp
        try:
            d = cio.load_result(path)
            T, parent, child = cio.result_to_transform(d)
            resp.transform = tu.matrix_to_transform_msg(T)
            resp.parent_frame = parent
            resp.child_frame = child
            resp.success = True
            resp.message = f"loaded {path}"
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.message = f"load failed: {exc}"
        return resp

    def _latest_result_path(self):
        try:
            files = [os.path.join(self.output_dir, f) for f in os.listdir(self.output_dir)
                     if f.startswith("handeye_") and f.endswith(".yaml")
                     and not f.endswith("_samples.yaml")]
            return max(files, key=os.path.getmtime) if files else ""
        except Exception:  # noqa: BLE001
            return ""


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCalibrationNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Tell the running calibration to unwind BEFORE the context is torn down,
        # otherwise its loops keep touching destroyed entities ("context is
        # invalid" / "cannot use Destroyable").
        node.get_logger().info("shutdown requested; stopping calibration...")
        node.request_shutdown()
    finally:
        node.request_shutdown()
        time.sleep(0.3)          # let the action callback exit its loops
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

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
SOLVING = "SOLVING"
VALIDATING = "VALIDATING"
SUCCESS = "SUCCESS"
WARNING = "WARNING"
PAUSED = "PAUSED"
CANCELED = "CANCELED"
FAILED = "FAILED"


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

        cb = ReentrantCallbackGroup()
        self.create_subscription(MarkerDetection, "marker_detection",
                                 self._marker_cb, 10, callback_group=cb)
        self.create_subscription(RobotState, "robot_state",
                                 self._robot_cb, 10, callback_group=cb)
        self.status_pub = self.create_publisher(CalibrationStatus, "calibration_status", 10)
        self.create_timer(0.2, self._publish_status)

        self._move_client = ActionClient(self, MoveToCalibrationPose,
                                         "move_to_calibration_pose", callback_group=cb)

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
        target_n = g.target_sample_count if g.target_sample_count > 0 else self.target_samples
        method = g.calibration_method if g.calibration_method else self.method
        settle = g.settle_time if g.settle_time > 0 else self.settle_time
        obs = g.observations_per_pose if g.observations_per_pose > 0 else self.obs_per_pose

        result = RunCalibration.Result()

        # ---- system check ----
        self._set_state(CHECKING_SYSTEM, "verifying camera/marker/robot streams")
        ok, why = self._system_check()
        if not ok:
            return self._finish_run(goal_handle, result, FAILED, method, why)

        if g.auto_move:
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

        result.gripper_to_camera = tu.matrix_to_transform_msg(solve_result.gripper_T_camera)
        result.sample_count = solve_result.sample_count
        result.translation_rms_m = val.translation_rms_m
        result.translation_max_m = val.translation_max_m
        result.rotation_rms_deg = val.rotation_rms_deg
        result.rotation_max_deg = val.rotation_max_deg
        result.saved_path = saved_path
        return self._finish_run(goal_handle, result, final_state, method,
                                "; ".join(val.messages) or "ok")

    def _go_to_rest(self, why=""):
        """Park the arm at the configured rest pose. Best-effort, never raises.

        Called at the end of every run and on reset, so it must not be able to
        turn a successful calibration into a failure: a rest move that is
        rejected or times out is logged and swallowed.
        """
        if not self.return_to_rest:
            return False
        if not any(self.rest_position):
            self.get_logger().info("no rest_position configured; leaving the arm in place")
            return False
        try:
            rest_T = tu.make_transform(tu.euler_to_matrix(*self.rest_rpy),
                                       self.rest_position)
            self.get_logger().info(
                f"returning to rest pose {self.rest_position}"
                + (f" ({why})" if why else ""))
            self._current_pose_index = -1
            ok = self._send_move(-1, rest_T)
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
        poses = self._load_calibration_poses()
        if not poses:
            self._set_state(FAILED, "no calibration poses configured")
            return False
        self.get_logger().info(f"auto collect: {len(poses)} poses, target {target_n} samples")
        self._dry_run_detected = False
        self._pose_validation = []          # (idx, ok) for the dry-run report
        for idx, pose_T in enumerate(poses):
            if self._cancel.is_set() or not rclpy.ok():
                return False
            self._wait_if_paused()
            with self._lock:
                if len(self._samples) >= target_n:
                    break
            self._current_pose_index = idx
            self._set_state(MOVING, f"moving to pose #{idx}")
            moved = self._send_move(idx, pose_T)
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
            self._collect_at_pose(idx, obs)

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

    def _send_move(self, idx, pose_T):
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move action server unavailable")
            return False
        goal = MoveToCalibrationPose.Goal()
        goal.pose_index = -1
        goal.target_pose = tu.matrix_to_pose_msg(pose_T)
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
            return False
        res_future = gh.get_result_async()
        deadline = time.time() + 60.0
        while not res_future.done() and time.time() < deadline:
            if self._cancel.is_set():
                gh.cancel_goal_async()
                return False
            time.sleep(0.05)
        if not res_future.done():
            return False
        res = res_future.result().result
        # The control node reports dry_run moves as successful-but-not-commanded.
        # Detect it so we don't collect physically identical samples.
        if "dry_run" in (res.error_message or ""):
            self._dry_run_detected = True
        return res.success

    def _collect_at_pose(self, idx, obs):
        self._set_state(WAITING_FOR_MARKER, f"pose #{idx}: waiting for stable marker")
        deadline = time.time() + self.marker_timeout
        cam_poses = []
        reason = "marker_timeout"
        while time.time() < deadline and len(cam_poses) < obs:
            if self._cancel.is_set() or not rclpy.ok():
                return
            robot_T, marker_T, dt, mdet, rstate = self._get_synced_pair()
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
            if mdet.stability_score < self.marker_stability_min:
                reason = f"low_stability_{mdet.stability_score:.2f}"
                time.sleep(0.02)
                continue
            cam_poses.append(marker_T)
            self._set_state(COLLECTING, f"pose #{idx}: {len(cam_poses)}/{obs} frames")
            time.sleep(0.02)

        if len(cam_poses) < max(1, obs // 2):
            self._set_state(SAMPLE_REJECTED, f"pose #{idx}: {reason} "
                            f"({len(cam_poses)} frames) -> {self.on_marker_timeout}")
            return

        # representative camera_T_target = average of collected frames
        camera_T_target = tu.average_transforms(cam_poses)
        robot_T, _, dt, mdet, rstate = self._get_synced_pair()
        if robot_T is None:
            self._set_state(SAMPLE_REJECTED, f"pose #{idx}: lost robot pose")
            return
        self._try_add_sample(idx, robot_T, camera_T_target,
                             reproj=mdet.reprojection_error if mdet else 0.0,
                             stability=mdet.stability_score if mdet else 0.0,
                             dt=dt)

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
        self._set_state(SAMPLE_ACCEPTED, f"pose #{idx}: sample {n} accepted "
                        f"(reproj={reproj:.2f}px stab={stability:.2f} dt={dt:.3f}s)")
        return True

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
            gtc, kb, kc, removed, hist = validator.remove_outliers(
                base, cam, solve_fn, self.max_t_rms, self.max_r_rms,
                self.max_removals, max(self.min_samples, 8))
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
        ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        path = output_path if output_path else os.path.join(
            self.output_dir, f"handeye_{method.lower()}_{ts}.yaml")
        validation = {
            "sample_count": val.sample_count,
            "translation_rms_m": round(val.translation_rms_m, 6),
            "translation_max_m": round(val.translation_max_m, 6),
            "rotation_rms_deg": round(val.rotation_rms_deg, 5),
            "rotation_max_deg": round(val.rotation_max_deg, 5),
        }
        source = {
            "robot": "Piper",
            "camera": "RealSense",
            "method": method,
        }
        d = cio.build_result_dict(
            solve_result.gripper_T_camera, self.gripper_frame, self.camera_frame,
            method, ts, validation, source)
        cio.save_result(d, path)
        # also dump raw samples
        with self._lock:
            samples = [cio.sample_to_dict(s.pose_index, s.base_T_gripper,
                                          s.camera_T_target, s.meta)
                       for s in self._samples]
        cio.save_samples(samples, path.replace(".yaml", "_samples.yaml"))
        self.get_logger().info(f"saved calibration -> {path}")
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
            for item in raw:
                pos = item["position"]
                rpy = item["rpy"]
                poses.append(tu.make_transform(tu.euler_to_matrix(*rpy), pos))
            if not poses:
                raise ValueError("no poses in file")
            self.get_logger().info(f"loaded {len(poses)} calibration poses from {path}")
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
        return [tu.make_transform(tu.euler_to_matrix(*rpy), pos) for pos, rpy in base]

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
        with self._lock:
            self._samples.clear()
            self._state = IDLE
            self._last_result = None
            self._last_validation = None
        self.get_logger().info("reset_calibration: cleared all samples")

        # Reset means "start over", so the arm belongs back at rest, not parked
        # at whatever pose the aborted run left it in.
        moved = self._go_to_rest(why="reset")

        resp.success = True
        resp.message = ("calibration reset; returned to rest pose" if moved
                        else "calibration reset")
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

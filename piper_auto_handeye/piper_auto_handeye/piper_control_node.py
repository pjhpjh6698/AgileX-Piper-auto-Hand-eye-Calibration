#!/usr/bin/env python3
"""Piper control adapter.

Exposes a clean RobotState stream and a SAFETY-validated MoveToCalibrationPose
action, on top of one of two interchangeable backends:

``agx`` (default) -- REAL ROBOT
    Talks to the vendored AgileX ROS 2 driver, ``agx_arm_ctrl``, which drives
    the arm over CAN through the pyAgxArm SDK. All standard message types.
    Note the two different stops this node offers: ``stop_motion`` holds the
    current pose (safe, the calibration stop), while ``hard_stop`` forwards to
    the driver's ``emergency_stop`` and drops the arm.

      reads  feedback/tcp_pose     (PoseStamped)  -> base_T_gripper
             feedback/joint_states (JointState)   -> joint_positions
             feedback/arm_status   (AgxArmStatus) -> faults
      writes control/move_p        (PoseStamped)  -> Cartesian goal
      calls  enable_agx_arm        (SetBool)      -> power the joints
             <driver>/set_parameters               -> per-goal speed_percent

``topic`` -- SIMULATION
    The older AgileX ``/pos_cmd`` interface. ``mock_robot_node`` and the Gazebo
    driver emulate exactly this, so the sim launches select it. It is not used
    against hardware any more; see the workspace README for why.

      reads  end_pose_topic    (PoseStamped)    -> base_T_gripper
             arm_status_topic  (PiperStatusMsg) -> enabled / error
             joint_state_topic (JointState)     -> joint_positions
      writes pos_cmd_topic     (PosCmd)         -> Cartesian goal
             enable_topic      (Bool)           -> enable arm

Both backends feed the SAME internal state machine (``_ingest_*``) and the same
action server, so behaviour verified in Gazebo carries over to the real arm.
dry_run is the master safety switch for both.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from auto_handeye_interfaces.action import MoveToCalibrationPose
from auto_handeye_interfaces.msg import RobotState

from . import transform_utils as tu
from . import agx_status
from . import agx_kinematics as ak
from .safety_validator import SafetyValidator, SafetyLimits

try:
    from agx_arm_msgs.msg import AgxArmStatus
    _HAVE_AGX_MSGS = True
except ImportError:  # only the sim-only topic backend can run without these
    AgxArmStatus = None
    _HAVE_AGX_MSGS = False

try:
    from piper_msgs.msg import PosCmd, PiperStatusMsg
    _HAVE_PIPER_MSGS = True
except ImportError:  # only the simulation backend needs these
    PosCmd = None
    PiperStatusMsg = None
    _HAVE_PIPER_MSGS = False


class PiperControlNode(Node):
    def __init__(self):
        super().__init__("piper_control_node")
        p = self.declare_parameter

        self.backend = str(p("control_backend", "agx").value).lower()
        if self.backend not in ("agx", "topic"):
            raise ValueError(f"control_backend must be 'agx' or 'topic', got {self.backend!r}")

        # --- agx backend (real robot, via agx_arm_ctrl) ---
        self.agx_driver_node = p("agx_driver_node", "/agx_arm_ctrl_single_node").value
        self.agx_tcp_pose_topic = p("agx_tcp_pose_topic", "feedback/tcp_pose").value
        self.agx_joint_state_topic = p("agx_joint_state_topic", "feedback/joint_states").value
        self.agx_arm_status_topic = p("agx_arm_status_topic", "feedback/arm_status").value
        self.agx_enable_service = p("agx_enable_service", "enable_agx_arm").value
        self.agx_estop_service = p("agx_estop_service", "emergency_stop").value
        # 'P' = point-to-point, 'L' = straight line. P is the default: calibration
        # poses differ a lot in orientation, where forcing a straight Cartesian
        # path invites singularities and joint-limit aborts mid-move.
        self.agx_move_mode = str(p("agx_move_mode", "P").value).upper()
        if self.agx_move_mode not in ("P", "L"):
            raise ValueError(f"agx_move_mode must be 'P' or 'L', got {self.agx_move_mode!r}")
        self.agx_set_speed = bool(p("agx_set_speed", True).value)
        self.agx_service_timeout = float(p("agx_service_timeout", 3.0).value)
        # joint-space goals
        self.joint_names = list(p("joint_names",
                                  ["joint1", "joint2", "joint3",
                                   "joint4", "joint5", "joint6"]).value)
        self.joint_goal_tolerance_deg = float(p("joint_goal_tolerance_deg", 1.0).value)
        self.joint_limit_margin_deg = float(p("joint_limit_margin_deg", 2.0).value)

        # --- topic backend (simulation) ---
        self.end_pose_topic = p("end_pose_topic", "/end_pose_stamped").value
        self.arm_status_topic = p("arm_status_topic", "/arm_status").value
        self.joint_state_topic = p("joint_state_topic", "/joint_states_single").value
        self.pos_cmd_topic = p("pos_cmd_topic", "/pos_cmd").value
        self.enable_topic = p("enable_topic", "/enable_flag").value

        self.base_frame = p("base_frame", "base_link").value
        self.gripper_frame = p("gripper_frame", "link6").value

        self.dry_run = bool(p("dry_run", True).value)
        ws_min = list(p("workspace_min", [0.05, -0.45, 0.05]).value)
        ws_max = list(p("workspace_max", [0.65, 0.45, 0.70]).value)
        self.max_step = float(p("max_step_distance", 0.35).value)
        self.max_speed = float(p("max_speed_fraction", 0.4).value)
        self.default_speed = float(p("default_speed_fraction", 0.2).value)
        self.move_timeout = float(p("movement_timeout", 30.0).value)
        self.pos_tol = float(p("goal_position_tolerance", 0.01).value)
        self.ori_tol_deg = float(p("goal_orientation_tolerance_deg", 2.0).value)
        self.settle_check = float(p("settle_check_time", 0.5).value)
        self.stopped_t_eps = float(p("stopped_translation_eps", 0.002).value)
        self.stopped_r_eps = float(p("stopped_rotation_eps_deg", 0.5).value)
        self.require_enabled = bool(p("require_enabled_to_move", True).value)
        self.auto_enable = bool(p("auto_enable", True).value)
        self.pose_timeout = float(p("pose_timeout", 1.0).value)
        # Seconds of silence after which a dry run is considered finished, so
        # the next one starts measuring from the arm's real pose again.
        self.dry_chain_timeout = float(p("dry_run_chain_timeout", 5.0).value)

        self.safety = SafetyValidator(SafetyLimits(
            ws_min, ws_max, self.max_step, self.max_speed))

        # state
        self._lock = threading.Lock()
        self._last_pose_T = None            # base_T_gripper (4x4)
        self._last_pose_time = None         # ros Time of last end_pose
        self._prev_pose_for_motion = None
        self._prev_motion_time = None
        self._is_moving = False
        self._enabled = False
        self._enable_requested = False
        self._err_text = ""
        self._joint_positions = []
        self._stop_requested = False   # set by the stop_motion service / GUI STOP
        # Simulated TCP during a dry run, so the step-distance check sees the
        # real pose-to-pose sequence instead of measuring everything from where
        # the arm happens to be parked. See _execute_move.
        self._dry_pose = None
        self._dry_pose_time = 0.0

        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.pos_cmd_pub = None      # topic backend
        self.enable_pub = None       # topic backend
        self.move_pub = None         # agx backend, Cartesian
        self.move_j_pub = None       # agx backend, joint space
        self.enable_cli = None       # agx backend
        self.estop_cli = None        # agx backend
        self.speed_cli = None        # agx backend
        self._agx_speed_pct = None   # last speed pushed to the driver

        if self.backend == "agx":
            self._setup_agx_backend(cb)
        else:
            self._setup_topic_backend(cb)

        self.state_pub = self.create_publisher(RobotState, "robot_state", 10)
        # 50 Hz: the collector time-syncs robot_state against 30 Hz marker
        # frames with a 0.2 s gate; at the old 10 Hz the sync margin was thin
        # and contributed to WAITING_FOR_MARKER stalls.
        self.create_timer(0.02, self._publish_state)

        # STOP / RESUME services (wired to the GUI stop button)
        self.create_service(Trigger, "stop_motion", self._srv_stop, callback_group=cb)
        self.create_service(Trigger, "clear_stop", self._srv_clear_stop, callback_group=cb)
        # NOT "emergency_stop": agx_arm_ctrl already owns that name with an
        # incompatible type (Empty), so both nodes in the same namespace would
        # collide -- and our own client would end up calling us instead of the
        # driver. "hard_stop" keeps the two distinct.
        self.create_service(Trigger, "hard_stop", self._srv_estop, callback_group=cb)

        self._action = ActionServer(
            self, MoveToCalibrationPose, "move_to_calibration_pose",
            execute_callback=self._execute_move,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb)

        mode = "DRY-RUN (no motion)" if self.dry_run else "LIVE (robot WILL move)"
        detail = (f"driver={self.agx_driver_node} move={self.agx_move_mode}"
                  if self.backend == "agx" else f"driver topics {self.pos_cmd_topic}")
        self.get_logger().info(
            f"piper_control_node up | backend={self.backend} ({detail}) | {mode}")
        self.get_logger().info(f"base='{self.base_frame}' gripper='{self.gripper_frame}' "
                               f"workspace {ws_min}..{ws_max}")

    # ------------------------------------------------------------------ #
    # backend setup
    # ------------------------------------------------------------------ #
    def _setup_agx_backend(self, cb):
        """Bind to the vendored AgileX driver (agx_arm_ctrl)."""
        if not _HAVE_AGX_MSGS:
            raise RuntimeError(
                "control_backend:=agx needs agx_arm_msgs, which is not on the path.\n"
                "  colcon build --packages-select agx_arm_msgs agx_arm_ctrl\n"
                "  source install/setup.bash")

        self.create_subscription(PoseStamped, self.agx_tcp_pose_topic,
                                 self._end_pose_cb, 10, callback_group=cb)
        self.create_subscription(JointState, self.agx_joint_state_topic,
                                 self._joint_cb, 10, callback_group=cb)
        self.create_subscription(AgxArmStatus, self.agx_arm_status_topic,
                                 self._agx_status_cb, 10, callback_group=cb)

        move_topic = f"control/move_{self.agx_move_mode.lower()}"
        self.move_pub = self.create_publisher(PoseStamped, move_topic, 10)
        # Joint-space goals go to a different topic and skip the arm's IK
        # entirely -- see _execute_move for why that is the preferred path.
        self.move_j_pub = self.create_publisher(JointState, "control/move_j", 10)

        self.enable_cli = self.create_client(SetBool, self.agx_enable_service,
                                             callback_group=cb)
        self.estop_cli = self.create_client(EmptySrv, self.agx_estop_service,
                                            callback_group=cb)
        # speed_percent lives on the driver node, not in the goal message, so a
        # per-goal speed has to be pushed as a parameter before publishing.
        self.speed_cli = self.create_client(
            SetParameters, f"{self.agx_driver_node}/set_parameters", callback_group=cb)

        self.get_logger().info(
            f"agx backend: <- {self.agx_tcp_pose_topic}, -> {move_topic}, "
            f"enable via {self.agx_enable_service}")

    def _setup_topic_backend(self, cb):
        self.create_subscription(PoseStamped, self.end_pose_topic,
                                 self._end_pose_cb, 10, callback_group=cb)
        self.create_subscription(JointState, self.joint_state_topic,
                                 self._joint_cb, 10, callback_group=cb)
        if _HAVE_PIPER_MSGS:
            self.create_subscription(PiperStatusMsg, self.arm_status_topic,
                                     self._status_cb, 10, callback_group=cb)
            self.pos_cmd_pub = self.create_publisher(PosCmd, self.pos_cmd_topic, 10)
        else:
            self.get_logger().warn(
                "piper_msgs not available: cannot command the real driver "
                "(mock_robot_node provides its own PosCmd). Move will fail unless mock is used.")
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)

    # ------------------------------------------------------------------ #
    # state ingestion -- both backends land here
    # ------------------------------------------------------------------ #
    def _ingest_pose(self, T):
        now = self.get_clock().now()
        with self._lock:
            self._last_pose_T = T
            self._last_pose_time = now
            # motion detection from frame-to-frame delta
            if self._prev_pose_for_motion is not None:
                dt = tu.translation_distance(self._prev_pose_for_motion, T)
                dr = math.degrees(tu.rotation_angle_between(self._prev_pose_for_motion, T))
                self._is_moving = (dt > self.stopped_t_eps or dr > self.stopped_r_eps)
            self._prev_pose_for_motion = T
            self._prev_motion_time = now

    def _ingest_joints(self, positions):
        with self._lock:
            self._joint_positions = list(positions)

    def _ingest_status(self, err_text, enabled):
        with self._lock:
            self._err_text = err_text
            self._enabled = bool(enabled)

    # ---- shared by both backends (tcp_pose / end_pose are both PoseStamped) ---- #
    def _end_pose_cb(self, msg: PoseStamped):
        self._ingest_pose(tu.pose_msg_to_matrix(msg.pose))

    def _joint_cb(self, msg: JointState):
        self._ingest_joints(msg.position)

    # ---- agx backend ---- #
    def _agx_status_cb(self, msg):
        err = agx_status.status_text(msg)
        if agx_status.is_teaching(msg):
            # Drag-teach mode silently swallows motion goals; surfacing it as a
            # fault is what stops a calibration run from stalling on "no arrival".
            err = "; ".join(filter(None, [err, "arm is in drag-teach mode"]))
        # The driver has no "are the drives powered" topic. Track what we asked
        # for, and treat any fault as not-ready.
        self._ingest_status(err, self._enable_requested and not err)

    # ---- topic backend ---- #
    def _status_cb(self, msg):
        err = int(msg.err_code)
        # heuristic: driver reports enabled once arm_status indicates normal ops.
        self._ingest_status("" if err == 0 else f"piper err_code={err}",
                            self._enable_requested and err == 0)

    # ------------------------------------------------------------------ #
    def _connected(self):
        if self._last_pose_time is None:
            return False
        age = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
        return age <= self.pose_timeout

    def _publish_state(self):
        with self._lock:
            T = self._last_pose_T
            moving = self._is_moving
            joints = list(self._joint_positions)
            err = self._err_text
            enabled = self._enabled
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.connected = self._connected()
        msg.enabled = bool(enabled)
        msg.moving = bool(moving) and msg.connected
        if T is not None:
            msg.tcp_pose = tu.matrix_to_pose_msg(T)
        msg.joint_positions = [float(j) for j in joints]
        msg.error_message = err
        self.state_pub.publish(msg)

    def enable(self, value=True):
        self._enable_requested = value
        if self.backend == "agx":
            if not value:
                self.get_logger().warn(
                    "DISABLING the arm: holding torque is released, it will fall")
            resp = self._call_service(self.enable_cli, SetBool.Request(data=value),
                                      self.agx_enable_service)
            if resp is not None and not resp.success:
                self.get_logger().error(f"enable({value}) rejected: {resp.message}")
            return
        b = Bool()
        b.data = value
        self.enable_pub.publish(b)

    def _call_service(self, client, request, name):
        """Call a service and wait for the reply. Returns None on failure.

        Safe to call from the action's execute callback: the node runs on a
        MultiThreadedExecutor with a ReentrantCallbackGroup, so another thread
        keeps spinning while this one waits on the future.
        """
        if client is None:
            return None
        if not client.wait_for_service(timeout_sec=self.agx_service_timeout):
            self.get_logger().error(
                f"service '{name}' unavailable after {self.agx_service_timeout:.0f}s "
                f"(is {self.agx_driver_node} running?)")
            return None
        future = client.call_async(request)
        deadline = time.time() + self.agx_service_timeout
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            self.get_logger().error(f"service '{name}' timed out")
            return None
        return future.result()

    def _agx_push_speed(self, speed_fraction):
        """Push a per-goal speed onto the driver's speed_percent parameter.

        PoseStamped carries no speed, so this is the only way the calibration's
        speed cap reaches the arm. A failure here is fatal to the move: silently
        running at the driver's default (100%) is not an acceptable fallback.
        """
        pct = int(round(min(max(speed_fraction, 0.0), 1.0) * 100))
        pct = max(pct, 1)               # 0 would mean "do not move"
        if pct == self._agx_speed_pct:  # already set; skip the round trip
            return True
        req = SetParameters.Request()
        req.parameters = [Parameter("speed_percent", Parameter.Type.INTEGER,
                                    pct).to_parameter_msg()]
        resp = self._call_service(self.speed_cli, req,
                                  f"{self.agx_driver_node}/set_parameters")
        if resp is None or not resp.results or not resp.results[0].successful:
            reason = "" if resp is None else resp.results[0].reason
            self.get_logger().error(f"could not set speed_percent={pct}: {reason}")
            return False
        self._agx_speed_pct = pct
        self.get_logger().info(f"driver speed_percent={pct}")
        return True

    def current_pose(self):
        with self._lock:
            return None if self._last_pose_T is None else self._last_pose_T.copy()

    # ---- dry-run pose chaining ---- #
    def _dry_pose_if_fresh(self):
        """Previous dry-run goal, or None if it is too old to be part of this run.

        A calibration sweep issues goals back to back, so a long gap means the
        previous sweep ended and the next dry run should start from where the
        arm actually is.
        """
        with self._lock:
            if self._dry_pose is None:
                return None
            if time.time() - self._dry_pose_time > self.dry_chain_timeout:
                self._dry_pose = None
                return None
            return self._dry_pose.copy()

    def _set_dry_pose(self, T):
        with self._lock:
            self._dry_pose = T.copy()
            self._dry_pose_time = time.time()

    def _clear_dry_pose(self):
        with self._lock:
            self._dry_pose = None

    # ------------------------------------------------------------------ #
    # MoveToCalibrationPose action
    # ------------------------------------------------------------------ #
    def _resolve_goal_pose(self, goal):
        if goal.pose_index >= 0:
            poses = self._load_pose_list()
            if goal.pose_index >= len(poses):
                return None, f"pose_index {goal.pose_index} out of range (have {len(poses)})"
            return poses[goal.pose_index], ""
        # explicit target
        return tu.pose_msg_to_matrix(goal.target_pose), ""

    def _load_pose_list(self):
        # poses declared as a flat parameter by the manager/launch; if not present
        # this adapter only supports explicit target poses.
        return getattr(self, "_pose_list_cache", [])

    def set_pose_list(self, pose_matrices):
        self._pose_list_cache = pose_matrices

    def _execute_move(self, goal_handle):
        goal = goal_handle.request
        result = MoveToCalibrationPose.Result()

        # Joint-space goals take precedence and take a different route: they go
        # straight to the joint controller, so the arm's Cartesian IK -- and any
        # disagreement between the URDF and the arm's own kinematic model -- is
        # out of the picture. Anything inside the joint limits always executes.
        if list(goal.target_joints):
            return self._execute_joint_move(goal_handle, list(goal.target_joints))

        target_T, err = self._resolve_goal_pose(goal)
        if target_T is None:
            result.success = False
            result.error_message = err
            goal_handle.abort()
            return result

        target_R, target_t = tu.decompose_transform(target_T)
        target_q = tu.matrix_to_quaternion(target_R)
        speed = self.safety.clamp_speed(goal.speed, self.default_speed)
        timeout = goal.timeout if goal.timeout > 0 else self.move_timeout

        dry = self.dry_run or goal.dry_run
        live = not dry

        cur = self.current_pose()
        if dry:
            # In a dry run the arm never moves, so current_pose() stays parked
            # for the whole sweep and the step-distance check would measure
            # "park -> pose N" for every pose instead of "pose N-1 -> pose N".
            # That rejects perfectly reachable poses near the end of the sweep
            # and sends you off editing a pose file that was fine. Chain from
            # the previous dry goal so the check sees the real step sequence.
            chained = self._dry_pose_if_fresh()
            if chained is not None:
                cur = chained
        cur_t = None if cur is None else tu.decompose_transform(cur)[1]

        reasons = self.safety.check_goal(target_t, target_q, cur_t, speed)
        if not self._connected():
            reasons.append("robot not connected (no recent end_pose)")
        if self._stop_requested:
            reasons.append("STOP is active (call clear_stop to re-enable motion)")

        # On-demand enable: a live move requires the arm enabled. Enabling only
        # powers the joints (no motion) and is a prerequisite; the user already
        # opted into motion by launching with dry_run:=false.
        if (live and self.require_enabled and not self._enabled
                and self.auto_enable and not reasons):
            self.get_logger().warn("robot not enabled; requesting enable before move")
            self.enable(True)
            t_enable = time.time()
            while time.time() - t_enable < 3.0 and not self._enabled:
                time.sleep(0.05)
        if live and self.require_enabled and not self._enabled:
            reasons.append("robot not enabled (auto-enable failed/timed out)")
        if reasons:
            msg = "; ".join(reasons)
            self.get_logger().error(f"MOVE REJECTED (safety): {msg}")
            result.success = False
            result.error_message = f"safety rejected: {msg}"
            goal_handle.abort()
            return result

        roll, pitch, yaw = tu.matrix_to_euler(target_R)
        self.get_logger().warn(
            f"MOVE {'[DRY-RUN] ' if dry else '[LIVE] '}-> "
            f"pos=({target_t[0]:.3f},{target_t[1]:.3f},{target_t[2]:.3f}) "
            f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f}) "
            f"speed={speed:.2f}")

        if dry:
            # Simulate arrival without commanding the robot, and remember where
            # the arm "would" now be so the next goal's step check is measured
            # from here rather than from the parked pose.
            self._set_dry_pose(target_T)
            self._feedback_once(goal_handle, target_T, 0.0, 0.0, 0.0, False)
            result.success = True
            result.final_pose = tu.matrix_to_pose_msg(target_T)
            result.error_message = "dry_run: not commanded"
            goal_handle.succeed()
            return result

        # A live move invalidates any simulated pose: from here on the real
        # feedback is the truth.
        self._clear_dry_pose()

        # LIVE: command the Cartesian goal
        if self.backend == "topic" and self.pos_cmd_pub is None:
            result.success = False
            result.error_message = "no PosCmd publisher (piper_msgs missing)"
            goal_handle.abort()
            return result
        # The speed cap has to reach the driver BEFORE the goal does; if it
        # cannot, abort rather than move at the driver's 100% default.
        if self.backend == "agx" and self.agx_set_speed and not self._agx_push_speed(speed):
            result.success = False
            result.error_message = ("could not set driver speed; refusing to move at "
                                    "the driver default")
            goal_handle.abort()
            return result
        self._send_pos_cmd(target_t, (roll, pitch, yaw), speed)

        # monitor until arrival, timeout, or cancel
        start = time.time()
        start_pose = self.current_pose()
        stopped_since = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                result.success = False
                result.error_message = "canceled"
                goal_handle.canceled()
                return result
            if self._stop_requested:
                self._stop()
                result.success = False
                result.error_message = "STOP requested"
                goal_handle.abort()
                return result
            # The arm can refuse a goal after accepting the command -- no IK
            # solution, joint limit, singularity. Without this the refusal shows
            # up only as a bare timeout N seconds later, with no reason given.
            with self._lock:
                arm_fault = self._err_text
            if arm_fault:
                self._stop()
                result.success = False
                result.error_message = f"arm reported: {arm_fault}"
                self.get_logger().error(f"MOVE ABORTED -- {arm_fault}")
                goal_handle.abort()
                return result
            elapsed = time.time() - start
            cur = self.current_pose()
            if cur is not None:
                pos_err = tu.translation_distance(cur, target_T)
                ori_err = math.degrees(tu.rotation_angle_between(cur, target_T))
                with self._lock:
                    moving = self._is_moving
                self._feedback_once(goal_handle, cur, pos_err, ori_err, elapsed, moving)
                arrived = (pos_err <= self.pos_tol and ori_err <= self.ori_tol_deg)
                if arrived and not moving:
                    if stopped_since is None:
                        stopped_since = time.time()
                    elif time.time() - stopped_since >= self.settle_check:
                        result.success = True
                        result.final_pose = tu.matrix_to_pose_msg(cur)
                        result.error_message = ""
                        goal_handle.succeed()
                        return result
                else:
                    stopped_since = None
            if elapsed > timeout:
                # A bare "timeout" tells you nothing about WHY. Report how far
                # the arm actually got and whether it moved at all -- that is
                # what separates "unreachable goal" from "it is still moving,
                # raise the timeout" from "the command never took effect".
                cur = self.current_pose()
                if cur is None:
                    detail = "no pose feedback"
                else:
                    moved = tu.translation_distance(start_pose, cur) if start_pose is not None else float("nan")
                    left = tu.translation_distance(cur, target_T)
                    ori_left = math.degrees(tu.rotation_angle_between(cur, target_T))
                    detail = (f"moved {moved * 1000:.0f} mm from the start, still "
                              f"{left * 1000:.0f} mm / {ori_left:.1f} deg from the goal")
                    if moved < 0.005:
                        detail += (" -- the arm never started moving; the goal is "
                                   "probably unreachable in this orientation")
                self._stop()
                result.success = False
                result.error_message = f"timeout after {timeout:.1f}s ({detail})"
                self.get_logger().error(f"MOVE TIMED OUT -- {detail}")
                goal_handle.abort()
                return result
            time.sleep(0.05)

        result.success = False
        result.error_message = "shutdown"
        goal_handle.abort()
        return result

    # ------------------------------------------------------------------ #
    # joint-space move
    # ------------------------------------------------------------------ #
    def _execute_joint_move(self, goal_handle, target_q):
        """Drive the arm to a joint configuration.

        Safety here is joint limits, not the Cartesian workspace box: a joint
        goal has no IK to fail, so what can go wrong is commanding a joint past
        its stop. The resulting flange position is still checked against the
        workspace bounds, using the arm's own forward kinematics, so a goal that
        would swing the arm outside the agreed volume is still refused.
        """
        result = MoveToCalibrationPose.Result()
        speed = self.safety.clamp_speed(goal_handle.request.speed, self.default_speed)
        timeout = goal_handle.request.timeout or self.move_timeout

        reasons = []
        if self.backend != "agx":
            reasons.append(f"joint-space goals need control_backend:=agx (have {self.backend})")
        if len(target_q) != len(self.joint_names):
            reasons.append(f"expected {len(self.joint_names)} joint values, got {len(target_q)}")
        if not self._connected():
            reasons.append("robot not connected")
        if self._stop_requested:
            reasons.append("STOP is active (call clear_stop to re-enable motion)")
        if not reasons and ak.HAVE_SDK:
            if not ak.within_joint_limits(target_q, margin_deg=self.joint_limit_margin_deg):
                bad = [f"j{i + 1}={math.degrees(q):.1f}deg"
                       for i, (q, (lo, hi)) in enumerate(zip(target_q, ak.JOINT_LIMITS))
                       if not (lo <= q <= hi)]
                reasons.append("joint target outside limits: " + ", ".join(bad or ["<margin>"]))
            else:
                t = ak.fk(target_q)[:3, 3]
                reasons += self.safety.check_position(t)

        dry = self.dry_run or goal_handle.request.dry_run
        live = not dry
        if (live and self.require_enabled and not self._enabled
                and self.auto_enable and not reasons):
            self.get_logger().warn("robot not enabled; requesting enable before move")
            self.enable(True)
            t_enable = time.time()
            while time.time() - t_enable < 3.0 and not self._enabled:
                time.sleep(0.05)
        if live and self.require_enabled and not self._enabled:
            reasons.append("robot not enabled (auto-enable failed/timed out)")

        if reasons:
            msg = "; ".join(reasons)
            self.get_logger().error(f"JOINT MOVE REJECTED (safety): {msg}")
            result.success = False
            result.error_message = f"safety rejected: {msg}"
            goal_handle.abort()
            return result

        deg = ", ".join(f"{math.degrees(q):.1f}" for q in target_q)
        self.get_logger().warn(
            f"MOVE-J {'[DRY-RUN] ' if dry else '[LIVE] '}-> [{deg}] deg speed={speed:.2f}")

        if dry:
            result.success = True
            result.error_message = "dry_run: not commanded"
            goal_handle.succeed()
            return result

        if self.agx_set_speed and not self._agx_push_speed(speed):
            result.success = False
            result.error_message = "could not set driver speed; refusing to move"
            goal_handle.abort()
            return result

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = [float(q) for q in target_q]
        self.move_j_pub.publish(js)

        start = time.time()
        start_q = list(self._joint_snapshot())
        stopped_since = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                result.success = False
                result.error_message = "canceled"
                goal_handle.canceled()
                return result
            if self._stop_requested:
                self._stop()
                result.success = False
                result.error_message = "STOP requested"
                goal_handle.abort()
                return result
            with self._lock:
                arm_fault = self._err_text
                moving = self._is_moving
            if arm_fault:
                self._stop()
                result.success = False
                result.error_message = f"arm reported: {arm_fault}"
                self.get_logger().error(f"JOINT MOVE ABORTED -- {arm_fault}")
                goal_handle.abort()
                return result

            elapsed = time.time() - start
            cur_q = self._joint_snapshot()
            err_deg = self._joint_error_deg(cur_q, target_q)
            cur_T = self.current_pose()
            if cur_T is not None:
                self._feedback_once(goal_handle, cur_T, 0.0, err_deg, elapsed, moving)
            if err_deg <= self.joint_goal_tolerance_deg and not moving:
                if stopped_since is None:
                    stopped_since = time.time()
                elif time.time() - stopped_since >= self.settle_check:
                    result.success = True
                    if cur_T is not None:
                        result.final_pose = tu.matrix_to_pose_msg(cur_T)
                    goal_handle.succeed()
                    return result
            else:
                stopped_since = None

            if elapsed > timeout:
                travelled = self._joint_error_deg(start_q, cur_q)
                detail = (f"still {err_deg:.1f} deg from the goal after moving "
                          f"{travelled:.1f} deg")
                if travelled < 0.5:
                    detail += " -- the arm never started moving"
                self._stop()
                result.success = False
                result.error_message = f"timeout after {timeout:.1f}s ({detail})"
                self.get_logger().error(f"JOINT MOVE TIMED OUT -- {detail}")
                goal_handle.abort()
                return result
            time.sleep(0.05)

        result.success = False
        result.error_message = "shutdown"
        goal_handle.abort()
        return result

    def _joint_snapshot(self):
        with self._lock:
            return list(self._joint_positions)

    @staticmethod
    def _joint_error_deg(a, b):
        """Largest per-joint difference, in degrees. inf if not comparable."""
        if not a or not b or len(a) < len(b):
            return float("inf")
        return max(abs(math.degrees(x - y)) for x, y in zip(a, b))

    def _feedback_once(self, goal_handle, cur_T, pos_err, ori_err, elapsed, moving):
        fb = MoveToCalibrationPose.Feedback()
        fb.current_pose = tu.matrix_to_pose_msg(cur_T)
        fb.position_error = float(pos_err)
        fb.orientation_error = float(ori_err)
        fb.elapsed_time = float(elapsed)
        fb.moving = bool(moving)
        goal_handle.publish_feedback(fb)

    def _send_pos_cmd(self, t, rpy, speed):
        if self.backend == "agx":
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.pose = tu.matrix_to_pose_msg(
                tu.make_transform(tu.euler_to_matrix(*rpy), t))
            self.move_pub.publish(msg)
            return
        cmd = PosCmd()
        cmd.x, cmd.y, cmd.z = float(t[0]), float(t[1]), float(t[2])
        cmd.roll, cmd.pitch, cmd.yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
        cmd.gripper = 0.0
        cmd.mode1 = 0
        # mode2 maps onto MotionCtrl_2's move_mode: 0=MOVE P, 1=MOVE J, 2=MOVE L.
        cmd.mode2 = 0  # MOVE P -- point-to-point to the Cartesian goal
        self.pos_cmd_pub.publish(cmd)

    def _srv_stop(self, req, resp):
        """Immediate stop: abort any running move and hold the current pose."""
        self._stop_requested = True
        self._stop()
        resp.success = True
        resp.message = ("STOP: motion halted, holding current pose. "
                        "Call clear_stop before moving again.")
        self.get_logger().warn("*** STOP requested via service ***")
        return resp

    def _srv_clear_stop(self, req, resp):
        self._stop_requested = False
        resp.success = True
        resp.message = "stop cleared; moves allowed again"
        self.get_logger().info("stop flag cleared")
        return resp

    def _srv_estop(self, req, resp):
        """Hard e-stop: cuts drive power. Separate from stop_motion on purpose."""
        self.emergency_stop()
        resp.success = True
        resp.message = ("EMERGENCY STOP: drive power cut. THE ARM HAS FALLEN. "
                        "Re-enable and re-home before continuing.")
        return resp

    def _stop(self):
        # Hold the current pose. Deliberately NOT the driver's emergency_stop
        # service: that cuts drive power and the arm falls, which would destroy
        # a wrist-mounted camera. Holding position is the safe stop for a
        # calibration run. Use the GUI's separate e-stop path for a real
        # emergency, where dropping the arm is the lesser harm.
        cur = self.current_pose()
        if cur is not None and (self.move_pub is not None or self.pos_cmd_pub is not None):
            R, t = tu.decompose_transform(cur)
            self._send_pos_cmd(t, tu.matrix_to_euler(R), self.default_speed)
        self.get_logger().warn("STOP requested: holding current pose")

    def emergency_stop(self):
        """Cut drive power immediately. THE ARM WILL FALL under gravity."""
        if self.backend != "agx":
            return self._stop()
        self.get_logger().error("*** EMERGENCY STOP: drives cut, the arm will fall ***")
        self._stop_requested = True
        self._call_service(self.estop_cli, EmptySrv.Request(), self.agx_estop_service)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PiperControlNode()
    except (RuntimeError, ValueError) as exc:
        # Misconfiguration is the most common startup failure here, so print the
        # remediation hints plainly rather than burying them in a traceback.
        print(f"\n[piper_control_node] startup failed:\n{exc}\n", flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Piper control adapter.

Design decision (see README): the workspace already ships a working Piper ROS
driver (``piper_ctrl_single_node``) that wraps ``piper_sdk`` over CAN. Rather
than re-wrap the SDK (risky, duplicate), this adapter talks to that driver over
ROS topics and adds calibration-specific services: a clean RobotState stream
and a SAFETY-validated MoveToCalibrationPose action.

  reads  end_pose_topic   (PoseStamped)    -> base_T_gripper
         arm_status_topic  (PiperStatusMsg) -> enabled / error
         joint_state_topic (JointState)     -> joint_positions
  writes pos_cmd_topic     (PosCmd, moveL)  -> Cartesian goal
         enable_topic      (Bool)           -> enable arm

The SAME adapter runs against the real driver or against mock_robot_node, which
emulates the driver's topics. dry_run is the master safety switch.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from auto_handeye_interfaces.action import MoveToCalibrationPose
from auto_handeye_interfaces.msg import RobotState

from . import transform_utils as tu
from .safety_validator import SafetyValidator, SafetyLimits

try:
    from piper_msgs.msg import PosCmd, PiperStatusMsg
    _HAVE_PIPER_MSGS = True
except ImportError:  # allow node import where piper_msgs isn't built (mock-only setups)
    PosCmd = None
    PiperStatusMsg = None
    _HAVE_PIPER_MSGS = False


class PiperControlNode(Node):
    def __init__(self):
        super().__init__("piper_control_node")
        p = self.declare_parameter

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
        self._err_code = 0
        self._joint_positions = []
        self._stop_requested = False   # set by the stop_motion service / GUI STOP

        cb = ReentrantCallbackGroup()
        self.create_subscription(PoseStamped, self.end_pose_topic,
                                 self._end_pose_cb, 10, callback_group=cb)
        self.create_subscription(JointState, self.joint_state_topic,
                                 self._joint_cb, 10, callback_group=cb)
        if _HAVE_PIPER_MSGS:
            self.create_subscription(PiperStatusMsg, self.arm_status_topic,
                                     self._status_cb, 10, callback_group=cb)
            self.pos_cmd_pub = self.create_publisher(PosCmd, self.pos_cmd_topic, 10)
        else:
            self.pos_cmd_pub = None
            self.get_logger().warn(
                "piper_msgs not available: cannot command the real driver "
                "(mock_robot_node provides its own PosCmd). Move will fail unless mock is used.")
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)

        self.state_pub = self.create_publisher(RobotState, "robot_state", 10)
        self.create_timer(0.1, self._publish_state)

        # STOP / RESUME services (wired to the GUI stop button)
        self.create_service(Trigger, "stop_motion", self._srv_stop, callback_group=cb)
        self.create_service(Trigger, "clear_stop", self._srv_clear_stop, callback_group=cb)

        self._action = ActionServer(
            self, MoveToCalibrationPose, "move_to_calibration_pose",
            execute_callback=self._execute_move,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb)

        mode = "DRY-RUN (no motion)" if self.dry_run else "LIVE (robot WILL move)"
        self.get_logger().info(f"piper_control_node up | backend=topic | {mode}")
        self.get_logger().info(f"base='{self.base_frame}' gripper='{self.gripper_frame}' "
                               f"workspace {ws_min}..{ws_max}")

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _end_pose_cb(self, msg: PoseStamped):
        T = tu.pose_msg_to_matrix(msg.pose)
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

    def _joint_cb(self, msg: JointState):
        with self._lock:
            self._joint_positions = list(msg.position)

    def _status_cb(self, msg):
        with self._lock:
            self._err_code = int(msg.err_code)
            # heuristic: driver reports enabled once arm_status indicates normal ops.
            self._enabled = (self._enable_requested and msg.err_code == 0)

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
            err = self._err_code
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
        msg.error_message = "" if err == 0 else f"piper err_code={err}"
        self.state_pub.publish(msg)

    def enable(self, value=True):
        self._enable_requested = value
        b = Bool()
        b.data = value
        self.enable_pub.publish(b)

    def current_pose(self):
        with self._lock:
            return None if self._last_pose_T is None else self._last_pose_T.copy()

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

        cur = self.current_pose()
        cur_t = None if cur is None else tu.decompose_transform(cur)[1]

        reasons = self.safety.check_goal(target_t, target_q, cur_t, speed)
        if not self._connected():
            reasons.append("robot not connected (no recent end_pose)")
        if self._stop_requested:
            reasons.append("STOP is active (call clear_stop to re-enable motion)")

        live = not (self.dry_run or goal.dry_run)
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

        dry = self.dry_run or goal.dry_run
        roll, pitch, yaw = tu.matrix_to_euler(target_R)
        self.get_logger().warn(
            f"MOVE {'[DRY-RUN] ' if dry else '[LIVE] '}-> "
            f"pos=({target_t[0]:.3f},{target_t[1]:.3f},{target_t[2]:.3f}) "
            f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f}) "
            f"speed={speed:.2f}")

        if dry:
            # Simulate arrival without commanding the robot.
            self._feedback_once(goal_handle, target_T, 0.0, 0.0, 0.0, False)
            result.success = True
            result.final_pose = tu.matrix_to_pose_msg(target_T)
            result.error_message = "dry_run: not commanded"
            goal_handle.succeed()
            return result

        # LIVE: send moveL command
        if self.pos_cmd_pub is None:
            result.success = False
            result.error_message = "no PosCmd publisher (piper_msgs missing)"
            goal_handle.abort()
            return result
        self._send_pos_cmd(target_t, (roll, pitch, yaw), speed)

        # monitor until arrival, timeout, or cancel
        start = time.time()
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
                self._stop()
                result.success = False
                result.error_message = f"timeout after {timeout:.1f}s"
                goal_handle.abort()
                return result
            time.sleep(0.05)

        result.success = False
        result.error_message = "shutdown"
        goal_handle.abort()
        return result

    def _feedback_once(self, goal_handle, cur_T, pos_err, ori_err, elapsed, moving):
        fb = MoveToCalibrationPose.Feedback()
        fb.current_pose = tu.matrix_to_pose_msg(cur_T)
        fb.position_error = float(pos_err)
        fb.orientation_error = float(ori_err)
        fb.elapsed_time = float(elapsed)
        fb.moving = bool(moving)
        goal_handle.publish_feedback(fb)

    def _send_pos_cmd(self, t, rpy, speed):
        cmd = PosCmd()
        cmd.x, cmd.y, cmd.z = float(t[0]), float(t[1]), float(t[2])
        cmd.roll, cmd.pitch, cmd.yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
        cmd.gripper = 0.0
        cmd.mode1 = 0
        cmd.mode2 = 1  # moveL (linear Cartesian), per existing driver convention
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

    def _stop(self):
        # Re-command the current pose as a hold (Piper has no explicit e-stop topic
        # in this driver; holding position is the safe fallback).
        cur = self.current_pose()
        if cur is not None and self.pos_cmd_pub is not None:
            R, t = tu.decompose_transform(cur)
            self._send_pos_cmd(t, tu.matrix_to_euler(R), self.default_speed)
        self.get_logger().warn("STOP requested: holding current pose")


def main(args=None):
    rclpy.init(args=args)
    node = PiperControlNode()
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


if __name__ == "__main__":
    main()

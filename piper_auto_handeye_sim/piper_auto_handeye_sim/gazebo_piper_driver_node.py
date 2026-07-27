#!/usr/bin/env python3
"""Gazebo stand-in for the real Piper CAN driver (``piper_ctrl_single_node``).

Same trick as ``mock_robot_node``, but backed by real physics: it speaks the
EXACT topic interface the real driver exposes, so ``piper_control_node`` and the
whole calibration stack run **completely unchanged**. What you verify in Gazebo
is therefore the same code path that runs on hardware.

  subscribes  /joint_states   (from joint_state_broadcaster)  -> FK -> pose
              /pos_cmd        (piper_msgs/PosCmd, Cartesian)  -> IK -> trajectory
              /enable_flag    (std_msgs/Bool)
  publishes   /end_pose_stamped   (PoseStamped)      base_T_gripper
              /arm_status         (PiperStatusMsg)   err_code
              /joint_states_single(JointState)
              <arm_controller>/joint_trajectory (JointTrajectory)

Why IK is needed
----------------
The real driver takes Cartesian moveL commands and does IK on the controller.
Gazebo's JointTrajectoryController takes joint positions. This node closes that
gap with a KDL Levenberg-Marquardt solver.

IK accuracy barely matters for calibration correctness: the pose actually
reached is measured by FK from /joint_states, not assumed from the command. It
only has to be good enough to (a) land somewhere different each time and (b)
satisfy piper_control_node's arrival tolerance.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import PyKDL as kdl

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .urdf_kdl import UrdfChainBuilder, solve_ik

try:
    from piper_msgs.msg import PosCmd, PiperStatusMsg
    _HAVE_PIPER_MSGS = True
except ImportError:
    PosCmd = None
    PiperStatusMsg = None
    _HAVE_PIPER_MSGS = False


def _quat_from_rot(M: kdl.Rotation):
    x, y, z, w = M.GetQuaternion()
    return x, y, z, w


class GazeboPiperDriverNode(Node):
    def __init__(self):
        super().__init__("gazebo_piper_driver_node")
        p = self.declare_parameter

        self.base_frame = p("base_frame", "base_link").value
        self.gripper_frame = p("gripper_frame", "link6").value
        self.end_pose_topic = p("end_pose_topic", "/end_pose_stamped").value
        self.arm_status_topic = p("arm_status_topic", "/arm_status").value
        self.joint_state_single_topic = p("joint_state_topic", "/joint_states_single").value
        self.pos_cmd_topic = p("pos_cmd_topic", "/pos_cmd").value
        self.enable_topic = p("enable_topic", "/enable_flag").value
        self.gazebo_joint_states = p("gazebo_joint_states_topic", "/joint_states").value
        self.traj_topic = p("trajectory_topic",
                            "/arm_controller/joint_trajectory").value
        self.arm_joints = list(p("arm_joints",
                                 ["joint1", "joint2", "joint3",
                                  "joint4", "joint5", "joint6"]).value)
        # Single-point trajectories: the controller interpolates from wherever
        # the arm is to the target over this long. Too short and the arm lags
        # far behind, so piper_control_node never sees it "arrive"; the poses
        # are far apart, so it needs real time.
        self.move_time = float(p("trajectory_time", 4.0).value)
        self.publish_rate = float(p("publish_rate", 50.0).value)
        self.ik_max_iter = int(p("ik_max_iterations", 500).value)
        self.ik_eps = float(p("ik_epsilon", 1e-6).value)
        self.ik_restarts = int(p("ik_restarts", 60).value)

        self._rng = np.random.default_rng(0)
        self._lock = threading.Lock()
        self._joint_pos = {}
        self._enabled = False
        self._chain = None
        self._chain_joints = []
        self._fk = None
        self._ik = None
        self._q_lower = None
        self._q_upper = None

        # robot_description is latched by robot_state_publisher
        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description",
                                 self._on_urdf, latched)

        self.create_subscription(JointState, self.gazebo_joint_states,
                                 self._on_joint_states, 10)
        self.create_subscription(Bool, self.enable_topic, self._on_enable, 10)

        self.pose_pub = self.create_publisher(PoseStamped, self.end_pose_topic, 10)
        self.joint_pub = self.create_publisher(JointState,
                                               self.joint_state_single_topic, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        if _HAVE_PIPER_MSGS:
            self.status_pub = self.create_publisher(PiperStatusMsg,
                                                    self.arm_status_topic, 10)
            self.create_subscription(PosCmd, self.pos_cmd_topic, self._on_pos_cmd, 10)
        else:
            self.status_pub = None
            self.get_logger().error(
                "piper_msgs not available: cannot emulate the Piper driver. "
                "Build Piper_ros/src/piper_msgs and re-source.")

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"gazebo_piper_driver_node up | emulating the Piper driver on "
            f"{self.base_frame} -> {self.gripper_frame}; waiting for /robot_description")

    # ------------------------------------------------------------------ #
    def _on_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            builder = UrdfChainBuilder(msg.data)
            chain, joints = builder.build_chain(self.base_frame, self.gripper_frame)
            lower, upper = builder.joint_limits(joints, msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to build KDL chain: {exc}")
            return

        self._chain = chain
        self._chain_joints = joints
        self._q_lower = kdl.JntArray(len(joints))
        self._q_upper = kdl.JntArray(len(joints))
        for i, (lo, hi) in enumerate(zip(lower, upper)):
            self._q_lower[i] = lo
            self._q_upper[i] = hi

        self._fk = kdl.ChainFkSolverPos_recursive(chain)
        # LMA is robust for 6-DoF pose IK and needs no separate velocity solver.
        self._ik = kdl.ChainIkSolverPos_LMA(chain, self.ik_eps, self.ik_max_iter)

        self.get_logger().info(
            f"KDL chain ready: {chain.getNrOfSegments()} segments, "
            f"{len(joints)} movable joints {joints}")

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_pos[name] = pos

    def _on_enable(self, msg: Bool):
        with self._lock:
            self._enabled = bool(msg.data)
        self.get_logger().info(f"enable_flag={msg.data} (no-op in Gazebo)")

    # ------------------------------------------------------------------ #
    def _current_q(self):
        """Chain joint positions as a KDL JntArray, or None if incomplete."""
        if self._chain is None:
            return None
        with self._lock:
            if any(j not in self._joint_pos for j in self._chain_joints):
                return None
            q = kdl.JntArray(len(self._chain_joints))
            for i, name in enumerate(self._chain_joints):
                q[i] = self._joint_pos[name]
        return q

    def _tick(self):
        q = self._current_q()
        if q is None:
            return
        frame = kdl.Frame()
        if self._fk.JntToCart(q, frame) < 0:
            self.get_logger().warn("FK failed", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.base_frame
        ps.pose.position.x = frame.p[0]
        ps.pose.position.y = frame.p[1]
        ps.pose.position.z = frame.p[2]
        qx, qy, qz, qw = _quat_from_rot(frame.M)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        js = JointState()
        js.header.stamp = now
        js.name = list(self._chain_joints)
        js.position = [q[i] for i in range(len(self._chain_joints))]
        self.joint_pub.publish(js)

        if self.status_pub is not None:
            st = PiperStatusMsg()
            st.err_code = 0
            self.status_pub.publish(st)

    # ------------------------------------------------------------------ #
    def _on_pos_cmd(self, msg):
        """Cartesian moveL -> IK -> JointTrajectory for the Gazebo controller."""
        if self._ik is None:
            self.get_logger().warn("PosCmd before the KDL chain was ready; ignoring")
            return
        q_init = self._current_q()
        if q_init is None:
            self.get_logger().warn("PosCmd before /joint_states arrived; ignoring")
            return

        target = kdl.Frame(
            kdl.Rotation.RPY(float(msg.roll), float(msg.pitch), float(msg.yaw)),
            kdl.Vector(float(msg.x), float(msg.y), float(msg.z)))

        # Random restarts: a single LMA call from the current configuration
        # fails on most genuinely reachable targets, because the arm has several
        # IK branches and LMA only finds a local one.
        lower = [self._q_lower[i] for i in range(len(self._chain_joints))]
        upper = [self._q_upper[i] for i in range(len(self._chain_joints))]
        q_out, ok = solve_ik(self._ik, self._fk, target, q_init, lower, upper,
                             attempts=self.ik_restarts, rng=self._rng)
        if not ok:
            self.get_logger().error(
                f"IK failed after {self.ik_restarts} restarts for target "
                f"({msg.x:.3f},{msg.y:.3f},{msg.z:.3f}); not moving")
            return

        reached = kdl.Frame()
        self._fk.JntToCart(q_out, reached)
        pos_err = (reached.p - target.p).Norm()
        ang_err = math.degrees(
            abs((reached.M.Inverse() * target.M).GetRotAngle()[0]))

        traj = JointTrajectory()
        traj.joint_names = list(self._chain_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [q_out[i] for i in range(len(self._chain_joints))]
        pt.velocities = [0.0] * len(self._chain_joints)
        pt.time_from_start.sec = int(self.move_time)
        pt.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

        self.get_logger().info(
            f"moveL -> ({msg.x:.3f},{msg.y:.3f},{msg.z:.3f}) "
            f"q=[{', '.join(f'{v:.3f}' for v in pt.positions)}]")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPiperDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

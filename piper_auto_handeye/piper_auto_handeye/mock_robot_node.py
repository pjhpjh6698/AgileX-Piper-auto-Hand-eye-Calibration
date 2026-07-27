#!/usr/bin/env python3
"""Mock Piper driver for hardware-free testing.

Emulates the EXISTING ``piper_ctrl_single_node`` interface so that
``piper_control_node`` (and the whole calibration stack) runs unchanged:

  subscribes  pos_cmd_topic  (PosCmd)   -> Cartesian moveL goal
              enable_topic   (Bool)     -> enable flag
  publishes   end_pose_topic (PoseStamped) base_T_gripper, interpolated
              arm_status_topic (PiperStatusMsg)  (if piper_msgs available)
              joint_state_topic (JointState)

Motion is simulated by interpolating the current pose toward the last commanded
pose at a bounded speed, so movement/settling behave realistically.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from . import transform_utils as tu

try:
    from piper_msgs.msg import PosCmd, PiperStatusMsg
    _HAVE_PIPER_MSGS = True
except ImportError:
    PosCmd = None
    PiperStatusMsg = None
    _HAVE_PIPER_MSGS = False


class MockRobotNode(Node):
    def __init__(self):
        super().__init__("mock_robot_node")
        p = self.declare_parameter
        self.end_pose_topic = p("end_pose_topic", "/end_pose_stamped").value
        self.arm_status_topic = p("arm_status_topic", "/arm_status").value
        self.joint_state_topic = p("joint_state_topic", "/joint_states_single").value
        self.pos_cmd_topic = p("pos_cmd_topic", "/pos_cmd").value
        self.enable_topic = p("enable_topic", "/enable_flag").value
        self.base_frame = p("base_frame", "base_link").value
        self.linear_speed = float(p("mock_linear_speed", 0.25).value)      # m/s
        self.angular_speed = float(p("mock_angular_speed", 1.0).value)     # rad/s
        rate = float(p("publish_rate", 50.0).value)

        # initial pose = a safe placeholder home
        init_pos = list(p("mock_init_position", [0.35, 0.0, 0.35]).value)
        init_rpy = list(p("mock_init_rpy", [-3.14, 0.0, 0.0]).value)
        self._lock = threading.Lock()
        self._cur = tu.make_transform(tu.euler_to_matrix(*init_rpy), init_pos)
        self._target = self._cur.copy()
        self._enabled = False

        self.pose_pub = self.create_publisher(PoseStamped, self.end_pose_topic, 10)
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.status_pub = (self.create_publisher(PiperStatusMsg, self.arm_status_topic, 10)
                           if _HAVE_PIPER_MSGS else None)

        if _HAVE_PIPER_MSGS:
            self.create_subscription(PosCmd, self.pos_cmd_topic, self._cmd_cb, 10)
        else:
            self.get_logger().warn("piper_msgs not built: mock accepts no PosCmd goals")
        self.create_subscription(Bool, self.enable_topic, self._enable_cb, 10)

        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"mock_robot_node up (emulates Piper driver) | init pos={init_pos} rpy={init_rpy}")

    def _cmd_cb(self, msg):
        with self._lock:
            self._target = tu.make_transform(
                tu.euler_to_matrix(msg.roll, msg.pitch, msg.yaw),
                [msg.x, msg.y, msg.z])
        self.get_logger().info(
            f"mock received moveL -> ({msg.x:.3f},{msg.y:.3f},{msg.z:.3f})")

    def _enable_cb(self, msg: Bool):
        with self._lock:
            self._enabled = bool(msg.data)
        self.get_logger().info(f"mock enable={msg.data}")

    def _tick(self):
        with self._lock:
            cur, target = self._cur.copy(), self._target.copy()
        # interpolate translation
        cR, ct = tu.decompose_transform(cur)
        tR, tt = tu.decompose_transform(target)
        dt_vec = tt - ct
        dist = float(np.linalg.norm(dt_vec))
        max_step_t = self.linear_speed * self._dt
        if dist > 1e-6:
            step = min(1.0, max_step_t / dist)
            new_t = ct + dt_vec * step
        else:
            new_t = tt
        # interpolate rotation via slerp fraction
        ang = tu.rotation_angle_between(cur, target)
        max_step_r = self.angular_speed * self._dt
        frac = 1.0 if ang < 1e-6 else min(1.0, max_step_r / ang)
        new_R = self._slerp_matrix(cR, tR, frac)
        new_T = tu.make_transform(new_R, new_t)
        with self._lock:
            self._cur = new_T
        self._publish(new_T)

    @staticmethod
    def _slerp_matrix(Ra, Rb, frac):
        qa = tu.matrix_to_quaternion(Ra)
        qb = tu.matrix_to_quaternion(Rb)
        dot = float(np.dot(qa, qb))
        if dot < 0:
            qb = -qb
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            q = qa + frac * (qb - qa)
            return tu.quaternion_to_matrix(q)
        theta = math.acos(dot)
        q = (math.sin((1 - frac) * theta) * qa + math.sin(frac * theta) * qb) / math.sin(theta)
        return tu.quaternion_to_matrix(q)

    def _publish(self, T):
        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.base_frame
        ps.pose = tu.matrix_to_pose_msg(T)
        self.pose_pub.publish(ps)

        js = JointState()
        js.header.stamp = now
        js.name = [f"joint{i+1}" for i in range(6)]
        js.position = [0.0] * 6
        self.joint_pub.publish(js)

        if self.status_pub is not None:
            st = PiperStatusMsg()
            st.err_code = 0
            self.status_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
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

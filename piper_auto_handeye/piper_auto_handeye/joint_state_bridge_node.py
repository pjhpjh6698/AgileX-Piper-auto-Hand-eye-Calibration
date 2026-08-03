#!/usr/bin/env python3
"""Bridge the driver's joint feedback onto /joint_states, gripper included.

robot_state_publisher needs ALL of the URDF's movable joints on /joint_states.
The arm driver only reports joint1..6 (it is launched with effector_type=none),
so the with-gripper model's finger joints would have no TF and RViz would flag
every gripper link as an error. This node re-publishes the driver's message
with the missing joints appended at a fixed position (0 = fingers closed).

Why not ros-humble-joint-state-publisher? It is not installed on every target
machine and pulling system apt packages breaks the "one colcon build" promise
of this workspace. These 40 lines replace the only feature of it we used.

  subscribes  feedback/joint_states   (sensor_msgs/JointState, from agx_arm_ctrl)
  publishes   /joint_states           (sensor_msgs/JointState, complete set)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


class JointStateBridgeNode(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")
        p = self.declare_parameter
        self.source_topic = p("source_topic", "feedback/joint_states").value
        # Joints in the URDF the driver never reports; held at fixed_value.
        self.extra_joints = list(p("extra_joints",
                                   ["gripper", "gripper_joint1",
                                    "gripper_joint2"]).value)
        self.fixed_value = float(p("extra_joint_value", 0.0).value)

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, self.source_topic, self._cb,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"bridging {self.source_topic} -> /joint_states "
            f"(+{len(self.extra_joints)} fixed gripper joints)")

    def _cb(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name) + [j for j in self.extra_joints
                                     if j not in msg.name]
        n_extra = len(out.name) - len(msg.name)
        out.position = list(msg.position) + [self.fixed_value] * n_extra
        # velocity/effort only if the source carried them, padded to match
        if msg.velocity:
            out.velocity = list(msg.velocity) + [0.0] * n_extra
        if msg.effort:
            out.effort = list(msg.effort) + [0.0] * n_extra
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
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

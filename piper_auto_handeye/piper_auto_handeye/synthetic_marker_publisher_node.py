#!/usr/bin/env python3
"""Synthetic marker publisher for fully hardware-free (camera-free) testing.

Given a GROUND-TRUTH gripper_T_camera and a fixed base_T_target, it derives the
marker observation the camera *would* see from the robot's current pose:

    camera_T_target = inv(gripper_T_camera) @ inv(base_T_gripper) @ base_T_target

and publishes it as a MarkerDetection on 'marker_detection'. This lets the whole
calibration state machine run and RECOVER the known gripper_T_camera with the
mock robot, with no camera or ArUco marker present.

Not for real use -- it is a test double for the ArUco detector.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from auto_handeye_interfaces.msg import MarkerDetection

from . import transform_utils as tu


class SyntheticMarkerPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_marker_publisher_node")
        p = self.declare_parameter
        self.end_pose_topic = p("end_pose_topic", "/end_pose_stamped").value
        self.camera_frame = p("camera_frame", "camera_color_optical_frame").value
        self.target_id = int(p("target_marker_id", 1).value)
        self.noise_m = float(p("noise_translation_m", 0.0).value)
        self.noise_deg = float(p("noise_rotation_deg", 0.0).value)

        gt_t = list(p("ground_truth_translation", [0.05, -0.03, 0.08]).value)
        gt_rpy = list(p("ground_truth_rpy", [0.1, 0.2, 0.05]).value)
        self.gripper_T_camera = tu.make_transform(tu.euler_to_matrix(*gt_rpy), gt_t)

        bt_t = list(p("base_target_translation", [0.55, 0.0, 0.15]).value)
        bt_rpy = list(p("base_target_rpy", [0.0, 0.0, 0.4]).value)
        self.base_T_target = tu.make_transform(tu.euler_to_matrix(*bt_rpy), bt_t)

        self._rng = np.random.default_rng(0)
        self.pub = self.create_publisher(MarkerDetection, "marker_detection", 10)
        self.create_subscription(PoseStamped, self.end_pose_topic, self._cb, 10)
        self.get_logger().warn(
            "SYNTHETIC marker publisher active (test double, not a real camera). "
            f"GT gripper_T_camera t={gt_t} rpy={gt_rpy}")

    def _cb(self, msg: PoseStamped):
        base_T_gripper = tu.pose_msg_to_matrix(msg.pose)
        camera_T_target = tu.compose_transform(
            tu.invert_transform(self.gripper_T_camera),
            tu.invert_transform(base_T_gripper),
            self.base_T_target)
        if self.noise_m > 0 or self.noise_deg > 0:
            dt = self._rng.normal(0, self.noise_m, 3)
            dang = np.deg2rad(self._rng.normal(0, self.noise_deg, 3))
            perturb = tu.make_transform(
                tu.euler_to_matrix(*dang), dt)
            camera_T_target = tu.compose_transform(camera_T_target, perturb)

        det = MarkerDetection()
        det.header = msg.header             # SAME timestamp as robot pose
        det.header.frame_id = self.camera_frame
        det.detected = True
        det.marker_id = self.target_id
        det.pose = tu.matrix_to_pose_msg(camera_T_target)
        det.reprojection_error = 0.2
        det.stability_score = 1.0
        det.rejection_reason = ""
        self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticMarkerPublisher()
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

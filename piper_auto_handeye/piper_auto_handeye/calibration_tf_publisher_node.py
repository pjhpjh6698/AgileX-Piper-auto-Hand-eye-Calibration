#!/usr/bin/env python3
"""Static TF publisher for the hand-eye result.

Loads a calibration YAML and broadcasts the STATIC transform
    parent = gripper_frame  ->  child = camera_color_optical_frame
which is gripper_T_camera (the direct output of calibrateHandEye, Eye-in-Hand).

Verify with:
    ros2 run tf2_ros tf2_echo <gripper_frame> <camera_frame>
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from auto_handeye_interfaces.srv import PublishCalibrationTf

from . import transform_utils as tu
from . import calibration_io as cio


class CalibrationTfPublisherNode(Node):
    def __init__(self):
        super().__init__("calibration_tf_publisher_node")
        p = self.declare_parameter
        self.calibration_file = p("calibration_file", "").value
        self.output_dir = p("output_directory", "").value or cio.default_output_dir()
        self.override_parent = p("parent_frame", "").value
        self.override_child = p("child_frame", "").value
        self.auto_publish = bool(p("auto_publish", True).value)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._last_key = None

        self.create_service(PublishCalibrationTf, "publish_calibration_tf", self._srv)

        if self.auto_publish:
            path = self.calibration_file or self._latest_result_path()
            if path and os.path.exists(path):
                ok, msg, _, _ = self._publish_from_file(path)
                self.get_logger().info(f"auto-publish: {msg}")
            else:
                self.get_logger().warn(
                    "no calibration file to auto-publish; call publish_calibration_tf service")

    def _latest_result_path(self):
        try:
            files = [os.path.join(self.output_dir, f) for f in os.listdir(self.output_dir)
                     if f.startswith("handeye_") and f.endswith(".yaml")
                     and not f.endswith("_samples.yaml")]
            return max(files, key=os.path.getmtime) if files else ""
        except Exception:  # noqa: BLE001
            return ""

    def _publish_from_file(self, path):
        try:
            d = cio.load_result(path)
            T, parent, child = cio.result_to_transform(d)
        except Exception as exc:  # noqa: BLE001
            return False, f"failed to load '{path}': {exc}", "", ""

        parent = self.override_parent or parent
        child = self.override_child or child
        if not parent or not child:
            return False, "parent/child frame missing", parent, child
        if parent == child:
            return False, f"parent == child ('{parent}'): refusing to publish", parent, child

        R, t = tu.decompose_transform(T)
        if not tu.is_valid_rotation(R, tol=1e-2) or not np.all(np.isfinite(t)):
            return False, "invalid transform (rotation/translation)", parent, child

        q = tu.matrix_to_quaternion(R)  # normalized
        key = (parent, child, tuple(np.round(t, 9)), tuple(np.round(q, 9)))
        if key == self._last_key:
            return True, f"already publishing {parent}->{child} (unchanged)", parent, child

        tfmsg = TransformStamped()
        tfmsg.header.stamp = self.get_clock().now().to_msg()
        tfmsg.header.frame_id = parent
        tfmsg.child_frame_id = child
        tfmsg.transform.translation.x = float(t[0])
        tfmsg.transform.translation.y = float(t[1])
        tfmsg.transform.translation.z = float(t[2])
        tfmsg.transform.rotation.x = float(q[0])
        tfmsg.transform.rotation.y = float(q[1])
        tfmsg.transform.rotation.z = float(q[2])
        tfmsg.transform.rotation.w = float(q[3])
        self._broadcaster.sendTransform(tfmsg)
        self._last_key = key
        self.get_logger().info(
            f"published static TF {parent} -> {child} "
            f"t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})")
        return True, f"published {parent}->{child}", parent, child

    def _srv(self, req, resp):
        if not req.publish:
            resp.success = True
            resp.message = ("stop requested; static TF cannot be un-published without "
                            "restart (latched). Restart node to clear.")
            return resp
        path = req.path or self.calibration_file or self._latest_result_path()
        if not path or not os.path.exists(path):
            resp.success = False
            resp.message = f"calibration file not found: {path}"
            return resp
        ok, msg, parent, child = self._publish_from_file(path)
        resp.success = ok
        resp.message = msg
        resp.parent_frame = parent
        resp.child_frame = child
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTfPublisherNode()
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

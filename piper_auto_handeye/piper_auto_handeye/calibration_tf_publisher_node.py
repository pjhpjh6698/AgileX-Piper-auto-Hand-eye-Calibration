#!/usr/bin/env python3
"""Static TF publisher for the hand-eye result.

Loads a calibration YAML and broadcasts gripper_T_camera (the direct output of
calibrateHandEye, Eye-in-Hand) as a STATIC transform.

RE-TARGETING, and why it matters
--------------------------------
The calibration is measured to the camera's OPTICAL frame, because that is the
frame the marker detector works in. But when the RealSense driver is running it
already owns that frame: it publishes

    camera_link -> camera_color_frame -> camera_color_optical_frame

Publishing gripper -> camera_color_optical_frame on top of that gives the
optical frame TWO parents. tf2 allows exactly one, so the tree SPLITS into
disconnected halves -- RViz then cannot relate the camera to the arm at all
("Could not find a connection ... not part of the same tree"), and every
lookup from base_link to the camera fails.

So by default the transform is re-targeted onto the camera's ROOT frame:

    gripper_T_root = gripper_T_optical  @  optical_T_root

with optical_T_root read from the driver's own static TF. The published edge
becomes gripper -> camera_link, the driver keeps its internal chain, every
frame has one parent, and base_link -> camera_color_optical_frame resolves
through the whole chain. The calibration itself is unchanged -- this only
chooses which end of the camera's rigid body to attach.

Set retarget_frame to "" to publish straight to the optical frame (correct when
no camera driver is running, e.g. replaying a result offline).

Verify with:
    ros2 run tf2_ros tf2_echo <gripper_frame> <camera_frame>
    ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

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
        # Camera root to attach to; "" publishes straight to the optical frame.
        self.retarget_frame = p("retarget_frame", "camera_link").value
        self.retarget_timeout = float(p("retarget_timeout", 5.0).value)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_key = None

        self.create_service(PublishCalibrationTf, "publish_calibration_tf", self._srv)

        if self.auto_publish:
            path = self.calibration_file or self._latest_result_path()
            if path and os.path.exists(path):
                # Deferred, not immediate: re-targeting needs the camera
                # driver's static TF, and the TF buffer only fills once the
                # executor spins -- which it does not do inside __init__. A
                # blocking lookup here would always time out.
                self._auto_path = path
                self._auto_deadline = None
                self._auto_timer = self.create_timer(0.5, self._try_auto_publish)
            else:
                self.get_logger().warn(
                    "no calibration file to auto-publish; call publish_calibration_tf service")

    def _latest_result_path(self):
        try:
            files = [os.path.join(self.output_dir, f) for f in os.listdir(self.output_dir)
                     if f.startswith("handeye_") and f.endswith(".yaml")
                     and not f.endswith("_samples.yaml")]
            # The fixed filename is the canonical result; timestamped files
            # are leftovers from the old format.
            fixed = cio.default_result_path()
            if fixed in files:
                return fixed
            return max(files, key=os.path.getmtime) if files else ""
        except Exception:  # noqa: BLE001
            return ""

    def _try_auto_publish(self):
        """Publish once the camera's TF is available, or give up and publish anyway.

        Retries while the re-target lookup is still failing, so a camera driver
        that is slower to start than we are does not cost us the correct tree.
        """
        import time as _time
        if self._auto_deadline is None:
            self._auto_deadline = _time.time() + self.retarget_timeout
        want_retarget = bool(self.retarget_frame)
        ready = (not want_retarget) or self._tf_buffer.can_transform(
            self.override_child or "camera_color_optical_frame",
            self.retarget_frame, rclpy.time.Time())
        if not ready and _time.time() < self._auto_deadline:
            return                       # keep waiting for the camera driver
        self._auto_timer.cancel()
        ok, msg, _, _ = self._publish_from_file(self._auto_path)
        self.get_logger().info(f"auto-publish: {msg}")

    def _retarget(self, T, child):
        """Move the published edge from the optical frame to the camera root.

        Returns (transform, new_child, note). Falls back to the optical frame,
        loudly, when the camera driver is not publishing its internal chain --
        an orphaned-but-published TF is easier to debug than a silent no-op.
        """
        target = self.retarget_frame
        if not target or target == child:
            return T, child, ""
        try:
            # No timeout: the caller has already waited for availability. A
            # blocking wait here would deadlock when called from a callback.
            tfs = self._tf_buffer.lookup_transform(child, target, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(
                f"cannot look up {child}->{target} ({exc}); publishing straight "
                f"to '{child}'. If the camera driver is running this WILL split "
                f"the TF tree -- start the camera first, or set retarget_frame:=''")
            return T, child, "no retarget"
        q = tfs.transform.rotation
        tr = tfs.transform.translation
        optical_T_root = tu.make_transform(
            tu.quaternion_to_matrix([q.x, q.y, q.z, q.w]), [tr.x, tr.y, tr.z])
        self.get_logger().info(
            f"re-targeting {child} -> {target} so the camera keeps one parent")
        return T @ optical_T_root, target, f"retargeted to {target}"

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

        T, child, note = self._retarget(T, child)

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

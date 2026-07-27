#!/usr/bin/env python3
"""Report the simulation's ground-truth gripper_T_camera, and score a result.

In Gazebo the answer is known exactly: it is the camera mount defined in the
xacro. Rather than hard-coding it (which would silently go stale the moment
somebody retunes the mount), this node reads it from tf:

    ground truth = link6 -> camera_color_optical_frame

Run it alone to print the ground truth, or point it at a calibration YAML to
get the error of that calibration:

    ros2 run piper_auto_handeye_sim ground_truth_reporter_node
    ros2 run piper_auto_handeye_sim ground_truth_reporter_node \
        --ros-args -p calibration_file:=<path to handeye_*.yaml>

This is the whole reason for simulating: on real hardware you can only measure
self-consistency (RMS), never true accuracy. Here you can measure both.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

import tf2_ros

from piper_auto_handeye import transform_utils as tu
from piper_auto_handeye import calibration_io as cio


class GroundTruthReporterNode(Node):
    def __init__(self):
        super().__init__("ground_truth_reporter_node")
        p = self.declare_parameter
        self.gripper_frame = p("gripper_frame", "link6").value
        self.camera_frame = p("camera_frame", "camera_color_optical_frame").value
        self.calibration_file = p("calibration_file", "").value
        self.output_dir = p("output_directory", "").value or cio.default_output_dir()
        self.once = bool(p("once", True).value)

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._reported = False
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"waiting for tf {self.gripper_frame} -> {self.camera_frame} ...")

    # ------------------------------------------------------------------ #
    def _lookup_ground_truth(self):
        try:
            tf = self._buffer.lookup_transform(
                self.gripper_frame, self.camera_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return tu.pose_msg_to_matrix(tf.transform)

    def _latest_result_path(self):
        try:
            files = [os.path.join(self.output_dir, f)
                     for f in os.listdir(self.output_dir)
                     if f.startswith("handeye_") and f.endswith(".yaml")
                     and not f.endswith("_samples.yaml")]
            return max(files, key=os.path.getmtime) if files else ""
        except Exception:  # noqa: BLE001
            return ""

    def _tick(self):
        if self._reported and self.once:
            return
        T_gt = self._lookup_ground_truth()
        if T_gt is None:
            self.get_logger().info("tf not available yet ...", throttle_duration_sec=5.0)
            return

        self._reported = True
        self._print_transform("GROUND TRUTH  gripper_T_camera "
                              f"({self.gripper_frame} -> {self.camera_frame})", T_gt)

        path = self.calibration_file or self._latest_result_path()
        if not path or not os.path.exists(path):
            self.get_logger().info(
                "no calibration file to compare yet. Run a calibration, then "
                "restart this node (or pass calibration_file:=...).")
            return

        try:
            T_est = cio.result_to_transform(cio.load_result(path))[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to load '{path}': {exc}")
            return

        self._print_transform(f"ESTIMATE      from {os.path.basename(path)}", T_est)
        self._print_error(T_gt, T_est)

    # ------------------------------------------------------------------ #
    def _print_transform(self, title, T):
        R, t = tu.decompose_transform(T)
        q = tu.matrix_to_quaternion(R)
        r, pi, y = tu.matrix_to_euler(R)
        log = self.get_logger()
        log.info("=" * 70)
        log.info(title)
        log.info(f"  t   (m)  : [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}]")
        log.info(f"  q  xyzw  : [{q[0]:+.6f}, {q[1]:+.6f}, {q[2]:+.6f}, {q[3]:+.6f}]")
        log.info(f"  rpy (deg): [{math.degrees(r):+.4f}, {math.degrees(pi):+.4f}, "
                 f"{math.degrees(y):+.4f}]")

    def _print_error(self, T_gt, T_est):
        _, t_gt = tu.decompose_transform(T_gt)
        _, t_est = tu.decompose_transform(T_est)
        d_t = np.linalg.norm(t_est - t_gt)
        d_r = math.degrees(tu.rotation_angle_between(T_gt, T_est))

        log = self.get_logger()
        log.info("=" * 70)
        log.info("TRUE ERROR vs ground truth  (not self-consistency -- real accuracy)")
        log.info(f"  translation : {d_t*1000:9.4f} mm")
        log.info(f"  rotation    : {d_r:9.5f} deg")
        log.info(f"  per-axis dt : [{(t_est[0]-t_gt[0])*1000:+.4f}, "
                 f"{(t_est[1]-t_gt[1])*1000:+.4f}, "
                 f"{(t_est[2]-t_gt[2])*1000:+.4f}] mm")
        # Thresholds are for a noise-free renderer; real cameras will be worse.
        if d_t < 0.002 and d_r < 0.5:
            log.info("  VERDICT     : EXCELLENT (< 2 mm, < 0.5 deg)")
        elif d_t < 0.005 and d_r < 1.0:
            log.info("  VERDICT     : GOOD (< 5 mm, < 1 deg)")
        elif d_t < 0.02 and d_r < 3.0:
            log.warn("  VERDICT     : MARGINAL -- check pose diversity and marker_length")
        else:
            log.error("  VERDICT     : BAD -- suspect a frame/convention error, "
                      "not just noise")
        log.info("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthReporterNode()
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

#!/usr/bin/env python3
"""ArUco target detector node.

Detects a SINGLE fixed ArUco marker (target_marker_id) and publishes its pose
as camera_T_target. Version-safe against OpenCV 4.5 (no ArucoDetector class)
and 4.7+ (no estimatePoseSingleMarkers) by using getPredefinedDictionary +
detectMarkers + solvePnP(IPPE_SQUARE) everywhere.

Publishes:
  ~/marker_detection (auto_handeye_interfaces/MarkerDetection)
  ~/target_pose      (geometry_msgs/PoseStamped)  camera_T_target, filtered
  ~/debug_image      (sensor_msgs/Image)          annotated (optional)
  TF camera_frame -> target_frame                 (optional, debug only)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from auto_handeye_interfaces.msg import MarkerDetection

from . import transform_utils as tu
from .pose_filter import PoseFilter


def _get_dictionary(name: str):
    """Version-safe predefined dictionary lookup."""
    aruco = cv2.aruco
    dict_id = getattr(aruco, name, None)
    if dict_id is None:
        dict_id = aruco.DICT_4X4_50
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)  # very old API


def _make_detector(dictionary):
    """Return a callable detect(gray) -> (corners, ids). Handles both APIs."""
    aruco = cv2.aruco
    if hasattr(aruco, "ArucoDetector"):          # OpenCV >= 4.7
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
        return detect
    else:                                        # OpenCV 4.5/4.6
        params = (aruco.DetectorParameters_create()
                  if hasattr(aruco, "DetectorParameters_create")
                  else aruco.DetectorParameters())

        def detect(gray):
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
            return corners, ids
        return detect


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        p = self.declare_parameter
        self.image_topic = p("image_topic", "/camera/camera/color/image_raw").value
        self.info_topic = p("camera_info_topic", "/camera/camera/color/camera_info").value
        self.dict_name = p("aruco_dictionary", "DICT_4X4_50").value
        self.target_id = int(p("target_marker_id", 1).value)
        self.marker_length = float(p("marker_length", 0.07).value)
        self.camera_frame = p("camera_frame", "camera_color_optical_frame").value
        self.target_frame = p("target_frame", "calibration_target").value
        self.publish_debug = bool(p("publish_debug_image", True).value)
        self.publish_tf = bool(p("publish_tf", False).value)
        self.max_reproj = float(p("maximum_reprojection_error", 3.0).value)
        self.min_area = float(p("minimum_marker_area", 400.0).value)
        self.border_margin = float(p("border_margin_px", 5.0).value)
        self.filter_window = int(p("pose_filter_window", 5).value)
        self.max_t_jump = float(p("max_translation_jump_m", 0.05).value)
        self.max_r_jump = float(p("max_rotation_jump_deg", 15.0).value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.dictionary = _get_dictionary(self.dict_name)
        self.detect = _make_detector(self.dictionary)
        self.pose_filter = PoseFilter(self.filter_window, self.max_t_jump, self.max_r_jump)

        # marker object points in its own frame (center origin, +z out of marker)
        h = self.marker_length / 2.0
        self.obj_points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                                   dtype=np.float32)

        self.det_pub = self.create_publisher(MarkerDetection, "marker_detection", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self.debug_pub = (self.create_publisher(Image, "debug_image", 1)
                          if self.publish_debug else None)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f"aruco_detector_node up | OpenCV {cv2.__version__} | dict={self.dict_name} "
            f"| target_id={self.target_id} | marker_length={self.marker_length} m")
        self.get_logger().info(
            f"subscribing image='{self.image_topic}' info='{self.info_topic}'")

    # ------------------------------------------------------------------ #
    def info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info("camera intrinsics received")

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failure: {exc}")
            return

        # Still publish the live view before intrinsics arrive, so the GUI always
        # shows the camera feed instead of a blank panel.
        if self.camera_matrix is None:
            self._publish_failure(msg.header, "waiting_for_camera_info")
            if self.debug_pub is not None:
                self._publish_debug(frame, None, None, None, None,
                                    "waiting for camera_info")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)

        reason, marker_corners = self._select_target(corners, ids, frame.shape)
        if reason:
            self.pose_filter.reset()
            self._publish_failure(msg.header, reason)
            if self.debug_pub is not None:
                self._publish_debug(frame, corners, ids, None, None, reason)
            return

        ok, rvec, tvec, reproj = self._solve(marker_corners)
        if not ok:
            self._publish_failure(msg.header, "solvepnp_failed")
            return
        if reproj > self.max_reproj:
            self._publish_failure(msg.header, f"reproj_error_{reproj:.2f}px")
            if self.debug_pub is not None:
                self._publish_debug(frame, corners, ids, rvec, tvec, f"reproj {reproj:.2f}")
            return

        R, _ = cv2.Rodrigues(rvec)
        if not tu.is_valid_rotation(R, tol=1e-2) or not np.all(np.isfinite(tvec)):
            self._publish_failure(msg.header, "invalid_pose")
            return

        camera_T_target = tu.make_transform(R, tvec.reshape(3))
        filtered, accepted = self.pose_filter.add(camera_T_target)
        stability = self.pose_filter.stability_score()

        # Publish using the SAME timestamp as the source image (critical for
        # time-syncing robot pose and marker pose downstream).
        pub_T = filtered if filtered is not None else camera_T_target
        self._publish_detection(msg.header, pub_T, reproj, stability,
                                "" if accepted else "pose_jump_filtered")
        if self.debug_pub is not None:
            self._publish_debug(frame, corners, ids, rvec, tvec,
                                f"OK reproj={reproj:.2f} stab={stability:.2f}")

    # ------------------------------------------------------------------ #
    def _select_target(self, corners, ids, shape):
        if ids is None or len(ids) == 0:
            return "no_markers", None
        h, w = shape[:2]
        for i, mid in enumerate(ids.flatten()):
            if int(mid) != self.target_id:
                continue
            c = corners[i].reshape(4, 2)
            area = cv2.contourArea(c.astype(np.float32))
            if area < self.min_area:
                return f"marker_too_small_{area:.0f}px", None
            m = self.border_margin
            if (c[:, 0].min() < m or c[:, 1].min() < m
                    or c[:, 0].max() > w - m or c[:, 1].max() > h - m):
                return "marker_at_border", None
            return "", c
        return f"target_id_{self.target_id}_not_seen", None

    def _solve(self, img_points):
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_points, img_points.astype(np.float32),
            self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return False, None, None, 0.0
        proj, _ = cv2.projectPoints(self.obj_points, rvec, tvec,
                                    self.camera_matrix, self.dist_coeffs)
        reproj = float(np.sqrt(np.mean(np.square(
            proj.reshape(-1, 2) - img_points))))
        return True, rvec, tvec, reproj

    # ------------------------------------------------------------------ #
    def _publish_detection(self, header, T, reproj, stability, reason):
        det = MarkerDetection()
        det.header = header
        det.header.frame_id = self.camera_frame
        det.detected = True
        det.marker_id = self.target_id
        det.pose = tu.matrix_to_pose_msg(T)
        det.reprojection_error = float(reproj)
        det.stability_score = float(stability)
        det.rejection_reason = reason
        self.det_pub.publish(det)

        ps = PoseStamped()
        ps.header = det.header
        ps.pose = det.pose
        self.pose_pub.publish(ps)

        if self.tf_broadcaster is not None:
            tfmsg = TransformStamped()
            tfmsg.header = det.header
            tfmsg.child_frame_id = self.target_frame
            tr = tu.matrix_to_transform_msg(T)
            tfmsg.transform = tr
            self.tf_broadcaster.sendTransform(tfmsg)

    def _publish_failure(self, header, reason):
        det = MarkerDetection()
        det.header = header
        det.header.frame_id = self.camera_frame
        det.detected = False
        det.marker_id = self.target_id
        det.reprojection_error = 0.0
        det.stability_score = 0.0
        det.rejection_reason = reason
        self.det_pub.publish(det)

    def _publish_debug(self, frame, corners, ids, rvec, tvec, status):
        img = frame.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
        if rvec is not None and tvec is not None:
            try:
                cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_length * 0.5)
            except AttributeError:
                pass
        color = (0, 200, 0) if status.startswith("OK") else (0, 0, 255)
        cv2.putText(img, f"target#{self.target_id}: {status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"debug image publish failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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

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

import math

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


def _make_params(refine="subpix", win_size=5, max_iter=50, min_accuracy=0.01,
                 logger=None):
    """Build DetectorParameters with sub-pixel corner refinement enabled.

    This matters more than anything else downstream. With the OpenCV defaults
    (CORNER_REFINE_NONE) marker corners land on whole pixels, and since
    solvePnP turns corner positions into a pose, a one-pixel corner wobble
    becomes degrees of orientation jitter at typical calibration distances --
    which is exactly the "unstable detection" that makes samples get rejected.
    Sub-pixel refinement costs a fraction of a millisecond per frame.
    """
    aruco = cv2.aruco
    params = (aruco.DetectorParameters_create()
              if hasattr(aruco, "DetectorParameters_create")
              else aruco.DetectorParameters())

    modes = {
        "none": getattr(aruco, "CORNER_REFINE_NONE", 0),
        "subpix": getattr(aruco, "CORNER_REFINE_SUBPIX", 1),
        "contour": getattr(aruco, "CORNER_REFINE_CONTOUR", 2),
        "apriltag": getattr(aruco, "CORNER_REFINE_APRILTAG", 3),
    }
    mode = modes.get(str(refine).lower())
    if mode is None:
        if logger:
            logger.warn(f"unknown corner_refinement '{refine}', falling back to subpix")
        mode = modes["subpix"]
    try:
        params.cornerRefinementMethod = mode
        params.cornerRefinementWinSize = int(win_size)
        params.cornerRefinementMaxIterations = int(max_iter)
        params.cornerRefinementMinAccuracy = float(min_accuracy)
    except AttributeError as exc:   # very old cv2.aruco builds
        if logger:
            logger.warn(f"corner refinement unavailable in this OpenCV build: {exc}")
    return params


def _make_detector(dictionary, params):
    """Return a callable detect(gray) -> (corners, ids). Handles both APIs."""
    aruco = cv2.aruco
    if hasattr(aruco, "ArucoDetector"):          # OpenCV >= 4.7
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
        return detect

    def detect(gray):                            # OpenCV 4.5/4.6
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
        return corners, ids
    return detect


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        p = self.declare_parameter
        self.image_topic = p("image_topic", "/camera/camera/color/image_raw").value
        self.info_topic = p("camera_info_topic", "/camera/camera/color/camera_info").value
        # 'aruco' = single marker; 'charuco' = ChArUco board (chessboard corners
        # interpolated from the markers -- many corners, much steadier pose).
        # Runtime-switchable via the parameter service; the GUI exposes it.
        self.target_type = str(p("target_type", "aruco").value).lower()
        self.dict_name = p("aruco_dictionary", "DICT_4X4_50").value
        self.target_id = int(p("target_marker_id", 1).value)
        self.marker_length = float(p("marker_length", 0.07).value)
        self.ch_squares_x = int(p("charuco_squares_x", 5).value)
        self.ch_squares_y = int(p("charuco_squares_y", 7).value)
        self.ch_square_len = float(p("charuco_square_length", 0.035).value)
        self.ch_marker_len = float(p("charuco_marker_length", 0.026).value)
        self.ch_min_corners = int(p("charuco_min_corners", 6).value)
        # Planar pose-flip handling: below this best/second reprojection-error
        # ratio the two PnP branches are considered indistinguishable and the
        # frame is disambiguated against history (or rejected). See _solve.
        self.ambiguity_ratio = float(p("ambiguity_min_ratio", 1.3).value)
        self._last_R = None
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
        self.corner_refinement = p("corner_refinement", "subpix").value
        self.refine_win = int(p("corner_refinement_win_size", 5).value)
        self.refine_iter = int(p("corner_refinement_max_iterations", 50).value)
        self.refine_acc = float(p("corner_refinement_min_accuracy", 0.01).value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.dictionary = _get_dictionary(self.dict_name)
        self.det_params = _make_params(self.corner_refinement, self.refine_win,
                                       self.refine_iter, self.refine_acc,
                                       self.get_logger())
        self.detect = _make_detector(self.dictionary, self.det_params)
        self.board = None
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            self.board = cv2.aruco.CharucoBoard_create(
                self.ch_squares_x, self.ch_squares_y,
                self.ch_square_len, self.ch_marker_len, self.dictionary)
        # allow the GUI to flip aruco <-> charuco while running
        self.add_on_set_parameters_callback(self._on_set_parameters)
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
            f"| target_id={self.target_id} | marker_length={self.marker_length} m "
            f"| corner_refinement={self.corner_refinement}")
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

        if self.target_type == "charuco":
            reason, rvec, tvec, reproj = self._solve_charuco(gray, corners, ids,
                                                             frame.shape)
            if reason:
                self.pose_filter.reset()
                self._publish_failure(msg.header, reason)
                if self.debug_pub is not None:
                    self._publish_debug(frame, corners, ids, None, None, reason)
                return
            ok = True
        else:
            reason, marker_corners = self._select_target(corners, ids, frame.shape)
            if reason:
                self.pose_filter.reset()
                self._last_R = None
                self._publish_failure(msg.header, reason)
                if self.debug_pub is not None:
                    self._publish_debug(frame, corners, ids, None, None, reason)
                return
            ok, rvec, tvec, reproj, why = self._solve(marker_corners)
            if not ok:
                self._publish_failure(msg.header, why or "solvepnp_failed")
                return
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
        self._last_R = R      # history for the ambiguity disambiguation

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
        # Say which IDs WERE decoded: "expects 1, saw [0]" turns a mystery
        # blackout into an obvious one-line config fix.
        seen = sorted({int(m) for m in ids.flatten()})
        return f"target_id_{self.target_id}_not_seen(saw:{seen})", None

    def _solve_charuco(self, gray, corners, ids, shape):
        """ChArUco path: interpolate chessboard corners, then board pose.

        Returns (reason, rvec, tvec, reproj); reason == "" on success. The
        published pose is camera_T_board (board origin per the OpenCV board
        definition), which slots into the calibration exactly like a marker
        pose -- the solver only needs a rigid target, not any particular one.
        """
        if self.board is None:
            return "charuco_unsupported_opencv", None, None, None
        if ids is None or len(ids) == 0:
            return "no_markers", None, None, None
        n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board,
            cameraMatrix=self.camera_matrix, distCoeffs=self.dist_coeffs)
        if ch_corners is None or n < self.ch_min_corners:
            return f"charuco_corners_{0 if n is None else n}<{self.ch_min_corners}", \
                None, None, None
        h, w = shape[:2]
        m = self.border_margin
        pts = ch_corners.reshape(-1, 2)
        if (pts[:, 0].min() < m or pts[:, 1].min() < m
                or pts[:, 0].max() > w - m or pts[:, 1].max() > h - m):
            return "board_at_border", None, None, None
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.board,
            self.camera_matrix, self.dist_coeffs, None, None)
        if not ok:
            return "charuco_pose_failed", None, None, None
        # reprojection error over the interpolated chessboard corners
        obj = self.board.chessboardCorners[ch_ids.flatten()]
        proj, _ = cv2.projectPoints(obj, rvec, tvec,
                                    self.camera_matrix, self.dist_coeffs)
        reproj = float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - pts) ** 2, axis=1))))
        if reproj > self.max_reproj:
            return f"reproj_error_{reproj:.2f}px", None, None, None
        return "", rvec, tvec, reproj

    def _on_set_parameters(self, params):
        """Runtime switches (used by the GUI): target type, marker ID, size.

        marker_length feeds solvePnP's object points, so a wrong value scales
        every camera_T_target translation by the same ratio and poisons the
        hand-eye translation -- being able to correct it live, to match
        whatever marker is actually on the table, matters.
        """
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == "target_type":
                value = str(prm.value).lower()
                if value not in ("aruco", "charuco"):
                    return SetParametersResult(
                        successful=False,
                        reason=f"target_type must be aruco|charuco, got {prm.value!r}")
                if value == "charuco" and self.board is None:
                    return SetParametersResult(
                        successful=False,
                        reason="this OpenCV build has no CharucoBoard support")
                self.target_type = value
                self.pose_filter.reset()   # pose stream changes target: restart clean
                self._last_R = None
                self.get_logger().info(f"target_type -> {value}")
            elif prm.name == "target_marker_id":
                try:
                    value = int(prm.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="target_marker_id must be an integer")
                if not 0 <= value <= 999:
                    return SetParametersResult(
                        successful=False,
                        reason=f"target_marker_id out of range: {value}")
                self.target_id = value
                self.pose_filter.reset()
                self._last_R = None
                self.get_logger().info(f"target_marker_id -> {value}")
            elif prm.name == "marker_length":
                try:
                    value = float(prm.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="marker_length must be a number")
                if not 0.005 <= value <= 0.5:
                    return SetParametersResult(
                        successful=False,
                        reason=f"marker_length must be 0.005..0.5 m, got {value}")
                self.marker_length = value
                h = value / 2.0
                self.obj_points = np.array(
                    [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                    dtype=np.float32)
                self.pose_filter.reset()
                self._last_R = None
                self.get_logger().info(f"marker_length -> {value * 1000:.1f} mm")
        return SetParametersResult(successful=True)

    def _solve(self, img_points):
        """Marker pose with the planar POSE-FLIP AMBIGUITY handled.

        A square marker viewed near-frontally has two PnP solutions tilted
        symmetrically about the viewing ray, with almost identical
        reprojection error. Taking solvePnP's single answer lets the winning
        branch flip frame to frame; each flip corrupts camera_T_target by tens
        of degrees, and a hand-eye run fed a mix of both branches "converges"
        to a garbage extrinsic with ~15 deg rotation RMS -- while every
        individual detection still looks perfectly fine.

        So: get BOTH solutions (solvePnPGeneric), and
          * if one clearly reprojects better (ratio >= ambiguity_min_ratio),
            take it;
          * otherwise disambiguate against the last published pose -- the
            marker and camera cannot have teleported between frames;
          * with no history and no clear winner, REJECT the frame. A dropped
            frame costs milliseconds; a flipped one poisons the whole solve.
        """
        pts = img_points.astype(np.float32)
        try:
            n_sol, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                self.obj_points, pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return False, None, None, 0.0, "solvepnp_failed"
        if not n_sol:
            return False, None, None, 0.0, "solvepnp_failed"

        errs = [float(np.ravel(e)[0]) for e in errs]
        order = np.argsort(errs)
        best, second = order[0], (order[1] if n_sol > 1 else None)

        pick = best
        if second is not None:
            e0, e1 = errs[best], errs[second]
            if e1 / max(e0, 1e-9) < self.ambiguity_ratio:
                # Reprojection cannot tell the branches apart -- use history.
                if self._last_R is None:
                    return False, None, None, 0.0, "pose_ambiguous"
                angs = []
                for i in (best, second):
                    R, _ = cv2.Rodrigues(rvecs[i])
                    angs.append(math.degrees(tu.rotation_angle(self._last_R.T @ R)))
                pick = (best, second)[int(np.argmin(angs))]
                if min(angs) > self.max_r_jump:
                    # neither branch continues the track: something else moved
                    return False, None, None, 0.0, "pose_ambiguous"
        return True, rvecs[pick], tvecs[pick], errs[pick], ""

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

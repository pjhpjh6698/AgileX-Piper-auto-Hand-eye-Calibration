#!/usr/bin/env python3
"""Qt widget for the Piper Eye-in-Hand calibration GUI.

Talks to the calibration stack ONLY through ROS interfaces:

  subscribes  calibration_status (CalibrationStatus)  state machine progress
              robot_state        (RobotState)         base_T_gripper + health
              marker_detection   (MarkerDetection)    camera_T_target + quality
              debug_image        (sensor_msgs/Image)  annotated camera view
  action      run_calibration    (RunCalibration)     start / cancel
  services    add_manual_sample, reset_calibration, save_calibration,
              load_calibration, pause_calibration, resume_calibration,
              stop_motion, clear_stop, publish_calibration_tf

Threading
---------
ROS callbacks arrive on the executor thread that rqt spins, NOT the Qt GUI
thread. Touching widgets from there crashes Qt, so every callback only emits a
Qt signal; the connected slot runs on the GUI thread (Qt queues cross-thread
emissions automatically). All widget updates live in the ``_on_*`` slots.

Images are converted with numpy rather than cv_bridge so the GUI package needs
no OpenCV dependency of its own.
"""

import math
import os

import numpy as np

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPlainTextEdit, QProgressBar, QPushButton, QSpinBox,
    QSplitter, QVBoxLayout, QWidget)

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

import rclpy                     # for rclpy.ok() in the shutdown guards
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from auto_handeye_interfaces.action import RunCalibration
from auto_handeye_interfaces.msg import CalibrationStatus, MarkerDetection, RobotState
from auto_handeye_interfaces.srv import (AddManualSample, LoadCalibration,
                                         PublishCalibrationTf, ResetCalibration,
                                         SaveCalibration)

METHODS = ["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"]

# state -> (background, foreground) for the big state banner
_STATE_COLORS = {
    "IDLE": ("#455a64", "#ffffff"),
    "CHECKING_SYSTEM": ("#0277bd", "#ffffff"),
    "MOVING": ("#ef6c00", "#ffffff"),
    "SETTLING": ("#f9a825", "#000000"),
    "WAITING_FOR_MARKER": ("#f9a825", "#000000"),
    "COLLECTING": ("#0288d1", "#ffffff"),
    "SAMPLE_ACCEPTED": ("#2e7d32", "#ffffff"),
    "SAMPLE_REJECTED": ("#c62828", "#ffffff"),
    "SOLVING": ("#6a1b9a", "#ffffff"),
    "VALIDATING": ("#6a1b9a", "#ffffff"),
    "SUCCESS": ("#1b5e20", "#ffffff"),
    "WARNING": ("#ef6c00", "#ffffff"),
    "FAILED": ("#b71c1c", "#ffffff"),
    "PAUSED": ("#616161", "#ffffff"),
    "CANCELED": ("#616161", "#ffffff"),
}


def _ok_label(text, ok):
    color = "#2e7d32" if ok else "#c62828"
    return f'<span style="color:{color};font-weight:bold;">{text}</span>'


class HandeyeGuiWidget(QWidget):
    """Main GUI panel. All ROS traffic is funnelled through Qt signals."""

    # ROS thread -> GUI thread
    sig_status = Signal(object)
    sig_robot = Signal(object)
    sig_marker = Signal(object)
    sig_image = Signal(object)
    sig_log = Signal(str)
    sig_result = Signal(object)
    sig_goal_done = Signal()
    sig_live = Signal(bool)

    def __init__(self, node):
        super().__init__()
        self.setObjectName("HandeyeGuiWidget")
        self.setWindowTitle("Piper Auto Hand-Eye Calibration")

        self._node = node
        self._goal_handle = None
        self._paused = False
        self._last_saved_path = ""
        self._closing = False

        self._build_ui()
        self._connect_signals()
        self._setup_ros()

        # dry_run / LIVE banner polling (the control node owns the flag)
        self._live_timer = QTimer(self)
        self._live_timer.timeout.connect(self._query_dry_run)
        self._live_timer.start(2000)
        self._query_dry_run()

        self._log("GUI ready. The GUI only talks over ROS -- closing it never "
                  "stops a running calibration.")

    # ------------------------------------------------------------------ #
    # UI construction
    # ------------------------------------------------------------------ #
    def _build_ui(self):
        root = QVBoxLayout(self)

        # ---- safety banner ----
        self.banner = QLabel("dry_run 상태 확인 중...")
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setStyleSheet(
            "background:#455a64;color:#fff;font-weight:bold;padding:6px;")
        root.addWidget(self.banner)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_right())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        root.addWidget(splitter, 1)

    # -------------------------- left column --------------------------- #
    def _build_left(self):
        w = QWidget()
        lay = QVBoxLayout(w)

        cam_box = QGroupBox("Camera (debug_image)")
        cam_lay = QVBoxLayout(cam_box)
        self.image_label = QLabel("영상 없음\n(aruco_detector_node 미실행 또는 카메라 없음)")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(480, 360)
        self.image_label.setStyleSheet("background:#212121;color:#9e9e9e;")
        cam_lay.addWidget(self.image_label)
        lay.addWidget(cam_box, 1)

        st_box = QGroupBox("Status")
        grid = QGridLayout(st_box)
        self.lbl_robot = QLabel("-")
        self.lbl_pose = QLabel("-")
        self.lbl_marker = QLabel("-")
        self.lbl_marker_q = QLabel("-")
        for lbl in (self.lbl_pose, self.lbl_marker_q):
            lbl.setStyleSheet("font-family:monospace;")
        grid.addWidget(QLabel("<b>Robot</b>"), 0, 0)
        grid.addWidget(self.lbl_robot, 0, 1)
        grid.addWidget(QLabel("<b>base_T_gripper</b>"), 1, 0)
        grid.addWidget(self.lbl_pose, 1, 1)
        grid.addWidget(QLabel("<b>Marker</b>"), 2, 0)
        grid.addWidget(self.lbl_marker, 2, 1)
        grid.addWidget(QLabel("<b>Quality</b>"), 3, 0)
        grid.addWidget(self.lbl_marker_q, 3, 1)
        grid.setColumnStretch(1, 1)
        lay.addWidget(st_box)

        log_box = QGroupBox("Log")
        log_lay = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setStyleSheet("font-family:monospace;font-size:11px;")
        log_lay.addWidget(self.log_view)
        lay.addWidget(log_box, 1)
        return w

    # -------------------------- right column -------------------------- #
    def _build_right(self):
        w = QWidget()
        lay = QVBoxLayout(w)

        # ---- settings ----
        cfg = QGroupBox("Calibration settings")
        g = QGridLayout(cfg)
        self.cmb_method = QComboBox()
        self.cmb_method.addItems(METHODS)
        self.spn_samples = QSpinBox()
        self.spn_samples.setRange(3, 100)
        self.spn_samples.setValue(15)
        self.spn_settle = QDoubleSpinBox()
        self.spn_settle.setRange(0.0, 10.0)
        self.spn_settle.setSingleStep(0.1)
        self.spn_settle.setValue(1.0)
        self.spn_settle.setSuffix(" s")
        self.spn_obs = QSpinBox()
        self.spn_obs.setRange(1, 50)
        self.spn_obs.setValue(10)
        self.chk_auto = QCheckBox("자동 이동 (auto_move)")
        self.chk_auto.setChecked(True)
        self.chk_save = QCheckBox("성공 시 자동 저장")
        self.chk_save.setChecked(True)
        g.addWidget(QLabel("Method"), 0, 0)
        g.addWidget(self.cmb_method, 0, 1)
        g.addWidget(QLabel("Target samples"), 1, 0)
        g.addWidget(self.spn_samples, 1, 1)
        g.addWidget(QLabel("Settle time"), 2, 0)
        g.addWidget(self.spn_settle, 2, 1)
        g.addWidget(QLabel("Obs / pose"), 3, 0)
        g.addWidget(self.spn_obs, 3, 1)
        g.addWidget(self.chk_auto, 4, 0, 1, 2)
        g.addWidget(self.chk_save, 5, 0, 1, 2)
        lay.addWidget(cfg)

        # ---- progress ----
        prog = QGroupBox("Progress")
        pl = QVBoxLayout(prog)
        self.lbl_state = QLabel("IDLE")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setStyleSheet(
            "background:#455a64;color:#fff;font-weight:bold;font-size:15px;padding:6px;")
        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.lbl_progress = QLabel("samples 0/15   pose -")
        self.lbl_message = QLabel("-")
        self.lbl_message.setWordWrap(True)
        pl.addWidget(self.lbl_state)
        pl.addWidget(self.bar)
        pl.addWidget(self.lbl_progress)
        pl.addWidget(self.lbl_message)
        lay.addWidget(prog)

        # ---- controls ----
        ctl = QGroupBox("Controls")
        cl = QGridLayout(ctl)
        self.btn_start = QPushButton("▶ Start")
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_cancel = QPushButton("■ Cancel")
        self.btn_reset = QPushButton("↺ Reset")
        self.btn_add = QPushButton("＋ Add sample")
        self.btn_stop = QPushButton("⛔ STOP")
        self.btn_clear_stop = QPushButton("STOP 해제")
        self.btn_start.setStyleSheet("background:#2e7d32;color:#fff;font-weight:bold;padding:8px;")
        self.btn_stop.setStyleSheet("background:#b71c1c;color:#fff;font-weight:bold;padding:8px;")
        self.btn_pause.setEnabled(False)
        self.btn_cancel.setEnabled(False)
        cl.addWidget(self.btn_start, 0, 0)
        cl.addWidget(self.btn_pause, 0, 1)
        cl.addWidget(self.btn_cancel, 1, 0)
        cl.addWidget(self.btn_reset, 1, 1)
        cl.addWidget(self.btn_add, 2, 0)
        cl.addWidget(self.btn_clear_stop, 2, 1)
        cl.addWidget(self.btn_stop, 3, 0, 1, 2)
        lay.addWidget(ctl)

        # ---- result ----
        res = QGroupBox("Result: gripper_T_camera")
        rl = QVBoxLayout(res)
        self.lbl_result = QLabel("아직 결과 없음")
        self.lbl_result.setStyleSheet("font-family:monospace;font-size:11px;")
        self.lbl_result.setTextInteractionFlags(Qt.TextSelectableByMouse)
        rl.addWidget(self.lbl_result)
        fl = QHBoxLayout()
        self.btn_save = QPushButton("Save")
        self.btn_load = QPushButton("Load")
        self.btn_pub = QPushButton("Publish TF")
        fl.addWidget(self.btn_save)
        fl.addWidget(self.btn_load)
        fl.addWidget(self.btn_pub)
        rl.addLayout(fl)
        lay.addWidget(res)

        lay.addStretch(1)
        return w

    # ------------------------------------------------------------------ #
    def _connect_signals(self):
        self.sig_status.connect(self._on_status)
        self.sig_robot.connect(self._on_robot)
        self.sig_marker.connect(self._on_marker)
        self.sig_image.connect(self._on_image)
        self.sig_log.connect(self._append_log)
        self.sig_result.connect(self._on_result)
        self.sig_goal_done.connect(self._on_goal_done)
        self.sig_live.connect(self._on_live)

        self.btn_start.clicked.connect(self._start)
        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_cancel.clicked.connect(self._cancel)
        self.btn_reset.clicked.connect(self._reset)
        self.btn_add.clicked.connect(self._add_sample)
        self.btn_stop.clicked.connect(self._stop_motion)
        self.btn_clear_stop.clicked.connect(self._clear_stop)
        self.btn_save.clicked.connect(self._save)
        self.btn_load.clicked.connect(self._load)
        self.btn_pub.clicked.connect(self._publish_tf)

    def _setup_ros(self):
        n = self._node
        self._subs = [
            n.create_subscription(CalibrationStatus, "calibration_status",
                                  lambda m: self.sig_status.emit(m), 10),
            n.create_subscription(RobotState, "robot_state",
                                  lambda m: self.sig_robot.emit(m), 10),
            n.create_subscription(MarkerDetection, "marker_detection",
                                  lambda m: self.sig_marker.emit(m), 10),
            n.create_subscription(Image, "debug_image",
                                  lambda m: self.sig_image.emit(m),
                                  qos_profile_sensor_data),
        ]
        self._run_client = ActionClient(n, RunCalibration, "run_calibration")
        self._srv = {
            "add": n.create_client(AddManualSample, "add_manual_sample"),
            "reset": n.create_client(ResetCalibration, "reset_calibration"),
            "save": n.create_client(SaveCalibration, "save_calibration"),
            "load": n.create_client(LoadCalibration, "load_calibration"),
            "pause": n.create_client(Trigger, "pause_calibration"),
            "resume": n.create_client(Trigger, "resume_calibration"),
            "stop": n.create_client(Trigger, "stop_motion"),
            "clear_stop": n.create_client(Trigger, "clear_stop"),
            "pub_tf": n.create_client(PublishCalibrationTf, "publish_calibration_tf"),
            "params": n.create_client(GetParameters,
                                      "/piper_control_node/get_parameters"),
        }

    # ------------------------------------------------------------------ #
    # ROS -> GUI slots
    # ------------------------------------------------------------------ #
    def _on_status(self, msg):
        bg, fg = _STATE_COLORS.get(msg.state, ("#455a64", "#ffffff"))
        self.lbl_state.setText(msg.state)
        self.lbl_state.setStyleSheet(
            f"background:{bg};color:{fg};font-weight:bold;font-size:15px;padding:6px;")
        self.bar.setValue(int(max(0.0, min(1.0, msg.progress)) * 100))
        pose = msg.current_pose_index
        self.lbl_progress.setText(
            f"samples {msg.current_sample_count}/{msg.target_sample_count}   "
            f"pose {pose if pose >= 0 else '-'}")
        if msg.message:
            self.lbl_message.setText(msg.message)
        if msg.translation_rms > 0.0 or msg.rotation_rms_deg > 0.0:
            self.lbl_message.setText(
                f"{msg.message}  |  RMS t={msg.translation_rms*1000:.2f} mm  "
                f"r={msg.rotation_rms_deg:.3f}°")

    def _on_robot(self, msg):
        parts = [
            _ok_label("connected" if msg.connected else "disconnected", msg.connected),
            _ok_label("enabled" if msg.enabled else "disabled", msg.enabled),
            ('<span style="color:#ef6c00;font-weight:bold;">MOVING</span>'
             if msg.moving else '<span style="color:#2e7d32;">stopped</span>'),
        ]
        if msg.error_message:
            parts.append(f'<span style="color:#c62828;">{msg.error_message}</span>')
        self.lbl_robot.setText(" | ".join(parts))

        p = msg.tcp_pose.position
        o = msg.tcp_pose.orientation
        r, pi, y = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        self.lbl_pose.setText(
            f"xyz [{p.x:+.4f} {p.y:+.4f} {p.z:+.4f}] m\n"
            f"rpy [{math.degrees(r):+7.2f} {math.degrees(pi):+7.2f} "
            f"{math.degrees(y):+7.2f}] °")

    def _on_marker(self, msg):
        if msg.detected:
            self.lbl_marker.setText(_ok_label(f"detected id={msg.marker_id}", True))
        else:
            reason = msg.rejection_reason or "not detected"
            self.lbl_marker.setText(_ok_label(reason, False))
        self.lbl_marker_q.setText(
            f"reproj {msg.reprojection_error:5.2f} px   "
            f"stability {msg.stability_score:4.2f}")

    def _on_image(self, msg):
        pix = self._image_to_pixmap(msg)
        if pix is None:
            return
        self.image_label.setPixmap(pix.scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _on_result(self, res):
        t = res.gripper_to_camera.translation
        q = res.gripper_to_camera.rotation
        r, p, y = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self._last_saved_path = res.saved_path or self._last_saved_path
        self.lbl_result.setText(
            f"state   : {res.state}\n"
            f"samples : {res.sample_count}\n"
            f"t (m)   : [{t.x:+.6f}, {t.y:+.6f}, {t.z:+.6f}]\n"
            f"q xyzw  : [{q.x:+.6f}, {q.y:+.6f}, {q.z:+.6f}, {q.w:+.6f}]\n"
            f"rpy (°) : [{math.degrees(r):+.3f}, {math.degrees(p):+.3f}, "
            f"{math.degrees(y):+.3f}]\n"
            f"RMS     : t {res.translation_rms_m*1000:.3f} mm "
            f"(max {res.translation_max_m*1000:.3f})\n"
            f"          r {res.rotation_rms_deg:.4f}° "
            f"(max {res.rotation_max_deg:.4f}°)\n"
            f"saved   : {res.saved_path or '-'}")

    def _on_goal_done(self):
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_cancel.setEnabled(False)
        self.btn_pause.setText("⏸ Pause")
        self._paused = False
        self._goal_handle = None

    def _on_live(self, live):
        if live:
            self.banner.setText("⚠  LIVE — dry_run=false : 로봇이 실제로 움직입니다. 비상정지 대기!")
            self.banner.setStyleSheet(
                "background:#b71c1c;color:#fff;font-weight:bold;padding:6px;")
        else:
            self.banner.setText("DRY-RUN — dry_run=true : 로봇은 움직이지 않습니다 (검증만)")
            self.banner.setStyleSheet(
                "background:#2e7d32;color:#fff;font-weight:bold;padding:6px;")

    # ------------------------------------------------------------------ #
    # GUI -> ROS actions
    # ------------------------------------------------------------------ #
    def _start(self):
        if not self._run_client.wait_for_server(timeout_sec=2.0):
            self._log("ERROR: run_calibration 액션 서버 없음 "
                      "(handeye_calibration_node 실행 중인지 확인)")
            return
        goal = RunCalibration.Goal()
        goal.target_sample_count = self.spn_samples.value()
        goal.auto_move = self.chk_auto.isChecked()
        goal.calibration_method = self.cmb_method.currentText()
        goal.settle_time = self.spn_settle.value()
        goal.observations_per_pose = self.spn_obs.value()
        goal.save_on_success = self.chk_save.isChecked()
        goal.output_path = ""

        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_cancel.setEnabled(True)
        self._log(f"Start: method={goal.calibration_method} "
                  f"samples={goal.target_sample_count} auto_move={goal.auto_move}")

        fut = self._run_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            gh = future.result()
        except Exception as exc:  # noqa: BLE001
            self.sig_log.emit(f"ERROR: goal 전송 실패: {exc}")
            self.sig_goal_done.emit()
            return
        if not gh.accepted:
            self.sig_log.emit("goal이 거절되었습니다.")
            self.sig_goal_done.emit()
            return
        self._goal_handle = gh
        self.sig_log.emit("goal 수락됨. 캘리브레이션 진행 중...")
        gh.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            res = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.sig_log.emit(f"ERROR: 결과 수신 실패: {exc}")
            self.sig_goal_done.emit()
            return
        self.sig_log.emit(f"완료: {res.state} — {res.message}")
        if res.saved_path:
            self.sig_log.emit(f"저장됨: {res.saved_path}")
        self.sig_result.emit(res)
        self.sig_goal_done.emit()

    def _on_feedback(self, fb):
        f = fb.feedback
        self.sig_log.emit(
            f"[{f.state}] {f.current_sample_count}/{f.target_sample_count} "
            f"pose={f.current_pose_index} {f.message}")

    def _cancel(self):
        if self._goal_handle is None:
            self._log("취소할 goal이 없습니다.")
            return
        self._log("취소 요청 전송")
        self._goal_handle.cancel_goal_async()

    def _toggle_pause(self):
        key = "resume" if self._paused else "pause"
        self._call_trigger(key, "재개" if self._paused else "일시정지")
        self._paused = not self._paused
        self.btn_pause.setText("▶ Resume" if self._paused else "⏸ Pause")

    def _stop_motion(self):
        self._call_trigger("stop", "STOP")

    def _clear_stop(self):
        self._call_trigger("clear_stop", "STOP 해제")

    def _reset(self):
        self._call(self._srv["reset"], ResetCalibration.Request(), "reset_calibration")

    def _add_sample(self):
        self._call(self._srv["add"], AddManualSample.Request(), "add_manual_sample")

    def _save(self):
        req = SaveCalibration.Request()
        req.path = ""
        self._call(self._srv["save"], req, "save_calibration")

    def _load(self):
        start = os.path.expanduser("~/.ros/piper_auto_handeye")
        path, _ = QFileDialog.getOpenFileName(
            self, "캘리브레이션 YAML 선택",
            start if os.path.isdir(start) else os.path.expanduser("~"),
            "YAML (*.yaml *.yml)")
        if not path:
            return
        req = LoadCalibration.Request()
        req.path = path
        self._last_saved_path = path
        self._call(self._srv["load"], req, "load_calibration")

    def _publish_tf(self):
        req = PublishCalibrationTf.Request()
        req.path = self._last_saved_path
        req.publish = True
        self._call(self._srv["pub_tf"], req, "publish_calibration_tf")

    # ------------------------------------------------------------------ #
    # service helpers (never block the GUI thread)
    # ------------------------------------------------------------------ #
    def _call_trigger(self, key, label):
        self._call(self._srv[key], Trigger.Request(), label)

    def _call(self, client, request, label):
        if self._closing or not rclpy.ok():
            return
        try:
            if not client.service_is_ready():
                if not client.wait_for_service(timeout_sec=1.0):
                    self._log(f"ERROR: 서비스 '{label}' 사용 불가 (노드 미실행?)")
                    return
            fut = client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self._log(f"ERROR: {label} 호출 실패: {exc}")
            return

        def done(f, label=label):
            try:
                resp = f.result()
            except Exception as exc:  # noqa: BLE001
                self.sig_log.emit(f"ERROR: {label} 실패: {exc}")
                return
            msg = getattr(resp, "message", "")
            ok = getattr(resp, "success", True)
            extra = ""
            if hasattr(resp, "sample_count"):
                extra = f" (samples={resp.sample_count})"
            elif getattr(resp, "saved_path", ""):
                extra = f" -> {resp.saved_path}"
                self._last_saved_path = resp.saved_path
            self.sig_log.emit(f"{'OK' if ok else 'FAIL'}: {label}: {msg}{extra}")

        fut.add_done_callback(done)

    def _query_dry_run(self):
        # Qt keeps firing this timer while rqt tears the ROS context down, and
        # every rclpy call then raises "context is not valid". Bail out early
        # and swallow the race rather than spraying tracebacks on exit.
        if self._closing or not rclpy.ok():
            return
        cli = self._srv["params"]
        try:
            if not cli.service_is_ready():
                return
            req = GetParameters.Request()
            req.names = ["dry_run"]
            fut = cli.call_async(req)
        except Exception:  # noqa: BLE001
            return

        def done(f):
            try:
                values = f.result().values
            except Exception:  # noqa: BLE001
                return
            if values and values[0].type == 1:  # PARAMETER_BOOL
                self.sig_live.emit(not values[0].bool_value)

        fut.add_done_callback(done)

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _quat_to_rpy(x, y, z, w):
        """Fixed-axis XYZ euler, matching transform_utils.matrix_to_euler."""
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-12:
            return 0.0, 0.0, 0.0
        x, y, z, w = x / n, y / n, z / n, w / n
        sy = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sy)
        if abs(sy) < 1.0 - 1e-9:
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        else:
            roll = math.atan2(-2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + z * z))
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _image_to_pixmap(msg):
        """sensor_msgs/Image -> QPixmap without cv_bridge."""
        try:
            enc = msg.encoding
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ("bgr8", "rgb8"):
                arr = buf.reshape(msg.height, msg.step // 1)[:, :msg.width * 3]
                arr = arr.reshape(msg.height, msg.width, 3)
                if enc == "bgr8":
                    arr = arr[:, :, ::-1]
                arr = np.ascontiguousarray(arr)
                img = QImage(arr.data, msg.width, msg.height,
                             msg.width * 3, QImage.Format_RGB888)
            elif enc == "mono8":
                arr = buf.reshape(msg.height, msg.step)[:, :msg.width]
                arr = np.ascontiguousarray(arr)
                img = QImage(arr.data, msg.width, msg.height,
                             msg.width, QImage.Format_Grayscale8)
            else:
                return None
            return QPixmap.fromImage(img.copy())
        except Exception:  # noqa: BLE001
            return None

    def _log(self, text):
        self._append_log(text)

    def _append_log(self, text):
        self.log_view.appendPlainText(text)

    # ------------------------------------------------------------------ #
    # rqt lifecycle
    # ------------------------------------------------------------------ #
    def save_settings(self, settings):
        settings.set_value("method", self.cmb_method.currentText())
        settings.set_value("samples", self.spn_samples.value())
        settings.set_value("auto_move", self.chk_auto.isChecked())

    def restore_settings(self, settings):
        method = settings.value("method")
        if method in METHODS:
            self.cmb_method.setCurrentText(method)
        samples = settings.value("samples")
        if samples is not None:
            try:
                self.spn_samples.setValue(int(samples))
            except (TypeError, ValueError):
                pass
        auto = settings.value("auto_move")
        if auto is not None:
            self.chk_auto.setChecked(str(auto).lower() in ("true", "1"))

    def shutdown(self):
        self._live_timer.stop()
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs = []

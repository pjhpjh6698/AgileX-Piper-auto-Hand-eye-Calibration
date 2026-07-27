# Piper Auto Hand-Eye Calibration (Eye-in-Hand, ArUco)

Automatic **Eye-in-Hand** hand-eye calibration for the **AgileX Piper** arm with an
**Intel RealSense** camera mounted on the end-effector, observing a fixed **ArUco**
marker. The system automatically moves the robot through a set of poses, collects
`(base_T_gripper, camera_T_target)` pairs, and computes `gripper_T_camera` with
OpenCV `calibrateHandEye()`.

> **Status:** MVP — single fixed ArUco marker target. ChArUco/GridBoard is left as a
> documented extension point (see `aruco_detector_node`), not yet implemented.

---

## 1. What this does

- Camera is **fixed to the robot flange** (`link6`); the ArUco marker is **fixed in the
  workspace**. This is the *Eye-in-Hand* configuration.
- Collects samples only while the robot is **fully stopped**, averages several marker
  frames per pose, rejects bad detections, solves, and **validates** the result via a
  closed-loop target-consistency check.
- Publishes the result as a **static TF** `link6 → camera_color_optical_frame`.
- Runs **without hardware** via a mock robot + synthetic marker (recovers a known
  transform exactly).

All positions are in **meters**; all rotations are handled as **rotation matrices /
quaternions** internally. Euler angles are used **only** for GUI/log display.

---

## 2. Packages

| Package | Type | Contents |
|---|---|---|
| `auto_handeye_interfaces` | ament_cmake | msgs (`MarkerDetection`, `RobotState`, `CalibrationStatus`), srvs (`ResetCalibration`, `SaveCalibration`, `LoadCalibration`, `PublishCalibrationTf`, `AddManualSample`), actions (`RunCalibration`, `MoveToCalibrationPose`) |
| `piper_auto_handeye` | ament_python | all nodes + ROS-free math core + config + launch + tests |
| `piper_auto_handeye_gui` | ament_python | rqt GUI plugin |

### Nodes
- `aruco_detector_node` — detects the target marker, publishes `camera_T_target`.
- `piper_control_node` — **adapter over the existing Piper driver** (`piper_ctrl_single_node`); publishes `RobotState`, serves the safety-validated `MoveToCalibrationPose` action.
- `handeye_calibration_node` — the state machine + `RunCalibration` action + services.
- `calibration_tf_publisher_node` — broadcasts the static result TF.
- `mock_robot_node` — emulates the Piper driver topics for hardware-free testing.
- `synthetic_marker_publisher_node` — camera-free marker test double (derives `camera_T_target` from a known ground truth).

### ROS-free core (unit-testable, pure numpy)
`transform_utils.py`, `calibration_solver.py`, `calibration_validator.py`,
`pose_filter.py`, `safety_validator.py`, `calibration_io.py`.

---

## 3. Design decision: reuse the existing Piper driver

The workspace already ships a working driver `Piper_ros/src/piper` that wraps
`piper_sdk` over CAN and exposes:

| Purpose | Topic | Type |
|---|---|---|
| read pose (`base_T_gripper`) | `/end_pose_stamped` | `geometry_msgs/PoseStamped` |
| read status | `/arm_status` | `piper_msgs/PiperStatusMsg` |
| read joints | `/joint_states_single` | `sensor_msgs/JointState` |
| move (Cartesian moveL) | `/pos_cmd` | `piper_msgs/PosCmd` (x,y,z m; roll,pitch,yaw rad; `mode2=1`) |
| enable | `/enable_flag` | `std_msgs/Bool` |

`piper_control_node` talks to these topics instead of re-wrapping the SDK. The **same**
adapter runs against the real driver or `mock_robot_node`. (Raw SDK calls
`GetArmEndPoseMsgs` / `EndPoseCtrl` / `EnableArm` are documented in `piper_control_node`
as a future `control_backend: sdk` option.)

---

## 4. Transform conventions (READ THIS)

Frame-direction errors are the #1 cause of hand-eye failures. Naming: `A_T_B` = "B
expressed in A" = maps a point in B to A (`p_A = A_T_B @ p_B`).

- `base_T_gripper` — gripper/flange in robot base (from `/end_pose_stamped`)
- `camera_T_target` — marker in camera optical frame (from the detector)
- `gripper_T_camera` — **the calibration result** (camera in gripper)
- `base_T_target` — used only for validation

**OpenCV `calibrateHandEye` (Eye-in-Hand) mapping:**
```
R/t_gripper2base  == base_T_gripper     (our list, per sample)
R/t_target2cam    == camera_T_target    (our list, per sample)
returns cam2gripper == gripper_T_camera (the static TF we publish)
```

**Quaternions are ROS order `(x, y, z, w)`** in all `transform_utils` public functions.
Convert to `(w, x, y, z)` only at library boundaries via
`quaternion_ros_to_wxyz` / `quaternion_wxyz_to_ros`.

**Published TF:** parent `link6` → child `camera_color_optical_frame` = `gripper_T_camera`.

---

## 5. Frames & topics (defaults, all in config)

- base=`base_link`, gripper=`link6`, camera=`camera_color_optical_frame`, target=`calibration_target`
- image=`/camera/camera/color/image_raw`, camera_info=`/camera/camera/color/camera_info`

Change them in `config/*.yaml` to match your system.

---

## 6. Dependencies & build

```bash
# ROS 2 Humble, Ubuntu 22.04
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge \
                 ros-humble-tf2-ros ros-humble-rqt-gui-py
pip install opencv-contrib-python numpy pyyaml   # aruco needs contrib

# Build (from your workspace root)
cd ~/piper_ros2_ws
colcon build --symlink-install \
  --packages-select auto_handeye_interfaces piper_auto_handeye piper_auto_handeye_gui
source install/setup.bash
```
`piper_msgs` (from `Piper_ros`) must be built/sourced for real-robot control; the
nodes degrade gracefully (mock-only) if it is missing.

---

## 7. Quick start — hardware-free (recommended first)

Runs the whole state machine with a mock robot and a **camera-free** synthetic marker
that encodes a known `gripper_T_camera`; the pipeline must recover it.

```bash
ros2 launch piper_auto_handeye mock_calibration.launch.py
# in another terminal, trigger it:
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 10, auto_move: true, calibration_method: PARK,
    settle_time: 0.3, observations_per_pose: 3, save_on_success: true}" --feedback
```
Result is saved to `~/.ros/piper_auto_handeye/handeye_park_*.yaml`. Publish + verify:
```bash
ros2 run piper_auto_handeye calibration_tf_publisher_node \
  --ros-args -p calibration_file:=$(ls -t ~/.ros/piper_auto_handeye/handeye_park_*.yaml | grep -v samples | head -1)
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

---

## 8. Real hardware

### 8a. Start the camera and check it
```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once   # intrinsics present?
```

### 8b. ArUco marker
Print a marker from the configured dictionary (`DICT_4X4_50`) with the configured ID
(`target_marker_id: 1`) and **measure the printed side length**; set `marker_length`
(meters) in `config/aruco.yaml` to the measured value. Fix the marker rigidly in the
workspace, visible from all calibration poses.

### 8c. Start the Piper driver (opens CAN)
```bash
# from Piper_ros; brings up /end_pose_stamped, /pos_cmd, /arm_status, /enable_flag
ros2 run piper piper_ctrl_single_node ...    # (per Piper_ros instructions; sets up can0)
```

### 8d. Detector + calibration stack (DRY-RUN first!)
```bash
ros2 launch piper_auto_handeye detection.launch.py            # aruco detector
ros2 launch piper_auto_handeye calibration.launch.py dry_run:=true
```
Confirm in dry-run that every pose is validated (no motion). Then, **only after**
verifying reach/collisions/marker-visibility in mock + RViz and standing by the
e-stop:
```bash
ros2 launch piper_auto_handeye calibration.launch.py dry_run:=false
```

Or everything at once (does **not** start the low-level driver):
```bash
ros2 launch piper_auto_handeye bringup.launch.py \
  use_realsense:=true dry_run:=true use_gui:=true
```

---

## 9. Automatic calibration
```bash
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 15, auto_move: true, calibration_method: PARK,
    save_on_success: true}" --feedback
```
Flow: system check → for each pose: move → settle → wait for stable marker → average
N frames → store `(base_T_gripper, camera_T_target)` → repeat → solve → validate
(optional outlier removal) → save → (publish TF via the service/node).

## 10. Manual calibration (`auto_move:=false`)
Move the robot yourself (teach/teleop), then capture the current pair:
```bash
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 15, auto_move: false}" --feedback &
# each time the robot is stopped at a new pose:
ros2 service call /add_manual_sample auto_handeye_interfaces/srv/AddManualSample "{}"
```

## 11. GUI
```bash
rqt --standalone piper_auto_handeye_gui
```
Camera view, marker/robot status, start/pause/cancel/reset/add-sample, method + target
count, live progress, result (`gripper_T_camera` t/quat/RPY + RMS), and save/load/publish-TF/log.
The GUI talks **only** via ROS topics/services/actions; closing it does not stop the
calibration manager.

---

## 12. Services
```bash
ros2 service call /reset_calibration  auto_handeye_interfaces/srv/ResetCalibration "{}"
ros2 service call /save_calibration   auto_handeye_interfaces/srv/SaveCalibration "{path: ''}"
ros2 service call /load_calibration   auto_handeye_interfaces/srv/LoadCalibration "{path: ''}"
ros2 service call /publish_calibration_tf auto_handeye_interfaces/srv/PublishCalibrationTf "{path: '', publish: true}"
```

## 13. Output
- Result: `~/.ros/piper_auto_handeye/handeye_<method>_<timestamp>.yaml` (translation,
  quaternion, 4×4 matrix, validation, source).
- Raw samples: `..._samples.yaml` (per-sample poses + reprojection error, stability,
  robot-marker Δt, accept/reject reason).

## 14. Verify in RViz
```bash
rviz2   # Fixed Frame = base_link; add TF display; confirm the camera frame sits at the flange
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

---

## 15. Validation & quality
`base_T_target_i = base_T_gripper_i @ gripper_T_camera @ camera_T_target_i` must be
(nearly) identical for all i. The validator reports translation/rotation RMS & max,
flags outliers, and can iteratively drop the worst samples (bounded). Thresholds
(`maximum_translation_rms_m`, `maximum_rotation_rms_deg`) in `config/handeye.yaml`
decide SUCCESS / WARNING / FAILED. Recommended **15–25 samples** with rotation about
**≥ 2 axes** and translation variety; `minimum_samples` floor is 10.

## 16. Tests
```bash
# ROS-free math (synthetic hand-eye recovery, noise, guards, filter):
cd src/autoCali/piper_auto_handeye && python3 -m pytest test/ -q
# or via colcon:
colcon test --packages-select piper_auto_handeye
```

---

## 17. Troubleshooting
- **`target_id_N_not_seen`** — wrong `target_marker_id`/dictionary, or marker not in view.
- **`reproj_error_*`** — wrong `marker_length`, motion blur, or bad intrinsics.
- **`waiting_for_camera_info`** — camera_info topic name wrong or camera not up.
- **Move rejected "not enabled"** — the adapter auto-enables before a live move
  (`auto_enable: true`); check `/arm_status` and CAN.
- **Move rejected "outside workspace"** — tune `workspace_min/max` in `config/piper.yaml`.
- **Low rotation diversity warning** — add poses that rotate about different axes.
- **Manager can't move** — is `piper_control_node` (or `mock_robot_node`) running and
  is the `move_to_calibration_pose` action server up (`ros2 action list`)?

## 18. ⚠️ Safety
- `dry_run` defaults to **true** everywhere. A live move needs `dry_run:=false` on the
  control node **and** a per-goal `dry_run=false`.
- `config/calibration_poses.yaml` poses are **placeholders**, not verified for your
  robot. Validate reach/collisions in mock + RViz **before** `dry_run:=false`.
- Workspace bounds, max step distance, max speed, NaN rejection, and timeouts are
  enforced in `safety_validator.py`. Start slow, stand by the e-stop.

# AgileX Piper Auto Hand-Eye Calibration

Automatic eye-in-hand calibration for the AgileX Piper arm with a RealSense
camera on the wrist. Press Start and the arm walks itself through a set of
poses, watches a fixed marker, and computes the camera's position on the
gripper. No jogging the robot by hand, no writing down poses.

한국어 문서는 [README.ko.md](README.ko.md)를 보세요.

## Features

- One launch file brings up the arm, camera, detector, solver, GUI and RViz.
- The arm plans and drives its own pose sweep; you only press Start.
- ArUco marker or ChArUco board, switchable from the GUI while running.
- Live result: the estimate is re-solved and shown after every accepted sample.
- Recovers automatically when the gripper hides the marker.
- Five solvers (Tsai-Lenz, Park, Horaud, Andreff, Daniilidis) via OpenCV.
- Korean and English GUI.
- Everything the robot needs is vendored, so a fresh machine is `colcon build`.

## Use case: eye-in-hand

The camera is bolted to the flange and moves with the arm; the marker sits
still in the workspace. What you want is `gripper_T_camera`, the fixed transform
from the flange to the camera.

For each pose the system records where the arm says its flange is
(`base_T_gripper`) and where the camera says the marker is
(`camera_T_target`). The marker never moves, so that constraint is enough to
solve for the camera's mounting — the marker's own position drops out of the
maths and never has to be measured.

Naming: `A_T_B` means "B expressed in A", so `p_A = A_T_B @ p_B`.

## Getting started

Requires ROS 2 Humble on Ubuntu 22.04.

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge \
                 ros-humble-tf2-ros ros-humble-rqt-gui-py
pip install opencv-contrib-python numpy pyyaml   # aruco needs contrib

mkdir -p ~/piper_ros2_ws && cd ~/piper_ros2_ws
git clone https://github.com/pjhpjh6698/AgileX-Piper-auto-Hand-eye-Calibration.git autoCali
cd autoCali
colcon build
source install/setup.bash
```

Build without `--symlink-install`. Mixing the two install layouts leaves the
console-script wrappers unable to find their package metadata.

## Usage

### 1. Print a target and measure it

Print an ArUco marker from `DICT_4X4_50` with ID 1, or a ChArUco board. Then
measure the print with a ruler and enter what you measured, not what you asked
the generator for.

This matters more than it looks. The target size is the only scale the pose
estimate has, so a size that is 10% off puts the camera 10% of the working
distance away from where it really is — and the rotation still comes out
perfect, so nothing downstream looks wrong. Printers scale silently: this rig
lost a day to a board printed at 133.3%, which turned 15 mm squares into 20 mm.

For a ChArUco board, measure the full span and divide, rather than measuring one
square:

```
width across all columns / columns  ==  height across all rows / rows
```

The two must agree. Enter the result in the GUI, or in `config/aruco.yaml`.

Fix the target rigidly in the workspace where the camera can see it from every
pose, roughly 20-30 cm in front of the arm.

### 2. Bring up CAN and check the arm answers

```bash
bash piper_auto_handeye/scripts/can_setup.sh          # every CAN adapter found
ros2 run piper_auto_handeye agx_arm_check             # read-only probe
```

`agx_arm_check` never enables the arm and never commands motion. It exits 0 only
when the link is up, frames are arriving, and the arm decodes a live pose with
no faults. Do not go past a `NOT READY`.

Find your interface with `ip -br link show type can` and pass it as
`--can-port` / `can_port:=` if it is not `can0`.

### 3. Calibrate

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py
```

This launch moves the robot. Keep a hand on the e-stop for the first run.

In the GUI: check the CAN panel is green, pick your target and enter its
measured size, press Start RViz, then press Start. The arm sweeps its poses and
stops on its own once it has enough samples. The result appears in the
`gripper_T_camera` panel and updates after every sample.

To watch the whole sweep without commanding any motion:

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py dry_run:=true
```

### 4. Check the result

Press Publish TF, and the camera frame appears on the wrist in the embedded
RViz. Move the arm and watch it: a good calibration keeps the camera frame glued
where the camera physically sits, and a bad one floats off the gripper or points
the wrong way.

Or look at it on its own:

```bash
ros2 launch piper_auto_handeye view_calibration.launch.py
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

Results are written to `~/.ros/piper_auto_handeye/handeye_<method>_<time>.yaml`,
alongside a `_samples.yaml` with every raw sample and why it was accepted.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `dry_run` | `false` | `true` validates the poses without moving the arm |
| `gui_lang` | `ko` | GUI language; `en` for English |
| `can_port` | `can0` | socketcan interface the arm is on |
| `wrist_camera_serial` | (empty) | empty uses the only camera attached |
| `calibration_method` | `TSAI` | `TSAI`, `PARK`, `HORAUD`, `ANDREFF`, `DANIILIDIS` |
| `use_gui` | `true` | start the rqt GUI |
| `use_realsense` | `true` | start the camera |
| `use_piper_driver` | `true` | start the vendored AgileX driver |

`ros2 launch piper_auto_handeye real_calibration.launch.py --show-args` lists
them all.

With more than one RealSense attached, pin the wrist one by serial. Keep the
leading underscore — realsense2_camera reads a bare numeric serial as an integer
and fails to match:

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py \
  wrist_camera_serial:=_123456789012        # rs-enumerate-devices -s
```

The GUI language can also be set outside the launch file:

```bash
HANDEYE_GUI_LANG=en rqt --standalone piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin
```

## Configuration

Frames and thresholds live in `piper_auto_handeye/config/`. The defaults are
`base_link`, `link6`, `camera_color_optical_frame` and `calibration_target`.

| File | What it holds |
|---|---|
| `aruco.yaml` | target type, marker ID and size, board geometry, detector tuning |
| `handeye.yaml` | frames, solver, sample counts, pose sweep, recovery, thresholds |
| `piper.yaml` | workspace bounds, step and speed limits |

Worth knowing:

- `target_samples` (30) is how many pose/marker pairs go into the solve.
  `observations_per_pose` (10) is how many camera frames are averaged into one
  such pair. Averaging N frames cuts random detection noise by about `sqrt(N)`;
  beyond 10 the remaining error is systematic and averaging does not touch it.
- The published TF is retargeted onto `camera_link` (`retarget_frame`), because
  the RealSense driver already owns `camera_color_optical_frame` and a frame can
  only have one parent. The `base_link` to optical-frame chain is unchanged.

## FAQ

Why does the solve fail with "not enough rotation diversity"?
: The poses are too similar. Hand-eye needs rotation about at least two axes.
  The built-in sweep already does this; if you supplied your own poses, add some
  that rotate about a different axis.

Why did ChArUco and ArUco disagree by a few centimetres?
: A wrong target size, almost always. See step 1 — a scale error shows up as
  pure translation with the rotation still exact, which is what makes it so hard
  to spot.

The marker is clearly visible but keeps being rejected.
: Check the reprojection error in the GUI status panel. A high value with a
  visible marker points at a wrong `marker_length` or bad camera intrinsics.
  If the value is fine but stability is low, the arm may still be settling —
  raise `settle_time`.

The arm refuses to move, "outside workspace".
: The goal is outside `workspace_min`/`workspace_max` in `config/piper.yaml`.

The gripper keeps covering the marker.
: That is expected in eye-in-hand and is handled: the wrist is nudged and the
  pose re-shot (the `RECOVERING` state). Tune with `marker_recovery_*` in
  `config/handeye.yaml`.

## Safety

This launch commands real motion. What protects a run are the limits in
`config/piper.yaml` — Cartesian workspace bounds, a cap on how far one move may
travel, a speed cap — plus two stops: `/stop_motion` holds the current pose and
is what the GUI STOP button calls, while `/hard_stop` cuts drive power and lets
the arm fall, so use it only when that is the lesser harm.

Run `dry_run:=true` once on a new setup before letting it move.

## Tests

```bash
colcon test --packages-select piper_auto_handeye piper_auto_handeye_gui
```

The maths core (`transform_utils`, `calibration_solver`, `calibration_validator`,
`pose_filter`, `safety_validator`) is ROS-free and tested against synthetic
hand-eye problems with a known answer.

## Packages

| Package | Contents |
|---|---|
| `piper_auto_handeye` | nodes, maths core, config, launch, tests |
| `piper_auto_handeye_gui` | rqt GUI plugin |
| `auto_handeye_interfaces` | messages, services, actions |
| `piper_auto_handeye_sim` | Gazebo verification rig |
| `agx_arm_description` | vendored URDF and meshes for the RViz robot model |
| `agx_arm_ctrl`, `agx_arm_msgs`, `pyagxarm_vendor` | vendored AgileX driver and SDK |

`piper_control_node` never opens the CAN bus itself; it talks to the AgileX
driver, which owns the bus. That keeps the calibration policy — safety limits,
goal validation, arrival detection — in one node that does not care how the arm
is reached.

## Vendored third-party code

Copied into this workspace so the whole path builds from one `colcon build`,
with no `pip install` and no `PYTHONPATH` edits. Attribution is a license
condition:

| Directory | Upstream | Version | License |
|---|---|---|---|
| `agx_arm_ctrl`, `agx_arm_msgs`, `agx_arm_description` | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) (branch `ros2`) | commit `91e6b2e`, copied 2026-07-27 | Apache-2.0 |
| `pyagxarm_vendor/pyAgxArm` | [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) | v1.0.0, commit `cc498c0`, copied 2026-07-27 | MIT |

No upstream `.py` file is modified. `agx_arm_description` carries two mesh
fixes: the Collada visuals are swapped for the STLs shipped beside them (the
`.dae` files fail to load in RViz on Humble), and a grey material is declared
on each visual, because STL carries no colour and RViz renders an untinted
model entirely in red.

## References

R. Tsai and R. Lenz, "A new technique for fully autonomous and efficient 3D
robotics hand/eye calibration", IEEE Transactions on Robotics and Automation,
1989.

The GUI and workflow follow the shape of
[easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2).

# Vendored `agx_arm_ctrl` / `agx_arm_msgs` (AgileX ROS 2 arm driver)

## What this is

The current AgileX ROS 2 driver, copied into this workspace so the whole
real-robot calibration path builds from one `colcon build`.

| | |
|---|---|
| Upstream | https://github.com/agilexrobotics/agx_arm_ros |
| Branch | `ros2` (the repo also has `dev`) |
| Commit | `91e6b2e5eb2d9880e85230d0add9945e27387d87` (2026-07-01) |
| Copied | 2026-07-27 |
| License | Apache-2.0 (upstream) |
| SDK | `pyAgxArm` v1.0.0, vendored in `pyagxarm_vendor` |

Two of the repo's four packages are vendored: `agx_arm_msgs` and
`agx_arm_ctrl`. `agx_arm_description` and `agx_arm_moveit` are not — the
calibration stack has its own URDF and does not plan with MoveIt. Skipping
`agx_arm_moveit` is also what keeps `ros2_control` out of the dependency set:
`agx_arm_ctrl` itself needs only rclpy, the standard message packages and scipy.

## Why this replaced `piper_ros`

This workspace previously vendored `piper_ros` (branch `humble`) and its
`piper_ctrl_single_node`. That driver's `/pos_cmd` interface has real
limitations, all of which `agx_arm_ctrl` fixes:

| | `piper_ctrl_single_node` | `agx_arm_ctrl` |
|---|---|---|
| Motion mode | `mode1`/`mode2` accepted then **ignored**; always MOVE P | separate `control/move_p`, `move_l`, `move_j`, `move_js`, `move_c` topics |
| Speed | hardcoded 50%, `PosCmd` has no speed field | `speed_percent` parameter → `set_speed_percent()` |
| Cartesian goal | custom `PosCmd`, quantised to 1 mm | standard `geometry_msgs/PoseStamped` |
| Pose feedback | `PoseStamped` on `end_pose_stamped` | `PoseStamped` on `feedback/tcp_pose` |
| Enable | `Bool` topic, fire-and-forget | `enable_agx_arm` service (`SetBool`), acknowledged |
| Stop | none | `emergency_stop` service |
| TCP offset | none | `set_tcp_offset()` |

## Interface used by this workspace

`piper_control_node` with `control_backend: agx` binds to:

    reads   feedback/tcp_pose      geometry_msgs/PoseStamped   base_T_gripper
            feedback/joint_states  sensor_msgs/JointState
            feedback/arm_status    agx_arm_msgs/AgxArmStatus
    writes  control/move_p         geometry_msgs/PoseStamped   Cartesian goal
            control/move_l         geometry_msgs/PoseStamped   (optional, straight line)
    calls   enable_agx_arm         std_srvs/SetBool
            emergency_stop         std_srvs/Empty

## Local modifications

`agx_arm_msgs` is byte-identical to upstream. Two changes in `agx_arm_ctrl`:

**1. `package.xml` — declare the dependencies upstream omits.**
Added `pyagxarm_vendor` as an `exec_depend`. Upstream leaves it out because it
assumes `pip install pyAgxArm`; without the declaration colcon may order the
build wrongly and `rosdep install` misses the SDK entirely.

**2. `agx_arm_ctrl_single_node.py` — make `speed_percent` settable at runtime.**
Added `add_on_set_parameters_callback(self._on_set_parameters)` and the
`_on_set_parameters` method. Upstream reads `speed_percent` exactly once, in
`_init_agx_arm`, and registers no parameter callback — so `ros2 param set` (and
`piper_control_node`'s per-goal speed push) updated the stored parameter while
the arm kept moving at whatever speed it was launched with. Silently ignoring a
speed reduction is a safety problem, not a cosmetic one, so the callback now
forwards the value to `agx_arm.set_speed_percent()` and rejects out-of-range
values instead of accepting them.

Both changes are marked `LOCAL PATCH` in the source. Re-apply them when
re-vendoring.

## Behaviour worth knowing

- **`speed_percent` is a node parameter, not per-message.** `PoseStamped`
  carries no speed. To honour a per-goal speed, `piper_control_node` sets the
  parameter on this node before publishing the goal; see `_agx_set_speed`.
- **Default `speed_percent` is 100.** The launch wrapper in
  `piper_auto_handeye` overrides it down to the calibration speed cap. Do not
  launch `agx_arm_ctrl` directly for a calibration run without setting it.
- `auto_enable` defaults to true, which powers the joints at startup.

## Re-vendoring

```bash
cd ~/piper_ros2_ws/autoCali
git clone --depth 1 -b ros2 https://github.com/agilexrobotics/agx_arm_ros /tmp/agx_arm_ros
rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' /tmp/agx_arm_ros/src/agx_arm_msgs/ agx_arm_msgs/
rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' /tmp/agx_arm_ros/src/agx_arm_ctrl/ agx_arm_ctrl/
# re-apply the pyagxarm_vendor exec_depend described above
colcon build
```

Then re-check `piper_control_node`'s `agx` backend against the node's
`_setup_publishers` / `_setup_subscribers` / `_setup_services`.

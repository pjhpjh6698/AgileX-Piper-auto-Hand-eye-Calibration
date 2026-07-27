# Vendored `pyAgxArm`

## What this is

`pyAgxArm/` is the AgileX unified Python arm SDK, **v1.0.0**, copied verbatim
into this workspace and wrapped in an `ament_python` package so that

```bash
source install/setup.bash
python3 -c "import pyAgxArm"
```

works on any machine that can `colcon build` this repo — no `pip install
pyAgxArm`, no `PYTHONPATH` edits.

`agx_arm_ctrl` (the AgileX ROS 2 driver, also vendored here) imports it, and so
does `agx_arm_check`, the calibration stack's pre-flight tool. Vendoring both
keeps the whole real-robot path reproducible from one `colcon build`.

## Provenance

| | |
|---|---|
| Upstream | https://github.com/agilexrobotics/pyAgxArm |
| Branch | `master` |
| Version | 1.0.0 (`pyAgxArm/version.py`) |
| Commit | `cc498c0` (2026-07-08) |
| Copied | 2026-07-27 |
| License | MIT (upstream) |

This supersedes the older `piper_sdk`, which this workspace used to vendor.
`piper_sdk` exposed fixed-point integers (0.001 mm / 0.001 deg) and a
Piper-only `C_PiperInterface_V2`; pyAgxArm returns SI floats (m, rad) and
covers the whole Agilex arm family behind `AgxArmFactory`.

## Local modifications

**None to any `.py` file.** The only deviation from upstream is one deletion:

- `demos/` — standalone example scripts, not importable library code.

`py.typed` and the `*.pyi` stubs are kept and installed (see `package_data` in
`setup.py`); dropping them would silently strip the SDK's type information.

## Runtime dependencies

`python-can>=3.3.4` and `typing-extensions>=3.7.4.3`, declared in `package.xml`
as `python3-can` / `python3-typing-extensions` so `rosdep install` covers them.

If `rosdep` is not in play, the system Python needs them directly:

```bash
sudo apt install python3-can            # or: pip3 install --user python-can typing_extensions
```

## API notes that matter to this workspace

Construction is a two-step factory, not a constructor:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
cfg = create_agx_arm_config(robot=ArmModel.PIPER, comm="can", channel="can_follower")
arm = AgxArmFactory.create_arm(cfg)     # NOT AgxArmFactory(cfg)
arm.connect()
```

`get_flange_pose().msg` is `[x, y, z, roll, pitch, yaw]` in **metres and
radians**, with `R = Rz @ Ry @ Rx` — the same convention as
`transform_utils.euler_to_matrix`, so no conversion is needed.

## Re-vendoring a newer release

```bash
cd ~/piper_ros2_ws/autoCali
git clone --depth 1 https://github.com/agilexrobotics/pyAgxArm /tmp/pyagx
rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' --exclude 'demos' \
      /tmp/pyagx/pyAgxArm/  pyagxarm_vendor/pyAgxArm/
# bump <version> in package.xml and version= in setup.py to match version.py
colcon build --packages-select pyagxarm_vendor
```

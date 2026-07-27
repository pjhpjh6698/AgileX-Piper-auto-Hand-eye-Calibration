#!/usr/bin/env python3
"""Pre-flight check: is the arm actually reachable over CAN?

Run this before every real-robot calibration session, and first whenever
something looks wrong. It is read-only -- it never enables the arm and never
commands motion.

    ros2 run piper_auto_handeye agx_arm_check
    ros2 run piper_auto_handeye agx_arm_check --can-port can_master
    ros2 run piper_auto_handeye agx_arm_check --watch

Exit status is 0 only when the arm answers and reports no fault, so it can gate
a launch script.

It separates three things that are easy to confuse:

  1. link  -- does the socketcan interface exist and is it UP at 1 Mbit/s?
  2. bus   -- are CAN frames arriving at all?
  3. arm   -- does pyAgxArm decode them into a live pose, and is the arm
              fault-free and enabled?

A common failure is 1 and 2 passing while 3 fails: the dongle is fine and
something is chattering on the bus, but the arm itself is powered off.

This talks to the SDK directly rather than to agx_arm_ctrl, deliberately -- it
has to be able to tell you the driver is fine and the *arm* is not.
"""

import argparse
import math
import subprocess
import sys
import time

GREEN, YELLOW, RED, DIM, RESET = "\033[32m", "\033[33m", "\033[31m", "\033[2m", "\033[0m"

# pyAgxArm arm_status codes that are faults (0x00 NORMAL, 0x0B..0x0D teaching).
FAULT_TEXT = {
    0x01: "emergency stop", 0x02: "no IK solution", 0x03: "singularity",
    0x04: "target angle exceeds joint limit", 0x05: "joint communication error",
    0x06: "joint brake not released", 0x07: "collision detected",
    0x08: "overspeed during drag-teaching", 0x09: "joint status error",
    0x0A: "unspecified arm error", 0x0E: "controller over-temperature",
    0x0F: "bleeder resistor over-temperature",
}


def _ok(msg):
    print(f"  {GREEN}[ OK ]{RESET} {msg}")


def _warn(msg):
    print(f"  {YELLOW}[WARN]{RESET} {msg}")


def _fail(msg):
    print(f"  {RED}[FAIL]{RESET} {msg}")


def _run(cmd, timeout=3):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return ""


def check_link(port):
    """socketcan interface present, UP, and at the arm's 1 Mbit/s bitrate."""
    print(f"\n1. socketcan link '{port}'")
    names = [ln.split()[0]
             for ln in _run(["ip", "-br", "link", "show", "type", "can"]).splitlines()
             if ln.strip()]
    if not names:
        _fail("no CAN interface at all -- is the USB-CAN dongle plugged in?")
        print(f"       {DIM}lsusb | grep -i 'CAN adapter'   # expect 1d50:606f{RESET}")
        return False
    print(f"       {DIM}available: {', '.join(names)}{RESET}")
    if port not in names:
        _fail(f"'{port}' not found")
        return False

    brief = _run(["ip", "-br", "link", "show", port]).split()
    state = brief[1] if len(brief) > 1 else "UNKNOWN"
    words = _run(["ip", "-d", "link", "show", port]).split()
    bitrate = words[words.index("bitrate") + 1] if "bitrate" in words else None

    if state != "UP":
        _fail(f"'{port}' is {state}")
        print(f"       {DIM}bash piper_auto_handeye/scripts/can_setup.sh {port}{RESET}")
        return False
    if bitrate != "1000000":
        _warn(f"'{port}' is UP but bitrate={bitrate}, the arm expects 1000000")
        return False
    _ok(f"'{port}' UP at 1 Mbit/s")
    return True


def check_bus(port, seconds=1.0):
    """Raw frames arriving -- proves something on the bus is transmitting."""
    print(f"\n2. CAN traffic on '{port}'")
    out = _run(["timeout", str(seconds), "candump", "-n", "50", port],
               timeout=seconds + 3)
    n = len([ln for ln in out.splitlines() if ln.strip()])
    if n == 0:
        _fail(f"no frames in {seconds:.0f}s -- link is up but nothing is transmitting")
        print(f"       {DIM}the arm is probably powered off, or the CAN cable is loose{RESET}")
        return False
    _ok(f"{n} frames in {seconds:.0f}s")
    return True


def _connect(port):
    """Open the arm. Raises with an actionable message on failure."""
    try:
        from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
    except ImportError as exc:
        raise RuntimeError(
            "cannot import pyAgxArm. Build the vendored copy and re-source:\n"
            "  colcon build --packages-select pyagxarm_vendor\n"
            "  source install/setup.bash\n"
            "It also needs python-can (sudo apt install python3-can).\n"
            f"underlying error: {exc}") from exc

    cfg = create_agx_arm_config(robot=ArmModel.PIPER, comm="can", channel=port)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    return arm


def _fault_text(status):
    """Fault string from an arm_status message, or '' when healthy."""
    if status is None:
        return "no status frame"
    msg = status.msg
    parts = []
    code = int(msg.arm_status)
    if code in FAULT_TEXT:
        parts.append(FAULT_TEXT[code])
    err = msg.err_status
    for i in range(1, 7):
        if getattr(err, f"joint_{i}_angle_limit", False):
            parts.append(f"joint{i} angle limit")
        if getattr(err, f"communication_status_joint_{i}", False):
            parts.append(f"joint{i} comm error")
    return "; ".join(parts)


def check_arm(port, timeout=5.0):
    """SDK level: decode the frames into a pose and read the fault state."""
    print(f"\n3. arm on '{port}'")
    try:
        arm = _connect(port)
    except Exception as exc:
        _fail(str(exc).splitlines()[0])
        for line in str(exc).splitlines()[1:]:
            print(f"       {DIM}{line}{RESET}")
        return False

    try:
        # Feedback structs exist immediately but stay at hz=0 until frames land.
        deadline = time.time() + timeout
        pose = None
        while time.time() < deadline:
            pose = arm.get_flange_pose()
            if pose is not None and pose.hz > 0:
                break
            time.sleep(0.05)
        if pose is None or pose.hz <= 0:
            _fail(f"connected but no pose decoded after {timeout:.0f}s "
                  f"(link up, arm powered off?)")
            return False

        _ok(f"feedback at {pose.hz:.0f} Hz")
        x, y, z, roll, pitch, yaw = pose.msg
        print(f"       flange xyz : {x * 1000:8.1f} {y * 1000:8.1f} {z * 1000:8.1f}  mm")
        print(f"       flange rpy : {math.degrees(roll):8.2f} {math.degrees(pitch):8.2f} "
              f"{math.degrees(yaw):8.2f}  deg")
        joints = arm.get_joint_angles()
        if joints is not None:
            print("       joints     : " +
                  " ".join(f"{math.degrees(j):7.2f}" for j in joints.msg) + "  deg")

        healthy = True
        fault = _fault_text(arm.get_arm_status())
        if fault:
            _fail(f"arm reports: {fault}")
            healthy = False
        else:
            _ok("no faults reported")

        # 255 = "all joints"; returns a plain bool, true only if every joint is on.
        if arm.get_joint_enable_status(255):
            _ok("all joint drivers ENABLED (arm is holding position)")
        else:
            _warn("joint drivers are DISABLED -- the arm is limp and cannot move")
            print(f"       {DIM}the driver enables it on request before a live move{RESET}")
        return healthy
    finally:
        arm.disconnect()


def watch(port, hz=5.0):
    """Continuously print the live pose. Ctrl-C to stop. Read-only."""
    arm = _connect(port)
    print(f"watching '{port}' -- Ctrl-C to stop\n")
    try:
        while True:
            pose = arm.get_flange_pose()
            joints = arm.get_joint_angles()
            if pose is not None and pose.hz > 0:
                x, y, z = pose.msg[0], pose.msg[1], pose.msg[2]
                js = ("" if joints is None else
                      " ".join(f"{math.degrees(j):7.2f}" for j in joints.msg))
                fault = _fault_text(arm.get_arm_status())
                tail = f"  {RED}{fault}{RESET}" if fault else ""
                print(f"\rxyz[mm] {x * 1000:7.1f} {y * 1000:7.1f} {z * 1000:7.1f} "
                      f"| j[deg] {js}{tail}   ", end="", flush=True)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        print()
    finally:
        arm.disconnect()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Read-only CAN/SDK connectivity check for the AgileX arm.")
    ap.add_argument("--can-port", default="can_follower",
                    help="socketcan interface name (default: can_follower)")
    ap.add_argument("--watch", action="store_true",
                    help="stream the live pose instead of running the checks")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="seconds to wait for the first feedback frame")
    args = ap.parse_args(argv)

    if args.watch:
        watch(args.can_port)
        return 0

    print(f"AgileX arm check -- port '{args.can_port}'")
    results = [check_link(args.can_port), check_bus(args.can_port)]
    # Only bother with the SDK if the link is usable; otherwise its error is
    # just noise on top of the real problem.
    if all(results):
        results.append(check_arm(args.can_port, args.timeout))

    print()
    if all(results):
        print(f"{GREEN}READY{RESET} -- the arm is reachable on '{args.can_port}'.")
        return 0
    print(f"{RED}NOT READY{RESET} -- fix the failures above before running a calibration.")
    return 1


if __name__ == "__main__":
    sys.exit(main())

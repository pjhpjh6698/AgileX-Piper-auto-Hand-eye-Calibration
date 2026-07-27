#!/usr/bin/env python3
"""Decoding for ``agx_arm_msgs/AgxArmStatus``.

The message carries raw protocol bytes. This turns the ones that matter into a
single human-readable fault string, empty when the arm is healthy, which is
what ``RobotState.error_message`` wants and what the GUI displays.

Kept separate from the node so the mapping can be unit-tested without ROS.
"""

# AgxArmStatus.arm_status. 0x00 NORMAL and the teaching states (0x0B..0x0D) are
# operating modes, not faults, so they produce no message.
ARM_STATUS_TEXT = {
    0x01: "emergency stop",
    0x02: "no IK solution for the commanded pose",
    0x03: "singularity",
    0x04: "target angle exceeds joint limit",
    0x05: "joint communication error",
    0x06: "joint brake not released",
    0x07: "collision detected",
    0x08: "overspeed during drag-teaching",
    0x09: "joint status error",
    0x0A: "unspecified arm error",
    0x0E: "main controller NTC over-temperature",
    0x0F: "bleeder resistor NTC over-temperature",
}

# AgxArmStatus.ctrl_mode values under which a Cartesian goal will not be obeyed.
CTRL_MODE_CAN = 0x01
CTRL_MODE_TEACHING = 0x02


def status_text(msg) -> str:
    """Fault description for an AgxArmStatus, or '' if the arm is healthy."""
    parts = []

    code = int(msg.arm_status)
    if code in ARM_STATUS_TEXT:
        parts.append(ARM_STATUS_TEXT[code])

    # Per-joint detail. The arrays are variable-length; upstream sizes them to
    # the joint count, so never index them blindly.
    for i, flagged in enumerate(getattr(msg, "joint_angle_limit", []) or []):
        if flagged:
            parts.append(f"joint{i + 1} angle limit")
    for i, flagged in enumerate(getattr(msg, "communication_status_joint", []) or []):
        if flagged:
            parts.append(f"joint{i + 1} comm error")

    # err_status is a bitfield summarising the same conditions. Only surface it
    # when the decoded flags explained nothing, so we do not double-report.
    err = int(getattr(msg, "err_status", 0) or 0)
    if err and not parts:
        parts.append(f"err_status=0x{err:X}")

    return "; ".join(parts)


def is_teaching(msg) -> bool:
    """True when the arm is in drag-teach mode and will ignore motion goals."""
    return int(msg.ctrl_mode) == CTRL_MODE_TEACHING

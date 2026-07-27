"""Calibration core: Piper control adapter + calibration manager + TF publisher.

Assumes the ArUco detector and the REAL Piper driver (piper_ctrl_single_node)
are already running (or use bringup.launch.py). dry_run defaults to true.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("piper_auto_handeye")
    handeye_cfg = os.path.join(pkg, "config", "handeye.yaml")
    piper_cfg = os.path.join(pkg, "config", "piper.yaml")
    poses_file = os.path.join(pkg, "config", "calibration_poses.yaml")

    dry_run = LaunchConfiguration("dry_run")

    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node",
        parameters=[piper_cfg, {"dry_run": dry_run}], output="screen")

    manager = Node(
        package="piper_auto_handeye", executable="handeye_calibration_node",
        name="handeye_calibration_node",
        parameters=[handeye_cfg, {"calibration_poses_file": poses_file}],
        output="screen")

    tf_pub = Node(
        package="piper_auto_handeye", executable="calibration_tf_publisher_node",
        name="calibration_tf_publisher_node",
        parameters=[handeye_cfg, {"auto_publish": False}], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="SAFETY: true = never command the robot"),
        control, manager, tf_pub,
    ])

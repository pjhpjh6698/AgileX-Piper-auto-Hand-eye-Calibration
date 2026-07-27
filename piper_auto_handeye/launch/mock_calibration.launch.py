"""Hardware-free calibration stack: mock robot + calibration manager + TF publisher.

The ArUco detector is included but real detections require a camera + marker.
For a fully synthetic pipeline (no camera), see the synthetic tests. This launch
is primarily for exercising the STATE MACHINE, action, and services with a mock
robot; run detection.launch.py with a real camera to feed real marker poses.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("piper_auto_handeye")
    handeye_cfg = os.path.join(pkg, "config", "handeye.yaml")
    piper_cfg = os.path.join(pkg, "config", "piper.yaml")
    aruco_cfg = os.path.join(pkg, "config", "aruco.yaml")
    poses_file = os.path.join(pkg, "config", "calibration_poses.yaml")

    use_detector = LaunchConfiguration("use_detector")
    use_synthetic_marker = LaunchConfiguration("use_synthetic_marker")
    auto_move = LaunchConfiguration("auto_move")

    mock = Node(
        package="piper_auto_handeye", executable="mock_robot_node",
        name="mock_robot_node", parameters=[piper_cfg], output="screen")

    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node",
        # dry_run False is SAFE here: the "robot" is the mock, no hardware moves.
        # Force the topic backend -- mock_robot_node emulates the driver topics,
        # and the sdk backend would try to open a real CAN bus instead.
        parameters=[piper_cfg, {"dry_run": False, "control_backend": "topic"}],
        output="screen")

    manager = Node(
        package="piper_auto_handeye", executable="handeye_calibration_node",
        name="handeye_calibration_node",
        parameters=[handeye_cfg, {"calibration_poses_file": poses_file}],
        output="screen")

    tf_pub = Node(
        package="piper_auto_handeye", executable="calibration_tf_publisher_node",
        name="calibration_tf_publisher_node",
        parameters=[handeye_cfg, {"auto_publish": False}], output="screen")

    detector = Node(
        condition=IfCondition(use_detector),
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", parameters=[aruco_cfg], output="screen")

    # Fully camera-free test double: derives marker poses from the mock robot
    # pose + a known ground-truth gripper_T_camera, so the whole pipeline runs
    # and RECOVERS the known transform with no hardware at all.
    synthetic = Node(
        condition=IfCondition(use_synthetic_marker),
        package="piper_auto_handeye", executable="synthetic_marker_publisher_node",
        name="synthetic_marker_publisher_node", parameters=[aruco_cfg], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("use_detector", default_value="false",
                              description="Launch ArUco detector (needs camera+marker)"),
        DeclareLaunchArgument("use_synthetic_marker", default_value="true",
                              description="Use camera-free synthetic marker (recommended for mock)"),
        DeclareLaunchArgument("auto_move", default_value="true"),
        mock, control, manager, tf_pub, detector, synthetic,
    ])

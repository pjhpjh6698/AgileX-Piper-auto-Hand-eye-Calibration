"""Full bringup: (RealSense) + detector + Piper control + manager + TF + (GUI).

Does NOT launch the low-level Piper driver (piper_ctrl_single_node) -- start that
separately (it opens the CAN bus). Use use_mock_robot:=true to run without the
real driver/hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("piper_auto_handeye")
    handeye_cfg = os.path.join(pkg, "config", "handeye.yaml")
    piper_cfg = os.path.join(pkg, "config", "piper.yaml")
    aruco_cfg = os.path.join(pkg, "config", "aruco.yaml")
    poses_file = os.path.join(pkg, "config", "calibration_poses.yaml")

    use_realsense = LaunchConfiguration("use_realsense")
    use_mock_robot = LaunchConfiguration("use_mock_robot")
    use_gui = LaunchConfiguration("use_gui")
    dry_run = LaunchConfiguration("dry_run")
    namespace = LaunchConfiguration("namespace")

    realsense = GroupAction(
        condition=IfCondition(use_realsense),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
            launch_arguments={"enable_color": "true", "pointcloud.enable": "false"}.items())])

    detector = Node(
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", namespace=namespace,
        parameters=[aruco_cfg], output="screen")

    mock = Node(
        condition=IfCondition(use_mock_robot),
        package="piper_auto_handeye", executable="mock_robot_node",
        name="mock_robot_node", parameters=[piper_cfg], output="screen")

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

    gui = ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=["rqt", "--force-discover", "--standalone",
             "piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin"],
        output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="false"),
        DeclareLaunchArgument("use_mock_robot", default_value="false"),
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="SAFETY: true = never command the robot"),
        DeclareLaunchArgument("namespace", default_value=""),
        realsense, detector, mock, control, manager, tf_pub, gui,
    ])

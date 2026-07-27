"""ONE-COMMAND real-robot Eye-in-Hand calibration (Piper + RealSense + GUI).

Brings up everything needed:
  - Piper driver (piper_ctrl_single_node, opens CAN)   [use_piper_driver]
  - RealSense camera                                   [use_realsense]
  - ArUco detector (publishes the GUI camera view)
  - Piper control adapter (safety-validated motion)
  - Hand-eye calibration manager (Tsai-Lenz by default)
  - Static TF publisher
  - RQt GUI                                            [use_gui]

##########################  SAFETY  ##########################
dry_run defaults to TRUE: nothing moves until you pass dry_run:=false.
Before ever running with dry_run:=false you MUST verify that every pose in
config/calibration_poses.yaml is reachable and collision-free for YOUR setup.
Keep a hand on the e-stop. The GUI shows a red "LIVE" banner when motion is armed.
##############################################################

Typical use:
  # 1) dry run first -- verifies poses, marker, and the whole pipeline, no motion
  ros2 launch piper_auto_handeye real_calibration.launch.py
  # 2) only after the dry run is clean:
  ros2 launch piper_auto_handeye real_calibration.launch.py dry_run:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, ExecuteProcess, TimerAction, LogInfo)
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
    use_piper_driver = LaunchConfiguration("use_piper_driver")
    use_gui = LaunchConfiguration("use_gui")
    dry_run = LaunchConfiguration("dry_run")
    can_port = LaunchConfiguration("can_port")
    method = LaunchConfiguration("calibration_method")

    # --- Piper low-level driver (opens the CAN bus) ---
    piper_driver = GroupAction(
        condition=IfCondition(use_piper_driver),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("piper"), "launch", "start_single_piper.launch.py"])),
            launch_arguments={"can_port": can_port, "auto_enable": "true"}.items())])

    # --- RealSense ---
    realsense = GroupAction(
        condition=IfCondition(use_realsense),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
            launch_arguments={"enable_color": "true",
                              "pointcloud.enable": "false"}.items())])

    detector = Node(
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", parameters=[aruco_cfg], output="screen")

    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node",
        parameters=[piper_cfg, {"dry_run": dry_run}], output="screen")

    manager = Node(
        package="piper_auto_handeye", executable="handeye_calibration_node",
        name="handeye_calibration_node",
        parameters=[handeye_cfg,
                    {"calibration_poses_file": poses_file,
                     "calibration_method": method}],
        output="screen")

    tf_pub = Node(
        package="piper_auto_handeye", executable="calibration_tf_publisher_node",
        name="calibration_tf_publisher_node",
        parameters=[handeye_cfg, {"auto_publish": False}], output="screen")

    # GUI last, after the nodes it talks to are up.
    # NOTE: rqt identifies the plugin by its FULL class path, and caches plugin
    # discovery -- so a freshly built plugin needs --force-discover.
    gui = TimerAction(period=4.0, actions=[ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=["rqt", "--force-discover", "--standalone",
             "piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin"],
        output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="SAFETY: true = validate only, robot never moves"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_realsense", default_value="true"),
        DeclareLaunchArgument("use_piper_driver", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("calibration_method", default_value="TSAI",
                              description="TSAI (Tsai-Lenz) | PARK | HORAUD | ANDREFF | DANIILIDIS"),
        LogInfo(msg=["[SAFETY] dry_run=", dry_run,
                     "  (true = no motion; pass dry_run:=false only after verifying poses)"]),
        piper_driver, realsense, detector, control, manager, tf_pub, gui,
    ])

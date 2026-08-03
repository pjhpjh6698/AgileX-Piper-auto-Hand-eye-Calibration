"""Look at the calibration result in RViz, on the live arm.

Brings up the pieces needed to SEE whether gripper_T_camera is right:

  robot_state_publisher   URDF -> the base_link..link6 TF chain
  agx_arm_ctrl            the arm, so /joint_states is real and the model moves
  piper_control_node      republishes joint states in the names the URDF uses
  calibration_tf_publisher_node   link6 -> camera_color_optical_frame
  (RealSense + detector)  optional, to also see the marker being detected
  rviz2

What to check once it is up: move the arm by hand or with a goal, and watch the
camera frame stay glued to the wrist in the place the real camera physically
sits. A calibration that is wrong shows up immediately as a camera frame
floating off the gripper or pointing the wrong way.

  ros2 launch piper_auto_handeye view_calibration.launch.py
  ros2 launch piper_auto_handeye view_calibration.launch.py use_camera:=false
  ros2 launch piper_auto_handeye view_calibration.launch.py \
      calibration_file:=/home/pjh/.ros/piper_auto_handeye/handeye_park_...yaml

This launch NEVER commands motion: no calibration manager runs, and the control
node comes up in dry_run.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("piper_auto_handeye")
    piper_cfg = os.path.join(pkg, "config", "piper.yaml")
    handeye_cfg = os.path.join(pkg, "config", "handeye.yaml")
    aruco_cfg = os.path.join(pkg, "config", "aruco.yaml")

    use_camera = LaunchConfiguration("use_camera")
    use_driver = LaunchConfiguration("use_driver")
    can_port = LaunchConfiguration("can_port")
    calib_file = LaunchConfiguration("calibration_file")
    wrist_serial = LaunchConfiguration("wrist_camera_serial")

    urdf = PathJoinSubstitution([
        FindPackageShare("agx_arm_description"),
        "agx_arm_urdf", "piper", "urdf", "piper_with_gripper_description.xacro"])

    # The URDF drives the base_link..link6 chain from /joint_states. Without it
    # the calibration TF is an orphan frame with nothing to hang off. The
    # bridge forwards the driver's joint1..6 and holds the gripper joints the
    # driver does not report at 0.
    jsp = Node(
        package="piper_auto_handeye", executable="joint_state_bridge_node",
        name="joint_state_bridge", output="screen")
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": ParameterValue(
            Command(["xacro ", urdf]), value_type=str)}])

    driver = GroupAction(
        condition=IfCondition(use_driver),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("agx_arm_ctrl"), "launch",
                "start_single_agx_arm.launch.py"])),
            launch_arguments={"can_port": can_port,
                              "arm_type": "piper",
                              "auto_enable": "false"}.items())])

    # dry_run: this node is here only to republish /joint_states under the
    # URDF's joint names. It must never move anything.
    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node", output="screen",
        parameters=[piper_cfg, {"dry_run": True}])

    tf_pub = Node(
        package="piper_auto_handeye", executable="calibration_tf_publisher_node",
        name="calibration_tf_publisher_node", output="screen",
        parameters=[handeye_cfg, {"auto_publish": True,
                                  "calibration_file": calib_file}])

    realsense = GroupAction(
        condition=IfCondition(use_camera),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
            launch_arguments={"serial_no": wrist_serial,
                              "camera_namespace": "wrist",
                              "camera_name": "camera",
                              "enable_color": "true",
                              "enable_depth": "false",
                              "enable_infra1": "false",
                              "enable_infra2": "false",
                              "rgb_camera.color_profile": "1280,720,30",
                              "pointcloud.enable": "false"}.items())])

    detector = Node(
        condition=IfCondition(use_camera),
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", output="screen",
        parameters=[aruco_cfg, {"publish_tf": True}])

    rviz = Node(package="rviz2", executable="rviz2", name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg, "rviz", "calibration.rviz")])

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="also start the wrist camera + detector"),
        DeclareLaunchArgument("use_driver", default_value="true",
                              description="start the arm driver so the model moves"),
        DeclareLaunchArgument("can_port", default_value="can_follower"),
        DeclareLaunchArgument("wrist_camera_serial", default_value="_338522300590"),
        DeclareLaunchArgument(
            "calibration_file", default_value="",
            description="result yaml to display; empty = newest in the output dir"),
        jsp, rsp, driver, control, tf_pub, realsense, detector, rviz,
    ])

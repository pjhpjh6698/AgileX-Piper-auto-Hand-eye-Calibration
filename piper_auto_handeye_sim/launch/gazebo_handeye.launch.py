"""One-command Gazebo hand-eye calibration verification.

Brings up:
  Gazebo (custom world with the ArUco target on the floor)
  the Piper + flange camera, spawned from urdf/piper_handeye_gazebo.xacro
  ros2_control : joint_state_broadcaster + arm_controller
  gazebo_piper_driver_node : emulates the real Piper driver topics
  piper_control_node       : UNCHANGED from hardware
  aruco_detector_node      : UNCHANGED from hardware
  handeye_calibration_node : UNCHANGED from hardware
  ground_truth_reporter_node : prints the true answer from tf

Everything above `gazebo_piper_driver_node` is exactly the hardware stack,
which is the whole point: the sim verifies the code that ships.

Usage:
    ros2 launch piper_auto_handeye_sim gazebo_handeye.launch.py
    # then, in another terminal:
    ros2 action send_goal /run_calibration \
      auto_handeye_interfaces/action/RunCalibration \
      "{target_sample_count: 14, auto_move: true, calibration_method: PARK, \
        save_on_success: true}" --feedback

use_sim_time is forced true everywhere: the calibration manager pairs robot
poses with marker detections by TIMESTAMP, so a node still on wall clock would
silently reject every sample as out of sync.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    sim_share = get_package_share_directory("piper_auto_handeye_sim")

    xacro_file = os.path.join(sim_share, "urdf", "piper_handeye_gazebo.xacro")
    world_file = os.path.join(sim_share, "worlds", "handeye_calibration.world")
    piper_cfg = os.path.join(sim_share, "config", "sim_piper.yaml")
    aruco_cfg = os.path.join(sim_share, "config", "sim_aruco.yaml")
    handeye_cfg = os.path.join(sim_share, "config", "sim_handeye.yaml")
    poses_file = os.path.join(sim_share, "config", "sim_calibration_poses.yaml")

    # Strip XML comments before publishing robot_description.
    #
    # gazebo_ros2_control starts its controller_manager by passing the whole
    # URDF as a command-line argument ("--param robot_description:=<xml>").
    # The ROS 2 argument parser chokes on the comment blocks xacro emits,
    # failing with "Couldn't parse parameter override rule" -- and then the
    # controller_manager never comes up, so `ros2 control load_controller`
    # waits forever on a service that will never appear.
    # (piper_gazebo's own launch file does the same thing, for the same reason.)
    robot_description = re.sub(r"<!--.*?-->", "",
                               xacro.process_file(xacro_file).toxml(),
                               flags=re.DOTALL)

    use_gui = LaunchConfiguration("gui")
    use_rqt = LaunchConfiguration("rqt")

    # Gazebo needs two things on GAZEBO_MODEL_PATH:
    #   1. our models/ dir, for model://aruco_marker
    #   2. the dir CONTAINING piper_description, for the arm meshes. The URDF
    #      refers to them as package://piper_description/meshes/*.STL, and
    #      sdformat rewrites that to model://piper_description/... on spawn, so
    #      Gazebo resolves it as a model name and needs the parent directory.
    #      Without this every link renders as an invisible collision-less blob
    #      ("Failed to find mesh file [model://piper_description/...]").
    #      Same applies to realsense2_description, which supplies the D435 mesh.
    mesh_pkg_parents = []
    for pkg in ("piper_description", "realsense2_description"):
        try:
            mesh_pkg_parents.append(
                os.path.dirname(get_package_share_directory(pkg)))
        except Exception:  # noqa: BLE001 - optional; only the visual suffers
            pass
    model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join([os.path.join(sim_share, "models")]
                              + mesh_pkg_parents
                              + [os.environ.get("GAZEBO_MODEL_PATH", "")]))

    # Disable the online model database. Gazebo Classic fetches its index from
    # models.gazebosim.org on startup, but that server was retired -- the
    # request hangs until it times out and the world never finishes loading
    # ("Getting models from[http://models.gazebosim.org/]" then nothing).
    # Every model this world uses (ground_plane, sun, aruco_marker) is local,
    # so there is nothing to lose by turning the lookup off.
    model_db = SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value="")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])),
        launch_arguments={"world": world_file,
                          "verbose": "true",
                          "gui": use_gui}.items())

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True,
                     "publish_frequency": 50.0}])

    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "piper", "-topic", "robot_description",
                   "-z", "0.0"])

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"], output="screen")
    load_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "arm_controller"], output="screen")
    load_grip = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "gripper_controller"], output="screen")

    # Gazebo stand-in for the real CAN driver
    driver = Node(
        package="piper_auto_handeye_sim", executable="gazebo_piper_driver_node",
        name="gazebo_piper_driver_node", output="screen",
        parameters=[{"use_sim_time": True,
                     "base_frame": "base_link",
                     "gripper_frame": "link6"}])

    # ---- the UNCHANGED hardware stack ----
    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node", output="screen",
        parameters=[piper_cfg, {"use_sim_time": True}])

    detector = Node(
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", output="screen",
        parameters=[aruco_cfg, {"use_sim_time": True}])

    manager = Node(
        package="piper_auto_handeye", executable="handeye_calibration_node",
        name="handeye_calibration_node", output="screen",
        parameters=[handeye_cfg,
                    {"use_sim_time": True,
                     "calibration_poses_file": poses_file}])

    tf_pub = Node(
        package="piper_auto_handeye", executable="calibration_tf_publisher_node",
        name="calibration_tf_publisher_node", output="screen",
        parameters=[handeye_cfg, {"use_sim_time": True, "auto_publish": False}])

    truth = Node(
        package="piper_auto_handeye_sim", executable="ground_truth_reporter_node",
        name="ground_truth_reporter_node", output="screen",
        parameters=[{"use_sim_time": True,
                     "gripper_frame": "link6",
                     "camera_frame": "camera_color_optical_frame"}])

    # The same rqt plugin used on hardware -- it speaks only ROS topics,
    # services and actions, so nothing about it is simulation-specific.
    # --force-discover is required because rqt caches plugin discovery and
    # would otherwise not see a freshly built plugin.
    gui = TimerAction(period=4.0, actions=[ExecuteProcess(
        condition=IfCondition(use_rqt),
        cmd=["rqt", "--force-discover", "--standalone",
             "piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin"],
        output="screen")])

    # Controllers can only be loaded once the model exists in Gazebo, and the
    # driver needs the controllers before it can command anything.
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[load_jsb]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))
    after_arm = RegisterEventHandler(
        OnProcessExit(target_action=load_arm,
                      on_exit=[load_grip, driver,
                               TimerAction(period=3.0,
                                           actions=[control, detector,
                                                    manager, tf_pub, truth,
                                                    gui])]))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="run the Gazebo client window"),
        DeclareLaunchArgument("rqt", default_value="false",
                              description="run the hand-eye rqt GUI"),
        model_path, model_db,
        gazebo, rsp, spawn,
        after_spawn, after_jsb, after_arm,
    ])

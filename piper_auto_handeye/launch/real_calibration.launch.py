"""ONE-COMMAND real-robot Eye-in-Hand calibration (Piper + RealSense + GUI).

Brings up everything needed:
  - AgileX arm driver (agx_arm_ctrl, opens CAN)        [use_piper_driver]
  - RealSense camera                                   [use_realsense]
  - ArUco detector (publishes the GUI camera view)
  - Piper control adapter (safety-validated motion)
  - Hand-eye calibration manager (Tsai-Lenz by default)
  - Static TF publisher
  - RQt GUI                                            [use_gui, gui_lang]

The AgileX driver (agx_arm_ctrl) and its SDK (pyAgxArm) are both vendored in
this workspace, so nothing outside `colcon build` is required. Bring the CAN
link up and prove the arm answers BEFORE launching:

  bash piper_auto_handeye/scripts/can_setup.sh can_follower
  ros2 run piper_auto_handeye agx_arm_check --can-port can_follower

The driver is launched with auto_enable:=false on purpose: nothing powers the
joints until piper_control_node asks for it, immediately before a live move.

##########################  SAFETY  ##########################
THIS LAUNCH MOVES THE ROBOT. Pressing Start commands real motion.
Verify that every pose in config/calibration_poses.yaml is reachable and
collision-free for YOUR setup before the first run, and keep a hand on the
e-stop. The GUI shows a red "LIVE" banner whenever motion is armed.

What actually protects you at runtime, in piper.yaml:
  workspace_min / workspace_max   Cartesian bounds; a goal outside is refused
  max_step_distance               refuses a single move longer than this
  max_speed_fraction              caps the speed pushed to the driver
  STOP button                     holds the current pose immediately
##############################################################

Typical use:
  ros2 launch piper_auto_handeye real_calibration.launch.py

  # walk the pose list without commanding anything (validation only):
  ros2 launch piper_auto_handeye real_calibration.launch.py dry_run:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, ExecuteProcess, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    gui_lang = LaunchConfiguration("gui_lang")
    dry_run = LaunchConfiguration("dry_run")
    can_port = LaunchConfiguration("can_port")
    backend = LaunchConfiguration("control_backend")
    method = LaunchConfiguration("calibration_method")
    wrist_serial = LaunchConfiguration("wrist_camera_serial")

    # --- AgileX low-level driver (opens the CAN bus); vendored in this workspace ---
    # auto_enable=false: the joints stay unpowered until piper_control_node
    # explicitly enables them for a live move, so a dry run never energises the arm.
    # speed_percent is a floor only -- piper_control_node overwrites it per goal.
    piper_driver = GroupAction(
        condition=IfCondition(use_piper_driver),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("agx_arm_ctrl"), "launch",
                "start_single_agx_arm.launch.py"])),
            launch_arguments={"can_port": can_port,
                              "arm_type": "piper",
                              "auto_enable": "false",
                              "speed_percent": "26"}.items())])

    # --- robot model -> TF chain (for the GUI's embedded RViz) ---
    # The WITH-GRIPPER model, matching what start_single_piper_rviz shows. The
    # driver only reports joint1..6 (effector_type=none), so the bridge
    # republishes them onto /joint_states with the gripper joints held at 0 --
    # without that, the gripper links have no TF and RViz flags them in red.
    # (Own node, not ros-humble-joint-state-publisher: that package is not
    # installed everywhere and this workspace promises one colcon build.)
    urdf = PathJoinSubstitution([
        FindPackageShare("agx_arm_description"),
        "agx_arm_urdf", "piper", "urdf", "piper_with_gripper_description.xacro"])
    jsp = Node(
        package="piper_auto_handeye", executable="joint_state_bridge_node",
        name="joint_state_bridge", output="screen")
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": ParameterValue(
            Command(["xacro ", urdf]), value_type=str)}])

    # --- RealSense (the WRIST camera) ---
    # Two RealSense cameras are attached, so pin the eye-in-hand one by serial
    # number: USB enumeration order is not stable across reboots and grabbing
    # the wrong camera would silently calibrate against the wrong rigid body.
    # realsense2_camera needs the serial with a leading underscore -- without it
    # the value is parsed as an integer and the match fails.
    # Depth is off: ArUco needs colour only, and two D455s on one controller can
    # saturate USB bandwidth otherwise.
    realsense = GroupAction(
        condition=IfCondition(use_realsense),
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
        package="piper_auto_handeye", executable="aruco_detector_node",
        name="aruco_detector_node", parameters=[aruco_cfg], output="screen")

    control = Node(
        package="piper_auto_handeye", executable="piper_control_node",
        name="piper_control_node",
        # can_port is NOT passed here: the driver owns the bus, this node only
        # talks to the driver over topics.
        parameters=[piper_cfg, {"dry_run": dry_run,
                                "control_backend": backend}], output="screen")

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
    # gui_lang travels in the environment rather than as a ROS parameter: rqt
    # constructs the plugin itself, so the widget has already built its labels
    # before any parameter client could answer.
    gui = TimerAction(period=4.0, actions=[ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=["rqt", "--force-discover", "--standalone",
             "piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin"],
        additional_env={"HANDEYE_GUI_LANG": gui_lang},
        output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true = validate poses only, robot never moves. "
                                          "Default false: this launch is the REAL-ROBOT "
                                          "entry point and Start moves the arm."),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("gui_lang", default_value="ko",
                              description="GUI language: ko (default) | en"),
        DeclareLaunchArgument("use_realsense", default_value="true"),
        DeclareLaunchArgument("use_piper_driver", default_value="true",
                              description="start the vendored AgileX agx_arm_ctrl driver"),
        DeclareLaunchArgument("control_backend", default_value="agx",
                              description="agx = via the AgileX driver (real robot) | "
                                          "topic = /pos_cmd interface (simulation only)"),
        DeclareLaunchArgument("can_port", default_value="can_follower",
                              description="socketcan interface the arm is on"),
        DeclareLaunchArgument("wrist_camera_serial", default_value="_338522300590",
                              description="serial of the WRIST D455 (keep the leading "
                                          "underscore; list with rs-enumerate-devices)"),
        DeclareLaunchArgument("calibration_method", default_value="TSAI",
                              description="TSAI (Tsai-Lenz) | PARK | HORAUD | ANDREFF | DANIILIDIS"),
        LogInfo(msg=["[SAFETY] dry_run=", dry_run,
                     "  (false = Start MOVES THE ROBOT; pass dry_run:=true to validate only)"]),
        LogInfo(msg=["[ROBOT] backend=", backend, " can_port=", can_port]),
        piper_driver, jsp, rsp, realsense, detector, control, manager, tf_pub, gui,
    ])

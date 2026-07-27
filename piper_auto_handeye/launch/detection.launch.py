"""Launch the ArUco detector (and optionally the RealSense camera)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("piper_auto_handeye")
    aruco_cfg = os.path.join(pkg, "config", "aruco.yaml")

    use_realsense = LaunchConfiguration("use_realsense")
    namespace = LaunchConfiguration("namespace")

    realsense = GroupAction(
        condition=IfCondition(use_realsense),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
            launch_arguments={"enable_color": "true", "pointcloud.enable": "false"}.items())])

    detector = Node(
        package="piper_auto_handeye",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        namespace=namespace,
        parameters=[aruco_cfg],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="false",
                              description="Also launch the RealSense driver"),
        DeclareLaunchArgument("namespace", default_value=""),
        realsense,
        detector,
    ])

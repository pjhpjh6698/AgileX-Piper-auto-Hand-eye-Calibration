import os
from glob import glob

from setuptools import find_packages, setup

package_name = "piper_auto_handeye"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    # also into lib/<pkg> so `ros2 run piper_auto_handeye can_setup.sh` works
    scripts=glob("scripts/*.sh"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="pjhpjh6698@gmail.com",
    description="Automatic Eye-in-Hand calibration for Piper + RealSense (ArUco).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector_node = piper_auto_handeye.aruco_detector_node:main",
            "piper_control_node = piper_auto_handeye.piper_control_node:main",
            "agx_arm_check = piper_auto_handeye.agx_arm_check:main",
            "generate_poses = piper_auto_handeye.generate_poses:main",
            "handeye_calibration_node = piper_auto_handeye.handeye_calibration_node:main",
            "calibration_tf_publisher_node = piper_auto_handeye.calibration_tf_publisher_node:main",
            "mock_robot_node = piper_auto_handeye.mock_robot_node:main",
            "synthetic_marker_publisher_node = piper_auto_handeye.synthetic_marker_publisher_node:main",
        ],
    },
)

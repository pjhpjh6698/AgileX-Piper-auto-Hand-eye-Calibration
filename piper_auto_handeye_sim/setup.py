import os
from glob import glob

from setuptools import find_packages, setup

package_name = "piper_auto_handeye_sim"


def _model_files():
    """Install the Gazebo model tree preserving its directory layout."""
    out = []
    for root, _dirs, files in os.walk("models"):
        if not files:
            continue
        out.append((os.path.join("share", package_name, root),
                    [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ] + _model_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="pjhpjh6698@gmail.com",
    description="Gazebo simulation for verifying Piper Eye-in-Hand calibration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gazebo_piper_driver_node = "
            "piper_auto_handeye_sim.gazebo_piper_driver_node:main",
            "ground_truth_reporter_node = "
            "piper_auto_handeye_sim.ground_truth_reporter_node:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "piper_auto_handeye_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="pjhpjh6698@gmail.com",
    description="RQt GUI plugin for Piper auto hand-eye calibration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # No console_scripts: this is an rqt plugin, discovered via plugin.xml and
    # launched with `rqt --standalone piper_auto_handeye_gui`.
)

"""Vendored AgileX pyAgxArm SDK.

Installs the upstream ``pyAgxArm`` package (and all subpackages) into the
colcon install tree, so `import pyAgxArm` works after `source
install/setup.bash` with no pip install and no PYTHONPATH juggling.

`agx_arm_ctrl` imports it, and so does the calibration stack's pre-flight
check. The sources under ``pyAgxArm/`` are upstream v1.0.0, unmodified --
see section 19 of the workspace README before touching them.
"""
from setuptools import find_packages, setup

package_name = "pyagxarm_vendor"

setup(
    name=package_name,
    version="1.0.0",
    # picks up pyAgxArm and every nested subpackage (api, protocols, ...)
    packages=find_packages(exclude=["test"]),
    # PEP 561 type-hint payload: without this the .pyi stubs and py.typed
    # marker are dropped and editors lose the SDK's type information.
    package_data={"": ["py.typed", "*.pyi"]},
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "python-can", "typing-extensions"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="pjhpjh6698@gmail.com",
    description="Vendored AgileX pyAgxArm SDK (v1.0.0) for the auto hand-eye workspace.",
    license="MIT",
)

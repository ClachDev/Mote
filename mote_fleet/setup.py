import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mote_fleet"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # The JSON Schema mirror of the wire contract, installed so a consumer
        # (the M3 dashboard, an external tool) can read it from the package
        # rather than vendor a copy.
        (os.path.join("share", package_name, "schema"), glob("schema/*.json")),
    ],
    install_requires=["setuptools"],
    # the 'test' extra tells colcon to run these tests with pytest
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Michael Johnson",
    maintainer_email="michael@clach.dev",
    description=(
        "Fleet control plane: the robot's agent, and the off-board "
        "enrollment/registry server"
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agent = mote_fleet.agent:main",
            "enroll = mote_fleet.enroll:main",
            "publish-map = mote_fleet.publish:main",
        ],
    },
)

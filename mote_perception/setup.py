import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mote_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    # the 'test' extra tells colcon to run these tests with pytest
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Michael Johnson",
    maintainer_email="michael@clach.dev",
    description="Camera-derived perception nodes for the mote",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_monitor = mote_perception.camera_monitor:main",
            "depth_obstacle_node = mote_perception.depth_obstacle_node:main",
        ],
    },
)

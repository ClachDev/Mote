import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mote_bringup"

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
    description="Launch files for the mote",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "odom_tf_relay = mote_bringup.odom_tf_relay:main",
            "system_monitor = mote_bringup.system_monitor:main",
            "bag_pruner = mote_bringup.bag_pruner:main",
            "site = mote_bringup.sites:main",
        ],
    },
)

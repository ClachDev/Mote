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
        (
            os.path.join("share", package_name, "provisioning"),
            glob("provisioning/*"),
        ),
        (os.path.join("share", package_name, "foxglove"), glob("foxglove/*.json")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Michael Johnson",
    maintainer_email="michael@clach.dev",
    description="Launch files for the mote",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "twist_relay = mote_bringup.twist_relay:main",
            "system_monitor = mote_bringup.system_monitor:main",
            "slip_monitor = mote_bringup.slip_monitor:main",
            "health_monitor = mote_bringup.health_monitor:main",
            "self_check = mote_bringup.self_check:main",
            "bag_pruner = mote_bringup.bag_pruner:main",
            "site = mote_bringup.sites:main",
            "identity = mote_bringup.identity:main",
            "provision = mote_bringup.provision:main",
            "dds_participants = mote_bringup.dds_participants:main",
            "sweep_orphans = mote_bringup.sweep_orphans:main",
            "explore = mote_bringup.explore:main",
        ],
    },
)

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mote_tasks"

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
    description="Behaviour-tree task layer that drives Nav2 to run missions",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_server = mote_tasks.task_server:main",
            "save_zone = mote_tasks.save_zone:main",
            "mission = mote_tasks.mission:main",
        ],
    },
)

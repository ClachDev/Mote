import os
from glob import glob

from setuptools import find_packages, setup

package_name = "auldbot_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Johnson",
    maintainer_email="mjohnson@augereai.com",
    description="Launch files for the auldbot",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)

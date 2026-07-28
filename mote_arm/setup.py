from setuptools import find_packages, setup

package_name = "mote_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # the 'test' extra tells colcon to run these tests with pytest
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Michael Johnson",
    maintainer_email="michael@clach.dev",
    description="SO-101 follower arm bring-up: joint states, safe jog control, bench tools",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "jog = mote_arm.jog:main",
            "arm_check = mote_arm.arm_check:main",
            "arm_calibrate = mote_arm.arm_calibrate:main",
            "arm_offsets = mote_arm.arm_offsets:main",
            "arm_pose = mote_arm.arm_pose:main",
            "arm_gains = mote_arm.arm_gains:main",
        ],
    },
)

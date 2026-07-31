"""What the drive mux actually does, measured against a real `twist_mux`.

`test_twist_mux.py` checks the shipped numbers agree with each other. This runs
the node those numbers configure, with the repo's own config file, and watches
the topic the wheel controller subscribes to. Four behaviours are worth a real
process rather than reasoning:

* **teleop wins while the operator is driving** — the whole point, and the thing
  that was broken before the mux existed;
* **the robot stops before Nav2 gets the wheels back.** The gap between the
  operator's last command and Nav2's first is measured here and compared against
  the controller's `cmd_vel_timeout`, because that inequality is the difference
  between a robot that pauses on handback and one that hands straight back
  mid-motion;
* **nothing is re-published.** The mux sits in the path the deadman protects, so
  a stored last command anywhere in it would turn "the operator's link dropped"
  into "the robot keeps going";
* **the message type is TwistStamped.** `use_stamped` is a twist_mux default
  rather than something the launch sets, so a release that flips it must fail
  here and not on a robot that silently stops moving.

Sources are told apart by their linear velocity, so every assertion is about
which one reached the wheels.
"""

import os
import pathlib
import random
import shutil
import subprocess
import threading
import time

import pytest
import yaml

from mote_bringup.sweep_orphans import reap_group, spawn_reapable

REPO = pathlib.Path(__file__).resolve().parents[2]
CONFIG = REPO / "mote_bringup" / "config" / "twist_mux.yaml"
CONTROLLERS = REPO / "mote_bringup" / "config" / "controllers.yaml"

MUX_PARAMS = yaml.safe_load(CONFIG.read_text())["twist_mux"]["ros__parameters"]
NAV_TOPIC = MUX_PARAMS["topics"]["navigation"]["topic"]
TELEOP_TOPIC = MUX_PARAMS["topics"]["teleop"]["topic"]
TELEOP_TIMEOUT = MUX_PARAMS["topics"]["teleop"]["timeout"]
LOCK_TOPIC = MUX_PARAMS["locks"]["pause_navigation"]["topic"]
CMD_VEL_TIMEOUT = yaml.safe_load(CONTROLLERS.read_text())["diff_drive_controller"][
    "ros__parameters"
]["cmd_vel_timeout"]

DRIVE_TOPIC = "/diff_drive_controller/cmd_vel"

# Distinguishable on sight, and both inside the controller's limits.
NAV_SPEED = 0.10
TELEOP_SPEED = 0.20


def _has_twist_mux() -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory

        get_package_share_directory("twist_mux")
        return shutil.which("ros2") is not None
    except Exception:
        return False


# skipif rather than importorskip: raising Skipped during module import aborts
# collection for the whole directory under pytest 9.
pytestmark = pytest.mark.skipif(
    not _has_twist_mux(), reason="needs the twist_mux package and the ros2 CLI"
)

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402


class _Harness(Node):
    """Both mux inputs, the lock, and a log of what reached the wheels."""

    def __init__(self):
        super().__init__("twist_mux_test_harness")
        self.nav = self.create_publisher(TwistStamped, NAV_TOPIC, 10)
        self.teleop = self.create_publisher(TwistStamped, TELEOP_TOPIC, 10)
        self.lock = self.create_publisher(Bool, LOCK_TOPIC, 10)
        self.out: list[tuple[float, TwistStamped]] = []
        self.create_subscription(TwistStamped, DRIVE_TOPIC, self._on_out, 50)
        self._nav_speed = 0.0
        self._teleop_speed = None
        # 20 Hz, the controller_server's rate; teleop is driven from the same
        # timer so the two sources cannot drift apart between runs.
        self.create_timer(0.05, self._publish)

    def _on_out(self, msg):
        self.out.append((time.monotonic(), msg))

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        if self._nav_speed:
            self.nav.publish(self._twist(self._nav_speed, stamp))
        if self._teleop_speed is not None:
            self.teleop.publish(self._twist(self._teleop_speed, stamp))

    def _twist(self, speed, stamp):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = speed
        return msg

    def drive_nav(self, on):
        self._nav_speed = NAV_SPEED if on else 0.0

    def drive_teleop(self, speed):
        """`None` stops publishing entirely, as releasing a button does."""
        self._teleop_speed = speed

    def since(self):
        """A marker for `self.out` so each phase reads only its own messages."""
        return len(self.out)

    def after(self, mark):
        return [m for _, m in self.out[mark:]]

    def set_lock(self, value):
        self.lock.publish(Bool(data=value))


def _who(msg) -> str:
    if abs(msg.twist.linear.x - TELEOP_SPEED) < 1e-6:
        return "teleop"
    if abs(msg.twist.linear.x - NAV_SPEED) < 1e-6:
        return "nav"
    return f"unknown({msg.twist.linear.x})"


@pytest.fixture(scope="module")
def mux():
    # A random domain keeps this off the default graph: the topic it publishes
    # is the one a robot on the bench drives on.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(80, 160))

    # spawn_reapable, not Popen: `ros2 run` forwards no signal to the node it
    # spawned, so terminating the wrapper alone leaks a twist_mux per run.
    proc = spawn_reapable(
        [
            "ros2",
            "run",
            "twist_mux",
            "twist_mux",
            "--ros-args",
            "--params-file",
            str(CONFIG),
            "-r",
            "__node:=twist_mux",
            "-r",
            f"cmd_vel_out:={DRIVE_TOPIC}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )

    rclpy.init()
    node = _Harness()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Wait for discovery by watching for the first message through the mux
    # rather than by sleeping a guessed interval.
    node.drive_nav(True)
    deadline = time.time() + 30
    while time.time() < deadline and not node.out:
        time.sleep(0.1)
    node.drive_nav(False)
    if not node.out:
        reap_group(proc)
        executor.shutdown()
        rclpy.try_shutdown()
        pytest.fail("no message reached the drive topic within 30 s")

    yield node

    reap_group(proc)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


def _settle(seconds=0.4):
    time.sleep(seconds)


def test_navigation_alone_reaches_the_wheels(mux):
    mux.drive_nav(True)
    mark = mux.since()
    _settle()
    mux.drive_nav(False)
    seen = mux.after(mark)
    assert seen, "nav commands did not reach the drive topic"
    assert {_who(m) for m in seen} == {"nav"}


def test_the_mux_forwards_twist_stamped(mux):
    """`use_stamped` is a twist_mux default, so it is checked and not assumed."""
    mux.drive_nav(True)
    mark = mux.since()
    _settle()
    mux.drive_nav(False)
    seen = mux.after(mark)
    assert seen
    assert all(isinstance(m, TwistStamped) for m in seen)
    # Forwarded unmodified: the stamp the controller measures staleness against
    # is still the producer's.
    assert all(m.header.frame_id == "base_footprint" for m in seen)
    assert all(m.header.stamp.sec or m.header.stamp.nanosec for m in seen)


def test_teleop_preempts_an_active_nav_goal(mux):
    """The behaviour the mux exists for: both sources up, the operator wins."""
    mux.drive_nav(True)
    mux.drive_teleop(TELEOP_SPEED)
    _settle()
    mark = mux.since()
    _settle()
    mux.drive_teleop(None)
    mux.drive_nav(False)
    seen = mux.after(mark)
    assert seen
    assert {_who(m) for m in seen} == {"teleop"}


def test_letting_go_stops_the_robot_before_nav_resumes(mux):
    """Measured, because this inequality is what makes handback safe.

    The gap between the operator's last command and Nav2's first must exceed the
    controller's `cmd_vel_timeout`, or the wheels never halt in between.
    """
    mux.drive_nav(True)
    mark = mux.since()
    mux.drive_teleop(TELEOP_SPEED)
    _settle(0.6)

    mux.drive_teleop(None)
    time.sleep(TELEOP_TIMEOUT + 0.6)
    mux.drive_nav(False)

    # Everything the wheels saw across the takeover, in order: teleop while the
    # operator drove, then nothing, then nav again.
    timeline = mux.out[mark:]
    last_teleop = max(t for t, m in timeline if _who(m) == "teleop")
    first_nav = min(t for t, m in timeline if _who(m) == "nav" and t > last_teleop)

    gap = first_nav - last_teleop
    assert gap > CMD_VEL_TIMEOUT, f"only {gap:.3f} s of silence, wheels never halt"
    # And not so long that a handback looks like a hang: the mux masks Nav2 for
    # its teleop timeout and no longer.
    assert gap < TELEOP_TIMEOUT + 0.3, f"nav suppressed for {gap:.3f} s"


def test_the_mux_never_republishes(mux):
    """Every source silent means the drive topic silent — the deadman's premise."""
    mux.drive_nav(True)
    _settle()
    mux.drive_nav(False)
    time.sleep(TELEOP_TIMEOUT + 0.5)

    mark = mux.since()
    time.sleep(1.0)
    assert mux.after(mark) == []


def test_the_pause_lock_holds_nav_off_and_lets_teleop_through(mux):
    mux.set_lock(True)
    _settle()

    mux.drive_nav(True)
    mark = mux.since()
    _settle(0.6)
    assert mux.after(mark) == [], "navigation drove the wheels while locked"

    mux.drive_teleop(TELEOP_SPEED)
    mark = mux.since()
    _settle(0.6)
    mux.drive_teleop(None)
    seen = mux.after(mark)
    assert seen, "the lock masked teleop too"
    assert {_who(m) for m in seen} == {"teleop"}

    mux.set_lock(False)
    time.sleep(TELEOP_TIMEOUT + 0.3)
    mark = mux.since()
    _settle(0.6)
    mux.drive_nav(False)
    seen = mux.after(mark)
    assert seen, "navigation did not resume after the lock was released"
    assert {_who(m) for m in seen} == {"nav"}

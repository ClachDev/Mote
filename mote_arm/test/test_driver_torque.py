"""The driver must never enable torque against a stale goal register.

A Feetech servo drives to whatever GOAL_POSITION holds the moment torque is
enabled. If the driver enables torque before writing the present position, the
arm snaps to a pose nobody commanded. These tests pin the ordering with a fake
bus, so the property is checked without hardware.

A random ROS_DOMAIN_ID keeps the test node off any live robot's graph.
"""

import os
import random

os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))

import pytest  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from mote_arm import arm_driver as arm_driver_mod  # noqa: E402
from mote_arm.config import ArmConfig  # noqa: E402

CFG = ArmConfig.from_dict(
    {
        "arm": {
            "port": "/dev/fake",
            "baud_rate": 1000000,
            "joints": [
                {
                    "name": "shoulder_pan",
                    "id": 1,
                    "min": -1.0,
                    "max": 1.0,
                    "home": 2048,
                },
                {"name": "gripper", "id": 6, "min": -1.0, "max": 1.0, "home": 2048},
            ],
        }
    }
)


class FakeBus:
    """Records the ordering of bus operations."""

    def __init__(self, *_a, **_kw):
        self.calls: list[tuple] = []
        self.positions = {1: 2100, 6: 1900}
        self.unreadable: set[int] = set()
        self.wrong_mode: set[int] = set()
        self.closed = False

    def open(self, allow_shared=False):
        self.calls.append(("open",))

    def close(self):
        self.closed = True
        self.calls.append(("close",))

    def ping(self, servo_id):
        return True

    def ensure_position_mode(self, servo_id):
        self.calls.append(("mode", servo_id))
        return servo_id not in self.wrong_mode

    def read_position(self, servo_id):
        if servo_id in self.unreadable:
            return None
        return self.positions.get(servo_id)

    def set_torque(self, servo_id, enable):
        self.calls.append(("torque", servo_id, enable))

    def write_goal(self, servo_id, counts, speed, acc):
        self.calls.append(("goal", servo_id, counts))


@pytest.fixture
def make_driver(monkeypatch):
    """Build an ArmDriver over a FakeBus, optionally prepared to misbehave.

    The bus must be prepared *before* the driver constructs, since enumeration
    and the mode check happen in __init__.
    """
    state = {}

    def factory(prepare=None):
        class PreparedBus(FakeBus):
            def __init__(self, *a, **kw):
                super().__init__(*a, **kw)
                if prepare is not None:
                    prepare(self)

        monkeypatch.setattr(arm_driver_mod, "FeetechBus", PreparedBus)
        monkeypatch.setattr(arm_driver_mod.config, "load", lambda: CFG)
        rclpy.init()
        state["node"] = arm_driver_mod.ArmDriver()
        return state["node"]

    yield factory
    if "node" in state:
        state["node"].destroy_node()
        rclpy.shutdown()


@pytest.fixture
def driver(make_driver):
    return make_driver()


def test_starts_limp(driver):
    """Every joint is explicitly torque-disabled at startup."""
    assert not driver._engaged
    disables = [c for c in driver.bus.calls if c[0] == "torque" and c[2] is False]
    assert {c[1] for c in disables} == {1, 6}
    assert not [c for c in driver.bus.calls if c[0] == "torque" and c[2] is True]


def test_goal_seeded_before_torque_enable(driver):
    """The safety property: goal written first, torque enabled after."""
    driver.bus.calls.clear()
    msg = JointState()
    msg.name = ["shoulder_pan"]
    msg.position = [0.2]
    driver._on_goal(msg)

    for servo_id in (1, 6):
        seed = next(
            i
            for i, c in enumerate(driver.bus.calls)
            if c[0] == "goal" and c[1] == servo_id
        )
        enable = next(
            i
            for i, c in enumerate(driver.bus.calls)
            if c[0] == "torque" and c[1] == servo_id and c[2] is True
        )
        assert seed < enable, f"servo {servo_id}: torque enabled before seeding"


def test_seeded_goal_is_present_position(driver):
    """Seeding uses the measured count, so engaging torque moves nothing."""
    driver.bus.calls.clear()
    request = SetBool.Request()
    request.data = True
    driver._on_set_torque(request, SetBool.Response())

    seeds = {c[1]: c[2] for c in driver.bus.calls if c[0] == "goal"}
    assert seeds == driver.bus.positions


def test_goal_is_clamped_to_soft_limits(driver):
    driver.bus.calls.clear()
    msg = JointState()
    msg.name = ["shoulder_pan"]
    msg.position = [99.0]  # far outside max 1.0
    driver._on_goal(msg)

    joint = CFG.joint("shoulder_pan")
    expected = joint.rad_to_counts(joint.max_rad)
    assert [c for c in driver.bus.calls if c[0] == "goal" and c[1] == 1][-1][
        2
    ] == expected


def test_unknown_joint_is_ignored(driver):
    driver.bus.calls.clear()
    msg = JointState()
    msg.name = ["left_wheel_joint"]  # a drive wheel on the shared bus!
    msg.position = [1.0]
    driver._on_goal(msg)
    assert not [c for c in driver.bus.calls if c[0] == "goal" and c[1] == 7]


def test_shutdown_disables_torque_and_closes(driver):
    driver._engage_all()
    driver.bus.calls.clear()
    driver.shutdown()
    disables = [c for c in driver.bus.calls if c[0] == "torque" and c[2] is False]
    assert {c[1] for c in disables} == {1, 6}
    assert driver.bus.closed


def test_failed_engage_is_retried_on_next_goal(make_driver):
    """A joint whose engage fails stays limp but is not abandoned.

    A single arm-wide torque flag would mark the arm engaged after the first
    goal and never retry the joint that failed its position read — leaving it
    silently limp for the rest of the session.
    """
    driver = make_driver(lambda bus: bus.unreadable.add(6))
    msg = JointState()
    msg.name = ["shoulder_pan"]
    msg.position = [0.2]
    driver._on_goal(msg)

    assert driver._engaged == {"shoulder_pan"}
    assert not [c for c in driver.bus.calls if c == ("torque", 6, True)]

    driver.bus.unreadable.clear()
    driver.bus.calls.clear()
    driver._on_goal(msg)

    assert driver._engaged == {"shoulder_pan", "gripper"}
    seed = driver.bus.calls.index(("goal", 6, driver.bus.positions[6]))
    enable = driver.bus.calls.index(("torque", 6, True))
    assert seed < enable


def test_unverified_mode_is_never_commanded(make_driver):
    """A servo not confirmed in position mode gets no goals and no torque.

    In wheel mode a position goal is obeyed as a speed, so the joint would
    spin continuously; excluding it from control is the only safe answer.
    """
    driver = make_driver(lambda bus: bus.wrong_mode.add(6))
    assert driver._ready == {"shoulder_pan"}

    msg = JointState()
    msg.name = ["gripper"]
    msg.position = [0.2]
    driver._on_goal(msg)

    assert not [c for c in driver.bus.calls if c[0] == "goal" and c[1] == 6]
    assert not [c for c in driver.bus.calls if c == ("torque", 6, True)]


def test_set_torque_service_reports_limp_joints(make_driver):
    driver = make_driver(lambda bus: bus.unreadable.add(6))
    request = SetBool.Request()
    request.data = True
    response = driver._on_set_torque(request, SetBool.Response())

    assert response.success is False
    assert "gripper" in response.message

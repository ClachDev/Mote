"""The wire-only robots are held to the wire.

``fake_robots.py`` exists so the dashboard can be checked against something, and
its whole claim is that it publishes the control-plane contract and nothing
else. That claim is worth exactly as much as a test: without one, a payload the
UI has stopped understanding would still sail through ``fleet-ui-check`` and the
fixture would quietly become a second, wrong definition of the wire.

Nothing here connects: ``FakeRobot`` builds its MQTT client but only touches the
network in ``start()``, so this tier runs anywhere paho imports — no broker, no
ROS, both architectures.
"""

import pytest

pytest.importorskip("paho.mqtt.client")

import fake_robots  # noqa: E402

from mote_bringup.spec import capability as spec_capability  # noqa: E402
from mote_bringup.spec import mission  # noqa: E402
from mote_fleet import protocol  # noqa: E402


def a_command(capability="goto", payload_input=None, **kwargs):
    return mission.command(
        "mote-01",
        capability,
        payload_input if payload_input is not None else {"target": "dropoff"},
        **kwargs,
    )


@pytest.fixture
def robot():
    """One robot, with its publishes intercepted instead of sent."""
    subject = fake_robots.FakeRobot("mote-01", host="127.0.0.1", port=1883)
    subject.published = []
    subject._publish = lambda leaf, payload, retain=True: subject.published.append(
        (leaf, payload)
    )
    return subject


def of(robot, leaf):
    return [payload for name, payload in robot.published if name == leaf]


def states(robot):
    return [payload["state"] for payload in of(robot, protocol.STATUS)]


@pytest.mark.parametrize("profile", fake_robots.PROFILES)
def test_health_and_pose_meet_the_contract(profile):
    robot = fake_robots.FakeRobot("mote-01", host="h", port=1, profile=profile)
    protocol.check(robot.health_payload(), protocol.HEALTH)
    protocol.check(robot.pose_payload(), protocol.POSE)


def test_an_unknown_profile_is_refused():
    with pytest.raises(ValueError):
        fake_robots.FakeRobot("mote-01", host="h", port=1, profile="haunted")


def test_health_carries_subsystems_and_only_claims_a_map_when_told():
    plain = fake_robots.FakeRobot("mote-01", host="h", port=1).health_payload()
    assert [row["name"] for row in plain["subsystems"]]
    # A fixture that invented a revision would show up on the dashboard as a
    # robot running an out-of-date map, which is a real state and must not be
    # faked into existence.
    assert plain["map"] is None

    told = fake_robots.FakeRobot(
        "mote-01", host="h", port=1, revision="20260708T000623"
    ).health_payload()
    assert told["map"]["revision"] == "20260708T000623"


def test_degraded_is_degraded_all_the_way_down():
    degraded = fake_robots.FakeRobot(
        "mote-01", host="h", port=1, profile="degraded"
    ).health_payload()
    assert degraded["state"] == protocol.DEGRADED
    assert protocol.DEGRADED in [row["state"] for row in degraded["subsystems"]]


def test_a_known_mission_runs_to_success(robot):
    command = a_command()
    robot._handle(command)
    assert states(robot) == [mission.DISPATCHED, mission.ACCEPTED]

    robot._mission["due"] = 0  # the mission's time is up
    robot.tick()
    assert states(robot)[-1] == mission.SUCCEEDED
    for payload in of(robot, protocol.STATUS):
        mission.check(payload, "status")
        assert payload["id"] == command["id"]


def test_the_capability_set_is_the_robots_own(robot):
    """Imported from ``mote_tasks``, never written here: the dashboard's
    dispatch form is generated from this document, so a fixture with a
    hand-written one would let the form drift from the robot it drives."""
    assert [item["key"] for item in robot.capabilities["capabilities"]] == [
        "goto",
        "fetch",
    ]
    assert robot.capabilities["platform_id"] == "mote-01"
    for item in robot.capabilities["capabilities"]:
        spec_capability.check(item)


@pytest.mark.parametrize(
    "command,failure_class",
    [
        (a_command(capability="wibble"), mission.UNKNOWN_CAPABILITY),
        (a_command(payload_input={}), mission.INVALID_INPUT),
        (a_command(payload_input={"target": "Nowhere"}), mission.INVALID_INPUT),
        (a_command(payload_input={"target": "nowhere"}), mission.UNRESOLVED_ZONE),
        (
            a_command(capability="fetch", payload_input={"target": "box"}),
            mission.INVALID_INPUT,
        ),
    ],
)
def test_a_refusal_is_typed_the_way_the_robot_types_it(robot, command, failure_class):
    robot._handle(command)
    assert states(robot) == [mission.DISPATCHED, mission.REJECTED]
    assert of(robot, protocol.STATUS)[-1]["failure"]["class"] == failure_class


def test_a_redelivery_is_recognised_not_re_run(robot):
    command = a_command(payload_input={"target": "home"})
    robot._handle(command)
    robot._handle(command)
    # The same mission id arriving twice re-publishes where it got to; it does
    # not start a second mission, and it is not rejected as "busy" with itself.
    assert states(robot) == [
        mission.DISPATCHED,
        mission.ACCEPTED,
        mission.ACCEPTED,
    ]


def test_a_second_mission_is_refused_while_one_is_in_flight(robot):
    robot._handle(a_command(payload_input={"target": "home"}))
    robot._handle(a_command(payload_input={"target": "pickup"}))
    rejection = of(robot, protocol.STATUS)[-1]
    assert states(robot)[-1] == mission.REJECTED
    assert rejection["failure"]["class"] == mission.BUSY
    # The busy failure names the mission that holds the lane, so a dispatcher
    # knows what to wait for rather than merely that it must wait.
    assert rejection["failure"]["recoverable"] is True
    assert "lane" in rejection["failure"]["detail"]


def test_the_will_is_an_offline_presence(robot):
    # paho keeps the will it was handed; this is the payload the *broker*
    # publishes when the socket drops, which is the whole point of the offline
    # profile and cannot be asserted from the robot's own publishes.
    will = robot.client._will_payload
    protocol.check(protocol.decode(will), protocol.PRESENCE)
    assert protocol.decode(will)["online"] is False
    assert robot.client._will_topic.decode() == "mote/v2/mote-01/presence"

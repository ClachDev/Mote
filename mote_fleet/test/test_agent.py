"""The agent's bridge, driven against a fake broker.

No mosquitto here: the MQTT client is injected, so every path — LWT, retained
presence, command in, status out, health from diagnostics — is exercised as
plain ROS spinning plus method calls. That keeps the bridge under test on both
CI architectures, where a broker is not installed. The version with a real
broker and a real behaviour tree is ``test_e2e_fleet.py``.
"""

import json
import os
import random
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from mote_bringup.spec import mission as spec_mission  # noqa: E402
from mote_fleet import agent  # noqa: E402
from mote_tasks import capabilities  # noqa: E402


class FakeMqtt:
    """Stands in for paho: records what was published, injects what arrives."""

    def __init__(self, client_id):
        self.client_id = client_id
        self.published = []
        self.subscriptions = []
        self.will = None
        self.target = None
        self.loop_running = False
        self.disconnected = False
        self.on_connect = self.on_disconnect = self.on_message = None

    # -- the paho surface the agent uses --
    def reconnect_delay_set(self, **_kwargs):
        pass

    def will_set(self, topic, payload, qos=0, retain=False):
        self.will = SimpleNamespace(topic=topic, payload=payload, retain=retain)

    def connect_async(self, host, port, keepalive=60):
        self.target = (host, port, keepalive)

    def loop_start(self):
        self.loop_running = True

    def loop_stop(self):
        self.loop_running = False

    def subscribe(self, topic, qos=0):
        self.subscriptions.append(topic)

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append(
            SimpleNamespace(topic=topic, payload=payload, retain=retain)
        )

    def disconnect(self):
        self.disconnected = True

    # -- test helpers --
    def connect(self):
        self.on_connect(self, None, {}, 0, None)

    def deliver(self, topic, payload):
        self.on_message(self, None, SimpleNamespace(topic=topic, payload=payload))

    def last(self, leaf):
        published = self.all_of(leaf)
        return published[-1] if published else None

    def all_of(self, leaf):
        import json

        from mote_fleet import protocol

        out = []
        for message in self.published:
            parsed = protocol.parse_topic(message.topic)
            if not parsed or parsed[1] != leaf:
                continue
            # A spec payload is checked by the spec, not by protocol.check —
            # which now refuses to pretend it owns one.
            out.append(
                json.loads(message.payload)
                if leaf in protocol.SPEC_PAYLOADS
                else protocol.decode(message.payload, leaf)
            )
        return out


class Peer(Node):
    """The rest of the robot: the task server's half of the ROS seam.

    It receives mission commands and publishes mission statuses and health —
    all built by the same modules the real task server builds them with, so the
    bridge under test is bridging the real shapes.
    """

    def __init__(self):
        super().__init__("peer")
        self.commands = []
        self.create_subscription(
            String,
            "task/command",
            lambda m: self.commands.append(json.loads(m.data)),
            10,
        )
        self.status_pub = self.create_publisher(String, "task/status", 10)
        self.capabilities_pub = self.create_publisher(
            String, "task/capabilities", agent.LATCHED
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics_agg", 10)

    def say(self, mission_id, state, capability="goto", **kwargs):
        """A status as the task server publishes one: source local, because on
        the robot nothing can tell a fleet mission from a bench one."""
        self.status_pub.publish(
            String(
                data=json.dumps(
                    spec_mission.status(
                        "mote-01",
                        mission_id,
                        capability,
                        state,
                        source=spec_mission.SOURCE_LOCAL,
                        **kwargs,
                    )
                )
            )
        )

    def advertise(self):
        self.capabilities_pub.publish(
            String(data=json.dumps(capabilities.capability_set("mote-01")))
        )

    def report(self, level=DiagnosticStatus.OK, summary="OK"):
        array = DiagnosticArray()
        array.status = [
            DiagnosticStatus(name="mote", level=level, message=summary),
            DiagnosticStatus(name="lidar", level=DiagnosticStatus.OK, message="ok"),
        ]
        self.diag_pub.publish(array)


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    from mote_bringup import identity, sites

    from mote_fleet import fleet_config

    identity.set_identity(id="mote-01", name="Scout", site="home")
    fleet_config.save(server="http://fleet:8080", broker_host="fleet", broker_port=1883)
    # The site/floor on the wire is the *active map bundle*, not identity's
    # entitlement field: a coordinate is only meaningful against the floor it
    # was measured in.
    sites.create("home", "ground")
    return tmp_path


@pytest.fixture
def fleet(home):
    # These nodes publish task/command, and nothing else may be what receives
    # it. Two layers say so: a high DDS domain keeps a live robot or sim on this
    # machine out of the graph, and a per-process namespace keeps sibling test
    # sessions out of it — colcon runs package tests in parallel, and mote_tasks
    # stands up a real task server on the same relative topic names.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])

    from mote_fleet.agent import MoteAgent

    clients = []

    def factory(client_id):
        client = FakeMqtt(client_id)
        clients.append(client)
        return client

    agent = MoteAgent(
        client_factory=factory,
        parameter_overrides=[
            Parameter("health_period", value=0.2),
            Parameter("pose_period", value=0.2),
            Parameter("command_timeout", value=1.0),
        ],
    )
    peer = Peer()
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    executor.add_node(peer)

    def spin(seconds=0.5):
        # Wall-clock, not iterations: spin_once returns immediately whenever a
        # timer is ready, and the agent has one every 50 ms, so counting
        # iterations would let a "2 second" spin finish in milliseconds and
        # never reach a timeout under test.
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    context = SimpleNamespace(
        agent=agent, peer=peer, mqtt=clients[0], spin=spin, protocol=None
    )
    from mote_fleet import protocol

    context.protocol = protocol
    yield context

    agent.close()
    executor.shutdown()
    agent.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


# ---- connection ---------------------------------------------------------


def test_the_agent_reads_its_broker_from_the_enrolled_config(fleet):
    assert fleet.mqtt.target == ("fleet", 1883, fleet.agent.keepalive)
    assert fleet.mqtt.client_id == "mote-agent-mote-01"


def test_the_last_will_marks_the_robot_offline_and_is_retained(fleet):
    protocol = fleet.protocol
    assert fleet.mqtt.will.topic == "mote/v2/mote-01/presence"
    assert fleet.mqtt.will.retain is True
    assert (
        protocol.decode(fleet.mqtt.will.payload, protocol.PRESENCE)["online"] is False
    )


def test_connecting_subscribes_to_commands_and_announces_presence(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    assert "mote/v2/mote-01/mission/command" in fleet.mqtt.subscriptions
    presence = fleet.mqtt.last(protocol.PRESENCE)
    assert presence["online"] is True
    assert presence["robot_id"] == "mote-01"


def test_a_clean_stop_says_offline_rather_than_dying(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.agent.close()
    assert fleet.mqtt.last(protocol.PRESENCE)["online"] is False
    assert fleet.mqtt.disconnected


def test_state_topics_are_retained(fleet):
    fleet.mqtt.connect()
    fleet.peer.report()
    fleet.spin(0.5)
    for message in fleet.mqtt.published:
        assert message.retain is True, message.topic


# ---- commands -----------------------------------------------------------


def send(fleet, capability="goto", payload_input=None, command_id=None):
    protocol = fleet.protocol
    payload = spec_mission.command(
        "mote-01",
        capability,
        payload_input if payload_input is not None else {"target": "kitchen"},
        mission_id=command_id,
    )
    fleet.mqtt.deliver(
        protocol.topic("mote-01", protocol.COMMAND), json.dumps(payload).encode()
    )
    fleet.spin(0.2)
    return payload


def test_a_mission_reaches_ros_and_reports_its_transitions(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    payload = send(fleet)

    # Byte for byte: the bridge adds nothing to what the dispatcher sent.
    assert fleet.peer.commands == [payload]
    assert fleet.mqtt.last(protocol.STATUS)["state"] == spec_mission.DISPATCHED

    fleet.peer.say(payload["id"], spec_mission.ACCEPTED)
    fleet.spin(0.3)
    assert fleet.mqtt.last(protocol.STATUS)["state"] == spec_mission.ACCEPTED

    fleet.peer.say(payload["id"], spec_mission.SUCCEEDED)
    fleet.spin(0.3)
    final = fleet.mqtt.last(protocol.STATUS)
    assert final["state"] == spec_mission.SUCCEEDED
    assert final["terminal"] is True
    assert final["id"] == payload["id"]
    # The task server said "local"; only the agent knows it dispatched this one.
    assert final["source"] == spec_mission.SOURCE_FLEET


def test_a_typed_failure_is_forwarded_untouched(fleet):
    """The bridge must not re-derive a class or a recoverability: the executor
    is the only thing that knows why its own mission failed."""
    protocol = fleet.protocol
    fleet.mqtt.connect()
    payload = send(fleet)
    failure = spec_mission.failure(
        spec_mission.OBSTRUCTED, "drive_to_zone: Nav2 ended the goal with status 6"
    )
    fleet.peer.say(payload["id"], spec_mission.FAILED, failure=failure)
    fleet.spin(0.3)
    assert fleet.mqtt.last(protocol.STATUS)["failure"] == failure


def test_a_second_mission_still_reaches_the_executor(fleet):
    """The lane belongs to the task server now. The agent forwarding both is
    what lets the *robot* answer `busy` naming the mission that holds it —
    including when the holder is a local one the agent cannot see."""
    fleet.mqtt.connect()
    first = send(fleet)
    fleet.peer.say(first["id"], spec_mission.ACCEPTED)
    fleet.spin(0.3)

    second = send(fleet, payload_input={"target": "lab"})
    assert [c["id"] for c in fleet.peer.commands] == [first["id"], second["id"]]


def test_a_redelivered_command_is_not_run_twice(fleet):
    fleet.mqtt.connect()
    payload = send(fleet)
    send(fleet, command_id=payload["id"])
    assert len(fleet.peer.commands) == 1


def test_a_malformed_command_is_ignored_not_dispatched(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    topic = protocol.topic("mote-01", protocol.COMMAND)
    fleet.mqtt.deliver(topic, b"{not json")
    fleet.mqtt.deliver(topic, json.dumps({"schema": 1, "id": "x"}).encode())
    # Addressed to another platform: forwarding it would run someone else's
    # mission on this robot.
    stray = spec_mission.command("mote-02", "goto", {"target": "kitchen"})
    fleet.mqtt.deliver(topic, json.dumps(stray).encode())
    fleet.spin(0.2)
    assert fleet.peer.commands == []


def test_a_locally_issued_mission_is_reported_with_no_correlation_id(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.say("bench-7", spec_mission.ACCEPTED)
    fleet.spin(0.3)
    status = fleet.mqtt.last(protocol.STATUS)
    assert status["source"] == spec_mission.SOURCE_LOCAL


def test_an_unanswered_mission_fails_on_its_own(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    send(fleet)
    fleet.spin(2.0)  # command_timeout is 1.0s in this fixture
    status = fleet.mqtt.last(protocol.STATUS)
    assert status["state"] == spec_mission.FAILED
    assert status["failure"]["class"] == spec_mission.TIMEOUT


# ---- capabilities --------------------------------------------------------


def test_the_capability_set_is_forwarded_retained(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.advertise()
    fleet.spin(0.3)
    document = fleet.mqtt.last(protocol.CAPABILITIES)
    assert [item["key"] for item in document["capabilities"]] == ["goto", "fetch"]
    assert all(
        message.retain
        for message in fleet.mqtt.published
        if message.topic.endswith(protocol.CAPABILITIES)
    )


def test_a_robot_that_has_advertised_nothing_publishes_nothing(fleet):
    """Forwarded, never authored: a robot whose task server is down offers no
    capabilities, and that is the true answer rather than a stale one."""
    fleet.mqtt.connect()
    fleet.spin(0.3)
    assert fleet.mqtt.last(fleet.protocol.CAPABILITIES) is None


# ---- health -------------------------------------------------------------


def test_health_is_unknown_until_the_monitor_reports(fleet):
    payload = fleet.agent.health_payload()
    assert payload["state"] == fleet.protocol.UNKNOWN
    assert "no diagnostics" in payload["summary"]


def test_health_forwards_the_monitors_rollup_and_subsystems(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.report(level=DiagnosticStatus.WARN, summary="DEGRADED: camera stale")
    fleet.spin(0.5)

    health = fleet.mqtt.last(protocol.HEALTH)
    assert health["state"] == protocol.DEGRADED
    assert health["summary"] == "DEGRADED: camera stale"
    assert [s["name"] for s in health["subsystems"]] == ["lidar"]
    assert health["site"] == "home"
    assert health["battery"] is None


def test_health_carries_the_mission_the_robot_is_busy_with(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.report()
    payload = send(fleet, "fetch", {"target": "red_box", "destination": "dropoff"})
    fleet.peer.say(payload["id"], spec_mission.ACCEPTED, capability="fetch")
    fleet.spin(0.5)

    running = fleet.mqtt.last(protocol.HEALTH)["mission"]
    assert running["capability"] == "fetch"
    assert running["state"] == spec_mission.ACCEPTED
    assert running["lane"] == spec_mission.DEFAULT_LANE


def test_health_goes_unknown_again_if_the_monitor_stops(fleet):
    fleet.agent.diagnostics_timeout = 0.1
    fleet.mqtt.connect()
    fleet.peer.report()
    fleet.spin(0.3)
    fleet.spin(0.5)
    assert fleet.agent.health_payload()["state"] == fleet.protocol.UNKNOWN
    assert "stopped reporting" in fleet.agent.health_payload()["summary"]


# ---- pose ---------------------------------------------------------------


def test_pose_is_not_published_before_the_robot_is_localised(fleet):
    fleet.mqtt.connect()
    fleet.spin(0.5)
    assert fleet.mqtt.last(fleet.protocol.POSE) is None


def test_pose_follows_the_map_frame_transform(fleet):
    import math

    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    protocol = fleet.protocol
    broadcaster = TransformBroadcaster(fleet.peer)
    tf = TransformStamped()
    tf.header.frame_id = "map"
    tf.child_frame_id = "base_link"
    tf.transform.translation.x = 1.5
    tf.transform.translation.y = -2.25
    tf.transform.rotation.z = math.sin(math.pi / 4)
    tf.transform.rotation.w = math.cos(math.pi / 4)

    fleet.mqtt.connect()
    for _ in range(10):
        tf.header.stamp = fleet.peer.get_clock().now().to_msg()
        broadcaster.sendTransform(tf)
        fleet.spin(0.1)

    pose = fleet.mqtt.last(protocol.POSE)
    assert pose is not None
    assert (pose["x"], pose["y"]) == (1.5, -2.25)
    assert pose["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    # The coordinate travels with the floor whose map frame it is measured in.
    assert (pose["site"], pose["floor"]) == ("home", "ground")


# ---- the map registry ---------------------------------------------------


def announce(fleet, revision, site="home", floor="ground", **extra):
    """Deliver a retained ``current`` announcement, as the broker would on
    connect."""
    protocol = fleet.protocol
    payload = protocol.current(
        site,
        floor,
        revision,
        url=f"/v1/sites/{site}/floors/{floor}/revisions/{revision}/bundle.tar.gz",
        **extra,
    )
    fleet.mqtt.deliver(protocol.registry_topic(site, floor), protocol.encode(payload))
    fleet.spin(0.3)
    return payload


def test_connecting_subscribes_to_every_floors_canonical_revision(fleet):
    fleet.mqtt.connect()
    assert fleet.protocol.any_floor() in fleet.mqtt.subscriptions


def test_an_announcement_for_this_robots_floor_is_pulled(fleet, monkeypatch):
    """The agent does not fetch on the paho thread — it hands the work to its
    worker — so what is asserted is that the pull happened with the announced
    revision, not how it got there."""
    from mote_fleet import mapsync

    pulled = []

    def fake_pull(server, announcement, **kwargs):
        pulled.append((server, announcement["revision"]))
        return {
            "action": "installed",
            "site": announcement["site"],
            "floor": announcement["floor"],
            "revision": announcement["revision"],
        }

    monkeypatch.setattr(mapsync, "pull", fake_pull)
    fleet.mqtt.connect()
    announce(fleet, "20260727T101500")
    fleet.spin(0.5)
    assert pulled == [("http://fleet:8080", "20260727T101500")]


def test_a_floor_this_robot_has_never_been_on_is_ignored(fleet, monkeypatch):
    from mote_fleet import mapsync

    monkeypatch.setattr(
        mapsync, "pull", lambda *a, **k: pytest.fail("should not have pulled")
    )
    fleet.mqtt.connect()
    announce(fleet, "20260727T101500", site="warehouse", floor="mezzanine")
    fleet.spin(0.3)


def test_an_announcement_of_the_revision_already_running_pulls_nothing(
    fleet, monkeypatch
):
    """Retained means this arrives on every reconnect, so the common case is
    an announcement of the map the robot is already on."""
    from mote_bringup import sites

    from mote_fleet import mapsync

    floor_dir = sites.floor_dir("home", "ground")
    (floor_dir / "maps" / "20260727T101500").mkdir(parents=True)
    sites._publish_revision(floor_dir, "20260727T101500")
    monkeypatch.setattr(
        mapsync, "pull", lambda *a, **k: pytest.fail("should not have pulled")
    )
    fleet.mqtt.connect()
    announce(fleet, "20260727T101500")
    fleet.spin(0.3)


def test_health_reports_the_map_revision_this_robot_is_running(fleet):
    from mote_bringup import sites

    floor_dir = sites.floor_dir("home", "ground")
    (floor_dir / "maps" / "20260727T101500").mkdir(parents=True)
    sites._publish_revision(floor_dir, "20260727T101500")
    fleet.mqtt.connect()
    fleet.spin(0.4)
    reported = fleet.mqtt.last(fleet.protocol.HEALTH)["map"]
    assert reported == {
        "site": "home",
        "floor": "ground",
        "revision": "20260727T101500",
    }


def test_health_reports_no_revision_when_the_floor_has_no_map(fleet):
    fleet.mqtt.connect()
    fleet.spin(0.4)
    assert fleet.mqtt.last(fleet.protocol.HEALTH)["map"]["revision"] is None


def test_a_malformed_announcement_is_ignored(fleet, monkeypatch):
    from mote_fleet import mapsync

    monkeypatch.setattr(
        mapsync, "pull", lambda *a, **k: pytest.fail("should not have pulled")
    )
    fleet.mqtt.connect()
    fleet.mqtt.deliver(
        fleet.protocol.registry_topic("home", "ground"), b'{"schema": 99}'
    )
    fleet.spin(0.3)

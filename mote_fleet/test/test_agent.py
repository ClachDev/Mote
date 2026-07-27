"""The agent's bridge, driven against a fake broker.

No mosquitto here: the MQTT client is injected, so every path — LWT, retained
presence, command in, status out, health from diagnostics — is exercised as
plain ROS spinning plus method calls. That keeps the bridge under test on both
CI architectures, where a broker is not installed. The version with a real
broker and a real behaviour tree is ``test_e2e_fleet.py``.
"""

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
        from mote_fleet import protocol

        for message in reversed(self.published):
            parsed = protocol.parse_topic(message.topic)
            if parsed and parsed[1] == leaf:
                return protocol.decode(message.payload, leaf)
        return None

    def all_of(self, leaf):
        from mote_fleet import protocol

        out = []
        for message in self.published:
            parsed = protocol.parse_topic(message.topic)
            if parsed and parsed[1] == leaf:
                out.append(protocol.decode(message.payload, leaf))
        return out


class Peer(Node):
    """The rest of the robot: receives commands, publishes status + health."""

    def __init__(self):
        super().__init__("peer")
        self.commands = []
        self.create_subscription(
            String, "task/command", lambda m: self.commands.append(m.data), 10
        )
        self.status_pub = self.create_publisher(String, "task/status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics_agg", 10)

    def say(self, text):
        self.status_pub.publish(String(data=text))

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
    assert fleet.mqtt.will.topic == "mote/v1/mote-01/presence"
    assert fleet.mqtt.will.retain is True
    assert (
        protocol.decode(fleet.mqtt.will.payload, protocol.PRESENCE)["online"] is False
    )


def test_connecting_subscribes_to_commands_and_announces_presence(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    assert "mote/v1/mote-01/task/command" in fleet.mqtt.subscriptions
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


def send(fleet, text, command_id=None):
    protocol = fleet.protocol
    payload = protocol.command(text, command_id=command_id)
    fleet.mqtt.deliver(
        protocol.topic("mote-01", protocol.COMMAND), protocol.encode(payload)
    )
    fleet.spin(0.2)
    return payload


def test_a_command_reaches_ros_and_reports_its_transitions(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    payload = send(fleet, "goto kitchen")

    assert fleet.peer.commands == ["goto kitchen"]
    assert fleet.mqtt.last(protocol.STATUS)["state"] == protocol.DISPATCHED

    fleet.peer.say("accepted: goto kitchen")
    fleet.spin(0.3)
    assert fleet.mqtt.last(protocol.STATUS)["state"] == protocol.ACCEPTED

    fleet.peer.say("succeeded: goto kitchen")
    fleet.spin(0.3)
    final = fleet.mqtt.last(protocol.STATUS)
    assert final["state"] == protocol.SUCCEEDED
    assert final["terminal"] is True
    assert final["id"] == payload["id"]
    assert final["source"] == protocol.SOURCE_FLEET


def test_a_second_command_never_reaches_ros(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    send(fleet, "goto kitchen")
    fleet.peer.say("accepted: goto kitchen")
    fleet.spin(0.3)

    send(fleet, "goto lab")
    assert fleet.peer.commands == ["goto kitchen"]
    rejection = fleet.mqtt.last(protocol.STATUS)
    assert rejection["state"] == protocol.REJECTED
    assert "goto kitchen" in rejection["detail"]


def test_a_redelivered_command_is_not_run_twice(fleet):
    fleet.mqtt.connect()
    payload = send(fleet, "goto kitchen")
    send(fleet, "goto kitchen", command_id=payload["id"])
    assert fleet.peer.commands == ["goto kitchen"]


def test_a_malformed_command_is_ignored_not_dispatched(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    topic = protocol.topic("mote-01", protocol.COMMAND)
    fleet.mqtt.deliver(topic, b"{not json")
    fleet.mqtt.deliver(topic, protocol.encode({"schema": 1, "id": "x"}))
    fleet.mqtt.deliver(topic, protocol.encode(protocol.command("   ")))
    fleet.spin(0.2)
    assert fleet.peer.commands == []


def test_a_locally_issued_task_is_reported_with_no_correlation_id(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.say("accepted: goto lab")
    fleet.spin(0.3)
    status = fleet.mqtt.last(protocol.STATUS)
    assert status["source"] == protocol.SOURCE_LOCAL
    assert status["id"] is None


def test_an_unanswered_command_fails_on_its_own(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    send(fleet, "goto kitchen")
    fleet.spin(2.0)  # command_timeout is 1.0s in this fixture
    status = fleet.mqtt.last(protocol.STATUS)
    assert status["state"] == protocol.FAILED
    assert "no verdict" in status["detail"]


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


def test_health_carries_the_task_the_robot_is_busy_with(fleet):
    protocol = fleet.protocol
    fleet.mqtt.connect()
    fleet.peer.report()
    send(fleet, "fetch red_box dropoff")
    fleet.peer.say("accepted: fetch red_box dropoff")
    fleet.spin(0.5)

    task = fleet.mqtt.last(protocol.HEALTH)["task"]
    assert task["command"] == "fetch red_box dropoff"
    assert task["state"] == protocol.ACCEPTED


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

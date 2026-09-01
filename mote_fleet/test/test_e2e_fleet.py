"""M1's acceptance test: enroll a robot, then dispatch a task to it over MQTT.

Everything real except the wheels — a mosquitto broker, the enrollment endpoint
and its SQLite registry, the ``enroll`` CLI, the agent with a genuine paho
client, and the actual ``mote_tasks`` behaviour tree driving a mock
``navigate_to_pose``. What is being proved is that the seams line up: that a
robot which has never heard of the fleet can be given an identity by the server,
that a mission published to ``mote/v2/<robot_id>/mission/command`` from a
process which shares no ROS graph with it turns into a Nav2 goal, that the
status transitions come back with the correlation id they went out with, and
that the refusals come back *typed* — which is the half of mission/v0 a unit
test cannot prove, because it is the seam between the executor's classification
and the bridge's forwarding.

The one thing it cannot prove is "off-LAN", which is a property of the tailnet
(M0) rather than of this code: the same MQTT connection over a WireGuard
interface is the same connection. ``docs/fleet/m1-verification.md`` records the
run that closes that half.

It runs wherever both a broker and ROS exist — the dev environment, and
``pixi run -e dev test-fleet``. In CI's robot environment there is no broker, so
it skips and the fake-client tests in ``test_agent.py`` carry the coverage.
"""

import json
import os
import random
import threading
import time

import pytest

# Importing the harness first is deliberate: it skips this module when rclpy,
# paho or a broker binary is missing, before the ROS imports below would fail.
from fleet_harness import (
    ZONES,
    Broker,
    MockNav,
    Operator,
    needs_broker,
    spin_until,
)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from mote_bringup.spec import mission  # noqa: E402

pytestmark = needs_broker


@pytest.fixture
def broker(tmp_path):
    server = Broker(tmp_path).start()
    yield server.port
    server.stop()


@pytest.fixture
def fleet_api(tmp_path, broker):
    from fleet_server import serve

    server = serve(
        db=tmp_path / "registry.db",
        host="127.0.0.1",
        port=0,
        broker_host="127.0.0.1",
        broker_port=broker,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    server.url = f"http://127.0.0.1:{server.server_address[1]}"
    yield server
    server.shutdown()
    server.server_close()


def test_enroll_then_dispatch_over_mqtt(tmp_path, monkeypatch, broker, fleet_api):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "mote"))
    # A private DDS domain: this test publishes task/command, and a live robot
    # or sim on this machine must never be what receives it.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))

    from mote_bringup import identity

    from mote_fleet import enroll, fleet_config, protocol

    # ---- enrollment: the robot has no identity at all yet ----
    assert identity.load() is None
    token = fleet_api.registry.new_token(single_use=True, note="e2e")
    enroll.main(["--server", fleet_api.url, "--token", token, "--name", "Scout"])

    robot_id = identity.robot_id()
    assert robot_id == "mote-01"
    assert fleet_config.broker() == ("127.0.0.1", broker)
    # Idempotent: the same machine enrolling again is the same robot.
    enroll.main(["--token", token])
    assert identity.robot_id() == "mote-01"
    assert len(fleet_api.registry.robots()) == 1

    # ---- the robot comes up ----
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)

    from mote_fleet.agent import MoteAgent
    from mote_tasks.task_server import TaskServer

    agent = MoteAgent(
        parameter_overrides=[
            Parameter("health_period", value=0.5),
            Parameter("pose_period", value=0.5),
            Parameter("keepalive", value=2),
        ]
    )
    tasks = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("tick_period", value=0.05),
            # The pick/place stubs stand in for the arm; shorten them so the
            # fetch mission below runs in test time rather than in robot time.
            Parameter("pick_duration", value=0.2),
            Parameter("place_duration", value=0.2),
        ]
    )
    nav = MockNav()
    executor = SingleThreadedExecutor()
    for node in (agent, tasks, nav):
        executor.add_node(node)

    operator = Operator(broker)
    try:
        assert spin_until(executor, lambda: agent.connected), "agent never connected"

        # ---- presence and health, retained, without being asked ----
        assert spin_until(executor, lambda: operator.of("presence")), "no presence"
        assert operator.of("presence")[-1]["online"] is True
        assert spin_until(executor, lambda: operator.of("health")), "no health"
        health = operator.of("health")[-1]
        assert health["robot_id"] == robot_id
        # No health monitor is running in this test, and the agent says so
        # rather than claiming the robot is fine.
        assert health["state"] == protocol.UNKNOWN

        # ---- what the robot says it can be asked to do, before anyone asks ----
        assert spin_until(
            executor, lambda: operator.capabilities(robot_id) is not None
        ), "the capability set never reached the broker"
        offered = operator.capabilities(robot_id)
        assert [item["key"] for item in offered["capabilities"]] == ["goto", "fetch"]
        assert offered["platform_id"] == robot_id

        # ---- the milestone's acceptance: dispatch a fetch, off the robot's
        # ROS graph entirely, and watch the transitions come back ----
        command_id = operator.dispatch(
            robot_id, "fetch", {"target": "lab", "destination": "kitchen"}
        )
        assert spin_until(
            executor,
            lambda: any(s["terminal"] for s in operator.statuses(command_id)),
            timeout=60.0,
        ), operator.statuses(command_id)

        states = [s["state"] for s in operator.statuses(command_id)]
        assert states == [
            mission.DISPATCHED,
            mission.ACCEPTED,
            mission.SUCCEEDED,
        ], states
        # The task server publishes `local` — it cannot tell a fleet mission
        # from a bench one — and the agent rewrites it for the ids it sent.
        assert all(
            s["source"] == mission.SOURCE_FLEET for s in operator.statuses(command_id)
        )

        # ...and it really ran the mission: drive to the object zone, then to
        # the drop zone, with the pick/place stubs in between.
        assert len(nav.goals) == 2, nav.goals
        assert nav.goals[0].pose.position.x == pytest.approx(4.0)  # lab
        assert nav.goals[1].pose.position.x == pytest.approx(-1.5)  # kitchen

        # ---- and a goto, the other capability ----
        goto_id = operator.dispatch(robot_id, "goto", {"target": "kitchen"})
        assert spin_until(
            executor, lambda: any(s["terminal"] for s in operator.statuses(goto_id))
        ), operator.statuses(goto_id)
        assert operator.statuses(goto_id)[-1]["state"] == mission.SUCCEEDED
        assert len(nav.goals) == 3

        # ---- a rejection is typed: the class is what a dispatcher acts on ----
        bad_id = operator.dispatch(robot_id, "goto", {"target": "nowhere"})
        assert spin_until(
            executor, lambda: any(s["terminal"] for s in operator.statuses(bad_id))
        ), operator.statuses(bad_id)
        rejection = operator.statuses(bad_id)[-1]
        assert rejection["state"] == mission.REJECTED
        assert rejection["failure"]["class"] == mission.UNRESOLVED_ZONE
        assert rejection["failure"]["recoverable"] is False
        assert "nowhere" in rejection["failure"]["detail"]
        assert len(nav.goals) == 3  # nothing was driven

        # ...and so is an input the capability's own schema refuses.
        malformed_id = operator.dispatch(robot_id, "goto", {"where": "kitchen"})
        assert spin_until(
            executor,
            lambda: any(s["terminal"] for s in operator.statuses(malformed_id)),
        ), operator.statuses(malformed_id)
        assert (
            operator.statuses(malformed_id)[-1]["failure"]["class"]
            == mission.INVALID_INPUT
        )
        assert len(nav.goals) == 3

        # ---- health reports the mission while it runs ----
        slow_id = operator.dispatch(
            robot_id, "fetch", {"target": "red_box", "destination": "kitchen"}
        )
        assert spin_until(
            executor,
            lambda: any(
                s["state"] == mission.ACCEPTED for s in operator.statuses(slow_id)
            ),
        ), operator.statuses(slow_id)
        assert spin_until(
            executor,
            lambda: (
                (operator.of("health")[-1].get("mission") or {}).get("id") == slow_id
            ),
        ), operator.of("health")[-1]

        # ---- and a second mission is refused while the lane is held ----
        busy_id = operator.dispatch(robot_id, "goto", {"target": "lab"})
        # For a *terminal* status, not merely any: the agent publishes
        # `dispatched` as it forwards, and the executor's refusal follows it
        # milliseconds later. Waiting on the first status arriving catches the
        # `dispatched` most of the time and the `rejected` sometimes, which is
        # a test that passes on how fast the machine is.
        assert spin_until(
            executor,
            lambda: any(s["terminal"] for s in operator.statuses(busy_id)),
        ), "no verdict on the second mission"
        busy = operator.statuses(busy_id)[-1]
        assert busy["state"] == mission.REJECTED
        assert busy["failure"]["class"] == mission.BUSY
        assert busy["failure"]["recoverable"] is True
        # It names the mission holding the lane, so a dispatcher knows what it
        # is waiting for rather than only that it must wait.
        assert slow_id in busy["failure"]["detail"]
    finally:
        agent.close()
        operator.close()
        executor.shutdown()
        for node in (agent, tasks, nav):
            node.destroy_node()
        rclpy.shutdown()


def test_a_dead_agent_is_reported_offline_by_the_broker(
    tmp_path, monkeypatch, broker, fleet_api
):
    """The Last Will, for real: kill the connection, do not close it.

    This is the claim that a robot losing power is noticed within the keepalive
    rather than whenever somebody notices the heartbeats stopped — so it is
    worth testing against a broker rather than against the agent's own belief
    about what it registered.
    """
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "mote"))
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))

    from mote_bringup import identity

    from mote_fleet import enroll

    token = fleet_api.registry.new_token()
    enroll.main(["--server", fleet_api.url, "--token", token])
    robot_id = identity.robot_id()

    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    from mote_fleet.agent import MoteAgent

    agent = MoteAgent(parameter_overrides=[Parameter("keepalive", value=2)])
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    operator = Operator(broker)
    try:
        assert spin_until(executor, lambda: agent.connected)
        assert spin_until(executor, lambda: operator.of("presence"))
        assert operator.of("presence")[-1]["online"] is True

        # Stop the network loop first so paho cannot reconnect, then drop the
        # socket without a DISCONNECT — what a power cut looks like to a broker.
        agent.client.loop_stop()
        agent.client._sock.close()

        assert spin_until(
            executor,
            lambda: operator.of("presence")[-1]["online"] is False,
            timeout=30.0,
        ), operator.of("presence")
        assert operator.of("presence")[-1]["reason"] == "last will"

        # And a subscriber arriving afterwards is told immediately, because the
        # will was retained.
        latecomer = Operator(broker)
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and not latecomer.of("presence"):
                time.sleep(0.1)
            assert latecomer.of("presence")[-1]["online"] is False
            assert latecomer.of("presence")[-1]["robot_id"] == robot_id
        finally:
            latecomer.close()
    finally:
        agent.client = None  # already dead; skip close()'s disconnect
        operator.close()
        executor.shutdown()
        agent.destroy_node()
        rclpy.shutdown()


def test_the_registry_survives_a_server_restart(tmp_path, broker):
    """Enrollment is durable: a fleet server that restarts still knows who is
    in the fleet, because the row store is a file rather than memory."""
    from fleet_server import serve

    first = serve(db=tmp_path / "registry.db", host="127.0.0.1", port=0)
    token = first.registry.new_token(single_use=False)
    first.registry.enroll(token=token, fingerprint="serial:aaa", name="Scout")
    first.server_close()

    second = serve(db=tmp_path / "registry.db", host="127.0.0.1", port=0)
    try:
        assert [r["robot_id"] for r in second.registry.robots()] == ["mote-01"]
        assert second.registry.robot("mote-01")["name"] == "Scout"
    finally:
        second.server_close()


def api_post(url, path, payload, token=""):
    import urllib.error
    import urllib.request

    request = urllib.request.Request(
        url + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_dispatch_through_the_fleet_api(tmp_path, monkeypatch, broker, fleet_api):
    """M3's acceptance: the *only* write path is the API.

    Everything real again — a mosquitto broker, the fleet server with its own
    paho client, the agent, and the `mote_tasks` tree — but the command is not
    published by the test. It is POSTed to `/v1/robots/<id>/dispatch`, which
    authorizes an operator token, writes the audit row, and publishes. The
    browser holds no credential that can reach the broker; this proves the same
    loop still closes when the write goes through the server (fleet.md Q5/Q7).
    """
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "mote"))
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))

    from mote_bringup import identity

    from mote_fleet import enroll

    enroll.main(["--server", fleet_api.url, "--token", fleet_api.registry.new_token()])
    robot_id = identity.robot_id()
    operator_token = fleet_api.registry.new_operator(name="michael")

    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)

    from mote_fleet.agent import MoteAgent
    from mote_tasks.task_server import TaskServer

    agent = MoteAgent(parameter_overrides=[Parameter("keepalive", value=2)])
    tasks = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("tick_period", value=0.05),
        ]
    )
    nav = MockNav()
    executor = SingleThreadedExecutor()
    for node in (agent, tasks, nav):
        executor.add_node(node)

    operator = Operator(broker)
    try:
        assert spin_until(executor, lambda: agent.connected), "agent never connected"

        # ---- a request with no operator token reaches no robot ----
        code, body = api_post(
            fleet_api.url,
            f"/v1/robots/{robot_id}/dispatch",
            {"capability": "goto", "input": {"target": "lab"}},
        )
        assert code == 401, body
        assert not any(
            topic.endswith("mission/command") for topic, _ in operator.messages
        )

        # ---- with one, it is audited and published ----
        code, answer = api_post(
            fleet_api.url,
            f"/v1/robots/{robot_id}/dispatch",
            {
                "schema": mission.SCHEMA,
                "capability": "goto",
                "input": {"target": "kitchen"},
            },
            token=operator_token,
        )
        assert code == 202, answer

        entry = fleet_api.registry.audit()[0]
        assert (entry["actor"], entry["result"]) == ("michael", "published")
        assert entry["command_id"] == answer["id"]

        # ---- and the robot really ran it, answering on the same id ----
        assert spin_until(
            executor,
            lambda: any(s["terminal"] for s in operator.statuses(answer["id"])),
            timeout=60.0,
        ), operator.statuses(answer["id"])
        assert [s["state"] for s in operator.statuses(answer["id"])] == [
            mission.DISPATCHED,
            mission.ACCEPTED,
            mission.SUCCEEDED,
        ]
        assert len(nav.goals) == 1
        assert nav.goals[0].pose.position.x == pytest.approx(-1.5)  # kitchen
    finally:
        agent.close()
        operator.close()
        fleet_api.publisher.close()
        executor.shutdown()
        for node in (agent, tasks, nav):
            node.destroy_node()
        rclpy.shutdown()

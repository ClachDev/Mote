"""M4's acceptance test: publish a map, promote it, watch a robot pull it.

Everything real except SLAM — a mosquitto broker, the fleet API and its registry,
the ``publish-map`` CLI, ``fleetctl promote``, and an agent with a genuine paho
client subscribed to the retained registry topic. What is being proved is the
loop the milestone is: a revision a robot saved locally becomes a *candidate*
that changes nothing, an operator's promotion flips the floor and announces it,
and a second robot — a different ``MOTE_HOME``, sharing no ROS graph and no
filesystem with the first — ends up with that revision staged and published.

The retained half is proved the way it matters: the second robot's agent is
started **after** the promotion, so the only thing that can have told it is the
broker handing over a retained message on connect.

Like ``test_e2e_fleet.py`` this needs both a broker and ROS, so it runs in the
dev environment and skips elsewhere, where ``test_map_registry.py`` and
``test_mapsync.py`` carry the coverage.
"""

import os
import random
import threading
import time

import pytest

# Importing the harness first is deliberate: it skips this module when rclpy,
# paho or a broker binary is missing, before the ROS imports below would fail.
from api_harness import write_revision
from fleet_harness import Broker, needs_broker, spin_until

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

pytestmark = needs_broker

SITE, FLOOR = "home", "ground"
REVISION = "20260728T090412"


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
        maps_dir=tmp_path / "fleet-sites",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    server.url = f"http://127.0.0.1:{server.server_address[1]}"
    server.token = server.registry.new_operator(name="e2e")
    yield server
    server.shutdown()
    server.server_close()
    server.publisher.close()


def test_publish_promote_and_pull(tmp_path, monkeypatch, broker, fleet_api, capsys):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))
    from mote_bringup import bundle, identity, sites

    from mote_fleet import enroll

    fleetctl = _fleetctl()

    # ---- robot one: enrol, "map" a floor, publish it ----
    mapper_home = tmp_path / "mapper"
    monkeypatch.setenv("MOTE_HOME", str(mapper_home))
    enroll.main(
        [
            "--server",
            fleet_api.url,
            "--token",
            fleet_api.registry.new_token(note="e2e"),
            "--name",
            "Mapper",
        ]
    )
    assert identity.robot_id() == "mote-01"
    sites.create(SITE, FLOOR)
    floor_dir = sites.floor_dir(SITE, FLOOR)
    write_revision(floor_dir / "maps" / REVISION, zones=False)
    (floor_dir / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  kitchen: {x: 1.0, y: 2.0, yaw: 0.0, radius: 1.5}\n"
    )
    sites._publish_revision(floor_dir, REVISION)

    from mote_fleet import publish

    publish.main([])
    published = capsys.readouterr().out
    assert f"published {SITE}/{FLOOR}/{REVISION}" in published
    # A candidate changes nothing: the floor has no canonical map at all yet.
    assert "still on no published map" in published

    floor = _get(fleet_api, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] is None
    assert [r["revision"] for r in floor["revisions"]] == [REVISION]
    # The floor's zones travelled inside the revision, in its own map frame.
    assert floor["revisions"][0]["zones"] == ["kitchen"]

    # ---- the operator promotes it ----
    operator_token = fleet_api.registry.new_operator(name="michael")
    fleetctl(
        [
            "--server",
            fleet_api.url,
            "--token",
            operator_token,
            "promote",
            SITE,
            FLOOR,
            REVISION,
        ]
    )
    assert "announced on" in capsys.readouterr().out
    assert _get(fleet_api, f"/v1/sites/{SITE}/floors/{FLOOR}")["canonical"] == REVISION

    # ---- robot two: a different machine, started after the promotion ----
    # One process is standing in for two robots, so the hardware fingerprint
    # has to differ — enrollment is idempotent on it, which is exactly what
    # would otherwise hand this "robot" mote-01's identity back.
    from mote_fleet import facts

    monkeypatch.setattr(facts, "fingerprint", lambda collected: "serial:puller")
    puller_home = tmp_path / "puller"
    monkeypatch.setenv("MOTE_HOME", str(puller_home))
    enroll.main(
        [
            "--server",
            fleet_api.url,
            "--token",
            fleet_api.registry.new_token(note="e2e"),
            "--name",
            "Puller",
        ]
    )
    assert identity.robot_id() == "mote-02"
    sites.create(SITE, FLOOR)  # it is on that floor, and has no map for it
    assert sites.resolve_map() == ""

    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    from mote_fleet.agent import MoteAgent

    agent = MoteAgent(
        parameter_overrides=[
            Parameter("health_period", value=0.5),
            Parameter("pose_period", value=0.5),
            Parameter("keepalive", value=2),
        ]
    )
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    try:
        # Nothing tells this agent about the map except a retained message
        # handed over on connect — it was not running when the promotion
        # happened, and it never polls.
        started = time.monotonic()
        assert spin_until(
            executor,
            lambda: sites.current_revision(sites.floor_dir(SITE, FLOOR)) == REVISION,
            timeout=30.0,
        ), "the agent did not pull the canonical revision"
        elapsed = time.monotonic() - started

        pulled = sites.floor_dir(SITE, FLOOR) / "maps" / REVISION
        assert bundle.validate(pulled, require_posegraph=False).ok
        assert sites.resolve_map()  # nav2_launch.py would now find a map
        # Zones arrived with it, in the frame they were bound in.
        zones = bundle.read_zones(sites.floor_dir(SITE, FLOOR) / "zones.yaml")
        assert list(zones["zones"]) == ["kitchen"]
        # And the fleet can see which revision this robot is actually running.
        assert spin_until(
            executor,
            lambda: (
                (agent.health_payload().get("map") or {}).get("revision") == REVISION
            ),
            timeout=10.0,
        )
        print(f"\npulled {REVISION} in {elapsed:.2f}s after the agent connected")
    finally:
        agent.close()
        executor.shutdown()
        agent.destroy_node()
        rclpy.shutdown()


def _get(server, path):
    """A read of the live server. Every ``/v1`` route needs an operator, so the
    harness's own token comes along."""
    import json
    import urllib.request

    request = urllib.request.Request(server.url + path)
    request.add_header("Authorization", f"Bearer {server.token}")
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.loads(response.read())


def _fleetctl():
    import fleetctl

    return fleetctl.main


def test_a_promotion_reaches_the_map_nav_is_serving(
    tmp_path, monkeypatch, broker, fleet_api, capsys
):
    """Promote → pull → install → load, with the fleet watching health.

    mote-01, 2026-09-21: a promoted revision was installed and Nav2 kept
    serving the old one until someone restarted it, while health — reading the
    symlink — said the new one. Here the robot's lane is held busy across the
    promotion, so the gap is visible on the wire (``installed`` new, served
    ``revision`` old), and then released so the load happens with no restart.
    """
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))
    from fleet_harness import Operator
    from nav2_msgs.srv import LoadMap
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    from mote_bringup import map_reload, sites
    from mote_bringup.map_reloader import MapReloader

    from mote_fleet import enroll, mapsync

    fleetctl = _fleetctl()
    second = "20260921T133157"

    # ---- a mapper offers two revisions; the first is promoted ----
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "mapper"))
    enroll.main(
        [
            "--server",
            fleet_api.url,
            "--token",
            fleet_api.registry.new_token(note="e2e"),
        ]
    )
    sites.create(SITE, FLOOR)
    mapper_floor = sites.floor_dir(SITE, FLOOR)
    for rev in (REVISION, second):
        write_revision(mapper_floor / "maps" / rev, zones=False)
        mapsync.publish(fleet_api.url, SITE, FLOOR, rev, "mote-01")
    operator_token = fleet_api.registry.new_operator(name="michael")

    def promote(rev):
        fleetctl(
            [
                "--server",
                fleet_api.url,
                "--token",
                operator_token,
                "promote",
                SITE,
                FLOOR,
                rev,
            ]
        )

    promote(REVISION)

    # ---- the robot: agent, map_reloader, and Nav2's map_server faked ----
    from mote_fleet import facts

    monkeypatch.setattr(facts, "fingerprint", lambda collected: "serial:driver")
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "driver"))
    enroll.main(
        [
            "--server",
            fleet_api.url,
            "--token",
            fleet_api.registry.new_token(note="e2e"),
        ]
    )
    robot_id = "mote-02"
    sites.create(SITE, FLOOR)
    floor = sites.floor_dir(SITE, FLOOR)

    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    from mote_fleet.agent import MoteAgent

    class Nav(Node):
        """map_server's load_map and the task server's lane."""

        def __init__(self):
            super().__init__("nav")
            self.loads = []
            self.busy = True
            self.create_service(LoadMap, "map_server/load_map", self._load)
            self.create_service(Trigger, map_reload.IDLE_SERVICE, self._idle)

        def _load(self, request, response):
            self.loads.append(request.map_url)
            response.result = LoadMap.Response.RESULT_SUCCESS
            return response

        def _idle(self, _request, response):
            response.success = not self.busy
            response.message = "mission m-1 (goto) holds the default lane"
            return response

    agent = MoteAgent(
        parameter_overrides=[
            Parameter("health_period", value=0.5),
            Parameter("keepalive", value=2),
        ]
    )
    nav = Nav()
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    executor.add_node(nav)
    operator = Operator(broker, client_id="watcher")
    reloader = None

    def health_map():
        for topic, payload in reversed(operator.messages):
            if topic == f"mote/v2/{robot_id}/health":
                return payload.get("map") or {}
        return {}

    try:
        assert spin_until(
            executor, lambda: sites.current_revision(floor) == REVISION, timeout=30.0
        ), "the agent did not pull the first canonical revision"
        # Nav2 comes up on the first revision, as a bringup would.
        reloader = MapReloader(
            parameter_overrides=[
                Parameter("map", value=str(floor / "map" / "map.yaml")),
                Parameter("poll_period", value=0.2),
            ]
        )
        executor.add_node(reloader)
        assert spin_until(
            executor, lambda: health_map().get("revision") == REVISION, timeout=10.0
        )

        promote(second)
        assert spin_until(
            executor,
            lambda: health_map().get("installed") == second,
            timeout=30.0,
        ), health_map()
        # Installed, not served: the mission in flight holds the lane.
        assert health_map()["revision"] == REVISION
        assert nav.loads == []

        nav.busy = False
        assert spin_until(
            executor, lambda: health_map().get("revision") == second, timeout=10.0
        ), health_map()
        assert nav.loads == [str(floor / "maps" / second / "map.yaml")]
        assert health_map()["error"] is None
    finally:
        operator.close()
        if reloader is not None:
            reloader.stop()
            reloader.destroy_node()
        agent.close()
        executor.shutdown()
        agent.destroy_node()
        nav.destroy_node()
        rclpy.shutdown()

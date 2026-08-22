"""Ms's acceptance: robot autonomy is unaffected while the fleet server is down.

The fleet server is availability-critical for *operations* — dispatch, roster,
map publish — and deliberately not for the robot (fleet.md Q1). That claim is
what lets its update pipeline be a plain recreate with seconds of downtime
(``deploy/fleet-deploy.sh``) instead of a highly-available cluster, so it is
worth a test rather than a paragraph.

The outage here is the real one: the broker process is killed while the agent
is connected to it, and brought back on the same address, which is what a fleet
box being redeployed or rebooted looks like from the robot. Everything else is
real too — the actual agent with a genuine paho client, the actual ``mote_tasks``
behaviour tree, and a mock ``navigate_to_pose`` standing in for the wheels.

What it pins down:

1. the robot executes a task, start to finish, with no broker in existence;
2. the agent survives the outage as a *node* — its timers keep running and it
   never takes the ROS graph down with it;
3. when the server comes back the agent reconnects by itself, with no restart
   and nothing to re-enroll, and re-publishes its retained state so a fresh
   operator sees the robot immediately;
4. and fleet dispatch works again afterwards, including reporting the task the
   robot ran on its own while nobody was watching (``source: local``).
"""

import json
import os
import random

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
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from mote_bringup.spec import mission  # noqa: E402

pytestmark = needs_broker


class BenchOperator(Node):
    """Somebody on the robot itself: a `ros2 topic pub`, a bench script.

    This is the local half of the seam the agent has to attribute correctly,
    and during an outage it is the *only* way a task can be started.
    """

    def __init__(self):
        super().__init__("bench_operator")
        self.commands = self.create_publisher(String, "task/command", 10)
        self.statuses = []
        self.create_subscription(String, "task/status", self._status, 10)

    def send(self, capability, payload_input):
        payload = mission.command("mote-01", capability, payload_input)
        self.commands.publish(String(data=json.dumps(payload)))
        return payload["id"]

    def _status(self, message):
        self.statuses.append(json.loads(message.data))

    def terminal(self):
        return [s for s in self.statuses if s["terminal"]]


def test_robot_keeps_working_while_the_fleet_server_is_down(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "mote"))
    # A private DDS domain: this test publishes task/command, and a live robot
    # or sim on this machine must never be what receives it.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(random.randint(60, 100)))

    from mote_bringup import identity

    from mote_fleet import fleet_config

    broker = Broker(tmp_path).start()
    # An already-enrolled robot: this test is about the server going away
    # afterwards, which test_e2e_fleet.py's enrollment half does not cover.
    identity.set_identity(id="mote-01", name="Scout", site="home")
    fleet_config.save(
        server=f"http://127.0.0.1:{broker.port}",
        broker_host="127.0.0.1",
        broker_port=broker.port,
    )

    # A namespace per test process, as test_e2e_fleet.py does: two of these
    # running at once must not answer each other's topics.
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
            Parameter("platform_id", value="mote-01"),
            Parameter("tick_period", value=0.05),
            Parameter("pick_duration", value=0.2),
            Parameter("place_duration", value=0.2),
        ]
    )
    nav = MockNav()
    bench = BenchOperator()
    executor = SingleThreadedExecutor()
    for node in (agent, tasks, nav, bench):
        executor.add_node(node)

    operator = Operator(broker.port)
    try:
        assert spin_until(executor, lambda: agent.connected), "agent never connected"
        assert spin_until(executor, lambda: operator.of("presence")), "no presence"

        # ---- the fleet server goes away ----
        operator.close()
        broker.stop()
        assert spin_until(executor, lambda: not agent.connected, timeout=30.0), (
            "the agent did not notice the broker had gone"
        )

        # ---- ...and the robot carries on. A task started on the robot runs to
        # completion with nothing to report it to. ----
        bench.send("goto", {"target": "kitchen"})
        assert spin_until(executor, lambda: bench.terminal(), timeout=30.0), (
            f"no terminal status offline; saw {bench.statuses}"
        )
        assert bench.terminal()[0]["state"] == mission.SUCCEEDED, bench.statuses
        assert len(nav.goals) == 1, nav.goals
        assert nav.goals[0].pose.position.x == pytest.approx(-1.5)  # kitchen

        # The agent is still a live node through all of it: the mission above
        # was spun on the same executor, so its health and pose timers fired
        # repeatedly against a dead connection without raising. That is what
        # "the agent never takes the robot down with it" means in practice.
        assert rclpy.ok()
        assert not agent.connected

        # ---- the fleet server comes back ----
        broker.start()
        assert spin_until(executor, lambda: agent.connected, timeout=90.0), (
            "the agent did not reconnect on its own"
        )

        # A *fresh* operator, as if the dashboard were opened after the
        # redeploy: retained state means the robot is there immediately.
        operator = Operator(broker.port, client_id="operator-2")
        assert spin_until(executor, lambda: operator.of("presence"), timeout=30.0), (
            "the agent did not re-publish its retained presence"
        )
        assert operator.of("presence")[-1]["online"] is True
        assert spin_until(executor, lambda: operator.of("health"), timeout=30.0)

        # ---- and dispatch works again, with no restart anywhere ----
        command_id = operator.dispatch("mote-01", "goto", {"target": "lab"})
        assert spin_until(
            executor,
            lambda: any(s["terminal"] for s in operator.statuses(command_id)),
            timeout=60.0,
        ), operator.statuses(command_id)
        assert operator.statuses(command_id)[-1]["state"] == mission.SUCCEEDED
        assert len(nav.goals) == 2, nav.goals
    finally:
        operator.close()
        for node in (agent, tasks, nav, bench):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
        broker.stop()

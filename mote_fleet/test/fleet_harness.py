"""Shared pieces for the tests that run a real broker and a real ROS graph.

``test_e2e_fleet.py`` (M1's acceptance: enroll, then dispatch over MQTT) and
``test_fleet_outage.py`` (Ms's: the robot is unaffected while the fleet server
is down) need the same scaffolding — a mosquitto to talk to, Nav2's action
server without Nav2, and an off-robot MQTT client that publishes commands and
collects everything retained. It lives here so the two tests share one
definition of "the fleet, minus the wheels".

Importing this module skips the importing test when rclpy, paho or a broker
binary is missing, which is what makes these files harmless in CI's robot
environment (no broker) and in the lint environment (no ROS).
"""

import json
import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
mqtt = pytest.importorskip("paho.mqtt.client")

from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.node import Node  # noqa: E402

#: Zones both tests drive to. Matches the sim world's coordinates closely
#: enough to read, but nothing here touches a simulator.
ZONES = """\
frame_id: map
zones:
  kitchen: {x: -1.5, y: 0.5, yaw: 0.0, radius: 1.5}
  lab: {x: 4.0, y: -2.0}
"""


def mosquitto_bin() -> str | None:
    """The broker binary, or None.

    conda-forge installs the broker into ``$PREFIX/sbin`` and only the clients
    into ``bin``, and pixi puts only ``bin`` on PATH — so ``which mosquitto``
    says no in an environment that has it.
    """
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        candidate = Path(prefix) / "sbin" / "mosquitto"
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return shutil.which("mosquitto")


BROKER_BIN = mosquitto_bin()

#: Put this in a module's ``pytestmark`` to require a broker.
needs_broker = pytest.mark.skipif(
    BROKER_BIN is None,
    reason="needs a mosquitto broker (pixi run -e dev / -e fleet)",
)


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Broker:
    """A mosquitto on a fixed port that can be stopped and started again.

    The outage test needs the *same* port to come back, because that is what a
    fleet server being rebooted looks like from the robot: the address it was
    told at enrollment does not change.
    """

    def __init__(self, tmp_path, port=None):
        self.port = port or free_port()
        self.conf = Path(tmp_path) / "mosquitto.conf"
        self.conf.write_text(
            f"listener {self.port} 127.0.0.1\nallow_anonymous true\npersistence false\n"
        )
        self.process = None

    def start(self, timeout=10.0):
        if self.process is not None:
            return self
        self.process = subprocess.Popen(
            [BROKER_BIN, "-c", str(self.conf)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.5):
                    return self
            except OSError:
                time.sleep(0.1)
        self.stop()
        raise RuntimeError("mosquitto did not start")

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=10)
        self.process = None


class MockNav(Node):
    """Nav2's action server, minus Nav2."""

    def __init__(self):
        super().__init__("mock_nav")
        self.goals = []
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self.execute
        )

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()


class Operator:
    """The off-robot side: publishes commands, collects everything retained."""

    def __init__(self, port, client_id="operator"):
        self.messages = []
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.client.on_message = self._on_message
        self.client.connect("127.0.0.1", port, keepalive=30)
        self.client.subscribe("mote/v1/#", qos=1)
        self.client.loop_start()

    def _on_message(self, _client, _userdata, message):
        self.messages.append((message.topic, json.loads(message.payload)))

    def of(self, leaf):
        return [payload for topic, payload in self.messages if topic.endswith(leaf)]

    def statuses(self, command_id):
        return [
            payload
            for payload in self.of("task/status")
            if payload.get("id") == command_id
        ]

    def dispatch(self, robot_id, text):
        from mote_fleet import protocol

        payload = protocol.command(text, issued_by="test")
        self.client.publish(
            protocol.topic(robot_id, protocol.COMMAND),
            protocol.encode(payload),
            qos=1,
            retain=False,
        )
        return payload["id"]

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


def spin_until(executor, condition, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)
    return condition()

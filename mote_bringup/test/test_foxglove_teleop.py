"""M2's acceptance test, the "drives" half: Foxglove's teleop reaches the wheels.

Everything real except the operator and the motors — the actual
``foxglove_bridge``, a WebSocket client speaking the protocol the Teleop panel
speaks, and the real ``twist_relay``. What is being proved is the seam the
milestone rests on: that a browser panel which can only emit unstamped
``geometry_msgs/Twist`` ends up commanding a controller that only accepts
``TwistStamped``, with no workstation on the robot's ROS graph.

The load-bearing detail is the **schema name**. Foxglove's Teleop panel
advertises the ROS 1 spelling ``geometry_msgs/Twist``; ROS 2's type is
``geometry_msgs/msg/Twist``. If the bridge did not normalise that, remote teleop
would fail silently — the panel would look connected and the robot would never
move — so the test advertises the ROS 1 spelling deliberately.

What it cannot prove is that the shipped layout renders, since Foxglove itself is
not importable here; ``docs/fleet/m2-verification.md`` records what that leaves
open. It runs in the dev environment (``pixi run -e dev test-foxglove``) and
skips in the robot environment, which carries no WebSocket client.
"""

import asyncio
import json
import os
import random
import socket
import struct
import subprocess
import threading
import time

import pytest

from mote_bringup.sweep_orphans import reap_group, spawn_reapable

try:
    import websockets
except ImportError:  # the robot environment carries no WebSocket client
    websockets = None


def _has_bridge() -> bool:
    """foxglove_bridge is a ROS executable in lib/, so it is never on PATH."""
    try:
        from ament_index_python.packages import get_package_share_directory

        get_package_share_directory("foxglove_bridge")
        return True
    except Exception:
        return False


# Deliberately a skipif rather than `pytest.importorskip`: raising Skipped while
# this module is being imported aborts collection for the whole directory under
# pytest 9, silently taking the other mote_bringup tests with it.
pytestmark = pytest.mark.skipif(
    websockets is None or not _has_bridge(),
    reason="needs the dev env's websockets client and the foxglove_bridge package",
)

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402

TELEOP_TOPIC = "/cmd_vel_teleop"
DRIVE_TOPIC = "/diff_drive_controller/cmd_vel"

# Bridge 3.3.0 speaks this subprotocol; the older `foxglove.websocket.v1` is
# refused with an HTTP 400 at the handshake.
SUBPROTOCOL = "foxglove.sdk.v1"

LINEAR_X, ANGULAR_Z = 0.15, -0.6


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _cdr_twist(linear_x: float, angular_z: float) -> bytes:
    """A CDR-encoded geometry_msgs/Twist: encapsulation header + six float64."""
    return struct.pack(
        "<BBHdddddd", 0x00, 0x01, 0x0000, linear_x, 0.0, 0.0, 0.0, 0.0, angular_z
    )


class _Sink(Node):
    def __init__(self):
        super().__init__("teleop_test_sink")
        self.received: list[TwistStamped] = []
        self.create_subscription(TwistStamped, DRIVE_TOPIC, self.received.append, 10)


@pytest.fixture(scope="module")
def teleop_stack():
    """The bridge and the relay, on a private ROS domain and a free port."""
    # A random domain keeps the test off the default graph — otherwise it would
    # command a real robot that happened to be on the bench.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(80, 160))
    port = _free_port()

    # spawn_reapable, not Popen: `ros2 run` forwards no signal to the node it
    # spawned, so terminating the wrapper alone leaks the bridge and the relay.
    procs = [
        spawn_reapable(
            [
                "ros2",
                "run",
                "foxglove_bridge",
                "foxglove_bridge",
                "--ros-args",
                "-p",
                f"port:={port}",
                "-p",
                "address:=127.0.0.1",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        ),
        spawn_reapable(
            [
                "ros2",
                "run",
                "mote_bringup",
                "twist_relay",
                "--ros-args",
                "-r",
                f"cmd_vel_in:={TELEOP_TOPIC}",
                "-r",
                f"cmd_vel_out:={DRIVE_TOPIC}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        ),
    ]

    deadline = time.time() + 30
    while time.time() < deadline:
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                break
        time.sleep(0.5)
    else:
        for p in procs:
            reap_group(p)
        pytest.fail("foxglove_bridge never opened its port")
    time.sleep(3)  # let the relay finish discovery

    rclpy.init(args=None)
    sink = _Sink()
    stop = threading.Event()

    def _spin():
        while not stop.is_set():
            rclpy.spin_once(sink, timeout_sec=0.05)

    spinner = threading.Thread(target=_spin, daemon=True)
    spinner.start()
    try:
        yield port, sink
    finally:
        stop.set()
        spinner.join(timeout=5)
        sink.destroy_node()
        rclpy.try_shutdown()
        for p in procs:
            reap_group(p)


async def _drive(port: int, schema_name: str, count: int = 20) -> None:
    """Advertise and publish exactly as the Teleop panel does."""
    async with websockets.connect(
        f"ws://127.0.0.1:{port}", subprotocols=[SUBPROTOCOL]
    ) as ws:
        info = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        assert info["op"] == "serverInfo", info
        assert "clientPublish" in info["capabilities"], (
            "the bridge must allow client publishing or teleop is impossible"
        )
        await ws.send(
            json.dumps(
                {
                    "op": "advertise",
                    "channels": [
                        {
                            "id": 1,
                            "topic": TELEOP_TOPIC,
                            "encoding": "cdr",
                            "schemaName": schema_name,
                        }
                    ],
                }
            )
        )
        await asyncio.sleep(2.0)  # the bridge creates the ROS publisher
        for _ in range(count):
            await ws.send(
                b"\x01" + struct.pack("<I", 1) + _cdr_twist(LINEAR_X, ANGULAR_Z)
            )
            await asyncio.sleep(0.05)
        await asyncio.sleep(1.5)


def test_the_panels_ros1_type_name_still_reaches_the_wheels(teleop_stack):
    """The whole remote-driving path, end to end, in the panel's own dialect."""
    port, sink = teleop_stack
    sink.received.clear()

    asyncio.run(_drive(port, "geometry_msgs/Twist"))

    assert sink.received, (
        "no TwistStamped on the drive topic: the bridge did not accept the "
        "panel's ROS 1 type name, or the relay did not convert it"
    )
    cmd = sink.received[0]
    assert cmd.twist.linear.x == pytest.approx(LINEAR_X)
    assert cmd.twist.angular.z == pytest.approx(ANGULAR_Z)


def test_the_relay_stamps_on_the_robot(teleop_stack):
    """The stamp is what cmd_vel_timeout measures, so it must not be empty."""
    port, sink = teleop_stack
    sink.received.clear()

    asyncio.run(_drive(port, "geometry_msgs/msg/Twist"))

    assert sink.received
    cmd = sink.received[0]
    assert (cmd.header.stamp.sec, cmd.header.stamp.nanosec) != (0, 0)
    assert cmd.header.frame_id == "base_footprint"

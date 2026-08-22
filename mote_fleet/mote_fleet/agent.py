"""``mote_agent`` — the robot's single connection to the fleet.

Every byte that leaves this robot for the fleet server leaves through here, and
nothing off-box joins the robot's ROS graph to get it. That is the property the
whole fleet design is built on (fleet.md Q1): because the agent is the sole
egress, there is no fleet-wide DDS graph to partition, no domain id to allocate,
and the robot's autonomy is unchanged by anything the fleet does.

**It is a bridge and a reporter, never in the control loop.** Nav2, SLAM and the
``mote_tasks`` behaviour tree run locally and keep running with the fleet server
unplugged: a dropped link means the agent stops reporting, not that the robot
stops. On reconnect it re-publishes its retained state, and the broker has
already told everyone it was gone (the Last Will, below).

What it does, in both directions:

*Up* — presence, health and pose. Presence is retained and doubles as the MQTT
Last Will, so a robot that loses power is marked offline by the broker within
the keepalive rather than after somebody notices a heartbeat stopped. Health is
the health monitor's own ``/diagnostics_agg`` roll-up, forwarded rather than
recomputed, so the fleet sees exactly what the robot sees. Pose is the map-frame
TF, carrying the site and floor it is meaningful in.

*Up* — what this robot can be asked to do. The task server publishes its
capability/v0 set on a latched ROS topic and the agent forwards it, retained,
so a dispatcher reads the input shapes instead of guessing them. Forwarded
rather than authored, deliberately: the capabilities are a fact about the
executor that is running, so a robot whose task server is down advertises
nothing, which is true.

*Down* — missions. A mission/v0 command carries a correlation id and a typed
input, and is handed to ROS **byte for byte**: this bridge does not parse,
rewrite or validate the input, because the executor is the thing that declared
the schema and a second opinion here could only disagree with it. What the
agent does keep is what is its own — deduplicating a redelivery, retaining
terminal statuses for a dispatcher that restarts, failing a mission the
executor never answered, and saying whether a status belongs to the fleet or
to somebody at the robot's own keyboard (:mod:`mote_fleet.dispatch`).

*Down* — the canonical map. The registry announces each floor's canonical map
revision on a retained topic, so this agent learns about a new map the instant
it connects rather than by polling, and installs it by staging the revision and
flipping one symlink (:mod:`mote_fleet.mapsync`, fleet.md Q4). Nothing about
that is in the control loop either: a map arrives, is staged, and is published
locally — the running navigation stack keeps using the map it loaded until it
is restarted.

Threading: paho runs its own network loop, so its callbacks arrive off the ROS
executor. Inbound commands are therefore pushed onto a queue and drained by a
ROS timer — the ROS publisher is only ever touched from the executor thread.
Map downloads get a worker thread of their own for the same reason in reverse:
a multi-megabyte transfer must not sit inside a timer callback.

    pixi run agent            # or: mote-agent.service (installed, not enabled)
"""

import json
import math
import os
import queue
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import tf2_ros

from mote_bringup import identity, sites
from mote_bringup.spec import SpecError
from mote_bringup.spec import mission as spec_mission

from mote_fleet import dispatch, fleet_config, mapsync, protocol

#: Matches the task server's capability publisher: the set is state, not an
#: event, so an agent that starts after the task server still learns it.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# diagnostic_msgs levels -> the contract's health states.
LEVEL_STATE = {
    DiagnosticStatus.OK: protocol.OK,
    DiagnosticStatus.WARN: protocol.DEGRADED,
    DiagnosticStatus.ERROR: protocol.FAULT,
    DiagnosticStatus.STALE: protocol.STALE,
}

# The health monitor's rolled-up status; the rest of the array are subsystems.
ROLLUP_NAME = "mote"

# How long the active site/floor is cached before re-reading ~/.mote/active.yaml.
SITE_CACHE_S = 30.0


def repo_version() -> str:
    """What software this robot is running, for the health payload.

    ``MOTE_VERSION`` first so a packaged install (M5) can bake it in; a git
    describe otherwise, which is what a source checkout can honestly answer.
    """
    baked = os.environ.get("MOTE_VERSION")
    if baked:
        return baked
    try:
        return (
            subprocess.run(
                ["git", "describe", "--always", "--dirty", "--tags"],
                capture_output=True,
                text=True,
                timeout=5,
                cwd=os.path.dirname(os.path.abspath(__file__)),
            ).stdout.strip()
            or "unknown"
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def host_uptime_s() -> float | None:
    try:
        with open("/proc/uptime") as handle:
            return float(handle.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def paho_client(client_id: str):
    """A paho client, imported here so the module stays importable without it."""
    import paho.mqtt.client as mqtt

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:  # paho 1.x
        client = mqtt.Client(client_id=client_id)
    # Reconnect with backoff forever: a fleet server that is down or a robot
    # that drove out of coverage must rejoin by itself, never by a restart.
    client.reconnect_delay_set(min_delay=1, max_delay=60)
    return client


class MoteAgent(Node):
    def __init__(self, client_factory=paho_client, **node_kwargs):
        super().__init__("mote_agent", **node_kwargs)
        self._client_factory = client_factory

        self.broker_host = self.declare_parameter("broker_host", "").value
        self.broker_port = self.declare_parameter("broker_port", 0).value
        health_period = self.declare_parameter("health_period", 5.0).value
        pose_period = self.declare_parameter("pose_period", 1.0).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.diagnostics_timeout = self.declare_parameter(
            "diagnostics_timeout", 15.0
        ).value
        self.keepalive = self.declare_parameter("keepalive", 20).value
        command_timeout = self.declare_parameter("command_timeout", 20.0).value
        # Map distribution is opt-out rather than opt-in: a robot that is on
        # the fleet should be running the fleet's canonical map for its floor.
        self.map_sync = self.declare_parameter("map_sync", True).value

        self.robot_id = None
        self.client = None
        self.connected = False
        self.version = repo_version()
        self.tracker = dispatch.CommandTracker(accept_timeout=command_timeout)
        self.capabilities = None
        self._inbound = queue.Queue()
        self._snapshot_due = False
        self._diagnostics = None
        self._diagnostics_at = None
        self._site = (None, None)
        self._site_at = 0.0
        self._pulls = queue.Queue()
        self._pull_results = queue.Queue()
        self._pull_thread = None

        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.create_subscription(String, "task/status", self._on_ros_status, 10)
        self.create_subscription(
            String, "task/capabilities", self._on_capabilities, LATCHED
        )
        self.create_subscription(
            DiagnosticArray, "diagnostics_agg", self._on_diagnostics, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Connecting is a timer, not a constructor step: a robot may boot before
        # it has enrolled, and it should join the fleet when it does rather than
        # need a restart. Re-reading the files each attempt is what allows that.
        self._connect_timer = self.create_timer(10.0, self._try_connect)
        self._try_connect()

        self.create_timer(0.05, self._drain_inbound)
        self.create_timer(1.0, self._drain_pulls)
        self.create_timer(health_period, self.publish_health)
        self.create_timer(pose_period, self.publish_pose)
        self.create_timer(1.0, self._tick_tracker)

    # ---- connection -----------------------------------------------------

    def _resolve(self) -> tuple[str, str, int] | None:
        """``(robot_id, broker_host, broker_port)`` once this robot can join."""
        robot_id = identity.robot_id()
        if not robot_id:
            self.get_logger().warning(
                f"no identity at {identity.identity_path()} — enrol this robot "
                "('pixi run enroll -- --server ... --token ...'); retrying",
                throttle_duration_sec=60.0,
            )
            return None
        host = self.broker_host
        port = self.broker_port
        if not host:
            configured = fleet_config.broker()
            if not configured:
                self.get_logger().warning(
                    f"no broker configured at {fleet_config.config_path()} — "
                    "enrol this robot, or set the broker_host parameter; retrying",
                    throttle_duration_sec=60.0,
                )
                return None
            host, port = configured
        return robot_id, host, int(port or fleet_config.DEFAULT_BROKER_PORT)

    def _try_connect(self):
        if self.client is not None:
            return
        resolved = self._resolve()
        if resolved is None:
            return
        self.robot_id, host, port = resolved
        self.tracker.platform_id = self.robot_id

        client = self._client_factory(f"mote-agent-{self.robot_id}")
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_mqtt_message
        # The Last Will is the whole reason a dropped robot is noticed
        # immediately: the broker publishes it when this connection dies,
        # retained, so it also becomes the answer any later subscriber gets.
        client.will_set(
            protocol.topic(self.robot_id, protocol.PRESENCE),
            protocol.encode(
                protocol.presence(self.robot_id, False, reason="last will")
            ),
            qos=protocol.QOS,
            retain=True,
        )
        self.get_logger().info(
            f"agent for {self.robot_id} connecting to mqtt://{host}:{port}"
        )
        # Assigned before loop_start: paho's network thread can reach CONNACK
        # (and therefore _on_connect) before the next statement here would run.
        self.client = client
        try:
            client.connect_async(host, port, keepalive=self.keepalive)
            client.loop_start()
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f"broker connect failed: {exc}; retrying")
            self.client = None

    def _on_connect(self, client, _userdata, *args):
        """paho's thread. Subscribe here; leave anything ROS-shaped to the
        executor by asking _drain_inbound for a snapshot."""
        self.connected = True
        self.get_logger().info(f"connected to broker as {self.robot_id}")
        client.subscribe(
            protocol.topic(self.robot_id, protocol.COMMAND), qos=protocol.QOS
        )
        if self.map_sync:
            # Retained, so this arrives immediately for every floor — including
            # the ones mapped while this robot was switched off.
            client.subscribe(protocol.any_floor(), qos=protocol.QOS)
        client.publish(
            protocol.topic(self.robot_id, protocol.PRESENCE),
            protocol.encode(
                protocol.presence(self.robot_id, True, version=self.version)
            ),
            qos=protocol.QOS,
            retain=True,
        )
        # Fill the retained topics straight away so an operator connecting a
        # second later sees this robot rather than a gap until the next timer.
        self._snapshot_due = True

    def _on_disconnect(self, _client, _userdata, *args):
        self.connected = False
        self.get_logger().warning("disconnected from broker; paho will retry")

    def _publish(self, leaf: str, payload: dict, retain: bool = True):
        if self.client is None or self.robot_id is None:
            return
        self.client.publish(
            protocol.topic(self.robot_id, leaf),
            protocol.encode(payload),
            qos=protocol.QOS,
            retain=retain,
        )

    # ---- inbound commands -----------------------------------------------

    def _on_mqtt_message(self, _client, _userdata, message):
        """Called on paho's thread — hand off, do not touch ROS from here."""
        self._inbound.put((message.topic, bytes(message.payload)))

    def _drain_inbound(self):
        if self._snapshot_due:
            self._snapshot_due = False
            self.publish_health()
            self.publish_pose()
            self.publish_capabilities()
        while True:
            try:
                topic, raw = self._inbound.get_nowait()
            except queue.Empty:
                return
            if protocol.parse_registry_topic(topic):
                self._handle_announcement(topic, raw)
            else:
                self._handle_command(topic, raw)

    def _handle_command(self, topic: str, raw: bytes):
        try:
            payload = spec_mission.check(json.loads(raw), "command")
        except (ValueError, SpecError) as exc:
            self.get_logger().warning(f"ignoring {topic}: {exc}")
            return
        if payload["platform_id"] != self.robot_id:
            self.get_logger().warning(
                f"ignoring a mission addressed to {payload['platform_id']!r}"
            )
            return

        action, update = self.tracker.submit(
            payload, self.get_clock().now().nanoseconds / 1e9
        )
        if action == dispatch.FORWARD:
            self.get_logger().info(
                f"dispatching {payload['capability']} (id {payload['id']})"
            )
            # Byte for byte: the executor declared the input schema, so the
            # bridge has nothing to add and everything to get wrong.
            self.command_pub.publish(String(data=json.dumps(payload)))
        else:
            self.get_logger().info(
                f"redelivery of {payload['id']}; re-publishing {update['state']}"
            )
        self._publish_status(update)

    def _on_ros_status(self, msg: String):
        try:
            payload = spec_mission.check(json.loads(msg.data), "status")
        except (ValueError, SpecError) as exc:
            self.get_logger().warning(f"unreadable task status: {exc}")
            return
        self._publish_status(
            self.tracker.on_status(payload, self.get_clock().now().nanoseconds / 1e9)
        )

    def _on_capabilities(self, msg: String):
        """Forward the task server's capability set, retained.

        Kept so it can be re-published on reconnect: the set arrives once, on
        a latched topic, and a broker that was down at that moment would
        otherwise leave the fleet with no idea what this robot offers until the
        task server restarted.
        """
        self.capabilities = msg.data
        self.publish_capabilities()

    def publish_capabilities(self):
        if self.client is None or self.robot_id is None or self.capabilities is None:
            return
        self.client.publish(
            protocol.topic(self.robot_id, protocol.CAPABILITIES),
            self.capabilities.encode(),
            qos=protocol.QOS,
            retain=True,
        )

    def _tick_tracker(self):
        for update in self.tracker.tick(self.get_clock().now().nanoseconds / 1e9):
            self.get_logger().warning(
                f"mission {update['id']}: {update['failure']['detail']}"
            )
            self._publish_status(update)

    def _publish_status(self, payload: dict):
        self._publish(protocol.STATUS, payload)

    # ---- the map registry -----------------------------------------------

    def _handle_announcement(self, topic: str, raw: bytes):
        """A floor's canonical revision changed (or we just connected)."""
        try:
            payload = protocol.decode(raw, protocol.CURRENT)
        except protocol.ProtocolError as exc:
            self.get_logger().warning(f"ignoring {topic}: {exc}")
            return
        if not mapsync.wants(payload, self._active_site_now()):
            return
        floor = f"{payload['site']}/{payload['floor']}"
        if (
            self._local_revision(payload["site"], payload["floor"])
            == payload["revision"]
        ):
            return
        self.get_logger().info(
            f"{floor}: fleet canonical map is {payload['revision']}, pulling"
        )
        self._pulls.put(payload)
        self._ensure_puller()

    def _ensure_puller(self):
        """One worker thread, started on the first pull and kept.

        A thread rather than a timer because a revision is megabytes over a
        link that may be a robot's wifi: doing it in the executor would stall
        pose and health reporting for as long as the transfer takes.
        """
        if self._pull_thread is not None and self._pull_thread.is_alive():
            return
        self._pull_thread = threading.Thread(
            target=self._pull_loop, name="mote-agent-mapsync", daemon=True
        )
        self._pull_thread.start()

    def _pull_loop(self):
        while True:
            try:
                announcement = self._pulls.get(timeout=30.0)
            except queue.Empty:
                return
            server = (fleet_config.load() or {}).get("server") or ""
            if not server:
                self._pull_results.put(
                    ("error", "no fleet server configured; cannot pull maps")
                )
                continue
            try:
                result = mapsync.pull(server, announcement)
            except mapsync.SyncError as exc:
                self._pull_results.put(("error", str(exc)))
                continue
            self._pull_results.put(("ok", result))

    def _drain_pulls(self):
        """Report what the worker did, from the executor thread."""
        while True:
            try:
                kind, result = self._pull_results.get_nowait()
            except queue.Empty:
                return
            if kind == "error":
                self.get_logger().warning(f"map sync: {result}")
                continue
            if result["action"] == "current":
                continue
            self.get_logger().info(
                f"{result['site']}/{result['floor']}: {result['action']} map "
                f"revision {result['revision']} (restart nav to load it)"
            )
            # Health carries the running revision, so the fleet can see the
            # difference between a robot that has the canonical map and one
            # that has not picked it up yet.
            self.publish_health()

    def _local_revision(self, site: str, floor: str) -> str | None:
        try:
            return sites.current_revision(sites.floor_dir(site, floor))
        except OSError:
            return None

    def _map_summary(self) -> dict | None:
        site, floor = self._active_site()
        if not site or not floor:
            return None
        return {
            "site": site,
            "floor": floor,
            "revision": self._local_revision(site, floor),
        }

    # ---- outbound telemetry ---------------------------------------------

    def _on_diagnostics(self, msg: DiagnosticArray):
        self._diagnostics = msg
        self._diagnostics_at = time.monotonic()

    def _active_site_now(self) -> tuple[str, str] | None:
        """The active site/floor, uncached — read on the executor thread via
        _drain_inbound, where a 30 s stale answer could skip a pull."""
        try:
            return sites.active()
        except Exception:  # a malformed active.yaml must not stop reporting
            return None

    def _active_site(self) -> tuple[str | None, str | None]:
        now = time.monotonic()
        if now - self._site_at > SITE_CACHE_S:
            try:
                self._site = sites.active() or (None, None)
            except Exception:  # a malformed active.yaml must not stop reporting
                self._site = (None, None)
            self._site_at = now
        return self._site

    def health_payload(self) -> dict:
        site, floor = self._active_site()
        common = dict(
            mission=self.tracker.summary(),
            site=site,
            floor=floor,
            version=self.version,
            uptime_s=host_uptime_s(),
            map=self._map_summary(),
        )
        fresh = (
            self._diagnostics_at is not None
            and time.monotonic() - self._diagnostics_at <= self.diagnostics_timeout
        )
        if not fresh:
            # No health monitor, or it stopped. Report that as its own state
            # rather than inventing an "ok" the robot never claimed.
            summary = (
                "no diagnostics from the health monitor"
                if self._diagnostics_at is None
                else "health monitor stopped reporting"
            )
            return protocol.health(
                self.robot_id, protocol.UNKNOWN, summary, [], **common
            )

        rollup, subsystems = None, []
        for status in self._diagnostics.status:
            if status.name == ROLLUP_NAME and rollup is None:
                rollup = status
            else:
                subsystems.append(
                    protocol.subsystem(
                        status.name,
                        LEVEL_STATE.get(status.level, protocol.UNKNOWN),
                        status.message,
                    )
                )
        if rollup is None:
            # An aggregate without the monitor's own roll-up: derive the worst
            # level present rather than drop the report.
            worst = max((s.level for s in self._diagnostics.status), default=0)
            state = LEVEL_STATE.get(worst, protocol.UNKNOWN)
            summary = f"{len(subsystems)} subsystems"
        else:
            state = LEVEL_STATE.get(rollup.level, protocol.UNKNOWN)
            summary = rollup.message
        return protocol.health(self.robot_id, state, summary, subsystems, **common)

    def publish_health(self):
        if self.robot_id is None:
            return
        self._publish(protocol.HEALTH, self.health_payload())

    def pose_payload(self) -> dict | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().info(
                f"no {self.map_frame}->{self.base_frame} transform yet: {exc}",
                throttle_duration_sec=30.0,
            )
            return None
        site, floor = self._active_site()
        return protocol.pose(
            self.robot_id,
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw_from_quaternion(tf.transform.rotation),
            frame_id=self.map_frame,
            site=site,
            floor=floor,
        )

    def publish_pose(self):
        if self.robot_id is None:
            return
        payload = self.pose_payload()
        if payload is not None:
            self._publish(protocol.POSE, payload)

    # ---- shutdown -------------------------------------------------------

    def close(self):
        """Say goodbye properly, so the fleet sees a clean stop, not a death."""
        if self.client is None:
            return
        if self.connected:
            self._publish(
                protocol.PRESENCE,
                protocol.presence(self.robot_id, False, reason="agent stopped"),
            )
        try:
            self.client.disconnect()
            self.client.loop_stop()
        except Exception:
            pass
        self.client = None


def main():
    rclpy.init()
    node = MoteAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what systemd's SIGTERM looks like from
        # in here. Catching it is what turns a service stop into "agent
        # stopped" on the wire rather than a Last Will that reads like a crash.
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

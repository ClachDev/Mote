"""The fleet control-plane wire contract, version 1.

Every message between a robot's agent and the fleet server crosses this module,
and nothing else defines the wire: the topic tree, the payload shapes, and the
task state machine all live here. It is deliberately **stdlib-only and
ROS-free** so the robot-side agent (which has rclpy) and the off-board fleet
server and operator CLI (which have neither ROS nor a checkout of the robot
software) can import the same definitions instead of agreeing by convention.
The mirror image is ``mote_perception/depth_wire.py``, which does the same job
for the inference link.

The contract is versioned in two places, for two different kinds of change:

* **The topic root carries the major version** — ``mote/v1/<robot_id>/...``. A
  breaking change to the tree (a topic moves, a payload field changes meaning)
  ships as ``mote/v2/...`` so both trees can be published at once while
  subscribers migrate. A subscriber never has to guess which contract a message
  came from.
* **Every payload carries ``schema``** — an integer that tracks the payload
  shape within a major version. Adding an optional field does not bump it;
  consumers must ignore fields they do not know.

The prose contract, with the field tables and the compatibility rules, is
``docs/fleet/control-plane.md``. The machine-readable mirror is
``mote_fleet/schema/*.schema.json``, and ``test_protocol.py`` fails if the three
ever disagree.

Topic tree (``mote/v1/<robot_id>/…``)::

    presence      retained, LWT   is the agent connected
    health        retained        rolled-up robot health + current task
    pose          retained        last known pose in the map frame
    task/command  not retained    fleet -> robot: one command, correlation id
    task/status   retained        robot -> fleet: that command's transitions

``task/command`` is the one topic that is **never retained**: a retained
command would be re-delivered to the robot every time it reconnects, which
turns a link flap into a re-dispatch. Everything else is retained, so an
operator UI that connects at any moment sees the current state of the fleet
without waiting for the next heartbeat.
"""

import json
import re
import uuid
from datetime import datetime, timezone

# Payload shape version. Bumped only for a breaking change *within* the v1 tree;
# see the module docstring for the split between this and the topic version.
SCHEMA = 1

ROOT = "mote"
VERSION = "v1"

PRESENCE = "presence"
HEALTH = "health"
POSE = "pose"
COMMAND = "task/command"
STATUS = "task/status"

# QoS 1 (at-least-once) everywhere: the broker may redeliver, and every consumer
# here is idempotent — status/health/pose are snapshots, and a command is keyed
# by its correlation id so a redelivery is recognised rather than re-run.
QOS = 1

# Task lifecycle. The agent owns this state machine; the robot's task_server has
# no notion of a correlation id (task/command is a bare std_msgs/String), so
# attribution comes from the agent's single-in-flight rule, not from ROS.
DISPATCHED = "dispatched"  # agent has forwarded it to ROS, no verdict yet
ACCEPTED = "accepted"  # the behaviour tree took it and is running
REJECTED = "rejected"  # refused outright (busy, unknown command, bad zone)
SUCCEEDED = "succeeded"
FAILED = "failed"

TASK_STATES = (DISPATCHED, ACCEPTED, REJECTED, SUCCEEDED, FAILED)
TERMINAL_STATES = frozenset({REJECTED, SUCCEEDED, FAILED})

# Where a status came from: a command this agent dispatched, or one issued
# locally on the robot (a `ros2 topic pub`, a bench script). Local tasks are
# reported too — the fleet should see a robot that is busy, whoever asked it.
SOURCE_FLEET = "fleet"
SOURCE_LOCAL = "local"

# Health roll-up. Mirrors diagnostic_msgs levels, plus "unknown" for the state
# before any diagnostics have arrived (the health monitor is a separate service
# and may simply not be running).
OK = "ok"
DEGRADED = "degraded"
FAULT = "fault"
STALE = "stale"
UNKNOWN = "unknown"

HEALTH_STATES = (OK, DEGRADED, FAULT, STALE, UNKNOWN)

# Required keys per payload kind — the authority the JSON Schema files and the
# doc are checked against.
REQUIRED = {
    PRESENCE: ("schema", "robot_id", "online", "stamp"),
    HEALTH: ("schema", "robot_id", "stamp", "state", "summary", "subsystems"),
    POSE: ("schema", "robot_id", "stamp", "frame_id", "x", "y", "yaw"),
    COMMAND: ("schema", "id", "command", "issued_at"),
    STATUS: ("schema", "robot_id", "id", "command", "state", "stamp", "source"),
}

TOPIC_RE = re.compile(rf"^{ROOT}/{VERSION}/([^/+#]+)/(.+)$")

# A robot id is a lowercase DNS label, because it is simultaneously a MagicDNS
# hostname, a level of this topic tree, and a directory name. The definition
# belongs to mote_bringup.identity, but the fleet server has to validate ids
# without importing the robot software, so it is restated here and
# ``test_protocol.py`` fails if the two ever diverge.
ID_RE = re.compile(r"^[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?$")


def valid_id(robot_id: str) -> bool:
    return bool(ID_RE.match(robot_id or ""))


class ProtocolError(ValueError):
    """A payload that does not meet the contract."""


def now() -> str:
    """A wire timestamp: RFC 3339, UTC, millisecond precision."""
    stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    return stamp.replace("+00:00", "Z")


def topic(robot_id: str, leaf: str) -> str:
    return f"{ROOT}/{VERSION}/{robot_id}/{leaf}"


def any_robot(leaf: str) -> str:
    """The subscription that matches ``leaf`` for every robot in the fleet."""
    return f"{ROOT}/{VERSION}/+/{leaf}"


def parse_topic(name: str) -> tuple[str, str] | None:
    """``(robot_id, leaf)`` for a v1 topic, else None."""
    match = TOPIC_RE.match(name)
    return (match.group(1), match.group(2)) if match else None


def encode(payload: dict) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode()


def decode(raw: bytes | str, kind: str | None = None) -> dict:
    """Parse a payload, optionally checking it against ``kind``'s contract."""
    try:
        payload = json.loads(raw)
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"payload is not JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError("payload is not a JSON object")
    if kind is not None:
        check(payload, kind)
    return payload


def check(payload: dict, kind: str) -> dict:
    """Reject a payload the rest of the code would only half-understand."""
    missing = [key for key in REQUIRED[kind] if key not in payload]
    if missing:
        raise ProtocolError(f"{kind} payload missing {', '.join(missing)}")
    version = payload.get("schema")
    if version != SCHEMA:
        # Same major topic tree but a different payload version: refuse rather
        # than guess. A v2 publisher would be on the v2 tree.
        raise ProtocolError(f"{kind} payload schema {version!r}, expected {SCHEMA}")
    return payload


def presence(robot_id: str, online: bool, **extra) -> dict:
    """Connection state. Published retained on connect, and set as the LWT so
    the broker publishes the offline form the moment the agent stops answering
    — that is what makes "robot dropped off" instant rather than a timeout."""
    return {
        "schema": SCHEMA,
        "robot_id": robot_id,
        "online": bool(online),
        "stamp": now(),
        **extra,
    }


def health(
    robot_id: str,
    state: str,
    summary: str,
    subsystems: list[dict],
    *,
    task: dict | None = None,
    site: str | None = None,
    floor: str | None = None,
    version: str | None = None,
    uptime_s: float | None = None,
    battery: dict | None = None,
) -> dict:
    """The rolled-up robot health snapshot.

    ``state``/``summary``/``subsystems`` come straight from the health monitor's
    ``/diagnostics_agg`` roll-up, so the fleet sees exactly what the robot sees.
    ``battery`` is in the contract but always null today: nothing on the robot
    can measure it (the power bank exposes no telemetry — see fleet.md), and a
    field a dashboard can render as "unknown" is better than one added later.
    """
    if state not in HEALTH_STATES:
        raise ProtocolError(f"unknown health state {state!r}")
    return {
        "schema": SCHEMA,
        "robot_id": robot_id,
        "stamp": now(),
        "state": state,
        "summary": summary,
        "subsystems": subsystems,
        "task": task,
        "site": site,
        "floor": floor,
        "version": version,
        "uptime_s": None if uptime_s is None else round(uptime_s, 1),
        "battery": battery,
    }


def subsystem(name: str, state: str, message: str) -> dict:
    if state not in HEALTH_STATES:
        raise ProtocolError(f"unknown health state {state!r}")
    return {"name": name, "state": state, "message": message}


def pose(
    robot_id: str,
    x: float,
    y: float,
    yaw: float,
    *,
    frame_id: str = "map",
    site: str | None = None,
    floor: str | None = None,
) -> dict:
    """Where the robot is, in the map frame of its active floor.

    ``site``/``floor`` travel with the pose because the map frame's origin is an
    accident of where SLAM started: coordinates from one floor mean nothing on
    another, so a consumer must know which basemap to draw them on.
    """
    return {
        "schema": SCHEMA,
        "robot_id": robot_id,
        "stamp": now(),
        "frame_id": frame_id,
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "yaw": round(float(yaw), 4),
        "site": site,
        "floor": floor,
    }


def command(text: str, *, command_id: str | None = None, issued_by: str = "") -> dict:
    """A task for one robot. ``id`` is the correlation id every status carries
    back, and the key the agent deduplicates redeliveries by."""
    return {
        "schema": SCHEMA,
        "id": command_id or uuid.uuid4().hex[:16],
        "command": text,
        "issued_at": now(),
        "issued_by": issued_by,
    }


def status(
    robot_id: str,
    command_id: str | None,
    text: str,
    state: str,
    *,
    detail: str = "",
    source: str = SOURCE_FLEET,
) -> dict:
    """One transition of one task. ``id`` is null for a locally-issued task —
    the fleet did not give it a correlation id, but should still see it run."""
    if state not in TASK_STATES:
        raise ProtocolError(f"unknown task state {state!r}")
    return {
        "schema": SCHEMA,
        "robot_id": robot_id,
        "id": command_id,
        "command": text,
        "state": state,
        "detail": detail,
        "source": source,
        "stamp": now(),
        "terminal": state in TERMINAL_STATES,
    }

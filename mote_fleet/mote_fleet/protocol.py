"""The fleet control-plane wire contract, version 2.

Every message between a robot's agent and the fleet server crosses this module,
and nothing else defines the wire: the topic tree, the payload shapes, and the
task state machine all live here. It is deliberately **stdlib-only and
ROS-free** so the robot-side agent (which has rclpy) and the off-board fleet
server and operator CLI (which have neither ROS nor a checkout of the robot
software) can import the same definitions instead of agreeing by convention.
The mirror image is ``mote_perception/depth_wire.py``, which does the same job
for the inference link.

**v2 is the adoption of the open specifications.** The mission half of this
tree — what a robot is told to do, and what it says came of it — is no longer
Mote's own shape but ``mission/v0``'s, built by :mod:`mote_bringup.spec.mission`
and advertised as ``capability/v0`` by :mod:`mote_tasks.capabilities`. A
command carries a capability key and a typed input where it carried the string
``fetch red_box dropoff``, and a failure carries a class and a recoverability
where it carried a sentence. That is a change of meaning in an existing
payload, which by the rule below is exactly what a new topic root is for; the
telemetry half (presence, health, pose) and the map registry moved with it
unchanged, because a tree has one version and not one per leaf.

What did *not* move is where the definitions live. This module still owns the
topic tree, the QoS and the retain discipline — the transport binding — and
:mod:`mote_bringup.spec` owns the payloads, because the payloads are now a
contract Mote implements rather than one it defines.

The contract is versioned in two places, for two different kinds of change:

* **The topic root carries the major version** — ``mote/v2/<robot_id>/...``. A
  breaking change to the tree (a topic moves, a payload field changes meaning)
  ships as the next root so both trees can be published at once while
  subscribers migrate. A subscriber never has to guess which contract a message
  came from.
* **Every payload carries ``schema``** — an integer that tracks the payload
  shape within a major version. Adding an optional field does not bump it;
  consumers must ignore fields they do not know.

The prose contract, with the field tables and the compatibility rules, is
``docs/fleet/control-plane.md``. The machine-readable mirror is
``mote_fleet/schema/*.schema.json``, and ``test_protocol.py`` fails if the three
ever disagree.

Topic tree (``mote/v2/<robot_id>/…``)::

    presence         retained, LWT   is the agent connected
    health           retained        rolled-up robot health + current mission
    pose             retained        last known pose in the map frame
    capabilities     retained        capability/v0: what this robot can be asked
    mission/command  not retained    fleet -> robot: one mission, correlation id
    mission/status   retained        robot -> fleet: that mission's transitions

``mission/command`` is the one topic that is **never retained**: a retained
command would be re-delivered to the robot every time it reconnects, which
turns a link flap into a re-dispatch. Everything else is retained, so an
operator UI that connects at any moment sees the current state of the fleet
without waiting for the next heartbeat — and, from v2, what each robot can be
asked to do, without asking it.

One subtree is about the fleet rather than about a robot (M4)::

    registry/site/<site>/floor/<floor>/current   retained   canonical map revision

Retained is the whole mechanism there: a robot that was switched off through a
mapping session learns the floor's canonical revision the instant it
reconnects, so map distribution needs no polling and has no missed-update case.
``registry`` is therefore a reserved first level — no robot may be allocated it.
"""

import json
import re
from datetime import datetime, timezone

# Payload shape version. Bumped only for a breaking change *within* the v2 tree;
# see the module docstring for the split between this and the topic version.
SCHEMA = 1

ROOT = "mote"
VERSION = "v2"

PRESENCE = "presence"
HEALTH = "health"
POSE = "pose"
CAPABILITIES = "capabilities"
COMMAND = "mission/command"
STATUS = "mission/status"
#: Specified and not yet published: the task layer has no cancel, so nothing
#: would honour one. The leaf is named here rather than left to be invented,
#: so the day it lands it lands where a reader already expects it.
CANCEL = "mission/cancel"

# The map registry's subtree (M4). It sits at the same level as a robot id
# because it is fleet-wide state rather than one robot's, and the name is
# reserved so the two can never collide.
REGISTRY = "registry"
CURRENT = "current"

#: First topic levels that are not robot ids. A robot called ``registry``
#: would publish its health into the map registry's subtree.
RESERVED_IDS = frozenset({REGISTRY})

# QoS 1 (at-least-once) everywhere: the broker may redeliver, and every consumer
# here is idempotent — status/health/pose are snapshots, and a command is keyed
# by its correlation id so a redelivery is recognised rather than re-run.
QOS = 1

# The mission lifecycle, the failure taxonomy and the payload shapes are
# mission/v0's, in ``mote_bringup.spec.mission``. They are deliberately *not*
# restated here: a second copy of a state machine is a second thing to keep in
# step, and this module's job in v2 is the transport binding, not the payload.
#
# What *is* here is the one thing the transport decides: which leaves are
# retained. ``mission/status`` is, so an operator UI connecting at any moment
# has the whole fleet's mission state with no polling; ``mission/command`` is
# not, so a reconnect is not a re-dispatch.

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
    CURRENT: ("schema", "site", "floor", "revision", "url", "stamp"),
}

#: Leaves whose payload is a specification's rather than Mote's, and which
#: therefore have no row above: ``mote_bringup.spec.mission.check`` and
#: ``spec.capability`` are the authority, and a copy of their required keys
#: here would be a copy free to disagree.
SPEC_PAYLOADS = frozenset({COMMAND, STATUS, CANCEL, CAPABILITIES})

TOPIC_RE = re.compile(rf"^{ROOT}/{VERSION}/([^/+#]+)/(.+)$")
REGISTRY_TOPIC_RE = re.compile(
    rf"^{ROOT}/{VERSION}/{REGISTRY}/site/([^/+#]+)/floor/([^/+#]+)/{CURRENT}$"
)

# A robot id is a lowercase DNS label, because it is simultaneously a MagicDNS
# hostname, a level of this topic tree, and a directory name. The definition
# belongs to mote_bringup.identity, but the fleet server has to validate ids
# without importing the robot software, so it is restated here and
# ``test_protocol.py`` fails if the two ever diverge.
ID_RE = re.compile(r"^[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?$")


def valid_id(robot_id: str) -> bool:
    """Is this a usable robot id? Shape *and* not a reserved topic level."""
    return bool(ID_RE.match(robot_id or "")) and robot_id not in RESERVED_IDS


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
    """``(robot_id, leaf)`` for a v1 robot topic, else None.

    A registry topic answers None: it is a v1 topic, but it is not *about* a
    robot, and a consumer that took ``registry`` for a robot id would invent a
    fleet member out of a map announcement.
    """
    match = TOPIC_RE.match(name)
    if not match or match.group(1) in RESERVED_IDS:
        return None
    return match.group(1), match.group(2)


def registry_topic(site: str, floor: str) -> str:
    """Where a floor's canonical map revision is announced, retained."""
    return f"{ROOT}/{VERSION}/{REGISTRY}/site/{site}/floor/{floor}/{CURRENT}"


def any_floor() -> str:
    """The subscription that matches every floor's canonical revision."""
    return f"{ROOT}/{VERSION}/{REGISTRY}/site/+/floor/+/{CURRENT}"


def parse_registry_topic(name: str) -> tuple[str, str] | None:
    """``(site, floor)`` for a registry ``current`` topic, else None."""
    match = REGISTRY_TOPIC_RE.match(name)
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
    if kind in SPEC_PAYLOADS:
        raise ProtocolError(
            f"{kind} is a specification payload; check it with "
            "mote_bringup.spec, not with this module"
        )
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
    mission: dict | None = None,
    site: str | None = None,
    floor: str | None = None,
    version: str | None = None,
    uptime_s: float | None = None,
    battery: dict | None = None,
    map: dict | None = None,
) -> dict:
    """The rolled-up robot health snapshot.

    ``state``/``summary``/``subsystems`` come straight from the health monitor's
    ``/diagnostics_agg`` roll-up, so the fleet sees exactly what the robot sees.
    ``battery`` is in the contract but always null today: nothing on the robot
    can measure it (the power bank exposes no telemetry — see fleet.md), and a
    field a dashboard can render as "unknown" is better than one added later.

    ``map`` is which map revision this robot is actually running (M4). It is
    reported rather than assumed because the registry's canonical revision is
    what a floor *should* be on, and the difference between the two is the only
    way to see a robot that has not picked up a new map.
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
        "mission": mission,
        "site": site,
        "floor": floor,
        "version": version,
        "uptime_s": None if uptime_s is None else round(uptime_s, 1),
        "battery": battery,
        "map": map,
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


def current(
    site: str,
    floor: str,
    revision: str,
    *,
    url: str,
    sha256: str = "",
    bytes_: int = 0,
    promoted_by: str = "",
) -> dict:
    """A floor's canonical map revision, published retained by the registry.

    This is the whole of map distribution's control channel: the revision id a
    robot should be running, and where to fetch it. Retained, so a robot that
    was off during the mapping session is told the moment it reconnects rather
    than at the next poll — and because a revision directory is immutable and
    published by an atomic symlink flip (``sites.py``), acting on it can never
    leave a half-installed map visible.

    ``sha256`` is what the puller checks the download against; ``url`` is
    relative to the fleet server so the same retained message stays correct
    when the box is reached by a different name.
    """
    return {
        "schema": SCHEMA,
        "site": site,
        "floor": floor,
        "revision": revision,
        "url": url,
        "sha256": sha256,
        "bytes": int(bytes_),
        "promoted_by": promoted_by,
        "stamp": now(),
    }

"""The wire contract holds, and its three statements of itself agree.

The contract exists in three places on purpose — the code that builds payloads,
the JSON Schema files a consumer reads, and the prose in
docs/fleet/control-plane.md — and the failure mode of that arrangement is drift.
These tests are what stops it: the schema files are checked against
``protocol.REQUIRED`` and against what the builders actually emit, so adding a
field to a payload without describing it fails here rather than in someone's
dashboard six months later.

From v2 this covers the *telemetry* half only. The mission payloads are
mission/v0's, built and checked by ``mote_bringup.spec.mission``, and Mote
publishes no schema mirror of them at all — the specification's own schemas are
the authority, and a copy here would be a copy to keep in step. What this file
still asserts about them is the one thing that *is* Mote's: that they travel on
the right topics, and that this module refuses to pretend it owns them.
"""

import json
from pathlib import Path

import pytest

from mote_fleet import protocol

SCHEMA_DIR = Path(__file__).resolve().parents[1] / "schema"

KIND_FILES = {
    protocol.PRESENCE: "presence.schema.json",
    protocol.HEALTH: "health.schema.json",
    protocol.POSE: "pose.schema.json",
    protocol.CURRENT: "current.schema.json",
}


def sample(kind: str) -> dict:
    """One fully-populated payload of each kind, straight from the builders."""
    if kind == protocol.PRESENCE:
        return protocol.presence("mote-01", True, version="v0")
    if kind == protocol.HEALTH:
        return protocol.health(
            "mote-01",
            protocol.OK,
            "OK",
            [protocol.subsystem("lidar", protocol.OK, "ok")],
            mission={
                "id": "abc",
                "capability": "goto",
                "state": "accepted",
                "lane": "default",
            },
            site="home",
            floor="ground",
            version="v0",
            uptime_s=12.34,
            map={"site": "home", "floor": "ground", "revision": "20260727T101500"},
        )
    if kind == protocol.POSE:
        return protocol.pose("mote-01", 1.0, 2.0, 0.5, site="home", floor="ground")
    if kind == protocol.CURRENT:
        return protocol.current(
            "home",
            "ground",
            "20260727T101500",
            url="/v1/sites/home/floors/ground/revisions/20260727T101500/bundle.tar.gz",
            sha256="sha256:" + "ab" * 32,
            bytes_=4096,
            promoted_by="michael",
        )
    raise AssertionError(f"no sample for {kind!r}")


# ---- topic tree ---------------------------------------------------------


def test_topic_carries_the_contract_version():
    assert protocol.topic("mote-01", protocol.HEALTH) == "mote/v2/mote-01/health"
    assert protocol.any_robot(protocol.POSE) == "mote/v2/+/pose"


def test_parse_topic_round_trips_every_leaf():
    for leaf in (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS):
        assert protocol.parse_topic(protocol.topic("mote-02", leaf)) == (
            "mote-02",
            leaf,
        )


def test_parse_topic_rejects_a_foreign_tree():
    assert protocol.parse_topic("mote/v1/mote-01/health") is None
    assert protocol.parse_topic("other/v2/mote-01/health") is None
    assert protocol.parse_topic("mote/v2/mote-01") is None


# ---- the map registry's subtree -----------------------------------------


def test_the_registry_topic_round_trips():
    topic = protocol.registry_topic("home", "ground")
    assert topic == "mote/v2/registry/site/home/floor/ground/current"
    assert protocol.parse_registry_topic(topic) == ("home", "ground")
    assert protocol.any_floor() == "mote/v2/registry/site/+/floor/+/current"


def test_the_registry_subtree_is_not_a_robot():
    """A consumer that read ``registry`` as a robot id would invent a fleet
    member out of a map announcement — and a robot allocated that id would
    publish its health into the registry's subtree."""
    assert protocol.parse_topic(protocol.registry_topic("home", "ground")) is None
    assert not protocol.valid_id("registry")
    assert protocol.parse_registry_topic("mote/v2/mote-01/health") is None


def test_robot_id_shape_matches_identity():
    """The server validates ids without importing the robot software, so the
    two definitions are duplicated — and must not drift."""
    # Skipped in the ROS-free fleet-server environment, where there is no
    # mote_bringup to compare against; it runs in the robot/CI environment.
    identity = pytest.importorskip("mote_bringup.identity")

    assert protocol.ID_RE.pattern == identity.ID_RE.pattern
    assert protocol.valid_id("mote-01")
    assert not protocol.valid_id("Mote-01")
    assert not protocol.valid_id("mote_01")
    assert not protocol.valid_id("-mote")
    assert not protocol.valid_id("")
    assert not protocol.valid_id("mote/01")  # would split the topic tree


# ---- payloads -----------------------------------------------------------


@pytest.mark.parametrize("kind", sorted(KIND_FILES))
def test_builders_satisfy_their_own_contract(kind):
    protocol.check(sample(kind), kind)


@pytest.mark.parametrize("kind", sorted(KIND_FILES))
def test_encode_decode_round_trip(kind):
    payload = sample(kind)
    assert protocol.decode(protocol.encode(payload), kind) == payload


def test_check_rejects_a_missing_field():
    payload = sample(protocol.HEALTH)
    del payload["summary"]
    with pytest.raises(protocol.ProtocolError, match="missing summary"):
        protocol.check(payload, protocol.HEALTH)


def test_check_rejects_a_foreign_schema_version():
    payload = sample(protocol.HEALTH)
    payload["schema"] = 99
    with pytest.raises(protocol.ProtocolError, match="expected 1"):
        protocol.check(payload, protocol.HEALTH)


def test_a_specification_payload_is_not_this_modules_to_check():
    """Two checkers for one payload is two contracts, and the one that is not
    the specification's would be the one that drifted."""
    for leaf in protocol.SPEC_PAYLOADS:
        with pytest.raises(protocol.ProtocolError, match="mote_bringup.spec"):
            protocol.check({"schema": 1}, leaf)


def test_the_mission_leaves_are_where_the_binding_says():
    """The specification's MQTT binding names these three leaves; Mote's tree
    puts them under its own root and version, and nowhere else."""
    assert protocol.topic("mote-01", protocol.COMMAND) == (
        "mote/v2/mote-01/mission/command"
    )
    assert protocol.topic("mote-01", protocol.STATUS) == (
        "mote/v2/mote-01/mission/status"
    )
    assert protocol.topic("mote-01", protocol.CANCEL) == (
        "mote/v2/mote-01/mission/cancel"
    )
    assert protocol.any_robot(protocol.CAPABILITIES) == "mote/v2/+/capabilities"


def test_decode_rejects_non_json_and_non_objects():
    with pytest.raises(protocol.ProtocolError, match="not JSON"):
        protocol.decode(b"{nope")
    with pytest.raises(protocol.ProtocolError, match="not a JSON object"):
        protocol.decode(b"[1, 2]")


def test_unknown_states_are_refused_at_the_source():
    with pytest.raises(protocol.ProtocolError):
        protocol.health("mote-01", "fine", "", [])


def test_pose_carries_the_frame_it_is_meaningful_in():
    payload = protocol.pose("mote-01", 1.0, 2.0, 0.5, site="home", floor="ground")
    # A map-frame coordinate without its floor is unplottable: the frame origin
    # is wherever SLAM happened to start.
    assert (payload["site"], payload["floor"]) == ("home", "ground")
    assert payload["frame_id"] == "map"


def test_the_map_a_robot_is_running_is_reported_not_assumed():
    """The registry says what a floor *should* be on; only the robot can say
    what it is actually running, and the gap is the point of the field."""
    assert sample(protocol.HEALTH)["map"]["revision"] == "20260727T101500"
    assert protocol.health("mote-01", protocol.OK, "", [])["map"] is None


def test_battery_is_present_and_null():
    """Reserved in the contract, unmeasurable on the robot (fleet.md)."""
    assert sample(protocol.HEALTH)["battery"] is None


def test_stamps_are_rfc3339_utc():
    stamp = protocol.now()
    assert stamp.endswith("Z")
    from datetime import datetime

    datetime.fromisoformat(stamp.replace("Z", "+00:00"))


# ---- the JSON Schema mirror --------------------------------------------


@pytest.mark.parametrize("kind,filename", sorted(KIND_FILES.items()))
def test_schema_file_required_matches_the_code(kind, filename):
    schema = json.loads((SCHEMA_DIR / filename).read_text())
    assert set(schema["required"]) == set(protocol.REQUIRED[kind])


@pytest.mark.parametrize("kind,filename", sorted(KIND_FILES.items()))
def test_schema_file_describes_every_field_the_builder_emits(kind, filename):
    schema = json.loads((SCHEMA_DIR / filename).read_text())
    undescribed = set(sample(kind)) - set(schema["properties"])
    assert not undescribed, f"{filename} does not describe {sorted(undescribed)}"


@pytest.mark.parametrize("kind,filename", sorted(KIND_FILES.items()))
def test_schema_file_pins_the_contract_version(kind, filename):
    schema = json.loads((SCHEMA_DIR / filename).read_text())
    assert schema["properties"]["schema"]["const"] == protocol.SCHEMA
    assert schema["$id"].endswith(f"/{protocol.VERSION}/{filename}")


def test_schema_enums_match_the_code():
    health = json.loads((SCHEMA_DIR / "health.schema.json").read_text())
    assert set(health["properties"]["state"]["enum"]) == set(protocol.HEALTH_STATES)


def test_the_health_summary_admits_every_mission_state():
    """The roster's "what is it doing" line is a summary of a mission/v0
    mission, so a state added there must not read as invalid here."""
    from mote_bringup.spec import mission

    health = json.loads((SCHEMA_DIR / "health.schema.json").read_text())
    states = health["properties"]["mission"]["properties"]["state"]["enum"]
    assert set(states) == set(mission.STATES)


def test_no_schema_file_survives_for_a_specification_payload():
    """A stale mirror of a payload Mote no longer defines would be read as
    authoritative by the next person who found it."""
    names = {path.name for path in SCHEMA_DIR.glob("*.schema.json")}
    assert not names & {"command.schema.json", "status.schema.json"}

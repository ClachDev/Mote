"""The wire contract holds, and its three statements of itself agree.

The contract exists in three places on purpose — the code that builds payloads,
the JSON Schema files a consumer reads, and the prose in
docs/fleet/control-plane.md — and the failure mode of that arrangement is drift.
These tests are what stops it: the schema files are checked against
``protocol.REQUIRED`` and against what the builders actually emit, so adding a
field to a payload without describing it fails here rather than in someone's
dashboard six months later.
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
    protocol.COMMAND: "command.schema.json",
    protocol.STATUS: "status.schema.json",
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
            task={"id": "abc", "command": "goto kitchen", "state": "accepted"},
            site="home",
            floor="ground",
            version="v0",
            uptime_s=12.34,
        )
    if kind == protocol.POSE:
        return protocol.pose("mote-01", 1.0, 2.0, 0.5, site="home", floor="ground")
    if kind == protocol.COMMAND:
        return protocol.command("goto kitchen", issued_by="tester")
    return protocol.status("mote-01", "abc", "goto kitchen", protocol.ACCEPTED)


# ---- topic tree ---------------------------------------------------------


def test_topic_carries_the_contract_version():
    assert protocol.topic("mote-01", protocol.HEALTH) == "mote/v1/mote-01/health"
    assert protocol.topic("mote-01", protocol.COMMAND) == "mote/v1/mote-01/task/command"
    assert protocol.any_robot(protocol.POSE) == "mote/v1/+/pose"


def test_parse_topic_round_trips_every_leaf():
    for leaf in (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS):
        assert protocol.parse_topic(protocol.topic("mote-02", leaf)) == (
            "mote-02",
            leaf,
        )


def test_parse_topic_rejects_a_foreign_tree():
    assert protocol.parse_topic("mote/v2/mote-01/health") is None
    assert protocol.parse_topic("other/v1/mote-01/health") is None
    assert protocol.parse_topic("mote/v1/mote-01") is None


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
    payload = sample(protocol.STATUS)
    payload["schema"] = 99
    with pytest.raises(protocol.ProtocolError, match="expected 1"):
        protocol.check(payload, protocol.STATUS)


def test_decode_rejects_non_json_and_non_objects():
    with pytest.raises(protocol.ProtocolError, match="not JSON"):
        protocol.decode(b"{nope")
    with pytest.raises(protocol.ProtocolError, match="not a JSON object"):
        protocol.decode(b"[1, 2]")


def test_unknown_states_are_refused_at_the_source():
    with pytest.raises(protocol.ProtocolError):
        protocol.status("mote-01", "abc", "goto x", "in-progress")
    with pytest.raises(protocol.ProtocolError):
        protocol.health("mote-01", "fine", "", [])


def test_terminal_flag_matches_the_state_machine():
    for state in protocol.TASK_STATES:
        payload = protocol.status("mote-01", "abc", "goto x", state)
        assert payload["terminal"] == (state in protocol.TERMINAL_STATES)


def test_pose_carries_the_frame_it_is_meaningful_in():
    payload = protocol.pose("mote-01", 1.0, 2.0, 0.5, site="home", floor="ground")
    # A map-frame coordinate without its floor is unplottable: the frame origin
    # is wherever SLAM happened to start.
    assert (payload["site"], payload["floor"]) == ("home", "ground")
    assert payload["frame_id"] == "map"


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
    assert schema["$id"].endswith(f"/v1/{filename}")


def test_schema_enums_match_the_code():
    health = json.loads((SCHEMA_DIR / "health.schema.json").read_text())
    status = json.loads((SCHEMA_DIR / "status.schema.json").read_text())
    assert set(health["properties"]["state"]["enum"]) == set(protocol.HEALTH_STATES)
    assert set(status["properties"]["state"]["enum"]) == set(protocol.TASK_STATES)
    assert set(status["properties"]["source"]["enum"]) == {
        protocol.SOURCE_FLEET,
        protocol.SOURCE_LOCAL,
    }

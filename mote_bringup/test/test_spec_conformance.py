"""Mote's real payloads, against the specification's own JSON Schemas.

Mote deliberately does **not** vendor those schemas. A vendored copy is a copy,
and the first time the spec moves it becomes a test that passes against a
document nobody publishes any more. So the authority stays where it is written,
and this test resolves it from a checkout: ``$AUGEREAI_SPEC``, or a sibling
``augereai-spec`` directory beside this repository.

It therefore skips on a machine that has neither that checkout nor
``jsonschema`` — which is most of them, and is the same bargain the broker and
node tests take. What it is *for* is the machine that has both: it is the only
check that Mote's payloads are conforming rather than merely self-consistent,
and everything it covers is built by the same functions the robot builds it
with, never hand-written for the test.
"""

import json
import os
from pathlib import Path

import pytest

from mote_bringup.spec import capability as cap
from mote_bringup.spec import mission
from mote_bringup.spec import zone

# The skip is deliberately *not* a module-level ``pytest.importorskip``. That
# raises ``Skipped``, which derives from ``BaseException``, and the
# launch_testing plugin this workspace loads imports every test module itself
# outside pytest's skip handling — so one module-level skip aborted collection
# of the entire directory, silently, reporting "no tests collected" rather than
# an error. Skipping per test keeps it a test-level outcome.


def validator_library():
    try:
        import jsonschema
    except ImportError:
        pytest.skip("no JSON Schema validator")
    return jsonschema


def spec_root() -> Path:
    named = os.environ.get("AUGEREAI_SPEC")
    candidates = [Path(named)] if named else []
    candidates.append(Path(__file__).resolve().parents[3] / "augereai-spec")
    for path in candidates:
        if (path / "schema").is_dir():
            return path
    pytest.skip("no augereai-spec checkout (set AUGEREAI_SPEC)")


@pytest.fixture(scope="module")
def validator_for():
    """A validator factory whose ``$ref``s resolve to the local checkout.

    The spec's ``$id``s are stable identifiers under ``https://spec.augereai.com/``
    and are not yet dereferenceable, but the directory layout mirrors the URI
    path exactly — so a resolver is a prefix substitution, and a test that
    reached the network to check a payload would be a test that fails on a
    train.
    """
    jsonschema = validator_library()
    from referencing import Registry, Resource

    root = spec_root() / "schema"
    prefix = "https://spec.augereai.com/"
    store = {}
    for path in root.rglob("*.schema.json"):
        document = json.loads(path.read_text())
        store[document["$id"]] = document
    registry = Registry().with_resources(
        (uri, Resource.from_contents(document)) for uri, document in store.items()
    )

    def make(name: str):
        return jsonschema.validators.Draft202012Validator(
            store[prefix + name], registry=registry
        )

    return make


def a_goto() -> dict:
    return cap.capability(
        "goto",
        version="1.0.0",
        display_name="Go to zone",
        summary="Drive to a named zone and stop there.",
        input_schema={
            "type": "object",
            "required": ["target"],
            "additionalProperties": False,
            "properties": {"target": cap.zone_ref()},
        },
        execution=cap.execution(cancellable=True, max_duration_s=600),
        preconditions=[
            cap.precondition("localized"),
            cap.precondition("zone_known", input_pointer="/target"),
        ],
        safety=cap.safety(
            motion="drives",
            reversible=True,
            emergency_stop="halts",
            hazards=["unexpected_motion"],
            max_speed_mps=0.22,
        ),
    )


def test_a_capability_set_conforms(validator_for):
    document = cap.capability_set(
        "mote-01", [a_goto()], platform_type="amr", revision=1
    )
    validator_for("capability/v0/capability-set.schema.json").validate(document)


def test_a_capability_conforms(validator_for):
    validator_for("capability/v0/capability.schema.json").validate(a_goto())


def test_every_mission_state_conforms(validator_for):
    """Including the ones Mote does not emit: the module can build them, so a
    later milestone adopting ``running`` starts from a checked shape."""
    validator = validator_for("mission/v0/mission-status.schema.json")
    for state in mission.STATES:
        extra = {}
        if state in mission.FAILED_STATES:
            extra["failure"] = mission.failure(mission.INTERNAL, "a defect")
        validator.validate(mission.status("mote-01", "abc", "goto", state, **extra))


def test_every_failure_class_conforms(validator_for):
    validator = validator_for("mission/v0/failure.schema.json")
    for kind in mission.FAILURE_CLASSES:
        validator.validate(
            mission.failure(kind, "detail", recoverable=False, at="accepted")
        )


def test_a_command_and_a_cancel_conform(validator_for):
    validator_for("mission/v0/mission-command.schema.json").validate(
        mission.command(
            "mote-01", "goto", {"target": "kitchen"}, issued_by="ui:michael"
        )
    )
    validator_for("mission/v0/mission-cancel.schema.json").validate(
        mission.cancel("mote-01", "abc", reason="operator changed their mind")
    )


def test_a_local_mission_status_conforms(validator_for):
    """``id: null`` is the shape a platform reports a mission it started
    itself with, and the one a schema is most likely to have forbidden."""
    validator_for("mission/v0/mission-status.schema.json").validate(
        mission.status(
            "mote-01", None, "goto", mission.ACCEPTED, source=mission.SOURCE_LOCAL
        )
    )


# -- zone/v0 ---------------------------------------------------------------


def a_floor():
    """One floor's zones, split — the migration a real ``zones.yaml`` takes."""
    return zone.split(
        {
            "frame_id": "map",
            "revision": 4,
            "zones": {
                "kitchen": {
                    "x": 2.0,
                    "y": 3.5,
                    "yaw": 1.57,
                    "radius": 1.5,
                    "kind": "room",
                    "display_name": "Kitchen",
                    "aliases": ["the kitchen", "galley"],
                },
                "ward_a": {"polygon": [[4, 0], [9, 0], [9, 3], [4, 3]], "kind": "room"},
                "server_room": {"x": 1.0, "y": 1.0, "kind": "keepout"},
            },
        },
        site="acme_hq",
        floor="ground",
        platform_id="mote-01",
        map_revision="2026-07-24T09-12-03",
    )


def test_a_vocabulary_conforms_and_carries_no_coordinates(validator_for):
    document, _ = a_floor()
    validator_for("zone/v0/zone-vocabulary.schema.json").validate(document)
    # The invariant, checked over the whole payload rather than over the keys
    # someone thought of: a geometry key here is the leak the split exists to
    # prevent, and it would look like a plausible coordinate rather than a crash.
    text = json.dumps(document)
    for key in zone.GEOMETRY_KEYS + ("frame_id", "map_revision", "pose", "footprint"):
        assert f'"{key}"' not in text, f"{key} leaked into the vocabulary"


def test_a_binding_conforms(validator_for):
    _, document = a_floor()
    validator_for("zone/v0/zone-binding.schema.json").validate(document)
    assert document["platform_id"] == "mote-01"
    assert document["map_revision"] == "2026-07-24T09-12-03"


def test_every_resolution_reason_conforms(validator_for):
    validator = validator_for("zone/v0/zone-resolution.schema.json")
    for reason in zone.REASONS:
        validator.validate(
            zone.resolution(
                "mote-01", "kitchen", reason=reason, queried_as="the kitchen"
            )
        )
    validator.validate(
        zone.resolution(
            "mote-01",
            "kitchen",
            resolved=True,
            site="acme_hq",
            floor="ground",
            frame_id="map",
            map_revision="2026-07-24T09-12-03",
            pose={"x": 2.0, "y": 3.5, "yaw": 1.57},
            kind="room",
            navigable=True,
            anchor_method="taught",
        )
    )


def test_a_zone_name_conforms_to_the_reference_schema(validator_for):
    validator = validator_for("zone/v0/zone-ref.schema.json")
    validator.validate("kitchen")
    with pytest.raises(validator_library().ValidationError):
        validator.validate("The Kitchen")

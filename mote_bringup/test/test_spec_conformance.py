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

jsonschema = pytest.importorskip("jsonschema", reason="no JSON Schema validator")


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

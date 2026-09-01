"""capability/v0's rules, and the one property the input validator must have.

The validator is a subset of JSON Schema written by hand. The risk in that is
not the keyword it lacks, it is a keyword it *ignores*: a capability advertising
``maxItems`` and getting no bound promises a constraint nothing enforces, and
nothing ever says so. So the load-bearing test here is
:func:`test_an_unimplemented_keyword_raises_rather_than_being_ignored`, and it
is written against the keyword set rather than against a list someone thought
of — a keyword added to a capability without a branch in the validator fails.
"""

import pytest

from mote_bringup.spec import SpecError
from mote_bringup.spec import capability as cap


def a_capability(**overrides):
    kwargs = dict(
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
        safety=cap.safety(motion="drives", reversible=True, emergency_stop="halts"),
    )
    key = overrides.pop("key", "goto")
    kwargs.update(overrides)
    return cap.capability(key, **kwargs)


# -- the validator --------------------------------------------------------


def test_an_unimplemented_keyword_raises_rather_than_being_ignored():
    """Every keyword the validator does not implement must be refused when the
    capability is declared. Ignoring one is a promise nothing keeps."""
    for keyword, value in (
        ("uniqueItems", True),
        ("multipleOf", 2),
        ("anyOf", []),
        ("patternProperties", {}),
        ("format", "date-time"),
    ):
        assert keyword not in cap.KEYWORDS
        with pytest.raises(SpecError, match="does not implement"):
            cap.check_schema({"type": "object", "properties": {"n": {keyword: value}}})


def test_the_declared_keyword_set_is_what_the_validator_branches_on():
    """A keyword listed as supported but never acted on would be the same
    silent hole from the other direction."""
    unenforced = {"title", "description", "default", "examples"}
    for keyword in cap.KEYWORDS - unenforced - {"$ref", "type", "properties"}:
        # Each of these must be able to reject *something*.
        cases = {
            "enum": ({"enum": ["a"]}, "b"),
            "const": ({"const": "a"}, "b"),
            "required": ({"type": "object", "required": ["a"]}, {}),
            "additionalProperties": (
                {"type": "object", "additionalProperties": False},
                {"a": 1},
            ),
            "items": ({"type": "array", "items": {"type": "string"}}, [1]),
            "minItems": ({"type": "array", "minItems": 2}, [1]),
            "maxItems": ({"type": "array", "maxItems": 1}, [1, 2]),
            "pattern": ({"type": "string", "pattern": "^a$"}, "b"),
            "minLength": ({"type": "string", "minLength": 2}, "a"),
            "maxLength": ({"type": "string", "maxLength": 1}, "ab"),
            "minimum": ({"type": "number", "minimum": 2}, 1),
            "maximum": ({"type": "number", "maximum": 1}, 2),
            "exclusiveMinimum": ({"type": "number", "exclusiveMinimum": 1}, 1),
            "exclusiveMaximum": ({"type": "number", "exclusiveMaximum": 1}, 1),
        }
        schema, value = cases[keyword]
        with pytest.raises(cap.InvalidInput):
            cap.validate_input(schema, value)


def test_a_boolean_is_not_a_number():
    """Python says ``isinstance(True, int)``; JSON does not. A flag arriving
    where a quantity belongs is bad input, not a zero."""
    with pytest.raises(cap.InvalidInput):
        cap.validate_input({"type": "integer"}, True)
    cap.validate_input({"type": "boolean"}, True)


def test_the_zone_ref_is_resolved_and_an_unknown_ref_is_not():
    cap.validate_input(cap.zone_ref(), "kitchen")
    with pytest.raises(cap.InvalidInput):
        cap.validate_input(cap.zone_ref(), "The Kitchen")
    with pytest.raises(SpecError, match="cannot resolve"):
        cap.validate_input({"$ref": "https://example.invalid/x.json"}, "a")


def test_zone_inputs_names_the_properties_a_dispatcher_must_pre_check():
    schema = {
        "type": "object",
        "properties": {
            "target": cap.zone_ref(),
            "drop": cap.zone_ref(),
            "n": {"type": "integer"},
        },
    }
    assert cap.zone_inputs(schema) == ["drop", "target"]


def test_bad_input_and_a_bad_schema_are_different_exceptions():
    """One is the caller's fault and becomes ``invalid_input`` on the wire; the
    other is this platform's and must never be reported as the caller's."""
    assert not issubclass(cap.InvalidInput, SpecError)
    assert not issubclass(SpecError, cap.InvalidInput)


# -- the declaration ------------------------------------------------------


def test_an_unprefixed_key_must_be_a_standard_one():
    a_capability(key="goto")
    a_capability(key="x_mote_calibrate")
    with pytest.raises(SpecError, match="standard key"):
        a_capability(key="wander")


def test_an_unbounded_capability_must_be_cancellable():
    with pytest.raises(SpecError, match="cancellable"):
        cap.execution(max_duration_s=None, cancellable=False)
    cap.execution(max_duration_s=None, cancellable=True)


def test_a_control_key_belongs_on_the_control_lane():
    with pytest.raises(SpecError, match="control lane"):
        a_capability(
            key="pause",
            input_schema={"type": "object", "properties": {}},
            execution=cap.execution(mode="immediate", cancellable=True),
        )


def test_a_restricted_capability_must_justify_itself_and_always_audit():
    with pytest.raises(SpecError, match="justification"):
        cap.safety(motion="drives", reversible=True, restricted=True)
    with pytest.raises(SpecError, match="audit"):
        a_capability(
            safety=cap.safety(
                motion="drives",
                reversible=True,
                restricted=True,
                justification="drives with a payload over people",
            ),
            auth=cap.auth(audit="never"),
        )


def test_a_precondition_pointer_must_name_a_real_input_property():
    """A pointer at nothing reads as "the zone is fine" on every mission —
    it can only be caught here, at declaration."""
    with pytest.raises(SpecError, match="names no property"):
        a_capability(
            preconditions=[cap.precondition("zone_known", input_pointer="/dest")]
        )
    a_capability(
        preconditions=[cap.precondition("zone_known", input_pointer="/target")]
    )


def test_a_precondition_that_needs_a_field_must_carry_it():
    with pytest.raises(SpecError, match="input_pointer"):
        cap.precondition("zone_known")
    with pytest.raises(SpecError, match="description"):
        cap.precondition("custom")
    assert cap.precondition("custom", description="the bay door is open")


def test_a_capability_set_refuses_duplicate_keys():
    one = a_capability()
    with pytest.raises(SpecError, match="duplicate"):
        cap.capability_set("mote-01", [one, one])
    document = cap.capability_set("mote-01", [one], platform_type="amr", revision=3)
    assert cap.find(document, "goto") == one
    assert cap.find(document, "dock") is None


def test_an_input_schema_must_be_an_object_schema():
    """A capability taking a bare string could never grow a second parameter
    without a major bump; an empty object can."""
    with pytest.raises(SpecError, match="object schema"):
        a_capability(input_schema={"type": "string"})
    a_capability(input_schema={"type": "object", "properties": {}})

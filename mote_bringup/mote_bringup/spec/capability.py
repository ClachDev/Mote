"""capability/v0 — what a platform can be asked to do, typed.

A **capability** is one invocable action: ``goto``, ``fetch``. Not a feature
("has an arm"), not a mode, not a topic — the test is whether a :mod:`mission
<mote_bringup.spec.mission>` can name it and get a terminal answer back. A
platform's whole offering is a **capability set**, one document with one
``revision``, republished when it changes; a consumer holding a higher revision
ignores a lower one, so an out-of-order redelivery is harmless rather than a
downgrade.

This module builds and checks those documents, and validates a mission's
``input`` against the capability's own ``input_schema``. What Mote actually
advertises is :mod:`mote_tasks.capabilities`, which is where the two keys it
implements are declared; nothing here knows about ``goto`` beyond the standard
registry's constraints on it.

**The input validator is a deliberate subset, and it refuses what it does not
implement.** A general JSON Schema engine is a dependency this package does not
have — the fleet server's container installs no framework, and the robot's
image is not the place to add one for two schemas with four keywords between
them. The danger in a hand-written validator is not the missing keyword, it is
the missing keyword that is *ignored*: a capability declaring ``maxItems`` and
getting no bound would advertise a promise nothing keeps, silently, forever. So
:func:`validate_input` raises :class:`~mote_bringup.spec.SpecError` on any
keyword it does not implement — a defect in the capability, surfaced when that
capability is declared and its tests run, rather than a hole in the field. The
supported subset is :data:`KEYWORDS`; growing it is adding a branch and a test,
and growing it is expected.
"""

import math
import re

from mote_bringup.spec import SpecError

SCHEMA = 1
VERSION = "v0"

#: ``^[a-z][a-z0-9_]*$``, unique within a capability set. **Unprefixed keys are
#: reserved for the standard registry**: a dispatcher seeing ``goto`` knows the
#: input shape without reading anyone's documentation, which is only true if a
#: vendor extension cannot take the name. Extensions are ``x_``-prefixed.
KEY_RE = re.compile(r"^[a-z][a-z0-9_]*$")

VENDOR_PREFIX = "x_"

#: The reserved keys. A platform advertising one of these must accept the
#: registry's input schema and must not contradict its safety fields.
STANDARD_KEYS = (
    "goto",
    "fetch",
    "dock",
    "undock",
    "return_home",
    "pause",
    "resume",
    "estop",
)

#: The keys the registry puts on the ``control`` lane, so a stop stays
#: dispatchable while a mission holds ``default``. That is the whole reason
#: lanes exist in v0.
CONTROL_KEYS = frozenset({"pause", "resume", "estop"})

SEMVER_RE = re.compile(
    r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(-[0-9A-Za-z.-]+)?$"
)

#: How a capability behaves in time. ``immediate`` completes in seconds with one
#: terminal state; ``mission`` is long-running and reports the lifecycle.
MODES = ("immediate", "mission")

#: ``natural`` — re-running reaches the same end state. ``key`` — safe to retry
#: only when deduplicated by mission id. ``none`` — every run is a distinct
#: physical effect, and **no layer may retry automatically**, whatever a
#: failure's ``recoverable`` says: recoverable is about the world cooperating,
#: idempotency is about a retry costing a second effect.
IDEMPOTENCY = ("natural", "key", "none")

MOTION = ("none", "stationary", "drives", "manipulates", "drives_and_manipulates")
SUPERVISION = ("none", "recommended", "required")
EMERGENCY_STOP = ("halts", "holds", "not_applicable")
AUDIT = ("always", "on_failure", "never")

HAZARDS = (
    "pinch",
    "crush",
    "sharp",
    "thermal",
    "chemical",
    "electrical",
    "laser",
    "high_speed",
    "payload_drop",
    "unexpected_motion",
)

#: Typed preconditions, so an executor can evaluate them and a planner can
#: avoid dispatching something that will bounce. The value is the extra key
#: each type requires.
PRECONDITION_TYPES = {
    "localized": None,
    "zone_known": "input_pointer",
    "zone_reachable": "input_pointer",
    "component_present": "component_kind",
    "platform_idle": None,
    "battery_above": "min_percent",
    "operator_present": None,
    "custom": "description",
}

#: The schema a location-taking input must ``$ref``. It is a schema rather than
#: a convention so a tool reading a capability set can tell *mechanically*
#: which inputs are places — and so a dispatcher can pre-check them against the
#: zone vocabulary before sending anything.
ZONE_REF = "https://spec.augereai.com/zone/v0/zone-ref.schema.json"

#: The one ``$ref`` target this validator resolves, inlined because the spec's
#: URIs are stable identifiers and not yet dereferenceable over HTTP.
REFS = {ZONE_REF: {"type": "string", "pattern": r"^[a-z][a-z0-9_]*$", "maxLength": 64}}


def zone_ref() -> dict:
    """An input property that names a place."""
    return {"$ref": ZONE_REF}


# -- the bounded input validator ------------------------------------------

#: Every JSON Schema keyword :func:`validate_input` implements. Anything else
#: in a capability's ``input_schema`` raises, rather than being ignored.
KEYWORDS = frozenset(
    {
        "$ref",
        "type",
        "enum",
        "const",
        "properties",
        "required",
        "additionalProperties",
        "items",
        "minItems",
        "maxItems",
        "pattern",
        "minLength",
        "maxLength",
        "minimum",
        "maximum",
        "exclusiveMinimum",
        "exclusiveMaximum",
        # Annotations: carried for a reader, meaningless to a validator.
        "title",
        "description",
        "default",
        "examples",
    }
)

_TYPES = {
    "object": dict,
    "array": list,
    "string": str,
    "boolean": bool,
    "number": (int, float),
    "integer": int,
    "null": type(None),
}


class InvalidInput(ValueError):
    """A mission's ``input`` that does not match its capability's schema.

    Distinct from :class:`~mote_bringup.spec.SpecError` on purpose: this is bad
    input from a caller and becomes a ``rejected`` status with failure class
    ``invalid_input``, whereas a SpecError here is a defect in the capability
    declaration and must not be reported as the caller's fault.
    """


def _is(value, name: str) -> bool:
    wanted = _TYPES[name]
    if name in ("number", "integer") and isinstance(value, bool):
        return False  # JSON has no bool-is-int; a flag is not a quantity
    if name == "number" and isinstance(value, float) and not math.isfinite(value):
        return False
    return isinstance(value, wanted)


def check_schema(schema: dict, where: str = "input_schema") -> None:
    """Walk a schema and raise on any keyword this validator does not implement.

    Called when a capability is *declared*, so an unenforceable constraint is a
    failing test on the platform that wrote it rather than a promise nothing
    keeps. Validating a specimen value would not do: a keyword is only reached
    when a value of that type turns up, so ``maxItems`` on an optional array
    would go unnoticed until the first mission that used it.
    """
    if not isinstance(schema, dict):
        raise SpecError(f"{where}: schema is not an object")
    unknown = sorted(set(schema) - KEYWORDS)
    if unknown:
        raise SpecError(
            f"{where}: schema uses {', '.join(unknown)}, which this validator "
            f"does not implement (supported: {', '.join(sorted(KEYWORDS))})"
        )
    ref = schema.get("$ref")
    if ref is not None and ref not in REFS:
        raise SpecError(f"{where}: cannot resolve $ref {ref!r}")
    for name in (
        [schema["type"]]
        if isinstance(schema.get("type"), str)
        else (schema.get("type") or ())
    ):
        if name not in _TYPES:
            raise SpecError(f"{where}: unknown type {name!r}")
    for key, sub in (schema.get("properties") or {}).items():
        check_schema(sub, f"{where}.{key}")
    if schema.get("items") is not None:
        check_schema(schema["items"], f"{where}[]")


def validate_input(schema: dict, value, where: str = "input") -> None:
    """Check ``value`` against ``schema``, raising :class:`InvalidInput`.

    Raises :class:`~mote_bringup.spec.SpecError` instead when the *schema* asks
    for something this validator does not implement — see the module docstring
    for why that is loud rather than lenient.
    """
    if not isinstance(schema, dict):
        raise SpecError(f"{where}: schema is not an object")
    unknown = sorted(set(schema) - KEYWORDS)
    if unknown:
        raise SpecError(
            f"{where}: schema uses {', '.join(unknown)}, which this validator "
            f"does not implement (supported: {', '.join(sorted(KEYWORDS))})"
        )
    ref = schema.get("$ref")
    if ref is not None:
        if ref not in REFS:
            raise SpecError(f"{where}: cannot resolve $ref {ref!r}")
        validate_input(REFS[ref], value, where)
        return

    declared = schema.get("type")
    if declared is not None:
        names = [declared] if isinstance(declared, str) else list(declared)
        for name in names:
            if name not in _TYPES:
                raise SpecError(f"{where}: unknown type {name!r}")
        if not any(_is(value, name) for name in names):
            raise InvalidInput(f"{where} is not {' or '.join(names)}")

    if "const" in schema and value != schema["const"]:
        raise InvalidInput(f"{where} must be {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        allowed = ", ".join(repr(item) for item in schema["enum"])
        raise InvalidInput(f"{where} must be one of {allowed}")

    if isinstance(value, str):
        _check_string(schema, value, where)
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        _check_number(schema, value, where)
    elif isinstance(value, list):
        _check_array(schema, value, where)
    elif isinstance(value, dict):
        _check_object(schema, value, where)


def _check_string(schema: dict, value: str, where: str) -> None:
    pattern = schema.get("pattern")
    if pattern is not None and not re.search(pattern, value):
        raise InvalidInput(f"{where} {value!r} does not match {pattern}")
    if "minLength" in schema and len(value) < schema["minLength"]:
        raise InvalidInput(f"{where} is shorter than {schema['minLength']} characters")
    if "maxLength" in schema and len(value) > schema["maxLength"]:
        raise InvalidInput(f"{where} is longer than {schema['maxLength']} characters")


def _check_number(schema: dict, value, where: str) -> None:
    if "minimum" in schema and value < schema["minimum"]:
        raise InvalidInput(f"{where} is below {schema['minimum']}")
    if "maximum" in schema and value > schema["maximum"]:
        raise InvalidInput(f"{where} is above {schema['maximum']}")
    if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
        raise InvalidInput(f"{where} must be above {schema['exclusiveMinimum']}")
    if "exclusiveMaximum" in schema and value >= schema["exclusiveMaximum"]:
        raise InvalidInput(f"{where} must be below {schema['exclusiveMaximum']}")


def _check_array(schema: dict, value: list, where: str) -> None:
    if "minItems" in schema and len(value) < schema["minItems"]:
        raise InvalidInput(f"{where} needs at least {schema['minItems']} items")
    if "maxItems" in schema and len(value) > schema["maxItems"]:
        raise InvalidInput(f"{where} takes at most {schema['maxItems']} items")
    items = schema.get("items")
    if items is not None:
        for index, item in enumerate(value):
            validate_input(items, item, f"{where}[{index}]")


def _check_object(schema: dict, value: dict, where: str) -> None:
    properties = schema.get("properties") or {}
    for key in schema.get("required") or ():
        if key not in value:
            raise InvalidInput(f"{where} is missing required property {key!r}")
    if schema.get("additionalProperties") is False:
        extra = sorted(set(value) - set(properties))
        if extra:
            raise InvalidInput(f"{where} has unexpected properties: {', '.join(extra)}")
    for key, sub in properties.items():
        if key in value:
            validate_input(sub, value[key], f"{where}.{key}")


def zone_inputs(schema: dict) -> list[str]:
    """The top-level input properties that name a place.

    What makes the zone/capability seam machine-readable: a dispatcher pre-checks
    exactly these against the vocabulary, without a table of which capability
    takes places where.
    """
    properties = (schema or {}).get("properties") or {}
    return sorted(
        key for key, sub in properties.items() if (sub or {}).get("$ref") == ZONE_REF
    )


# -- the documents --------------------------------------------------------


def execution(
    *,
    mode: str = "mission",
    cancellable: bool = False,
    idempotency: str = "natural",
    max_duration_s: float | None = None,
    lane: str = "default",
) -> dict:
    if mode not in MODES:
        raise SpecError(f"unknown execution mode {mode!r}")
    if idempotency not in IDEMPOTENCY:
        raise SpecError(f"unknown idempotency {idempotency!r}")
    if max_duration_s is None and not cancellable:
        # Something unbounded that cannot be stopped is not dispatchable.
        raise SpecError("a capability with no max_duration_s must be cancellable")
    return {
        "mode": mode,
        "cancellable": bool(cancellable),
        "idempotency": idempotency,
        "max_duration_s": None if max_duration_s is None else float(max_duration_s),
        "lane": lane,
    }


def precondition(kind: str, *, blocking: bool = True, **extra) -> dict:
    """One typed precondition.

    ``blocking`` is the whole point of the flag: an unmet blocking precondition
    rejects the mission with class ``precondition``, and an unmet non-blocking
    one is reported in the ``accepted`` status's ``warnings`` and execution
    proceeds. A degraded start nobody was told about is what that prevents.
    """
    if kind not in PRECONDITION_TYPES:
        raise SpecError(f"unknown precondition type {kind!r}")
    needed = PRECONDITION_TYPES[kind]
    if needed is not None and extra.get(needed) in (None, ""):
        raise SpecError(f"a {kind!r} precondition requires {needed}")
    return {"type": kind, "blocking": bool(blocking), **extra}


def safety(
    *,
    motion: str,
    reversible: bool,
    human_supervision: str = "none",
    emergency_stop: str = "not_applicable",
    hazards=(),
    max_speed_mps: float | None = None,
    max_payload_kg: float | None = None,
    restricted: bool = False,
    justification: str = "",
) -> dict:
    """Declarative safety metadata: what invoking this does in the world.

    Not a functional-safety claim, and nothing here is safety-rated. It exists
    so an operator UI can warn, a policy engine can gate and a planner can
    refuse. An empty ``hazards`` list asserts that none of the listed classes
    apply; it does not assert that the capability is safe.
    """
    if motion not in MOTION:
        raise SpecError(f"unknown motion {motion!r}")
    if human_supervision not in SUPERVISION:
        raise SpecError(f"unknown human_supervision {human_supervision!r}")
    if emergency_stop not in EMERGENCY_STOP:
        raise SpecError(f"unknown emergency_stop {emergency_stop!r}")
    unknown = sorted(set(hazards) - set(HAZARDS))
    if unknown:
        raise SpecError(f"unknown hazards: {', '.join(unknown)}")
    if restricted and not justification.strip():
        # So the flag cannot be set as reflex.
        raise SpecError("a restricted capability must carry a justification")
    return {
        "motion": motion,
        "reversible": bool(reversible),
        "human_supervision": human_supervision,
        "emergency_stop": emergency_stop,
        "hazards": list(hazards),
        "max_speed_mps": max_speed_mps,
        "max_payload_kg": max_payload_kg,
        "restricted": bool(restricted),
        "justification": justification,
    }


def auth(*, required_scopes=(), audit: str = "always") -> dict:
    if audit not in AUDIT:
        raise SpecError(f"unknown audit policy {audit!r}")
    return {"required_scopes": list(required_scopes), "audit": audit}


#: What a capability that says nothing about authorization gets. Declared as a
#: value rather than built by :func:`auth` so :func:`capability` can name it
#: without the parameter of the same name shadowing the function.
DEFAULT_AUTH = {"required_scopes": [], "audit": "always"}


def capability(
    key: str,
    *,
    version: str,
    display_name: str,
    summary: str,
    input_schema: dict,
    execution: dict,
    safety: dict,
    preconditions=(),
    auth: dict | None = None,
    result_schema: dict | None = None,
) -> dict:
    """One declared capability, checked before it can be advertised."""
    if not KEY_RE.match(key or ""):
        raise SpecError(f"capability key {key!r} must match {KEY_RE.pattern}")
    if not SEMVER_RE.match(version or ""):
        raise SpecError(f"capability version {version!r} is not semver")
    if not isinstance(input_schema, dict) or input_schema.get("type") != "object":
        # A capability taking a bare string could never grow a second parameter
        # without a major bump; an empty object can.
        raise SpecError(f"{key}: input_schema must be an object schema")
    # Prove the schema is one this platform can actually enforce, at declaration
    # time. A capability advertising a keyword the validator ignores would
    # promise a constraint nothing checks.
    check_schema(input_schema, f"{key} input_schema")
    if result_schema is not None:
        check_schema(result_schema, f"{key} result_schema")
    declared = {
        "schema": SCHEMA,
        "key": key,
        "version": version,
        "display_name": display_name,
        "summary": summary,
        "input_schema": input_schema,
        "result_schema": result_schema,
        "execution": execution,
        "preconditions": list(preconditions),
        "safety": safety,
        "auth": auth if auth is not None else DEFAULT_AUTH.copy(),
    }
    check(declared)
    return declared


def check(declared: dict) -> dict:
    """Everything about one capability that must hold before it is advertised.

    Reserved-key rules included: an unprefixed key that is not in the standard
    registry is the one failure that costs a *dispatcher* rather than this
    platform, because it takes a name the registry may later define.
    """
    key = declared.get("key", "")
    if key not in STANDARD_KEYS and not key.startswith(VENDOR_PREFIX):
        raise SpecError(
            f"capability key {key!r} is neither a standard key "
            f"({', '.join(STANDARD_KEYS)}) nor {VENDOR_PREFIX}-prefixed"
        )
    execution = declared["execution"]
    if key in CONTROL_KEYS and execution["lane"] != "control":
        raise SpecError(
            f"{key!r} belongs on the control lane, not {execution['lane']!r}"
        )
    safety = declared["safety"]
    if safety["restricted"] and declared["auth"]["audit"] != "always":
        raise SpecError(f"{key!r} is restricted, so auth.audit must be 'always'")
    for item in declared["preconditions"]:
        kind = item.get("type")
        if kind not in PRECONDITION_TYPES:
            raise SpecError(f"{key!r}: unknown precondition type {kind!r}")
        pointer = item.get("input_pointer")
        if pointer is not None:
            _check_pointer(key, declared["input_schema"], pointer)
    return declared


def _check_pointer(key: str, input_schema: dict, pointer: str) -> None:
    """A ``zone_known`` precondition that points at nothing can never hold.

    It is checked here rather than at evaluation time because the failure is
    silent at evaluation: a pointer to a property that does not exist reads as
    "the zone is fine" on every mission.
    """
    if not pointer.startswith("/"):
        raise SpecError(f"{key!r}: input_pointer {pointer!r} must start with '/'")
    node = input_schema
    for token in pointer[1:].split("/"):
        node = ((node or {}).get("properties") or {}).get(token)
        if node is None:
            raise SpecError(
                f"{key!r}: input_pointer {pointer!r} names no property of input_schema"
            )


def capability_set(
    platform_id: str,
    capabilities,
    *,
    platform_type: str = "",
    revision: int = 0,
    generated_at: str | None = None,
) -> dict:
    """Everything one platform can be asked to do, as one document."""
    capabilities = list(capabilities)
    keys = [item["key"] for item in capabilities]
    duplicates = sorted({key for key in keys if keys.count(key) > 1})
    if duplicates:
        raise SpecError(f"duplicate capability keys: {', '.join(duplicates)}")
    if revision < 0:
        raise SpecError("capability set revision must not be negative")
    from mote_bringup.spec import mission  # local: only for the timestamp format

    return {
        "schema": SCHEMA,
        "platform_id": platform_id,
        "platform_type": platform_type,
        "generated_at": generated_at or mission.now(),
        "revision": int(revision),
        "capabilities": capabilities,
    }


def find(document: dict, key: str) -> dict | None:
    """The capability ``key`` in a capability set, or None."""
    for item in document.get("capabilities") or ():
        if item.get("key") == key:
            return item
    return None

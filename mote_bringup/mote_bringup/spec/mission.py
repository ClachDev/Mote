"""mission/v0 — one instruction to one platform, and what became of it.

A **mission** is one invocation of one :mod:`capability
<mote_bringup.spec.capability>` on one platform: an id chosen by the
dispatcher, a lifecycle, and exactly one terminal state. This module is the
payload half of that contract — :func:`command`, :func:`cancel`, :func:`status`
and :func:`failure`, plus the state machine and the failure taxonomy they are
checked against. Who may hold the lane, and for how long a terminal status is
remembered, is :mod:`mote_fleet.dispatch`; who executes is
:mod:`mote_tasks.task_server`.

Three of the spec's rules are load-bearing here rather than in a caller, so
this module enforces them and no caller has to remember them:

**``failure`` is present exactly on ``rejected`` and ``failed``.** A terminal
state with no reason is the thing a dispatcher cannot act on, and a
non-terminal state carrying one reads as over when it is not. :func:`status`
refuses both.

**``terminal`` is carried on the wire** even though it is derivable, so a
consumer stops watching without holding a table of final states — and a state
added in a later version does not silently read as non-terminal to an old one.
:func:`status` computes it, so a producer cannot get the two out of step.

**``recoverable`` is set per failure, not looked up from the class.** The spec
is explicit that it is a statement about the world: a ``precondition`` failure
on a battery threshold clears itself on the dock, one on a missing component
does not. So for the three classes whose recoverability genuinely depends on
the instance — ``precondition``, ``unresolved_zone`` and ``timeout`` —
:func:`failure` **requires** the caller to say, and raises rather than guess.
For the rest the answer is a property of the class and defaulting it is not a
guess: the same invalid input fails identically forever, and corridors clear.

What Mote does not emit is as much a part of conformance as what it does.
``running``, ``blocked`` and ``cancelled`` are optional in v0 and Mote's
behaviour tree publishes none of them: it reports that it took a mission and
then that the mission ended. A consumer **must not** require them, which is
precisely what makes this spec implementable on an executor that reports a bare
accept.
"""

import re
import uuid
from datetime import datetime, timezone

from mote_bringup.spec import SpecError

#: Payload shape version, as carried in every payload's ``schema`` field.
SCHEMA = 1

#: The spec's own version, for the topics and routes that name it.
VERSION = "v0"

# -- the lifecycle --------------------------------------------------------

DISPATCHED = "dispatched"  # reached the dispatch layer; NOT an executor ack
ACCEPTED = "accepted"  # the executor owns it and will report a terminal state
RUNNING = "running"  # optional: it has begun acting physically
BLOCKED = "blocked"  # optional: not progressing, still owned
SUCCEEDED = "succeeded"
FAILED = "failed"
REJECTED = "rejected"  # never started; reachable only from `dispatched`
CANCELLED = "cancelled"  # stopped on request; carries no failure

STATES = (
    DISPATCHED,
    ACCEPTED,
    RUNNING,
    BLOCKED,
    SUCCEEDED,
    FAILED,
    REJECTED,
    CANCELLED,
)

TERMINAL_STATES = frozenset({SUCCEEDED, FAILED, REJECTED, CANCELLED})

#: The states a mission can fail *from*, and therefore the legal values of
#: ``failure.at`` — the field that separates "never started, retry freely" from
#: "died halfway, go and look at it".
NON_TERMINAL_STATES = tuple(s for s in STATES if s not in TERMINAL_STATES)

#: States that must carry a ``failure``, and the only ones that may.
FAILED_STATES = frozenset({REJECTED, FAILED})

# -- the failure taxonomy -------------------------------------------------

UNKNOWN_CAPABILITY = "unknown_capability"
INVALID_INPUT = "invalid_input"
PRECONDITION = "precondition"
BUSY = "busy"
UNRESOLVED_ZONE = "unresolved_zone"
UNREACHABLE = "unreachable"
OBSTRUCTED = "obstructed"
TIMEOUT = "timeout"
HARDWARE = "hardware"
SAFETY_STOP = "safety_stop"
INTERNAL = "internal"

FAILURE_CLASSES = (
    UNKNOWN_CAPABILITY,
    INVALID_INPUT,
    PRECONDITION,
    BUSY,
    UNRESOLVED_ZONE,
    UNREACHABLE,
    OBSTRUCTED,
    TIMEOUT,
    HARDWARE,
    SAFETY_STOP,
    INTERNAL,
)

#: Whether re-dispatching the identical mission has a prospect of succeeding —
#: for the classes where that is a property of the class rather than of the
#: instance. The three classes absent from this table are the spec's "depends"
#: rows, and :func:`failure` refuses to invent an answer for them.
RECOVERABLE = {
    UNKNOWN_CAPABILITY: False,  # the platform will not grow the key by itself
    INVALID_INPUT: False,  # the same input fails identically forever
    BUSY: True,  # the lane frees
    UNREACHABLE: False,  # no route exists; a retry re-derives the same map
    OBSTRUCTED: True,  # corridors clear
    HARDWARE: False,
    SAFETY_STOP: False,
    INTERNAL: False,  # a defect in the platform software
}

#: Classes whose recoverability is a fact about this particular failure. A
#: ``battery_above`` precondition clears itself on the dock; a
#: ``component_present`` one does not, and both arrive here as ``precondition``.
INSTANCE_RECOVERABLE = frozenset({PRECONDITION, UNRESOLVED_ZONE, TIMEOUT})

# -- lanes ----------------------------------------------------------------

#: At most one non-terminal mission per (platform, lane).
DEFAULT_LANE = "default"

#: The one further lane v0 defines by convention, carrying ``pause``,
#: ``resume`` and ``estop`` — so a stop is dispatchable while a mission runs.
CONTROL_LANE = "control"

# -- where a status came from ---------------------------------------------

SOURCE_FLEET = "fleet"
SOURCE_LOCAL = "local"

SOURCES = (SOURCE_FLEET, SOURCE_LOCAL)

# -- shapes ---------------------------------------------------------------

#: Opaque to consumers, which **must not** parse it. Bounded and restricted so
#: it survives a topic level, a path segment and a log line unescaped.
ID_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,64}$")

#: The spec's platform id: a DNS label. Mote's own ``robot_id`` is a stricter
#: subset of this (``mote_fleet.protocol.ID_RE``); the looser rule is used when
#: *reading* a payload, because another platform's id is not Mote's to narrow.
PLATFORM_ID_RE = re.compile(r"^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$")

#: A capability key, as ``capability.KEY_RE``, restated so this module does not
#: depend on that one to check a payload it received.
CAPABILITY_RE = re.compile(r"^[a-z][a-z0-9_]*$")

LANE_RE = re.compile(r"^[a-z][a-z0-9_]*$")

MAX_DETAIL = 500

#: Required keys per payload kind — the authority the docs and the conformance
#: test are checked against.
REQUIRED = {
    "command": ("schema", "id", "platform_id", "capability", "input", "issued_at"),
    "cancel": ("schema", "id", "platform_id", "issued_at"),
    "status": (
        "schema",
        "id",
        "platform_id",
        "capability",
        "state",
        "terminal",
        "source",
        "stamp",
    ),
    "failure": ("class", "recoverable", "detail"),
}


def now() -> str:
    """A wire timestamp: RFC 3339, UTC, millisecond precision."""
    stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    return stamp.replace("+00:00", "Z")


def new_id() -> str:
    """A correlation id, for a dispatcher that has no id scheme of its own.

    The *dispatcher* generates it — never the platform. A platform-assigned id
    cannot exist until the platform has seen the command, so a dispatcher that
    timed out waiting for one could only send another and hope.
    """
    return uuid.uuid4().hex[:16]


def _text(where: str, value, limit: int) -> str:
    text = "" if value is None else str(value)
    if len(text) > limit:
        raise SpecError(f"{where} is longer than {limit} characters")
    return text


def _match(pattern: re.Pattern, where: str, value) -> str:
    text = "" if value is None else str(value)
    if not pattern.match(text):
        raise SpecError(f"{where} {text!r} does not match {pattern.pattern}")
    return text


def failure(
    failure_class: str,
    detail: str,
    *,
    recoverable: bool | None = None,
    at: str | None = None,
    retry_after_s: float | None = None,
) -> dict:
    """Why a mission did not succeed, typed so a caller need not read prose.

    ``recoverable`` may be omitted only for a class whose answer is a property
    of the class (see :data:`RECOVERABLE`); for ``precondition``,
    ``unresolved_zone`` and ``timeout`` it is required, because the spec makes
    it a statement about *this* failure and a default would be a guess dressed
    as a contract.
    """
    if failure_class not in FAILURE_CLASSES:
        raise SpecError(f"unknown failure class {failure_class!r}")
    if recoverable is None:
        if failure_class in INSTANCE_RECOVERABLE:
            raise SpecError(
                f"{failure_class!r} failures must say whether they are "
                "recoverable: it depends on this failure, not on the class"
            )
        recoverable = RECOVERABLE[failure_class]
    if at is not None and at not in NON_TERMINAL_STATES:
        raise SpecError(
            f"failure.at {at!r} is not a state a mission can fail from "
            f"(one of {', '.join(NON_TERMINAL_STATES)})"
        )
    if retry_after_s is not None:
        retry_after_s = float(retry_after_s)
        if retry_after_s < 0:
            raise SpecError("retry_after_s must not be negative")
    return {
        "class": failure_class,
        "recoverable": bool(recoverable),
        "detail": _text("failure.detail", detail, MAX_DETAIL),
        "at": at,
        "retry_after_s": retry_after_s,
    }


def command(
    platform_id: str,
    capability: str,
    payload_input: dict | None = None,
    *,
    mission_id: str | None = None,
    lane: str = DEFAULT_LANE,
    issued_by: str = "",
    capability_version: str | None = None,
    parent_id: str | None = None,
    deadline: str | None = None,
) -> dict:
    """One instruction: invoke this capability with this input.

    ``input`` is an object even for a capability that takes nothing — a bare
    scalar could never grow a second parameter without a major bump.
    """
    if payload_input is None:
        payload_input = {}
    if not isinstance(payload_input, dict):
        raise SpecError("mission input must be an object")
    return {
        "schema": SCHEMA,
        "id": _match(ID_RE, "mission id", mission_id or new_id()),
        "platform_id": _match(PLATFORM_ID_RE, "platform_id", platform_id),
        "capability": _match(CAPABILITY_RE, "capability", capability),
        "capability_version": capability_version,
        "input": payload_input,
        "issued_at": now(),
        "issued_by": _text("issued_by", issued_by, 200),
        "lane": _match(LANE_RE, "lane", lane),
        "parent_id": parent_id,
        "deadline": deadline,
    }


def cancel(
    platform_id: str, mission_id: str, *, issued_by: str = "", reason: str = ""
) -> dict:
    """A *request* to stop a mission.

    Not an instruction: the mission is not cancelled until a status says so,
    and a cancel naming an unknown or already-terminal id is a no-op rather
    than an error — the caller's goal is already met, and an error there turns
    every retry into a spurious alarm.
    """
    return {
        "schema": SCHEMA,
        "id": _match(ID_RE, "mission id", mission_id),
        "platform_id": _match(PLATFORM_ID_RE, "platform_id", platform_id),
        "issued_at": now(),
        "issued_by": _text("issued_by", issued_by, 200),
        "reason": _text("reason", reason, 300),
    }


def progress(
    fraction: float | None = None, phase: str = "", eta_s: float | None = None
) -> dict:
    """Advisory progress for a long-running mission.

    Null throughout is a legitimate answer and not a gap: an executor that
    cannot estimate how far along it is should say so rather than invent a
    fraction an operator will read as measured.
    """
    if fraction is not None:
        fraction = float(fraction)
        if not 0.0 <= fraction <= 1.0:
            raise SpecError("progress.fraction must be between 0 and 1")
    if eta_s is not None:
        eta_s = float(eta_s)
        if eta_s < 0:
            raise SpecError("progress.eta_s must not be negative")
    return {"fraction": fraction, "phase": _text("phase", phase, 120), "eta_s": eta_s}


def status(
    platform_id: str,
    mission_id: str | None,
    capability: str,
    state: str,
    *,
    lane: str = DEFAULT_LANE,
    detail: str = "",
    source: str = SOURCE_FLEET,
    failure: dict | None = None,
    progress: dict | None = None,
    result: dict | None = None,
    warnings=(),
) -> dict:
    """One transition of one mission — a snapshot, not a delta.

    The latest status for an id is the whole truth about that mission, so a
    consumer that missed the earlier ones is less informed about how it got
    here and not confused about where it is.

    ``id`` is null for a mission the platform started by itself, which is
    reported with ``source: "local"`` because a fleet should see a platform
    that is busy whoever asked it to be.
    """
    if state not in STATES:
        raise SpecError(f"unknown mission state {state!r}")
    if source not in SOURCES:
        raise SpecError(f"unknown status source {source!r}")
    terminal = state in TERMINAL_STATES
    if state in FAILED_STATES:
        if failure is None:
            raise SpecError(f"a {state!r} status must carry a failure")
    elif failure is not None:
        raise SpecError(f"a {state!r} status must not carry a failure")
    if result is not None and state != SUCCEEDED:
        raise SpecError("only a succeeded status may carry a result")
    return {
        "schema": SCHEMA,
        "id": None if mission_id is None else _match(ID_RE, "mission id", mission_id),
        "platform_id": _match(PLATFORM_ID_RE, "platform_id", platform_id),
        "capability": _match(CAPABILITY_RE, "capability", capability),
        "state": state,
        "terminal": terminal,
        "source": source,
        "stamp": now(),
        "lane": _match(LANE_RE, "lane", lane),
        "detail": _text("detail", detail, MAX_DETAIL),
        "progress": progress,
        "result": result,
        "failure": failure,
        "warnings": [_text("warning", w, 300) for w in warnings],
    }


def check(payload: dict, kind: str) -> dict:
    """Reject a payload the rest of the code would only half-understand.

    Checks the required keys and the payload version, and — for a status — the
    two invariants a consumer relies on and a producer can get wrong silently:
    ``terminal`` agreeing with ``state``, and ``failure`` present exactly where
    it belongs.
    """
    if kind not in REQUIRED:
        raise SpecError(f"unknown payload kind {kind!r}")
    if not isinstance(payload, dict):
        raise SpecError(f"{kind} payload is not a JSON object")
    missing = [key for key in REQUIRED[kind] if key not in payload]
    if missing:
        raise SpecError(f"{kind} payload missing {', '.join(missing)}")
    if kind != "failure" and payload.get("schema") != SCHEMA:
        # Same spec version, a payload shape we do not know: refuse rather than
        # guess. There is no partial-understanding mode.
        raise SpecError(
            f"{kind} payload schema {payload.get('schema')!r}, expected {SCHEMA}"
        )
    if kind == "status":
        state = payload["state"]
        if state not in STATES:
            raise SpecError(f"unknown mission state {state!r}")
        if bool(payload["terminal"]) != (state in TERMINAL_STATES):
            raise SpecError(
                f"status for {state!r} says terminal={payload['terminal']!r}"
            )
        carried = payload.get("failure")
        if (state in FAILED_STATES) != (carried is not None):
            raise SpecError(f"a {state!r} status carries failure={carried!r}")
        if carried is not None:
            check(carried, "failure")
            if carried["class"] not in FAILURE_CLASSES:
                raise SpecError(f"unknown failure class {carried['class']!r}")
    return payload

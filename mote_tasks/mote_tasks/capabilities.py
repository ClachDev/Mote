"""What this robot can be asked to do, as a capability/v0 document.

This replaces the command grammar. ``fetch red_box dropoff`` was a string a
dispatcher had to know the shape of by having read ``task_server``'s docstring,
and the only way to discover the shape was to send a wrong one and read the
refusal. A capability set is the same information declared: the key, the typed
input, what has to be true before it will be taken, and what it does in the
physical world.

Two keys, both from the spec's **standard registry**, so a dispatcher that has
never seen a Mote knows the input shape: ``goto`` and ``fetch``. The registry's
property names are used verbatim — ``fetch`` delivers to ``destination``, not
to the ``drop_zone`` the old grammar's third word was called — because
"accepts the registry's input schema" is the whole value of an unprefixed key.
The one place Mote narrows the registry is ``additionalProperties: false``: an
input the registry admits is still admitted, and a dispatcher's typo becomes a
rejection instead of a silently ignored field.

Everything a *dispatcher* needs is declared and nothing else is. In particular
``execution.cancellable`` is **false** for both, because the task layer has no
cancel: there is a twist_mux pause lock an operator can assert, and that is not
the same thing, and saying ``true`` here would promise a message nothing
handles. ``max_duration_s`` is therefore non-null and ``task_server`` enforces
it — the spec makes an unbounded, uncancellable capability undispatchable, and
it is right to.

``goto`` is ``idempotency: natural`` — driving somewhere twice ends in the same
place. ``fetch`` is ``none``: every run picks something up and puts it down, so
a retry is a second physical effect, and no layer may retry it automatically
whatever a failure's ``recoverable`` says.
"""

from mote_bringup.spec import capability as cap

#: Bumped when anything in :func:`capability_set`'s output changes, so a
#: consumer holding a higher revision can ignore a lower one and an
#: out-of-order redelivery is harmless rather than a downgrade.
REVISION = 1

PLATFORM_TYPE = "amr"

GOTO = "goto"
FETCH = "fetch"

#: The capability an operator's mission names, per key. ``task_server`` maps a
#: key to a tree through this, so adding a capability is adding a row rather
#: than a branch in the dispatcher.
KEYS = (GOTO, FETCH)


def goto(max_speed_mps: float | None = None) -> dict:
    return cap.capability(
        GOTO,
        version="1.0.0",
        display_name="Go to zone",
        summary="Drive to a named zone and stop there.",
        input_schema={
            "type": "object",
            "required": ["target"],
            "additionalProperties": False,
            "properties": {
                "target": dict(cap.zone_ref(), description="The zone to drive to.")
            },
        },
        execution=cap.execution(
            mode="mission",
            cancellable=False,
            idempotency="natural",
            max_duration_s=600.0,
        ),
        preconditions=[
            cap.precondition("localized"),
            cap.precondition("zone_known", input_pointer="/target"),
        ],
        safety=cap.safety(
            motion="drives",
            reversible=True,
            emergency_stop="halts",
            hazards=["unexpected_motion"],
            max_speed_mps=max_speed_mps,
        ),
        auth=cap.auth(required_scopes=["fleet.dispatch"]),
    )


def fetch(max_speed_mps: float | None = None) -> dict:
    return cap.capability(
        FETCH,
        version="1.0.0",
        display_name="Fetch",
        summary="Collect something and deliver it to a named zone.",
        input_schema={
            "type": "object",
            "required": ["target", "destination"],
            "additionalProperties": False,
            "properties": {
                "target": {
                    "type": "string",
                    "maxLength": 120,
                    "description": (
                        "What to collect: a zone name, or an open-vocabulary "
                        "object label for the detector."
                    ),
                },
                "destination": dict(cap.zone_ref(), description="Where to deliver it."),
            },
        },
        execution=cap.execution(
            mode="mission",
            cancellable=False,
            idempotency="none",
            max_duration_s=900.0,
        ),
        preconditions=[
            cap.precondition("localized"),
            cap.precondition("zone_known", input_pointer="/destination"),
            # Machine-opaque, and deliberately non-blocking: the label branch
            # needs the off-board detector, which a robot may legitimately be
            # running without. An unmet non-blocking precondition is reported
            # in the accepted status's warnings, so a fetch that started
            # degraded is visible rather than merely slow to fail.
            cap.precondition(
                "custom",
                blocking=False,
                description=(
                    "an object label (rather than a zone name) as target needs "
                    "the perception stack's detector to be running"
                ),
            ),
        ],
        safety=cap.safety(
            motion="drives_and_manipulates",
            reversible=True,
            emergency_stop="halts",
            hazards=["unexpected_motion", "pinch"],
            max_speed_mps=max_speed_mps,
            max_payload_kg=None,
        ),
        auth=cap.auth(required_scopes=["fleet.dispatch"]),
    )


def capability_set(platform_id: str, *, max_speed_mps: float | None = None) -> dict:
    """This robot's whole offering, ready to advertise.

    ``max_speed_mps`` comes from ``robot.yaml``'s ``max_wheel_speed`` by way of
    the launch file, rather than being written here: it is a measurement of the
    drive, and a second copy of it would be free to disagree with the one the
    controller enforces. ``None`` is the honest answer where nothing passed it,
    and the spec says so — null means unknown, never unlimited.
    """
    return cap.capability_set(
        platform_id,
        [goto(max_speed_mps), fetch(max_speed_mps)],
        platform_type=PLATFORM_TYPE,
        revision=REVISION,
    )

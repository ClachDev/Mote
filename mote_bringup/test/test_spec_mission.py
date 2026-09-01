"""mission/v0's rules, as plain function calls.

Every rule here is one a producer can break silently — a status whose
``terminal`` disagrees with its ``state``, a terminal state with no reason on
it, a ``recoverable`` invented by a lookup where the spec says it is a fact
about the instance. None of them fail loudly in the field: they fail as a
dispatcher that retries forever, or one that stops watching a mission that has
not finished.
"""

import pytest

from mote_bringup.spec import SpecError
from mote_bringup.spec import mission


def test_terminal_is_computed_not_supplied():
    """A producer cannot get ``terminal`` out of step with ``state``, because
    it never writes it. A consumer stops watching on this field alone."""
    for state in mission.STATES:
        kwargs = {}
        if state in mission.FAILED_STATES:
            kwargs["failure"] = mission.failure(mission.INTERNAL, "x")
        payload = mission.status("mote-01", "abc", "goto", state, **kwargs)
        assert payload["terminal"] == (state in mission.TERMINAL_STATES)
        mission.check(payload, "status")


def test_exactly_the_failed_states_carry_a_failure():
    for state in mission.STATES:
        if state in mission.FAILED_STATES:
            with pytest.raises(SpecError, match="must carry a failure"):
                mission.status("mote-01", "abc", "goto", state)
        else:
            with pytest.raises(SpecError, match="must not carry a failure"):
                mission.status(
                    "mote-01",
                    "abc",
                    "goto",
                    state,
                    failure=mission.failure(mission.INTERNAL, "x"),
                )


def test_check_catches_a_hand_built_payload_that_lies():
    payload = mission.status("mote-01", "abc", "goto", mission.ACCEPTED)
    payload["terminal"] = True
    with pytest.raises(SpecError, match="terminal"):
        mission.check(payload, "status")

    payload = mission.status(
        "mote-01",
        "abc",
        "goto",
        mission.FAILED,
        failure=mission.failure("hardware", ""),
    )
    payload["failure"] = None
    with pytest.raises(SpecError, match="failure"):
        mission.check(payload, "status")


def test_recoverable_must_be_stated_for_the_classes_that_depend_on_it():
    """The spec's "depends" rows. A ``battery_above`` precondition clears
    itself on the dock and a ``component_present`` one does not, and both
    arrive as ``precondition`` — so a default would be a guess."""
    for kind in mission.INSTANCE_RECOVERABLE:
        with pytest.raises(SpecError, match="recoverable"):
            mission.failure(kind, "detail")
        assert mission.failure(kind, "detail", recoverable=True)["recoverable"] is True

    assert mission.failure(mission.BUSY, "x")["recoverable"] is True
    assert mission.failure(mission.INVALID_INPUT, "x")["recoverable"] is False


def test_every_failure_class_has_a_recoverability_answer():
    """No class may fall through both tables: a failure with no answer is a
    failure a dispatcher cannot decide about."""
    answered = set(mission.RECOVERABLE) | mission.INSTANCE_RECOVERABLE
    assert answered == set(mission.FAILURE_CLASSES)
    assert not set(mission.RECOVERABLE) & mission.INSTANCE_RECOVERABLE


def test_failure_at_names_a_state_a_mission_can_fail_from():
    assert mission.failure("hardware", "x", at=mission.ACCEPTED)["at"] == "accepted"
    with pytest.raises(SpecError, match="fail from"):
        mission.failure("hardware", "x", at=mission.SUCCEEDED)


def test_a_local_mission_has_a_null_id_and_still_validates():
    payload = mission.status(
        "mote-01", None, "goto", mission.ACCEPTED, source=mission.SOURCE_LOCAL
    )
    assert payload["id"] is None
    mission.check(payload, "status")


def test_command_defaults_to_the_default_lane_and_an_empty_input():
    payload = mission.command("mote-01", "goto")
    assert payload["lane"] == mission.DEFAULT_LANE
    assert payload["input"] == {}
    mission.check(payload, "command")


def test_a_correlation_id_is_bounded_and_unparsed():
    with pytest.raises(SpecError):
        mission.command("mote-01", "goto", mission_id="has spaces")
    with pytest.raises(SpecError):
        mission.command("mote-01", "goto", mission_id="x" * 65)
    assert mission.ID_RE.match(mission.new_id())


def test_a_payload_of_an_unknown_schema_is_refused_not_guessed_at():
    payload = mission.command("mote-01", "goto")
    payload["schema"] = 2
    with pytest.raises(SpecError, match="schema"):
        mission.check(payload, "command")


def test_only_a_succeeded_mission_may_carry_a_result():
    with pytest.raises(SpecError, match="result"):
        mission.status("mote-01", "abc", "goto", mission.ACCEPTED, result={})
    assert mission.status("mote-01", "abc", "goto", mission.SUCCEEDED, result={"a": 1})


def test_progress_of_nothing_is_a_legitimate_answer():
    assert mission.progress()["fraction"] is None
    with pytest.raises(SpecError):
        mission.progress(fraction=1.5)


def test_cancel_names_the_mission_and_nothing_about_the_capability():
    payload = mission.cancel("mote-01", "abc", reason="operator changed their mind")
    assert set(payload) == {
        "schema",
        "id",
        "platform_id",
        "issued_at",
        "issued_by",
        "reason",
    }
    mission.check(payload, "cancel")

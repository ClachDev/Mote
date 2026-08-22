"""What the agent must remember about a mission, and for how long.

Half of what this file used to test does not exist any more: the parser for the
task layer's sentences, and the single-in-flight rule the agent enforced because
it had no correlation id to enforce it with. Both were consequences of an
untyped seam, and mission/v0 removed the seam. The lane now belongs to the
executor (``mote_tasks.task_server``), which is the thing that actually holds
it, and this module keeps the three properties that are genuinely the bridge's:
a redelivery must not re-execute, an outcome must outlive the dispatcher that
asked for it, and a mission nobody answered must not be watched forever.
"""

import pytest

from mote_bringup.spec import mission

from mote_fleet import dispatch


@pytest.fixture
def tracker():
    return dispatch.CommandTracker(platform_id="mote-01", accept_timeout=10.0)


def a_command(mission_id="id-1", capability="goto", **kwargs):
    return mission.command(
        "mote-01", capability, {"target": "kitchen"}, mission_id=mission_id, **kwargs
    )


def a_status(mission_id, state, capability="goto", **kwargs):
    return mission.status("mote-01", mission_id, capability, state, **kwargs)


# ---- the happy path ------------------------------------------------------


def test_a_mission_runs_to_success(tracker):
    action, update = tracker.submit(a_command(), now=0.0)
    assert action == dispatch.FORWARD
    assert update["state"] == mission.DISPATCHED
    assert update["source"] == mission.SOURCE_FLEET

    accepted = tracker.on_status(a_status("id-1", mission.ACCEPTED), now=1.0)
    assert accepted["source"] == mission.SOURCE_FLEET
    assert [entry.id for entry in tracker.in_flight] == ["id-1"]

    tracker.on_status(a_status("id-1", mission.SUCCEEDED), now=2.0)
    assert tracker.in_flight == []


def test_the_status_is_passed_through_not_rebuilt(tracker):
    """Everything but ``source`` is the executor's word. A failure re-derived
    here would be a second opinion about a mission this process is not
    running."""
    tracker.submit(a_command(), now=0.0)
    failure = mission.failure(mission.OBSTRUCTED, "drive_to_zone: Nav2 aborted")
    published = tracker.on_status(
        a_status("id-1", mission.FAILED, failure=failure), now=1.0
    )
    assert published["failure"] == failure
    assert published["terminal"] is True


# ---- deduplication and retention ----------------------------------------


def test_a_redelivered_command_is_not_dispatched_twice(tracker):
    tracker.submit(a_command(), now=0.0)
    tracker.on_status(a_status("id-1", mission.ACCEPTED), now=1.0)

    action, update = tracker.submit(a_command(), now=2.0)
    assert action == dispatch.DUPLICATE
    assert update["state"] == mission.ACCEPTED  # re-states what it already knows


def test_a_redelivery_after_completion_replays_the_outcome(tracker):
    """Within the retention window an id is not fresh. This is the difference
    between "a retry is safe" and "a retry is safe until it succeeds"."""
    tracker.submit(a_command(), now=0.0)
    tracker.on_status(a_status("id-1", mission.SUCCEEDED), now=1.0)

    action, update = tracker.submit(a_command(), now=2.0)
    assert action == dispatch.DUPLICATE
    assert update["state"] == mission.SUCCEEDED


def test_an_outcome_survives_an_hour_and_is_forgotten_after_the_window(tracker):
    tracker.submit(a_command(), now=0.0)
    tracker.on_status(a_status("id-1", mission.SUCCEEDED), now=1.0)

    assert tracker.submit(a_command(), now=3600.0)[0] == dispatch.DUPLICATE
    # Past the window the id is fresh again, which the spec permits explicitly:
    # a dispatcher needing exactly-once across longer gaps keeps its own record.
    assert tracker.submit(a_command(), now=7201.0)[0] == dispatch.FORWARD


def test_a_running_mission_is_never_evicted_under_pressure(tracker):
    """Forgetting one still in flight would make its own status look local when
    it lands, and would free an id the executor still holds."""
    tracker.submit(a_command(mission_id="running"), now=0.0)
    tracker.on_status(a_status("running", mission.ACCEPTED), now=0.0)
    for index in range(dispatch.MAX_REMEMBERED + 20):
        key = f"done-{index}"
        tracker.submit(a_command(mission_id=key), now=1.0)
        tracker.on_status(a_status(key, mission.SUCCEEDED), now=1.0)
    assert "running" in tracker.missions
    assert len(tracker.missions) <= dispatch.MAX_REMEMBERED + 1


# ---- attribution ---------------------------------------------------------


def test_a_locally_issued_mission_is_reported_but_not_owned(tracker):
    """A bench script's mission and the fleet's look identical on the robot;
    only this module knows which ids it dispatched."""
    update = tracker.on_status(a_status("bench-7", mission.ACCEPTED), now=0.0)
    assert update["source"] == mission.SOURCE_LOCAL
    assert tracker.in_flight == []


def test_a_status_with_no_id_at_all_is_local(tracker):
    update = tracker.on_status(a_status(None, mission.ACCEPTED), now=0.0)
    assert update["source"] == mission.SOURCE_LOCAL


# ---- the unanswered mission ----------------------------------------------


def test_an_unanswered_mission_fails_with_class_timeout(tracker):
    tracker.submit(a_command(), now=0.0)
    assert tracker.tick(now=5.0) == []

    (update,) = tracker.tick(now=11.0)
    assert update["state"] == mission.FAILED
    assert update["id"] == "id-1"
    assert update["failure"]["class"] == mission.TIMEOUT
    # The task server may simply not be up yet, so the identical mission has
    # every prospect of being taken.
    assert update["failure"]["recoverable"] is True
    assert update["failure"]["at"] == mission.DISPATCHED

    assert tracker.tick(now=20.0) == []  # fires once, not every tick


def test_an_accepted_mission_is_never_timed_out(tracker):
    """Missions take minutes; only the verdict is on this clock. The mission's
    own bound is its capability's max_duration_s, enforced by the executor."""
    tracker.submit(a_command(capability="fetch"), now=0.0)
    tracker.on_status(a_status("id-1", mission.ACCEPTED, capability="fetch"), now=1.0)
    assert tracker.tick(now=10_000.0) == []
    assert [entry.id for entry in tracker.in_flight] == ["id-1"]


# ---- what health reports -------------------------------------------------


def test_the_health_summary_names_the_mission_in_flight(tracker):
    assert tracker.summary() is None
    tracker.submit(a_command(), now=0.0)
    tracker.on_status(a_status("id-1", mission.ACCEPTED), now=1.0)
    assert tracker.summary() == {
        "id": "id-1",
        "capability": "goto",
        "state": mission.ACCEPTED,
        "lane": mission.DEFAULT_LANE,
    }
    tracker.on_status(a_status("id-1", mission.SUCCEEDED), now=2.0)
    assert tracker.summary() is None

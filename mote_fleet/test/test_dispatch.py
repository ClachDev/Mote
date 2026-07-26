"""The single-in-flight rule, including the cases that make it necessary.

The interesting tests here are not the happy path — they are the ones where two
things want the robot at once, or where a status line could plausibly belong to
either of two commands. That ambiguity is inherent to bridging a correlation id
onto a bare-String ROS topic, and these are the cases that pin down how it is
resolved.
"""

import pytest

from mote_fleet import dispatch, protocol


@pytest.fixture
def tracker():
    return dispatch.CommandTracker(accept_timeout=10.0)


# ---- parsing the task server's own status strings -----------------------


def test_parse_accepted():
    parsed = dispatch.parse_status("accepted: goto kitchen")
    assert (parsed.kind, parsed.subject) == (protocol.ACCEPTED, "goto kitchen")


def test_parse_rejected_names_the_command_it_refused():
    parsed = dispatch.parse_status("rejected: 'goto nowhere' (unknown zone 'nowhere')")
    assert parsed.kind == protocol.REJECTED
    assert parsed.subject == "goto nowhere"
    assert parsed.detail == "unknown zone 'nowhere'"
    assert not parsed.names_running


def test_parse_rejected_busy_names_the_running_task_instead():
    parsed = dispatch.parse_status("rejected: busy with 'fetch red_box dropoff'")
    assert parsed.subject == "fetch red_box dropoff"
    assert parsed.names_running


def test_parse_failed_strips_the_tree_tip():
    parsed = dispatch.parse_status("failed: goto kitchen (at DriveTo)")
    assert (parsed.kind, parsed.subject, parsed.detail) == (
        protocol.FAILED,
        "goto kitchen",
        "at DriveTo",
    )


def test_parse_ignores_anything_else():
    assert dispatch.parse_status("tree: WaitForTask [RUNNING]") is None
    assert dispatch.parse_status("Zones ['home'] from /tmp/z.yaml") is None


# ---- the happy path ------------------------------------------------------


def test_a_command_runs_to_success(tracker):
    action, update = tracker.submit("id-1", "goto kitchen", now=0.0)
    assert action == dispatch.FORWARD
    assert update.state == protocol.DISPATCHED

    accepted = tracker.on_ros_status("accepted: goto kitchen")
    assert (accepted.state, accepted.command_id) == (protocol.ACCEPTED, "id-1")
    assert tracker.in_flight

    done = tracker.on_ros_status("succeeded: goto kitchen")
    assert (done.state, done.command_id) == (protocol.SUCCEEDED, "id-1")
    assert not tracker.in_flight


def test_the_slot_is_free_again_after_a_terminal_state(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    tracker.on_ros_status("accepted: goto kitchen")
    tracker.on_ros_status("failed: goto kitchen (at DriveTo)")
    action, _ = tracker.submit("id-2", "goto lab", now=1.0)
    assert action == dispatch.FORWARD


# ---- the rules that make retries safe ------------------------------------


def test_a_second_command_is_rejected_without_reaching_ros(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    tracker.on_ros_status("accepted: goto kitchen")

    action, update = tracker.submit("id-2", "goto lab", now=1.0)
    assert action == dispatch.BUSY
    assert update.state == protocol.REJECTED
    assert update.command_id == "id-2"  # the rejection is about the NEW command
    assert "goto kitchen" in update.detail
    # The in-flight command is untouched.
    assert tracker.command_id == "id-1"


def test_a_redelivered_command_is_not_dispatched_twice(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    tracker.on_ros_status("accepted: goto kitchen")

    action, update = tracker.submit("id-1", "goto kitchen", now=1.0)
    assert action == dispatch.DUPLICATE
    assert update.state == protocol.ACCEPTED  # re-states what it already knows


def test_a_redelivery_after_completion_replays_the_outcome(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    tracker.on_ros_status("accepted: goto kitchen")
    tracker.on_ros_status("succeeded: goto kitchen")

    action, update = tracker.submit("id-1", "goto kitchen", now=2.0)
    assert action == dispatch.DUPLICATE
    assert update.state == protocol.SUCCEEDED


# ---- attribution ---------------------------------------------------------


def test_our_command_rejected_as_busy_is_attributed_to_us(tracker):
    """A local task was already running when ours arrived."""
    action, _ = tracker.submit("id-1", "goto kitchen", now=0.0)
    assert action == dispatch.FORWARD
    update = tracker.on_ros_status("rejected: busy with 'fetch red_box dropoff'")
    assert (update.state, update.command_id) == (protocol.REJECTED, "id-1")
    assert update.source == protocol.SOURCE_FLEET
    assert not tracker.in_flight


def test_someone_elses_command_rejected_because_ours_runs_is_reported_as_local(tracker):
    """The mirror image: our mission is running and a local command bounced."""
    tracker.submit("id-1", "goto kitchen", now=0.0)
    tracker.on_ros_status("accepted: goto kitchen")

    update = tracker.on_ros_status("rejected: busy with 'goto kitchen'")
    assert update.source == protocol.SOURCE_LOCAL
    assert update.command_id is None
    # Ours is still running — a local rejection must not free the slot.
    assert tracker.in_flight and tracker.state == protocol.ACCEPTED


def test_a_locally_issued_task_is_reported_but_not_owned(tracker):
    update = tracker.on_ros_status("accepted: goto lab")
    assert update.source == protocol.SOURCE_LOCAL
    assert update.command_id is None
    assert not tracker.in_flight


def test_a_status_for_a_different_command_is_not_taken_as_ours(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    update = tracker.on_ros_status("accepted: fetch red_box dropoff")
    assert update.source == protocol.SOURCE_LOCAL
    # Ours is still waiting for its own verdict.
    assert tracker.state == protocol.DISPATCHED


def test_a_terminal_status_before_acceptance_is_not_ours(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    update = tracker.on_ros_status("succeeded: goto kitchen")
    assert update.source == protocol.SOURCE_LOCAL
    assert tracker.state == protocol.DISPATCHED


# ---- the timeout ---------------------------------------------------------


def test_an_unanswered_command_fails_and_frees_the_slot(tracker):
    tracker.submit("id-1", "goto kitchen", now=0.0)
    assert tracker.tick(now=5.0) is None

    update = tracker.tick(now=11.0)
    assert (update.state, update.command_id) == (protocol.FAILED, "id-1")
    assert "10s" in update.detail
    assert not tracker.in_flight

    assert tracker.tick(now=20.0) is None  # fires once, not every tick


def test_an_accepted_mission_is_never_timed_out(tracker):
    """Missions take minutes; only the verdict is on a clock."""
    tracker.submit("id-1", "fetch red_box dropoff", now=0.0)
    tracker.on_ros_status("accepted: fetch red_box dropoff")
    assert tracker.tick(now=10_000.0) is None
    assert tracker.in_flight

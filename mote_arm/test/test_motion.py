"""Streaming supervision: the rule that stops a move the arm is not completing."""

import pytest

from mote_arm.motion import LagSupervisor, advance, lag_of


def test_lag_is_the_worst_joint():
    setpoint = {"a": 1.0, "b": 2.0}
    assert lag_of(setpoint, {"a": 0.9, "b": 1.5}) == pytest.approx(0.5)


def test_unread_joints_do_not_count_as_lag():
    # A joint missing from /joint_states is a reporting gap, not a stall; the
    # opposite reading would stop every move the instant a read was dropped.
    assert lag_of({"a": 1.0, "b": 2.0}, {"a": 1.0}) == pytest.approx(0.0)
    assert lag_of({"a": 1.0}, {}) == pytest.approx(0.0)


def test_brief_lag_does_not_stop_the_move():
    supervisor = LagSupervisor(max_lag=0.1, stall_time=1.0)
    for _ in range(3):
        assert supervisor.update(0.5, 0.25) is True
    # Catching up resets the clock: the arm is keeping up again, so the next
    # three lagging ticks are as harmless as the first three.
    assert supervisor.update(0.0, 0.25) is True
    for _ in range(3):
        assert supervisor.update(0.5, 0.25) is True


def test_sustained_lag_stops_the_move():
    supervisor = LagSupervisor(max_lag=0.1, stall_time=1.0)
    for _ in range(3):
        assert supervisor.update(0.5, 0.25) is True
    assert supervisor.update(0.5, 0.25) is False


def test_supervisor_rejects_nonsense_thresholds():
    with pytest.raises(ValueError):
        LagSupervisor(max_lag=0.0)
    with pytest.raises(ValueError):
        LagSupervisor(stall_time=-1.0)


def test_advance_never_overshoots():
    assert advance(0.0, 1.0, 0.25) == pytest.approx(0.25)
    assert advance(0.0, 1.0, 5.0) == pytest.approx(1.0)
    assert advance(1.0, 0.0, 0.25) == pytest.approx(0.75)
    assert advance(1.0, 1.0, 0.25) == pytest.approx(1.0)


def test_advance_of_zero_step_holds():
    assert advance(0.5, 1.0, 0.0) == pytest.approx(0.5)

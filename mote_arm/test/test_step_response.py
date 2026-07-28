"""Step-response scoring against synthetic traces.

These are the numbers a gain decision is made on, so each one is checked
against a trace whose answer is known by construction: a joint that stops
short, one that overshoots and rings, and one that hunts around its goal.
"""

import pytest
from mote_arm.step_response import (
    RAD_PER_COUNT,
    Sample,
    StepMetrics,
    droop_verdict,
    settle_band,
    summarise,
)


def trace(positions, load=200, rate=50.0):
    """A trace holding each position in turn, one sample per 1/rate seconds."""
    return [Sample(t=i / rate, rad=rad, load=load) for i, rad in enumerate(positions)]


def held(value, seconds=1.0, rate=50.0):
    return [value] * int(seconds * rate)


def test_stops_short_reports_the_shortfall():
    # Commanded -0.200, settles at -0.129: the kp=16 measurement.
    samples = trace([0.0] * 5 + held(-0.129))
    m = summarise(samples, start_rad=0.0, goal_rad=-0.2)

    assert m.abs_error == pytest.approx(0.071, abs=1e-6)
    assert m.steady_error == pytest.approx(-0.071, abs=1e-6)
    assert m.travel_fraction == pytest.approx(0.645, abs=1e-3)
    assert m.overshoot == 0.0


def test_overshoot_measured_against_commanded_travel():
    samples = trace([0.0, -0.15, -0.25, -0.21] + held(-0.2))
    m = summarise(samples, start_rad=0.0, goal_rad=-0.2)

    assert m.overshoot == pytest.approx(0.25, abs=1e-6)  # 0.05 past a 0.2 step
    assert m.abs_error == pytest.approx(0.0, abs=1e-9)


def test_settling_time_is_when_it_last_enters_the_band():
    # Drifts in slowly, inside the band from t=0.1 s onward.
    samples = trace([0.0, -0.1, -0.199, -0.2, -0.2] + held(-0.2))
    m = summarise(samples, start_rad=0.0, goal_rad=-0.2)

    assert m.settling_time == pytest.approx(2 / 50.0)


def test_never_settling_reports_none():
    # Still travelling when the hold ends.
    samples = trace([0.0, -0.05, -0.10, -0.15, -0.20])
    m = summarise(samples, start_rad=0.0, goal_rad=-0.2, hold_window=0.02)

    assert m.settling_time is None


def test_hunting_counts_reversals_and_ripple():
    # Oscillates +-3 counts about the goal: a buzzing servo.
    wobble = [-0.2 + 3 * RAD_PER_COUNT * (1 if i % 2 else -1) for i in range(50)]
    m = summarise(trace(wobble), start_rad=0.0, goal_rad=-0.2)

    # The hold window is the trailing 0.5 s — 25 of the 50 samples.
    assert m.reversals >= 20
    assert m.ripple_counts == pytest.approx(6.0, abs=0.01)


def test_quantisation_alone_is_not_hunting():
    # A still joint dithering by one count must not read as oscillation.
    dither = [-0.2 + (RAD_PER_COUNT if i % 2 else 0.0) for i in range(50)]
    m = summarise(trace(dither), start_rad=0.0, goal_rad=-0.2)

    assert m.reversals == 0


def test_load_is_read_from_the_hold_not_the_move():
    samples = [Sample(t=0.0, rad=0.0, load=900)] + [
        Sample(t=0.02 * (i + 1), rad=-0.2, load=180) for i in range(50)
    ]
    m = summarise(samples, start_rad=0.0, goal_rad=-0.2)

    assert m.hold_load == pytest.approx(180)
    assert m.peak_load == 900  # the move's peak is still reported


def test_settle_band_never_tighter_than_two_counts():
    assert settle_band(0.001) == pytest.approx(2 * RAD_PER_COUNT)
    assert settle_band(1.0) == pytest.approx(0.02)


def test_empty_or_zero_step_is_rejected():
    with pytest.raises(ValueError):
        summarise([], start_rad=0.0, goal_rad=-0.2)
    with pytest.raises(ValueError):
        summarise(trace(held(0.0)), start_rad=0.0, goal_rad=0.0)


def metrics(error, load):
    return StepMetrics(
        start_rad=0.0,
        goal_rad=-0.2,
        final_rad=-0.2 + error,
        steady_error=error,
        abs_error=abs(error),
        travel_fraction=1 - abs(error) / 0.2,
        overshoot=0.0,
        settling_time=0.5,
        ripple=0.0,
        ripple_counts=0.0,
        reversals=0,
        hold_load=load,
        peak_load=int(load),
        samples=150,
    )


def test_verdict_names_droop_when_kp_times_error_holds():
    # The measured numbers: kp*error 1.14 vs 1.06, load flat near 190.
    verdict = droop_verdict([(16, metrics(0.071, 196)), (32, metrics(0.033, 176))])
    assert "proportional droop" in verdict


def test_verdict_names_saturation_when_load_is_pinned():
    verdict = droop_verdict([(16, metrics(0.071, 980)), (32, metrics(0.068, 990))])
    assert "saturation" in verdict


def test_verdict_flags_an_error_that_ignores_gain():
    verdict = droop_verdict([(16, metrics(0.070, 200)), (128, metrics(0.068, 205))])
    assert "barely moved" in verdict


def test_verdict_needs_two_gains():
    assert "not enough" in droop_verdict([(32, metrics(0.033, 176))])


def test_verdict_survives_a_joint_that_arrived_exactly():
    # A zero error is a division by zero in the error ratio, not a crash.
    verdict = droop_verdict([(16, metrics(0.071, 200)), (128, metrics(0.0, 200))])
    assert verdict

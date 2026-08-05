"""Supervision shared by everything that streams setpoints to the arm.

Streaming a trajectory means the caller, not the servo, is responsible for
noticing that the arm has stopped keeping up. The measure is *lag*: how far the
arm trails the setpoint it was last given. Sustained lag means it is driving
against something it is not overcoming, and the move should stop where it is
rather than hold against the load.

ROS-free, so the rule can be unit-tested without hardware. Used by
``arm_pose go`` and by episode replay.
"""

from __future__ import annotations

from collections.abc import Mapping


def lag_of(setpoint: Mapping[str, float], measured: Mapping[str, float]) -> float:
    """Largest per-joint distance between a setpoint and where the arm is.

    Joints absent from ``measured`` contribute no lag: an unread joint is a
    reporting gap, not evidence of a stall.
    """
    return max(
        (
            abs(measured[name] - value)
            for name, value in setpoint.items()
            if name in measured
        ),
        default=0.0,
    )


class LagSupervisor:
    """Stops a streamed move once the arm has trailed its setpoint for too long.

    A single late sample is normal — the arm is always a little behind a moving
    setpoint. Only lag that *stays* above ``max_lag`` for ``stall_time`` counts.
    """

    def __init__(self, max_lag: float = 0.15, stall_time: float = 1.5):
        if max_lag <= 0:
            raise ValueError("max_lag must be positive")
        if stall_time <= 0:
            raise ValueError("stall_time must be positive")
        self.max_lag = max_lag
        self.stall_time = stall_time
        self.lagging_for = 0.0

    def update(self, lag: float, dt: float) -> bool:
        """Feed one observation; returns False once the move should stop."""
        if lag > self.max_lag:
            self.lagging_for += dt
        else:
            self.lagging_for = 0.0
        return self.lagging_for < self.stall_time


def advance(current: float, target: float, max_step: float) -> float:
    """Move ``current`` towards ``target`` by at most ``max_step``."""
    if max_step < 0:
        raise ValueError("max_step must not be negative")
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)

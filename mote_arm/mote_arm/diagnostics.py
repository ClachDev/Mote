"""Where the arm's motion is being lost, measured rather than reasoned about.

Split out of the mirror because it answers a question the mirror cannot: when
teleop stutters or falls short of its range, the candidate causes are
indistinguishable from outside the process, and each one is a number.
"""

from __future__ import annotations

import time


class Diagnostics:
    """Where the motion is being lost, measured rather than reasoned about.

    Teleop that stutters or falls short of its range has three candidate causes
    and they are indistinguishable from the outside: the mirror not ticking at
    the rate it claims, leader poses arriving in gaps, or the arm not achieving
    the velocity it is being asked for. Each is a number, so each is printed:

      tick   how fast this loop really runs, and its worst period
      leader rate the leader pose advances, and the worst gap between two
      cmd    rate the commanded pose advances -- what the mirror is asking for
      arm    rate the measured pose advances -- what the arm actually did
      lag    how far the arm trails the command right now

    cmd at the rate limit with arm well below it, and a lag that grows, is the
    arm failing to follow. Both low is the mirror loop. A leader gap over the
    deadman is the keyboard loop, which runs at its own rate. Reported for the
    joint that moved most over the window, since that is the one being driven.
    """

    def __init__(self, node, period: float = 0.5):
        self._node = node
        self._period = period
        self._reset(time.monotonic())
        self.leader_stamps: list[float] = []

    def _reset(self, now: float) -> None:
        self._start = now
        self._ticks = 0
        self._dt_max = 0.0
        self._last_tick = now
        self._commanded0 = self._node.mirror.commanded
        self._measured0 = self._node.mirror.measured
        self.leader_stamps = []

    def on_command(self, now: float) -> None:
        self.leader_stamps.append(now)

    def tick(self, now: float) -> None:
        self._ticks += 1
        self._dt_max = max(self._dt_max, now - self._last_tick)
        self._last_tick = now
        elapsed = now - self._start
        if elapsed < self._period:
            return

        commanded = self._node.mirror.commanded
        measured = self._node.mirror.measured
        moved = {n: abs(v - self._commanded0.get(n, v)) for n, v in commanded.items()}
        joint = max(moved, key=moved.get, default=None)

        gaps = [b - a for a, b in zip(self.leader_stamps, self.leader_stamps[1:])]
        parts = [
            f"tick {self._ticks / elapsed:4.1f}Hz worst {self._dt_max * 1e3:5.1f}ms",
            f"leader {len(self.leader_stamps) / elapsed:4.1f}Hz "
            f"worst gap {max(gaps, default=0.0) * 1e3:5.1f}ms",
        ]
        if joint is not None:
            cmd_rate = moved[joint] / elapsed
            arm_rate = (
                abs(measured.get(joint, 0.0) - self._measured0.get(joint, 0.0))
                / elapsed
            )
            lag = abs(commanded[joint] - measured.get(joint, commanded[joint]))
            parts.append(
                f"{joint} cmd {cmd_rate:.3f} arm {arm_rate:.3f} rad/s  lag {lag:+.3f} rad"
            )
        state = self._node.mirror.state
        if self._node.mirror.stalled:
            state += " STALLED:" + ",".join(self._node.mirror.stalled)
        parts.append(state)
        self._node.get_logger().info("diag  " + "  |  ".join(parts))
        self._reset(now)

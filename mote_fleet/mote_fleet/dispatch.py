"""What the agent must remember about a mission, and for how long.

This module used to hold the single-in-flight rule *and* a parser for the
sentences the task layer published, because ``task/command`` was a bare string
with no correlation id: the agent could not tell which refusal belonged to
which command, so it kept the robot from ever seeing two, and worked out
attribution from the command text every status line echoed. Both halves are
gone. A mission/v0 command carries an id, a status carries it back, and the
lane belongs to the executor that actually holds it
(:mod:`mote_tasks.task_server`).

What is left is genuinely the agent's, and none of it is the executor's:

**Deduplication.** MQTT QoS 1 is at-least-once, so the broker may redeliver a
command; an operator may click twice; a dispatcher that timed out may resend
deliberately, which the spec makes safe on purpose. A redelivered id must
republish the status that mission already has and **must not** re-execute it.

**Retention.** A dispatcher that restarts must be able to learn the outcome of
what it sent, so a terminal status is remembered — the spec says at least an
hour, and at least as long as the longest ``max_duration_s`` offered. Within
that window an id is not fresh: a redelivery of a *finished* mission's command
returns the outcome rather than starting a second one, which is the difference
between "retry is safe" and "retry is safe until it succeeds".

**The unanswered command.** ``dispatched`` is not an acknowledgement — it means
the payload reached the dispatch layer, and a robot whose task server is not
running looks exactly like a robot that has not answered yet. So a mission with
no verdict inside ``accept_timeout`` is failed with class ``timeout``, which
frees the dispatcher rather than leaving it watching a mission nobody owns.

**Attribution.** ``source`` is the fleet's view — did this mission come from a
dispatched command — and only this module can answer it: on the robot, a
command the agent forwarded and one a bench script published are the same
message on the same topic. A status whose id is not one we dispatched is
reported as ``local``, so an operator watching the fleet sees a robot that is
busy whoever asked it to be.

**A cancel ends work; it does not start any.** It never passes through
:meth:`CommandTracker.submit`: it creates no mission, takes no lane and is not
deduplicated, since sending the same cancel twice asks for the same thing
twice and the executor ignores the second. :meth:`CommandTracker.cancel` only
decides whether it is worth forwarding. It is not worth forwarding for a
mission already known to be over, whose outcome is retained. Otherwise it is
forwarded, including for an id this agent never dispatched: a mission somebody
started at the robot's own keyboard is still one an operator watching the
fleet may need to stop. One ordering case is the tracker's to close. A cancel
for a mission still ``dispatched`` can overtake the command it names on the
way to the executor, which then finds no such mission and ignores it. So the
tracker keeps that cancel and hands it back, once, when ``accepted`` arrives
(:meth:`CommandTracker.cancels_due`).

It is deliberately free of ROS and MQTT, so every awkward case is a plain
function call in ``test_dispatch.py``.
"""

from dataclasses import dataclass, field

from mote_bringup.spec import mission

# What the agent should do with an inbound command.
FORWARD = "forward"  # a mission we have not seen: publish it to ROS
DUPLICATE = "duplicate"  # this id again: re-publish the status it already has
# ... and with an inbound cancel.
FINISHED = "finished"  # the mission is already over: nothing to stop

#: How long a terminal status is remembered. The spec's floor is one hour and
#: "at least the longest max_duration_s you offer"; Mote's longest is fetch's
#: 900 s, so the hour governs.
RETENTION_S = 3600.0

#: A ceiling on remembered missions, so a dispatcher looping on a fresh id
#: every second cannot grow this without bound on a robot with 4 GB of RAM.
#: Reached only by a caller that is already misbehaving, and the eviction is
#: oldest-first, so the ids most likely to be redelivered are the last to go.
MAX_REMEMBERED = 512


@dataclass
class Mission:
    """One mission this agent forwarded, and where it got to."""

    id: str
    capability: str
    lane: str
    state: str
    sent_at: float
    status: dict = field(default_factory=dict)
    finished_at: float | None = None
    #: A cancel that arrived before the executor had taken the mission, held
    #: to be sent again once it has.
    early_cancel: dict | None = None

    @property
    def terminal(self) -> bool:
        return self.state in mission.TERMINAL_STATES


class CommandTracker:
    """The agent's memory of the missions it has dispatched."""

    def __init__(
        self,
        platform_id: str = "",
        accept_timeout: float = 20.0,
        retention_s: float = RETENTION_S,
    ):
        self.platform_id = platform_id
        #: How long a forwarded command may go unanswered before the agent
        #: calls it failed. Only covers the *verdict* — an accepted mission may
        #: then run for as long as its capability allows.
        self.accept_timeout = accept_timeout
        self.retention_s = retention_s
        self.missions: dict[str, Mission] = {}

    # -- inbound ----------------------------------------------------------

    def submit(self, command: dict, now: float) -> tuple[str, dict]:
        """Decide what to do with an inbound command.

        Returns ``(action, status)``. The status is published either way — a
        command that produces no status is a command an operator watches
        disappear — and for a duplicate it is the one that mission already has,
        republished rather than recomputed.
        """
        # Expire first, then look up: an id whose window has passed must read
        # as fresh, and checking before evicting would keep answering with a
        # status the retention rule says has been forgotten.
        self._evict(now)
        known = self.missions.get(command["id"])
        if known is not None:
            return DUPLICATE, known.status
        entry = Mission(
            id=command["id"],
            capability=command["capability"],
            lane=command.get("lane") or mission.DEFAULT_LANE,
            state=mission.DISPATCHED,
            sent_at=now,
        )
        entry.status = self._status(entry, mission.DISPATCHED)
        self.missions[entry.id] = entry
        return FORWARD, entry.status

    def on_status(self, payload: dict, now: float) -> dict:
        """Record a status the executor published, and attribute it.

        The payload is passed through rather than rebuilt: the executor is the
        author of everything in it except ``source``, and re-deriving a state
        or a failure here would be a second opinion about a mission this
        process is not running.
        """
        entry = self.missions.get(payload.get("id"))
        if entry is None:
            payload["source"] = mission.SOURCE_LOCAL
            return payload
        payload["source"] = mission.SOURCE_FLEET
        entry.state = payload["state"]
        entry.status = payload
        if entry.terminal and entry.finished_at is None:
            entry.finished_at = now
            entry.early_cancel = None
        return payload

    def cancel(self, payload: dict, now: float) -> str:
        """Decide what to do with an inbound cancel: forward it, or not.

        :data:`FINISHED` for a mission this agent knows is over, since its
        outcome is already retained and there is nothing left to stop.
        :data:`FORWARD` otherwise, even for an id this agent did not dispatch.
        The executor is the authority on what is in flight, and it treats a
        cancel naming anything else as the no-op the spec says it is.
        """
        self._evict(now)
        entry = self.missions.get(payload["id"])
        if entry is None:
            return FORWARD
        if entry.terminal:
            return FINISHED
        if entry.state == mission.DISPATCHED:
            entry.early_cancel = payload
        return FORWARD

    def cancels_due(self) -> list[dict]:
        """Cancels to send again, because their mission has now been taken.

        Each is handed back once. A mission that ends without ever being
        accepted takes its held cancel with it.
        """
        due = []
        for entry in self.missions.values():
            if entry.early_cancel is None or entry.state == mission.DISPATCHED:
                continue
            if not entry.terminal:
                due.append(entry.early_cancel)
            entry.early_cancel = None
        return due

    def tick(self, now: float) -> list[dict]:
        """Fail every mission the executor never answered, and forget old ones."""
        overdue = []
        for entry in self.missions.values():
            if entry.state != mission.DISPATCHED:
                continue
            if now - entry.sent_at < self.accept_timeout:
                continue
            failure = mission.failure(
                mission.TIMEOUT,
                f"no verdict from the task server within {self.accept_timeout:.0f}s",
                # The task server may simply not be running yet; when it is,
                # the identical mission has every prospect of being taken.
                recoverable=True,
                at=mission.DISPATCHED,
            )
            entry.state = mission.FAILED
            entry.finished_at = now
            entry.status = self._status(entry, mission.FAILED, failure=failure)
            overdue.append(entry.status)
        self._evict(now)
        return overdue

    # -- what health reports ----------------------------------------------

    @property
    def in_flight(self) -> list[Mission]:
        return [entry for entry in self.missions.values() if not entry.terminal]

    def summary(self) -> dict | None:
        """The in-flight mission for the health payload, or None.

        One lane, so at most one; the first is the answer rather than a list,
        because a dashboard row showing "which mission is this robot on" has
        room for one and a list would be a lie about the shape of the fleet.
        """
        for entry in self.in_flight:
            return {
                "id": entry.id,
                "capability": entry.capability,
                "state": entry.state,
                "lane": entry.lane,
            }
        return None

    # -- internals --------------------------------------------------------

    def _status(self, entry: Mission, state: str, **kwargs) -> dict:
        return mission.status(
            self.platform_id,
            entry.id,
            entry.capability,
            state,
            lane=entry.lane,
            source=mission.SOURCE_FLEET,
            **kwargs,
        )

    def _evict(self, now: float):
        expired = [
            key
            for key, entry in self.missions.items()
            if entry.finished_at is not None
            and now - entry.finished_at > self.retention_s
        ]
        for key in expired:
            del self.missions[key]
        # Only terminal missions are evictable under pressure: forgetting one
        # that is still running would make its own status look local when it
        # lands, and would free an id the executor still holds.
        while len(self.missions) > MAX_REMEMBERED:
            oldest = min(
                (e for e in self.missions.values() if e.terminal),
                key=lambda e: e.finished_at or 0.0,
                default=None,
            )
            if oldest is None:
                return
            del self.missions[oldest.id]

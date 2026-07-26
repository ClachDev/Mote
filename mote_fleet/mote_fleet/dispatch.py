"""The single-in-flight rule: how a correlation id survives the ROS seam.

``task/command`` is a bare ``std_msgs/String`` and ``task/status`` answers with
bare strings too — the task layer has no notion of a request id, and giving it
one would mean changing a working interface for the fleet's convenience. So the
correlation id lives *upstream* of the ROS seam, in the agent, and this module
is the whole of that bookkeeping. It is deliberately free of ROS and MQTT so the
awkward cases can be tested as plain function calls.

The rule (fleet.md Q1) is **one in-flight command per robot**:

* a command arrives, the agent forwards it and remembers its id;
* a second command arriving before the first reaches a terminal state is
  rejected by the agent itself, without touching ROS — the robot never sees two;
* a *redelivery* of the same id (QoS 1 does that, and so does an operator
  clicking twice) re-publishes the current status instead of re-dispatching;
* the first terminal status releases the slot.

Attribution then has two independent handles rather than one. Position alone
("the next verdict must be mine") would misfire the moment anything else
publishes to ``task/command`` — a bench script, an operator's ``ros2 topic
pub`` — so it is backed up by the fact that every status line **echoes the
command text it is about**::

    accepted: goto kitchen
    rejected: 'goto nowhere' (unknown zone 'nowhere')
    rejected: busy with 'fetch red_box dropoff'     <- names the RUNNING task
    succeeded: goto kitchen
    failed: goto kitchen (at DriveTo)

A status whose subject is not our in-flight command is reported as a
locally-issued task (``source: "local"``) rather than silently attributed to
the fleet: an operator watching the fleet should see a robot that is busy,
whoever asked it to be.
"""

import re
from dataclasses import dataclass

from mote_fleet import protocol

# What the agent should do with an inbound command.
FORWARD = "forward"  # new command: publish it to ROS
DUPLICATE = "duplicate"  # same id again: re-publish the status it already has
BUSY = "busy"  # another command is in flight: reject without touching ROS

_REJECT_BUSY_RE = re.compile(r"^busy with '(.*)'$")
_REJECT_CMD_RE = re.compile(r"^'(.*)' \((.*)\)$", re.DOTALL)
_FAILED_AT_RE = re.compile(r"^(.*) \(at (.*)\)$", re.DOTALL)


@dataclass(frozen=True)
class RosStatus:
    """One parsed ``task/status`` line."""

    kind: str  # a protocol task state
    subject: str  # the command text the line is about
    detail: str
    #: True for ``rejected: busy with 'X'``, where the subject is the *running*
    #: task and the rejected command is unnamed.
    names_running: bool = False


@dataclass(frozen=True)
class Update:
    """A status transition the agent should publish."""

    command_id: str | None
    command: str
    state: str
    detail: str
    source: str


def parse_status(text: str) -> RosStatus | None:
    """Parse a ``task/status`` line, or None if it is not one we understand."""
    kind, sep, rest = text.partition(":")
    kind, rest = kind.strip(), rest.strip()
    if not sep or kind not in protocol.TASK_STATES:
        return None
    if kind == protocol.REJECTED:
        busy = _REJECT_BUSY_RE.match(rest)
        if busy:
            return RosStatus(kind, busy.group(1), "busy", names_running=True)
        quoted = _REJECT_CMD_RE.match(rest)
        if quoted:
            return RosStatus(kind, quoted.group(1), quoted.group(2))
        return RosStatus(kind, rest, rest)
    if kind == protocol.FAILED:
        at = _FAILED_AT_RE.match(rest)
        if at:
            return RosStatus(kind, at.group(1), f"at {at.group(2)}")
    return RosStatus(kind, rest, "")


class CommandTracker:
    """Holds at most one fleet command and attributes ROS statuses to it."""

    def __init__(self, accept_timeout: float = 20.0):
        #: How long a forwarded command may go unanswered before the agent
        #: calls it failed and frees the slot. Only covers the *verdict* — an
        #: accepted mission may then run for as long as it likes.
        self.accept_timeout = accept_timeout
        self.command_id: str | None = None
        self.command: str = ""
        self.state: str | None = None
        self.detail: str = ""
        self.sent_at: float = 0.0

    @property
    def in_flight(self) -> bool:
        return self.state is not None and self.state not in protocol.TERMINAL_STATES

    def submit(self, command_id: str, text: str, now: float) -> tuple[str, Update]:
        """Decide what to do with an inbound command.

        Returns ``(action, update)`` where action is FORWARD / DUPLICATE / BUSY
        and update is the status the agent should publish either way — a
        command that produces no status is a command an operator watches
        disappear.
        """
        if command_id == self.command_id and self.state is not None:
            return DUPLICATE, self._update(self.state, self.detail)
        if self.in_flight:
            busy = Update(
                command_id=command_id,
                command=text,
                state=protocol.REJECTED,
                detail=(
                    f"busy with '{self.command}' (id {self.command_id}); "
                    "one command in flight per robot"
                ),
                source=protocol.SOURCE_FLEET,
            )
            return BUSY, busy
        self.command_id, self.command = command_id, text
        self.state, self.detail, self.sent_at = protocol.DISPATCHED, "", now
        return FORWARD, self._update(protocol.DISPATCHED, "")

    def on_ros_status(self, text: str) -> Update | None:
        """Turn a ``task/status`` line into the transition to publish."""
        parsed = parse_status(text)
        if parsed is None:
            return None
        if self._is_ours(parsed):
            self.state, self.detail = parsed.kind, parsed.detail
            return self._update(parsed.kind, parsed.detail)
        # Not ours: a task someone started on the robot itself. Report it with a
        # null correlation id so the fleet sees the robot is occupied.
        return Update(
            command_id=None,
            command=parsed.subject,
            state=parsed.kind,
            detail=parsed.detail,
            source=protocol.SOURCE_LOCAL,
        )

    def tick(self, now: float) -> Update | None:
        """Fail a command the task server never answered, freeing the slot."""
        if self.state != protocol.DISPATCHED:
            return None
        if now - self.sent_at < self.accept_timeout:
            return None
        detail = f"no verdict from the task server within {self.accept_timeout:.0f}s"
        self.state, self.detail = protocol.FAILED, detail
        return self._update(protocol.FAILED, detail)

    def _is_ours(self, parsed: RosStatus) -> bool:
        if self.command_id is None or not self.in_flight:
            return False
        if parsed.kind in (protocol.ACCEPTED, protocol.REJECTED):
            if self.state != protocol.DISPATCHED:
                return False
            if parsed.names_running:
                # "busy with 'X'": ours is the command being rejected, unless X
                # *is* ours — in which case a local command was rejected
                # because our accepted mission is running.
                return parsed.subject != self.command
            return parsed.subject == self.command
        # succeeded/failed name the task that finished.
        return self.state == protocol.ACCEPTED and parsed.subject == self.command

    def _update(self, state: str, detail: str) -> Update:
        return Update(
            command_id=self.command_id,
            command=self.command,
            state=state,
            detail=detail,
            source=protocol.SOURCE_FLEET,
        )

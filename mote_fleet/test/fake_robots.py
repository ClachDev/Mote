"""A fleet that exists only on the wire.

The dashboard consumes the control-plane contract and nothing else: presence,
health, pose and task status arriving over MQTT, and one command going back the
other way. So the cheapest honest thing to point it at is a script that
publishes exactly that contract — no ROS, no Nav2, no hardware — which is what
this is. It is **not** a second robot implementation: every payload is built by
``protocol.py``, the same module the real agent builds them with, so a change to
the wire changes this fixture or fails it.

What it does model, because the UI renders each of them differently:

* **presence, with a Last Will** — the ``offline`` profile connects, publishes
  its retained state, then drops the socket *without* a DISCONNECT, which is the
  only way to see the broker publish the will on the robot's behalf.
* **health with subsystems** — an ``ok`` robot and a ``degraded`` one, so the
  roll-up and the per-subsystem rows both have something to draw.
* **a pose that moves** — retained, republished a few times a second, so the map
  is live rather than a single dot.
* **task status transitions** — ``dispatched`` → ``accepted`` → ``succeeded``
  for a command the robot's grammar knows, ``dispatched`` → ``rejected`` for one
  it does not, and a redelivery re-publishes the last status rather than
  re-running the task. That is the agent's single-in-flight rule (``dispatch.py``
  owns it for real); here it is just enough of it to drive the status log.

Run it against a broker of your own while working on ``server/ui/``::

    python mote_fleet/test/fake_robots.py --host 127.0.0.1 --port 1883

or let ``ui_check.py`` bring up a whole private stack around it
(``pixi run fleet-ui-check``).
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import paho.mqtt.client as mqtt  # noqa: E402

from mote_fleet import protocol  # noqa: E402

#: Command words the robot's task layer knows (``mote_tasks/task_server.py``).
#: Anything else is rejected, which is a case the status log should be able to
#: show without an operator having to invent a broken robot.
VERBS = ("fetch", "goto")

#: Zone names the sim worlds' ``zones.yaml`` files all carry, so a `goto` typed
#: into the dashboard against the shipped basemap succeeds.
ZONES = ("pickup", "dropoff", "home")

PROFILES = ("ok", "degraded", "offline")


def subsystems(state: str) -> list[dict]:
    """A health roll-up shaped like the health monitor's ``/diagnostics_agg``.

    The names are the real robot's; the degraded one is a real degraded case
    (``slip_monitor`` reports slip as DEGRADED, never FAULT).
    """
    rows = [
        protocol.subsystem("drive", protocol.OK, "2 servos, 50 Hz"),
        protocol.subsystem("lidar", protocol.OK, "10.0 Hz"),
        protocol.subsystem("localisation", protocol.OK, "icp residual 0.9 cm"),
        protocol.subsystem("system", protocol.OK, "cpu 31%, 46 C, disk 38%"),
    ]
    if state == protocol.DEGRADED:
        rows[2] = protocol.subsystem(
            "localisation", protocol.DEGRADED, "slip: 0.14 m over 1.0 s"
        )
    return rows


class FakeRobot:
    """One robot, on the wire only.

    Publishing happens from the ticker thread and from paho's network thread
    (a command reply), which is safe: ``paho`` serialises publishes internally,
    and the only shared state is the in-flight command, guarded below.
    """

    def __init__(
        self,
        robot_id: str,
        *,
        host: str,
        port: int,
        profile: str = "ok",
        site: str = "office_world",
        floor: str = "ground",
        centre: tuple[float, float] = (0.0, 0.0),
        radius: float = 0.6,
        period_s: float = 40.0,
        task_seconds: float = 2.0,
        zones: tuple[str, ...] = ZONES,
        revision: str = "",
    ):
        if profile not in PROFILES:
            raise ValueError(f"unknown profile {profile!r} (want {'/'.join(PROFILES)})")
        self.id = robot_id
        self.host = host
        self.port = port
        self.profile = profile
        self.site = site
        self.floor = floor
        self.centre = centre
        self.radius = radius
        self.period_s = period_s
        self.task_seconds = task_seconds
        self.zones = zones
        self.revision = revision
        self.started = time.monotonic()
        self.dropped = False

        self._lock = threading.Lock()
        self._task = None  # the in-flight command, or None
        self._last_status = {}  # command id -> the status last published for it

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-{robot_id}"
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        # The will is what makes "the robot dropped off" instant instead of a
        # timeout — set before connecting, because the broker records it from
        # the CONNECT packet.
        self.client.will_set(
            protocol.topic(robot_id, protocol.PRESENCE),
            protocol.encode(
                protocol.presence(robot_id, False, reason="connection lost")
            ),
            qos=protocol.QOS,
            retain=True,
        )

    # -- lifecycle --------------------------------------------------------

    def start(self):
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        return self

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if getattr(reason_code, "is_failure", False):
            print(f"{self.id}: broker refused the connection: {reason_code}")
            return
        # Subscribing here rather than after connect() is what keeps a
        # reconnect from coming back deaf.
        client.subscribe(protocol.topic(self.id, protocol.COMMAND), qos=protocol.QOS)
        self._publish(
            protocol.PRESENCE, protocol.presence(self.id, True, agent="fake_robots")
        )
        self.tick()

    def drop(self):
        """Die the way a robot on a failing link does: no DISCONNECT packet, so
        the broker publishes the will. Closing the socket after stopping the
        network loop is what makes it a drop rather than a clean goodbye."""
        self.dropped = True
        self.client.loop_stop()
        try:
            self.client.socket().close()
        except (AttributeError, OSError):
            pass

    def close(self):
        """Go offline politely — the agent's own shutdown path."""
        if self.dropped:
            return
        self._publish(
            protocol.PRESENCE, protocol.presence(self.id, False, reason="stopped")
        )
        self.client.loop_stop()
        self.client.disconnect()

    # -- publishing -------------------------------------------------------

    def _publish(self, leaf: str, payload: dict, retain: bool = True):
        self.client.publish(
            protocol.topic(self.id, leaf),
            protocol.encode(payload),
            qos=protocol.QOS,
            retain=retain,
        )

    def health_payload(self) -> dict:
        state = protocol.DEGRADED if self.profile == "degraded" else protocol.OK
        summary = (
            "slip detected while turning"
            if state == protocol.DEGRADED
            else "all subsystems nominal"
        )
        with self._lock:
            task = dict(self._task["summary"]) if self._task else None
        return protocol.health(
            self.id,
            state,
            summary,
            subsystems(state),
            task=task,
            site=self.site,
            floor=self.floor,
            version="fake-robots",
            uptime_s=time.monotonic() - self.started,
            # Which revision this robot is *running*. Left out unless told, so
            # a fixture never claims a revision the registry then reports as
            # out of date — that banner should mean a real robot behind a map.
            map=(
                {"site": self.site, "floor": self.floor, "revision": self.revision}
                if self.revision
                else None
            ),
        )

    def pose_payload(self) -> dict:
        """A slow circle. Retained, so the map is populated the instant the page
        loads and moves afterwards."""
        angle = 2 * math.pi * ((time.monotonic() - self.started) / self.period_s)
        return protocol.pose(
            self.id,
            self.centre[0] + self.radius * math.cos(angle),
            self.centre[1] + self.radius * math.sin(angle),
            angle + math.pi / 2,
            site=self.site,
            floor=self.floor,
        )

    def tick(self, health: bool = True):
        """One round of the periodic publishes."""
        if self.dropped:
            return
        if health:
            self._publish(protocol.HEALTH, self.health_payload())
        self._publish(protocol.POSE, self.pose_payload())
        self._finish_due_task()

    # -- commands ---------------------------------------------------------

    def _on_message(self, _client, _userdata, message):
        try:
            payload = protocol.decode(message.payload, protocol.COMMAND)
        except protocol.ProtocolError as exc:
            print(f"{self.id}: refusing a malformed command: {exc}")
            return
        self._handle(payload)

    def _handle(self, payload: dict):
        command_id, text = payload["id"], payload["command"]
        with self._lock:
            previous = self._last_status.get(command_id)
            if previous is not None:
                # A redelivery is recognised, never re-run: the broker may
                # redeliver a QoS-1 command, and the correlation id is what
                # tells the two apart.
                self._publish(protocol.STATUS, previous)
                return
            busy = self._task
        if busy is not None:
            self._reply(
                command_id, text, protocol.REJECTED, f"busy with {busy['text']!r}"
            )
            return

        self._reply(command_id, text, protocol.DISPATCHED)
        refusal = self._refuse(text)
        if refusal:
            self._reply(command_id, text, protocol.REJECTED, refusal)
            return
        with self._lock:
            self._task = {
                "id": command_id,
                "text": text,
                "due": time.monotonic() + self.task_seconds,
                "summary": {
                    "id": command_id,
                    "command": text,
                    "state": protocol.ACCEPTED,
                },
            }
        self._reply(command_id, text, protocol.ACCEPTED, "running")

    def _refuse(self, text: str) -> str:
        """The task layer's grammar, as far as a wire fake can honour it."""
        words = text.split()
        if not words:
            return "empty command"
        verb, rest = words[0], words[1:]
        if verb not in VERBS:
            return f"unknown command {verb!r}"
        if verb == "goto":
            if len(rest) != 1:
                return "goto takes one zone"
            if rest[0] not in self.zones:
                return f"unknown zone {rest[0]!r}"
        if verb == "fetch" and len(rest) != 2:
            return "fetch takes a target and a drop zone"
        return ""

    def _reply(self, command_id, text, state, detail=""):
        payload = protocol.status(self.id, command_id, text, state, detail=detail)
        with self._lock:
            self._last_status[command_id] = payload
        self._publish(protocol.STATUS, payload)

    def _finish_due_task(self):
        with self._lock:
            task = self._task
            if task is None or time.monotonic() < task["due"]:
                return
            self._task = None
        self._reply(task["id"], task["text"], protocol.SUCCEEDED, "arrived")
        self._publish(protocol.HEALTH, self.health_payload())


def run(
    robots: list[FakeRobot], *, tick_s: float = 0.5, health_every: int = 6, until=None
):
    """Tick every robot until interrupted (or until ``until()`` is true).

    Pose goes out every tick so the map moves; health every few, which is close
    to the agent's own 5 s heartbeat and keeps the "health is current" rule in
    the UI on the right side of its staleness window.
    """
    stop = threading.Event()

    def _signal(_number, _frame):
        stop.set()

    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, _signal)

    count = 0
    while not stop.is_set() and not (until and until()):
        for robot in robots:
            robot.tick(health=count % health_every == 0)
        count += 1
        stop.wait(tick_s)


def build(args) -> list[FakeRobot]:
    robots = []
    for index, spec in enumerate(args.robot):
        robot_id, _, profile = spec.partition(":")
        robots.append(
            FakeRobot(
                robot_id,
                host=args.host,
                port=args.port,
                profile=profile or "ok",
                site=args.site,
                floor=args.floor,
                # Spread the circles along the default site's corridor so two
                # robots are not one dot, and none of them drives into a wall.
                centre=(index * 3.0 - 3.0, 0.0),
                radius=0.6,
                task_seconds=args.task_seconds,
                zones=tuple(args.zones.split(",")) if args.zones else ZONES,
                revision=args.revision,
            )
        )
    return robots


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="fake-robots", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument("--host", default="127.0.0.1", help="broker host")
    parser.add_argument("--port", type=int, default=1883, help="broker MQTT port")
    parser.add_argument(
        "--robot",
        action="append",
        metavar="ID[:PROFILE]",
        help=f"a robot to publish as; PROFILE is one of {'/'.join(PROFILES)} "
        "(repeatable; default: mote-01:ok mote-02:degraded mote-03:offline)",
    )
    parser.add_argument("--site", default="office_world")
    parser.add_argument("--floor", default="ground")
    parser.add_argument(
        "--revision",
        default="",
        help="the map revision these robots are running (default: report none, "
        "so the dashboard never shows a made-up out-of-date map)",
    )
    parser.add_argument(
        "--zones",
        default=",".join(ZONES),
        help="zone names `goto` will accept; anything else is rejected",
    )
    parser.add_argument(
        "--task-seconds",
        type=float,
        default=2.0,
        help="how long an accepted task takes to succeed",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds (default: run until interrupted)",
    )
    args = parser.parse_args(argv)
    args.robot = args.robot or ["mote-01:ok", "mote-02:degraded", "mote-03:offline"]

    robots = build(args)
    for robot in robots:
        robot.start()
        print(
            f"{robot.id}: {robot.profile}, publishing {protocol.topic(robot.id, '#')}"
        )
    # Let the retained state land before the will-path robot drops: an offline
    # robot the dashboard has never seen any health for is a less interesting
    # (and less realistic) row than one that reported and then vanished.
    time.sleep(1.0)
    for robot in robots:
        if robot.profile == "offline":
            robot.drop()
            print(f"{robot.id}: dropped the socket — the broker publishes its will")

    deadline = time.monotonic() + args.duration if args.duration else None
    try:
        run(robots, until=(lambda: time.monotonic() > deadline) if deadline else None)
    finally:
        for robot in robots:
            robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

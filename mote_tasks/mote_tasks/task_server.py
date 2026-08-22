"""Hosts the mission behaviour trees and executes mission/v0 missions.

What used to arrive here was a sentence — ``fetch red_box dropoff`` — parsed by
splitting on spaces, and what went back was another one: ``rejected: 'goto
nowhere' (unknown zone 'nowhere')``. Both ends of that are now typed. A
:mod:`mission <mote_bringup.spec.mission>` command names a **capability key**
and carries an **input object** validated against that capability's own
``input_schema``, and every transition goes back as a mission status whose
failures carry a class and a recoverability rather than prose. The document
saying which keys exist and what they take is :mod:`mote_tasks.capabilities`,
published retained on ``task/capabilities`` so a dispatcher can read it instead
of guessing.

Both topics are still ``std_msgs/String``; the string is now JSON. A custom
message would have bought type-checking inside the ROS graph and cost the
property that matters more — that the agent forwards these payloads to MQTT
byte-for-byte, so there is exactly one definition of the wire and the bridge
cannot reinterpret it.

**This node owns the lane**, which the fleet agent used to. The single-inflight
rule belonged upstream only because ``task/command`` had no correlation id, so
the agent could not tell one refusal from another and had to keep the robot
from ever seeing two commands. Now that a mission has an id, the executor is
the right place: it is the thing that actually holds the lane, it sees missions
issued locally on the robot as well as by the fleet, and the ``busy`` rejection
it publishes names the in-flight mission's id, which is what the spec asks for.
The agent keeps what is genuinely its own — deduplicating a redelivered
command, retaining terminal statuses, and failing one the executor never
answered.

Every status this node publishes says ``source: "local"``, and that is not a
placeholder. ``source`` is a statement about the *fleet's* view — did this
mission come from a dispatched command — and this node cannot answer it: a
command on ``task/command`` looks identical whether the agent forwarded it or a
bench script published it. The agent knows, because it knows which ids it
dispatched, and rewrites the field on the way out. So the honest answer here is
the one that claims least.

Preconditions are evaluated, not merely declared. ``localized`` is a fresh
``map``->``base_link`` transform: without one Nav2 cannot drive anyway, and
rejecting at dispatch tells the dispatcher *why* where flailing in Nav2 for two
minutes does not. ``zone_known`` is the zone lookup, and its refusal carries
zone/v0's own reason.

The active tree is ticked at ``tick_period`` while a mission is running and at
the slower ``idle_tick_period`` between them, since a tree waiting for work has
nothing to advance. Accepting a mission is unaffected either way: it happens in
the subscription callback, not on a tick.

    ros2 run mote_tasks task_server        # or: pixi run tasks
"""

import json
import os
from datetime import datetime, timezone

import py_trees
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mote_bringup import identity
from mote_bringup.spec import SpecError
from mote_bringup.spec import capability as spec_capability
from mote_bringup.spec import mission as spec_mission

from mote_tasks import capabilities, zones
from mote_tasks.trees import fetch, goto
from mote_tasks.trees.common import FAILURE_KEY, TASK_KEY

#: The capability set is state, not an event: a dispatcher that connects after
#: the robot booted must still learn what it can ask for. Transient-local is
#: the ROS half of the retained MQTT topic the agent forwards it to.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: What a robot with no identity file calls itself. It is a legal platform id,
#: it is not a lie, and a robot that has not enrolled has no fleet to confuse.
UNENROLLED = "unenrolled"

#: Whether a zone that would not resolve is worth re-dispatching unchanged.
#: Only ``stale_revision`` clears itself — the map sync installs the canonical
#: revision without anyone asking. Teaching a zone, switching floor or fixing a
#: name are all a human changing something first, which is exactly what
#: ``recoverable`` says has not happened.
ZONE_REASON_RECOVERABLE = {
    "unknown_name": False,
    "unbound": False,
    "wrong_floor": False,
    "stale_revision": True,
    "not_navigable": False,
    "ambiguous": False,
}


class TaskServer(Node):
    def __init__(self, **node_kwargs):
        super().__init__("task_server", **node_kwargs)
        zones_file = self.declare_parameter("zones_file", "").value
        tick_period = self.declare_parameter("tick_period", 0.1).value
        idle_tick_period = self.declare_parameter("idle_tick_period", 1.0).value
        pick_duration = self.declare_parameter("pick_duration", 3.0).value
        place_duration = self.declare_parameter("place_duration", 3.0).value
        platform_id = self.declare_parameter("platform_id", "").value
        # robot.yaml's max_wheel_speed, passed by the launch file rather than
        # written into capabilities.py, so the advertised speed and the one the
        # controller enforces cannot disagree. 0 means "not passed" -> null.
        max_speed = self.declare_parameter("max_speed_mps", 0.0).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.localization_timeout = self.declare_parameter(
            "localization_timeout", 5.0
        ).value

        if not zones_file:
            zones_file = os.path.join(
                get_package_share_directory("mote_tasks"),
                "config",
                "zones.default.yaml",
            )
        # load_floor, not load_zones: a name in the floor's vocabulary that
        # this robot has never been taught must reach the resolver, so a
        # mission for it can be refused as `unbound` — "I know that place,
        # nobody has driven me there" — rather than as an unknown name, which
        # sends an operator hunting for a typo that is not there.
        self.zones = zones.load_floor(zones_file)
        taught = sorted(name for name, z in self.zones.items() if z.bound)
        untaught = sorted(name for name, z in self.zones.items() if not z.bound)
        self.get_logger().info(f"Zones {taught} from {zones_file}")
        if untaught:
            self.get_logger().warning(
                f"named here but not taught on this robot: {', '.join(untaught)} "
                "(drive there and run save-zone)"
            )

        self.platform_id = platform_id or identity.robot_id() or UNENROLLED
        self.capabilities = capabilities.capability_set(
            self.platform_id, max_speed_mps=max_speed or None
        )

        self.fetch_tree = fetch.create_fetch_tree(pick_duration, place_duration)
        self.goto_tree = goto.create_goto_tree()
        for tree in (self.fetch_tree, self.goto_tree):
            tree.setup(node=self)
        #: capability key -> (tree, the function that turns input into blackboard
        #: state). Adding a capability is adding a row here and one in
        #: capabilities.py, never a branch in a dispatcher.
        self.handlers = {
            capabilities.GOTO: (self.goto_tree, self._prepare_goto),
            capabilities.FETCH: (self.fetch_tree, self._prepare_fetch),
        }
        # The active tree is the one tick() drives; the others sit idle. It
        # starts on fetch, which just idles in WaitForTask until a mission sets
        # the active tree and the task key together.
        self.active = self.fetch_tree

        self.blackboard = py_trees.blackboard.Client(name="task_server")
        for key in (
            TASK_KEY,
            FAILURE_KEY,
            fetch.OBJECT_POSE_KEY,
            fetch.OBJECT_LABEL_KEY,
            fetch.DROP_POSE_KEY,
            goto.GOTO_POSE_KEY,
        ):
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(TASK_KEY, None)
        self.blackboard.set(FAILURE_KEY, None)

        #: The in-flight mission, or None. One per lane; Mote declares one lane.
        self.mission = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, "task/status", 1)
        self.capabilities_pub = self.create_publisher(
            String, "task/capabilities", LATCHED
        )
        self.capabilities_pub.publish(String(data=json.dumps(self.capabilities)))
        self.create_subscription(String, "task/command", self.on_command, 1)
        self.last_tip = None
        # Two rates, because a tree between missions has nothing to advance: it
        # idles in WaitForTask, whose whole update() is one blackboard read, and
        # ticking that at the mission rate is ten wake-ups a second to learn
        # nothing has changed. An idle rate faster than the mission rate would
        # be a contradiction, so it is floored at it.
        self.tick_period = tick_period
        self.idle_tick_period = max(idle_tick_period, tick_period)
        self.ticking_fast = False
        self.tick_timer = self.create_timer(self.idle_tick_period, self.tick)

    def _set_tick_rate(self, active: bool):
        """Tick at the mission rate while a mission runs, slowly between them.

        The timer is *reset* as well as re-periodded, because setting a period
        does not move the expiry already pending — measured: a 5 s timer 0.3 s
        into its period still reports 4.7 s to go after its period is set to
        0.05 s, and 0.05 s after ``reset()``. Without the reset, a mission
        accepted just after an idle tick would wait out the rest of the idle
        period before the tree ticked at all. Accepting is already independent
        of the tick (``on_command`` publishes the outcome itself); what this
        protects is the first tick of the accepted tree, which is what sends
        the Nav2 goal and starts the robot driving.
        """
        if active == self.ticking_fast:
            return
        self.ticking_fast = active
        period = self.tick_period if active else self.idle_tick_period
        self.tick_timer.timer_period_ns = int(period * 1e9)
        self.tick_timer.reset()

    # -- publishing -------------------------------------------------------

    def publish_status(self, payload: dict):
        state = payload["state"]
        note = payload["detail"] or (payload["failure"] or {}).get("detail", "")
        self.get_logger().info(f"{state}: {payload['capability']} {note}".rstrip())
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _status(self, command: dict, state: str, **kwargs):
        self.publish_status(
            spec_mission.status(
                self.platform_id,
                command.get("id"),
                command.get("capability", ""),
                state,
                lane=command.get("lane") or spec_mission.DEFAULT_LANE,
                source=spec_mission.SOURCE_LOCAL,
                **kwargs,
            )
        )

    def _reject(self, command: dict, failure: dict):
        self._status(command, spec_mission.REJECTED, failure=failure)

    # -- inbound missions -------------------------------------------------

    def on_command(self, msg: String):
        try:
            command = spec_mission.check(json.loads(msg.data), "command")
        except (ValueError, SpecError) as exc:
            # No id and no capability means no status: there is nothing to
            # correlate a rejection with, and inventing an id would create a
            # mission the dispatcher never asked about.
            self.get_logger().warning(f"ignoring a malformed command: {exc}")
            return
        if command["platform_id"] != self.platform_id:
            self.get_logger().warning(
                f"ignoring a command addressed to {command['platform_id']!r}; "
                f"this platform is {self.platform_id!r}"
            )
            return

        declared = spec_capability.find(self.capabilities, command["capability"])
        if declared is None:
            offered = ", ".join(sorted(self.handlers))
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.UNKNOWN_CAPABILITY,
                    f"this platform offers {offered}",
                    at=spec_mission.DISPATCHED,
                ),
            )
            return
        pinned = command.get("capability_version")
        if pinned and pinned.split(".")[0] != declared["version"].split(".")[0]:
            # Reject rather than reinterpret: the dispatcher built this input
            # against a contract that is not the one running here.
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.UNKNOWN_CAPABILITY,
                    f"{command['capability']} {declared['version']} is offered, "
                    f"not the pinned {pinned}",
                    at=spec_mission.DISPATCHED,
                ),
            )
            return

        lane = command.get("lane") or spec_mission.DEFAULT_LANE
        if self.mission is not None and self.mission["lane"] == lane:
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.BUSY,
                    f"mission {self.mission['id']} ({self.mission['capability']}) "
                    f"holds the {lane} lane",
                    at=spec_mission.DISPATCHED,
                ),
            )
            return

        try:
            spec_capability.validate_input(declared["input_schema"], command["input"])
        except spec_capability.InvalidInput as exc:
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.INVALID_INPUT, str(exc), at=spec_mission.DISPATCHED
                ),
            )
            return

        blocked = self._preconditions(declared, command["input"])
        if blocked is not None:
            self._reject(command, blocked)
            return

        tree, prepare = self.handlers[command["capability"]]
        try:
            summary = prepare(command["input"])
        except zones.ZoneUnresolved as exc:
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.UNRESOLVED_ZONE,
                    f"{exc.reason}: {exc}",
                    recoverable=ZONE_REASON_RECOVERABLE[exc.reason],
                    at=spec_mission.DISPATCHED,
                ),
            )
            return
        except ValueError as exc:  # a defect here, not the caller's input
            self._reject(
                command,
                spec_mission.failure(
                    spec_mission.INTERNAL, str(exc), at=spec_mission.DISPATCHED
                ),
            )
            return

        self.active = tree
        self.blackboard.set(FAILURE_KEY, None)
        self.blackboard.set(TASK_KEY, summary)
        self.mission = {
            "id": command["id"],
            "capability": command["capability"],
            "lane": lane,
            "summary": summary,
            "deadline": self._deadline(declared, command),
        }
        self._set_tick_rate(active=True)
        self._status(
            command,
            spec_mission.ACCEPTED,
            detail=summary,
            warnings=self._warnings(declared, command["input"]),
        )

    def _deadline(self, declared: dict, command: dict) -> float:
        """When this mission must have finished, on the node's own clock.

        The capability's ``max_duration_s``, or the command's ``deadline`` if
        the dispatcher named an earlier one. Enforced rather than advertised:
        the capability declares ``cancellable: false``, and the spec is right
        that something unbounded which cannot be stopped is not dispatchable.
        """
        started = self._now()
        bound = started + float(declared["execution"]["max_duration_s"])
        wanted = command.get("deadline")
        if wanted:
            try:
                remaining = _seconds_until(wanted)
            except ValueError:
                self.get_logger().warning(f"ignoring unreadable deadline {wanted!r}")
                return bound
            return min(bound, started + remaining)
        return bound

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- preconditions ----------------------------------------------------

    def _localized(self) -> bool:
        """A ``map``->``base`` transform no older than ``localization_timeout``."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return False
        age = self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)
        return age < Duration(seconds=self.localization_timeout)

    def _unmet(self, item: dict, payload_input: dict) -> str | None:
        """Why this precondition does not hold, or None when it does.

        A ``custom`` precondition is machine-opaque and must not be silently
        treated as satisfied: this platform declares only non-blocking ones, so
        every unevaluated ``custom`` reaches the operator as a warning on the
        accepted status rather than as nothing at all.
        """
        kind = item["type"]
        if kind == "localized":
            return (
                None
                if self._localized()
                else (
                    f"no {self.map_frame}->{self.base_frame} transform within "
                    f"{self.localization_timeout:.0f}s"
                )
            )
        if kind == "zone_known":
            name = payload_input.get(item["input_pointer"].lstrip("/"))
            zone, reason = zones.resolve_reason(self.zones, name)
            return None if reason is None else f"zone {name!r}: {reason}"
        if kind == "custom":
            return f"not evaluated: {item['description']}"
        return f"not evaluated: {kind}"

    def _preconditions(self, declared: dict, payload_input: dict) -> dict | None:
        """The first unmet *blocking* precondition, as a failure, or None.

        ``zone_known`` is skipped here and left to the tree's own lookup, which
        raises the typed zone reason: the spec classifies an unresolved zone as
        ``unresolved_zone`` rather than ``precondition``, and reporting it twice
        under two classes would be worse than reporting it once under the right
        one.
        """
        for item in declared["preconditions"]:
            if not item.get("blocking", True) or item["type"] == "zone_known":
                continue
            unmet = self._unmet(item, payload_input)
            if unmet is not None:
                return spec_mission.failure(
                    spec_mission.PRECONDITION,
                    f"{item['type']}: {unmet}",
                    # Localisation comes back on its own once the robot sees
                    # enough of the map; nobody has to change the request.
                    recoverable=item["type"] == "localized",
                    at=spec_mission.DISPATCHED,
                )
        return None

    def _warnings(self, declared: dict, payload_input: dict) -> list:
        return [
            f"{item['type']}: {unmet}"
            for item in declared["preconditions"]
            if not item.get("blocking", True)
            and (unmet := self._unmet(item, payload_input)) is not None
        ]

    # -- preparing a tree -------------------------------------------------

    def _prepare_goto(self, payload_input: dict) -> str:
        pose, zone = goto.prepare(self.zones, payload_input)
        self.blackboard.set(goto.GOTO_POSE_KEY, pose)
        return f"goto {zone.name}"

    def _prepare_fetch(self, payload_input: dict) -> str:
        object_pose, object_label, drop_pose, drop = fetch.prepare(
            self.zones, payload_input
        )
        if object_pose is not None:
            self.blackboard.set(fetch.OBJECT_POSE_KEY, object_pose)
        else:
            self.blackboard.unset(fetch.OBJECT_POSE_KEY)
        self.blackboard.set(fetch.OBJECT_LABEL_KEY, object_label)
        self.blackboard.set(fetch.DROP_POSE_KEY, drop_pose)
        return f"fetch {object_label or payload_input['target']} -> {drop.name}"

    # -- the tick ---------------------------------------------------------

    def tick(self):
        self.active.tick()
        root = self.active.root
        tip = root.tip()
        label = f"{tip.name} [{tip.status.name}]" if tip else "-"
        if label != self.last_tip:
            self.get_logger().info(f"tree: {label}")
            self.last_tip = label
        if self.mission is None:
            return
        if root.status == py_trees.common.Status.SUCCESS:
            self._finish(spec_mission.SUCCEEDED)
        elif root.status == py_trees.common.Status.FAILURE:
            self._finish(spec_mission.FAILED, failure=self._tree_failure(label))
        elif self._now() > self.mission["deadline"]:
            self._finish(
                spec_mission.FAILED,
                failure=spec_mission.failure(
                    spec_mission.TIMEOUT,
                    f"still at {label} when the mission's time ran out",
                    # It might have been merely slow: the corridor it was
                    # waiting on clears without anyone changing the request.
                    recoverable=True,
                    at=spec_mission.ACCEPTED,
                ),
            )

    def _tree_failure(self, label: str) -> dict:
        """The failing behaviour's own account, or ``internal`` if it gave none.

        A behaviour that failed and said nothing about why is a gap in this
        software — which is what ``internal`` means — rather than a fact about
        the building, so the default must not be one of the world-shaped
        classes.
        """
        reported = self.blackboard.get(FAILURE_KEY)
        if not reported:
            return spec_mission.failure(
                spec_mission.INTERNAL,
                f"the behaviour tree failed at {label} without reporting why",
                at=spec_mission.ACCEPTED,
            )
        return spec_mission.failure(
            reported["class"],
            reported["detail"],
            recoverable=reported.get("recoverable"),
            at=spec_mission.ACCEPTED,
        )

    def _finish(self, state: str, failure: dict | None = None):
        mission, self.mission = self.mission, None
        self.blackboard.set(TASK_KEY, None)
        self.blackboard.set(FAILURE_KEY, None)
        self._set_tick_rate(active=False)
        self.publish_status(
            spec_mission.status(
                self.platform_id,
                mission["id"],
                mission["capability"],
                state,
                lane=mission["lane"],
                source=spec_mission.SOURCE_LOCAL,
                detail=mission["summary"],
                failure=failure,
            )
        )


def _seconds_until(stamp: str) -> float:
    """How long is left until an RFC 3339 instant, by wall clock.

    Wall clock deliberately, even under simulated time: a deadline is the
    dispatcher's statement about when the mission stops being worth doing, and
    it was made against a real clock.
    """
    when = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
    return max(0.0, (when - datetime.now(timezone.utc)).total_seconds())


def main():
    rclpy.init()
    node = TaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

"""``mission`` — dispatch one mission to the task server from a terminal.

Typing a mission by hand used to be ``ros2 topic pub`` with a sentence in it.
Now that ``task/command`` carries a mission/v0 payload, the same thing by hand
is forty characters of escaped JSON, which is not a bench tool — so this is the
bench tool. It publishes exactly what the fleet agent publishes and reads back
exactly what the fleet reads, so a mission that works here works dispatched, and
the reverse.

    ros2 run mote_tasks mission goto target=kitchen
    ros2 run mote_tasks mission fetch target=red_box destination=dropoff
    ros2 run mote_tasks mission --list          # what this robot offers

It is *not* a second dispatcher. There is no correlation-id bookkeeping, no
retention and no single-inflight rule here: the task server enforces the lane
and this waits for whatever it says. Anything the fleet needs is the agent's
(``mote_fleet.dispatch``).
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mote_bringup import identity
from mote_bringup.spec import SpecError
from mote_bringup.spec import mission as spec_mission

from mote_tasks.task_server import LATCHED, UNENROLLED


def parse_input(words) -> dict:
    """``target=kitchen`` pairs, or one ``{...}`` argument as the whole object.

    A ``key=value`` grammar starts guessing types the moment an input has a
    number or a boolean in it, and this tool has no business guessing where a
    capability declared them. Everything a pair produces is a string; for
    anything else, write the JSON.
    """
    words = list(words)
    if len(words) == 1 and words[0].lstrip().startswith("{"):
        parsed = json.loads(words[0])
        if not isinstance(parsed, dict):
            raise SystemExit("input JSON must be an object")
        return parsed
    payload = {}
    for word in words:
        key, sep, value = word.partition("=")
        if not sep or not key:
            raise SystemExit(f"input {word!r} is not key=value")
        payload[key] = value
    return payload


def describe(document: dict) -> str:
    lines = []
    for item in document.get("capabilities") or ():
        schema = item.get("input_schema") or {}
        required = set(schema.get("required") or ())
        fields = " ".join(
            f"{name}=<{name}>" if name in required else f"[{name}=<{name}>]"
            for name in (schema.get("properties") or {})
        )
        lines.append(f"{item['key']} {fields}".rstrip())
        lines.append(f"    {item.get('summary', '')}")
    return "\n".join(lines) or "this robot has advertised no capabilities"


class Dispatcher(Node):
    def __init__(self, platform_id: str):
        super().__init__("mission_cli")
        self.platform_id = platform_id
        self.capabilities = None
        self.statuses = []
        self.publisher = self.create_publisher(String, "task/command", 1)
        self.create_subscription(
            String, "task/capabilities", self._on_capabilities, LATCHED
        )
        self.create_subscription(String, "task/status", self._on_status, 10)

    def _on_capabilities(self, msg: String):
        self.capabilities = json.loads(msg.data)

    def _on_status(self, msg: String):
        try:
            self.statuses.append(spec_mission.check(json.loads(msg.data), "status"))
        except (ValueError, SpecError) as exc:
            print(f"! unreadable status: {exc}", file=sys.stderr)

    def spin(self, seconds: float, until=None):
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        while self.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None or until()


def report(status: dict) -> None:
    line = f"{status['stamp']}  {status['state']}"
    failure = status.get("failure")
    if failure:
        retry = "retryable" if failure["recoverable"] else "not retryable"
        line += f"  [{failure['class']}, {retry}] {failure['detail']}"
    elif status.get("detail"):
        line += f"  ({status['detail']})"
    print(line, flush=True)
    for warning in status.get("warnings") or ():
        print(f"{' ' * 26}! {warning}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="mission", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument("capability", nargs="?", help="a capability key, e.g. goto")
    parser.add_argument("input", nargs="*", help="key=value input properties")
    parser.add_argument(
        "--list", action="store_true", help="print this robot's capability set"
    )
    parser.add_argument("--platform-id", default="", help="override this robot's id")
    parser.add_argument(
        "--wait", type=float, default=300.0, help="seconds to follow the mission"
    )
    # `ros2 run` hands the tool ROS's own arguments too.
    argv = [word for word in (argv if argv is not None else sys.argv[1:])]
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    rclpy.init()
    node = Dispatcher(args.platform_id or identity.robot_id() or UNENROLLED)
    try:
        # Latched, so this is a discovery wait rather than a poll: the set is
        # already published if the task server is up.
        node.spin(3.0, until=lambda: node.capabilities is not None)
        if args.list or not args.capability:
            print(describe(node.capabilities or {}))
            return 0 if node.capabilities else 1

        payload = spec_mission.command(
            node.platform_id, args.capability, parse_input(args.input)
        )
        node.spin(2.0, until=lambda: node.publisher.get_subscription_count() > 0)
        if not node.publisher.get_subscription_count():
            print("nothing is subscribed to task/command — is `pixi run tasks` up?")
            return 1
        node.publisher.publish(String(data=json.dumps(payload)))
        print(
            f"-> {node.platform_id}: {args.capability} {payload['input']} "
            f"(id {payload['id']})"
        )

        seen = 0

        def terminal():
            nonlocal seen
            for status in node.statuses[seen:]:
                if status["id"] != payload["id"]:
                    continue  # another mission, most likely the fleet's
                report(status)
            seen = len(node.statuses)
            return any(
                s["id"] == payload["id"] and s["terminal"] for s in node.statuses
            )

        node.spin(args.wait, until=terminal)
        final = [s for s in node.statuses if s["id"] == payload["id"] and s["terminal"]]
        if not final:
            print(f"no terminal status within {args.wait:.0f}s", file=sys.stderr)
            return 1
        return 0 if final[-1]["state"] == spec_mission.SUCCEEDED else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())

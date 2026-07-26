"""``fleetctl`` — the operator's side of the control plane, from a terminal.

M1's job is to prove the loop end to end: enroll a robot, dispatch a task to it
over MQTT from off the robot's LAN, and watch the status transitions come back.
This is the tool that does that, and it is deliberately a CLI — the fleet UI is
M3, and building a browser app to find out whether the wire works would put the
proof behind the thing it is supposed to be proving.

    fleetctl token new                       mint an enrollment token
    fleetctl robots                          the registry roster
    fleetctl dispatch mote-01 "goto kitchen" send a task, follow it to terminal
    fleetctl watch                           tail the whole fleet's topics

``dispatch`` writes straight to the broker. That is right for M1 and wrong for
M3: once there are operators rather than an operator, dispatch has to be
mediated by the fleet API so it can be authorized and audited, and the browser's
broker credential becomes subscribe-only (fleet.md Q5/Q7). The topic tree does
not change when that happens — only who is allowed to publish to it.

``token`` talks to the registry file directly rather than over HTTP, because
minting a credential is a thing you do while sitting on the fleet box, and an
unauthenticated endpoint that hands out enrollment tokens would defeat the
point of having them.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json  # noqa: E402
import urllib.error  # noqa: E402
import urllib.request  # noqa: E402

from mote_fleet import protocol  # noqa: E402
from registry import Registry, default_db  # noqa: E402

DEFAULT_SERVER = "http://localhost:8080"
DEFAULT_BROKER = "localhost"


def _get(server: str, path: str) -> dict:
    url = server.rstrip("/") + path
    try:
        with urllib.request.urlopen(url, timeout=15) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        sys.exit(f"{url}: {exc.code} {exc.reason}")
    except urllib.error.URLError as exc:
        sys.exit(f"{url}: {exc.reason}")


def _client(broker: str, port: int, client_id: str):
    import paho.mqtt.client as mqtt

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:  # paho 1.x
        client = mqtt.Client(client_id=client_id)
    client.connect(broker, port, keepalive=30)
    return client


def cmd_token(args):
    registry = Registry(args.db)
    if args.action == "new":
        token = registry.new_token(single_use=not args.reusable, note=args.note)
        kind = "reusable" if args.reusable else "single-use"
        print(token)
        print(
            f"  ({kind}; give it to one robot: enroll --token {token})", file=sys.stderr
        )
        return
    for row in registry.tokens():
        state = (
            f"used by {row['used_by']} at {row['used_at']}"
            if row["used_at"]
            else "unused"
        )
        kind = "reusable" if not row["single_use"] else "single-use"
        print(f"{row['token']}  {kind:11}  {state}  {row['note']}")


def cmd_robots(args):
    robots = _get(args.server, "/v1/robots").get("robots", [])
    if not robots:
        print("no robots enrolled")
        return
    print(f"{'ID':12} {'NAME':16} {'SITE':10} {'ENROLLED':21} FINGERPRINT")
    for robot in robots:
        print(
            f"{robot['robot_id']:12} {robot['name'][:16]:16} "
            f"{(robot['site'] or '-')[:10]:10} {robot['enrolled_at']:21} "
            f"{robot['fingerprint']}"
        )


def cmd_dispatch(args):
    payload = protocol.command(" ".join(args.command), issued_by=args.issued_by)
    seen = []
    done = []

    def on_message(_client, _userdata, message):
        try:
            status = protocol.decode(message.payload, protocol.STATUS)
        except protocol.ProtocolError as exc:
            print(f"! malformed status: {exc}", file=sys.stderr)
            return
        if status["id"] != payload["id"]:
            # Another task on the same robot — most likely one started locally.
            return
        line = f"{status['stamp']}  {status['state']}"
        if status.get("detail"):
            line += f"  ({status['detail']})"
        if line in seen:
            return
        seen.append(line)
        print(line, flush=True)
        if status.get("terminal"):
            done.append(status["state"])

    client = _client(args.broker, args.port, f"fleetctl-{payload['id']}")
    client.on_message = on_message
    # Subscribe before publishing: the first transition can arrive in
    # milliseconds, and a status nobody was listening for is a status lost.
    client.subscribe(protocol.topic(args.robot_id, protocol.STATUS), qos=protocol.QOS)
    client.loop_start()
    time.sleep(0.2)
    print(f"-> {args.robot_id}: {payload['command']}  (id {payload['id']})")
    client.publish(
        protocol.topic(args.robot_id, protocol.COMMAND),
        protocol.encode(payload),
        qos=protocol.QOS,
        retain=False,  # never retained: see protocol.py
    )

    deadline = time.monotonic() + args.wait
    while not done and time.monotonic() < deadline:
        time.sleep(0.1)
    client.loop_stop()
    client.disconnect()
    if not done:
        sys.exit(f"no terminal status within {args.wait:.0f}s")
    sys.exit(0 if done[0] == protocol.SUCCEEDED else 1)


def cmd_watch(args):
    robot = args.robot_id or "+"

    def on_message(_client, _userdata, message):
        parsed = protocol.parse_topic(message.topic)
        if parsed is None:
            return
        robot_id, leaf = parsed
        try:
            payload = protocol.decode(message.payload)
        except protocol.ProtocolError:
            print(f"{robot_id:10} {leaf:12} <unparseable>")
            return
        print(f"{robot_id:10} {leaf:12} {_summarise(leaf, payload)}", flush=True)

    client = _client(args.broker, args.port, "fleetctl-watch")
    client.on_message = on_message
    for leaf in (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS):
        client.subscribe(f"{protocol.ROOT}/{protocol.VERSION}/{robot}/{leaf}", qos=1)
    print(f"watching {robot} on {args.broker}:{args.port} (ctrl-c to stop)")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        client.disconnect()


def _summarise(leaf: str, payload: dict) -> str:
    if leaf == protocol.PRESENCE:
        return (
            "online"
            if payload.get("online")
            else f"OFFLINE ({payload.get('reason', '')})"
        )
    if leaf == protocol.HEALTH:
        task = payload.get("task") or {}
        busy = f"  task={task.get('command')} [{task.get('state')}]" if task else ""
        return f"{payload.get('state')}: {payload.get('summary')}{busy}"
    if leaf == protocol.POSE:
        return (
            f"x={payload.get('x')} y={payload.get('y')} yaw={payload.get('yaw')} "
            f"({payload.get('site')}/{payload.get('floor')})"
        )
    detail = f" ({payload['detail']})" if payload.get("detail") else ""
    return f"{payload.get('state')} '{payload.get('command')}'{detail}"


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="fleetctl", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument("--server", default=DEFAULT_SERVER, help="fleet API base URL")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument(
        "--db",
        default=default_db(),
        help="registry SQLite file (default: $MOTE_FLEET_HOME/registry.db)",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_token = sub.add_parser("token", help="mint or list enrollment tokens")
    p_token.add_argument("action", choices=["new", "list"], nargs="?", default="new")
    p_token.add_argument(
        "--reusable",
        action="store_true",
        help="let more than one robot enroll with it (bench use)",
    )
    p_token.add_argument("--note", default="", help="what this token is for")
    p_token.set_defaults(func=cmd_token)

    p_robots = sub.add_parser("robots", help="list enrolled robots")
    p_robots.set_defaults(func=cmd_robots)

    p_dispatch = sub.add_parser("dispatch", help="send a task and follow it")
    p_dispatch.add_argument("robot_id")
    p_dispatch.add_argument("command", nargs="+", help="e.g. goto kitchen")
    p_dispatch.add_argument(
        "--wait", type=float, default=120.0, help="seconds to wait for a terminal state"
    )
    p_dispatch.add_argument("--issued-by", default="fleetctl")
    p_dispatch.set_defaults(func=cmd_dispatch)

    p_watch = sub.add_parser("watch", help="tail the fleet's topics")
    p_watch.add_argument("robot_id", nargs="?", help="one robot (default: all)")
    p_watch.set_defaults(func=cmd_watch)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()

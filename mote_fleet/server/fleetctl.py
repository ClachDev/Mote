"""``fleetctl`` — the operator's side of the control plane, from a terminal.

Enroll a robot, dispatch a task to it from off its LAN, and watch the status
transitions come back. The fleet UI (M3) does the same things in a browser;
this stays because a CLI composes, exits with a status, and needs no display.

    fleetctl token new                        mint an enrollment token
    fleetctl operator new --name michael      mint an operator token
    fleetctl robots                           the registry roster
    fleetctl dispatch mote-01 goto target=kitchen
                                              send a mission, follow it to terminal
    fleetctl audit                            who dispatched what
    fleetctl watch                            tail the whole fleet's topics
    fleetctl sites [site floor]               the map registry: floors, revisions
    fleetctl promote home ground <rev>        make a candidate map canonical

**Dispatch goes through the fleet API, not to the broker.** M1's version
published straight to `mission/command`, which is right when there is one operator
and no record; from M3 the API is the single write path, so every dispatch is
authorized against an operator token and written to the audit log before it is
published (fleet.md Q5/Q7). The topic tree did not change — only who may
publish to it — so ``watch`` and the status half of ``dispatch`` still read
directly from the broker, which is the cheap, live, no-service-in-the-middle
read path the design asks for.

The token for that lives in ``--token`` or ``$MOTE_FLEET_TOKEN``.

``token``/``operator`` talk to the registry file directly rather than over
HTTP, because minting a credential is a thing you do while sitting on the fleet
box, and an unauthenticated endpoint that hands out credentials would defeat the
point of having them.
"""

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json  # noqa: E402
import urllib.error  # noqa: E402
import urllib.request  # noqa: E402

from mote_bringup.spec import SpecError  # noqa: E402
from mote_bringup.spec import mission  # noqa: E402
from mote_fleet import protocol  # noqa: E402
from registry import Registry, RegistryError, default_db  # noqa: E402

DEFAULT_SERVER = "http://localhost:8080"
DEFAULT_BROKER = "localhost"
TOKEN_ENV = "MOTE_FLEET_TOKEN"


def _request(server: str, path: str, *, payload=None, token: str = "") -> dict:
    url = server.rstrip("/") + path
    request = urllib.request.Request(url)
    if payload is not None:
        request.data = json.dumps(payload).encode()
        request.add_header("Content-Type", "application/json")
        request.method = "POST"
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        detail = ""
        try:
            detail = json.loads(exc.read()).get("error", "")
        except (ValueError, OSError):
            pass
        sys.exit(f"{url}: {exc.code} {exc.reason}{f' — {detail}' if detail else ''}")
    except urllib.error.URLError as exc:
        sys.exit(f"{url}: {exc.reason}")


def _get(server: str, path: str, token: str = "") -> dict:
    return _request(server, path, token=token)


def subscriber(subscriptions):
    """An ``on_connect`` callback that subscribes to ``subscriptions``.

    A named function rather than a closure inline so the property it exists for
    — that every *re*-connect resubscribes — is testable without a broker.
    """

    def on_connect(client, _userdata, *_args):
        for topic in subscriptions:
            client.subscribe(topic, qos=protocol.QOS)

    return on_connect


def _client(broker: str, port: int, client_id: str, subscriptions=()):
    """A connected client that **re-subscribes every time it connects**.

    Subscriptions belong to an MQTT session, and paho's default session is a
    clean one — so a client that subscribes once, at startup, and then survives
    a broker restart comes back subscribed to *nothing*. It stays connected and
    goes silent forever, which is indistinguishable from a quiet fleet. That is
    why the subscribe lives in ``on_connect`` rather than beside the connect,
    the same arrangement the agent uses (``agent.py:_on_connect``).
    """
    import paho.mqtt.client as mqtt

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:  # paho 1.x
        client = mqtt.Client(client_id=client_id)

    client.on_connect = subscriber(subscriptions)
    # Reconnect forever with backoff: an operator's terminal should outlive a
    # fleet-server redeploy without them noticing.
    client.reconnect_delay_set(min_delay=1, max_delay=30)
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


def cmd_sites(args):
    """The map registry. Without a floor: what every floor is on. With one:
    every candidate revision and whether it could be promoted."""
    if not (args.site and args.floor):
        floors = _get(args.server, "/v1/sites").get("sites", [])
        if not floors:
            print("no site bundles on the fleet server")
            return
        print(f"{'SITE':16} {'FLOOR':10} {'CANONICAL':18} CANDIDATES")
        for floor in floors:
            candidates = ", ".join(floor["candidates"]) or "-"
            print(
                f"{floor['site'][:16]:16} {floor['floor'][:10]:10} "
                f"{(floor['canonical'] or '-'):18} {candidates}"
            )
        return
    detail = _get(args.server, f"/v1/sites/{args.site}/floors/{args.floor}")
    print(f"{args.site}/{args.floor}  canonical: {detail['canonical'] or 'none'}")
    for revision in detail["revisions"]:
        marker = "*" if revision["canonical"] else " "
        who = revision.get("robot_id") or revision.get("uploaded_by") or "-"
        when = revision.get("uploaded_at") or "-"
        zones = len(revision.get("zones") or [])
        state = "ok" if revision["ok"] else "UNUSABLE"
        print(
            f" {marker} {revision['revision']:20} {state:9} {who:10} {when:21} {zones} zones"
        )
        for problem in revision["errors"] + revision["warnings"]:
            print(f"     - {problem}")


def cmd_promote(args):
    """Make one candidate revision the floor's canonical map.

    This is the one write in the whole map path, which is why it is an
    operator's: uploads are inert candidates, and nothing a robot does changes
    which map a floor is on (fleet.md Q4).
    """
    answer = _request(
        args.server,
        f"/v1/sites/{args.site}/floors/{args.floor}/revisions/{args.revision}/promote",
        payload={"schema": protocol.SCHEMA},
        token=_token(args),
    )
    print(f"{args.site}/{args.floor} -> {answer['revision']}  ({answer['sha256']})")
    for warning in answer.get("warnings") or []:
        print(f"  note: {warning}")
    if answer.get("announced"):
        print(f"  announced on {answer['topic']} (retained); agents will pull it")
    else:
        # The flip already happened — the symlink is the fact. Say so plainly
        # rather than implying the promotion failed.
        print(
            f"  PROMOTED BUT NOT ANNOUNCED: {answer.get('detail')}\n"
            "  robots keep their current map until the broker is reachable; "
            "the fleet server re-announces every floor at startup."
        )


def cmd_operator(args):
    registry = Registry(args.db)
    if args.action == "new":
        try:
            token = registry.new_operator(name=args.name, note=args.note)
        except RegistryError as exc:
            sys.exit(str(exc))
        print(token)
        print(
            f"  (operator '{args.name}'; export {TOKEN_ENV}={token}, or paste it "
            "into the dashboard)",
            file=sys.stderr,
        )
        return
    if args.action == "revoke":
        if not args.token:
            sys.exit("which token? fleetctl operator revoke --token <token>")
        sys.exit(0 if registry.revoke_operator(args.token) else "no such live token")
    for row in registry.operators():
        state = f"revoked {row['revoked_at']}" if row["revoked_at"] else "live"
        used = row["last_used_at"] or "never used"
        print(f"{row['name']:16} {state:24} {used:22} {row['note']}")


def cmd_audit(args):
    query = f"/v1/audit?limit={args.limit}"
    if args.robot_id:
        query += f"&robot_id={args.robot_id}"
    rows = _get(args.server, query, token=_token(args)).get("audit", [])
    if not rows:
        print("nothing dispatched yet")
        return
    print(f"{'WHEN':21} {'WHO':14} {'ROBOT':10} {'RESULT':12} COMMAND")
    for row in reversed(rows):
        detail = f"  ({row['detail']})" if row["detail"] else ""
        print(
            f"{row['stamp']:21} {row['actor'][:14]:14} {row['robot_id'][:10]:10} "
            f"{row['result']:12} {row['command']}{detail}"
        )


def _token(args) -> str:
    token = args.token or os.environ.get(TOKEN_ENV, "")
    if not token:
        sys.exit(
            f"an operator token is required: --token, or {TOKEN_ENV} in the "
            "environment. Mint one on the fleet box with "
            "'fleetctl operator new --name <you>'."
        )
    return token


def _input(words) -> dict:
    """Turn ``target=kitchen`` pairs into a mission input object.

    A single ``{...}`` argument is taken as the whole object instead, because
    the moment an input has a number, a boolean or a nested value in it, a
    ``key=value`` shell grammar starts guessing types — and this CLI has no
    business guessing where a capability declared them. Everything a pair
    produces is a string; anything else, write the JSON.
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


def cmd_dispatch(args):
    """Dispatch through the API, then follow the robot's own status on the
    broker. Two connections because they are two different things: the write is
    authorized and recorded by the fleet server, the read is the live control
    plane with nothing in the middle."""
    received = []
    seen = []
    done = []

    def on_message(_client, _userdata, message):
        # Collect, do not filter: the robot's first transitions can arrive
        # before the HTTP response has told us which correlation id to look
        # for, and a status discarded then is a status lost.
        try:
            received.append(mission.check(json.loads(message.payload), "status"))
        except (ValueError, SpecError) as exc:
            print(f"! malformed status: {exc}", file=sys.stderr)

    def report(command_id):
        for status in list(received):
            if status["id"] != command_id:
                # Another mission on the same robot — most likely a local one.
                continue
            line = f"{status['stamp']}  {status['state']}"
            failure = status.get("failure")
            if failure:
                # The class first and the prose after it: the class is what a
                # caller acts on, and putting it last would bury it behind a
                # sentence of variable length.
                retry = "retryable" if failure["recoverable"] else "not retryable"
                line += f"  [{failure['class']}, {retry}] {failure['detail']}"
            elif status.get("detail"):
                line += f"  ({status['detail']})"
            for warning in status.get("warnings") or ():
                line += f"\n{' ' * 26}! {warning}"
            if line in seen:
                continue
            seen.append(line)
            print(line, flush=True)
            if status.get("terminal"):
                done.append(status["state"])

    token = _token(args)
    # Subscribe before dispatching: the first transition can arrive in
    # milliseconds, and a status nobody was listening for is a status lost.
    client = _client(
        args.broker,
        args.port,
        f"fleetctl-{os.getpid()}",
        subscriptions=[protocol.topic(args.robot_id, protocol.STATUS)],
    )
    client.on_message = on_message
    client.loop_start()
    time.sleep(0.2)
    answer = _request(
        args.server,
        f"/v1/robots/{args.robot_id}/dispatch",
        payload={
            "schema": mission.SCHEMA,
            "capability": args.capability,
            "input": _input(args.input),
            "issued_by": args.issued_by,
        },
        token=token,
    )
    command_id = answer["id"]
    print(
        f"-> {args.robot_id}: {answer['capability']} {answer['input']}  "
        f"(id {command_id})"
    )

    deadline = time.monotonic() + args.wait
    while not done and time.monotonic() < deadline:
        report(command_id)
        time.sleep(0.1)
    report(command_id)
    client.loop_stop()
    client.disconnect()
    if not done:
        sys.exit(f"no terminal status within {args.wait:.0f}s")
    sys.exit(0 if done[0] == mission.SUCCEEDED else 1)


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

    client = _client(
        args.broker,
        args.port,
        "fleetctl-watch",
        subscriptions=[
            f"{protocol.ROOT}/{protocol.VERSION}/{robot}/{leaf}"
            for leaf in (
                protocol.PRESENCE,
                protocol.HEALTH,
                protocol.POSE,
                protocol.STATUS,
                protocol.CAPABILITIES,
            )
        ],
    )
    client.on_message = on_message
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
        running = payload.get("mission") or {}
        busy = (
            f"  mission={running.get('capability')} [{running.get('state')}]"
            if running
            else ""
        )
        return f"{payload.get('state')}: {payload.get('summary')}{busy}"
    if leaf == protocol.POSE:
        return (
            f"x={payload.get('x')} y={payload.get('y')} yaw={payload.get('yaw')} "
            f"({payload.get('site')}/{payload.get('floor')})"
        )
    if leaf == protocol.CAPABILITIES:
        keys = [item["key"] for item in payload.get("capabilities") or ()]
        return f"revision {payload.get('revision')}: {', '.join(keys) or 'none'}"
    failure = payload.get("failure")
    note = f" [{failure['class']}] {failure['detail']}" if failure else ""
    return f"{payload.get('state')} {payload.get('capability')}{note}"


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
    parser.add_argument(
        "--token",
        default="",
        help=f"operator token for the write routes (default: ${TOKEN_ENV})",
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

    p_operator = sub.add_parser("operator", help="mint or list operator tokens")
    p_operator.add_argument(
        "action", choices=["new", "list", "revoke"], nargs="?", default="list"
    )
    p_operator.add_argument("--name", default="", help="who this token is for")
    p_operator.add_argument("--note", default="", help="what it is for")
    p_operator.set_defaults(func=cmd_operator)

    p_robots = sub.add_parser("robots", help="list enrolled robots")
    p_robots.set_defaults(func=cmd_robots)

    p_audit = sub.add_parser("audit", help="what was dispatched, by whom")
    p_audit.add_argument("--limit", type=int, default=50)
    p_audit.add_argument("--robot-id", default="", dest="robot_id")
    p_audit.set_defaults(func=cmd_audit)

    p_dispatch = sub.add_parser("dispatch", help="send a mission and follow it")
    p_dispatch.add_argument("robot_id")
    p_dispatch.add_argument("capability", help="a capability key, e.g. goto")
    p_dispatch.add_argument(
        "input",
        nargs="*",
        help="key=value input properties, e.g. target=kitchen. A single "
        "argument that starts with '{' is read as the whole input object.",
    )
    p_dispatch.add_argument(
        "--wait", type=float, default=120.0, help="seconds to wait for a terminal state"
    )
    p_dispatch.add_argument("--issued-by", default="fleetctl")
    p_dispatch.set_defaults(func=cmd_dispatch)

    p_sites = sub.add_parser("sites", help="the map registry")
    p_sites.add_argument("site", nargs="?", default="")
    p_sites.add_argument("floor", nargs="?", default="")
    p_sites.set_defaults(func=cmd_sites)

    p_promote = sub.add_parser("promote", help="make a map revision canonical")
    p_promote.add_argument("site")
    p_promote.add_argument("floor")
    p_promote.add_argument("revision")
    p_promote.set_defaults(func=cmd_promote)

    p_watch = sub.add_parser("watch", help="tail the fleet's topics")
    p_watch.add_argument("robot_id", nargs="?", help="one robot (default: all)")
    p_watch.set_defaults(func=cmd_watch)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()

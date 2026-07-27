"""``fleetctl`` — the operator's side of the control plane, from a terminal.

Enroll a robot, dispatch a task to it from off its LAN, and watch the status
transitions come back. The fleet UI (M3) does the same things in a browser;
this stays because a CLI composes, exits with a status, and needs no display.

    fleetctl token new                        mint an enrollment token
    fleetctl operator new --name michael      mint an operator token
    fleetctl broker sync                      push credentials to the broker
    fleetctl robots                           the registry roster
    fleetctl dispatch mote-01 "goto kitchen"  send a task, follow it to terminal
    fleetctl audit                            who dispatched what
    fleetctl watch                            tail the whole fleet's topics

**Everything except `token`/`operator`/`broker` needs an operator token** since
M7 — the API's read routes are authorized too, not just dispatch — and the
broker is no longer anonymous, so `watch` and `dispatch` fetch the operator's
own subscribe-only broker login from `/v1/config` rather than connecting
unauthenticated. One credential in your environment, both paths.

**Dispatch goes through the fleet API, not to the broker.** M1's version
published straight to `task/command`, which is right when there is one operator
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

import credentials  # noqa: E402
from mote_fleet import protocol  # noqa: E402
from registry import Registry, RegistryError, default_db, fleet_home  # noqa: E402

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


def _broker_credential(args) -> tuple[str, str] | None:
    """This operator's subscribe-only broker login, from ``/v1/config``.

    Since M7 the broker is not anonymous, so ``watch`` and the status half of
    ``dispatch`` need a credential — and the right one to use is the operator's
    own, fetched with the operator token they already hold. The alternative,
    reading the registry file, would only work while sitting on the fleet box,
    which is exactly the constraint ``fleetctl`` exists to avoid.
    """
    config = _get(args.server, "/v1/config", token=_token(args))
    broker = config.get("broker") or {}
    if not broker.get("username"):
        return None
    return broker["username"], broker.get("password") or ""


def _client(broker: str, port: int, client_id: str, subscriptions=(), credential=None):
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

    if credential:
        client.username_pw_set(*credential)
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
    robots = _get(args.server, "/v1/robots", token=_token(args)).get("robots", [])
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


def _sync_broker(registry, args, why: str):
    """Push the registry's credentials to the broker, and say what happened.

    Minting or revoking an operator changes who may connect, and a change the
    broker has not read is not in force yet — a revoked operator would keep
    receiving the fleet's telemetry until something reloaded it. Reported rather
    than silent, because the files are right on disk either way and the operator
    needs to know which half succeeded.
    """
    ok, detail = credentials.sync(
        registry, args.broker_auth_dir, reload_cmd=args.broker_reload_cmd or None
    )
    if ok:
        print(f"  (broker credentials {why} and reloaded)", file=sys.stderr)
    else:
        print(
            f"  (broker credentials {why}, but the broker did not reload: "
            f"{detail or 'no running broker'} — it will read them on next start)",
            file=sys.stderr,
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
        _sync_broker(registry, args, "written")
        return
    if args.action == "revoke":
        if not args.token:
            sys.exit("which token? fleetctl operator revoke --token <token>")
        if not registry.revoke_operator(args.token):
            sys.exit("no such live token")
        # Revocation is only real once the broker has forgotten the credential.
        _sync_broker(registry, args, "rewritten")
        return
    for row in registry.operators():
        state = f"revoked {row['revoked_at']}" if row["revoked_at"] else "live"
        used = row["last_used_at"] or "never used"
        print(
            f"{row['name']:16} {state:24} {used:22} "
            f"{row['broker_user'] or '-':24} {row['note']}"
        )


def cmd_broker(args):
    """Regenerate the broker's credential files from the registry, and reload.

    The fleet server does this by itself on every enrollment. This verb is for
    the two cases where nothing else would: a broker restarted with an empty
    file, and a registry restored from a backup.
    """
    registry = Registry(args.db)
    auth = credentials.BrokerAuth(args.broker_auth_dir)
    principals = registry.broker_principals()
    if args.action == "show":
        print(f"password_file {auth.passwd_path}")
        print(f"acl_file      {auth.acl_path}")
        print(
            f"principals    {len(principals['robots'])} robots, "
            f"{len(principals['operators'])} operators, 1 server"
        )
        for user in sorted(principals["users"]):
            print(f"  {user}")
        return
    _sync_broker(registry, args, "regenerated")


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
            received.append(protocol.decode(message.payload, protocol.STATUS))
        except protocol.ProtocolError as exc:
            print(f"! malformed status: {exc}", file=sys.stderr)

    def report(command_id):
        for status in list(received):
            if status["id"] != command_id:
                # Another task on the same robot — most likely a local one.
                continue
            line = f"{status['stamp']}  {status['state']}"
            if status.get("detail"):
                line += f"  ({status['detail']})"
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
        credential=_broker_credential(args),
    )
    client.on_message = on_message
    client.loop_start()
    time.sleep(0.2)
    answer = _request(
        args.server,
        f"/v1/robots/{args.robot_id}/dispatch",
        payload={
            "schema": protocol.SCHEMA,
            "command": " ".join(args.command),
            "issued_by": args.issued_by,
        },
        token=token,
    )
    command_id = answer["id"]
    print(f"-> {args.robot_id}: {answer['command']}  (id {command_id})")

    deadline = time.monotonic() + args.wait
    while not done and time.monotonic() < deadline:
        report(command_id)
        time.sleep(0.1)
    report(command_id)
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
            )
        ],
        credential=_broker_credential(args),
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
    parser.add_argument(
        "--token",
        default="",
        help=f"operator token for the API (default: ${TOKEN_ENV})",
    )
    parser.add_argument(
        "--broker-auth-dir",
        default=str(fleet_home() / "broker"),
        help="the broker's generated password_file/acl_file directory "
        "(default: $MOTE_FLEET_HOME/broker)",
    )
    parser.add_argument(
        "--broker-reload-cmd",
        default=" ".join(credentials.DEFAULT_RELOAD_CMD),
        help="how to make a running broker re-read those files (default: "
        "broker.sh reload). Needed when the broker was not started by "
        "broker.sh — e.g. a container someone else runs.",
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

    p_broker = sub.add_parser(
        "broker", help="regenerate the broker's credential files from the registry"
    )
    p_broker.add_argument("action", choices=["sync", "show"], nargs="?", default="sync")
    p_broker.set_defaults(func=cmd_broker)

    p_robots = sub.add_parser("robots", help="list enrolled robots")
    p_robots.set_defaults(func=cmd_robots)

    p_audit = sub.add_parser("audit", help="what was dispatched, by whom")
    p_audit.add_argument("--limit", type=int, default=50)
    p_audit.add_argument("--robot-id", default="", dest="robot_id")
    p_audit.set_defaults(func=cmd_audit)

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

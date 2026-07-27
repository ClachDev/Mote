"""``enroll`` — ask the fleet server who this robot is.

M0 had the operator type an id into ``~/.mote/robot.yaml``. From M1 the server
owns the id space, and this is the robot side of that exchange: present an
enrollment token and a hardware fingerprint, receive an id, a name, and the
address of the control-plane broker, then write both files the agent needs::

    $MOTE_HOME/robot.yaml    id / name / site      (mote_bringup.identity)
    $MOTE_HOME/fleet.yaml    server + broker       (mote_fleet.fleet_config)

Three properties are worth stating because they are what make this safe to run
from an unattended first boot:

**It is idempotent.** The server keys on the hardware fingerprint
(:mod:`mote_fleet.facts`), so running it twice — or after a wiped ``~/.mote`` —
returns the same id rather than minting a second fleet member.

**It adopts an existing id rather than replacing it.** A robot that already has
an M0 operator-set id offers that id to the server, which records it as-is. The
M0 fleet does not get renumbered by upgrading to M1; it gets registered.

**It refuses to silently re-key a robot.** If the server answers with an id
different from the one on disk, that is a real conflict — two enrollments, or a
recycled fingerprint — so it stops and says so instead of rewriting identity
under a running fleet. ``--force`` is the deliberate override.

    pixi run enroll -- --server http://fleet-box:8080 --token <token>
    pixi run enroll -- --token <token> --name Scout --site home
"""

import argparse
import json
import sys
import urllib.error
import urllib.request

from mote_bringup import identity

from mote_fleet import facts, fleet_config, protocol

TIMEOUT = 15.0


def post_enroll(server: str, payload: dict, timeout: float = TIMEOUT) -> dict:
    """POST /v1/enroll, raising RuntimeError with the server's own reason."""
    url = server.rstrip("/") + "/v1/enroll"
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        try:
            reason = json.loads(exc.read()).get("error", exc.reason)
        except Exception:
            reason = exc.reason
        raise RuntimeError(f"{url}: {exc.code} {reason}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"{url}: {exc.reason}") from exc


def request_enrollment(
    *, server: str, token: str, name: str = "", site: str = "", requested_id: str = ""
) -> dict:
    """Run the exchange. Writes nothing — the caller decides to accept it."""
    collected = facts.collect()
    payload = {
        "schema": protocol.SCHEMA,
        "token": token,
        "fingerprint": facts.fingerprint(collected),
        "facts": collected,
        "name": name,
        "site": site,
        "robot_id": requested_id,
    }
    answer = post_enroll(server, payload)
    if not protocol.valid_id(answer.get("robot_id") or ""):
        raise RuntimeError(
            f"server returned an unusable robot id: {answer.get('robot_id')!r}"
        )
    return answer


def persist(server: str, answer: dict) -> dict:
    """Write the identity and fleet files the agent reads."""
    robot_id = answer["robot_id"]
    identity.set_identity(
        id=robot_id,
        name=answer.get("name") or robot_id,
        site=answer.get("site") or None,
    )
    broker = answer.get("broker") or {}
    return fleet_config.save(
        server=server,
        broker_host=broker.get("host") or "",
        broker_port=broker.get("port") or fleet_config.DEFAULT_BROKER_PORT,
        # M7: the answer also carries this robot's broker credential, and this
        # is the only time it is ever sent. Enrolling again issues a new one.
        broker_username=broker.get("username") or "",
        broker_password=broker.get("password") or "",
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="enroll", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument(
        "--server",
        help="fleet server base URL (default: the one this robot enrolled with)",
    )
    parser.add_argument("--token", required=True, help="enrollment token")
    parser.add_argument("--name", default="", help="human label, e.g. 'Scout'")
    parser.add_argument("--site", default="", help="site whose bundles this robot uses")
    parser.add_argument(
        "--id",
        default="",
        help="ask for a specific id (defaults to the one already on disk, so an "
        "M0 robot keeps its id)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="accept an id that differs from the one already on disk",
    )
    args = parser.parse_args(argv)

    current = identity.load() or {}
    config = fleet_config.load() or {}
    server = args.server or config.get("server")
    if not server:
        sys.exit(
            "no --server given and this robot has never enrolled "
            f"(expected {fleet_config.config_path()})"
        )

    requested = args.id or current.get("id", "")
    if requested and not protocol.valid_id(requested):
        sys.exit(f"invalid robot id {requested!r}")

    try:
        answer = request_enrollment(
            server=server,
            token=args.token,
            name=args.name or current.get("name", ""),
            site=args.site or current.get("site", ""),
            requested_id=requested,
        )
    except (RuntimeError, ValueError) as exc:
        sys.exit(f"enrollment failed: {exc}")

    robot_id = answer["robot_id"]
    if current.get("id") and current["id"] != robot_id and not args.force:
        sys.exit(
            f"the server enrolled this machine as {robot_id}, but "
            f"{identity.identity_path()} says {current['id']}. Nothing was "
            "written. Re-run with --force to take the server's answer, or with "
            f"--id {current['id']} to keep this one."
        )
    persist(server, answer)

    broker = answer.get("broker") or {}
    print(f"enrolled as {robot_id} ({'new' if answer.get('created') else 'existing'})")
    print(f"  identity: {identity.identity_path()}")
    print(f"  fleet:    {fleet_config.config_path()}")
    print(f"  broker:   {broker.get('host')}:{broker.get('port')}")
    if broker.get("username"):
        print(f"  login:    {broker['username']} (password written, not shown)")
    else:
        print(
            "  login:    none — this server issued no broker credential, so the "
            "agent will connect anonymously"
        )
    print("next: start the agent with 'pixi run agent'")


if __name__ == "__main__":
    main()

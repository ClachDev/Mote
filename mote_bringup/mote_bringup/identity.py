"""Robot identity: the stable ``robot_id`` this machine is known by.

    $MOTE_HOME/robot.yaml
        schema: 1
        id: mote-01          # the fleet's primary key
        name: Front desk     # human label, free text
        site: home           # which site's bundles this robot is entitled to

The ``id`` is the fleet's primary key: it keys the MQTT topic tree
(``mote/<robot_id>/...``), the registry row, and the robot's MagicDNS name on
the tailnet. It is deliberately *not* the hostname — the hostname is already
ambiguous in this repo (``auldbot`` in ``pixi.toml`` vs ``mote`` in the docs) and
re-imaging a Pi must not silently mint a new fleet member. It lives in
``MOTE_HOME`` (see :mod:`mote_bringup.mote_home`), so it is stable across
reboots *and* across software updates, which replace only the package.

Do not confuse this file with ``mote_description/config/robot.yaml``: that one
is the shared hardware description that ships identically to every robot.

Until the fleet server exists, the id is **operator-set** — assigned here, or
written by the cloud-init provisioning template at first boot
(``mote_bringup/provisioning/``). Once enrollment lands the server allocates the
id and the agent rewrites this file as a cache; the shape does not change.

Console script ``identity`` (pixi task: identity):
    identity show                 the full record (and where it came from)
    identity id                   just the id, for scripts
    identity set --id mote-01 [--name "Front desk"] [--site home]
"""

import argparse
import os
import re
import sys

import yaml

from mote_bringup import mote_home

SCHEMA = 1

# Lowercase DNS label: it becomes a MagicDNS hostname, an MQTT topic level, and a
# directory name, so keep it to the intersection all three accept.
ID_RE = re.compile(r"^[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?$")

FIELDS = ("id", "name", "site")


def identity_path():
    return mote_home.path("robot.yaml")


def validate_id(robot_id: str) -> str:
    if not ID_RE.match(robot_id or ""):
        raise ValueError(
            f"invalid robot id {robot_id!r}: use lowercase letters, digits and "
            "hyphens (max 32 chars, must start and end alphanumeric), e.g. mote-01"
        )
    return robot_id


def load() -> dict | None:
    """This robot's identity record, or None if it has never been set."""
    path = identity_path()
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text()) or {}
    if not data.get("id"):
        return None
    return data


def robot_id() -> str | None:
    """This robot's id, or None if unset."""
    data = load()
    return data["id"] if data else None


def require_id() -> str:
    """This robot's id, or exit with the command that sets it."""
    rid = robot_id()
    if not rid:
        sys.exit(
            f"no robot identity at {identity_path()} — set one with:\n"
            "    pixi run identity set --id mote-01 --name 'Front desk'"
        )
    return rid


def set_identity(*, id=None, name=None, site=None) -> dict:
    """Create or update the identity record, keeping unspecified fields."""
    data = load() or {}
    if id is not None:
        data["id"] = validate_id(id)
    if name is not None:
        data["name"] = name
    if site is not None:
        data["site"] = site
    if not data.get("id"):
        raise ValueError("an id is required: identity set --id mote-01")
    data.setdefault("name", data["id"])
    record = {"schema": SCHEMA, **{k: data[k] for k in FIELDS if data.get(k)}}

    path = identity_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".robot.yaml.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(record, sort_keys=False))
    os.replace(tmp, path)
    return record


def main():
    parser = argparse.ArgumentParser(
        prog="identity", description=__doc__.split("\n\n")[0]
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("show", help="print this robot's identity record")
    sub.add_parser("id", help="print just the robot id")
    p_set = sub.add_parser("set", help="create or update the identity record")
    p_set.add_argument("--id", help="fleet primary key, e.g. mote-01")
    p_set.add_argument("--name", help="human label, e.g. 'Front desk'")
    p_set.add_argument("--site", help="site whose bundles this robot uses")
    args = parser.parse_args()

    if args.cmd == "set":
        try:
            record = set_identity(id=args.id, name=args.name, site=args.site)
        except ValueError as exc:
            sys.exit(str(exc))
        print(f"{identity_path()}:")
        print(yaml.safe_dump(record, sort_keys=False).rstrip())
        return

    if args.cmd == "id":
        print(require_id())
        return

    data = load()
    if not data:
        print(f"no identity set (expected {identity_path()})")
        print("    pixi run identity set --id mote-01 --name 'Front desk'")
        return
    print(f"{identity_path()}:")
    print(yaml.safe_dump(data, sort_keys=False).rstrip())


if __name__ == "__main__":
    main()

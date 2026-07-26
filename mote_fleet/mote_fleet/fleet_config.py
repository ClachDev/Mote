"""Where this robot's fleet lives: ``$MOTE_HOME/fleet.yaml``.

    schema: 1
    server: http://fleet-box:8080     # enrollment + registry API
    broker:
      host: fleet-box                 # MQTT control plane
      port: 1883

Written by ``enroll`` from the server's own answer, so a robot learns its
broker from the same exchange that gave it its id — there is no second thing to
configure and no way for the two to disagree.

It sits beside ``robot.yaml`` under ``MOTE_HOME`` for the same reason identity
does: it is per-robot state, so an update replaces the package around it and
cannot take the robot off the fleet. The host names are normally MagicDNS names
on the tailnet (``fleet-box``, not an IP), which is what keeps the file valid
when the fleet server changes networks.
"""

import os

import yaml

from mote_bringup import mote_home

SCHEMA = 1

DEFAULT_BROKER_PORT = 1883


def config_path():
    return mote_home.path("fleet.yaml")


def load() -> dict | None:
    """This robot's fleet config, or None if it has never enrolled."""
    path = config_path()
    if not path.exists():
        return None
    return yaml.safe_load(path.read_text()) or None


def broker(config: dict | None = None) -> tuple[str, int] | None:
    """``(host, port)`` of the control-plane broker, or None if unconfigured."""
    config = load() if config is None else config
    if not config:
        return None
    entry = config.get("broker") or {}
    host = entry.get("host")
    if not host:
        return None
    return host, int(entry.get("port", DEFAULT_BROKER_PORT))


def save(*, server: str, broker_host: str, broker_port: int = DEFAULT_BROKER_PORT):
    record = {
        "schema": SCHEMA,
        "server": server,
        "broker": {"host": broker_host, "port": int(broker_port)},
    }
    path = config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".fleet.yaml.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(record, sort_keys=False))
    os.replace(tmp, path)
    return record

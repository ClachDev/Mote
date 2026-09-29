"""The ROS-free half of loading a promoted map into a running Nav2.

Two messages cross the ROS graph, both ``std_msgs/String`` carrying JSON on
latched topics, so a node that starts late still learns the current state:

``map/installed`` — published by the fleet agent after ``mapsync`` has staged a
revision and flipped the floor's ``map`` symlink. It is a fact about the disk,
not an instruction: the agent says what it installed and ``map_reloader``
decides when Nav2 takes it.

``map/serving`` — published by ``map_reloader``: the revision ``map_server`` is
serving now, whether a load is under way, and the reason the last load failed.
Health's ``map.revision`` reads it, so the fleet sees the revision Nav2 is
actually using rather than the one on disk.

What lives here is everything about those two messages and the site layout that
does not need a graph, so it is tested as plain function calls.
"""

import json
import os
from pathlib import Path

from mote_bringup import sites

INSTALLED_TOPIC = "map/installed"
SERVING_TOPIC = "map/serving"
#: The task server's answer to "may the map change now?" (std_srvs/Trigger):
#: success is true when no mission holds the lane.
IDLE_SERVICE = "task/idle"


def locate(map_yaml: str) -> tuple[str, str, str] | None:
    """``(site, floor, revision)`` of a map yaml inside the sites tree.

    Resolves the floor's ``map`` symlink, so the path ``nav2_launch.py`` hands
    ``map_server`` names the revision that symlink pointed at when this is
    called. ``None`` for a map outside the tree (a bare ``map:=/tmp/x.yaml``),
    which has no floor for a promotion to name.
    """
    if not map_yaml:
        return None
    try:
        real = Path(os.path.realpath(map_yaml))
        rel = real.relative_to(Path(os.path.realpath(sites.sites_dir())))
    except (OSError, ValueError):
        return None
    parts = rel.parts
    # <site>/floors/<floor>/maps/<rev>/map.yaml
    if len(parts) != 6 or parts[1] != "floors" or parts[3] != "maps":
        return None
    return parts[0], parts[2], parts[4]


def revision_yaml(site: str, floor: str, revision: str) -> Path:
    """The map yaml of one revision, by its immutable directory.

    Loaded through the revision directory rather than the ``map`` symlink, so
    what ``map_server`` was asked to serve is exactly the revision reported.
    """
    return sites.floor_dir(site, floor) / "maps" / revision / "map.yaml"


def installed(site: str, floor: str, revision: str) -> str:
    return json.dumps({"site": site, "floor": floor, "revision": revision})


def serving(
    site: str | None,
    floor: str | None,
    revision: str | None,
    *,
    loading: str | None = None,
    error: str | None = None,
) -> str:
    return json.dumps(
        {
            "site": site,
            "floor": floor,
            "revision": revision,
            "loading": loading,
            "error": error,
        }
    )


def parse(raw: str) -> dict | None:
    """A decoded ``map/installed`` or ``map/serving`` payload, or None."""
    try:
        payload = json.loads(raw)
    except ValueError:
        return None
    return payload if isinstance(payload, dict) else None


def wanted(serving_now: tuple, installed_now: dict | None) -> str | None:
    """The revision to load, or None when there is nothing to do.

    ``serving_now`` is ``(site, floor, revision)``. Only an installation on
    the floor being served counts: the agent keeps every floor this robot
    holds current, and a new map for the floor upstairs is not a reason to
    touch the one Nav2 is driving on.
    """
    if not installed_now:
        return None
    site, floor, revision = serving_now
    if site is None or (installed_now.get("site"), installed_now.get("floor")) != (
        site,
        floor,
    ):
        return None
    target = installed_now.get("revision")
    return target if target and target != revision else None

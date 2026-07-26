"""Named places (zones) loaded from a zones YAML file.

A **zone** is a named pose in a map frame the robot can navigate to — as a
fetch waypoint (`pickup`/`dropoff`) or a `goto <zone>` target. A zone may also
carry an **area footprint**, so it can answer "is (x, y) inside this zone?".
The footprint is optional metadata on the one zone concept, not a separate
kind of thing: a bare zone is just a pose; a room-like zone adds a footprint.
Today the only footprint is a circle (`radius`); a `polygon` slots in the same
place once map room-detection can emit one (see the polygon follow-up).

Schema:

    frame_id: <str, optional, default "map">
    zones:
      <name>:
        x: <float, required>
        y: <float, required>
        yaw: <float, optional, default 0.0>
        radius: <float, optional>    # circular footprint centred on (x, y)
        # polygon: [[x, y], ...]     # (future) explicit footprint vertices

x and y are metres in frame_id; yaw is radians about +z.

Example:

    frame_id: map
    zones:
      pickup:  {x: 1.8, y: -1.5, yaw: 0.0}
      dropoff: {x: -1.8, y: 1.5}
      kitchen: {x: 2.0, y: 2.0, radius: 1.5}
"""

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_xy_yaw(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


@dataclass(frozen=True)
class Circle:
    """A circular footprint centred on the zone pose."""

    x: float
    y: float
    radius: float

    def contains(self, px: float, py: float) -> bool:
        return math.hypot(px - self.x, py - self.y) <= self.radius


# A footprint is anything with a ``contains(px, py) -> bool``. Circle is the
# only one today; a Polygon (ray-cast membership over explicit vertices) is the
# planned second, chosen in _footprint() by which key the spec carries.
Footprint = Circle


@dataclass(frozen=True)
class Zone:
    """A named place: a pose to navigate to, plus an optional area footprint."""

    name: str
    pose: PoseStamped
    footprint: Footprint | None = None


def _footprint(spec: dict, x: float, y: float) -> Footprint | None:
    if "radius" in spec:
        return Circle(x, y, float(spec["radius"]))
    return None


def append_zone(
    path, name: str, x: float, y: float, yaw: float, radius: float | None = None
) -> bool:
    """Write/replace one zone in the file, creating the file if needed.

    ``radius`` (optional) gives the zone a circular footprint. Returns True if
    an existing zone was replaced.
    """
    path = Path(path).expanduser()
    data = (
        yaml.safe_load(path.read_text()) or {} if path.exists() else {"frame_id": "map"}
    )
    if not isinstance(data.get("zones"), dict):
        data["zones"] = {}
    replaced = name in data["zones"]
    entry = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)}
    if radius is not None:
        entry["radius"] = round(radius, 3)
    data["zones"][name] = entry
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    os.replace(tmp, path)
    return replaced


def load_zones(path: str) -> dict[str, Zone]:
    data = yaml.safe_load(Path(path).expanduser().read_text())
    frame_id = data.get("frame_id", "map")
    zones = {}
    for name, spec in data["zones"].items():
        missing = [k for k in ("x", "y") if k not in spec]
        if missing:
            raise ValueError(f"zone '{name}' missing required key(s) {missing}")
        x, y = float(spec["x"]), float(spec["y"])
        pose = pose_from_xy_yaw(frame_id, x, y, float(spec.get("yaw", 0.0)))
        zones[name] = Zone(name, pose, _footprint(spec, x, y))
    return zones


def containing(zones: dict[str, Zone], x: float, y: float) -> list[str]:
    """Names of the zones whose footprint contains ``(x, y)``, nearest-pose
    first — so ``containing(...)[0]`` is the best single answer to "which zone
    am I in". Zones with no footprint never match. Empty when in no zone.
    """
    hits = []
    for zone in zones.values():
        if zone.footprint is not None and zone.footprint.contains(x, y):
            p = zone.pose.pose.position
            hits.append((math.hypot(x - p.x, y - p.y), zone.name))
    return [name for _, name in sorted(hits)]

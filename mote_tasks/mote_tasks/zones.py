"""Named places (zones) loaded from a zones YAML file.

A **zone** is a named pose in a map frame the robot can navigate to — as a
fetch waypoint (`pickup`/`dropoff`) or a `goto <zone>` target. A zone may also
carry an **area footprint**, so it can answer "is (x, y) inside this zone?".
The footprint is optional metadata on the one zone concept, not a separate
kind of thing: a bare zone is just a pose; a room-like zone adds a footprint.
A footprint is either a circle (`radius`, the simple taught default) or a
`polygon` of explicit vertices, which follows the actual room outline — an
L-shaped ward or a corridor stretch that no circle can describe.

A zone also carries a **vocabulary**: what it is called and what kind of place
it is (zone/v0). This file holds both halves because they are taught together
— you drive somewhere, name it, and say what it is — but they are not the same
kind of fact. The names travel; the coordinates do not. `(2.0, 3.5)` in this
robot's map frame is a different physical point in the next robot's, so the
fleet publishes the vocabulary and never the binding (`mote_bringup.bundle`'s
`vocabulary`, served at `/v1/zones`).

Schema:

    frame_id: <str, optional, default "map">
    vocabulary_revision: <int, optional, default 0>   # bumped by save-zone
    zones:
      <name>:
        # the binding — this robot's frame, never shared
        x: <float, required unless polygon>
        y: <float, required unless polygon>
        yaw: <float, optional, default 0.0>
        radius: <float, optional>       # circular footprint centred on (x, y)
        polygon: [[x, y], ...]          # footprint outline, >= 3 vertices
        # the vocabulary — shared by every robot at this site
        kind: <str, optional, default "area">     # bundle.ZONE_KINDS
        display_name: <str, optional>             # what an operator sees
        aliases: [<str>, ...]                     # what people call it
        navigable: <bool, optional>               # false for keepout/slow
        parent: <str, optional>                   # enclosing zone on this floor
        tags: [<str>, ...]
        description: <str, optional>

x, y and the polygon vertices are metres in frame_id; yaw is radians about +z.
A polygon takes precedence over a radius when a zone carries both. x and y are
the pose `goto` navigates to; a polygon-only zone derives one from the outline
(a point guaranteed to lie inside it), so map room-detection can emit a zone
without teaching a pose.

Example:

    frame_id: map
    zones:
      pickup:  {x: 1.8, y: -1.5, yaw: 0.0, kind: pickup}
      dropoff: {x: -1.8, y: 1.5, kind: dropoff}
      kitchen: {x: 2.0, y: 2.0, radius: 1.5, kind: room,
                display_name: "The Kitchen", aliases: [galley]}
      ward:    {x: 6.0, y: 1.0, polygon: [[4, 0], [9, 0], [9, 3], [4, 3]]}
      plant:   {x: 0.0, y: 4.0, radius: 1.0, kind: keepout}
"""

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped
from mote_bringup import bundle


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


@dataclass(frozen=True)
class Polygon:
    """A footprint following an outline of explicit vertices.

    The outline is a simple (non-self-intersecting) polygon in the zones file's
    frame, closed implicitly, in either winding order. Concave outlines are
    supported — membership is a ray cast, not a convex-hull test — so an
    L-shaped room or a corridor stretch is representable.
    """

    vertices: tuple[tuple[float, float], ...]

    def _edges(self):
        return zip(self.vertices, self.vertices[1:] + self.vertices[:1])

    def contains(self, px: float, py: float) -> bool:
        if any(_on_segment(px, py, a, b) for a, b in self._edges()):
            return True  # the boundary is inside, as with Circle's radius
        inside = False
        for (ax, ay), (bx, by) in self._edges():
            # Half-open in y (lower vertex inclusive) so a ray through a vertex
            # crosses exactly once.
            if (ay > py) != (by > py):
                if px < ax + (py - ay) * (bx - ax) / (by - ay):
                    inside = not inside
        return inside

    def centroid(self) -> tuple[float, float]:
        """The area centroid, which for a concave outline may fall outside it."""
        a2 = cx = cy = 0.0
        for (ax, ay), (bx, by) in self._edges():
            cross = ax * by - bx * ay
            a2 += cross
            cx += (ax + bx) * cross
            cy += (ay + by) * cross
        if abs(a2) < 1e-12:  # degenerate (collinear) outline
            n = len(self.vertices)
            return (
                sum(v[0] for v in self.vertices) / n,
                sum(v[1] for v in self.vertices) / n,
            )
        return cx / (3.0 * a2), cy / (3.0 * a2)

    def representative_point(self) -> tuple[float, float]:
        """A point guaranteed to lie inside the outline.

        The centroid where that is inside; otherwise the midpoint of the widest
        interior span of the horizontal line through it — so a U- or L-shaped
        room gets a pose in the room rather than in the notch outside it.
        """
        cx, cy = self.centroid()
        if self.contains(cx, cy):
            return cx, cy
        crossings = sorted(
            ax + (cy - ay) * (bx - ax) / (by - ay)
            for (ax, ay), (bx, by) in self._edges()
            if (ay > cy) != (by > cy)
        )
        spans = list(zip(crossings[::2], crossings[1::2]))
        if not spans:
            return cx, cy
        lo, hi = max(spans, key=lambda s: s[1] - s[0])
        return (lo + hi) / 2.0, cy


def _on_segment(px: float, py: float, a, b, tol: float = 1e-9) -> bool:
    (ax, ay), (bx, by) = a, b
    length = math.hypot(bx - ax, by - ay)
    cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(cross) > tol * max(length, 1.0):
        return False
    return (
        min(ax, bx) - tol <= px <= max(ax, bx) + tol
        and min(ay, by) - tol <= py <= max(ay, by) + tol
    )


# A footprint is anything with a ``contains(px, py) -> bool``; which one a zone
# gets follows from the key its spec carries (``polygon`` over ``radius``).
Footprint = Circle | Polygon


@dataclass(frozen=True)
class Zone:
    """A named place: a pose to navigate to, plus an optional area footprint.

    ``pose`` and ``footprint`` are the binding — only meaningful in this
    robot's ``frame_id``. Everything else is the vocabulary, and is the same on
    every robot at the site.
    """

    name: str
    pose: PoseStamped
    footprint: Footprint | None = None
    kind: str = "area"
    navigable: bool = True
    display_name: str = ""
    aliases: tuple = ()
    parent: str | None = None
    tags: tuple = ()
    description: str = ""

    @property
    def label(self) -> str:
        """What to call it when talking to a human."""
        return self.display_name or self.name


def _polygon(name: str, raw) -> Polygon:
    if not isinstance(raw, (list, tuple)) or len(raw) < 3:
        raise ValueError(f"zone '{name}' polygon needs at least 3 [x, y] vertices")
    vertices = []
    for vertex in raw:
        if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
            raise ValueError(f"zone '{name}' polygon vertex {vertex!r} is not [x, y]")
        vertices.append((float(vertex[0]), float(vertex[1])))
    return Polygon(tuple(vertices))


def append_zone(
    path,
    name: str,
    x: float,
    y: float,
    yaw: float,
    radius: float | None = None,
    kind: str | None = None,
) -> bool:
    """Write/replace one zone in the file, creating the file if needed.

    ``radius`` (optional) gives the zone a circular footprint, replacing any
    footprint it had. Re-teaching without one keeps the existing footprint, so
    capturing a better pose for a room does not discard its outline. Returns
    True if an existing zone was replaced.

    Re-teaching is a new *coordinate*, never a new name, so the vocabulary a
    zone already carries is carried through untouched unless ``kind`` says
    otherwise — driving somewhere to capture a better pose must not silently
    drop the aliases an operator typed into the file by hand.
    """
    path = Path(path).expanduser()
    data = (
        yaml.safe_load(path.read_text()) or {} if path.exists() else {"frame_id": "map"}
    )
    if not isinstance(data.get("zones"), dict):
        data["zones"] = {}
    previous = data["zones"].get(name) or {}
    entry = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)}
    if radius is not None:
        entry["radius"] = round(radius, 3)
    else:
        entry.update({k: previous[k] for k in ("radius", "polygon") if k in previous})
    entry.update({k: previous[k] for k in bundle.VOCABULARY_KEYS if k in previous})
    if kind is not None:
        if kind not in bundle.ZONE_KINDS:
            raise ValueError(
                f"unknown kind '{kind}' (one of {', '.join(bundle.ZONE_KINDS)})"
            )
        entry["kind"] = kind
        # A constraint zone is not a destination, and the flag saying so has to
        # be in the file: a reader that inferred it from the kind would be a
        # second copy of the rule, free to disagree with this one.
        entry.pop("navigable", None)
        if kind in bundle.CONSTRAINT_KINDS:
            entry["navigable"] = False
    replaced = name in data["zones"]
    data["zones"][name] = entry
    # The vocabulary revision is what a binding elsewhere records itself as
    # built against, so it has to move whenever a name could have.
    data["vocabulary_revision"] = int(data.get("vocabulary_revision") or 0) + 1
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
        polygon = _polygon(name, spec["polygon"]) if "polygon" in spec else None
        missing = [k for k in ("x", "y") if k not in spec]
        if missing:
            if polygon is None:
                raise ValueError(f"zone '{name}' missing required key(s) {missing}")
            x, y = polygon.representative_point()
        else:
            x, y = float(spec["x"]), float(spec["y"])
        pose = pose_from_xy_yaw(frame_id, x, y, float(spec.get("yaw", 0.0)))
        footprint = polygon
        if footprint is None and "radius" in spec:
            footprint = Circle(x, y, float(spec["radius"]))
        try:
            term = bundle.zone_term(Path(path).name, name, spec)
        except bundle.BundleError as exc:
            raise ValueError(str(exc)) from exc
        zones[name] = Zone(
            name,
            pose,
            footprint,
            kind=term["kind"],
            navigable=term["navigable"],
            display_name=term["display_name"],
            aliases=tuple(term["aliases"]),
            parent=term["parent"],
            tags=tuple(term["tags"]),
            description=term["description"],
        )
    # zone/v0: a conforming platform rejects a vocabulary with a collision.
    # Loading one anyway would mean `goto kitchen` picking a winner by dict
    # order — which is the guess the spec exists to forbid, and it would be
    # made silently, once per boot, differently after an edit.
    clashes = bundle.ambiguities(
        [{"name": z.name, "aliases": list(z.aliases)} for z in zones.values()]
    )
    if clashes:
        raise ValueError(f"{Path(path).name}: {clashes[0]}")
    return zones


def resolve(zones: dict[str, Zone], query: str) -> Zone | None:
    """The zone a human's words name, or None.

    Exact name first, then aliases and display names case-insensitively and
    whitespace-normalised, so "the Kitchen" reaches ``kitchen``. Ambiguity
    cannot arise here because :func:`load_zones` refuses a vocabulary that
    contains any.
    """
    if query in zones:
        return zones[query]
    wanted = bundle.normalise_alias(query)
    for zone in zones.values():
        spellings = (zone.name, zone.display_name, *zone.aliases)
        if any(s and bundle.normalise_alias(s) == wanted for s in spellings):
            return zone
    return None


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

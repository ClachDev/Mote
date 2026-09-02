"""Named places (zones), loaded from a floor's ``zones.yaml``.

A **zone** is a named pose the robot can navigate to — as a fetch waypoint
(`pickup`/`dropoff`) or a `goto <zone>` target. A zone may also carry an **area
footprint**, so it can answer "is (x, y) inside this zone?". The footprint is
optional metadata on the one zone concept, not a separate kind of thing: a bare
zone is just a pose; a room-like zone adds a footprint. A footprint is either a
circle (`radius`, the simple taught default) or a `polygon` of explicit
vertices, which follows the actual room outline — an L-shaped ward or a corridor
stretch that no circle can describe.

**A zone is a coordinate in the floor's frame — a fact about the building.**
The kitchen does not move. A map revision is an *estimate* of the same layout
registered into that frame, so re-mapping the floor changes how well the robot
knows where it is and changes nothing about where the kitchen is; where the two
disagree it is the map that gets aligned. So the floor holds its zones, in one
`floors/<floor>/zones.yaml`, and neither a map revision nor one robot owns them.
A promoted revision carries a copy, which is how a floor's places reach a robot
that has never driven there.

Geometry reaches a floor three ways, and only the first involves a robot:
driving there and running `save-zone`, which is the one that also measures an
approach heading; `segment-map` reading room outlines off a saved map; and the
fleet dashboard's zone editor, where an operator places and drags zones on a
candidate revision.

**A zone is also a place-name**, so the naming half is one human name and a
free-text `note` — nothing else. The mission layer's resolver already knows what
a store room is; what it cannot know is that this building's store room is where
the stationery lives, which is what the note is for. A floor written before this
carries `kind`, `display_name`, `aliases`, `parent` and `tags`; they still load
(`kind: keepout` still means non-navigable, and `description` is read as the
note it was) and they are never written again.

Example:

    frame_id: map
    vocabulary_revision: 4
    zones:
      the kitchen:
        x: 2.0
        y: 3.5
        yaw: 0.0
        note: the good kettle is in the store room
      plant: {x: 1.0, y: 0.5, radius: 0.4, navigable: false}
"""

import math
from dataclasses import dataclass
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from mote_bringup import bundle
from mote_bringup.spec import zone as zone_spec


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
        # Boundary inclusive, as zone/v0 requires and as Polygon is.
        return math.hypot(px - self.x, py - self.y) <= self.radius


@dataclass(frozen=True)
class Polygon:
    """A footprint following an outline of explicit vertices.

    The outline is a simple (non-self-intersecting) polygon in the floor's
    frame, closed implicitly, in either winding order. Concave outlines are
    supported — membership is a ray cast, not a convex-hull test — so an
    L-shaped room or a corridor stretch is representable.

    The maths is :mod:`mote_bringup.spec.zone`'s, not this class's. zone/v0
    makes the boundary normatively *inside* and the ordering normatively by
    distance, which are exactly the rules two implementations disagree about
    silently — and the fleet server needs the same answers, so there is one
    implementation and this is a view of it.
    """

    vertices: tuple[tuple[float, float], ...]

    def contains(self, px: float, py: float) -> bool:
        return zone_spec.polygon_contains(self.vertices, px, py)

    def centroid(self) -> tuple[float, float]:
        return zone_spec.centroid(self.vertices)

    def representative_point(self) -> tuple[float, float]:
        return zone_spec.representative_point(self.vertices)


# A footprint is anything with a ``contains(px, py) -> bool``; which one a zone
# gets follows from the key its spec carries (``polygon`` over ``radius``).
Footprint = Circle | Polygon


@dataclass(frozen=True)
class Zone:
    """A named place: a pose to navigate to, plus an optional area footprint.

    ``pose`` and ``footprint`` are coordinates in the floor's frame. Every robot
    on the floor holds the same ones, because the floor holds them.
    """

    name: str
    pose: PoseStamped | None = None
    footprint: Footprint | None = None
    #: Whether a robot may be dispatched here. Not vocabulary — it is the
    #: planner's contract — but it travels with the names because it is not a
    #: coordinate and every robot at the site needs the same answer.
    navigable: bool = True
    #: Free text for where reality diverges from what the name implies:
    #: "stationery lives here, not in the office". The one field a prior
    #: cannot supply, and the reason there is no alias list — another name
    #: this place answers to belongs in the sentence a resolver reads.
    note: str = ""
    #: What made the zone: ``save-zone``, ``segment-map`` or ``editor``. A note
    #: about provenance and nothing more — a zone read off a map is as much a
    #: coordinate in the floor's frame as one a robot was driven to.
    source: str = ""

    @property
    def label(self) -> str:
        """What to call it when talking to a human — which is its name.

        Kept as a property because a zone is *labelled* in half a dozen places
        and the split between a machine name and a human one was exactly the
        thing place-names removed; a caller that asks for a label should not
        have to know that the answer is now the same field.
        """
        return self.name


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
    note: str | None = None,
    navigable: bool | None = None,
    *,
    site: str = "",
    floor: str = "",
) -> bool:
    """Teach one zone into a floor, writing the floor's ``zones.yaml`` back.

    ``radius`` (optional) gives the zone a circular footprint, replacing any
    footprint it had. Re-teaching without one keeps the existing footprint, so
    capturing a better pose for a room does not discard its outline. Returns
    True if an existing zone was replaced.

    Re-teaching is a new *coordinate*, never a new name: the name, note and
    ``navigable`` a zone already carries come through untouched unless ``note``
    or ``navigable`` says otherwise, because driving somewhere to capture a
    better pose must not silently drop what an operator typed in by hand.

    ``path`` is the floor directory. A floor still holding zone/v0's two
    documents is read through and replaced by the single file here, so the
    first ``save-zone`` on such a floor is also its migration — nobody has to
    run one, and nobody can forget to.
    """
    path = Path(path).expanduser()
    if path.suffix == ".yaml":
        # A caller that named the file means the floor it is in. Matched on the
        # name rather than on ``is_file``, because the file may not exist yet
        # and creating a *directory* called zones.yaml is the one outcome
        # nothing recovers from.
        path = path.parent
    try:
        floor_zones = bundle.read_floor(path, site, floor)
    except bundle.BundleError:
        floor_zones = {
            "site": site,
            "floor": floor,
            "frame_id": "map",
            "revision": 0,
            "zones": {},
        }
    previous = floor_zones["zones"].get(name) or {}
    entry = dict(
        bundle.zone_term("save-zone", name, previous),
        x=round(x, 3),
        y=round(y, 3),
        yaw=round(yaw, 3),
        source=zone_spec.SAVE_ZONE,
    )
    if radius is not None:
        entry["radius"] = round(radius, 3)
    elif "polygon" in previous:
        entry["polygon"] = previous["polygon"]
    elif "radius" in previous:
        entry["radius"] = previous["radius"]
    if note is not None:
        entry["note"] = note
    if navigable is not None:
        entry["navigable"] = bool(navigable)
    replaced = name in floor_zones["zones"]
    floor_zones["zones"][name] = entry
    # What a reader compares to tell which of two copies of a floor's zones is
    # the later one, so it has to move whenever anything here could have.
    floor_zones["revision"] = int(floor_zones.get("revision") or 0) + 1
    bundle.write_floor(path, floor_zones, site=site, floor=floor)
    return replaced


def load_zones(path) -> dict[str, Zone]:
    """Every zone this floor names.

    ``path`` is either a floor directory holding ``zones.yaml`` or that file
    named outright, which is what a sim world's committed zones and the packaged
    fallback are.
    """
    try:
        floor = bundle.read_floor(path)
    except bundle.BundleError as exc:
        raise ValueError(str(exc)) from exc
    frame_id = floor["frame_id"]
    zones = {}
    for name, spec in floor["zones"].items():
        polygon = (
            Polygon(tuple(tuple(v) for v in spec["polygon"]))
            if "polygon" in spec
            else None
        )
        footprint = polygon
        if "x" in spec:
            x, y = spec["x"], spec["y"]
            pose = pose_from_xy_yaw(frame_id, x, y, float(spec.get("yaw", 0.0)))
            if footprint is None and "radius" in spec:
                footprint = Circle(x, y, float(spec["radius"]))
        else:
            # A polygon with no pose of its own — what ``segment-map`` writes,
            # since it read a room off a map rather than driving to it. The
            # point is guaranteed to lie inside the outline, which a centroid
            # is not for a concave room.
            x, y = polygon.representative_point()
            pose = pose_from_xy_yaw(frame_id, x, y, 0.0)
        zones[name] = Zone(
            name,
            pose,
            footprint,
            navigable=spec["navigable"],
            note=spec["note"],
            source=spec.get("source", ""),
        )
    # zone/v0: a conforming platform rejects a vocabulary with a collision.
    # Loading one anyway would mean `goto kitchen` picking a winner by dict
    # order — which is the guess the spec exists to forbid, and it would be
    # made silently, once per boot, differently after an edit.
    clashes = bundle.ambiguities([{"name": z.name} for z in zones.values()])
    if clashes:
        raise ValueError(f"{Path(path).name}: {clashes[0]}")
    return zones


class ZoneUnresolved(ValueError):
    """A name this robot cannot act on, with zone/v0's reason for it.

    The reason is the point: a name no zone on the floor answers to is a mistake
    in the request, while a place marked ``navigable: false`` exists and was
    drawn on purpose. mission/v0 carries it out as ``failure.class:
    "unresolved_zone"`` with the reason in ``detail``.
    """

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


def resolve(zones: dict[str, Zone], query: str) -> Zone | None:
    """The zone a human's words name, or None.

    Exact name first, then the name case-insensitively and whitespace-
    normalised, so "The  Kitchen" reaches ``the kitchen``. That is the whole of
    the matching: a zone has one name, and the other spellings a place answers
    to are a job for the mission layer's resolver reading the ``note``, not for
    a list of aliases an operator has to keep in step by hand. Ambiguity cannot
    arise here because :func:`load_zones` refuses a floor that contains any.
    """
    if query in zones:
        return zones[query]
    wanted = bundle.normalise_name(query)
    for zone in zones.values():
        if bundle.normalise_name(zone.name) == wanted:
            return zone
    return None


def resolve_reason(zones: dict[str, Zone], query) -> tuple[Zone | None, str | None]:
    """``(zone, reason)`` — the zone, and why it cannot be driven to.

    Two of zone/v0's reasons are answerable here:

    * ``unknown_name`` — no zone on this floor answers to the query.
    * ``not_navigable`` — a constraint zone used as a destination. It exists
      and an operator drew it on purpose, so saying "unknown" would send them
      hunting for a spelling mistake that is not there.

    ``wrong_floor`` and ``stale_revision`` are the two this robot cannot yet
    answer: it holds one floor at a time, and the map bundle does not declare
    frame continuity — which zone/v0 says is out of its own scope too.
    ``ambiguous`` cannot arise, because :func:`load_zones` refuses a floor that
    contains one. ``unbound`` cannot arise either, and Mote never reports it: a
    zone is a coordinate in the floor's frame, so a name with no coordinate is
    not a zone on this floor — it is a name nobody has placed.
    """
    zone = resolve(zones, query) if isinstance(query, str) else None
    if zone is None:
        return None, zone_spec.UNKNOWN_NAME
    if not zone.navigable:
        return zone, zone_spec.NOT_NAVIGABLE
    return zone, None


def destination(zones: dict[str, Zone], query, where: str = "zone") -> Zone:
    """The zone to navigate to, or raise :class:`ZoneUnresolved`."""
    zone, reason = resolve_reason(zones, query)
    if reason == zone_spec.NOT_NAVIGABLE:
        raise ZoneUnresolved(
            reason,
            f"{where} {zone.name!r} is not a destination — it is marked "
            "navigable: false",
        )
    if reason is not None:
        known = sorted(name for name, z in zones.items() if z.navigable)
        raise ZoneUnresolved(
            reason,
            f"{where} {query!r} is not a place here; "
            f"navigable zones are {', '.join(known)}",
        )
    return zone


def containing(zones: dict[str, Zone], x: float, y: float) -> list[str]:
    """Names of the zones whose footprint contains ``(x, y)``, nearest-pose
    first — so ``containing(...)[0]`` is the best single answer to "which zone
    am I in". Zones with no footprint never match. Empty when in no zone.
    """
    hits = []
    for zone in zones.values():
        if zone.footprint is None:
            continue
        if zone.footprint.contains(x, y):
            p = zone.pose.pose.position
            hits.append((math.hypot(x - p.x, y - p.y), zone.name))
    return [name for _, name in sorted(hits)]

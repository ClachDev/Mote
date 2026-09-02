"""Named places (zones), loaded from a floor's zone/v0 documents.

A **zone** is a named pose in a map frame the robot can navigate to — as a
fetch waypoint (`pickup`/`dropoff`) or a `goto <zone>` target. A zone may also
carry an **area footprint**, so it can answer "is (x, y) inside this zone?".
The footprint is optional metadata on the one zone concept, not a separate kind
of thing: a bare zone is just a pose; a room-like zone adds a footprint. A
footprint is either a circle (`radius`, the simple taught default) or a
`polygon` of explicit vertices, which follows the actual room outline — an
L-shaped ward or a corridor stretch that no circle can describe.

**A zone's two halves live in two files** (zone/v0, `mote_bringup.spec.zone`):

    floors/<floor>/vocabulary.yaml   what the places are CALLED. No
                                     coordinates, so it is safe to share with
                                     every robot at the site.
    floors/<floor>/binding.yaml      where geometry says they are, in this
                                     floor's map frame. Travels only inside
                                     the revision that frame belongs to.

They are not the same kind of fact. `(2.0, 3.5)` in this map frame is a
different physical point in the next robot's, so the fleet publishes the
vocabulary and never the binding on its own. A legacy combined `zones.yaml` is
still read (`bundle.read_floor` migrates it) and is replaced by the pair the
first time anything writes.

Geometry reaches a floor three ways, and only the first involves a robot:
driving there and running `save-zone`, which is the one that also measures an
approach heading; `segment-map` reading room outlines off a saved map; and the
fleet dashboard's zone editor, where an operator places and drags zones on a
candidate revision. A promoted revision then carries the result to every robot
at the site, so a robot can hold geometry for a floor it has never driven.

What the split buys a reader is :data:`unbound`. :func:`load_floor` returns
every name the floor carries, bound or not, so a name with no geometry in the
binding this robot holds is answerable as "I know that place, nothing has said
where it is" rather than as an unknown name — which sent an operator hunting
for a typo that was not there. :func:`load_zones` is the same minus the unbound
ones, for every caller that only ever wanted a pose.

**A zone is a place-name**, so the vocabulary is one human name and a free-text
`note` — nothing else. The mission layer's resolver already knows what a store
room is; what it cannot know is that this building's store room is where the
stationery lives, which is what the note is for. A floor written before this
carries `kind`, `display_name`, `aliases`, `parent` and `tags`; they still load
(`kind: keepout` still means non-navigable, and `description` is read as the
note it was) and they are never written again.

Example (`binding.yaml` alongside):

    schema: 1
    site: acme_hq
    floor: ground
    revision: 4
    zones:
      - {name: the kitchen, note: 'the good kettle is in the store room'}
      - {name: plant, navigable: false}
"""

import math
from dataclasses import dataclass
from datetime import datetime, timezone
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

    The outline is a simple (non-self-intersecting) polygon in the binding's
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

    ``pose`` and ``footprint`` are the binding — only meaningful in this
    robot's ``frame_id``. Everything else is the vocabulary, and is the same on
    every robot at the site.
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
    #: Whether the binding this robot holds carries geometry for it. A name in
    #: the vocabulary with no binding is a real place with no pose here — which
    #: is the whole reason zone/v0 splits the two, and the difference between
    #: "you typed it wrong" and "nothing has said where that is".
    bound: bool = True
    #: A zone this robot holds a binding for that the site's vocabulary does
    #: not name. Usable here; never advertised as a shared zone.
    local: bool = False

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
    platform_id: str | None = None,
) -> bool:
    """Teach one zone into a floor, writing back the zone/v0 pair.

    ``radius`` (optional) gives the zone a circular footprint, replacing any
    footprint it had. Re-teaching without one keeps the existing footprint, so
    capturing a better pose for a room does not discard its outline. Returns
    True if an existing zone was replaced.

    Re-teaching is a new *coordinate*, never a new name, so the vocabulary a
    zone already carries is carried through untouched unless ``note`` or
    ``navigable`` says otherwise — driving somewhere to capture a better pose
    must not silently drop what an operator typed in by hand. Under the split
    that is no longer a rule this function has to remember: the two documents
    are written separately, and a coordinate cannot reach the one that holds
    the names.

    ``path`` is the floor directory. A legacy combined file is migrated on the
    way through, so the first ``save-zone`` on an old floor is also what splits
    it — nobody has to run a migration, and nobody can forget to.
    """
    path = Path(path).expanduser()
    if path.suffix == ".yaml":
        # A caller that named the old combined file means the floor it is in.
        # Matched on the name rather than on ``is_file``, because the file may
        # not exist yet and creating a *directory* called zones.yaml is the one
        # outcome nothing recovers from.
        path = path.parent
    try:
        floor_zones = bundle.read_floor(path, site, floor)
    except bundle.BundleError:
        floor_zones = {
            "site": site,
            "floor": floor,
            "platform_id": platform_id or "",
            "frame_id": "map",
            "revision": 0,
            "map_revision": "",
            "zones": {},
        }
    previous = floor_zones["zones"].get(name) or {}
    entry = dict(
        bundle.zone_term("save-zone", name, previous),
        x=round(x, 3),
        y=round(y, 3),
        yaw=round(yaw, 3),
        bound=True,
        anchor=zone_spec.anchor(zone_spec.TAUGHT, at=_stamp()),
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
    # The vocabulary revision is what a binding records itself as built
    # against, so it has to move whenever a name could have.
    floor_zones["revision"] = int(floor_zones.get("revision") or 0) + 1
    bundle.write_floor(
        path,
        floor_zones,
        site=site,
        floor=floor,
        platform_id=platform_id,
    )
    return replaced


def _stamp() -> str:
    stamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
    return stamp.replace("+00:00", "Z")


def load_floor(path) -> dict[str, Zone]:
    """Every zone this floor names, bound or not.

    ``path`` is either a floor directory holding the zone/v0 pair
    (``vocabulary.yaml`` + ``binding.yaml``) or a legacy combined
    ``zones.yaml``, which :func:`mote_bringup.bundle.read_floor` migrates on
    read. Both produce the same structure, so nothing downstream knows which
    layout is on disk.

    A zone with no binding comes back with ``pose=None`` and ``bound=False``
    rather than being dropped. That is the point of the split: the robot can
    then say ``unbound`` — "I know that place, nothing has told me where it
    is" — where before it could only say the name was unknown, which sent an
    operator hunting for a typo that was not there.
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
        pose = None
        footprint = polygon
        if spec.get("bound"):
            x, y = spec["x"], spec["y"]
            pose = pose_from_xy_yaw(frame_id, x, y, float(spec.get("yaw", 0.0)))
            if footprint is None and "radius" in spec:
                footprint = Circle(x, y, float(spec["radius"]))
        zones[name] = Zone(
            name,
            pose,
            footprint,
            navigable=spec["navigable"],
            note=spec["note"],
            bound=bool(spec.get("bound")),
            local=bool(spec.get("local")),
        )
    # zone/v0: a conforming platform rejects a vocabulary with a collision.
    # Loading one anyway would mean `goto kitchen` picking a winner by dict
    # order — which is the guess the spec exists to forbid, and it would be
    # made silently, once per boot, differently after an edit.
    clashes = bundle.ambiguities([{"name": z.name} for z in zones.values()])
    if clashes:
        raise ValueError(f"{Path(path).name}: {clashes[0]}")
    return zones


def load_zones(path) -> dict[str, Zone]:
    """The zones this robot can actually drive to.

    :func:`load_floor` minus the unbound ones, for every caller that only ever
    wanted a pose — ``containing``, the dashboard's basemap, ``save-zone``.
    The task layer uses ``load_floor``, because refusing a mission is where the
    difference between "unknown" and "unbound" is worth saying.
    """
    return {name: zone for name, zone in load_floor(path).items() if zone.bound}


class ZoneUnresolved(ValueError):
    """A name this robot cannot act on, with zone/v0's reason for it.

    The reason is the point. "Not found" collapses two different faults an
    operator does different things about: a name that is not in the vocabulary
    at all is a mistake in the request, while one that is there and unbound
    here is a gap in the geometry this robot holds, on a floor where its
    neighbours may know the place perfectly well. mission/v0 carries it out as
    ``failure.class: "unresolved_zone"`` with the reason in ``detail``.
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
    arise here because :func:`load_floor` refuses a vocabulary that contains
    any.
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

    Three of zone/v0's six reasons are answerable here, and the split is what
    made the second one answerable at all:

    * ``unknown_name`` — not in this floor's vocabulary. A mistake in the
      request.
    * ``unbound`` — in the vocabulary, and the binding this robot holds carries
      no geometry for it. Not a typo, so the remedies are the three that put a
      coordinate there: place it in the dashboard's zone editor on a candidate
      revision and promote that; pull the revision that already binds it (the
      robot may be running an older one); or drive there and ``save-zone``,
      which is the one that measures an approach heading.
    * ``not_navigable`` — a constraint zone used as a destination. It exists
      and an operator drew it on purpose, so saying "unknown" would send them
      hunting for a spelling mistake that is not there.

    ``wrong_floor`` and ``stale_revision`` are the two this robot cannot yet
    answer: it holds one floor at a time, and the map bundle does not declare
    frame continuity — which zone/v0 says is out of its own scope too.
    ``ambiguous`` cannot arise, because :func:`load_floor` refuses a vocabulary
    that contains one.
    """
    zone = resolve(zones, query) if isinstance(query, str) else None
    if zone is None:
        return None, zone_spec.UNKNOWN_NAME
    if not zone.navigable:
        return zone, zone_spec.NOT_NAVIGABLE
    if not zone.bound:
        return zone, zone_spec.UNBOUND
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
    if reason == zone_spec.UNBOUND:
        raise ZoneUnresolved(
            reason,
            f"{where} {zone.name!r} is a place on this floor, but the map "
            "revision this robot is running has no geometry for it — place it "
            "in the dashboard's zone editor and promote, pull the revision "
            "that binds it, or drive there and run save-zone",
        )
    if reason is not None:
        known = sorted(name for name, z in zones.items() if z.navigable and z.bound)
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
        # An unbound zone has a footprint only if a binding gave it one, so
        # `pose is None` here would mean a footprint in no frame.
        if zone.pose is None or zone.footprint is None:
            continue
        if zone.footprint.contains(x, y):
            p = zone.pose.pose.position
            hits.append((math.hypot(x - p.x, y - p.y), zone.name))
    return [name for _, name in sorted(hits)]

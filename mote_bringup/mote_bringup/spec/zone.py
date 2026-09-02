"""zone/v0 shapes: the two documents a floor's zones are *serialised* into.

**A zone is a coordinate in the floor's frame — a fact about the building.**
The kitchen does not move. A map is an *estimate* of the same layout,
registered into that frame; a new map is a better or worse estimate and
changes nothing about where the kitchen is. Where a map and the zones
disagree it is the map that gets aligned — pose-graph continuation today,
rigid alignment as the fallback. So a floor's zones are one document,
``zones.yaml``, belonging to the floor: not to a map revision, and not to one
robot.

zone/v0 itself assumes otherwise — "names are shared, coordinates are not,
maps are never shared" — because it describes a heterogeneous fleet in which
every platform carries its own SLAM frame. Mote's fleet is neither: M4
distributes one canonical revision to every robot on a floor, and the frame is
the floor's. The spec's two documents therefore live here as **views over the
single record**, built at the wire and never stored:

* a :func:`vocabulary` — site, floor, and what the places are *called*. No
  coordinates, no frame, no map reference. This is what ``GET /v1/zones``
  serves to a dispatcher that has never seen a basemap.
* a :func:`binding` — the poses and footprints, stamped with the
  ``platform_id``, ``frame_id`` and ``map_revision`` of whoever is serialising
  them, filled in *at* serialisation because none of the three is a property
  of the floor.

The vocabulary view is **built** from the fields a vocabulary may carry, never
*stripped* of the ones it may not, and that is the whole safety property:
stripping holds only until someone adds a geometry key and forgets this
function exists, and the leak would be a plausible-looking coordinate rather
than a crash.

**A zone is a place-name**: a human name bound to geometry, and the record
carries only what a prior cannot guess. The semantics come from the mission
layer's resolver, which already knows what a store room is; what it cannot know
is that *this* building's store room is where the stationery lives. So the
vocabulary is the :data:`name` and a free-text :data:`note`, and nothing else.
``kind``, ``display_name``, ``aliases``, ``parent`` and ``tags`` were a
taxonomy for a reader that did not need one — five fields to fill in, four ways
to spell one place, and a machine name beside a human one for a resolver that
reads either. They are **tolerated on read** so that no floor written before
this has to be rewritten, and they are neither written nor served.

This module is stdlib-only, like the rest of :mod:`mote_bringup.spec`. Reading
and writing these documents as YAML is :mod:`mote_bringup.bundle`'s, which
already owns the site bundle's files and already imports PyYAML; what lives
here is the shapes and the rules — which are needed identically by the robot
that resolves a name, the ``save-map`` that validates a floor, and the fleet
server that serves a vocabulary to a dispatcher.
"""

import math
import re

from mote_bringup.spec import SpecError

SCHEMA = 1
VERSION = "v0"

#: Kinds a **retired** ``kind`` field used to give a zone to say that a robot
#: may not or should not go there. The taxonomy is gone; this pair is kept
#: because it is the only record an already-written floor has that a zone is a
#: keepout, and dropping it on read would turn a barrier into a destination —
#: silently, on the first load after an upgrade. :func:`term` reads it to seed
#: ``navigable`` and writes ``navigable`` back, which is the migration.
CONSTRAINT_KINDS = frozenset(("keepout", "slow"))

#: A place-name: what an operator calls the room, and what a dispatcher types.
#: One field, so there is one answer to "what is this place called" — the
#: machine name beside a display name was two fields for one fact, and the
#: resolver reads the human one anyway. Any printable text, with no leading or
#: trailing space to make two names look identical and resolve differently.
ZONE_NAME_RE = re.compile(r"^(?!\s)[^\x00-\x1f\x7f]+(?<!\s)$")

#: A site or floor name — a directory name at both ends of the wire.
PLACE_RE = re.compile(r"^[a-z0-9]([a-z0-9_-]{0,61}[a-z0-9])?$")

#: Vocabulary keys a zone entry may carry, beside the geometry that binds it.
#: The list is what :func:`term` builds from; it is not a filter applied to
#: something larger.
#:
#: ``note`` is the whole of the record that a prior cannot supply: "stationery
#: lives here, not in the office". ``navigable`` is not vocabulary at all — it
#: is the planner's contract, a fact about whether a robot may be sent here —
#: but it travels with the names because it is not a coordinate and every robot
#: at the site needs it.
VOCABULARY_KEYS = (
    "note",
    "navigable",
)

#: Fields a zone entry may still carry from before place-names, read so that an
#: old floor loads and dropped on the way out. ``description`` is ``note``'s
#: former spelling and is read *into* it; the rest are read only for the
#: ``navigable`` default above.
LEGACY_KEYS = ("kind", "display_name", "aliases", "parent", "tags", "description")

#: What made this zone, and nothing beyond that. It is a provenance note for an
#: operator listing or filtering zones; nothing may read it as a claim about the
#: coordinate's validity, accuracy or portability, because under this model
#: there is no such claim to make. A re-map moves the map, not the kitchen.
SAVE_ZONE = "save-zone"
SEGMENT_MAP = "segment-map"
EDITOR = "editor"
SOURCES = (SAVE_ZONE, SEGMENT_MAP, EDITOR)

#: zone/v0 requires every binding entry to carry an ``anchor.method``, so the
#: wire needs an answer to a question this model does not ask. The mapping is
#: here, once, and is applied only when :func:`bound` serialises — never stored
#: and never read back. ``taught`` is the fallback because a floor that records
#: no source was, in every case Mote has, driven to and captured.
_ANCHOR_METHOD = {SAVE_ZONE: "taught", SEGMENT_MAP: "derived", EDITOR: "external"}

# -- why a name did not resolve -------------------------------------------

UNKNOWN_NAME = "unknown_name"  # no zone on this floor answers to the query
WRONG_FLOOR = "wrong_floor"  # a zone, but not on the active floor
STALE_REVISION = "stale_revision"  # bound against a revision with no continuity
NOT_NAVIGABLE = "not_navigable"  # a constraint zone used as a destination
AMBIGUOUS = "ambiguous"  # the query matched more than one zone

#: zone/v0 has a sixth, ``unbound``: a name in the vocabulary that this platform
#: holds no coordinate for. Mote cannot produce it and does not list it. A zone
#: is a coordinate in the floor's frame, so a name with no coordinate is not a
#: zone on this floor at all — it is a name nobody has placed, which is what
#: ``unknown_name`` says.
REASONS = (UNKNOWN_NAME, WRONG_FLOOR, STALE_REVISION, NOT_NAVIGABLE, AMBIGUOUS)


def _place(where: str, value) -> str:
    text = "" if value is None else str(value)
    if not PLACE_RE.match(text):
        raise SpecError(f"{where} {text!r} is not a site/floor name")
    return text


# -- what a zone is called --------------------------------------------------


def term(where: str, name, entry: dict) -> dict:
    """What one zone is called, and the note beside the name.

    Both fields are optional in the file, and a floor taught before place-names
    reads perfectly: its ``kind``/``display_name``/``aliases``/``parent``/
    ``tags`` are accepted and dropped, and its ``description`` is read as the
    ``note`` it was.
    """
    return {
        "name": str(name),
        "note": str(entry.get("note") or entry.get("description") or ""),
        "navigable": _navigable(where, name, entry),
    }


def _navigable(where: str, name, entry: dict) -> bool:
    """Whether a robot may be dispatched here — stated, or read off a legacy kind.

    A zone says nothing about this and is a destination. The exception is a
    floor written before place-names, where ``kind: keepout`` is the only place
    the fact was recorded: reading it here is what carries a barrier across the
    change rather than turning it into somewhere to drive to. The contradiction
    (a keepout that says it is navigable) is still refused, because the flag
    would otherwise mean whichever of the two the file mentioned last.
    """
    constraint = str(entry.get("kind") or "") in CONSTRAINT_KINDS
    navigable = entry.get("navigable")
    if navigable is None:
        return not constraint
    if not isinstance(navigable, bool):
        raise SpecError(f"{where}: zone {name!r} navigable must be true or false")
    if navigable and constraint:
        raise SpecError(
            f"{where}: zone {name!r} is a {entry['kind']} zone, which is not a "
            "destination; navigable cannot be true"
        )
    return navigable


def vocabulary(site: str, floor: str, terms, *, revision: int = 0) -> dict:
    """The names-only view: which places exist here and what they are called.

    Carries no coordinates, no frame and no map reference — not because a
    coordinate would be wrong, but because the caller this is for has no
    basemap to draw one on, and a number it cannot place is worse than no
    number. Built from :data:`VOCABULARY_KEYS`, never stripped of the rest.
    """
    return {
        "schema": SCHEMA,
        "site": _place("site", site),
        "floor": _place("floor", floor),
        "revision": _revision("vocabulary", revision),
        "zones": [
            {key: item[key] for key in ("name",) + VOCABULARY_KEYS} for item in terms
        ],
    }


def _revision(where: str, raw) -> int:
    if raw is None:
        return 0
    try:
        revision = int(raw)
    except (TypeError, ValueError) as exc:
        raise SpecError(f"{where} revision must be an integer") from exc
    if revision < 0:
        raise SpecError(f"{where} revision must not be negative")
    return revision


def normalise_name(text: str) -> str:
    """The form two spellings of one place have to share to count as a clash.

    A place-name is matched case-insensitively and whitespace-normalised, so
    "The Kitchen" and "the  kitchen" are the same query. Collision detection has
    to use the *same* comparison the resolver will, or a vocabulary passes
    authoring and is ambiguous in the field.
    """
    return " ".join(str(text).split()).casefold()


#: The spelling this had while a zone carried aliases as well as a name. Kept
#: as a name because the comparison did not change with them.
normalise_alias = normalise_name


def check_vocabulary(terms) -> list:
    """Everything wrong with a floor's vocabulary, as operator-readable lines.

    Returns problems rather than raising because the two callers want opposite
    things from the same rules: the robot refuses to load an ambiguous
    vocabulary (it could not honour ``goto`` unambiguously), while the fleet
    server reports one and still serves the map, which is unaffected.
    """
    terms = list(terms)
    problems = [
        f"zone {item['name']!r} is not a name anyone can type (want printable "
        "text with no leading or trailing space)"
        for item in terms
        if not ZONE_NAME_RE.match(item["name"])
    ]
    return problems + ambiguities(terms)


def ambiguities(terms) -> list:
    """The subset of :func:`check_vocabulary` that makes a name unanswerable.

    Split out because it is the only half a robot must *refuse*: a zone whose
    name is hard to type is still a zone it can be told to drive to, but two
    zones answering to one query means ``goto`` has no single answer, and
    guessing between them is the one thing zone/v0 says a resolver must not do.
    Now that a zone has one name and no aliases, the only way to make one is to
    call two places the same thing — which is worth saying plainly.
    """
    problems = []
    claimed = {}
    for item in terms:
        name = item["name"]
        key = normalise_name(name)
        owner = claimed.get(key)
        if owner is not None and owner != name:
            problems.append(
                f"zones {owner!r} and {name!r} both answer to {key!r}; "
                "a query matching both can only be ambiguous"
            )
        else:
            claimed[key] = name
    return problems


# -- where a zone is -------------------------------------------------------


#: :data:`_ANCHOR_METHOD` read the other way, for :func:`source_from_anchor`.
_ANCHOR_SOURCE = {method: name for name, method in _ANCHOR_METHOD.items()}


def source_from_anchor(anchor) -> str:
    """The ``source`` a zone/v0 ``anchor`` implies, for reading an old document.

    The inverse of the mapping :func:`bound` writes, kept beside it so the two
    cannot drift. ``fiducial``, which Mote never wrote, and anything else
    unrecognised come back as ``""``.
    """
    method = anchor.get("method") if isinstance(anchor, dict) else None
    return _ANCHOR_SOURCE.get(method, "")


def read_source(value) -> str:
    """The ``source`` a document or a browser submitted, or ``""``.

    A value outside :data:`SOURCES` is dropped rather than refused. It is a note
    about what made the zone, nothing reads it to decide anything, and costing an
    operator a whole floor over a field with no consequences would be the wrong
    price.
    """
    text = str(value or "")
    return text if text in SOURCES else ""


def bound(
    name: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    *,
    footprint: dict | None = None,
    source: str = "",
) -> dict:
    """One zone's geometry, as an entry in a :func:`binding` view.

    The name is **not** refused for its spelling. A name is a fact about a floor
    an operator already has, the map it is drawn on is perfectly good, and
    refusing to read the floor over a spelling would be the wrong price.
    :func:`check_vocabulary` reports it instead, which is where an operator can
    act on it. What is refused is a *vocabulary* that cannot be resolved at all
    — two places called the same thing — because that one has no correct
    behaviour to fall back on.

    ``anchor`` is required by zone/v0 and is filled from ``source`` here, which
    is the one place the mapping lives. It is a fact about what made the zone,
    not about what the coordinate is worth.
    """
    if footprint is not None:
        check_footprint(name, footprint)
    return {
        "name": str(name),
        "pose": {
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw": round(float(yaw), 4),
        },
        "footprint": footprint,
        "anchor": {"method": _ANCHOR_METHOD.get(source, "taught"), "by": source},
    }


def check_footprint(name, footprint: dict) -> dict:
    """A circle or a polygon, and nothing else pretending to be one."""
    if not isinstance(footprint, dict):
        raise SpecError(f"zone {name!r} footprint must be an object")
    kind = footprint.get("type")
    if kind == "circle":
        radius = footprint.get("radius")
        if not isinstance(radius, (int, float)) or radius <= 0:
            raise SpecError(f"zone {name!r} circle needs a positive radius")
    elif kind == "polygon":
        vertices = footprint.get("vertices")
        if not isinstance(vertices, list) or len(vertices) < 3:
            raise SpecError(f"zone {name!r} polygon needs at least 3 vertices")
        for vertex in vertices:
            if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
                raise SpecError(f"zone {name!r} has a vertex that is not [x, y]")
    else:
        raise SpecError(
            f"zone {name!r} footprint type {kind!r} is not circle or polygon"
        )
    return footprint


def binding(
    platform_id: str,
    site: str,
    floor: str,
    bindings,
    *,
    frame_id: str = "map",
    map_revision: str = "",
    vocabulary_revision: int = 0,
) -> dict:
    """The geometry view: where the floor's places are, in a named frame.

    ``platform_id``, ``frame_id`` and ``map_revision`` are supplied by whoever
    is serialising and are not properties of the floor — a zone is a coordinate
    in the floor's frame, and the map revision is an estimate registered into
    it rather than a frame of its own. They are here because zone/v0 requires
    them, and they say which platform answered and what it was running at the
    time.
    """
    return {
        "schema": SCHEMA,
        "platform_id": platform_id,
        "site": _place("site", site),
        "floor": _place("floor", floor),
        "frame_id": frame_id or "map",
        "map_revision": map_revision,
        "vocabulary_revision": _revision("binding vocabulary", vocabulary_revision),
        "bindings": list(bindings),
    }


# -- resolution ------------------------------------------------------------


def resolution(
    platform_id: str,
    name: str,
    *,
    queried_as: str = "",
    resolved: bool = False,
    reason: str | None = None,
    site: str | None = None,
    floor: str | None = None,
    frame_id: str | None = None,
    map_revision: str | None = None,
    pose: dict | None = None,
    navigable: bool | None = None,
    anchor_method: str | None = None,
    candidates=(),
) -> dict:
    """The answer to "what does this name mean to this platform right now".

    ``frame_id``, ``map_revision`` and ``pose`` are **required together** when
    resolved: a pose without the frame and revision it belongs to is exactly
    the portable-looking coordinate this spec exists to prevent.
    """
    if resolved:
        if reason is not None:
            raise SpecError("a resolved zone carries no reason")
        if pose is None or not frame_id or not map_revision:
            raise SpecError(
                "a resolved zone must carry its pose, frame_id and map_revision "
                "together — a coordinate without them is not portable, it is wrong"
            )
    else:
        if reason not in REASONS:
            raise SpecError(
                f"unresolved zone needs a reason (one of {', '.join(REASONS)})"
            )
        if pose is not None:
            raise SpecError("an unresolved zone must not carry a pose")
    return {
        "schema": SCHEMA,
        "platform_id": platform_id,
        "name": name,
        "queried_as": queried_as,
        "resolved": bool(resolved),
        "reason": reason,
        "site": site,
        "floor": floor,
        "frame_id": frame_id,
        "map_revision": map_revision,
        "pose": pose,
        "navigable": navigable,
        "anchor_method": anchor_method,
        "candidates": list(candidates),
    }


# -- containment, and the pose a polygon implies ---------------------------
#
# The geometry is zone/v0's normative containment semantics and it lives here
# rather than in the task layer for two reasons. The *server* needs it too — a
# zone drawn as an outline still has to say where a mission navigates to, and
# that point is derived from the outline — and one implementation of "is this
# point inside" is the only way two ends of a fleet can agree about a boundary
# case.


def edges(vertices):
    return zip(vertices, list(vertices[1:]) + [vertices[0]])


def on_segment(px: float, py: float, a, b, tol: float = 1e-9) -> bool:
    (ax, ay), (bx, by) = a, b
    length = math.hypot(bx - ax, by - ay)
    cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(cross) > tol * max(length, 1.0):
        return False
    return (
        min(ax, bx) - tol <= px <= max(ax, bx) + tol
        and min(ay, by) - tol <= py <= max(ay, by) + tol
    )


def polygon_contains(vertices, px: float, py: float) -> bool:
    """Even-odd ray cast, **boundary inclusive**.

    The boundary belonging to the zone is normative, and it is what makes
    adjacent zones overlap on their shared border rather than leaving a strip
    no zone owns. Concave outlines are the point: an L-shaped ward is not a
    disc, and forcing it to be one either spills into the next room or misses
    half its own.
    """
    if any(on_segment(px, py, a, b) for a, b in edges(vertices)):
        return True
    inside = False
    for (ax, ay), (bx, by) in edges(vertices):
        # Half-open in y (lower vertex inclusive) so a ray through a vertex
        # crosses exactly once.
        if (ay > py) != (by > py):
            if px < ax + (py - ay) * (bx - ax) / (by - ay):
                inside = not inside
    return inside


def centroid(vertices) -> tuple:
    """The area centroid, which for a concave outline may fall outside it."""
    a2 = cx = cy = 0.0
    for (ax, ay), (bx, by) in edges(vertices):
        cross = ax * by - bx * ay
        a2 += cross
        cx += (ax + bx) * cross
        cy += (ay + by) * cross
    if abs(a2) < 1e-12:  # degenerate (collinear) outline
        n = len(vertices)
        return (
            sum(v[0] for v in vertices) / n,
            sum(v[1] for v in vertices) / n,
        )
    return cx / (3.0 * a2), cy / (3.0 * a2)


def representative_point(vertices) -> tuple:
    """A point guaranteed to lie inside the outline.

    The centroid where that is inside; otherwise the midpoint of the widest
    interior span of the horizontal line through it — so a U- or L-shaped room
    gets a pose in the room rather than in the notch outside it.

    This is the pose a polygon-only zone gets — what ``segment-map`` writes,
    since it read a room off a map rather than driving to it. Derived here, by
    the one implementation of "inside", so a reader deriving its own would be a
    second one. A zone that could not say where a mission navigates to would be
    a footprint pretending to be a destination.
    """
    cx, cy = centroid(vertices)
    if polygon_contains(vertices, cx, cy):
        return cx, cy
    crossings = sorted(
        ax + (cy - ay) * (bx - ax) / (by - ay)
        for (ax, ay), (bx, by) in edges(vertices)
        if (ay > cy) != (by > cy)
    )
    spans = list(zip(crossings[::2], crossings[1::2]))
    if not spans:
        return cx, cy
    lo, hi = max(spans, key=lambda s: s[1] - s[0])
    return (lo + hi) / 2.0, cy


# -- the keys that are geometry -------------------------------------------

#: Geometry keys a zone entry may carry. Named here because the tests walk a
#: serialised :func:`vocabulary` for them — the leak this module exists to
#: prevent is a coordinate reaching a document that promises none — and because
#: a key in neither this list nor :data:`VOCABULARY_KEYS` is a key nobody has
#: decided about.
GEOMETRY_KEYS = ("x", "y", "yaw", "radius", "polygon")

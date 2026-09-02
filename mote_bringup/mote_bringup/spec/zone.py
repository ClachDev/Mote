"""zone/v0 — places are named once for a fleet and located once per robot.

**Names are shared. Coordinates are not. Maps are never shared.**

A robot's map frame has its origin wherever that robot's SLAM session happened
to start, so ``(2.0, 3.5)`` on one robot is a different physical point on the
one beside it — and no fleet-level transform fixes it, because the discrepancy
is not a constant offset but two independent estimates of a building drifting
against each other. Mote has stated that invariant since the site bundles
landed. What zone/v0 adds is the **split**, and this module is it:

* a :func:`vocabulary` — site, floor, and what the places are *called*. No
  coordinates, no frame, no map reference. Safe to broadcast to every robot at
  the site, and to a dispatcher that has never seen one.
* a :func:`binding` — one platform's poses and footprints for those names,
  stamped with the platform id, the frame and the map revision they are only
  valid against. **It must not be copied to another platform.**

The split is structural rather than a rule someone has to remember. The
vocabulary document is **built** from the fields a vocabulary may carry, never
*stripped* of the ones it may not: stripping holds only until someone adds a
geometry key and forgets this function exists, and the leak would be a
plausible-looking coordinate rather than a crash.

**A zone is a place-name**: a human name bound to geometry, and the record
carries only what a prior cannot guess. The semantics come from the mission
layer's resolver, which already knows what a store room is; what it cannot know
is that *this* building's store room is where the stationery lives. So the
vocabulary is the :data:`name` and a free-text :data:`note`, and nothing else.
``kind``, ``display_name``, ``aliases``, ``parent`` and ``tags`` were a
taxonomy for a reader that did not need one — five fields to fill in, four ways
to spell one place, and a machine name beside a human one for a resolver that
reads either. They are **tolerated on read** so that no floor taught before
this has to be re-taught, and they are neither written nor served.

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

#: How a binding's coordinate came to be, which is what tells a consumer whether
#: to trust it after the map changes. ``fiducial`` is the only one that survives
#: re-mapping without re-teaching, and the only one under which two platforms
#: can independently arrive at the same physical point without sharing a map.
#: Mote writes three of the four:
#:
#: * ``taught`` — a robot was driven there and ``save-zone`` captured its pose.
#:   A measurement, taken by that platform in that map frame.
#: * ``derived`` — read off a saved map by an algorithm, which ``by`` names:
#:   ``segment-map``'s room outlines, and the pose a polygon-only zone gets.
#: * ``external`` — resolved off the platform. What the fleet dashboard's zone
#:   editor writes for geometry an operator placed or moved on the map: no
#:   robot measured it and no algorithm read it off the map, so neither of the
#:   other two would be true. ``by`` names what did it.
#:
#: ``external`` is the closest of zone/v0's four to "a person pointed at the
#: map" rather than an exact fit — the spec glosses it as an off-platform
#: localisation system. The enum is closed, so the alternatives were stamping a
#: click as a measurement or as an algorithm's output, which are the two larger
#: lies. A successor revision should carry a method for it (mote #616).
ANCHOR_METHODS = ("taught", "derived", "fiducial", "external")

TAUGHT = "taught"
DERIVED = "derived"
FIDUCIAL = "fiducial"
EXTERNAL = "external"

#: What the zone editor puts in an ``external`` anchor's ``by``. Shared with
#: ``server/ui/zone_editor.mjs``, which stamps it, and with the server, which
#: recognises it in order to record *which* operator was at the keyboard.
EDITOR = "zone-editor"

# -- why a name did not resolve -------------------------------------------

UNKNOWN_NAME = "unknown_name"  # not in the vocabulary
UNBOUND = "unbound"  # in it; this platform's binding has no geometry for it
WRONG_FLOOR = "wrong_floor"  # bound, but not on the active floor
STALE_REVISION = "stale_revision"  # bound against a revision with no continuity
NOT_NAVIGABLE = "not_navigable"  # a constraint zone used as a destination
AMBIGUOUS = "ambiguous"  # the query matched more than one zone

REASONS = (UNKNOWN_NAME, UNBOUND, WRONG_FLOOR, STALE_REVISION, NOT_NAVIGABLE, AMBIGUOUS)

#: ``unknown_name`` and ``unbound`` are deliberately distinct, and this is the
#: pair the split exists to make representable. The first is a mistake in the
#: request: no floor names that place. The second is a name the floor does
#: carry with no geometry beside it in the binding this platform holds — which
#: may be because the promoted revision binds it for nobody, because this
#: platform is running an older revision than the one that binds it, or because
#: it has simply never been given a coordinate here. An operator does different
#: things about them, and collapsing both to "not found" hides the fleet's most
#: common real fault.
DISTINCT_REASONS = (UNKNOWN_NAME, UNBOUND)


def _place(where: str, value) -> str:
    text = "" if value is None else str(value)
    if not PLACE_RE.match(text):
        raise SpecError(f"{where} {text!r} is not a site/floor name")
    return text


# -- the vocabulary --------------------------------------------------------


def term(where: str, name, entry: dict) -> dict:
    """One zone's naming half: what the place is called, and a note about it.

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
    """The shared document: which places exist here and what they are called.

    Carries no coordinates, no frame and no map reference, because none of those
    are portable between robots — which is exactly what makes it safe to
    broadcast to every platform at the site.
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


# -- the binding -----------------------------------------------------------


def anchor(
    method: str = TAUGHT,
    *,
    at: str | None = None,
    by: str = "",
    fiducial_id: str | None = None,
    offset: dict | None = None,
    confidence: float | None = None,
) -> dict:
    """How this coordinate came to be.

    ``confidence`` is the platform's own estimate; null is a legitimate answer
    and means "not estimated", never "certain".
    """
    if method not in ANCHOR_METHODS:
        raise SpecError(f"unknown anchor method {method!r}")
    if method == "fiducial" and not fiducial_id:
        # The one anchor that survives re-mapping does so by naming a marker;
        # without the marker it is a taught pose wearing a better label.
        raise SpecError("a fiducial anchor must name its fiducial_id")
    if confidence is not None and not 0.0 <= float(confidence) <= 1.0:
        raise SpecError("anchor confidence must be between 0 and 1")
    record = {"method": method, "at": at, "by": by, "confidence": confidence}
    if fiducial_id:
        record["fiducial_id"] = fiducial_id
    if offset is not None:
        record["offset"] = offset
    return record


def read_anchor(where: str, value) -> dict:
    """One anchor as it arrives from a document or a browser, through
    :func:`anchor` so an unknown method is refused at the edge.

    A submitted anchor is the only part of a zone's provenance the platform
    does not author, so it is the one part that could claim anything: a caller
    that copied it straight through would let a browser record a click as a
    measurement. Refusing here is what stops the claim reaching a stored
    revision, where nothing afterwards can tell it from a real one.
    """
    if not isinstance(value, dict):
        raise SpecError(f"{where} anchor is not a mapping")
    return anchor(
        value.get("method") or TAUGHT,
        at=value.get("at"),
        by=str(value.get("by") or ""),
        fiducial_id=value.get("fiducial_id"),
        offset=value.get("offset"),
        confidence=value.get("confidence"),
    )


def bound(
    name: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    *,
    footprint: dict | None = None,
    anchored: dict | None = None,
) -> dict:
    """One zone's coordinate half, for a :func:`binding`.

    The name is **not** refused for its spelling. A name is a fact about a
    floor an operator already has, the map it is bound to is perfectly good,
    and refusing to read the floor over a spelling would be the wrong price.
    :func:`check_vocabulary` reports it instead, which is where an operator can
    act on it, and a binding never leaves this robot anyway. What is refused is
    a *vocabulary* that cannot be resolved at all — two places called the same
    thing — because that one has no correct behaviour to fall back on.
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
        "anchor": anchored if anchored is not None else anchor(),
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
    """The private document: where those places are in this map frame.

    ``map_revision`` is not bookkeeping. A binding is valid only against a
    revision that declares frame continuity with the one it was bound in, and
    a platform whose active revision is not continuous with its binding must
    resolve every affected zone as ``stale_revision`` rather than return the old
    coordinate. Zones, map and pose-graph travel together or not at all.
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
# binding this module writes must carry a pose, and for a zone drawn as an
# outline that pose has to be derived from the outline — and one
# implementation of "is this point inside" is the only way two ends of a fleet
# can agree about a boundary case.


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

    This is what a polygon-only zone's **binding pose** is, computed once when
    the binding is written rather than by every reader. zone/v0 requires a
    binding to carry a pose, and it is right to: a binding is where a mission
    navigates to, and a zone that could not say where that is would be a
    footprint pretending to be a destination.
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


# -- the migration ---------------------------------------------------------

#: Geometry keys a legacy combined ``zones.yaml`` entry may carry. Named here
#: because :func:`split` has to know which half of an entry is which, and
#: because a key that is in neither list is a key nobody has decided about.
GEOMETRY_KEYS = ("x", "y", "yaw", "radius", "polygon")


def split(
    zones: dict, *, site: str, floor: str, platform_id: str, map_revision: str = ""
):
    """``(vocabulary, binding)`` from a combined ``zones.yaml``'s parsed form.

    This is the migration, and it is a *split* rather than two filters over the
    same dict: each document is built from its own key list, so a geometry key
    added later cannot leak into the vocabulary by being forgotten.

    An entry that carries its own ``anchor`` keeps it. That matters for the one
    combined file nobody hand-wrote: the zone editor packs its result as a
    ``zones.yaml`` and it comes back through here, so dropping the anchor would
    re-stamp every coordinate an operator placed as one a robot drove to.
    Where an entry says nothing, the binding is anchored ``taught`` with no
    timestamp — the honest record for a file written before there was a field
    to say otherwise, which did not say when or by whom either.
    """
    terms, bindings = [], []
    for name, entry in zones["zones"].items():
        terms.append(term("zones.yaml", name, entry))
        carried = entry.get("anchor")
        carried = read_anchor(f"zone {name!r}", carried) if carried else None
        footprint = None
        if entry.get("polygon"):
            footprint = {"type": "polygon", "vertices": entry["polygon"]}
        elif entry.get("radius") is not None:
            footprint = {"type": "circle", "radius": entry["radius"]}
        if "x" in entry and "y" in entry:
            bindings.append(
                bound(
                    name,
                    entry["x"],
                    entry["y"],
                    entry.get("yaw", 0.0),
                    footprint=footprint,
                    anchored=carried or anchor(TAUGHT),
                )
            )
            continue
        if footprint is None or footprint["type"] != "polygon":
            raise SpecError(f"zone {name!r} has neither a pose nor an outline")
        # A polygon-only zone — what ``segment-map`` emits, since it is reading
        # rooms off a map rather than driving to them. The pose is derived from
        # the outline *here*, once, because zone/v0 requires a binding to carry
        # one and because a reader deriving its own would be a second
        # implementation of "inside".
        px, py = representative_point(footprint["vertices"])
        bindings.append(
            bound(
                name,
                px,
                py,
                footprint=footprint,
                anchored=carried or anchor(DERIVED, by="polygon"),
            )
        )
    revision = zones.get("revision", 0)
    return (
        vocabulary(site, floor, terms, revision=revision),
        binding(
            platform_id,
            site,
            floor,
            bindings,
            frame_id=zones.get("frame_id") or "map",
            map_revision=map_revision,
            vocabulary_revision=revision,
        ),
    )


def merge(vocabulary_doc: dict, binding_doc: dict | None) -> dict:
    """One combined view, in the shape ``zones.yaml`` parses to.

    What a *reader* wants — the task layer resolving a name, the dashboard
    drawing a floor — is both halves at once, and rebuilding that join in three
    places would be three chances to get the unbound case wrong. The join is
    outer on the vocabulary: a name the binding has no geometry for is present
    with none, which is what makes ``unbound`` answerable rather than
    indistinguishable from ``unknown_name``.
    """
    bindings = {
        item["name"]: item for item in ((binding_doc or {}).get("bindings") or ())
    }
    zones = {}
    for item in vocabulary_doc.get("zones") or ():
        zones[item["name"]] = dict(item, **_geometry(bindings.pop(item["name"], None)))
    for name, item in bindings.items():
        # A binding for a name the vocabulary does not have is a *local
        # extension*: this platform holds a binding for a place nobody has
        # named for the site. It stays usable here and is never advertised as a shared zone.
        zones[name] = dict(term("binding", name, {}), **_geometry(item), local=True)
    return {
        "site": vocabulary_doc.get("site") or "",
        "floor": vocabulary_doc.get("floor") or "",
        "platform_id": (binding_doc or {}).get("platform_id") or "",
        "frame_id": (binding_doc or {}).get("frame_id") or "map",
        "revision": vocabulary_doc.get("revision", 0),
        "map_revision": (binding_doc or {}).get("map_revision") or "",
        "zones": zones,
    }


def _geometry(item: dict | None) -> dict:
    if item is None:
        return {"bound": False}
    geometry = {"bound": True, "anchor": item.get("anchor") or anchor()}
    pose = item.get("pose")
    if pose:
        geometry.update(x=pose["x"], y=pose["y"], yaw=pose.get("yaw", 0.0))
    footprint = item.get("footprint")
    if footprint:
        if footprint["type"] == "circle":
            geometry["radius"] = footprint["radius"]
        else:
            geometry["polygon"] = footprint["vertices"]
    return geometry

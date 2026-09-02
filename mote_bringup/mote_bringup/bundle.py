"""Site bundles as bytes: read one, validate one, pack one, unpack one.

:mod:`mote_bringup.sites` owns the bundle *layout* — which directories exist,
what a revision is, which symlink is live. This module owns the bundle
*content*: what is inside a map revision, whether it is any good, and how it
travels over a wire. The split matters because the two ends that need the
answer are not the same machine. A robot writes a revision with
``sites.save_map``; the fleet server, which has neither ROS nor a checkout,
re-validates it before making it canonical (fleet.md Q4) because an upload can
truncate where a local save could not. One module, imported by both, rather
than a second implementation on the server that agrees only by convention.

That is also why this file is **ROS-free**, the same discipline as
:mod:`mote_fleet.protocol` and ``mote_perception/depth_wire.py``: the fleet
server has no ROS, no framework and no checkout, and this file is the one thing
it imports from the robot's side of the tree.

What the design asks of this file is precisely that — **ROS-free and
torch-free**, so a container with neither can run it. It is not dependency-free,
and twice in review that distinction had to be relearned: a hand-rolled YAML
parser and a hand-rolled PNG decoder were both written to satisfy a stricter
rule ("stdlib only") that the design never set, and both were wrong in ways the
libraries are not.

The parser mangled a zone called ``Café`` into ``Caf\xe9`` with no error at all,
raised on an apostrophe, and could not read the polygon shape
``safe_dump(default_flow_style=None)`` emits — i.e. the output of
``segment-map`` and ``save-zone``, so a segmented floor could not be published.
The decoder shipped an unhandled exception that dropped the upload connection
outright and an unbounded inflate. Both now use the library that the rest of
the system already uses to write and read these files: PyYAML and Pillow, which
the fleet image installs beside paho. Parsing input this module exists to
distrust is the last place to save a few megabytes.

What a validated revision has to contain::

    map.yaml        resolution/origin/image — the map frame itself
    <image>         the occupancy PNG map.yaml names (map.png in practice)
    meta.yaml       provenance: when it was saved, from which bag, cleaning stats
    map.posegraph   slam_toolbox graph, and
    map.data        ...its data — together, what lets mapping continue in this
                    frame later. A revision without them is servable but is a
                    dead end, which is an error for a published revision and a
                    warning for one being read.

A floor's zones are one document, ``zones.yaml``, and it belongs to the
**floor**. A zone is a coordinate in the floor's frame — a fact about the
building — and a map revision is an *estimate* of that layout registered into
the frame, so re-mapping moves the map and not the kitchen. A revision carries
a copy of the floor's zones so that they reach a robot which has never driven
there, and installing one replaces the floor's copy; it never owns them.
"""

import gzip
import hashlib
import io
import os
import tarfile
from dataclasses import dataclass, field
from pathlib import Path

import yaml
from PIL import Image, UnidentifiedImageError

from mote_bringup.spec import SpecError
from mote_bringup.spec import zone

# A map an order of magnitude past the largest floor anyone has mapped here
# (the 1158x761 hospital) is a decompression bomb, not a building. Pillow
# enforces this itself, so the bound and the check are the library's.
Image.MAX_IMAGE_PIXELS = 64 * 1024 * 1024

SCHEMA = 1

MAP_YAML = "map.yaml"
META_YAML = "meta.yaml"

#: A floor's zones: what the places are called, and where they are.
ZONES_YAML = "zones.yaml"

#: What a floor outside a site bundle calls itself. zone/v0 requires a
#: vocabulary to name its site and floor, and it is right to — a document with
#: no place is a document nobody can file. Mote still has floors with no site:
#: the legacy ``~/.mote/zones.yaml`` a robot used before site bundles existed,
#: and a bench directory. Naming them ``local/default`` says so, and is a
#: better answer than an empty string that would only be discovered on upload.
LOCAL_SITE = "local"
LOCAL_FLOOR = "default"
POSEGRAPH = "map.posegraph"
POSEGRAPH_DATA = "map.data"

# The zone vocabulary's rules are zone/v0's, in ``mote_bringup.spec.zone``.
# They are re-exported here rather than restated because this module is what
# ``save-map`` and the fleet server both import, and a second copy of "what a
# place-name may be" is a second copy free to disagree with the one the robot
# resolves against.
CONSTRAINT_KINDS = zone.CONSTRAINT_KINDS
ZONE_NAME_RE = zone.ZONE_NAME_RE
VOCABULARY_KEYS = zone.VOCABULARY_KEYS
normalise_name = zone.normalise_name
check_vocabulary = zone.check_vocabulary
ambiguities = zone.ambiguities
read_source = zone.read_source

#: Present and non-empty in every revision, whoever wrote it.
REQUIRED = (MAP_YAML, META_YAML)

#: Present in a revision that mapping can be *continued* from (sites.py:396-406).
CONTINUABLE = (POSEGRAPH, POSEGRAPH_DATA)

#: Everything a revision is allowed to carry over the wire. An upload naming
#: anything else is refused rather than quietly dropped: a bundle with a
#: surprise in it is a bundle we do not understand.
ALLOWED = frozenset(
    (
        MAP_YAML,
        META_YAML,
        ZONES_YAML,
        POSEGRAPH,
        POSEGRAPH_DATA,
        "map.png",
        "map_raw.png",
        "map_raw.yaml",
        "diagnostics.png",
    )
)

#: Bounds on an unpacked bundle. The upload route is reachable by anything on
#: the tailnet (M7 adds the credential), so "a tar bomb cannot fill the fleet
#: box's disk" is a property this module has to hold on its own.
MAX_MEMBERS = 32
MAX_UNPACKED = 256 * 1024 * 1024

#: Trinary occupancy values as ``map_saver`` writes them: 0 occupied, 205
#: unknown, 254 free. Read with slack either side, because the cleaning pass
#: (sites._promote_cleaned) goes through cv2 and need not land exactly on them.
OCCUPIED_MAX = 25
FREE_MIN = 230

#: A map with nothing free in it is not a map anybody can navigate, and one
#: with nothing occupied never saw a wall. Both are what a mapping run that
#: never got going looks like from here (fleet.md Q4, "occupancy isn't
#: degenerate"). The free floor is deliberately tiny — a legitimate first
#: revision of a big floor can be almost entirely unknown.
MIN_FREE_FRACTION = 0.001

#: No floor is 10 km across. A resolution/size pair that claims otherwise is a
#: units mistake, and it would blow up every consumer's view transform.
MAX_EXTENT_M = 10_000.0


class BundleError(ValueError):
    """A bundle, or a file in one, that does not meet the contract."""


def zone_term(where: str, name, entry: dict) -> dict:
    """:func:`mote_bringup.spec.zone.term`, raising this module's error.

    The rules are the specification's and are not restated. What is restated
    is *which exception a bundle reader raises*, because every caller here —
    the upload route's 422, ``save-map``'s refusal, the zone editor — catches
    ``BundleError`` and a spec exception escaping as a 500 would turn "your
    file says a keepout is navigable" into "the server broke".

    Every field a zone once carried besides its name is still *accepted* here —
    a floor taught before place-names must load without being re-taught — and
    only ``name``, ``note`` and ``navigable`` come back out.
    """
    try:
        return zone.term(where, name, entry)
    except SpecError as exc:
        raise BundleError(str(exc)) from exc


# --------------------------------------------------------------------------
# the YAML the bundle is written in
# --------------------------------------------------------------------------


class _Loader(yaml.SafeLoader):
    """``SafeLoader``, except that a timestamp stays the text it was written as.

    A bundle's values travel: ``meta.yaml``'s provenance is served as JSON by
    the fleet server and rendered by the dashboard, and ``datetime`` is the one
    thing ``safe_load`` returns that ``json.dumps`` refuses — so an unquoted
    ``saved: 2026-07-05T11:16:46`` would cost the floor route its whole
    response rather than one field. Quoting is not something a bundle can be
    relied on to do: a revision may be hand-edited, or seeded by rsync from
    before the registry existed, and only the local writer (``sites.save_map``)
    goes through ``yaml.safe_dump``.

    The value is passed through verbatim rather than parsed and reformatted,
    because a provenance stamp is a record of what was written, and inventing
    a normal form for it would make the served string differ from the file.
    """


_Loader.add_constructor("tag:yaml.org,2002:timestamp", lambda loader, node: node.value)


def load_yaml(text: str):
    """Parse bundle YAML, raising :class:`BundleError` rather than a YAMLError.

    PyYAML, deliberately — see this module's docstring. What is added here is
    the error type, so that every "this bundle is not readable" failure reaches
    a caller as one exception class, and :class:`_Loader`'s guarantee that
    every scalar in a bundle is JSON-serialisable.
    """
    try:
        return yaml.load(text, Loader=_Loader)
    except yaml.YAMLError as exc:
        raise BundleError(f"not valid YAML: {_one_line(exc)}") from exc


def _one_line(exc) -> str:
    """PyYAML's errors are multi-line and end up in HTTP bodies and logs."""
    return " ".join(str(exc).split())


def load_yaml_file(path) -> dict:
    """:func:`load_yaml` over a file, as a mapping. Raises BundleError."""
    path = Path(path)
    try:
        text = path.read_text()
    except OSError as exc:
        raise BundleError(f"{path.name}: {exc}") from exc
    try:
        value = load_yaml(text)
    except BundleError as exc:
        raise BundleError(f"{path.name}: {exc}") from exc
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise BundleError(
            f"{path.name}: expected a mapping, got {type(value).__name__}"
        )
    return value


# --------------------------------------------------------------------------
# the files in a revision
# --------------------------------------------------------------------------

#: The scalar keys of a ``map_saver`` ``map.yaml`` and how each is read.
MAP_KEYS = {
    "image": str,
    "mode": str,
    "resolution": float,
    "negate": int,
    "occupied_thresh": float,
    "free_thresh": float,
}


def read_map(path) -> dict:
    """A revision's ``map.yaml`` as plain JSON-able values.

    Everything a consumer needs for the world→pixel transform, checked hard
    enough that a consumer never has to: a resolution that is not a positive
    number, or an origin that is not a coordinate, is refused here rather than
    turning into a robot drawn off the edge of a floor.
    """
    raw = load_yaml_file(path)
    meta = {}
    for key, cast in MAP_KEYS.items():
        if key in raw and raw[key] is not None:
            try:
                meta[key] = cast(raw[key])
            except (TypeError, ValueError) as exc:
                raise BundleError(f"{Path(path).name}: bad {key}: {exc}") from exc
    origin = raw.get("origin")
    if isinstance(origin, (list, tuple)):
        try:
            meta["origin"] = [float(value) for value in origin]
        except (TypeError, ValueError) as exc:
            raise BundleError(f"{Path(path).name}: bad origin: {exc}") from exc

    missing = [key for key in ("image", "resolution", "origin") if key not in meta]
    if missing:
        raise BundleError(f"{Path(path).name} is missing {', '.join(missing)}")
    if not meta["resolution"] > 0:
        raise BundleError(f"{Path(path).name}: resolution must be positive")
    if len(meta["origin"]) < 2:
        raise BundleError(f"{Path(path).name}: origin needs at least x and y")
    if any(value != value or abs(value) == float("inf") for value in meta["origin"]):
        raise BundleError(f"{Path(path).name}: origin is not finite")
    image = meta["image"]
    if "/" in image or "\\" in image or image in ("", ".", ".."):
        raise BundleError(f"{Path(path).name}: image must be a plain file name")
    free = meta.get("free_thresh")
    occupied = meta.get("occupied_thresh")
    if free is not None and occupied is not None and not free < occupied:
        raise BundleError(
            f"{Path(path).name}: free_thresh {free} is not below "
            f"occupied_thresh {occupied}"
        )
    return meta


def read_zones(path) -> dict:
    """One ``zones.yaml``, parsed and checked.

    :func:`read_floor` is what a *floor* reads through; this is the file, and it
    is also what a sim world's committed ``<world>.zones.yaml`` and the packaged
    fallback are.
    """
    return parse_zones(load_yaml_file(path), Path(path).name)


def parse_zones(raw: dict, where: str = "zones") -> dict:
    """:func:`read_zones` over a document already in memory.

    The zone editor submits one rather than writing a file first, and it must
    go through the same reader: a second parser for "the shape a combined
    zones file has" is the thing whose two implementations disagreed last time.
    """
    zones = raw.get("zones") or {}
    if not isinstance(zones, dict):
        raise BundleError(f"{where}: 'zones' must be a mapping")
    parsed = {}
    for name, entry in zones.items():
        if not isinstance(entry, dict):
            raise BundleError(f"{where}: zone {name!r} is not a mapping")
        record = {"name": str(name), **zone_term(where, name, entry)}
        for key in ("x", "y", "yaw", "radius"):
            if entry.get(key) is not None:
                try:
                    record[key] = float(entry[key])
                except (TypeError, ValueError) as exc:
                    raise BundleError(
                        f"{where}: zone {name!r} has a bad {key}"
                    ) from exc
        polygon = entry.get("polygon")
        if polygon is not None:
            record["polygon"] = _polygon(where, name, polygon)
        # What made the zone, carried rather than re-invented downstream. It is
        # a note and nothing decides anything from it, so an unrecognised value
        # is dropped rather than costing the floor its map.
        source = zone.read_source(entry.get("source"))
        if source:
            record["source"] = source
        if "x" not in record or "y" not in record:
            # A polygon-only zone is legal — the loader derives a pose inside
            # the outline (mote_tasks.zones) — but a zone with neither is not
            # a place at all.
            if "polygon" not in record:
                raise BundleError(f"{where}: zone {name!r} has no position")
        parsed[str(name)] = record
    return {
        "frame_id": raw.get("frame_id") or "map",
        "revision": _revision(where, raw.get("revision")),
        "zones": parsed,
    }


def _revision(where: str, raw) -> int:
    if raw is None:
        return 0
    try:
        revision = int(raw)
    except (TypeError, ValueError) as exc:
        raise BundleError(f"{where}: revision must be an integer") from exc
    if revision < 0:
        raise BundleError(f"{where}: revision must not be negative")
    return revision


def _polygon(where: str, name, polygon) -> list:
    if not isinstance(polygon, list) or len(polygon) < 3:
        raise BundleError(f"{where}: zone {name!r} polygon needs at least 3 vertices")
    vertices = []
    for vertex in polygon:
        if not isinstance(vertex, (list, tuple)) or len(vertex) != 2:
            raise BundleError(f"{where}: zone {name!r} has a vertex that is not [x, y]")
        try:
            vertices.append([float(vertex[0]), float(vertex[1])])
        except (TypeError, ValueError) as exc:
            raise BundleError(
                f"{where}: zone {name!r} has a non-numeric vertex"
            ) from exc
    return vertices


def vocabulary(zones: dict, site: str, floor: str) -> dict:
    """The names-only view of :func:`read_floor`'s output, as a zone/v0 document.

    This is what ``GET /v1/zones`` serves: which places exist here and what they
    are called, for a dispatcher with no basemap to draw a coordinate on. It is
    **built** from :data:`VOCABULARY_KEYS` rather than stripped of the geometry
    keys, and that is the one safety property it has — stripping holds only
    until someone adds a key and never reads this function, and the leak would
    be a plausible-looking coordinate rather than a crash.
    """
    terms = [
        {key: item[key] for key in ("name",) + VOCABULARY_KEYS}
        for item in zones["zones"].values()
    ]
    document = zone.vocabulary(site, floor, terms, revision=zones.get("revision", 0))
    document["problems"] = check_vocabulary(terms)
    return document


def binding(
    zones: dict,
    site: str,
    floor: str,
    platform_id: str,
    *,
    map_revision: str = "",
) -> dict:
    """The geometry view of the same output, as a zone/v0 document.

    ``platform_id`` and ``map_revision`` say who answered and what they were
    running when they did; neither is stored, because neither is a property of
    the floor. A zone is a coordinate in the floor's frame, and a map revision
    is an estimate registered into that frame rather than a frame of its own.
    """
    bindings = []
    for item in zones["zones"].values():
        footprint = None
        if item.get("polygon"):
            footprint = {"type": "polygon", "vertices": item["polygon"]}
        elif item.get("radius") is not None:
            footprint = {"type": "circle", "radius": item["radius"]}
        if "x" in item and "y" in item:
            x, y, yaw = item["x"], item["y"], item.get("yaw", 0.0)
        elif footprint is not None and footprint["type"] == "polygon":
            # zone/v0 requires a binding entry to carry a pose: it is where a
            # mission navigates to, and an outline alone cannot say. Derived
            # here rather than stored, so nothing has to keep it in step with
            # the outline it comes from.
            x, y = zone.representative_point(footprint["vertices"])
            yaw = 0.0
        else:
            raise BundleError(
                f"zone {item['name']!r} has neither a pose nor an outline"
            )
        bindings.append(
            zone.bound(
                item["name"],
                x,
                y,
                yaw,
                footprint=footprint,
                source=item.get("source", ""),
            )
        )
    return zone.binding(
        platform_id,
        site,
        floor,
        bindings,
        frame_id=zones.get("frame_id") or "map",
        map_revision=map_revision,
        vocabulary_revision=zones.get("revision", 0),
    )


def write_floor(directory, merged: dict):
    """Write a floor's zones — one file, replaced atomically."""
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    document = {
        "frame_id": merged.get("frame_id") or "map",
        "revision": _revision(ZONES_YAML, merged.get("revision")),
        "zones": {name: _entry(item) for name, item in sorted(merged["zones"].items())},
    }
    _atomic(directory / ZONES_YAML, document)


def _entry(item: dict) -> dict:
    """One zone as it is written: the key carries the name, so the value does
    not, and a field at its default is left out rather than restated."""
    entry = {}
    for key in ("x", "y", "yaw", "radius"):
        if item.get(key) is not None:
            entry[key] = item[key]
    if item.get("polygon"):
        entry["polygon"] = item["polygon"]
    if item.get("note"):
        entry["note"] = item["note"]
    if not item.get("navigable", True):
        entry["navigable"] = False
    if item.get("source"):
        entry["source"] = item["source"]
    return entry


def _atomic(path: Path, document: dict):
    document = {key: value for key, value in document.items() if key != "problems"}
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(document, sort_keys=False, default_flow_style=None))
    os.replace(tmp, path)


def read_floor(directory, site: str = "", floor: str = "") -> dict:
    """A floor's zones.

    ``directory`` is the floor directory, or a ``zones.yaml`` named outright —
    which is what a sim world's committed file and the packaged fallback are.
    ``site``/``floor`` are what the caller knows about where it is reading from
    and are stamped onto the result; the file records neither, because the
    floor's place in the layout is the directory it is in and a copy in the file
    would be free to disagree with the path after a rename.
    """
    directory = Path(directory).expanduser()
    path = directory if directory.is_file() else directory / ZONES_YAML
    if not path.is_file():
        raise BundleError(f"{directory}: no {ZONES_YAML}")
    zones = read_zones(path)
    return {
        "site": site or LOCAL_SITE,
        "floor": floor or LOCAL_FLOOR,
        "frame_id": zones["frame_id"],
        "revision": zones["revision"],
        "zones": zones["zones"],
    }


def png_size(path) -> tuple[int, int] | None:
    """Pixel dimensions of a map image, or None if it cannot be read.

    ``Image.open`` is lazy — it parses the header and stops — so this costs no
    more than reading the bytes by hand did, and it is right about formats this
    module never anticipated.
    """
    try:
        with Image.open(path) as image:
            return image.size
    except (OSError, ValueError):
        # UnidentifiedImageError and DecompressionBombError are both subclasses
        # of these; a size nobody can read is None, whatever the reason.
        return None


def occupancy(path) -> dict:
    """Fractions of free / occupied / unknown cells in an occupancy PNG.

    This is the check that catches the mapping run that never got going: a
    revision can have every file in place, a sane ``map.yaml``, and still be a
    uniform grey rectangle. Nothing else in the pipeline looks at the pixels.

    Failures are returned, never raised — :func:`validate` documents that it
    reports rather than throws, and the upload path depends on it. ``corrupt``
    separates "these bytes are not an image anyone can read", which fails a
    revision, from "this image is not one I can count", which only costs the
    check.
    """
    try:
        with Image.open(path) as image:
            grey = image.convert("L")
            counts = grey.histogram()
    except Image.DecompressionBombError as exc:
        return {"reason": f"decompression bomb: {_one_line(exc)}", "corrupt": True}
    except UnidentifiedImageError:
        return {"reason": "not an image this can read", "corrupt": True}
    except OSError as exc:
        # Pillow raises OSError for a truncated or damaged file, which is what
        # a half-finished upload looks like.
        return {"reason": f"could not decode: {_one_line(exc)}", "corrupt": True}
    except ValueError as exc:
        return {"reason": f"could not convert to greyscale: {_one_line(exc)}"}

    if len(counts) < 256:
        return {"reason": f"expected 256 grey levels, got {len(counts)}"}
    occupied = sum(counts[: OCCUPIED_MAX + 1])
    free = sum(counts[FREE_MIN:])
    total = sum(counts)
    if not total:
        return {"reason": "the image has no pixels", "corrupt": True}
    return {
        "total": total,
        "free": round(free / total, 6),
        "occupied": round(occupied / total, 6),
        "unknown": round((total - free - occupied) / total, 6),
    }


# --------------------------------------------------------------------------
# validation
# --------------------------------------------------------------------------


@dataclass
class Report:
    """What a revision is, and what is wrong with it.

    Errors and warnings are kept apart on purpose. An error is "this must not
    become a floor's canonical map"; a warning is "this is servable but you
    should know" — a revision with no posegraph navigates perfectly and simply
    cannot be mapped further, and telling an operator that is more useful than
    either rejecting it or staying quiet.
    """

    revision: str = ""
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    map: dict | None = None
    meta: dict = field(default_factory=dict)
    zones: dict | None = None
    files: dict = field(default_factory=dict)
    occupancy: dict | None = None

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        if self.ok:
            return "valid" if not self.warnings else f"valid ({self.warnings[0]})"
        return "; ".join(self.errors)

    def as_dict(self) -> dict:
        return {
            "revision": self.revision,
            "ok": self.ok,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "map": self.map,
            "meta": self.meta,
            "zones": sorted(self.zones["zones"]) if self.zones else [],
            "occupancy": self.occupancy,
            "files": self.files,
        }


def validate(revision_dir, *, require_posegraph: bool = True) -> Report:
    """Check one map revision directory. Never raises: the whole point is to
    answer *why* a bundle is unacceptable, and a caller that only wants a
    verdict reads ``report.ok``."""
    revision_dir = Path(revision_dir)
    report = Report(revision=revision_dir.name)
    if not revision_dir.is_dir():
        report.errors.append(f"{revision_dir} is not a directory")
        return report

    for entry in sorted(revision_dir.iterdir()):
        if entry.is_file():
            report.files[entry.name] = entry.stat().st_size

    for name in REQUIRED:
        if name not in report.files:
            report.errors.append(f"{name} is missing")
        elif not report.files[name]:
            report.errors.append(f"{name} is empty")

    # One message however many of them are absent. slam_toolbox writes the
    # posegraph and its data as a pair, either half missing means exactly the
    # same thing, and a line per file produced two entries with word-for-word
    # identical text — which reads as two separate problems.
    missing = [name for name in CONTINUABLE if not report.files.get(name)]
    if missing:
        message = (
            f"{' and '.join(missing)} {'is' if len(missing) == 1 else 'are'} "
            "missing — mapping cannot be continued in this frame "
            "(extend, don't remap)"
        )
        (report.errors if require_posegraph else report.warnings).append(message)

    if MAP_YAML in report.files and report.files[MAP_YAML]:
        try:
            report.map = read_map(revision_dir / MAP_YAML)
        except BundleError as exc:
            report.errors.append(str(exc))

    if report.map:
        _validate_image(revision_dir, report)

    if report.files.get(META_YAML):
        try:
            report.meta = load_yaml_file(revision_dir / META_YAML)
        except BundleError as exc:
            report.errors.append(str(exc))
        if not report.meta.get("saved"):
            report.warnings.append("meta.yaml records no save time")

    # A revision carries a copy of the floor's zones. It does not own them —
    # the floor does — but it is how a floor's places reach a robot that has
    # never driven there, so a revision without them is worth saying.
    if ZONES_YAML in report.files:
        try:
            report.zones = read_floor(revision_dir)
        except BundleError as exc:
            report.errors.append(str(exc))
        else:
            # An ambiguous vocabulary is a warning here, not an error: the map
            # is good and every coordinate in it is good. What it costs is
            # dispatch by name, so it must be *said* — but refusing to publish
            # a floor's map over two rooms called the same thing would be the
            # wrong price.
            report.warnings.extend(
                f"zones: {problem}"
                for problem in check_vocabulary(report.zones["zones"].values())
            )
    else:
        report.warnings.append(
            f"no {ZONES_YAML} — this revision names no places, so a robot "
            "installing it has nowhere to be sent by name. Place them in the "
            "dashboard's zone editor on this revision, or drive there and run "
            "save-zone"
        )

    unexpected = sorted(set(report.files) - ALLOWED)
    if unexpected:
        report.warnings.append(f"unrecognised files: {', '.join(unexpected)}")
    return report


def _validate_image(revision_dir: Path, report: Report):
    image = revision_dir / report.map["image"]
    if not image.is_file():
        report.errors.append(f"map.yaml names an image that is not here: {image.name}")
        return
    size = png_size(image)
    if size is None:
        report.errors.append(f"{image.name} is not a PNG this can measure")
        return
    width, height = size
    report.map["width"], report.map["height"] = width, height
    if not width or not height:
        report.errors.append(f"{image.name} has a zero dimension")
        return
    extent = max(width, height) * report.map["resolution"]
    if extent > MAX_EXTENT_M:
        report.errors.append(
            f"{image.name} spans {extent:.0f} m at "
            f"{report.map['resolution']} m/px — check the resolution"
        )
        return

    # The raw and the cleaned map are the same frame with different pixels
    # (sites._promote_cleaned), so a size that differs means one of them is
    # not what it claims and every zone bound on this floor is suspect.
    raw = revision_dir / "map_raw.png"
    if raw.is_file():
        raw_size = png_size(raw)
        if raw_size is not None and raw_size != size:
            report.errors.append(
                f"map_raw.png is {raw_size[0]}x{raw_size[1]} but "
                f"{image.name} is {width}x{height} — they must share a frame"
            )

    report.occupancy = occupancy(image)
    if "reason" in report.occupancy:
        note = f"could not read occupancy: {report.occupancy['reason']}"
        if report.occupancy.get("corrupt"):
            report.errors.append(f"{image.name} is not a readable PNG — {note}")
        else:
            report.warnings.append(note)
        return
    if report.occupancy["free"] < MIN_FREE_FRACTION:
        report.errors.append(
            "the map has no free space — nothing here can be navigated"
        )
    if not report.occupancy["occupied"]:
        report.errors.append("the map has no occupied cells — nothing was mapped")


# --------------------------------------------------------------------------
# the wire form
# --------------------------------------------------------------------------


def pack(revision_dir, extra: dict | None = None) -> bytes:
    """One revision as a gzipped tar of plain files, no directory prefix.

    Flat and metadata-free by design: the receiver decides where a revision
    lands (its own ``maps/<rev>/``), and a tar that cannot name a path cannot
    write outside one. ``extra`` adds files that live at floor level but must
    travel with the frame — ``zones.yaml``.

    Packing is **deterministic**: fixed member order, fixed modes, and a zeroed
    mtime in both the tar headers and the gzip one. The same revision therefore
    always packs to the same bytes, which is what lets the registry announce a
    digest it can still serve after a restart without keeping the upload's
    original bytes on disk alongside the files it unpacked from them.
    """
    revision_dir = Path(revision_dir)
    members = {
        entry.name: entry.read_bytes()
        for entry in sorted(revision_dir.iterdir())
        if entry.is_file() and entry.name in ALLOWED
    }
    for name, blob in (extra or {}).items():
        if name not in ALLOWED:
            raise BundleError(f"{name} is not part of a site bundle")
        members[name] = blob

    buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=buffer, mode="wb", mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for name, blob in sorted(members.items()):
                info = tarfile.TarInfo(name)
                info.size = len(blob)
                info.mode = 0o644
                info.mtime = 0
                info.uid = info.gid = 0
                info.uname = info.gname = ""
                archive.addfile(info, io.BytesIO(blob))
    return buffer.getvalue()


def unpack(blob: bytes, destination) -> dict:
    """Write a packed revision into ``destination``. Returns ``{name: size}``.

    Every member is checked before anything is written: plain files only, names
    from :data:`ALLOWED` only, bounded in count and size. The archive is
    arriving over an open port, so "a tar cannot make this process write
    somewhere it did not choose" is enforced here rather than assumed.
    """
    destination = Path(destination)
    try:
        archive = tarfile.open(fileobj=io.BytesIO(blob), mode="r:gz")
    except (tarfile.TarError, EOFError, OSError) as exc:
        raise BundleError(f"not a readable bundle archive: {exc}") from exc
    with archive:
        members = archive.getmembers()
        if len(members) > MAX_MEMBERS:
            raise BundleError(f"bundle has {len(members)} members (max {MAX_MEMBERS})")
        total = 0
        for member in members:
            if not member.isfile():
                raise BundleError(f"bundle member {member.name!r} is not a plain file")
            if member.name not in ALLOWED:
                raise BundleError(
                    f"bundle member {member.name!r} is not part of a site bundle"
                )
            total += member.size
            if total > MAX_UNPACKED:
                raise BundleError(f"bundle unpacks to more than {MAX_UNPACKED} bytes")
        destination.mkdir(parents=True, exist_ok=True)
        written = {}
        for member in members:
            source = archive.extractfile(member)
            if source is None:
                raise BundleError(f"bundle member {member.name!r} has no content")
            (destination / member.name).write_bytes(source.read())
            written[member.name] = member.size
    return written


def digest(blob: bytes) -> str:
    """``sha256:<hex>`` — what a puller checks a download against."""
    return "sha256:" + hashlib.sha256(blob).hexdigest()

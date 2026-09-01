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

A floor's zones are two documents (zone/v0, ``mote_bringup.spec.zone``), and
only one of them belongs in a revision. ``binding.yaml`` — poses, footprints,
anchors — travels *inside* a published revision, because those coordinates are
only meaningful in the map frame they were taught in and the two must move
together or the fleet ends up drawing a kitchen through a wall.
``vocabulary.yaml`` — the names — stays at floor level, because the rooms did
not change what they are called when the robot re-mapped them. A combined
``zones.yaml`` is still read (:func:`read_floor` migrates it) and is replaced
by the pair the first time anything writes.
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

#: The combined names-and-coordinates file every floor used to have. Still
#: read, never written: :func:`read_floor` migrates one it finds, because a
#: robot that has been mapping a building for a year should not have to be
#: re-taught to gain the split.
ZONES_YAML = "zones.yaml"

#: The two halves zone/v0 splits it into. ``binding.yaml`` is coordinates in
#: one robot's map frame and travels *inside* a map revision; ``vocabulary.yaml``
#: is names, sits at floor level, and is the one of the two that may be
#: broadcast to every robot at the site.
BINDING_YAML = "binding.yaml"
VOCABULARY_YAML = "vocabulary.yaml"

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
        BINDING_YAML,
        VOCABULARY_YAML,
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
    """A **combined** ``zones.yaml`` — the shape a floor had before the split.

    Kept because that shape is still written by the sim world files and by any
    robot mapping a building since before zone/v0. It is not what a floor reads
    through: :func:`read_floor` is, and it puts a combined file through
    ``zone.split`` so the legacy path and the split path cannot produce
    different structures.
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
        zone = {"name": str(name), **zone_term(where, name, entry)}
        for key in ("x", "y", "yaw", "radius"):
            if entry.get(key) is not None:
                try:
                    zone[key] = float(entry[key])
                except (TypeError, ValueError) as exc:
                    raise BundleError(
                        f"{where}: zone {name!r} has a bad {key}"
                    ) from exc
        polygon = entry.get("polygon")
        if polygon is not None:
            zone["polygon"] = _polygon(where, name, polygon)
        if "x" not in zone or "y" not in zone:
            # A polygon-only zone is legal — the loader derives a pose inside
            # the outline (mote_tasks.zones) — but a zone with neither is not
            # a place at all.
            if "polygon" not in zone:
                raise BundleError(f"{where}: zone {name!r} has no position")
        parsed[str(name)] = zone
    return {
        "frame_id": raw.get("frame_id") or "map",
        "revision": _revision(where, raw.get("vocabulary_revision")),
        "zones": parsed,
    }


def _revision(where: str, raw) -> int:
    if raw is None:
        return 0
    try:
        revision = int(raw)
    except (TypeError, ValueError) as exc:
        raise BundleError(f"{where}: vocabulary_revision must be an integer") from exc
    if revision < 0:
        raise BundleError(f"{where}: vocabulary_revision must not be negative")
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
    """The shared half of :func:`read_floor`'s output, as a zone/v0 document.

    This is the whole point of the split. A vocabulary travels — to a
    dispatcher, to a second robot at the same site, to anything that needs to
    know what places can be *named* — because names are portable. The binding
    beside it is not.

    Local extensions are left out: a zone this robot was taught but nobody has
    named for the site is real and usable here, and advertising it as a shared
    zone would be this robot inventing vocabulary for its neighbours.
    """
    terms = [
        {key: item[key] for key in ("name",) + VOCABULARY_KEYS}
        for item in zones["zones"].values()
        if not item.get("local")
    ]
    document = zone.vocabulary(site, floor, terms, revision=zones.get("revision", 0))
    document["problems"] = check_vocabulary(terms)
    return document


def binding(zones: dict, site: str, floor: str, platform_id: str) -> dict:
    """The private half, as a zone/v0 document. Never leaves this robot."""
    bindings = []
    for item in zones["zones"].values():
        footprint = None
        if item.get("polygon"):
            footprint = {"type": "polygon", "vertices": item["polygon"]}
        elif item.get("radius") is not None:
            footprint = {"type": "circle", "radius": item["radius"]}
        anchored = item.get("anchor") or zone.anchor()
        if "x" in item and "y" in item:
            x, y, yaw = item["x"], item["y"], item.get("yaw", 0.0)
        elif footprint is not None and footprint["type"] == "polygon":
            # zone/v0 requires a binding to carry a pose: it is where a mission
            # navigates to, and an outline alone cannot say. Derived once, here.
            x, y = zone.representative_point(footprint["vertices"])
            yaw = 0.0
            anchored = zone.anchor(zone.DERIVED, by="polygon")
        else:
            raise BundleError(
                f"zone {item['name']!r} has neither a pose nor an outline"
            )
        bindings.append(
            zone.bound(item["name"], x, y, yaw, footprint=footprint, anchored=anchored)
        )
    return zone.binding(
        platform_id,
        site,
        floor,
        bindings,
        frame_id=zones.get("frame_id") or "map",
        map_revision=zones.get("map_revision") or "",
        vocabulary_revision=zones.get("revision", 0),
    )


def write_floor(
    directory,
    merged: dict,
    *,
    site: str = "",
    floor: str = "",
    platform_id: str | None = None,
):
    """Write a floor's zones as the split pair, replacing a combined file.

    Both documents are written before either is moved into place, and the
    combined file is removed only afterwards: a floor caught halfway through
    this by a power cut must come back as *one* readable layout, not as a
    vocabulary with no coordinates under it.
    """
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    site = site or merged.get("site") or LOCAL_SITE
    floor = floor or merged.get("floor") or LOCAL_FLOOR
    if platform_id is None:
        platform_id = merged.get("platform_id") or ""
    _atomic(directory / VOCABULARY_YAML, vocabulary(merged, site, floor))
    _atomic(directory / BINDING_YAML, binding(merged, site, floor, platform_id))
    legacy = directory / ZONES_YAML
    if legacy.exists():
        # Kept, not deleted: it is the only record of what the floor looked
        # like before the split, and it costs a few kilobytes.
        legacy.rename(directory / f"{ZONES_YAML}.premigration")


def _atomic(path: Path, document: dict):
    document = {key: value for key, value in document.items() if key != "problems"}
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(document, sort_keys=False, default_flow_style=None))
    os.replace(tmp, path)


def read_floor(directory, site: str = "", floor: str = "") -> dict:
    """A floor's zones, from whichever of the two layouts is on disk.

    The split pair wins where it exists. A lone ``zones.yaml`` is **migrated on
    read** rather than refused: a robot that has been mapping a building for a
    year should not have to be re-taught to gain the split, and the sim worlds
    and the committed default ship as combined files on purpose — one file is
    the right shape for a fixture that has exactly one robot in it.

    ``site``/``floor`` are only needed to stamp a *migration*; reading a split
    pair takes them from the documents, which is where they belong.
    """
    directory = Path(directory).expanduser()
    if directory.is_file():
        return _migrate(directory, site, floor)
    vocabulary_path = directory / VOCABULARY_YAML
    binding_path = directory / BINDING_YAML
    if vocabulary_path.is_file() or binding_path.is_file():
        # Either half alone is a legitimate directory. A **map revision**
        # carries only the binding, because coordinates travel with the frame
        # they mean something in and the names of the rooms do not; a floor
        # nobody has driven yet carries only the vocabulary, which is the
        # portability the split buys. What comes back is the join either way.
        vocabulary_doc = (
            load_yaml_file(vocabulary_path)
            if vocabulary_path.is_file()
            else {"site": site, "floor": floor, "revision": 0, "zones": []}
        )
        binding_doc = load_yaml_file(binding_path) if binding_path.is_file() else None
        try:
            merged = zone.merge(vocabulary_doc, binding_doc)
        except SpecError as exc:
            raise BundleError(f"{vocabulary_path.name}: {exc}") from exc
        return _typed(vocabulary_path.name, merged)
    legacy = directory / ZONES_YAML
    if legacy.is_file():
        # site/floor are stamped onto the migrated documents and then thrown
        # away by the merge, so a caller that has them passes them and one that
        # does not — a revision directory, whose path names a revision and not
        # a floor — gets placeholders rather than a wrong answer dressed up as
        # a right one.
        return _migrate(legacy, site, floor)
    raise BundleError(f"{directory}: no {VOCABULARY_YAML} and no {ZONES_YAML}")


def _migrate(path, site: str, floor: str) -> dict:
    """A combined ``zones.yaml``, read through the split and back.

    Round-tripping through :func:`~mote_bringup.spec.zone.split` rather than
    parsing the legacy shape directly is deliberate: it means the legacy path
    and the split path produce the *same* structure by construction, so a bug
    in one is a bug in both rather than a difference nobody notices until a
    floor is migrated.
    """
    zones = read_zones(path)
    try:
        vocabulary_doc, binding_doc = zone.split(
            zones,
            site=site or LOCAL_SITE,
            floor=floor or LOCAL_FLOOR,
            platform_id="",
        )
        return _typed(Path(path).name, zone.merge(vocabulary_doc, binding_doc))
    except SpecError as exc:
        raise BundleError(f"{Path(path).name}: {exc}") from exc


def _typed(where: str, merged: dict) -> dict:
    """Numbers as numbers, and a polygon that is a list of pairs."""
    for name, item in merged["zones"].items():
        for key in ("x", "y", "yaw", "radius"):
            if item.get(key) is not None:
                try:
                    item[key] = float(item[key])
                except (TypeError, ValueError) as exc:
                    raise BundleError(
                        f"{where}: zone {name!r} has a bad {key}"
                    ) from exc
        if item.get("polygon") is not None:
            item["polygon"] = _polygon(where, name, item["polygon"])
        if "x" not in item and "polygon" not in item and item.get("bound"):
            raise BundleError(f"{where}: zone {name!r} has no position")
    return merged


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

    # A revision carries the *binding*: coordinates are only meaningful in the
    # map frame beside them, so they travel with the map or not at all. The
    # vocabulary is a floor-level fact about the building and does not have to.
    if BINDING_YAML in report.files or ZONES_YAML in report.files:
        try:
            report.zones = read_floor(revision_dir)
        except BundleError as exc:
            report.errors.append(str(exc))
        else:
            # An ambiguous vocabulary is a warning here, not an error: the map
            # is good and every coordinate in it is good. What it costs is
            # dispatch by name, so it must be *said* — but refusing to publish
            # a floor's map over a duplicated alias would be the wrong price.
            report.warnings.extend(
                f"vocabulary: {problem}"
                for problem in check_vocabulary(report.zones["zones"].values())
            )
    else:
        report.warnings.append(f"no {BINDING_YAML} — this floor has no taught places")

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
    # not what it claims and every zone taught on this floor is suspect.
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

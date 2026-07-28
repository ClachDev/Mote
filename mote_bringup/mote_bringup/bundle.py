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

It is *not* dependency-free, and that was a correction. It first shipped with a
hand-rolled parser for "the YAML subset bundles are written in", to keep the
server's dependency list at exactly "python". Three things were wrong with that.
The list was already "python plus paho"; PyYAML is the library that *writes*
these files, so a second reader can only ever approximate it; and it did — a
zone called ``Café`` came back as ``Caf\xe9`` with no error at all, an
apostrophe in a room name raised, and the block-sequence-of-flow-pairs shape
that ``safe_dump(default_flow_style=None)`` emits for a polygon — i.e. the
output of ``segment-map`` and ``save-zone`` — did not parse. Parsing input this
module exists to distrust is the last place to save a 200 KB pure-Python wheel,
so it reads YAML with PyYAML and the fleet image installs it. The stdlib-only
rule still holds where it earns its keep: :mod:`mote_fleet.protocol`, and the
PNG reader below, which is 140 lines against Pillow's 4 MB and is only ever
pointed at one well-specified format.

What a validated revision has to contain::

    map.yaml        resolution/origin/image — the map frame itself
    <image>         the occupancy PNG map.yaml names (map.png in practice)
    meta.yaml       provenance: when it was saved, from which bag, cleaning stats
    map.posegraph   slam_toolbox graph, and
    map.data        ...its data — together, what lets mapping continue in this
                    frame later. A revision without them is servable but is a
                    dead end, which is an error for a published revision and a
                    warning for one being read.

``zones.yaml`` is a floor-level file rather than a revision-level one, but it
travels *inside* a published revision: zone coordinates are only meaningful in
the map frame they were taught in, so the two must move together or the fleet
ends up drawing a kitchen through a wall.
"""

import gzip
import hashlib
import io
import tarfile
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SCHEMA = 1

MAP_YAML = "map.yaml"
META_YAML = "meta.yaml"
ZONES_YAML = "zones.yaml"
POSEGRAPH = "map.posegraph"
POSEGRAPH_DATA = "map.data"

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

#: An occupancy map this reader will decode, in raw bytes. Sized so the
#: 1158x761 hospital floor (0.9 MB) has three orders of magnitude of headroom
#: while a decompression bomb does not: the decoder inflates to this at most.
MAX_PIXELS = 512 * 1024 * 1024

#: No floor is 10 km across. A resolution/size pair that claims otherwise is a
#: units mistake, and it would blow up every consumer's view transform.
MAX_EXTENT_M = 10_000.0


class BundleError(ValueError):
    """A bundle, or a file in one, that does not meet the contract."""


# --------------------------------------------------------------------------
# the YAML the bundle is written in
# --------------------------------------------------------------------------


def load_yaml(text: str):
    """Parse bundle YAML, raising :class:`BundleError` rather than a YAMLError.

    PyYAML, deliberately — see this module's docstring. The only thing added
    here is the error type, so that every "this bundle is not readable" failure
    reaches a caller as one exception class.
    """
    try:
        return yaml.safe_load(text)
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
    """A floor's ``zones.yaml`` — ``{frame_id, zones: {name: {...}}}``.

    The authority on what a zone *means* is ``mote_tasks.zones``; this is the
    off-robot reader for the same file, so the fleet can draw taught places and
    the operator can see the ``goto`` targets they are about to type. It keeps
    the shape and checks the numbers rather than reimplementing membership.
    """
    raw = load_yaml_file(path)
    zones = raw.get("zones") or {}
    if not isinstance(zones, dict):
        raise BundleError(f"{Path(path).name}: 'zones' must be a mapping")
    parsed = {}
    for name, entry in zones.items():
        if not isinstance(entry, dict):
            raise BundleError(f"{Path(path).name}: zone {name!r} is not a mapping")
        zone = {"name": str(name)}
        for key in ("x", "y", "yaw", "radius"):
            if entry.get(key) is not None:
                try:
                    zone[key] = float(entry[key])
                except (TypeError, ValueError) as exc:
                    raise BundleError(
                        f"{Path(path).name}: zone {name!r} has a bad {key}"
                    ) from exc
        polygon = entry.get("polygon")
        if polygon is not None:
            zone["polygon"] = _polygon(Path(path).name, name, polygon)
        if "x" not in zone or "y" not in zone:
            # A polygon-only zone is legal — the loader derives a pose inside
            # the outline (mote_tasks.zones) — but a zone with neither is not
            # a place at all.
            if "polygon" not in zone:
                raise BundleError(f"{Path(path).name}: zone {name!r} has no position")
        parsed[str(name)] = zone
    return {"frame_id": raw.get("frame_id") or "map", "zones": parsed}


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


def png_size(path) -> tuple[int, int] | None:
    """Pixel dimensions from a PNG header.

    Reading 24 bytes beats making a browser wait for the image to decode before
    it can place a robot, and beats a Pillow dependency for two numbers at a
    fixed offset.
    """
    try:
        with open(path, "rb") as handle:
            header = handle.read(24)
    except OSError:
        return None
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    return int.from_bytes(header[16:20], "big"), int.from_bytes(header[20:24], "big")


def occupancy(path) -> dict:
    """Fractions of free / occupied / unknown cells in an occupancy PNG.

    This is the check that catches the mapping run that never got going: a
    revision can have every file in place, a sane ``map.yaml``, and still be a
    uniform grey rectangle. Nothing else in the pipeline looks at the pixels.

    Decoded here rather than with an image library: an 8-bit greyscale PNG is
    zlib plus five one-line filters, against 4 MB of Pillow on a box whose whole
    job is answering HTTP. Anything this cannot read returns ``reason`` instead
    of counts — unreadable pixels are a thing to report, not to guess — with
    ``corrupt`` saying which kind: a flavour this decoder does not do (16-bit,
    palette, interlaced) is a map somebody else can still serve, while a stream
    that contradicts its own header is what a truncated upload looks like.
    """
    try:
        blob = Path(path).read_bytes()
    except OSError as exc:
        return {"reason": str(exc), "corrupt": False}
    rows = _decode_png_gray(blob)
    if isinstance(rows, dict):
        return rows
    free = occupied = unknown = 0
    for row in rows:
        for value in row:
            if value <= OCCUPIED_MAX:
                occupied += 1
            elif value >= FREE_MIN:
                free += 1
            else:
                unknown += 1
    total = free + occupied + unknown
    if not total:
        return {"reason": "the image has no pixels", "corrupt": True}
    return {
        "total": total,
        "free": round(free / total, 6),
        "occupied": round(occupied / total, 6),
        "unknown": round(unknown / total, 6),
    }


def _broken(reason: str) -> dict:
    """A PNG that no reader will accept: the bytes are wrong."""
    return {"reason": reason, "corrupt": True}


def _unsupported(reason: str) -> dict:
    """A valid PNG in a flavour this decoder does not do."""
    return {"reason": reason, "corrupt": False}


def _decode_png_gray(blob: bytes):
    """Rows of 8-bit samples from the first channel, or why not.

    A failure is ``{"reason": ..., "corrupt": bool}``. The distinction is the
    one :func:`validate` acts on: a PNG this reader does not *support* (16-bit,
    palette, interlaced) is a map somebody else can still serve, so it costs
    the occupancy check and a warning. A PNG that is *broken* — a bad filter
    byte, a short IDAT, a stream that does not match its own header — is what a
    truncated upload looks like, and no consumer will read it either.
    """
    if blob[:8] != b"\x89PNG\r\n\x1a\n":
        return _broken("not a PNG")
    width = height = depth = colour = interlace = None
    data = bytearray()
    offset = 8
    while offset + 8 <= len(blob):
        length = int.from_bytes(blob[offset : offset + 4], "big")
        kind = blob[offset + 4 : offset + 8]
        body = blob[offset + 8 : offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            if len(body) < 13:
                return _broken("truncated IHDR")
            width = int.from_bytes(body[0:4], "big")
            height = int.from_bytes(body[4:8], "big")
            depth, colour, _, _, interlace = body[8:13]
        elif kind == b"IDAT":
            data += body
        elif kind == b"IEND":
            break
    if width is None:
        return _broken("no IHDR")
    if depth != 8:
        return _unsupported(f"bit depth {depth} is not 8")
    if interlace:
        return _unsupported("interlaced")
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(colour)
    if channels is None:
        return _unsupported(f"unknown colour type {colour}")
    if colour == 3:
        return _unsupported("palette images carry no occupancy value")
    stride = width * channels
    expected = (stride + 1) * height
    if expected > MAX_PIXELS:
        return _unsupported(
            f"image is {width}x{height}, larger than this reader will decode"
        )
    # Inflate no further than the header says the image is. A PNG that declares
    # itself 1x1 and carries 256 MB of compressed zeroes is otherwise a
    # decompression bomb on a route that, until M7, anything on the tailnet can
    # reach: 261 KB in, 786 MB of RSS out.
    try:
        raw = zlib.decompressobj().decompress(bytes(data), expected + 1)
    except zlib.error as exc:
        return _broken(f"corrupt image data: {exc}")
    if len(raw) > expected:
        return _broken("image data is larger than its dimensions allow")
    if len(raw) < expected:
        return _broken("truncated image data")
    rows = []
    previous = bytearray(stride)
    position = 0
    for _ in range(height):
        filter_type = raw[position]
        line = bytearray(raw[position + 1 : position + 1 + stride])
        position += 1 + stride
        if filter_type > 4:
            return _broken(f"unknown PNG filter {filter_type}")
        _unfilter(filter_type, line, previous, channels)
        rows.append(line[::channels] if channels > 1 else line)
        previous = line
    return rows


def _unfilter(filter_type: int, line: bytearray, previous: bytearray, channels: int):
    """Reverse one scanline filter, in place. Callers check the type first:
    every failure in this decoder is a returned reason, never an exception, or
    :func:`validate` cannot honour its own "never raises" contract."""
    if filter_type == 0:
        return
    for index in range(len(line)):
        left = line[index - channels] if index >= channels else 0
        up = previous[index]
        upper_left = previous[index - channels] if index >= channels else 0
        if filter_type == 1:
            line[index] = (line[index] + left) & 0xFF
        elif filter_type == 2:
            line[index] = (line[index] + up) & 0xFF
        elif filter_type == 3:
            line[index] = (line[index] + (left + up) // 2) & 0xFF
        else:
            line[index] = (line[index] + _paeth(left, up, upper_left)) & 0xFF


def _paeth(left: int, up: int, upper_left: int) -> int:
    estimate = left + up - upper_left
    distances = (
        abs(estimate - left),
        abs(estimate - up),
        abs(estimate - upper_left),
    )
    smallest = min(distances)
    if distances[0] == smallest:
        return left
    if distances[1] == smallest:
        return up
    return upper_left


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

    for name in CONTINUABLE:
        if report.files.get(name):
            continue
        message = (
            f"{name} is missing — mapping cannot be continued in this frame "
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

    if ZONES_YAML in report.files:
        try:
            report.zones = read_zones(revision_dir / ZONES_YAML)
        except BundleError as exc:
            report.errors.append(str(exc))
    else:
        report.warnings.append("no zones.yaml — this floor has no taught places")

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

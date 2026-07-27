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

That is also why this file is **stdlib-only and ROS-free**, the same discipline
as :mod:`mote_fleet.protocol` and ``mote_perception/depth_wire.py``. There is
no PyYAML here either: the fleet server's whole dependency list is "python",
and the bundle's YAML is a small, known subset — flat scalars in ``map.yaml``,
one nesting level in ``meta.yaml``, and flow-style mappings with polygon lists
in ``zones.yaml``. :func:`load_yaml` parses exactly that subset and refuses the
rest rather than half-understanding it; ``test_bundle.py`` differential-tests it
against PyYAML on every bundle file committed in this repo.

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

#: No floor is 10 km across. A resolution/size pair that claims otherwise is a
#: units mistake, and it would blow up every consumer's view transform.
MAX_EXTENT_M = 10_000.0


class BundleError(ValueError):
    """A bundle, or a file in one, that does not meet the contract."""


# --------------------------------------------------------------------------
# the YAML subset
# --------------------------------------------------------------------------


def load_yaml(text: str):
    """Parse the YAML subset site bundles are written in.

    Block mappings and sequences by indentation, flow mappings and sequences
    (which may span lines — the hospital's polygons do), comments, and plain
    scalars. Anchors, tags, multi-line scalars, multiple documents and merge
    keys are **not** supported and raise: silently mis-reading a map's origin
    would put every robot in the wrong place, so an unknown construct is an
    error rather than a guess.
    """
    lines = []
    for number, raw in enumerate(text.splitlines(), start=1):
        content = _strip_comment(raw)
        if not content.strip():
            continue
        if content.lstrip().startswith(("---", "...")):
            continue
        if content.lstrip().startswith(("&", "*", "!", ">", "|")):
            raise BundleError(f"line {number}: unsupported YAML construct {content!r}")
        lines.append((len(content) - len(content.lstrip()), content.strip(), number))
    if not lines:
        return None
    value, index = _parse_block(lines, 0, lines[0][0])
    if index != len(lines):
        raise BundleError(f"line {lines[index][2]}: unexpected indentation")
    return value


def _strip_comment(line: str) -> str:
    quote = ""
    for index, char in enumerate(line):
        if quote:
            if char == quote:
                quote = ""
        elif char in "\"'":
            quote = char
        elif char == "#" and (index == 0 or line[index - 1] in " \t"):
            return line[:index]
    return line


def _parse_block(lines, index: int, indent: int):
    """One block collection at ``indent``. Returns ``(value, next_index)``."""
    if lines[index][1].startswith("- ") or lines[index][1] == "-":
        return _parse_sequence(lines, index, indent)
    return _parse_mapping(lines, index, indent)


def _parse_sequence(lines, index: int, indent: int):
    items = []
    while index < len(lines) and lines[index][0] == indent:
        line_indent, content, number = lines[index]
        if not (content.startswith("- ") or content == "-"):
            break
        rest = content[1:].strip()
        index += 1
        if not rest:
            if index < len(lines) and lines[index][0] > line_indent:
                value, index = _parse_block(lines, index, lines[index][0])
            else:
                value = None
        elif _open_flow(rest):
            joined, index = _join_flow(lines, index, rest, number)
            value = parse_flow(joined)
        else:
            value = _scalar(rest, number)
        items.append(value)
    return items, index


def _parse_mapping(lines, index: int, indent: int):
    mapping = {}
    while index < len(lines) and lines[index][0] == indent:
        _, content, number = lines[index]
        if content.startswith("- "):
            break
        key, separator, rest = _split_key(content)
        if not separator:
            raise BundleError(f"line {number}: expected 'key: value', got {content!r}")
        key = _scalar(key.strip(), number)
        rest = rest.strip()
        index += 1
        if not rest:
            if index < len(lines) and lines[index][0] > indent:
                value, index = _parse_block(lines, index, lines[index][0])
            else:
                value = None
        elif _open_flow(rest):
            joined, index = _join_flow(lines, index, rest, number)
            value = parse_flow(joined)
        else:
            value = _scalar(rest, number)
        mapping[key] = value
    return mapping, index


def _split_key(content: str):
    """Split ``key: value`` at the first ``:`` outside quotes and brackets."""
    quote, depth = "", 0
    for index, char in enumerate(content):
        if quote:
            if char == quote:
                quote = ""
        elif char in "\"'":
            quote = char
        elif char in "[{":
            depth += 1
        elif char in "]}":
            depth -= 1
        elif char == ":" and depth == 0:
            if index + 1 == len(content) or content[index + 1] in " \t":
                return content[:index], ":", content[index + 1 :]
    return content, "", ""


def _open_flow(text: str) -> bool:
    return text.startswith(("[", "{"))


def _join_flow(lines, index: int, first: str, number: int):
    """Consume lines until the flow collection started in ``first`` closes."""
    joined = first
    while _flow_depth(joined) > 0:
        if index >= len(lines):
            raise BundleError(f"line {number}: unterminated flow collection")
        joined += " " + lines[index][1]
        index += 1
    if _flow_depth(joined) < 0:
        raise BundleError(f"line {number}: unbalanced brackets")
    return joined, index


def _flow_depth(text: str) -> int:
    quote, depth = "", 0
    for char in text:
        if quote:
            if char == quote:
                quote = ""
        elif char in "\"'":
            quote = char
        elif char in "[{":
            depth += 1
        elif char in "]}":
            depth -= 1
    return depth


def parse_flow(text: str):
    """Parse one flow collection or scalar, e.g. ``{x: 1, polygon: [[0, 0]]}``."""
    value, offset = _flow_value(text, 0)
    if text[offset:].strip():
        raise BundleError(f"trailing text after value: {text[offset:]!r}")
    return value


def _flow_value(text: str, index: int):
    index = _skip_space(text, index)
    if index >= len(text):
        raise BundleError("expected a value")
    if text[index] == "[":
        return _flow_collection(text, index, "]")
    if text[index] == "{":
        return _flow_collection(text, index, "}")
    return _flow_scalar(text, index)


def _flow_collection(text: str, index: int, closer: str):
    items, mapping = [], {}
    index = _skip_space(text, index + 1)
    while index < len(text) and text[index] != closer:
        key_or_value, index = _flow_value(text, index)
        index = _skip_space(text, index)
        if closer == "}":
            if index >= len(text) or text[index] != ":":
                raise BundleError(f"expected ':' after key {key_or_value!r}")
            value, index = _flow_value(text, index + 1)
            mapping[key_or_value] = value
        else:
            items.append(key_or_value)
        index = _skip_space(text, index)
        if index < len(text) and text[index] == ",":
            index = _skip_space(text, index + 1)
    if index >= len(text):
        raise BundleError(f"expected '{closer}'")
    return (mapping if closer == "}" else items), index + 1


def _flow_scalar(text: str, index: int):
    if text[index] in "\"'":
        quote = text[index]
        end = text.find(quote, index + 1)
        if end < 0:
            raise BundleError(f"unterminated string at offset {index}")
        return text[index + 1 : end], end + 1
    end = index
    while end < len(text) and text[end] not in ",:[]{}":
        end += 1
    return _scalar(text[index:end].strip(), None), end


def _skip_space(text: str, index: int) -> int:
    while index < len(text) and text[index] in " \t":
        index += 1
    return index


def _scalar(token: str, number):
    if len(token) >= 2 and token[0] == token[-1] and token[0] in "\"'":
        return token[1:-1]
    lowered = token.lower()
    if lowered in ("null", "~", ""):
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        return int(token)
    except ValueError:
        pass
    try:
        return float(token)
    except ValueError:
        pass
    where = f"line {number}: " if number else ""
    if token.startswith(("&", "*", "!")):
        raise BundleError(f"{where}unsupported YAML construct {token!r}")
    if ": " in token:
        # `a: b: c` is a nested mapping in real YAML and a string here, so it
        # is refused rather than read as a value nobody wrote.
        raise BundleError(f"{where}ambiguous value {token!r} — quote it")
    return token


def load_yaml_file(path) -> dict:
    """``load_yaml`` over a file, as a mapping. Raises BundleError."""
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

    Decoded here rather than with an image library because the fleet server has
    no dependencies, and an 8-bit greyscale PNG is zlib plus five one-line
    filters. Anything else (16-bit, palette, interlaced) returns ``reason``
    instead of counts — unreadable pixels are a thing to report, not to guess.
    """
    try:
        blob = Path(path).read_bytes()
    except OSError as exc:
        return {"reason": str(exc)}
    rows = _decode_png_gray(blob)
    if isinstance(rows, str):
        return {"reason": rows}
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
        return {"reason": "the image has no pixels"}
    return {
        "total": total,
        "free": round(free / total, 6),
        "occupied": round(occupied / total, 6),
        "unknown": round(unknown / total, 6),
    }


def _decode_png_gray(blob: bytes):
    """Rows of 8-bit samples from the first channel, or a reason it cannot be."""
    if blob[:8] != b"\x89PNG\r\n\x1a\n":
        return "not a PNG"
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
                return "truncated IHDR"
            width = int.from_bytes(body[0:4], "big")
            height = int.from_bytes(body[4:8], "big")
            depth, colour, _, _, interlace = body[8:13]
        elif kind == b"IDAT":
            data += body
        elif kind == b"IEND":
            break
    if width is None:
        return "no IHDR"
    if depth != 8:
        return f"bit depth {depth} is not 8"
    if interlace:
        return "interlaced"
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(colour)
    if channels is None:
        return f"unknown colour type {colour}"
    if colour == 3:
        return "palette images carry no occupancy value"
    try:
        raw = zlib.decompress(bytes(data))
    except zlib.error as exc:
        return f"corrupt image data: {exc}"
    stride = width * channels
    if len(raw) < (stride + 1) * height:
        return "truncated image data"
    rows = []
    previous = bytearray(stride)
    position = 0
    for _ in range(height):
        filter_type = raw[position]
        line = bytearray(raw[position + 1 : position + 1 + stride])
        position += 1 + stride
        _unfilter(filter_type, line, previous, channels)
        rows.append(line[::channels] if channels > 1 else line)
        previous = line
    return rows


def _unfilter(filter_type: int, line: bytearray, previous: bytearray, channels: int):
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
        elif filter_type == 4:
            line[index] = (line[index] + _paeth(left, up, upper_left)) & 0xFF
        else:
            raise BundleError(f"unknown PNG filter {filter_type}")


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
        report.warnings.append(
            f"could not read occupancy: {report.occupancy['reason']}"
        )
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

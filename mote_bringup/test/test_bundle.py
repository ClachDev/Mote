"""The shared bundle module: the YAML subset, the validator, and the wire form.

Two ends depend on this file agreeing with itself — a robot writing a map
revision and a fleet server deciding whether to serve it — and one of them has
no PyYAML, so the parser here is first-party. That makes the differential test
below the important one: every bundle YAML file committed in this repo is parsed
both ways and must come out identical. A parser that is *nearly* right about an
origin puts every robot in the wrong place.
"""

import gzip
import struct
import zlib
from pathlib import Path

import pytest
import yaml

from mote_bringup import bundle

REPO = Path(__file__).resolve().parents[2]


# ---- the YAML subset ----------------------------------------------------


def committed_yaml() -> list[Path]:
    """Every YAML file that is part of a site bundle in this repository."""
    roots = [
        REPO / "mote_simulation" / "sim_home" / "sites",
        REPO / "mote_simulation" / "worlds",
        REPO / "mote_tasks" / "config",
    ]
    found = []
    for root in roots:
        found += sorted(p for p in root.rglob("*.yaml") if p.is_file())
    return found


@pytest.mark.parametrize("path", committed_yaml(), ids=lambda p: p.name)
def test_the_subset_parser_agrees_with_pyyaml(path):
    assert bundle.load_yaml(path.read_text()) == yaml.safe_load(path.read_text())


def test_multi_line_flow_mappings_and_polygon_lists():
    """The hospital's room zones are written this way by the world generator:
    a flow mapping that wraps onto a second line, holding a list of vertices."""
    parsed = bundle.load_yaml(
        "zones:\n"
        "  ward: {x: 1.0, y: 2.0, yaw: 1.571,\n"
        "    polygon: [[0, 0], [2, 0], [2, 2], [0, 2]]}\n"
    )
    assert parsed["zones"]["ward"]["polygon"][2] == [2, 2]
    assert parsed["zones"]["ward"]["x"] == 1.0


def test_comments_and_quotes_do_not_confuse_each_other():
    parsed = bundle.load_yaml(
        "# leading comment\nsaved: '2026-07-27T10:15:00'  # when\n"
        "note: 'a # inside quotes'\n"
    )
    assert parsed == {"saved": "2026-07-27T10:15:00", "note": "a # inside quotes"}


def test_scalars_keep_their_types():
    parsed = bundle.load_yaml(
        "i: 3\nf: 0.05\nt: true\nf2: false\nn: null\ntilde: ~\ns: trinary\n"
    )
    assert parsed == {
        "i": 3,
        "f": 0.05,
        "t": True,
        "f2": False,
        "n": None,
        "tilde": None,
        "s": "trinary",
    }


def test_block_sequences_nest():
    assert bundle.load_yaml("a:\n  - 1\n  - 2\nb:\n  c: 3\n") == {
        "a": [1, 2],
        "b": {"c": 3},
    }


@pytest.mark.parametrize(
    "text",
    [
        "a: &anchor 1\n",  # anchors would need a reference table
        "a: [1, 2\n",  # never closed
        "just a scalar and: then: two colons\n",
    ],
)
def test_constructs_outside_the_subset_are_refused_not_guessed(text):
    with pytest.raises(bundle.BundleError):
        bundle.load_yaml(text)


# ---- map.yaml -----------------------------------------------------------


def test_read_map_of_a_real_committed_map(tmp_path):
    path = (
        REPO / "mote_simulation/sim_home/sites/office_world/floors/ground/map/map.yaml"
    )
    meta = bundle.read_map(path)
    assert meta["resolution"] == 0.05
    assert len(meta["origin"]) == 3
    assert meta["image"] == "map.png"


@pytest.mark.parametrize(
    "text,message",
    [
        ("resolution: 0.05\norigin: [0, 0, 0]\n", "missing image"),
        ("image: map.png\norigin: [0, 0, 0]\n", "missing resolution"),
        ("image: map.png\nresolution: 0\norigin: [0, 0, 0]\n", "must be positive"),
        ("image: map.png\nresolution: 0.05\norigin: [0]\n", "at least x and y"),
        ("image: ../etc/passwd\nresolution: 0.05\norigin: [0, 0]\n", "plain file name"),
        (
            "image: m.png\nresolution: 0.05\norigin: [0, 0]\n"
            "free_thresh: 0.9\noccupied_thresh: 0.2\n",
            "is not below",
        ),
    ],
)
def test_a_map_yaml_that_cannot_be_trusted_is_refused(tmp_path, text, message):
    path = tmp_path / "map.yaml"
    path.write_text(text)
    with pytest.raises(bundle.BundleError, match=message):
        bundle.read_map(path)


# ---- zones --------------------------------------------------------------


def test_zones_carry_poses_and_footprints(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "frame_id: map\nzones:\n"
        "  kitchen: {x: 1.0, y: 2.0, yaw: 0.0, radius: 1.5}\n"
        "  ward: {x: 4.0, y: 1.0, polygon: [[3, 0], [5, 0], [5, 2], [3, 2]]}\n"
    )
    zones = bundle.read_zones(path)
    assert zones["frame_id"] == "map"
    assert zones["zones"]["kitchen"]["radius"] == 1.5
    assert len(zones["zones"]["ward"]["polygon"]) == 4


def test_a_polygon_only_zone_is_legal(tmp_path):
    """mote_tasks derives a pose inside the outline, so a polygon with no x/y
    is a place even though it has no stated position."""
    path = tmp_path / "zones.yaml"
    path.write_text("zones:\n  ward: {polygon: [[0, 0], [2, 0], [2, 2]]}\n")
    assert "polygon" in bundle.read_zones(path)["zones"]["ward"]


@pytest.mark.parametrize(
    "text,message",
    [
        ("zones:\n  ward: {yaw: 1.0}\n", "no position"),
        ("zones:\n  ward: {polygon: [[0, 0], [1, 1]]}\n", "at least 3 vertices"),
        ("zones:\n  ward: {polygon: [[0, 0], [1, 1], [2]]}\n", "not \\[x, y\\]"),
        ("zones: [1, 2]\n", "must be a mapping"),
    ],
)
def test_a_zone_that_is_not_a_place_is_refused(tmp_path, text, message):
    path = tmp_path / "zones.yaml"
    path.write_text(text)
    with pytest.raises(bundle.BundleError, match=message):
        bundle.read_zones(path)


# ---- the occupancy image ------------------------------------------------


def png(path, width, height, values, colour=0, depth=8):
    """A greyscale PNG built here rather than by an image library, so the
    decoder is tested against the format."""

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    raw = b"".join(
        b"\x00" + bytes(values(x, y) for x in range(width)) for y in range(height)
    )
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, depth, colour, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def test_occupancy_counts_a_real_saved_map():
    path = REPO / "mote_simulation/sim_home/sites/mote_world/floors/ground/map/map.png"
    counts = bundle.occupancy(path)
    assert counts["total"] == 117 * 117
    assert counts["occupied"] > 0 and counts["free"] > 0
    assert abs(counts["free"] + counts["occupied"] + counts["unknown"] - 1.0) < 1e-6


def test_occupancy_says_why_when_it_cannot_read_the_pixels(tmp_path):
    path = tmp_path / "map.png"
    png(path, 4, 4, lambda x, y: 254, depth=8, colour=3)
    assert "palette" in bundle.occupancy(path)["reason"]


def test_png_size_reads_the_header_not_the_image():
    path = (
        REPO / "mote_simulation/sim_home/sites/office_world/floors/ground/map/map.png"
    )
    assert bundle.png_size(path) == (438, 238)


# ---- validation ---------------------------------------------------------


def revision(directory, **kwargs) -> Path:
    """A complete revision, as ``sites.save_map`` leaves one."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "map.yaml").write_text(
        kwargs.get(
            "map_yaml",
            "image: map.png\nmode: trinary\nresolution: 0.05\n"
            "origin: [-2.9, -2.9, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
        )
    )
    png(
        directory / "map.png",
        kwargs.get("width", 20),
        kwargs.get("height", 20),
        kwargs.get("pixels", lambda x, y: 0 if x == 0 else 254),
    )
    (directory / "meta.yaml").write_text("schema: 1\nsaved: '2026-07-27T10:15:00'\n")
    if kwargs.get("posegraph", True):
        (directory / "map.posegraph").write_bytes(b"graph")
        (directory / "map.data").write_bytes(b"data")
    if kwargs.get("zones", True):
        (directory / "zones.yaml").write_text(
            "frame_id: map\nzones:\n  a: {x: 0, y: 0}\n"
        )
    return directory


def test_a_complete_revision_validates(tmp_path):
    report = bundle.validate(revision(tmp_path / "20260727T101500"))
    assert report.ok, report.errors
    assert report.warnings == []
    assert report.map["width"] == 20
    assert report.occupancy["occupied"] > 0


def test_a_missing_posegraph_is_an_error_for_a_publisher_and_a_warning_for_a_reader(
    tmp_path,
):
    """The distinction is real: such a revision navigates perfectly and simply
    cannot be mapped further, so a robot must not publish it while a server
    that already holds one must still serve it."""
    directory = revision(tmp_path / "rev", posegraph=False)
    assert not bundle.validate(directory).ok
    lenient = bundle.validate(directory, require_posegraph=False)
    assert lenient.ok
    assert any("map.posegraph" in warning for warning in lenient.warnings)


def test_a_truncated_upload_is_caught(tmp_path):
    directory = revision(tmp_path / "rev")
    (directory / "map.yaml").write_text("")
    assert "map.yaml is empty" in bundle.validate(directory).errors


def test_a_map_with_no_free_space_is_refused(tmp_path):
    """What a mapping run that never got going looks like: every file present,
    a sane map.yaml, and a uniform grey rectangle."""
    directory = revision(tmp_path / "rev", pixels=lambda x, y: 205)
    errors = bundle.validate(directory).errors
    assert any("no free space" in error for error in errors)


def test_a_map_with_no_walls_is_refused(tmp_path):
    directory = revision(tmp_path / "rev", pixels=lambda x, y: 254)
    assert any("no occupied" in error for error in bundle.validate(directory).errors)


def test_the_image_must_be_the_one_map_yaml_names(tmp_path):
    directory = revision(tmp_path / "rev")
    (directory / "map.png").unlink()
    assert any("not here" in error for error in bundle.validate(directory).errors)


def test_a_raw_map_of_a_different_size_is_refused(tmp_path):
    """map.png and map_raw.png are the same frame with different pixels, so a
    size that differs means every zone taught on this floor is suspect."""
    directory = revision(tmp_path / "rev")
    png(directory / "map_raw.png", 10, 10, lambda x, y: 254)
    assert any("share a frame" in error for error in bundle.validate(directory).errors)


def test_a_floor_with_no_zones_validates_but_says_so(tmp_path):
    report = bundle.validate(revision(tmp_path / "rev", zones=False))
    assert report.ok
    assert any("no zones.yaml" in warning for warning in report.warnings)


# ---- the wire form ------------------------------------------------------


def test_pack_round_trips_through_unpack(tmp_path):
    source = revision(tmp_path / "20260727T101500")
    blob = bundle.pack(source)
    written = bundle.unpack(blob, tmp_path / "landed")
    assert set(written) == {
        "map.yaml",
        "map.png",
        "meta.yaml",
        "map.posegraph",
        "map.data",
        "zones.yaml",
    }
    assert bundle.validate(tmp_path / "landed").ok
    for name in written:
        assert (tmp_path / "landed" / name).read_bytes() == (source / name).read_bytes()


def test_packing_is_deterministic(tmp_path):
    """The registry announces a digest and re-packs the stored files to serve
    it, so the same revision has to pack to the same bytes every time."""
    source = revision(tmp_path / "rev")
    assert bundle.pack(source) == bundle.pack(source)
    assert bundle.digest(bundle.pack(source)).startswith("sha256:")


def test_extra_files_travel_with_the_frame(tmp_path):
    source = revision(tmp_path / "rev", zones=False)
    blob = bundle.pack(source, {"zones.yaml": b"frame_id: map\nzones: {}\n"})
    assert "zones.yaml" in bundle.unpack(blob, tmp_path / "landed")


def test_a_file_that_is_not_part_of_a_bundle_cannot_be_packed(tmp_path):
    with pytest.raises(bundle.BundleError, match="not part of a site bundle"):
        bundle.pack(revision(tmp_path / "rev"), {"authorized_keys": b"..."})


def hostile_tar(members) -> bytes:
    """A tar built by hand, so unpack is tested against what an attacker sends
    rather than against what tarfile is willing to build."""
    import io
    import tarfile

    buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=buffer, mode="wb", mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for name, kind, payload in members:
                info = tarfile.TarInfo(name)
                info.type = kind
                if kind == tarfile.SYMTYPE:
                    info.linkname = payload
                    archive.addfile(info)
                else:
                    info.size = len(payload)
                    archive.addfile(info, io.BytesIO(payload))
    return buffer.getvalue()


@pytest.mark.parametrize(
    "name",
    ["../map.yaml", "/etc/map.yaml", "sub/map.yaml", "authorized_keys"],
)
def test_unpack_refuses_a_member_that_names_somewhere_else(tmp_path, name):
    import tarfile

    blob = hostile_tar([(name, tarfile.REGTYPE, b"x")])
    with pytest.raises(bundle.BundleError, match="not part of a site bundle"):
        bundle.unpack(blob, tmp_path / "landed")
    assert not (tmp_path / "landed").exists()


def test_unpack_refuses_a_symlink(tmp_path):
    import tarfile

    blob = hostile_tar([("map.yaml", tarfile.SYMTYPE, "/etc/passwd")])
    with pytest.raises(bundle.BundleError, match="not a plain file"):
        bundle.unpack(blob, tmp_path / "landed")


def test_unpack_refuses_something_that_is_not_an_archive(tmp_path):
    with pytest.raises(bundle.BundleError, match="not a readable bundle"):
        bundle.unpack(b"definitely not a tarball", tmp_path / "landed")

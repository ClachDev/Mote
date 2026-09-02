"""The shared bundle module: reading a bundle, validating it, and its wire form.

Two ends depend on this file agreeing with itself — a robot writing a map
revision and a fleet server deciding whether to serve it — so the tests that
matter most here read what the **real writers** actually emit. An earlier
version of this module parsed YAML itself and was tested against the bundle
files that happened to be committed; all of those passed while
``segment-map``'s and ``save-zone``'s output did not parse at all, because no
committed file had that shape. The corpus is not the contract: the writers are.
"""

import gzip
import json
import struct
import zlib
from pathlib import Path

import pytest
import yaml

from mote_bringup import bundle

REPO = Path(__file__).resolve().parents[2]


# ---- what the real writers emit -----------------------------------------
#
# Every one of these calls the actual production writer and reads the file back
# through the validator's own reader. They are the regression test for the
# hand-rolled parser this module used to carry.


def test_segment_map_output_is_a_bundle_this_can_read(tmp_path):
    """`pixi run segment-map --write` (#69) proposes polygon zones.

    Its writer emits a block sequence of flow pairs for the vertex list — the
    exact shape the hand-rolled parser rejected, which made the whole path
    segment a floor -> publish-map fail with a 422.
    """
    from mote_bringup.map_cleanup.room_segmentation import Room
    from mote_bringup.map_cleanup.rooms_cli import merge_into_zones

    rooms = [
        Room(
            name="room_1",
            polygon=[(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)],
            pose=(1.25, 3.4),
            area_m2=12.0,
            clearance_m=1.2,
        )
    ]
    added, _ = merge_into_zones(tmp_path, rooms)
    assert added == ["room_1"]

    zones = bundle.read_floor(tmp_path)["zones"]
    assert zones["room_1"]["polygon"][2] == [4.0, 3.0]
    assert zones["room_1"]["source"] == "segment-map"
    text = (tmp_path / bundle.ZONES_YAML).read_text()
    assert bundle.load_yaml(text) == yaml.safe_load(text)


def test_a_file_keeps_the_source_it_carries(tmp_path):
    """What made a zone survives the reader, and an unrecognised value does not.

    ``source`` says what put the coordinate there — ``save-zone``,
    ``segment-map`` or the dashboard's ``editor``. Nothing decides anything from
    it, which is why a value outside the three is dropped rather than costing
    the floor its map; and why a zone that says nothing carries nothing rather
    than a default that would read as a claim.
    """
    (tmp_path / bundle.ZONES_YAML).write_text(
        "zones:\n"
        "  the kitchen: {x: 1.0, y: 2.0, source: editor}\n"
        "  office: {x: 3.0, y: 4.0}\n"
        "  yard: {x: 5.0, y: 6.0, source: surveyed}\n"
    )
    zones = bundle.read_floor(tmp_path, "home", "ground")["zones"]
    assert zones["the kitchen"]["source"] == "editor"
    assert "source" not in zones["office"]
    assert "source" not in zones["yard"]


@pytest.mark.parametrize(
    "name",
    [
        "Café",  # safe_dump escapes it; the old reader returned Caf\xE9 silently
        "Matron's office",  # single-quoted by the dumper; the old reader raised
        "ward east",
    ],
)
def test_a_zone_name_survives_the_round_trip(tmp_path, name):
    path = tmp_path / "zones.yaml"
    path.write_text(
        yaml.safe_dump(
            {"zones": {name: {"x": 1.0, "y": 2.0, "yaw": 0.0}}},
            sort_keys=False,
            default_flow_style=None,
        )
    )
    assert name in bundle.read_zones(path)["zones"]


def test_a_saved_revision_validates(tmp_path, monkeypatch):
    """sites.save_map writes site.yaml/meta.yaml; the server reads them."""
    import sys

    monkeypatch.setenv("MOTE_HOME", str(tmp_path / "home"))
    sys.path.insert(0, str(REPO / "mote_bringup"))
    from mote_bringup import sites

    sites.create("home", "ground")
    floor = sites.floor_dir("home", "ground")
    rev = floor / "maps" / "20260728T090412"
    revision(rev)
    site_yaml = floor.parents[1] / "site.yaml"
    assert bundle.load_yaml(site_yaml.read_text())["name"] == "home"
    assert bundle.validate(rev, require_posegraph=False).ok


def test_not_yaml_at_all_is_a_bundle_error(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text("zones: [1, 2\n")
    with pytest.raises(bundle.BundleError, match="not valid YAML"):
        bundle.read_zones(path)


@pytest.mark.parametrize(
    "written",
    [
        "2026-07-05T11:16:46",  # what an rsync-seeded meta.yaml carries
        "2026-07-05 11:16:46",  # YAML's other spelling, also implicitly typed
        "2026-07-05T11:16:46Z",
        "2026-07-05",  # resolves to a date, not a datetime
    ],
)
def test_an_unquoted_timestamp_is_read_as_the_text_it_was_written_as(tmp_path, written):
    """A bundle's values are served as JSON, and ``datetime`` is the one thing
    ``safe_load`` returns that ``json.dumps`` refuses. Quoting cannot be
    assumed: only ``sites.save_map`` goes through ``safe_dump``, and a
    revision may be hand-edited or predate the registry.
    """
    path = tmp_path / "meta.yaml"
    path.write_text(f"schema: 1\nsaved: {written}\n")
    assert bundle.load_yaml_file(path)["saved"] == written


def test_a_revision_report_is_json_serialisable_whatever_the_meta_says(tmp_path):
    directory = revision(tmp_path / "20260705T111531")
    (directory / "meta.yaml").write_text("schema: 1\nsaved: 2026-07-05T11:16:46\n")
    report = bundle.validate(directory)
    assert report.ok, report.errors
    assert json.loads(json.dumps(report.as_dict()))["meta"]["saved"] == (
        "2026-07-05T11:16:46"
    )


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


def png(path, width, height, values, colour=0, depth=8, **kwargs):
    """A PNG assembled byte by byte rather than by an image library.

    Worth keeping now that Pillow does the reading: these fixtures are the only
    way to hand the validator a *deliberately broken* image — a bad scanline
    filter, a header claiming more pixels than the data holds — which is what a
    truncated upload looks like and what the server has to answer for.
    """

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    marker = bytes([kwargs.get("filter_type", 0)])
    if kwargs.get("blank"):
        # A header that claims a huge image, with one row of data behind it:
        # the shape of a decompression bomb, without writing one to disk.
        raw = marker + bytes(1)
    else:
        raw = b"".join(
            marker + bytes(values(x, y) for x in range(width)) for y in range(height)
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


def test_a_palette_png_is_counted_rather_than_skipped(tmp_path):
    """The hand-rolled decoder gave up on palette images and warned. Pillow
    resolves the palette, so a map saved in one is checked like any other —
    the degeneracy check covers more revisions than it used to, not fewer."""
    path = tmp_path / "map.png"
    png(path, 4, 4, lambda x, y: 254, depth=8, colour=3)
    counts = bundle.occupancy(path)
    assert "reason" not in counts
    assert counts["total"] == 16


def test_a_broken_png_is_reported_not_raised(tmp_path):
    """validate() promises never to raise; reading pixels must keep that
    promise. An upload whose map.png carries an invalid scanline filter byte
    used to reach the HTTP handler as an exception, which meant no response at
    all and an audit row wedged at 'receiving'."""
    path = tmp_path / "map.png"
    png(path, 4, 4, lambda x, y: 254, filter_type=9)
    counts = bundle.occupancy(path)
    assert counts["corrupt"]

    rev = revision(tmp_path / "20260727T101500")
    png(rev / "map.png", 20, 20, lambda x, y: 254, filter_type=9)
    report = bundle.validate(rev)  # must not raise
    assert not report.ok
    assert any("readable PNG" in e for e in report.errors)


def test_something_that_is_not_an_image_at_all(tmp_path):
    path = tmp_path / "map.png"
    path.write_bytes(b"this is not a PNG")
    counts = bundle.occupancy(path)
    assert counts["corrupt"]
    assert bundle.png_size(path) is None


def test_a_decompression_bomb_is_refused(tmp_path):
    """A PNG whose header claims more pixels than any building has.

    Pillow enforces this with MAX_IMAGE_PIXELS, which bundle.py sets; the
    hand-rolled decoder this replaced had to grow the bound after review found
    a 261 KB upload that inflated to 786 MB.
    """
    path = tmp_path / "map.png"
    png(path, 40_000, 40_000, lambda x, y: 254, blank=True)
    counts = bundle.occupancy(path)
    assert "bomb" in counts["reason"]
    assert counts["corrupt"]


def test_a_map_that_fails_the_extent_check_is_not_then_decoded(tmp_path):
    """The sanity check has already failed; decoding a million pixels in pure
    python afterwards only spends time on a revision that is being refused."""
    rev = revision(
        tmp_path / "20260727T101500",
        map_yaml="image: map.png\nmode: trinary\nresolution: 5000\n"
        "origin: [0, 0, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
    )
    report = bundle.validate(rev)
    assert not report.ok
    assert any("check the resolution" in e for e in report.errors)
    assert report.occupancy is None


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
        # A revision carries a copy of the floor's zones, so that a floor's
        # places reach a robot which has never driven there.
        bundle.write_floor(
            directory,
            {
                "frame_id": "map",
                "revision": 1,
                "zones": {"a": {"name": "a", "x": 0.0, "y": 0.0, "yaw": 0.0}},
            },
            site="home",
            floor="ground",
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
    # One entry naming both halves, not one per file. slam_toolbox writes the
    # posegraph and its data as a pair, so a line each produced two warnings
    # with word-for-word identical text — two problems, to anyone reading them.
    assert lenient.warnings == [
        "map.posegraph and map.data are missing — mapping cannot be continued "
        "in this frame (extend, don't remap)"
    ]


def test_half_a_posegraph_names_only_the_half_that_is_missing(tmp_path):
    directory = revision(tmp_path / "rev")
    (directory / "map.data").unlink()
    report = bundle.validate(directory, require_posegraph=False)
    assert report.warnings == [
        "map.data is missing — mapping cannot be continued in this frame "
        "(extend, don't remap)"
    ]


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
    size that differs means every zone bound on this floor is suspect."""
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


def test_extra_files_travel_with_the_revision(tmp_path):
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

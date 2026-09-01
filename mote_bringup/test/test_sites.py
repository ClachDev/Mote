"""Filesystem-level tests for the site bundle library (no ROS graph needed)."""

import os

import pytest
import yaml

from mote_bringup import sites


@pytest.fixture(autouse=True)
def mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    return tmp_path


def make_revision(fdir, rev, marker=b""):
    rev_dir = fdir / "maps" / rev
    rev_dir.mkdir(parents=True)
    (rev_dir / "map.png").write_bytes(b"P" + marker)
    (rev_dir / "map.yaml").write_text("image: map.png\n")
    return rev_dir


def test_create_seeds_floor_and_activates(mote_home):
    sites.create("home")
    assert sites.active() == ("home", "ground")
    fdir = sites.floor_dir("home", "ground")
    # A new floor has no zone documents at all. Seeding empty ones would make a
    # hand-written zones.yaml dropped in beside them ambiguous, and the empty
    # pair would win silently.
    assert fdir.is_dir()
    assert sorted(path.name for path in fdir.iterdir()) == []
    assert sites.has_zones(fdir) is False
    meta = yaml.safe_load((sites.site_dir("home") / "site.yaml").read_text())
    assert meta == {"schema": 1, "name": "home", "default_floor": "ground"}


def test_create_does_not_steal_active(mote_home):
    sites.create("home")
    sites.create("office", floor="mezzanine")
    assert sites.active() == ("home", "ground")
    sites.use("office")
    assert sites.active() == ("office", "mezzanine")


def test_add_floor_and_use_floor(mote_home):
    sites.create("home")
    sites.add_floor("upstairs")
    assert sites.active() == ("home", "upstairs")
    assert sites.floors("home") == ["ground", "upstairs"]
    sites.use("home", "ground")
    assert sites.active() == ("home", "ground")


def test_use_rejects_unknown(mote_home):
    sites.create("home")
    with pytest.raises(SystemExit):
        sites.use("nowhere")
    with pytest.raises(SystemExit):
        sites.use("home", "attic")


def test_resolve_requires_published_revision(mote_home):
    assert sites.resolve_map() == ""
    sites.create("home")
    fdir = sites.floor_dir("home", "ground")
    make_revision(fdir, "r1")
    assert sites.resolve_map() == ""
    sites._publish_revision(fdir, "r1")
    assert sites.resolve_map() == str(fdir / "map" / "map.yaml")
    assert yaml.safe_load(open(sites.resolve_map()))["image"] == "map.png"


def test_use_map_flips_atomically_and_validates(mote_home):
    sites.create("home")
    fdir = sites.floor_dir("home", "ground")
    make_revision(fdir, "r1", b"1")
    make_revision(fdir, "r2", b"2")
    sites._publish_revision(fdir, "r2")
    assert sites.current_revision(fdir) == "r2"
    sites.use_map("r1")
    assert sites.current_revision(fdir) == "r1"
    assert (fdir / "map" / "map.png").read_bytes() == b"P1"
    with pytest.raises(SystemExit):
        sites.use_map("r9")


def test_prune_keeps_newest_and_current(mote_home):
    sites.create("home")
    fdir = sites.floor_dir("home", "ground")
    for rev in ("r1", "r2", "r3", "r4", "r5"):
        make_revision(fdir, rev)
    sites._publish_revision(fdir, "r1")
    sites._prune_revisions(fdir)
    assert sites.revisions(fdir) == ["r1", "r3", "r4", "r5"]
    assert sites.current_revision(fdir) == "r1"


def test_resolve_zones_and_write_target(mote_home):
    assert sites.resolve_zones() == ""
    with pytest.raises(SystemExit):
        sites.zones_for_write()
    sites.create("home")
    expected = sites.floor_dir("home", "ground")
    # Nothing to resolve until something is taught — but that is where it goes.
    assert sites.resolve_zones() == ""
    assert sites.zones_for_write() == expected

    from mote_tasks.zones import append_zone

    append_zone(expected, "bench", 1.0, 2.0, 0.0, site="home", floor="ground")
    # A floor, not a file: its zones are two documents, and which one a reader
    # wants is the reader's business.
    assert sites.resolve_zones() == str(expected)


def test_dangling_active_is_ignored(mote_home):
    sites.create("home")
    sites.set_active("home", "vanished")
    assert sites.active() is None
    assert sites.resolve_map() == ""


def test_latest_mapping_bag_wants_recent_activity(mote_home):
    assert sites.latest_mapping_bag() is None
    old = sites.bags_dir("mapping") / "20260101_000000"
    old.mkdir(parents=True)
    (old / "seg_0.mcap").write_bytes(b"x")
    os.utime(old / "seg_0.mcap", (0, 0))
    os.utime(old, (0, 0))
    assert sites.latest_mapping_bag() is None
    fresh = sites.bags_dir("mapping") / "20260101_000001"
    fresh.mkdir()
    (fresh / "seg_0.mcap").write_bytes(b"x")
    assert sites.latest_mapping_bag() == fresh


def test_revision_meta_round_trip(mote_home):
    sites.create("home")
    fdir = sites.floor_dir("home", "ground")
    rev_dir = make_revision(fdir, "r1")
    assert sites.revision_meta(fdir, "r1") == {}
    (rev_dir / "meta.yaml").write_text(
        yaml.safe_dump({"schema": 1, "bag": "bags/mapping/20260101_000001"})
    )
    assert sites.revision_meta(fdir, "r1")["bag"] == "bags/mapping/20260101_000001"


def _stage_raw_map(rev_dir):
    """Write a synthetic ROS occupancy PNG + yaml as if map_saver just ran."""
    import cv2
    import numpy as np

    m = np.full((100, 140), 205, np.uint8)  # unknown
    m[20:80, 20:120] = 254  # free room
    m[20:80, 20] = 0  # walls (axis-aligned so a direction is detectable)
    m[20:80, 119] = 0
    m[20, 20:120] = 0
    m[79, 20:120] = 0
    rng = np.random.default_rng(0)
    ys, xs = rng.integers(21, 79, 40), rng.integers(21, 119, 40)
    m[ys, xs] = 0  # speckle clutter to declutter
    rev_dir.mkdir(parents=True)
    cv2.imwrite(str(rev_dir / "map.png"), m)
    (rev_dir / "map.yaml").write_text(
        "image: map.png\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )


def test_promote_cleaned_serves_clean_and_keeps_raw(mote_home):
    import cv2

    fdir = mote_home / "f"
    rev_dir = fdir / "maps" / "r1"
    _stage_raw_map(rev_dir)
    clean = sites.promote_cleaned(rev_dir)

    assert clean["ok"] and clean["removed"] > 0
    # raw retained, cleaned promoted to the served map.png, diagnostics written
    assert (rev_dir / "map_raw.png").exists()
    assert (rev_dir / "diagnostics.png").exists()
    assert "map_raw.png" in (rev_dir / "map_raw.yaml").read_text()
    assert "image: map.png" in (rev_dir / "map.yaml").read_text()  # frame untouched
    raw = cv2.imread(str(rev_dir / "map_raw.png"), cv2.IMREAD_GRAYSCALE)
    served = cv2.imread(str(rev_dir / "map.png"), cv2.IMREAD_GRAYSCALE)
    assert raw.shape == served.shape  # same frame => zones stay valid
    assert (raw != served).any()  # served is cleaned, not the raw bytes


def test_promote_cleaned_failure_falls_back_to_raw(mote_home):
    fdir = mote_home / "f"
    rev_dir = fdir / "maps" / "r1"
    rev_dir.mkdir(parents=True)
    (rev_dir / "map.png").write_text("not a png")  # cv2 cannot read -> failure
    (rev_dir / "map.yaml").write_text("image: map.png\nresolution: 0.05\n")
    clean = sites.promote_cleaned(rev_dir)

    assert clean["ok"] is False and "error" in clean
    assert (rev_dir / "map.png").read_bytes() == (rev_dir / "map_raw.png").read_bytes()


def test_cli_round_trip(mote_home, capsys):
    sites.main(["create", "beta"])
    sites.main(["create", "alpha"])
    sites.main(["use", "alpha"])
    sites.main(["list"])
    out = capsys.readouterr().out
    assert "alpha/ground *" in out
    assert "beta/ground" in out

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
    assert yaml.safe_load((fdir / "zones.yaml").read_text()) == {
        "frame_id": "map",
        "zones": {},
    }
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
    expected = sites.floor_dir("home", "ground") / "zones.yaml"
    assert sites.resolve_zones() == str(expected)
    assert sites.zones_for_write() == expected


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


def test_cli_round_trip(mote_home, capsys):
    sites.main(["create", "beta"])
    sites.main(["create", "alpha"])
    sites.main(["use", "alpha"])
    sites.main(["list"])
    out = capsys.readouterr().out
    assert "alpha/ground *" in out
    assert "beta/ground" in out

"""The robot's half of map distribution, against a real fleet server.

This is M4's acceptance without SLAM and without a broker: a robot's saved
revision is published to the registry, an operator promotes it, and a *second*
robot — a different ``MOTE_HOME`` — pulls the canonical revision and ends up
with the map staged, published, and the zones that were taught in its frame.

No ROS here. ``mapsync`` is deliberately ROS-free so the whole flow can be
exercised as function calls; the agent's use of it (subscribe, queue, worker
thread) is ``test_agent.py``.
"""

import pathlib

import pytest
from api_harness import enroll, get, post, post_bytes, write_revision

from mote_bringup import bundle, mote_home, sites
from mote_fleet import mapsync, protocol

SITE, FLOOR = "home", "ground"
REVISION = "20260727T101500"


@pytest.fixture
def robot_home(tmp_path, monkeypatch):
    """A robot's ``MOTE_HOME``, empty. Every site path resolves through it."""
    home = tmp_path / "robot-home"
    home.mkdir()
    monkeypatch.setenv("MOTE_HOME", str(home))
    assert mote_home.mote_dir() == home
    return home


def publish_and_promote(server, tmp_path, operator, revision=REVISION, **kwargs):
    """A revision on the server, canonical, as if a robot had published it."""
    enroll(server, "serial:aaa", name="Scout")
    directory = write_revision(tmp_path / f"src-{revision}", **kwargs)
    blob = bundle.pack(directory)
    post_bytes(
        server,
        f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/{revision}?robot_id=mote-01",
        blob,
    )
    _, promoted = post(
        server,
        f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/{revision}/promote",
        {"schema": protocol.SCHEMA},
        token=operator,
    )
    return promoted


def announcement(promoted) -> dict:
    """What the agent would receive on the retained topic."""
    return protocol.current(
        promoted["site"],
        promoted["floor"],
        promoted["revision"],
        url=promoted["url"],
        sha256=promoted["sha256"],
        bytes_=promoted["bytes"],
        promoted_by=promoted["promoted_by"],
    )


# ---- pulling ------------------------------------------------------------


def test_a_robot_pulls_the_canonical_revision_and_publishes_it(
    server, operator, robot_home, tmp_path
):
    sites.create(SITE, FLOOR)  # this robot is on that floor
    promoted = publish_and_promote(server, tmp_path, operator)
    result = mapsync.pull(server.url, announcement(promoted))
    assert result["action"] == "installed"

    floor_dir = sites.floor_dir(SITE, FLOOR)
    assert sites.current_revision(floor_dir) == REVISION
    assert (floor_dir / "map" / "map.yaml").is_file()
    # And it is a map the robot can actually use, not just files that arrived:
    # this is the path nav2_launch.py resolves at launch time.
    assert bundle.validate(floor_dir / "maps" / REVISION).ok
    # The path nav2_launch.py resolves at launch time — through the symlink,
    # so a later revision is picked up by a restart and not by editing config.
    resolved = pathlib.Path(sites.resolve_map())
    assert resolved.name == "map.yaml"
    assert resolved.resolve().parent.name == REVISION


def test_pulling_the_revision_it_is_already_on_does_nothing(
    server, operator, robot_home, tmp_path
):
    promoted = publish_and_promote(server, tmp_path, operator)
    mapsync.pull(server.url, announcement(promoted))
    assert mapsync.pull(server.url, announcement(promoted))["action"] == "current"


def test_a_revision_already_on_disk_is_flipped_rather_than_downloaded(
    server, operator, robot_home, tmp_path
):
    """The publishing robot's own case: it already has the bytes, so promotion
    costs it one symlink and no transfer."""
    promoted = publish_and_promote(server, tmp_path, operator)
    mapsync.pull(server.url, announcement(promoted))
    sites._publish_revision(sites.floor_dir(SITE, FLOOR), REVISION)
    older = sites.floor_dir(SITE, FLOOR) / "maps" / "20260101T000000"
    write_revision(older)
    sites._publish_revision(sites.floor_dir(SITE, FLOOR), older.name)

    server.shutdown()  # nothing may be fetched
    assert mapsync.pull(server.url, announcement(promoted))["action"] == "flipped"
    assert sites.current_revision(sites.floor_dir(SITE, FLOOR)) == REVISION


def test_zones_arrive_with_the_map_and_the_old_ones_are_kept(
    server, operator, robot_home, tmp_path
):
    """A revision from another mapping session is another map frame, so the
    zones taught in the old one are wrong the moment it is published — but
    losing every taught place silently is not acceptable either."""
    floor_dir = sites.floor_dir(SITE, FLOOR)
    floor_dir.mkdir(parents=True)
    (floor_dir / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  old: {x: 0, y: 0}\n"
    )
    promoted = publish_and_promote(server, tmp_path, operator)
    mapsync.pull(server.url, announcement(promoted))

    zones = bundle.read_zones(floor_dir / "zones.yaml")["zones"]
    assert set(zones) == {"kitchen", "sluice", "ward"}
    kept = list(floor_dir.glob("zones.*.yaml"))
    assert len(kept) == 1
    assert "old" in bundle.read_zones(kept[0])["zones"]


def test_a_download_that_is_not_what_was_announced_is_refused(
    server, operator, robot_home, tmp_path
):
    promoted = publish_and_promote(server, tmp_path, operator)
    lie = announcement(promoted)
    lie["sha256"] = "sha256:" + "0" * 64
    with pytest.raises(mapsync.SyncError, match="announced as"):
        mapsync.pull(server.url, lie)
    assert not sites.floor_dir(SITE, FLOOR).exists()


def test_a_server_that_is_not_there_is_an_error_not_a_half_install(
    robot_home, operator
):
    with pytest.raises(mapsync.SyncError):
        mapsync.pull(
            "http://127.0.0.1:1",
            {"site": SITE, "floor": FLOOR, "revision": REVISION, "url": "/v1/x"},
        )
    assert not sites.floor_dir(SITE, FLOOR).exists()


def test_an_unusable_revision_from_the_fleet_is_refused(robot_home, tmp_path):
    """Belt and braces: the server validated it on the way in, and the robot
    checks again on the way out of the tarball, because what it is about to do
    is replace the map it navigates with."""
    directory = write_revision(tmp_path / "broken")
    (directory / "map.yaml").write_text("image: map.png\n")  # no resolution/origin
    with pytest.raises(bundle.BundleError):
        sites.install_revision(SITE, FLOOR, REVISION, bundle.pack(directory))
    assert sites.current_revision(sites.floor_dir(SITE, FLOOR)) is None


# ---- which floors a robot cares about -----------------------------------


def test_a_robot_pulls_its_own_floor(robot_home):
    sites.create(SITE, FLOOR)
    assert mapsync.wants({"site": SITE, "floor": FLOOR}, sites.active())


def test_a_robot_pulls_a_floor_it_already_holds(robot_home):
    """So a robot that moves between two floors keeps both current instead of
    re-downloading on arrival."""
    sites.create(SITE, FLOOR)
    sites.add_floor("first")
    sites.use(SITE, FLOOR)
    assert mapsync.wants({"site": SITE, "floor": "first"}, sites.active())


def test_a_robot_ignores_a_floor_of_a_building_it_has_never_been_in(robot_home):
    sites.create(SITE, FLOOR)
    assert not mapsync.wants(
        {"site": "warehouse", "floor": "mezzanine"}, sites.active()
    )


# ---- publishing ---------------------------------------------------------


def test_publishing_packs_the_floors_zones_into_the_revision(
    server, robot_home, tmp_path
):
    """Zones live at floor level but must travel inside the revision, because
    they are coordinates in that revision's frame."""
    enroll(server, "serial:ccc", name="Scout")
    floor_dir = sites.floor_dir(SITE, FLOOR)
    write_revision(floor_dir / "maps" / REVISION, zones=False)
    (floor_dir / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  bay: {x: 1.5, y: -2.0}\n"
    )
    sites._publish_revision(floor_dir, REVISION)

    answer = mapsync.publish(server.url, SITE, FLOOR, REVISION, "mote-01")
    assert answer["revision"] == REVISION
    _, zones = get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json")
    # Still the floor's original map: uploading is not publishing.
    assert zones["zones"][0]["name"] == "kitchen"
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    uploaded = next(r for r in floor["revisions"] if r["revision"] == REVISION)
    assert uploaded["zones"] == ["bay"]


def test_publishing_a_split_floor_sends_the_binding_in_the_revision(
    server, robot_home, tmp_path
):
    """The zone/v0 layout, end to end.

    The coordinates go *inside* the revision, because they are only meaningful
    in that revision's map frame. The names ride along too — but an upload is
    inert, and that now covers names as well as coordinates: until an operator
    promotes, ``/v1/zones`` still answers with the floor's published
    vocabulary. Otherwise a robot could rename every room on a floor its
    neighbours are driving, by uploading a map nobody accepted.
    """
    enroll(server, "serial:ddd", name="Scout")
    floor_dir = sites.floor_dir(SITE, FLOOR)
    write_revision(floor_dir / "maps" / REVISION, zones=False)
    # Written with `bundle`, not with `mote_tasks.zones`: these tests run in the
    # ROS-free `fleet` environment, and reaching for the task layer's writer
    # here would be the seam the split exists to keep — the robot's half needs
    # ROS, the fleet's half must never.
    bundle.write_floor(
        floor_dir,
        {
            "frame_id": "map",
            "revision": 1,
            "zones": {
                "bay": {
                    "name": "bay",
                    "kind": "dock",
                    "display_name": "",
                    "aliases": [],
                    "navigable": True,
                    "parent": None,
                    "tags": [],
                    "description": "",
                    "bound": True,
                    "x": 1.5,
                    "y": -2.0,
                    "yaw": 0.0,
                }
            },
        },
        site=SITE,
        floor=FLOOR,
        platform_id="mote-01",
    )
    assert (floor_dir / "binding.yaml").is_file()
    sites._publish_revision(floor_dir, REVISION)

    mapsync.publish(server.url, SITE, FLOOR, REVISION, "mote-01")
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    uploaded = next(r for r in floor["revisions"] if r["revision"] == REVISION)
    assert uploaded["zones"] == ["bay"]

    _, vocabulary = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    assert "bay" not in [item["name"] for item in vocabulary["zones"]]
    # ...and what it does serve is names and nothing else.
    assert vocabulary["zones"]
    for item in vocabulary["zones"]:
        assert not {"x", "y", "yaw", "radius", "polygon"} & set(item)


def test_publishing_a_revision_the_robot_does_not_have_is_refused(server, robot_home):
    with pytest.raises(mapsync.SyncError, match="no revision"):
        mapsync.publish(server.url, SITE, FLOOR, "20990101T000000", "mote-01")


def test_publishing_an_unusable_revision_never_leaves_the_robot(
    server, robot_home, tmp_path
):
    floor_dir = sites.floor_dir(SITE, FLOOR)
    write_revision(floor_dir / "maps" / REVISION, posegraph=False)
    with pytest.raises(mapsync.SyncError, match="refusing to publish"):
        mapsync.publish(server.url, SITE, FLOOR, REVISION, "mote-01")

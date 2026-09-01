import math

import yaml

import pytest

from mote_tasks.zones import (
    Polygon,
    append_zone,
    containing,
    load_zones,
    resolve,
    yaw_from_quaternion,
)

# An L: the unit-ish square [0, 4] x [0, 4] with its north-east quadrant
# removed, so the area centroid (1.667, 1.667) is inside but a naive bounding
# box or circle would claim the missing corner.
L_SHAPE = Polygon(((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)))

# A U opening north: the centroid falls in the notch, outside the polygon.
U_SHAPE = Polygon(((0, 0), (6, 0), (6, 6), (4, 6), (4, 2), (2, 2), (2, 6), (0, 6)))


def test_yaw_from_quaternion_round_trip():
    for yaw in (-3.0, -1.5, 0.0, 0.7, 3.1):
        z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        assert yaw_from_quaternion(0.0, 0.0, z, w) == pytest.approx(yaw)


def test_append_zone_creates_replaces_and_loads(tmp_path):
    """Teaching writes the zone/v0 pair, and reading joins it back."""
    assert append_zone(tmp_path, "bin", 1.0, -2.0, 0.5) is False
    assert append_zone(tmp_path, "sofa", 0.25, 0.0, -1.0) is False
    assert append_zone(tmp_path, "bin", 1.5, -2.5, 0.5) is True

    # The names went one way and the coordinates the other. This is the whole
    # split, checked over the text rather than over the keys someone thought
    # of: a coordinate in the shared document is the leak it exists to stop.
    shared = (tmp_path / "vocabulary.yaml").read_text()
    assert "bin" in shared and "sofa" in shared
    for leak in ("x:", "y:", "yaw:", "radius:", "polygon:", "frame_id:"):
        assert leak not in shared, f"{leak} leaked into the vocabulary"
    assert "x: 1.5" in (tmp_path / "binding.yaml").read_text()

    zones = load_zones(tmp_path)
    assert sorted(zones) == ["bin", "sofa"]
    assert zones["bin"].pose.pose.position.x == pytest.approx(1.5)
    assert zones["bin"].pose.header.frame_id == "map"
    assert zones["bin"].footprint is None
    yaw = yaw_from_quaternion(
        0.0,
        0.0,
        zones["sofa"].pose.pose.orientation.z,
        zones["sofa"].pose.pose.orientation.w,
    )
    assert yaw == pytest.approx(-1.0)


def test_append_zone_with_radius_gives_footprint(tmp_path):
    append_zone(tmp_path, "kitchen", 2.0, 3.0, 0.0, radius=1.5)
    zones = load_zones(tmp_path)
    fp = zones["kitchen"].footprint
    assert fp is not None
    assert fp.radius == pytest.approx(1.5)
    assert fp.contains(2.4, 3.0) is True  # 0.4 m from centre
    assert fp.contains(4.0, 3.0) is False  # 2.0 m from centre


def test_load_zones_missing_coord_raises(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text("zones:\n  bad: {x: 1.0}\n")
    with pytest.raises(ValueError):
        load_zones(str(path))


def test_containing_nearest_first(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  pickup: {x: 0.0, y: 0.0}\n"  # no footprint -> never matches
        "  big:    {x: 0.0, y: 0.0, radius: 5.0}\n"
        "  small:  {x: 1.0, y: 0.0, radius: 2.0}\n"
        "  far:    {x: 20.0, y: 20.0, radius: 1.0}\n"
    )
    zones = load_zones(str(path))
    # (1.2, 0) is inside big (dist 1.2) and small (dist 0.2), nearest first;
    # pickup has no footprint so it never appears.
    assert containing(zones, 1.2, 0.0) == ["small", "big"]
    assert containing(zones, 10.0, 0.0) == []  # outside every footprint
    assert containing(zones, 0.0, 5.0) == ["big"]  # exactly on the radius counts


def test_polygon_contains_convex():
    square = Polygon(((0, 0), (2, 0), (2, 2), (0, 2)))
    assert square.contains(1.0, 1.0) is True
    assert square.contains(3.0, 1.0) is False
    assert square.contains(1.0, -0.001) is False
    assert square.contains(2.0, 1.0) is True  # on an edge
    assert square.contains(0.0, 0.0) is True  # on a vertex


def test_polygon_contains_concave():
    # The removed north-east quadrant is outside even though it is well inside
    # the bounding box and inside a circle drawn round the shape.
    assert L_SHAPE.contains(1.0, 1.0) is True
    assert L_SHAPE.contains(3.0, 1.0) is True
    assert L_SHAPE.contains(1.0, 3.0) is True
    assert L_SHAPE.contains(3.0, 3.0) is False
    assert L_SHAPE.contains(2.5, 2.5) is False


def test_polygon_winding_order_does_not_matter():
    reversed_l = Polygon(tuple(reversed(L_SHAPE.vertices)))
    for px, py in ((1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (2.5, 2.5)):
        assert reversed_l.contains(px, py) == L_SHAPE.contains(px, py)


def test_representative_point_is_inside_a_u_shape():
    cx, cy = U_SHAPE.centroid()
    assert U_SHAPE.contains(cx, cy) is False  # the centroid sits in the notch
    rx, ry = U_SHAPE.representative_point()
    assert U_SHAPE.contains(rx, ry) is True


def test_polygon_zone_loads_and_contains(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  ward: {x: 3.0, y: 0.5, yaw: 1.5,\n"
        "    polygon: [[0, 0], [4, 0], [4, 2], [2, 2], [2, 4], [0, 4]]}\n"
    )
    zones = load_zones(str(path))
    ward = zones["ward"]
    assert ward.pose.pose.position.x == pytest.approx(3.0)  # the taught pose wins
    assert ward.footprint.contains(3.0, 1.0) is True
    assert ward.footprint.contains(3.0, 3.0) is False
    assert containing(zones, 3.0, 3.0) == []
    assert containing(zones, 3.0, 1.0) == ["ward"]


def test_polygon_only_zone_derives_a_pose_inside_the_outline(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  hall: {polygon: [[0, 0], [6, 0], [6, 6], [4, 6], [4, 2], [2, 2], [2, 6], [0, 6]]}\n"
    )
    zones = load_zones(str(path))
    p = zones["hall"].pose.pose.position
    assert zones["hall"].footprint.contains(p.x, p.y) is True


def test_polygon_beats_radius_when_a_zone_carries_both(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  ward: {x: 1.0, y: 1.0, radius: 10.0,\n"
        "    polygon: [[0, 0], [2, 0], [2, 2], [0, 2]]}\n"
    )
    footprint = load_zones(str(path))["ward"].footprint
    assert isinstance(footprint, Polygon)
    assert footprint.contains(5.0, 5.0) is False  # the radius would have matched


@pytest.mark.parametrize(
    "polygon",
    ["[[0, 0], [1, 1]]", "[[0, 0], [1, 1], [2, 2, 2]]", "5"],
)
def test_load_zones_bad_polygon_raises(tmp_path, polygon):
    path = tmp_path / "zones.yaml"
    path.write_text(f"zones:\n  bad: {{x: 0.0, y: 0.0, polygon: {polygon}}}\n")
    with pytest.raises(ValueError):
        load_zones(str(path))


def test_append_zone_keeps_a_polygon_but_radius_replaces_it(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n  ward: {x: 1.0, y: 1.0, polygon: [[0, 0], [2, 0], [2, 2], [0, 2]]}\n"
    )
    # Re-teaching the pose must not silently un-room the zone. It is also the
    # first write to a combined file, so it is the migration: nobody runs one,
    # and nobody can forget to.
    append_zone(path, "ward", 1.5, 0.5, 0.0)
    assert (tmp_path / "vocabulary.yaml").exists()
    assert not (tmp_path / "zones.yaml").exists()
    ward = load_zones(tmp_path)["ward"]
    assert isinstance(ward.footprint, Polygon)
    assert ward.pose.pose.position.x == pytest.approx(1.5)

    # ...but an explicit --radius is a deliberate new footprint.
    append_zone(tmp_path, "ward", 1.5, 0.5, 0.0, radius=3.0)
    assert load_zones(tmp_path)["ward"].footprint.radius == pytest.approx(3.0)


def test_save_zone_output_is_readable_by_the_bundle_validator(tmp_path):
    """`save-zone` writes the file the fleet server has to validate on upload.

    The two live in different packages and are written and read by different
    code, so the round trip is only true if something checks it. It was not:
    the polygon shape this dumper emits did not parse (m4-verification.md §2).
    """
    from mote_bringup import bundle

    append_zone(tmp_path, "kitchen", 1.0, 2.0, 0.5, radius=1.5)
    append_zone(tmp_path, "ward_east", 4.0, 1.0, 0.0)
    binding = yaml.safe_load((tmp_path / "binding.yaml").read_text())
    for item in binding["bindings"]:
        if item["name"] == "ward_east":
            item["footprint"] = {
                "type": "polygon",
                "vertices": [[3.0, 0.0], [5.0, 0.0], [5.0, 2.0]],
            }
    (tmp_path / "binding.yaml").write_text(
        yaml.safe_dump(binding, sort_keys=False, default_flow_style=None)
    )
    append_zone(tmp_path, "ward_east", 4.1, 1.1, 0.0)  # re-teach: keeps the outline

    zones = bundle.read_floor(tmp_path)["zones"]
    assert zones["kitchen"]["radius"] == 1.5
    assert zones["ward_east"]["polygon"][2] == [5.0, 2.0]
    assert load_zones(tmp_path)["ward_east"].footprint is not None


# ---- the vocabulary half (zone/v0) --------------------------------------


def write_zones(tmp_path, body: str):
    path = tmp_path / "zones.yaml"
    path.write_text(f"frame_id: map\nzones:\n{body}")
    return path


def test_a_zone_with_no_vocabulary_is_a_navigable_place(tmp_path):
    """Both fields are optional, so no existing zones.yaml needed rewriting."""
    zone = load_zones(str(write_zones(tmp_path, "  bench: {x: 1.0, y: 2.0}\n")))[
        "bench"
    ]
    assert (zone.note, zone.navigable) == ("", True)
    assert zone.label == "bench"  # the label *is* the name


def test_a_place_name_is_what_a_person_would_write(tmp_path):
    """A zone is a place-name, so the name is the human one — spaces, accents
    and all. There is no second field to put the readable spelling in."""
    path = write_zones(
        tmp_path,
        "  store room: {x: 1.0, y: 2.0, note: 'stationery lives here'}\n"
        '  "Café": {x: 3.0, y: 4.0}\n',
    )
    zones = load_zones(str(path))
    assert sorted(zones) == ["Café", "store room"]
    assert zones["store room"].note == "stationery lives here"
    assert zones["store room"].label == "store room"


def test_a_legacy_description_is_read_as_the_note_it_was(tmp_path):
    path = write_zones(tmp_path, "  office: {x: 1.0, y: 2.0, description: kettle}\n")
    assert load_zones(str(path))["office"].note == "kettle"


def test_the_retired_fields_load_and_are_dropped(tmp_path):
    """A floor taught before place-names must not need re-teaching, and none of
    what it says about itself may survive as vocabulary."""
    path = write_zones(
        tmp_path,
        "  kitchen: {x: 1.0, y: 2.0, kind: room, display_name: The Kitchen,\n"
        "    aliases: [galley], parent: null, tags: [wet]}\n",
    )
    zone = load_zones(str(path))["kitchen"]
    assert zone.label == "kitchen"
    for retired in ("kind", "display_name", "aliases", "parent", "tags"):
        assert not hasattr(zone, retired)


def test_a_legacy_keepout_is_still_not_navigable(tmp_path):
    """The one thing `kind` is still read for. Dropping it outright would turn
    every barrier on every already-taught floor into a destination — silently,
    on the first load after the upgrade."""
    path = write_zones(tmp_path, "  sluice: {x: 1.0, y: 2.0, kind: keepout}\n")
    assert load_zones(str(path))["sluice"].navigable is False


def test_a_navigable_keepout_is_refused_rather_than_honoured(tmp_path):
    path = write_zones(
        tmp_path, "  sluice: {x: 1.0, y: 2.0, kind: keepout, navigable: true}\n"
    )
    with pytest.raises(ValueError, match="not a destination"):
        load_zones(str(path))


def test_a_kind_nothing_recognises_is_ignored_rather_than_refused(tmp_path):
    """It used to be refused against a fixed list. There is no list now, and
    refusing a floor over a field nothing reads would be the wrong price."""
    path = write_zones(tmp_path, "  lounge: {x: 1.0, y: 2.0, kind: snug}\n")
    assert load_zones(str(path))["lounge"].navigable is True


def test_an_ambiguous_vocabulary_is_refused_at_load(tmp_path):
    """zone/v0: a conforming platform rejects a collision rather than picking a
    winner. Loading it anyway would make `goto the kitchen` depend on dict
    order. With one name per zone, a collision is two places called the same.
    """
    path = write_zones(
        tmp_path,
        "  the kitchen: {x: 1.0, y: 2.0}\n  The  Kitchen: {x: 3.0, y: 4.0}\n",
    )
    with pytest.raises(ValueError, match="ambiguous"):
        load_zones(str(path))


def test_resolve_matches_the_name_and_nothing_else(tmp_path):
    """Exactly, then case-insensitively and whitespace-normalised — which is
    what makes a name with a space in it typeable. A retired alias or display
    name reaches nothing: the other spellings a place answers to are the
    mission layer's resolver's job, reading the note."""
    path = write_zones(
        tmp_path,
        "  the kitchen: {x: 1.0, y: 2.0, display_name: The Galley,\n"
        "    aliases: [galley]}\n",
    )
    zones = load_zones(str(path))
    for query in ("the kitchen", "The Kitchen", "the  KITCHEN"):
        assert resolve(zones, query).name == "the kitchen", query
    for query in ("galley", "The Galley", "pantry"):
        assert resolve(zones, query) is None, query


def test_re_teaching_a_pose_keeps_the_vocabulary(tmp_path):
    """Driving somewhere to capture a better pose is a new coordinate, never a
    rename — dropping the note an operator typed would be silent data loss.
    """
    append_zone(tmp_path, "kitchen", 1.0, 2.0, 0.0, radius=1.5, note="the kettle")
    append_zone(tmp_path, "kitchen", 1.2, 2.2, 0.1)
    zone = load_zones(tmp_path)["kitchen"]
    assert zone.note == "the kettle"
    assert zone.pose.pose.position.x == 1.2


def test_teaching_bumps_the_vocabulary_revision(tmp_path):
    """A binding elsewhere records which vocabulary it was built against, so
    the counter has to move whenever a name could have."""

    def revision():
        return yaml.safe_load((tmp_path / "vocabulary.yaml").read_text())["revision"]

    append_zone(tmp_path, "kitchen", 1.0, 2.0, 0.0)
    first = revision()
    append_zone(tmp_path, "ward", 3.0, 4.0, 0.0)
    assert revision() > first
    # The binding says which vocabulary it was built against, which is what
    # makes a stale one detectable rather than merely wrong.
    binding = yaml.safe_load((tmp_path / "binding.yaml").read_text())
    assert binding["vocabulary_revision"] == revision()


def test_teaching_a_place_a_robot_may_not_go_writes_the_flag(tmp_path):
    """`--no-navigable` is what `--kind keepout` was: the fact, rather than a
    taxonomy the fact had to be inferred from."""
    append_zone(tmp_path, "sluice", 1.0, 2.0, 0.0, navigable=False)
    document = yaml.safe_load((tmp_path / "vocabulary.yaml").read_text())
    assert document["zones"][0]["navigable"] is False
    assert load_zones(tmp_path)["sluice"].navigable is False


def test_a_taught_note_travels_in_the_vocabulary_and_not_the_binding(tmp_path):
    """The note is a fact about the building, so it is shared; the pose is a
    coordinate in this robot's frame, so it is not."""
    append_zone(tmp_path, "store room", 1.0, 2.0, 0.0, note="stationery lives here")
    vocabulary = yaml.safe_load((tmp_path / "vocabulary.yaml").read_text())
    assert vocabulary["zones"][0]["note"] == "stationery lives here"
    binding = yaml.safe_load((tmp_path / "binding.yaml").read_text())
    assert "note" not in binding["bindings"][0]

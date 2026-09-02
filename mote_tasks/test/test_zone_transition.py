"""Reading a floor written while zone/v0's vocabulary/binding split stood.

A zone is a coordinate in the floor's frame — a fact about the building — so a
floor's zones are one document and the floor owns it. Floors written before that
hold two: ``vocabulary.yaml`` for the names and ``binding.yaml`` for the poses.
``bundle._read_split_pair`` is the only code in the tree that knows this, and
these are its tests.

What it has to be is *invisible*: a floor in the old layout must load exactly as
the same floor does after any write has rewritten it, or every reader downstream
has two shapes to think about and the transition is not a transition.
"""

import yaml

from mote_bringup import bundle
from mote_bringup.spec import zone as zone_spec

from mote_tasks import zones as mote_zones


def a_split_floor(directory, *, vocabulary, bindings):
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "vocabulary.yaml").write_text(
        yaml.safe_dump(
            zone_spec.vocabulary(
                "acme_hq",
                "ground",
                [zone_spec.term("test", name, entry) for name, entry in vocabulary],
                revision=3,
            ),
            sort_keys=False,
        )
    )
    (directory / "binding.yaml").write_text(
        yaml.safe_dump(
            zone_spec.binding(
                "mote-01",
                "acme_hq",
                "ground",
                bindings,
                map_revision="20260727T101500",
            ),
            sort_keys=False,
        )
    )
    return directory


def a_pair(tmp_path):
    return a_split_floor(
        tmp_path / "floor",
        vocabulary=[
            ("kitchen", {"note": "the good kettle"}),
            ("plant", {"navigable": False}),
        ],
        bindings=[
            zone_spec.bound(
                "kitchen",
                2.0,
                3.5,
                1.57,
                footprint={"type": "circle", "radius": 1.5},
                source=zone_spec.SAVE_ZONE,
            ),
            zone_spec.bound("plant", 1.0, 0.5),
        ],
    )


def test_the_pair_reads_as_the_one_file_it_is_rewritten_to(tmp_path):
    """The acceptance: the old layout and the new one are the same floor.

    Read the pair, teach a zone into it — which is what rewrites it — and read
    it again. Everything the first read said about the zones that were already
    there has to survive verbatim; anything that did not would be a reader
    somewhere getting a different answer depending on when the floor was last
    written.
    """
    floor = a_pair(tmp_path)
    before = bundle.read_floor(floor, "acme_hq", "ground")

    mote_zones.append_zone(
        floor, "office", 8.0, 9.0, 0.0, site="acme_hq", floor="ground"
    )
    after = bundle.read_floor(floor, "acme_hq", "ground")

    assert (floor / bundle.ZONES_YAML).is_file()
    assert not (floor / bundle.VOCABULARY_YAML).is_file()
    assert not (floor / bundle.BINDING_YAML).is_file()
    # ...and the pair is kept rather than deleted: it is the only record of
    # anything the join dropped.
    assert (floor / f"{bundle.VOCABULARY_YAML}.premigration").is_file()
    assert (floor / f"{bundle.BINDING_YAML}.premigration").is_file()

    assert after["frame_id"] == before["frame_id"]
    for name in before["zones"]:
        assert after["zones"][name] == before["zones"][name]


def test_the_zones_a_reader_sees_are_the_same_either_way(tmp_path):
    """The same floor through ``mote_tasks``, which is what a mission uses."""
    floor = a_pair(tmp_path)
    before = mote_zones.load_zones(floor)
    mote_zones.append_zone(
        floor, "office", 8.0, 9.0, 0.0, site="acme_hq", floor="ground"
    )
    after = mote_zones.load_zones(floor)

    assert sorted(before) == ["kitchen", "plant"]
    assert sorted(after) == ["kitchen", "office", "plant"]
    for name, zone in before.items():
        assert after[name].name == zone.name
        assert after[name].note == zone.note
        assert after[name].navigable == zone.navigable
        assert after[name].footprint == zone.footprint
        assert after[name].pose.pose.position.x == zone.pose.pose.position.x
        assert after[name].pose.pose.position.y == zone.pose.pose.position.y


def test_a_name_with_no_coordinate_is_dropped(tmp_path):
    """The split let a floor name a place with nothing saying where it is, and
    a robot answered ``unbound`` for it. A zone is a coordinate in the floor's
    frame, so such a name is not a zone here — it is a name nobody has placed,
    and it is left in the ``.premigration`` copy rather than loaded as a zone
    with no position.
    """
    floor = a_split_floor(
        tmp_path / "floor",
        vocabulary=[("kitchen", {}), ("ward_a", {})],
        bindings=[zone_spec.bound("kitchen", 2.0, 3.5)],
    )
    assert sorted(mote_zones.load_zones(floor)) == ["kitchen"]
    assert mote_zones.resolve(mote_zones.load_zones(floor), "ward_a") is None


def test_a_binding_the_vocabulary_did_not_name_is_an_ordinary_zone(tmp_path):
    """Under the split this was a "local extension" — usable here, never
    advertised. Every zone on the floor is the floor's, so it is a zone."""
    floor = a_split_floor(
        tmp_path / "floor",
        vocabulary=[("kitchen", {})],
        bindings=[
            zone_spec.bound("kitchen", 2.0, 3.5),
            zone_spec.bound("my bench", 1.0, 1.0),
        ],
    )
    zones = mote_zones.load_zones(floor)
    assert sorted(zones) == ["kitchen", "my bench"]
    assert mote_zones.destination(zones, "my bench").name == "my bench"

    published = bundle.vocabulary(bundle.read_floor(floor), "acme_hq", "ground")
    assert sorted(item["name"] for item in published["zones"]) == [
        "kitchen",
        "my bench",
    ]


def test_the_provenance_a_pair_recorded_comes_through(tmp_path):
    """``anchor.method`` was how the split recorded what made a coordinate;
    ``source`` is, and the mapping between them is one dict in ``zone.py``."""
    floor = a_split_floor(
        tmp_path / "floor",
        vocabulary=[("kitchen", {}), ("ward a", {}), ("yard", {})],
        bindings=[
            zone_spec.bound("kitchen", 2.0, 3.5, source=zone_spec.SAVE_ZONE),
            zone_spec.bound("ward a", 4.0, 5.0, source=zone_spec.SEGMENT_MAP),
            zone_spec.bound("yard", 6.0, 7.0, source=zone_spec.EDITOR),
        ],
    )
    zones = mote_zones.load_zones(floor)
    assert zones["kitchen"].source == "save-zone"
    assert zones["ward a"].source == "segment-map"
    assert zones["yard"].source == "editor"


def test_a_floor_with_neither_layout_says_so(tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    try:
        bundle.read_floor(empty)
    except bundle.BundleError as exc:
        assert "zones.yaml" in str(exc)
    else:  # pragma: no cover - the assertion above is the test
        raise AssertionError("an empty floor directory should not read")

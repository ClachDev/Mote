"""The vocabulary/binding split, from the robot's side.

The split's dividend is a distinction the robot could not previously draw:
between a name that means nothing here and a name that means something on this
floor which *this* robot has not been taught. An operator does different things
about them — fix a typo, or drive somewhere and run ``save-zone`` — and before
the split both arrived as "unknown zone", which sent them looking for the typo.

The other half of the dividend is what does *not* travel. Everything published
here is names; everything with a coordinate in it stays on the robot that
measured it. That is checked over the whole document rather than over the keys
someone thought of, because the failure mode is a plausible-looking coordinate
rather than a crash.
"""

import yaml

import pytest

from mote_bringup import bundle
from mote_bringup.spec import zone as zone_spec

from mote_tasks import zones as mote_zones


def a_floor(tmp_path, *, vocabulary, bindings):
    (tmp_path / "vocabulary.yaml").write_text(
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
    (tmp_path / "binding.yaml").write_text(
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
    return tmp_path


def test_a_named_but_untaught_zone_resolves_unbound(tmp_path):
    """The distinction the split exists to make representable."""
    floor = a_floor(
        tmp_path,
        vocabulary=[("kitchen", {}), ("ward_a", {})],
        bindings=[zone_spec.bound("kitchen", 2.0, 3.5)],
    )
    zones = mote_zones.load_floor(floor)
    assert sorted(zones) == ["kitchen", "ward_a"]
    assert zones["ward_a"].bound is False and zones["ward_a"].pose is None

    _, reason = mote_zones.resolve_reason(zones, "ward_a")
    assert reason == zone_spec.UNBOUND
    _, reason = mote_zones.resolve_reason(zones, "nowhere")
    assert reason == zone_spec.UNKNOWN_NAME

    # And the refusal says what to do about it, which is not "check the spelling".
    with pytest.raises(mote_zones.ZoneUnresolved, match="save-zone") as excinfo:
        mote_zones.destination(zones, "ward_a")
    assert excinfo.value.reason == zone_spec.UNBOUND


def test_only_bound_zones_are_drivable_and_containable(tmp_path):
    floor = a_floor(
        tmp_path,
        vocabulary=[("kitchen", {}), ("ward_a", {})],
        bindings=[
            zone_spec.bound(
                "kitchen", 2.0, 3.5, footprint={"type": "circle", "radius": 1.5}
            )
        ],
    )
    zones = mote_zones.load_floor(floor)
    assert sorted(mote_zones.load_zones(floor)) == ["kitchen"]
    # An unbound zone is in no place, so it can contain nothing.
    assert mote_zones.containing(zones, 2.0, 3.5) == ["kitchen"]


def test_a_binding_the_vocabulary_does_not_name_is_a_local_extension(tmp_path):
    """This robot was taught a place nobody has named for the site.

    It stays usable here — refusing it would lose a taught pose over a naming
    gap — and it is left out of the vocabulary, because advertising it would be
    one robot inventing shared vocabulary for its neighbours.
    """
    floor = a_floor(
        tmp_path,
        vocabulary=[("kitchen", {})],
        bindings=[
            zone_spec.bound("kitchen", 2.0, 3.5),
            zone_spec.bound("my_bench", 1.0, 1.0),
        ],
    )
    zones = mote_zones.load_floor(floor)
    assert zones["my_bench"].local is True and zones["my_bench"].bound is True
    assert mote_zones.destination(zones, "my_bench").name == "my_bench"

    published = bundle.vocabulary(bundle.read_floor(floor), "acme_hq", "ground")
    assert [item["name"] for item in published["zones"]] == ["kitchen"]


def test_the_shared_document_carries_no_coordinates(tmp_path):
    floor = a_floor(
        tmp_path,
        vocabulary=[("kitchen", {"note": "the good kettle"})],
        bindings=[
            zone_spec.bound(
                "kitchen", 2.0, 3.5, 1.57, footprint={"type": "circle", "radius": 1.5}
            )
        ],
    )
    shared = (floor / "vocabulary.yaml").read_text()
    assert "the good kettle" in shared
    for leak in zone_spec.GEOMETRY_KEYS + ("frame_id", "map_revision", "pose"):
        assert f"{leak}:" not in shared, f"{leak} leaked into the vocabulary"

    # ...and the private one carries the three things that say what its numbers
    # are only true against.
    private = yaml.safe_load((floor / "binding.yaml").read_text())
    assert private["platform_id"] == "mote-01"
    assert private["frame_id"] == "map"
    assert private["map_revision"] == "20260727T101500"


def test_a_legacy_combined_file_still_loads(tmp_path):
    """A robot that has been mapping a building for a year is not re-taught.

    Its retired fields load and are dropped: the zone keeps its name, its
    coordinate and its footprint, and `galley` no longer reaches anything.
    """
    (tmp_path / "zones.yaml").write_text(
        "frame_id: map\nzones:\n"
        "  kitchen: {x: 2.0, y: 3.5, radius: 1.5, kind: room, aliases: [galley]}\n"
    )
    zones = mote_zones.load_floor(tmp_path)
    assert zones["kitchen"].bound is True
    assert zones["kitchen"].footprint is not None
    assert mote_zones.resolve(zones, "kitchen").name == "kitchen"
    assert mote_zones.resolve(zones, "galley") is None

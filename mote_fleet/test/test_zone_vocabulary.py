"""The zone vocabulary over a real socket: what places can be named here.

The question a dispatcher actually asks is *what may I say?*, and until this
route existed the fleet API could not answer it — the roster, the basemaps and
the dispatch route were all served, and the names were not. The workarounds
were an out-of-band document or scraping the list a robot prints when it
refuses an unknown zone, which is an accident of an error message rather than
a contract.

What is asserted here is the property that makes serving it safe at all: the
vocabulary carries **no coordinates**. Not "the ones we remembered to remove" —
none, checked by walking the payload rather than by naming the keys we thought
of, because the leak this guards against is a *future* geometry key added to
``zones.yaml`` by someone who never reads this file. A vocabulary is portable
between robots; a binding is not, and a coordinate that escapes into a portable
document looks entirely plausible right up until a second robot acts on it.

The binding's own route is unchanged and still served — see
``test_map_registry.py`` — because the client that draws zones on a basemap
already has the basemap.
"""

import pytest
from api_harness import expect_error, get, write_bundle

from mote_bringup import bundle

SITE, FLOOR = "home", "ground"

#: Any of these appearing anywhere in a vocabulary payload is the bug this
#: whole split exists to prevent.
GEOMETRY_KEYS = frozenset(
    ("x", "y", "yaw", "radius", "polygon", "frame_id", "pose", "footprint")
)


def coordinates_in(payload) -> list:
    """Every geometry-shaped key anywhere in the payload, however nested.

    Deliberately a walk rather than a check of the keys this test happens to
    know about: the failure mode is a key nobody here anticipated.
    """
    found = []
    if isinstance(payload, dict):
        found += [key for key in payload if key in GEOMETRY_KEYS]
        for value in payload.values():
            found += coordinates_in(value)
    elif isinstance(payload, list):
        for item in payload:
            found += coordinates_in(item)
    return found


# ---- one floor's vocabulary ---------------------------------------------


def test_a_floor_serves_its_names_kinds_and_aliases(server):
    status, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    assert status == 200
    assert (body["site"], body["floor"]) == (SITE, FLOOR)
    assert body["revision"] == 4
    by_name = {term["name"]: term for term in body["zones"]}
    assert sorted(by_name) == ["kitchen", "sluice", "ward"]
    assert by_name["kitchen"]["kind"] == "room"
    assert by_name["kitchen"]["display_name"] == "The Kitchen"
    assert by_name["kitchen"]["aliases"] == ["galley"]
    assert by_name["kitchen"]["navigable"] is True


def test_the_vocabulary_carries_no_coordinates(server):
    _, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    assert coordinates_in(body) == []


def test_the_binding_still_carries_them(server):
    """The control: the same floor, the route that is allowed to say where."""
    _, body = get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json")
    assert coordinates_in(body) != []


def test_a_constraint_zone_is_named_but_not_navigable(server):
    """A keepout belongs in the vocabulary — an operator draws it on the same
    floor plan — and the flag is what stops it being dispatched to."""
    _, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    sluice = next(t for t in body["zones"] if t["name"] == "sluice")
    assert sluice["kind"] == "keepout"
    assert sluice["navigable"] is False


def test_a_zone_taught_before_any_of_this_is_a_plain_area(server, tmp_path):
    """Every vocabulary field is optional, so no zones.yaml needed rewriting."""
    floor = tmp_path / "sites" / SITE / "floors" / FLOOR
    (floor / "map" / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  bench: {x: 1.0, y: 1.0}\n"
    )
    _, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    bench = next(t for t in body["zones"] if t["name"] == "bench")
    assert bench["kind"] == "area"
    assert bench["navigable"] is True
    assert bench["aliases"] == []
    assert body["revision"] == 0


def test_an_unknown_floor_is_404(server):
    expect_error(lambda: get(server, f"/v1/zones/{SITE}/attic"), 404)


def test_a_malformed_path_is_404(server):
    expect_error(lambda: get(server, f"/v1/zones/{SITE}"), 404)
    expect_error(lambda: get(server, f"/v1/zones/{SITE}/{FLOOR}/extra"), 404)


@pytest.mark.parametrize(
    "path",
    [
        "/v1/zones/..%2F..%2Fetc/ground",
        "/v1/zones/home/..%2F..%2Fregistry.db",
    ],
)
def test_a_name_cannot_escape_the_bundle_root(server, path):
    """Percent-encoded, so the traversal survives the client and is refused
    here rather than normalised away before it ever arrives."""
    expect_error(lambda: get(server, path), 400)


# ---- the whole fleet's vocabulary ---------------------------------------


def test_every_floor_is_listed_in_one_call(server, tmp_path):
    write_bundle(tmp_path / "sites", "depot", "first")
    status, body = get(server, "/v1/zones")
    assert status == 200
    assert {(v["site"], v["floor"]) for v in body["vocabularies"]} == {
        ("home", "ground"),
        ("depot", "first"),
    }
    assert coordinates_in(body) == []


def test_a_named_but_unmapped_floor_still_has_a_vocabulary(server, tmp_path):
    """The point of the split, stated as a test.

    Names are a fact about the building and do not wait on a SLAM session, so a
    floor an operator has named but no robot has mapped answers here. Gating
    this on a published revision — as the *binding* is rightly gated — would
    have handed back the portability the split exists to buy.
    """
    floor = tmp_path / "sites" / SITE / "floors" / "attic"
    floor.mkdir(parents=True)
    (floor / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  loft: {x: 0.0, y: 0.0, kind: room}\n"
    )
    _, body = get(server, f"/v1/zones/{SITE}/attic")
    assert [t["name"] for t in body["zones"]] == ["loft"]

    # ...while the binding for that floor is still refused, because there is no
    # map frame for those coordinates to be in.
    expect_error(lambda: get(server, f"/v1/maps/{SITE}/attic/zones.json"), 404)

    _, listing = get(server, "/v1/zones")
    assert (SITE, "attic") in {(v["site"], v["floor"]) for v in listing["vocabularies"]}


def test_a_floor_with_no_zones_is_skipped_not_an_error(server, tmp_path):
    (tmp_path / "sites" / SITE / "floors" / "empty").mkdir(parents=True)
    status, body = get(server, "/v1/zones")
    assert status == 200
    assert (SITE, "empty") not in {
        (v["site"], v["floor"]) for v in body["vocabularies"]
    }


# ---- problems are reported, not hidden ----------------------------------


def test_an_ambiguous_vocabulary_is_reported_and_still_served(server, tmp_path):
    """The server reports; it does not refuse. The map is unaffected by a
    duplicated alias, and a floor's basemap must not stop being served over
    one — but a dispatcher has to be able to see that a name is unanswerable."""
    floor = tmp_path / "sites" / SITE / "floors" / FLOOR
    (floor / "map" / "zones.yaml").write_text(
        "frame_id: map\nzones:\n"
        "  kitchen: {x: 1.0, y: 1.0}\n"
        "  galley: {x: 2.0, y: 2.0, aliases: [Kitchen]}\n"
    )
    status, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    assert status == 200
    assert len(body["zones"]) == 2
    assert any("ambiguous" in problem for problem in body["problems"])


def test_a_name_a_dispatcher_cannot_type_is_reported(server, tmp_path):
    floor = tmp_path / "sites" / SITE / "floors" / FLOOR
    (floor / "map" / "zones.yaml").write_text(
        'frame_id: map\nzones:\n  "Café": {x: 1.0, y: 1.0}\n'
    )
    _, body = get(server, f"/v1/zones/{SITE}/{FLOOR}")
    # Served verbatim rather than silently slugified: inventing 'cafe' would be
    # a rename nobody asked for, and the operator is the one who must fix it.
    assert [t["name"] for t in body["zones"]] == ["Café"]
    assert any("dispatchable" in problem for problem in body["problems"])


@pytest.mark.parametrize(
    "entry, message",
    [
        ("{x: 1.0, y: 1.0, kind: lounge}", "unknown kind"),
        ("{x: 1.0, y: 1.0, kind: keepout, navigable: true}", "not a destination"),
        ("{x: 1.0, y: 1.0, aliases: galley}", "list of strings"),
    ],
)
def test_a_vocabulary_the_file_cannot_mean_is_refused(tmp_path, entry, message):
    """These are not ambiguities to report — they are files with no reading.

    A ``navigable: true`` keepout is the interesting one: honouring it would
    make the flag mean whatever was typed last, so the contradiction is refused
    at the parse rather than resolved by precedence.
    """
    path = tmp_path / "zones.yaml"
    path.write_text(f"frame_id: map\nzones:\n  a: {entry}\n")
    with pytest.raises(bundle.BundleError, match=message):
        bundle.read_zones(path)

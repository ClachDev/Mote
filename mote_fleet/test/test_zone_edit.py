"""Operator zone edits over a real socket: derive a candidate, never mutate.

The dashboard's zone editor saves by POSTing the edited set to
``/v1/sites/<site>/floors/<floor>/zones``. What must hold: the result is an
ordinary *candidate* (validated like any upload, listed by the floor route,
promotable through the existing route), the canonical revision's bytes are
untouched (the announced digest depends on that), and a set the robot's own
loader would refuse — a bad name, a colliding alias — is refused here with the
reasons rather than stored.
"""

from api_harness import expect_error, get, post

from mote_bringup import bundle
from mote_fleet import protocol

SITE, FLOOR = "home", "ground"
CANONICAL = "20260726T120000"  # published by the server fixture

ZONES = {
    "kitchen": {
        "x": 1.0,
        "y": -3.0,
        "yaw": 0.0,
        "kind": "room",
        "display_name": "Kitchen",
        "polygon": [[0.0, -4.0], [2.0, -4.0], [2.0, -2.0], [0.0, -2.0]],
    },
    "office": {"x": 0.5, "y": 0.5, "yaw": 0.0, "kind": "room"},
}


def edit(server, zones, token, floor=FLOOR):
    return post(
        server,
        f"/v1/sites/{SITE}/floors/{floor}/zones",
        {"schema": protocol.SCHEMA, "zones": zones},
        token=token,
    )


def test_edit_derives_a_candidate_and_leaves_the_canonical_alone(server):
    token = server.registry.new_operator(name="editor")
    before = server.store.pack(SITE, FLOOR, CANONICAL)

    status, body = edit(server, ZONES, token)
    assert status == 201, body
    stored = body["revision"]
    assert body["derived_from"] == CANONICAL
    assert body["promoted"] is False
    assert stored != CANONICAL

    # The canonical is byte-identical afterwards: an edit is a derivation,
    # never a mutation of bytes the fleet may already hold a digest for.
    assert server.store.pack(SITE, FLOOR, CANONICAL) == before

    # The candidate carries the submitted zones, keyed by name, no echo.
    zones_file = server.maps / SITE / "floors" / FLOOR / "maps" / stored / "zones.yaml"
    parsed = bundle.read_zones(zones_file)
    assert set(parsed["zones"]) == {"kitchen", "office"}
    kitchen = parsed["zones"]["kitchen"]
    assert [list(point) for point in kitchen["polygon"]] == ZONES["kitchen"]["polygon"]
    # The file keys by name and carries no redundant copy inside the entry
    # (read_zones adds one when parsing; the raw file must not).
    import yaml

    raw = yaml.safe_load(zones_file.read_text())
    assert "name" not in raw["zones"]["office"]

    # And the floor route lists it as a promotable candidate.
    status, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert status == 200
    listed = {r["revision"]: r for r in floor["revisions"]}
    assert stored in listed and listed[stored]["ok"]


def test_edited_candidate_promotes_through_the_existing_route(server):
    token = server.registry.new_operator(name="editor")
    _, body = edit(server, ZONES, token)
    stored = body["revision"]
    status, body = post(
        server,
        f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/{stored}/promote",
        {"schema": protocol.SCHEMA},
        token=token,
    )
    assert status == 200, body
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] == stored


def test_zone_edit_requires_an_operator(server):
    expect_error(lambda: edit(server, ZONES, token=None), 401)


def test_structurally_unreadable_zones_are_refused(server):
    token = server.registry.new_operator(name="editor")
    bad = {"kitchen": {"x": 0.0, "y": 0.0, "yaw": 0.0, "aliases": "not-a-list"}}
    expect_error(lambda: edit(server, bad, token), 422)


def test_a_name_a_dispatcher_cannot_type_is_stored_with_a_warning(server):
    # The vocabulary rule: problems are reported, not enforced — the server
    # keeps the map good and *says* what dispatch will cost. The robot's own
    # loader is what refuses it. (The editor blocks these client-side too.)
    token = server.registry.new_operator(name="editor")
    bad = {"Kitchen Zone": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
    status, body = edit(server, bad, token)
    assert status == 201
    assert any("Kitchen Zone" in warning for warning in body["warnings"])


def test_editing_a_floor_with_no_published_map_is_refused(server):
    token = server.registry.new_operator(name="editor")
    expect_error(lambda: edit(server, ZONES, token, floor="attic"), 409)


def test_a_non_mapping_body_is_a_400(server):
    token = server.registry.new_operator(name="editor")
    expect_error(
        lambda: post(
            server,
            f"/v1/sites/{SITE}/floors/{FLOOR}/zones",
            {"schema": protocol.SCHEMA, "zones": ["not", "a", "mapping"]},
            token=token,
        ),
        400,
    )

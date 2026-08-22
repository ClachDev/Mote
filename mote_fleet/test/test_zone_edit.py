"""Operator zone edits over a real socket: derive a candidate, never mutate.

The dashboard's zone editor saves by POSTing the edited set to
``/v1/sites/<site>/floors/<floor>/zones``. What must hold: the result is an
ordinary *candidate* (validated like any upload, listed by the floor route,
promotable through the existing route), the source revision's bytes are
untouched (the announced digest depends on that), and a set the robot's own
loader would refuse — a bad name, a colliding alias — is refused here with the
reasons rather than stored.

The body's ``revision`` is what makes an *unpromoted* map editable, which is the
case the pipeline is built around: a fresh build arrives carrying `zone_01`..
`zone_07` from `segment-map`, and an edit that could only derive from the
canonical revision would have meant promoting those placeholder names in order
to be allowed to fix them.
"""

import yaml
from api_harness import (
    expect_error,
    get,
    packed_revision,
    post,
    post_bytes,
    write_revision,
)

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
        "aliases": ["galley"],
        "polygon": [[0.0, -4.0], [2.0, -4.0], [2.0, -2.0], [0.0, -2.0]],
    },
    "office": {"x": 0.5, "y": 0.5, "yaw": 0.0, "kind": "room"},
}


def edit(server, zones, token, floor=FLOOR, revision=None):
    body = {"schema": protocol.SCHEMA, "zones": zones}
    if revision is not None:
        body["revision"] = revision
    return post(server, f"/v1/sites/{SITE}/floors/{floor}/zones", body, token=token)


def upload_candidate(server, tmp_path, revision="20260801T090000", **kwargs):
    """A candidate on the fixture's floor, as a robot publishes one. Needs the
    ``robot`` fixture, since an upload names an enrolled robot."""
    blob = packed_revision(tmp_path, name=revision, **kwargs)
    status, body = post_bytes(
        server,
        f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/{revision}?robot_id=mote-01",
        blob,
        content_type="application/gzip",
    )
    assert status == 201, body
    return body["revision"]


def stored_zones(server, revision):
    path = server.maps / SITE / "floors" / FLOOR / "maps" / revision / "zones.yaml"
    return bundle.read_zones(path), yaml.safe_load(path.read_text())


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
    parsed, raw = stored_zones(server, stored)
    assert set(parsed["zones"]) == {"kitchen", "office"}
    kitchen = parsed["zones"]["kitchen"]
    assert [list(point) for point in kitchen["polygon"]] == ZONES["kitchen"]["polygon"]
    assert kitchen["aliases"] == ["galley"]
    # The file keys by name and carries no redundant copy inside the entry
    # (read_zones adds one when parsing; the raw file must not).
    assert "name" not in raw["zones"]["office"]

    # And the floor route lists it as a promotable candidate.
    status, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert status == 200
    listed = {r["revision"]: r for r in floor["revisions"]}
    assert stored in listed and listed[stored]["ok"]


def test_a_candidates_zones_can_be_edited_without_promoting_it(server, tmp_path, robot):
    """The case the pipeline is built around: rename the rooms of a map that is
    not published, and keep it unpublished. The derived candidate must carry the
    *edited* candidate's map bytes, not the canonical one's — the operator drew
    those coordinates on the candidate's map, and the two frames differ."""
    token = server.registry.new_operator(name="editor")
    candidate = upload_candidate(server, tmp_path, width=64, height=48)
    before = server.store.pack(SITE, FLOOR, candidate)

    status, body = edit(server, ZONES, token, revision=candidate)
    assert status == 201, body
    stored = body["revision"]
    assert body["derived_from"] == candidate

    # Nothing was promoted, and neither source was touched.
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] == CANONICAL
    assert server.store.pack(SITE, FLOOR, candidate) == before

    # The map under the edited zones is the candidate's, which its size proves:
    # the fixture's canonical revision is 40x30 and this one is 64x48.
    status, meta = get(
        server, f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/{stored}/map.json"
    )
    assert (meta["width"], meta["height"]) == (64, 48)


def test_editing_a_candidate_on_a_floor_with_nothing_published(server, tmp_path, robot):
    """A floor whose only revision is a candidate is exactly the floor whose
    zones most need renaming, and it has no canonical revision to derive from."""
    token = server.registry.new_operator(name="editor")
    blob = packed_revision(tmp_path, name="attic-rev")
    status, body = post_bytes(
        server,
        f"/v1/sites/{SITE}/floors/attic/revisions/20260801T101010?robot_id=mote-01",
        blob,
        content_type="application/gzip",
    )
    assert status == 201, body
    candidate = body["revision"]

    status, body = edit(server, ZONES, token, floor="attic", revision=candidate)
    assert status == 201, body
    assert body["derived_from"] == candidate
    _, floor = get(server, f"/v1/sites/{SITE}/floors/attic")
    assert floor["canonical"] is None  # still nothing published


def test_the_vocabulary_revision_advances_from_the_edited_revision(
    server, tmp_path, robot
):
    """A carry-forward has to be able to tell which naming is newer, so the
    counter continues from the revision that was edited rather than restarting.
    The fixture's zones.yaml is at 4."""
    token = server.registry.new_operator(name="editor")
    _, body = edit(server, ZONES, token)
    first = body["revision"]
    assert stored_zones(server, first)[1]["vocabulary_revision"] == 5

    # Editing the result again continues from *it*, not from the canonical.
    _, body = edit(server, ZONES, token, revision=first)
    assert stored_zones(server, body["revision"])[1]["vocabulary_revision"] == 6


def test_a_revision_with_no_posegraph_can_still_have_its_zones_edited(
    server, tmp_path, robot
):
    """A derivation is held to the bar its source already cleared, not to the
    upload's. A revision with no posegraph is one nothing can extend, which is a
    *warning* the review pane shows beside a `promotable` verdict — so an edit of
    one must succeed, or that pane offers a button that can only fail. (It did,
    in a browser: every sim site bundle is such a revision, and so is any floor
    seeded by rsync from before the registry existed — which is why this one is
    written straight onto disk, an upload being refused for the same reason.)"""
    token = server.registry.new_operator(name="editor")
    seeded = "20260801T090000"
    write_revision(
        server.maps / SITE / "floors" / FLOOR / "maps" / seeded, posegraph=False
    )

    status, body = edit(server, ZONES, token, revision=seeded)
    assert status == 201, body
    assert any("posegraph" in warning for warning in body["warnings"])

    # And the bar for a robot's *upload* is unchanged: a mapping session that
    # produced no posegraph produced a map that cannot be extended, and that is
    # still refused where the session can still be re-run.
    expect_error(
        lambda: post_bytes(
            server,
            f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/20260803T090000?robot_id=mote-01",
            packed_revision(tmp_path, name="no-graph", posegraph=False),
            content_type="application/gzip",
        ),
        422,
    )


def test_an_unknown_source_revision_is_a_404(server):
    token = server.registry.new_operator(name="editor")
    expect_error(lambda: edit(server, ZONES, token, revision="20990101T000000"), 404)


def test_a_source_revision_that_is_not_a_name_is_refused(server):
    # The revision reaches the filesystem as a path component, so the traversal
    # story is the same one `_names` tells for site and floor.
    token = server.registry.new_operator(name="editor")
    expect_error(lambda: edit(server, ZONES, token, revision="../../etc"), 400)


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

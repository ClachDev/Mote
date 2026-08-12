"""The map registry over a real socket: upload, validate, promote, announce.

M4's acceptance, minus the wheels: a robot publishes a map revision, the server
keeps it as a candidate and changes nothing, an operator promotes it, and the
retained announcement a robot would pull from is published. The second robot's
map of the same floor is kept beside the first — never merged, because a map
frame's origin is an accident of where SLAM started.

The broker is the same stub the rest of the API tests use, so what is asserted
here is *what the server publishes and with what retain flag*; the real MQTT hop
is ``test_e2e_fleet.py``'s. The robot's half of the exchange — staging a pulled
revision and flipping the symlink — is ``test_mapsync.py``.
"""

import pytest
from api_harness import (
    enroll,
    expect_error,
    get,
    get_bytes,
    packed_revision,
    post,
    post_bytes,
    write_png,
    write_revision,
)

from mote_bringup import bundle
from mote_fleet import protocol

SITE, FLOOR = "home", "ground"


def upload(server, blob, robot_id="mote-01", revision="20260727T101500", **kwargs):
    site = kwargs.pop("site", SITE)
    floor = kwargs.pop("floor", FLOOR)
    return post_bytes(
        server,
        f"/v1/sites/{site}/floors/{floor}/revisions/{revision}?robot_id={robot_id}",
        blob,
    )


def promote(server, revision, token, site=SITE, floor=FLOOR):
    return post(
        server,
        f"/v1/sites/{site}/floors/{floor}/revisions/{revision}/promote",
        {"schema": protocol.SCHEMA},
        token=token,
    )


# ---- what the registry holds --------------------------------------------


def test_sites_lists_every_floor_and_what_it_is_on(server):
    status, body = get(server, "/v1/sites")
    assert status == 200
    assert body["sites"] == [
        {
            "site": "home",
            "floor": "ground",
            "canonical": "20260726T120000",
            "candidates": [],
            "revisions": ["20260726T120000"],
        }
    ]


def test_a_floor_reports_each_revision_with_why_it_could_be_promoted(server):
    _, body = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    revision = body["revisions"][0]
    assert revision["canonical"] is True
    assert revision["ok"] is True
    assert revision["zones"] == ["kitchen", "sluice", "ward"]
    assert revision["map"]["resolution"] == 0.05


def test_an_unknown_floor_is_404(server):
    expect_error(lambda: get(server, "/v1/sites/home/floors/attic"), 404)


def test_a_floor_serves_a_revision_whose_meta_timestamp_was_left_unquoted(server):
    """The registry serves revisions it did not write — uploaded, restored from
    a backup, or seeded by rsync from before it existed — so it cannot assume
    the typing YAML gives a file. An unquoted ``saved:`` is a ``datetime``,
    which ``json.dumps`` refuses, and a route that raises there answers with no
    status line at all: the connection closes, and the dashboard's floor fetch
    fails in a way indistinguishable from the server being down.
    """
    meta = server.maps / SITE / "floors" / FLOOR / "maps" / "20260726T120000"
    (meta / "meta.yaml").write_text("schema: 1\nsaved: 2026-07-05T11:16:46\n")
    status, body = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert status == 200
    assert body["revisions"][0]["meta"]["saved"] == "2026-07-05T11:16:46"


def test_a_candidate_uploaded_with_an_unquoted_meta_timestamp_is_accepted(
    server, robot, tmp_path
):
    """The same value on the way in: the report is stored as JSON beside the
    revision, so an untyped one would break the upload's own response too.
    """
    directory = tmp_path / "rev"
    write_revision(directory)
    (directory / "meta.yaml").write_text("schema: 1\nsaved: 2026-07-05T11:16:46\n")
    status, body = upload(server, bundle.pack(directory))
    assert status == 201
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    candidate = [r for r in floor["revisions"] if r["revision"] == body["revision"]][0]
    assert candidate["meta"]["saved"] == "2026-07-05T11:16:46"


# ---- uploading a candidate ----------------------------------------------


def test_an_uploaded_revision_is_a_candidate_and_changes_nothing(
    server, robot, tmp_path
):
    status, body = upload(server, packed_revision(tmp_path))
    assert status == 201
    assert body["revision"] == "20260727T101500"
    assert body["promoted"] is False
    # The floor is still on the map it was on. This is the property that makes
    # publishing safe to run after every mapping session.
    assert body["canonical"] == "20260726T120000"
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] == "20260726T120000"
    assert [r["revision"] for r in floor["revisions"]] == [
        "20260726T120000",
        "20260727T101500",
    ]
    # Nothing was announced: an upload is not a publication.
    assert server.publisher.published == []


def test_an_upload_is_recorded_against_the_robot_that_sent_it(server, robot, tmp_path):
    upload(server, packed_revision(tmp_path))
    operator_token = server.registry.new_operator(name="auditor")
    _, body = get(server, "/v1/audit", token=operator_token)
    row = body["audit"][0]
    assert (row["action"], row["actor"], row["result"]) == (
        "map.upload",
        "mote-01",
        "stored",
    )
    assert row["command"] == f"{SITE}/{FLOOR}/20260727T101500"


def test_an_upload_from_a_robot_this_fleet_does_not_know_is_404(server, tmp_path):
    expect_error(lambda: upload(server, packed_revision(tmp_path), "mote-99"), 404)


def test_an_upload_without_a_robot_id_is_400(server, tmp_path):
    expect_error(
        lambda: post_bytes(
            server,
            f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/20260727T101500",
            packed_revision(tmp_path),
        ),
        400,
    )


def test_an_incomplete_revision_is_refused_with_its_reasons(server, robot, tmp_path):
    """The server re-validates what the robot already checked, because an
    upload can truncate where a local save could not."""
    blob = packed_revision(tmp_path, posegraph=False)
    body = expect_error(lambda: upload(server, blob), 422)
    assert any("map.posegraph" in error for error in body["errors"])
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert len(floor["revisions"]) == 1  # nothing was stored


def test_a_degenerate_map_is_refused(server, robot, tmp_path):
    """Every file present, a sane map.yaml, and a uniform grey rectangle —
    what a mapping run that never got going produces. Nothing else in the
    pipeline looks at the pixels."""
    directory = write_revision(tmp_path / "blank")
    write_png(directory / "map.png", 40, 30, fill=b"\xcd")  # all unknown
    body = expect_error(lambda: upload(server, bundle.pack(directory)), 422)
    assert any("no free space" in error for error in body["errors"])


def test_an_undecodable_image_answers_and_closes_its_audit_row(server, robot, tmp_path):
    """A map.png with an invalid scanline filter byte.

    The decoder used to raise through validate() — which documents that it
    never does — so the handler died with no HTTP response at all and left the
    upload's audit row saying 'receiving' for ever. The bytes are the point:
    this is what a truncated or bit-flipped transfer looks like, which is the
    case the server-side re-validation exists for.
    """
    directory = write_revision(tmp_path / "corrupt")
    write_png(directory / "map.png", 40, 30, fill=b"\xfe", filter_type=9)
    blob = bundle.pack(directory)

    body = expect_error(lambda: upload(server, blob), 422)
    assert any("readable PNG" in error for error in body["errors"])

    operator_token = server.registry.new_operator(name="auditor")
    _, audit = get(server, "/v1/audit", token=operator_token)
    row = next(r for r in audit["audit"] if r["action"] == "map.upload")
    assert row["result"] != "receiving"


def test_something_that_is_not_a_bundle_is_refused(server, robot):
    body = expect_error(lambda: upload(server, b"not a tarball at all"), 400)
    assert "bundle" in body["error"]


def test_re_uploading_the_same_revision_does_not_mint_a_second(server, robot, tmp_path):
    blob = packed_revision(tmp_path)
    upload(server, blob)
    status, body = upload(server, blob)
    assert (status, body["revision"]) == (201, "20260727T101500")
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert len(floor["revisions"]) == 2


def test_two_robots_mapping_one_floor_keep_both_maps(server, robot, tmp_path):
    """The conflict rule: no merge, no overwrite, two candidates, and an
    operator picks. Silently merging two map frames would break every taught
    zone coordinate."""
    enroll(server, "serial:bbb", name="Scout two")
    first = packed_revision(tmp_path, name="one")
    second = packed_revision(tmp_path, name="two", width=60, height=50)
    upload(server, first, "mote-01")
    status, body = upload(server, second, "mote-02")
    assert status == 201
    # Same proposed id, different content: the second is stored beside the
    # first under a qualified id rather than replacing it.
    assert body["revision"] == "20260727T101500-2"
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    stored = {r["revision"]: r for r in floor["revisions"]}
    assert stored["20260727T101500"]["map"]["width"] == 40
    assert stored["20260727T101500-2"]["map"]["width"] == 60
    assert floor["canonical"] == "20260726T120000"  # still neither of them


# ---- promotion ----------------------------------------------------------


def test_promoting_flips_the_floor_and_announces_it_retained(
    server, robot, operator, tmp_path
):
    upload(server, packed_revision(tmp_path))
    status, body = promote(server, "20260727T101500", operator)
    assert status == 200
    assert body["revision"] == "20260727T101500"
    assert body["announced"] is True

    topic = protocol.registry_topic(SITE, FLOOR)
    announcement = server.publisher.retained(topic)
    assert announcement is not None, "the announcement must be retained"
    protocol.check(announcement, protocol.CURRENT)
    assert announcement["revision"] == "20260727T101500"
    assert announcement["promoted_by"] == "michael"
    assert announcement["sha256"].startswith("sha256:")

    # And the floor really is on it, through the route the dashboard reads.
    _, maps = get(server, "/v1/maps")
    assert maps["maps"][0]["revision"] == "20260727T101500"


def test_the_announced_bundle_is_the_one_that_is_served(
    server, robot, operator, tmp_path
):
    upload(server, packed_revision(tmp_path))
    _, promoted = promote(server, "20260727T101500", operator)
    status, content_type, blob = get_bytes(server, promoted["url"])
    assert (status, content_type) == (200, "application/gzip")
    # The digest on the retained announcement is the digest of the bytes the
    # route serves, which is what makes the puller's check meaningful.
    assert bundle.digest(blob) == promoted["sha256"]


def test_promotion_needs_an_operator_token(server, robot, tmp_path):
    upload(server, packed_revision(tmp_path))
    expect_error(lambda: promote(server, "20260727T101500", None), 401)
    assert server.publisher.published == []


def test_promoting_a_revision_that_is_not_there_is_404(server, operator):
    expect_error(lambda: promote(server, "20260101T000000", operator), 404)


def test_a_promotion_that_cannot_be_announced_still_happened(
    server, robot, operator, tmp_path
):
    """The flip is the fact; the announcement is best effort. Reporting the
    promotion as failed would be a lie an operator could not act on — the
    symlink has already moved."""
    upload(server, packed_revision(tmp_path))
    server.publisher.fail = "broker unreachable"
    status, body = promote(server, "20260727T101500", operator)
    assert status == 200
    assert body["announced"] is False
    assert "unreachable" in body["detail"]
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] == "20260727T101500"


def test_startup_re_announces_what_is_actually_published(server):
    """Which is what repairs a promotion made while the broker was down, and a
    broker that lost its retained state with its volume."""
    assert server.announce_all() == (1, True)
    announcement = server.publisher.retained(protocol.registry_topic(SITE, FLOOR))
    assert announcement["revision"] == "20260726T120000"


def test_startup_reannounce_retries_until_the_broker_answers(server, robot, tmp_path):
    """The compose start races: the server and mosquitto come up together, so
    the first publish usually fails. One attempt would leave every retained
    topic stale until somebody restarted the server."""
    upload(server, packed_revision(tmp_path))
    operator_token = server.registry.new_operator(name="op")
    promote(server, "20260727T101500", operator_token)

    # Both shapes a broker that is not up yet produces: a refusal, and a
    # socket error out of paho.
    failures = {"left": 2}

    def flaky(topic, payload, **kwargs):
        if failures["left"] == 2:
            failures["left"] -= 1
            return False, "not connected"
        if failures["left"]:
            failures["left"] -= 1
            raise OSError("broker is still starting")
        return True, ""

    server.publisher.publish = flaky
    assert server.announce_all_until_delivered(attempts=5, first_delay=0.01) == 1
    assert failures["left"] == 0


def test_a_promotion_is_audited(server, robot, operator, tmp_path):
    upload(server, packed_revision(tmp_path))
    promote(server, "20260727T101500", operator)
    _, body = get(server, "/v1/audit", token=operator)
    row = next(r for r in body["audit"] if r["action"] == "map.promote")
    assert (row["actor"], row["result"]) == ("michael", "promoted")
    assert row["command"] == f"{SITE}/{FLOOR}/20260727T101500"


def test_an_anonymous_promotion_attempt_is_recorded(server, robot, operator, tmp_path):
    upload(server, packed_revision(tmp_path))
    expect_error(lambda: promote(server, "20260727T101500", None), 401)
    _, body = get(server, "/v1/audit", token=operator)
    row = body["audit"][0]
    assert (row["actor"], row["result"]) == ("anonymous", "unauthorized")


# ---- what a robot pulls -------------------------------------------------


def test_a_revision_can_be_pulled_and_unpacks_to_the_same_files(server):
    status, content_type, blob = get_bytes(
        server,
        f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/20260726T120000/bundle.tar.gz",
    )
    assert (status, content_type) == (200, "application/gzip")
    assert bundle.validate(_unpacked(blob, server)).ok


def _unpacked(blob, server):
    landed = server.maps.parent / "landed"
    bundle.unpack(blob, landed)
    return landed


def test_pulling_a_revision_that_is_not_there_is_404(server):
    expect_error(
        lambda: get_bytes(
            server, f"/v1/sites/{SITE}/floors/{FLOOR}/revisions/nope/bundle.tar.gz"
        ),
        404,
    )


# ---- reviewing a candidate before promoting it --------------------------
#
# An operator who cannot see a candidate is promoting on faith in a timestamp,
# which is what the dashboard's review view exists to end. These three routes
# are the whole of what it reads: the candidate's own transform, its own
# pixels, and its own zones — all three named by the revision, none of them
# reachable through the canonical basemap's routes.


def review(revision, leaf, site=SITE, floor=FLOOR):
    return f"/v1/sites/{site}/floors/{floor}/revisions/{revision}/{leaf}"


@pytest.fixture
def candidate(server, robot, tmp_path):
    """A stored candidate whose map is visibly not the canonical one."""
    upload(server, packed_revision(tmp_path, name="candidate", width=60, height=45))
    return "20260727T101500"


def test_a_candidates_map_json_describes_that_revision(server, candidate):
    status, body = get(server, review(candidate, "map.json"))
    assert status == 200
    assert (body["width"], body["height"]) == (60, 45)
    assert body["revision"] == candidate
    assert body["resolution"] == 0.05
    assert body["origin"][:2] == [-2.927, -2.934]
    # The whole point of the route: a client that follows this URL gets the
    # candidate's pixels. Left pointing at /v1/maps — as it was — the review
    # view would draw the canonical map under the candidate's label, which is
    # exactly the promotion-on-faith this replaces.
    assert body["image_url"] == review(candidate, "map.png")


def test_a_candidates_map_png_is_not_the_canonical_maps(server, candidate):
    status, content_type, candidate_bytes = get_bytes(
        server, review(candidate, "map.png")
    )
    assert (status, content_type) == (200, "image/png")
    _, _, canonical_bytes = get_bytes(server, f"/v1/maps/{SITE}/{FLOOR}/map.png")
    assert candidate_bytes != canonical_bytes
    # Nothing was promoted to get here: reviewing is a read.
    _, floor = get(server, f"/v1/sites/{SITE}/floors/{FLOOR}")
    assert floor["canonical"] == "20260726T120000"


def test_a_candidates_zones_are_its_own(server, robot, tmp_path):
    directory = write_revision(tmp_path / "rev")
    (directory / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  loading_bay: {x: 9.0, y: -1.0}\n"
    )
    upload(server, bundle.pack(directory))
    status, body = get(server, review("20260727T101500", "zones.json"))
    assert status == 200
    assert body["revision"] == "20260727T101500"
    assert body["source"] == "revision"
    assert [zone["name"] for zone in body["zones"]] == ["loading_bay"]
    # And the floor's published binding is untouched by having been reviewed.
    _, published = get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json")
    assert sorted(zone["name"] for zone in published["zones"]) == [
        "kitchen",
        "sluice",
        "ward",
    ]


def test_a_revision_with_no_zones_falls_back_to_the_floors(server, robot, tmp_path):
    """``_zones_file``'s rule, unchanged: a floor seeded by rsync keeps its
    zones at floor level, and a revision that carries none inherits them.

    ``source`` is what makes that safe to show a reviewer. Inherited zones were
    taught in a *previous* session's frame, so they draw perfectly over this map
    and are wrong by however far the two origins differ — nothing in the
    coordinates says which case this is, so the payload does.
    """
    upload(server, packed_revision(tmp_path, name="bare", zones=False))
    floor_dir = server.maps / SITE / "floors" / FLOOR
    (floor_dir / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  lobby: {x: 0.0, y: 0.0}\n"
    )
    _, body = get(server, review("20260727T101500", "zones.json"))
    assert [zone["name"] for zone in body["zones"]] == ["lobby"]
    assert body["source"] == "floor"


def test_a_revision_with_no_zones_anywhere_is_404(server, robot, tmp_path):
    upload(server, packed_revision(tmp_path, name="bare", zones=False))
    expect_error(lambda: get(server, review("20260727T101500", "zones.json")), 404)


@pytest.mark.parametrize("leaf", ["map.json", "map.png", "zones.json"])
def test_reviewing_a_revision_that_is_not_there_is_404(server, leaf):
    expect_error(lambda: get(server, review("20260101T000000", leaf)), 404)


@pytest.mark.parametrize("leaf", ["map.json", "map.png", "zones.json"])
def test_a_review_route_refuses_a_name_it_could_not_have_written(server, leaf):
    expect_error(lambda: get(server, review("..%2F..%2Fregistry.db", leaf)), 400)


@pytest.mark.parametrize("leaf", ["map.json", "map.png", "zones.json"])
def test_reviewing_needs_no_operator_token(server, candidate, leaf):
    """Reads are unauthenticated exactly as every other read route is; M7
    changes that for all of them at once rather than for these three."""
    assert get_bytes(server, review(candidate, leaf))[0] == 200


def test_the_first_candidate_on_a_floor_can_be_reviewed_and_promoted(
    server, robot, operator, tmp_path
):
    """The bootstrap case: a floor whose only revisions are candidates.

    Nothing about review may depend on there already being a canonical map, or
    the first promotion on any floor could never be made — which is precisely
    the floor an operator most needs to look at before promoting.
    """
    upload(server, packed_revision(tmp_path, name="first"), floor="loft")
    _, detail = get(server, f"/v1/sites/{SITE}/floors/loft")
    assert detail["canonical"] is None
    assert [r["revision"] for r in detail["revisions"]] == ["20260727T101500"]

    _, meta = get(server, review("20260727T101500", "map.json", floor="loft"))
    assert meta["revision"] == "20260727T101500"
    _, zones = get(server, review("20260727T101500", "zones.json", floor="loft"))
    assert zones["zones"]
    assert (
        get_bytes(server, review("20260727T101500", "map.png", floor="loft"))[0] == 200
    )

    status, body = promote(server, "20260727T101500", operator, floor="loft")
    assert (status, body["revision"]) == (200, "20260727T101500")


# ---- zones on the basemap -----------------------------------------------


def test_zones_are_served_for_the_floor_being_drawn(server):
    status, body = get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json")
    assert status == 200
    assert body["frame_id"] == "map"
    by_name = {zone["name"]: zone for zone in body["zones"]}
    assert by_name["kitchen"]["radius"] == 1.5
    assert len(by_name["ward"]["polygon"]) == 4


def test_zones_travel_with_the_revision_that_becomes_canonical(
    server, robot, operator, tmp_path
):
    """Zone coordinates only mean anything in the frame they were taught in,
    so promoting a map promotes its zones."""
    directory = write_revision(tmp_path / "rev")
    (directory / "zones.yaml").write_text(
        "frame_id: map\nzones:\n  loading_bay: {x: 9.0, y: -1.0}\n"
    )
    upload(server, bundle.pack(directory))
    promote(server, "20260727T101500", operator)
    _, body = get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json")
    assert [zone["name"] for zone in body["zones"]] == ["loading_bay"]


def test_a_floor_with_no_zones_is_404_rather_than_an_empty_lie(
    server, robot, operator, tmp_path
):
    upload(server, packed_revision(tmp_path, name="nozones", zones=False))
    promote(server, "20260727T101500", operator)
    expect_error(lambda: get(server, f"/v1/maps/{SITE}/{FLOOR}/zones.json"), 404)


# ---- names --------------------------------------------------------------


@pytest.mark.parametrize(
    "path",
    [
        "/v1/sites/..%2F..%2Fetc/floors/ground",
        "/v1/sites/home/floors/ground/revisions/..%2F..%2F..%2Fregistry.db/bundle.tar.gz",
    ],
)
def test_a_name_cannot_escape_the_registry(server, path):
    expect_error(lambda: get(server, path), 400)

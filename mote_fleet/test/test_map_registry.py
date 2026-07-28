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
    assert revision["zones"] == ["kitchen", "ward"]
    assert revision["map"]["resolution"] == 0.05


def test_an_unknown_floor_is_404(server):
    expect_error(lambda: get(server, "/v1/sites/home/floors/attic"), 404)


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

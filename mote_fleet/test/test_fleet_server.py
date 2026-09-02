"""The fleet API, exercised over a real socket.

The server is stdlib ``http.server``, so there is nothing to mock: it binds an
ephemeral port and the tests speak HTTP to it. That also covers the bits a unit
test of the registry would miss — status codes, the shape of the JSON a robot
parses, and the fact that a bad request is answered rather than dropped.

The one thing stubbed is the broker: dispatch's publisher is injected, so the
authorize → audit → publish order is tested here and the real MQTT hop is tested
where it belongs, against a real broker in ``test_e2e_fleet.py``. The live
server, and the helpers for talking to it, are ``conftest.py`` +
``api_harness.py``; the map registry's own routes are ``test_map_registry.py``.
"""

import json
import urllib.error
import urllib.request

import pytest

import fleet_server
from api_harness import (
    enroll,
    expect_error,
    get,
    get_bytes,
    post,
    post_raw,
)

from mote_bringup.spec import mission
from mote_fleet import protocol


def dispatch(
    server, robot_id, capability="goto", payload_input=None, token=None, **extra
):
    return post(
        server,
        f"/v1/robots/{robot_id}/dispatch",
        {
            "schema": protocol.SCHEMA,
            "capability": capability,
            "input": {"target": "kitchen"} if payload_input is None else payload_input,
            **extra,
        },
        token=token,
    )


def test_healthz_names_the_contract(server):
    status, body = get(server, "/healthz")
    assert status == 200
    assert body["ok"] is True
    assert body["contract"] == "mote/v2"
    assert body["robots"] == 0


def test_enrollment_returns_an_id_and_the_broker(server):
    status, body = enroll(server, "serial:aaa", name="Scout", site="home")
    assert status == 201
    assert body["robot_id"] == "mote-01"
    assert body["name"] == "Scout"
    assert body["site"] == "home"
    assert body["created"] is True
    assert body["broker"] == {"host": "fleet-box", "port": 1883}


def test_re_enrolling_answers_200_and_the_same_id(server):
    enroll(server, "serial:aaa")
    status, body = enroll(server, "serial:aaa")
    assert (status, body["robot_id"], body["created"]) == (200, "mote-01", False)


def test_the_roster_lists_what_was_enrolled(server):
    enroll(server, "serial:aaa", name="Scout")
    enroll(server, "serial:bbb", name="Rover")
    status, body = get(server, "/v1/robots")
    assert status == 200
    assert [r["robot_id"] for r in body["robots"]] == ["mote-01", "mote-02"]
    assert [r["name"] for r in body["robots"]] == ["Scout", "Rover"]


def test_one_robot_can_be_fetched_by_id(server):
    enroll(server, "serial:aaa")
    status, body = get(server, "/v1/robots/mote-01")
    assert (status, body["robot_id"]) == (200, "mote-01")


def test_unknown_robot_is_404(server):
    expect_error(lambda: get(server, "/v1/robots/nope"), 404)


def test_unknown_route_is_404(server):
    expect_error(lambda: get(server, "/v1/nothing"), 404)


def test_a_missing_token_is_401(server):
    body = expect_error(
        lambda: post(
            server,
            "/v1/enroll",
            {"schema": protocol.SCHEMA, "fingerprint": "serial:aaa"},
        ),
        401,
    )
    assert "token" in body["error"]


def test_a_used_single_use_token_is_401(server):
    token = server.registry.new_token(single_use=True)
    enroll(server, "serial:aaa", token=token)
    body = expect_error(lambda: enroll(server, "serial:bbb", token=token), 401)
    assert "already used" in body["error"]


def test_a_missing_fingerprint_is_400(server):
    token = server.registry.new_token()
    expect_error(
        lambda: post(server, "/v1/enroll", {"schema": protocol.SCHEMA, "token": token}),
        400,
    )


def test_an_invalid_requested_id_is_400(server):
    expect_error(lambda: enroll(server, "serial:aaa", robot_id="Mote_01"), 400)


def test_a_conflicting_requested_id_is_409(server):
    enroll(server, "serial:aaa", robot_id="mote-07")
    body = expect_error(lambda: enroll(server, "serial:bbb", robot_id="mote-07"), 409)
    assert "already taken" in body["error"]


def test_a_non_json_body_is_400(server):
    expect_error(lambda: post_raw(server, "/v1/enroll", b"{not json"), 400)


# ---- the dashboard's bootstrap ----------------------------------------------


def test_config_tells_the_browser_where_the_broker_is(server):
    status, body = get(server, "/v1/config")
    assert status == 200
    assert body["contract"] == "mote/v2"
    assert body["broker"]["ws_port"] == 9001
    # Null on purpose: the page falls back to the host it was loaded from, so
    # MagicDNS, a tailnet address and localhost all work with no rebuild.
    assert body["broker"]["ws_host"] is None
    assert body["topics"]["health"] == protocol.HEALTH
    assert "{robot_id}" in body["foxglove_url"]


def test_the_ui_is_served_at_the_root(server):
    status, content_type, body = get_bytes(server, "/")
    assert status == 200
    assert content_type.startswith("text/html")
    assert b"mote" in body


def test_es_modules_are_served_as_javascript(server):
    """A browser refuses a module served as anything but JavaScript, and .mjs
    is not in every stdlib mimetypes table."""
    _, content_type, body = get_bytes(server, "/app.mjs")
    assert content_type.startswith("text/javascript")
    assert b"import" in body


def test_the_ui_cannot_serve_files_outside_itself(server):
    expect_error(lambda: get(server, "/../registry.db"), 404)
    expect_error(lambda: get(server, "/%2e%2e/registry.db"), 404)


# ---- dispatch: authorize, audit, publish ------------------------------------


def test_a_dispatch_is_published_with_a_correlation_id(server, operator, robot):
    status, body = dispatch(server, robot, token=operator)
    assert status == 202
    assert (body["capability"], body["input"]) == ("goto", {"target": "kitchen"})
    assert body["issued_by"] == "ui:michael"
    topic, payload, retain = server.publisher.published[0]
    # Never retained: a retained command re-fires on every reconnect.
    assert (topic, retain) == ("mote/v2/mote-01/mission/command", False)
    assert payload["capability"] == "goto"
    assert payload["input"] == {"target": "kitchen"}
    assert payload["platform_id"] == "mote-01"
    assert payload["lane"] == mission.DEFAULT_LANE
    assert payload["id"] == body["id"]
    assert payload["schema"] == mission.SCHEMA


def test_a_dispatch_is_audited_before_it_is_published(server, operator, robot):
    _, body = dispatch(server, robot, token=operator)
    entry = server.registry.audit()[0]
    assert entry["id"] == body["audit_id"]
    assert (entry["actor"], entry["result"]) == ("michael", "published")
    assert entry["command_id"] == body["id"]
    # The audit column is prose for a human; the machine-readable record of
    # what was dispatched is the mission id beside it.
    assert entry["command"] == "goto target='kitchen'"


def test_dispatch_without_a_token_is_refused_and_recorded(server, robot):
    body = expect_error(lambda: dispatch(server, robot), 401)
    assert "operator token" in body["error"]
    assert not server.publisher.published
    # The attempt is in the log: "who tried" is the half of an audit trail a
    # dashboard never shows you.
    entry = server.registry.audit()[0]
    assert (entry["actor"], entry["result"]) == ("anonymous", "unauthorized")
    assert entry["command"] == "goto target='kitchen'"


def test_a_revoked_token_stops_working(server, operator, robot):
    server.registry.revoke_operator(operator)
    expect_error(lambda: dispatch(server, robot, token=operator), 401)
    assert not server.publisher.published


def test_dispatch_to_an_unknown_robot_is_404(server, operator):
    expect_error(lambda: dispatch(server, "mote-99", token=operator), 404)
    assert not server.publisher.published
    assert server.registry.audit()[0]["result"] == "rejected"


def test_a_missing_capability_is_400(server, operator, robot):
    expect_error(lambda: dispatch(server, robot, "   ", token=operator), 400)


def test_an_input_that_is_not_an_object_is_400(server, operator, robot):
    """The one shape check this route does make. A bare string input could not
    be validated against any capability's schema, so it cannot reach a robot
    that would then have to guess what it meant."""
    expect_error(
        lambda: dispatch(server, robot, payload_input="kitchen", token=operator), 400
    )


def test_an_overlong_mission_is_400(server, operator, robot):
    expect_error(
        lambda: dispatch(
            server, robot, payload_input={"target": "x" * 300}, token=operator
        ),
        400,
    )


def test_the_capability_set_is_not_second_guessed(server, operator, robot):
    """The robot owns its capabilities and answers with a typed failure. A copy
    of the input schema here would be a second contract to keep in step, and it
    would refuse missions a newer robot understands."""
    status, _ = dispatch(
        server, robot, "x_wibble", {"flange": "sideways"}, token=operator
    )
    assert status == 202
    assert server.publisher.published[0][1]["capability"] == "x_wibble"


def test_a_lane_and_a_deadline_travel_untouched(server, operator, robot):
    status, _ = dispatch(
        server,
        robot,
        token=operator,
        lane="control",
        deadline="2026-08-22T12:00:00.000Z",
    )
    assert status == 202
    payload = server.publisher.published[0][1]
    assert payload["lane"] == "control"
    assert payload["deadline"] == "2026-08-22T12:00:00.000Z"


def test_a_broker_that_is_down_is_reported_not_swallowed(server, operator, robot):
    server.publisher.fail = "broker fleet-box:1883 unreachable"
    body = expect_error(lambda: dispatch(server, robot, token=operator), 503)
    assert "unreachable" in body["error"]
    entry = server.registry.audit()[0]
    assert entry["result"] == "error"
    assert "unreachable" in entry["detail"]


def test_the_audit_route_needs_an_operator_token(server, operator, robot):
    expect_error(lambda: get(server, "/v1/audit", token=""), 401)
    dispatch(server, robot, token=operator)
    status, body = get(server, "/v1/audit", token=operator)
    assert status == 200
    assert body["audit"][0]["command"] == "goto target='kitchen'"


def test_audit_can_be_filtered_by_robot(server, operator, robot):
    dispatch(server, robot, token=operator)
    _, body = get(server, "/v1/audit?robot_id=mote-02", token=operator)
    assert body["audit"] == []


# ---- basemaps ---------------------------------------------------------------


def test_maps_are_listed_from_the_site_bundles(server):
    status, body = get(server, "/v1/maps")
    assert status == 200
    # The route kept its shape across M4 and gained the canonical revision it
    # is showing — which is the one thing a viewer cannot work out itself.
    assert body["maps"] == [
        {
            "site": "home",
            "floor": "ground",
            "revision": "20260726T120000",
            "candidates": 0,
        }
    ]


def test_a_floors_map_metadata_carries_the_transform(server):
    _, body = get(server, "/v1/maps/home/ground/map.json")
    assert body["resolution"] == 0.05
    assert body["origin"] == [-2.927, -2.934, 0.0]
    # Dimensions come from the PNG header, so the browser can place a robot
    # before the image has decoded.
    assert (body["width"], body["height"]) == (40, 30)
    assert body["image_url"] == "/v1/maps/home/ground/map.png"


def test_the_basemap_image_is_served(server):
    status, content_type, body = get_bytes(server, "/v1/maps/home/ground/map.png")
    assert status == 200
    assert content_type == "image/png"
    assert body.startswith(b"\x89PNG")


def test_an_unknown_floor_is_404(server):
    expect_error(lambda: get(server, "/v1/maps/home/attic/map.json"), 404)


def test_a_site_name_cannot_escape_the_maps_directory(server):
    expect_error(lambda: get(server, "/v1/maps/..%2F..%2Fetc/ground/map.json"), 400)
    expect_error(lambda: get(server, "/v1/maps/home/ground/../map.yaml"), 404)


# ---- fleetctl's subscriptions survive a broker restart ----------------------


def test_a_reconnecting_client_resubscribes_every_time():
    """The bug this exists for: MQTT subscriptions belong to a session, and
    paho's default session is a clean one — so a client that subscribes once at
    startup comes back from a broker restart subscribed to nothing. It stays
    connected and silent, which looks exactly like a quiet fleet.

    Measured against the unfixed version: 7 lines before a broker restart, 7
    after, process still alive (docs/fleet/m3-verification.md §7).
    """
    import fleetctl

    class FakeClient:
        def __init__(self):
            self.subscribed = []

        def subscribe(self, topic, qos=0):
            self.subscribed.append((topic, qos))

    topics = ["mote/v2/+/health", "mote/v2/+/pose"]
    on_connect = fleetctl.subscriber(topics)
    client = FakeClient()

    on_connect(client, None, {}, 0)
    assert [t for t, _ in client.subscribed] == topics
    # ...and again, because this is what a reconnect calls.
    on_connect(client, None, {}, 0)
    assert [t for t, _ in client.subscribed] == topics + topics
    assert {qos for _, qos in client.subscribed} == {protocol.QOS}


# ---- the auth gate: every route, walked ------------------------------------
#
# These read `fleet_server.ROUTES` rather than a list kept here, so a route
# added to the server is covered the day it is added and a route that quietly
# stops requiring a credential fails a test rather than a review.

#: A value for every path variable the table uses. A route introducing a new
#: one fails here with a KeyError, which is the intended way to be told.
SAMPLES = {
    "robot_id": "mote-01",
    "site": "home",
    "floor": "ground",
    "revision": "20260726T120000",
    "leaf": "map.json",
}


def url_for(route):
    return route.template.format(**SAMPLES)


def call(server, route, token):
    """One request at a route, answered as ``(status, body bytes)``.

    Raw rather than parsed because the table holds a route serving gzip, and a
    test of who may reach a route has no business decoding what it serves.
    """
    request = urllib.request.Request(
        server.url + url_for(route),
        data=None if route.method == "GET" else json.dumps({}).encode(),
        headers={"Content-Type": "application/json"},
        method=route.method,
    )
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


GATED = [r for r in fleet_server.ROUTES if r.auth == fleet_server.OPERATOR]
OPEN = [r for r in fleet_server.ROUTES if r.auth != fleet_server.OPERATOR]


#: What the gate says, and the only refusal these tests are about.
REFUSAL = b"operator token"


@pytest.mark.parametrize("route", GATED, ids=url_for)
def test_every_gated_route_refuses_an_anonymous_request(server, robot, route):
    """M3 put a token on dispatch alone, so the roster, the basemaps, the audit
    log and the broker's address were readable by anything that could reach the
    port. This is the statement, per route, that they are not."""
    status, body = call(server, route, "")
    assert (status, REFUSAL in body) == (401, True)
    assert not server.publisher.published


@pytest.mark.parametrize("route", GATED, ids=url_for)
def test_every_gated_route_refuses_an_unknown_token(server, robot, route):
    assert call(server, route, "not-a-real-token")[0] == 401


@pytest.mark.parametrize("route", GATED, ids=url_for)
def test_every_gated_route_refuses_a_revoked_token(server, robot, route):
    token = server.registry.new_operator(name="leaver")
    server.registry.revoke_operator(token)
    assert call(server, route, token)[0] == 401


@pytest.mark.parametrize("route", OPEN, ids=url_for)
def test_an_open_route_never_asks_for_an_operator_token(server, robot, route):
    """The carve-outs, read off the table rather than out of prose: /healthz,
    because a liveness probe that needs a secret is one nobody wires up, and the
    three robot-facing routes, which carry their own credential (enrollment) or
    none at all (M4's map exchange, until robots have one).

    An open route may still refuse the request — enrollment without an
    enrollment token does, and an upload with no bundle in it does — so what is
    asserted is that it never refuses for want of an *operator*."""
    status, body = call(server, route, "")
    assert REFUSAL not in body, f"{route.template} is gated after all"
    assert status != 403


def test_the_table_names_which_routes_are_open(server):
    """Restated here so that opening a route is a deliberate edit to a test and
    not a line nobody reads."""
    assert {(r.method, r.template) for r in OPEN} == {
        ("GET", "/healthz"),
        ("POST", "/v1/enroll"),
        ("POST", "/v1/sites/{site}/floors/{floor}/revisions/{revision}"),
        ("GET", "/v1/sites/{site}/floors/{floor}/revisions/{revision}/bundle.tar.gz"),
    }


def test_every_route_names_a_handler_that_exists(server):
    for route in fleet_server.ROUTES:
        assert callable(getattr(fleet_server.FleetHandler, route.handler))


def test_authorization_is_checked_before_the_route_exists(server):
    """A 404 for an anonymous caller would say which routes are real."""
    expect_error(lambda: get(server, "/v1/nothing", token=""), 401)
    expect_error(lambda: post(server, "/v1/nothing", {"schema": protocol.SCHEMA}), 401)


def test_a_known_route_with_a_token_is_404_when_it_does_not_exist(server):
    expect_error(lambda: get(server, "/v1/nothing"), 404)


def test_the_static_ui_stays_open(server):
    """The page has to load in order to ask for a token, and it carries no
    fleet data until it has one."""
    status, content_type, body = get_bytes(server, "/index.html", token="")
    assert status == 200
    assert content_type.startswith("text/html")
    assert b"operator token" in body


def test_healthz_stays_open(server):
    status, body = get(server, "/healthz", token="")
    assert (status, body["ok"]) == (200, True)


def test_a_robot_pulls_a_bundle_without_a_credential(server, robot, tmp_path):
    """The other half of M4's upload: an agent told which revision is canonical
    has no operator token, and the alternative to this carve-out is a fleet whose
    maps never reach its robots."""
    from api_harness import packed_revision, post_bytes

    revision = "20260727T101500"
    post_bytes(
        server,
        f"/v1/sites/home/floors/ground/revisions/{revision}?robot_id={robot}",
        packed_revision(tmp_path),
    )
    status, content_type, blob = get_bytes(
        server,
        f"/v1/sites/home/floors/ground/revisions/{revision}/bundle.tar.gz",
        token="",
    )
    assert (status, content_type) == (200, "application/gzip")
    assert blob


# ---- what the route table matches ------------------------------------------


def test_a_path_variable_never_swallows_a_separator():
    """Matching is on the raw path, so an escaped separator stays inside one
    segment and is refused by name validation rather than becoming two
    components."""
    route, variables = fleet_server.match_route("GET", "/v1/zones/..%2F..%2Fetc/ground")
    assert route.handler == "_vocabulary"
    assert variables["site"] == "..%2F..%2Fetc"


def test_a_literal_segment_wins_over_a_variable_one():
    """``bundle.tar.gz`` is the robot's route and the review leaves are the
    operator's; they differ in one segment and in what they cost to reach."""
    pull, _ = fleet_server.match_route(
        "GET", "/v1/sites/home/floors/ground/revisions/r1/bundle.tar.gz"
    )
    review, _ = fleet_server.match_route(
        "GET", "/v1/sites/home/floors/ground/revisions/r1/map.png"
    )
    assert (pull.handler, pull.auth) == ("_bundle", fleet_server.ROBOT)
    assert (review.handler, review.auth) == ("_revision", fleet_server.OPERATOR)


def test_a_method_is_part_of_the_match():
    """The same path uploads under POST and is not readable under GET."""
    path = "/v1/sites/home/floors/ground/revisions/r1"
    assert fleet_server.match_route("POST", path)[0].handler == "_upload"
    assert fleet_server.match_route("GET", path)[0] is None

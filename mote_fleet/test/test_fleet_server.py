"""The fleet API, exercised over a real socket.

The server is stdlib ``http.server``, so there is nothing to mock: it binds an
ephemeral port and the tests speak HTTP to it. That also covers the bits a unit
test of the registry would miss — status codes, the shape of the JSON a robot
parses, and the fact that a bad request is answered rather than dropped.

The one thing stubbed is the broker: dispatch's publisher is injected, so the
authorize → audit → publish order is tested here and the real MQTT hop is tested
where it belongs, against a real broker in ``test_e2e_fleet.py``.
"""

import json
import struct
import threading
import urllib.error
import urllib.request
import zlib

import pytest
from fleet_server import serve

from mote_fleet import protocol


class FakeBroker:
    """Stands in for ``BrokerLink``. ``fail`` is how "the broker is down" is
    tested without taking a broker down."""

    def __init__(self):
        self.published = []
        self.fail = ""

    def publish(self, topic, payload):
        if self.fail:
            return False, self.fail
        self.published.append((topic, json.loads(payload)))
        return True, ""

    def close(self):
        pass


def write_png(path, width, height):
    """A real (grey) PNG, so the server's header reader is tested against the
    format rather than against a fixture that agrees with it."""

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    raw = b"".join(b"\x00" + b"\x80" * width for _ in range(height))
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def write_bundle(root, site, floor, *, width=40, height=30):
    """A site bundle as ``sites.py`` writes one, published symlink and all."""
    revision = root / site / "floors" / floor / "maps" / "20260726T120000"
    revision.mkdir(parents=True)
    (revision / "map.yaml").write_text(
        "image: map.png\nmode: trinary\nresolution: 0.050\n"
        "origin: [-2.927, -2.934, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    write_png(revision / "map.png", width, height)
    (revision.parent.parent / "map").symlink_to(
        revision.relative_to(revision.parent.parent)
    )
    return revision


@pytest.fixture
def server(tmp_path):
    maps = tmp_path / "sites"
    maps.mkdir()
    write_bundle(maps, "home", "ground")
    httpd = serve(
        db=tmp_path / "registry.db",
        host="127.0.0.1",
        port=0,
        broker_host="fleet-box",
        broker_port=1883,
        publisher=FakeBroker(),
        maps_dir=maps,
    )
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    httpd.url = f"http://127.0.0.1:{httpd.server_address[1]}"
    yield httpd
    httpd.shutdown()
    httpd.server_close()


def get(server, path, token=None):
    request = urllib.request.Request(server.url + path)
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, json.loads(response.read())


def get_bytes(server, path):
    with urllib.request.urlopen(server.url + path, timeout=10) as response:
        return response.status, response.headers["Content-Type"], response.read()


def post(server, path, payload, token=None):
    request = urllib.request.Request(
        server.url + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, json.loads(response.read())


def enroll(server, fingerprint, **extra):
    token = extra.pop("token", None) or server.registry.new_token()
    return post(
        server,
        "/v1/enroll",
        {
            "schema": protocol.SCHEMA,
            "token": token,
            "fingerprint": fingerprint,
            **extra,
        },
    )


def test_healthz_names_the_contract(server):
    status, body = get(server, "/healthz")
    assert status == 200
    assert body["ok"] is True
    assert body["contract"] == "mote/v1"
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


def expect_error(call, code):
    with pytest.raises(urllib.error.HTTPError) as caught:
        call()
    assert caught.value.code == code
    return json.loads(caught.value.read())


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
    request = urllib.request.Request(
        server.url + "/v1/enroll", data=b"{not json", method="POST"
    )
    with pytest.raises(urllib.error.HTTPError) as caught:
        urllib.request.urlopen(request, timeout=10)
    assert caught.value.code == 400


# ---- the dashboard's bootstrap ----------------------------------------------


def test_config_tells_the_browser_where_the_broker_is(server):
    status, body = get(server, "/v1/config")
    assert status == 200
    assert body["contract"] == "mote/v1"
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


@pytest.fixture
def operator(server):
    return server.registry.new_operator(name="michael")


@pytest.fixture
def robot(server):
    enroll(server, "serial:aaa", name="Scout")
    return "mote-01"


def dispatch(server, robot_id, command, token=None, **extra):
    return post(
        server,
        f"/v1/robots/{robot_id}/dispatch",
        {"schema": protocol.SCHEMA, "command": command, **extra},
        token=token,
    )


def test_a_dispatch_is_published_with_a_correlation_id(server, operator, robot):
    status, body = dispatch(server, robot, "goto kitchen", token=operator)
    assert status == 202
    assert body["command"] == "goto kitchen"
    assert body["issued_by"] == "ui:michael"
    topic, payload = server.publisher.published[0]
    assert topic == "mote/v1/mote-01/task/command"
    assert payload["command"] == "goto kitchen"
    assert payload["id"] == body["id"]
    assert payload["schema"] == protocol.SCHEMA


def test_a_dispatch_is_audited_before_it_is_published(server, operator, robot):
    _, body = dispatch(server, robot, "goto kitchen", token=operator)
    entry = server.registry.audit()[0]
    assert entry["id"] == body["audit_id"]
    assert (entry["actor"], entry["result"]) == ("michael", "published")
    assert entry["command_id"] == body["id"]


def test_dispatch_without_a_token_is_refused_and_recorded(server, robot):
    body = expect_error(lambda: dispatch(server, robot, "goto kitchen"), 401)
    assert "operator token" in body["error"]
    assert not server.publisher.published
    # The attempt is in the log: "who tried" is the half of an audit trail a
    # dashboard never shows you.
    entry = server.registry.audit()[0]
    assert (entry["actor"], entry["result"]) == ("anonymous", "unauthorized")
    assert entry["command"] == "goto kitchen"


def test_a_revoked_token_stops_working(server, operator, robot):
    server.registry.revoke_operator(operator)
    expect_error(lambda: dispatch(server, robot, "goto kitchen", token=operator), 401)
    assert not server.publisher.published


def test_dispatch_to_an_unknown_robot_is_404(server, operator):
    expect_error(
        lambda: dispatch(server, "mote-99", "goto kitchen", token=operator), 404
    )
    assert not server.publisher.published
    assert server.registry.audit()[0]["result"] == "rejected"


def test_an_empty_command_is_400(server, operator, robot):
    expect_error(lambda: dispatch(server, robot, "   ", token=operator), 400)


def test_an_overlong_command_is_400(server, operator, robot):
    expect_error(
        lambda: dispatch(server, robot, "goto " + "x" * 300, token=operator), 400
    )


def test_the_grammar_is_not_second_guessed(server, operator, robot):
    """The task layer owns the grammar and answers `rejected:` with a reason.
    A parser here would be a second grammar to keep in step."""
    status, _ = dispatch(server, robot, "wibble the flange", token=operator)
    assert status == 202
    assert server.publisher.published[0][1]["command"] == "wibble the flange"


def test_a_broker_that_is_down_is_reported_not_swallowed(server, operator, robot):
    server.publisher.fail = "broker fleet-box:1883 unreachable"
    body = expect_error(
        lambda: dispatch(server, robot, "goto kitchen", token=operator), 503
    )
    assert "unreachable" in body["error"]
    entry = server.registry.audit()[0]
    assert entry["result"] == "error"
    assert "unreachable" in entry["detail"]


def test_the_audit_route_needs_an_operator_token(server, operator, robot):
    expect_error(lambda: get(server, "/v1/audit"), 401)
    dispatch(server, robot, "goto kitchen", token=operator)
    status, body = get(server, "/v1/audit", token=operator)
    assert status == 200
    assert body["audit"][0]["command"] == "goto kitchen"


def test_audit_can_be_filtered_by_robot(server, operator, robot):
    dispatch(server, robot, "goto kitchen", token=operator)
    _, body = get(server, "/v1/audit?robot_id=mote-02", token=operator)
    assert body["audit"] == []


# ---- basemaps ---------------------------------------------------------------


def test_maps_are_listed_from_the_site_bundles(server):
    status, body = get(server, "/v1/maps")
    assert status == 200
    assert body["maps"] == [{"site": "home", "floor": "ground"}]


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

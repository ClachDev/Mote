"""The enrollment endpoint, exercised over a real socket.

The server is stdlib ``http.server``, so there is nothing to mock: it binds an
ephemeral port and the tests speak HTTP to it. That also covers the bits a unit
test of the registry would miss — status codes, the shape of the JSON a robot
parses, and the fact that a bad request is answered rather than dropped.
"""

import json
import threading
import urllib.error
import urllib.request

import pytest
from fleet_server import serve

from mote_fleet import protocol


@pytest.fixture
def server(tmp_path):
    httpd = serve(
        db=tmp_path / "registry.db",
        host="127.0.0.1",
        port=0,
        broker_host="fleet-box",
        broker_port=1883,
    )
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    httpd.url = f"http://127.0.0.1:{httpd.server_address[1]}"
    yield httpd
    httpd.shutdown()
    httpd.server_close()


def get(server, path):
    with urllib.request.urlopen(server.url + path, timeout=10) as response:
        return response.status, json.loads(response.read())


def post(server, path, payload):
    request = urllib.request.Request(
        server.url + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
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

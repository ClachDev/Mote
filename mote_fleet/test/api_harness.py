"""Shared pieces for the tests that speak HTTP to a real fleet server.

``test_fleet_server.py`` (enrollment, dispatch, audit, basemaps) and
``test_map_registry.py`` (M4's upload → candidate → promote → announce) run
against the same process: a real ``http.server`` on an ephemeral port, with the
broker stubbed because that hop is tested against a real mosquitto in
``test_e2e_fleet.py``. This module is the scaffolding they share, in the same
spirit as ``fleet_harness.py`` — one definition of "the fleet server, minus the
broker" rather than two that drift.

Nothing here needs ROS, so these tests run in the fleet environment too.
"""

import json
import struct
import threading
import urllib.error
import urllib.request
import zlib
from pathlib import Path

from fleet_server import serve

from mote_bringup import bundle
from mote_fleet import protocol


class FakeBroker:
    """Stands in for ``BrokerLink``. ``fail`` is how "the broker is down" is
    tested without taking a broker down."""

    def __init__(self):
        self.published = []
        self.fail = ""

    def publish(self, topic, payload, retain=False):
        if self.fail:
            return False, self.fail
        self.published.append((topic, json.loads(payload), retain))
        return True, ""

    def retained(self, topic):
        """The last retained payload on ``topic``, or None — what a subscriber
        connecting now would be handed."""
        for name, payload, retain in reversed(self.published):
            if name == topic and retain:
                return payload
        return None

    def close(self):
        pass


def write_png(path, width, height, fill=b"\x80", filter_type=0):
    """A real PNG, so the header reader and the occupancy decoder are tested
    against the format rather than against a fixture that agrees with them."""

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    raw = b"".join(bytes([filter_type]) + fill * width for _ in range(height))
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def write_occupancy_png(path, width, height):
    """A trinary occupancy map with a wall, floor and unknown space — i.e. one
    that is not degenerate, which the validator checks for."""

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                row.append(0)  # wall
            elif x < width // 8:
                row.append(205)  # unknown
            else:
                row.append(254)  # free
        rows.append(bytes(row))
    raw = b"".join(b"\x00" + row for row in rows)
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


MAP_YAML = (
    "image: map.png\nmode: trinary\nresolution: 0.050\n"
    "origin: [-2.927, -2.934, 0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
)

ZONES_YAML = (
    "frame_id: map\nvocabulary_revision: 4\nzones:\n"
    "  kitchen: {x: 1.0, y: 2.0, yaw: 0.0, radius: 1.5, kind: room,\n"
    "    display_name: The Kitchen, aliases: [galley]}\n"
    "  ward: {x: 4.0, y: 1.0, yaw: 1.57, kind: room,\n"
    "    polygon: [[3.0, 0.0], [5.0, 0.0], [5.0, 2.0], [3.0, 2.0]]}\n"
    "  sluice: {x: 6.0, y: 6.0, radius: 0.8, kind: keepout}\n"
)


def write_revision(directory, *, width=40, height=30, zones=True, posegraph=True):
    """A complete map revision as ``sites.save_map`` leaves one."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "map.yaml").write_text(MAP_YAML)
    write_occupancy_png(directory / "map.png", width, height)
    (directory / "meta.yaml").write_text(
        "schema: 1\nsaved: '2026-07-27T10:15:00'\nclean:\n  skipped: true\n"
    )
    if posegraph:
        (directory / "map.posegraph").write_bytes(b"posegraph\n")
        (directory / "map.data").write_bytes(b"posedata\n")
    if zones:
        (directory / "zones.yaml").write_text(ZONES_YAML)
    return directory


def write_bundle(root, site, floor, revision="20260726T120000", **kwargs):
    """A site bundle as ``sites.py`` writes one, published symlink and all."""
    revision_dir = write_revision(
        Path(root) / site / "floors" / floor / "maps" / revision, **kwargs
    )
    floor_dir = revision_dir.parent.parent
    link = floor_dir / "map"
    if link.is_symlink():
        link.unlink()
    link.symlink_to(revision_dir.relative_to(floor_dir))
    return revision_dir


def packed_revision(tmp_path, name="rev", **kwargs) -> bytes:
    """One revision packed for upload, without a site bundle around it."""
    return bundle.pack(write_revision(Path(tmp_path) / name, **kwargs))


def start_server(tmp_path, **kwargs):
    """A listening fleet server with a stub broker. Caller shuts it down."""
    maps = Path(tmp_path) / "sites"
    maps.mkdir(exist_ok=True)
    httpd = serve(
        db=Path(tmp_path) / "registry.db",
        host="127.0.0.1",
        port=0,
        broker_host="fleet-box",
        broker_port=1883,
        publisher=kwargs.pop("publisher", None) or FakeBroker(),
        maps_dir=maps,
        **kwargs,
    )
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    httpd.url = f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.maps = maps
    # Every /v1 route needs an operator, so one comes with the harness and
    # ``get`` sends it unless a test says otherwise.
    httpd.token = httpd.registry.new_operator(name="harness")
    return httpd


# -- speaking to it ---------------------------------------------------------

#: Sent when a call names no token. ``token=""`` means "send none", which is
#: what the tests that are about the *absence* of a credential pass.
DEFAULT = object()


def _authorize(request, server, token):
    token = server.token if token is DEFAULT else token
    if token:
        request.add_header("Authorization", f"Bearer {token}")


def get(server, path, token=DEFAULT):
    request = urllib.request.Request(server.url + path)
    _authorize(request, server, token)
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, json.loads(response.read())


def get_bytes(server, path, token=DEFAULT):
    request = urllib.request.Request(server.url + path)
    _authorize(request, server, token)
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, response.headers["Content-Type"], response.read()


def post(server, path, payload, token=None):
    # Unlike ``get``, this defaults to *no* token: enrollment carries its own
    # credential in the body, and every dispatch test is explicit about which
    # operator it is acting as.
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


def post_raw(server, path, blob, content_type="application/json"):
    """POST a body the server is expected to refuse, unparsed."""
    request = urllib.request.Request(
        server.url + path,
        data=blob,
        headers={"Content-Type": content_type},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, response.read()


def post_bytes(server, path, blob, content_type="application/gzip"):
    request = urllib.request.Request(
        server.url + path,
        data=blob,
        headers={"Content-Type": content_type},
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


def expect_error(call, code):
    """Assert a request is refused with ``code``, and return the server's own
    reason — which is half of what these routes promise."""
    try:
        call()
    except urllib.error.HTTPError as exc:
        assert exc.code == code, f"expected {code}, got {exc.code}"
        try:
            return json.loads(exc.read())
        except ValueError:
            return {}
    else:
        raise AssertionError(f"expected HTTP {code}")

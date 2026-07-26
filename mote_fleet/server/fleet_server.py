"""The fleet API: enrollment, the registry, mediated dispatch, and the UI.

This is the process that owns the ``robot_id`` space. A clean robot boots with
a tailnet key and an enrollment token, calls ``POST /v1/enroll``, and is told
who it is and where the control-plane broker lives — which is what supersedes
M0's operator-typed id (fleet.md Q3).

It runs off the robot, on a box that need not have ROS, a checkout, or a GPU,
so it imports nothing but the standard library and the shared wire contract
(``mote_fleet/protocol.py``) — the same arrangement as the inference server and
``depth_wire.py``. ``http.server`` rather than a web framework stays a
deliberate floor: a dozen routes, no templating, no ORM, and one fewer
dependency to solve on whatever the fleet box turns out to be. The UI it serves
is static files, so there is nothing to render server-side.

Routes (all JSON except the UI and the basemap; ``schema`` on every payload).
The contract is ``docs/fleet/fleet-api.md``::

    GET  /healthz                            liveness + how many robots
    GET  /v1/config                          what the browser needs to bootstrap
    GET  /v1/robots                          the roster
    GET  /v1/robots/<id>                     one row
    POST /v1/enroll                          allocate (or return) a robot id
    POST /v1/robots/<id>/dispatch            authorize, audit, then publish
    GET  /v1/audit                           what was dispatched, by whom
    GET  /v1/maps                            basemaps this server can serve
    GET  /v1/maps/<site>/<floor>/map.json    resolution + origin (the Q5 transform)
    GET  /v1/maps/<site>/<floor>/map.png     the basemap image
    GET  /                                   the operator UI (static files)

    pixi run fleet-server -- --db ~/fleet/registry.db --broker-host fleet-box

**Dispatch is mediated here, and only here.** M1's ``fleetctl`` published
straight to the broker; from M3 every write to ``task/command`` goes through
``POST /v1/robots/<id>/dispatch``, which authorizes an operator token, writes an
audit row, and only then publishes (fleet.md Q5/Q7). The topic tree does not
change — only who may publish to it. The browser therefore never holds a broker
credential that can publish, and the UI's MQTT client cannot publish at all
(``ui/mqtt.mjs`` implements no PUBLISH packet).

**Security posture for M3:** the read routes are still unauthenticated, exactly
as M1 left them, and the broker is still anonymous. What M3 adds is a credential
on the *write* path and a record of who used it. That stays proportionate only
while the tailnet is the boundary; M7 is where operator auth reaches the read
routes, per-robot broker credentials land, and Tailscale ACLs stop robots
reaching each other. Until then, do not expose this port to a network the robots
are not already trusted on.
"""

import argparse
import json
import mimetypes
import re
import socket
import sys
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_fleet import protocol  # noqa: E402
from registry import (  # noqa: E402
    ID_PREFIX,
    Registry,
    RegistryError,
    default_db,
    fleet_home,
)

MAX_BODY = 64 * 1024

#: Longest task string the API will forward. The grammar itself belongs to the
#: robot's task layer (fleet.md: "the fleet adds no second grammar"), so this is
#: a bound on the wire, not a parser.
MAX_COMMAND = 200

#: Site and floor names are directory names in a site bundle; a name outside
#: this cannot be a path component, which is the whole path-traversal story.
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")

#: Where the operator UI deep-links for the single-robot view. A robot's
#: MagicDNS name is its id (M0) and ``foxglove_bridge`` listens on 8765 (M2).
FOXGLOVE_URL = "foxglove://open?ds=foxglove-websocket&ds.url=ws://{robot_id}:8765"

UI_DIR = Path(__file__).resolve().parent / "ui"

# The UI is ES modules, and a browser refuses a module served as anything but
# JavaScript. The `.mjs` extension is what lets node run the same files under
# test without a package.json declaring the tree a module.
mimetypes.add_type("text/javascript", ".mjs")

#: The scalar keys of a ``map_saver`` ``map.yaml``, and how to read each.
MAP_KEYS = {
    "image": str,
    "mode": str,
    "resolution": float,
    "negate": int,
    "occupied_thresh": float,
    "free_thresh": float,
}


class BrokerLink:
    """The server's own MQTT connection, used for exactly one thing: publishing
    a command that has already been authorized and recorded.

    paho is imported on first use so the module keeps a stdlib-only import
    surface — a fleet box that only ever serves enrollment never needs it, and
    the tests inject a fake in its place.
    """

    def __init__(self, host: str, port: int = 1883, keepalive: int = 30):
        self.host = host
        self.port = int(port)
        self.keepalive = keepalive
        self._client = None
        self._lock = threading.Lock()

    def _connect(self):
        import paho.mqtt.client as mqtt

        try:
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2, client_id="mote-fleet-api"
            )
        except AttributeError:  # paho 1.x
            client = mqtt.Client(client_id="mote-fleet-api")
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.connect(self.host, self.port, keepalive=self.keepalive)
        client.loop_start()
        return client

    def publish(self, topic: str, payload: bytes) -> tuple[bool, str]:
        """``(published, detail)``. A broker that is down is reported, never
        swallowed: an operator told "dispatched" must know the command left."""
        with self._lock:
            if self._client is None:
                try:
                    self._client = self._connect()
                except (OSError, ValueError, ImportError) as exc:
                    return False, f"broker {self.host}:{self.port} unreachable: {exc}"
            client = self._client
        try:
            info = client.publish(topic, payload, qos=protocol.QOS, retain=False)
            info.wait_for_publish(timeout=10)
        except (OSError, ValueError, RuntimeError) as exc:
            with self._lock:
                self._client = None
            return False, f"publish to {self.host}:{self.port} failed: {exc}"
        if info.rc != 0:
            return False, f"broker refused the publish (rc={info.rc})"
        return True, ""

    def close(self):
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            try:
                client.disconnect()
                client.loop_stop()
            except Exception:
                pass


class Unauthorized(Exception):
    """No usable operator token on a request that writes."""


class FleetHandler(BaseHTTPRequestHandler):
    server_version = "mote-fleet/1"

    # -- plumbing ---------------------------------------------------------

    def _send(self, code: int, payload: dict):
        self._send_bytes(code, "application/json", json.dumps(payload).encode())

    def _send_bytes(self, code: int, content_type: str, body: bytes, **headers):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        for name, value in headers.items():
            self.send_header(name.replace("_", "-"), value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _error(self, code: int, message: str):
        self._send(code, {"schema": protocol.SCHEMA, "error": message})

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length > MAX_BODY:
            raise ValueError(f"request body too large ({length} bytes)")
        try:
            return json.loads(self.rfile.read(length) or b"{}")
        except ValueError as exc:
            raise ValueError(f"body is not JSON: {exc}") from exc

    def log_message(self, fmt, *args):
        # BaseHTTPRequestHandler logs to stderr with its own format; keep the
        # shape but drop the client-port noise so a journal stays readable.
        print(f"{self.address_string()} {fmt % args}", file=sys.stderr)

    def _operator(self) -> dict:
        """The operator behind this request, or raise. Bearer tokens only: a
        credential in a query string ends up in every access log there is."""
        header = self.headers.get("Authorization", "")
        scheme, _, token = header.partition(" ")
        if scheme.lower() != "bearer" or not token.strip():
            raise Unauthorized("an operator token is required (Authorization: Bearer)")
        operator = self.server.registry.operator(token.strip())
        if operator is None:
            raise Unauthorized("unknown or revoked operator token")
        return operator

    # -- routes -----------------------------------------------------------

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        path, _, query = self.path.partition("?")
        path = path.rstrip("/") or "/"
        params = urllib.parse.parse_qs(query)

        if path == "/healthz":
            self._send(
                200,
                {
                    "schema": protocol.SCHEMA,
                    "ok": True,
                    "service": "mote-fleet",
                    "contract": f"{protocol.ROOT}/{protocol.VERSION}",
                    "robots": len(self.server.registry.robots()),
                },
            )
        elif path == "/v1/config":
            self._send(200, self.server.ui_config())
        elif path == "/v1/robots":
            self._send(
                200,
                {"schema": protocol.SCHEMA, "robots": self.server.registry.robots()},
            )
        elif path.startswith("/v1/robots/"):
            robot = self.server.registry.robot(path.rsplit("/", 1)[-1])
            if robot is None:
                self._error(404, "no such robot")
            else:
                self._send(200, {"schema": protocol.SCHEMA, **robot})
        elif path == "/v1/audit":
            self._audit(params)
        elif path == "/v1/maps":
            self._send(
                200, {"schema": protocol.SCHEMA, "maps": self.server.list_maps()}
            )
        elif path.startswith("/v1/maps/"):
            self._map(path[len("/v1/maps/") :])
        elif path.startswith("/v1/"):
            self._error(404, f"no route {path}")
        else:
            self._static(path)

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        try:
            body = self._body()
        except ValueError as exc:
            self._error(400, str(exc))
            return
        if path == "/v1/enroll":
            self._enroll(body)
        elif path.startswith("/v1/robots/") and path.endswith("/dispatch"):
            self._dispatch(path[len("/v1/robots/") : -len("/dispatch")], body)
        else:
            self._error(404, f"no route {path}")

    # -- enrollment -------------------------------------------------------

    def _enroll(self, body: dict):
        token = (body.get("token") or "").strip()
        fingerprint = (body.get("fingerprint") or "").strip()
        requested_id = (body.get("robot_id") or "").strip()
        if not token:
            self._error(401, "an enrollment token is required")
            return
        if not fingerprint:
            self._error(400, "a hardware fingerprint is required")
            return
        if requested_id and not protocol.valid_id(requested_id):
            self._error(400, f"invalid robot id {requested_id!r}")
            return

        try:
            robot, created = self.server.registry.enroll(
                token=token,
                fingerprint=fingerprint,
                facts=body.get("facts") or {},
                name=(body.get("name") or "").strip(),
                site=(body.get("site") or "").strip(),
                requested_id=requested_id,
                prefix=self.server.id_prefix,
            )
        except RegistryError as exc:
            # An unusable token is the one enrollment failure that is a
            # credential problem rather than a request problem.
            code = 401 if "token" in str(exc) else 409
            self._error(code, str(exc))
            return

        print(
            f"enrolled {robot['robot_id']} "
            f"({'new' if created else 'existing'}, {robot['fingerprint']})",
            file=sys.stderr,
        )
        self._send(
            201 if created else 200,
            {
                "schema": protocol.SCHEMA,
                "robot_id": robot["robot_id"],
                "name": robot["name"],
                "site": robot["site"],
                "created": created,
                "enrolled_at": robot["enrolled_at"],
                "broker": {
                    "host": self.server.broker_host,
                    "port": self.server.broker_port,
                },
                "contract": f"{protocol.ROOT}/{protocol.VERSION}",
            },
        )

    # -- dispatch + audit -------------------------------------------------

    def _dispatch(self, robot_id: str, body: dict):
        """Authorize, record, publish — in that order.

        The order is the point. The audit row is written *before* the publish
        and closed with the outcome after, so a command that reached the broker
        can never be missing from the log. A log with an extra "we tried" line
        is the failure mode to prefer.
        """
        registry = self.server.registry
        remote = self.address_string()
        command = " ".join(str(body.get("command") or "").split())
        try:
            operator = self._operator()
        except Unauthorized as exc:
            registry.record(
                actor="anonymous",
                action="dispatch",
                robot_id=robot_id,
                command=command[:MAX_COMMAND],
                result="unauthorized",
                detail=str(exc),
                remote=remote,
            )
            self._error(401, str(exc))
            return

        actor = operator["name"]
        if not command:
            self._error(400, "a command is required")
            return
        if len(command) > MAX_COMMAND:
            self._error(400, f"command longer than {MAX_COMMAND} characters")
            return
        if not protocol.valid_id(robot_id):
            self._error(400, f"invalid robot id {robot_id!r}")
            return
        if registry.robot(robot_id) is None:
            registry.record(
                actor=actor,
                action="dispatch",
                robot_id=robot_id,
                command=command,
                result="rejected",
                detail="no such robot",
                remote=remote,
            )
            self._error(404, "no such robot")
            return

        # The grammar stays the task layer's: `fetch <target> <drop_zone>` and
        # `goto <zone>` are validated by the robot, which answers `rejected:`
        # with a reason the operator can act on. A parser here would be a second
        # grammar to keep in step (fleet.md Q5), and it would reject commands a
        # newer robot understands.
        payload = protocol.command(
            command, issued_by=str(body.get("issued_by") or "").strip() or f"ui:{actor}"
        )
        entry = registry.record(
            actor=actor,
            action="dispatch",
            robot_id=robot_id,
            command=command,
            command_id=payload["id"],
            result="publishing",
            remote=remote,
        )
        published, detail = self.server.publisher.publish(
            protocol.topic(robot_id, protocol.COMMAND), protocol.encode(payload)
        )
        registry.finish(entry["id"], "published" if published else "error", detail)
        print(
            f"dispatch {robot_id} '{command}' by {actor} "
            f"({'published' if published else detail})",
            file=sys.stderr,
        )
        if not published:
            self._error(503, detail)
            return
        self._send(
            202,
            {
                "schema": protocol.SCHEMA,
                "robot_id": robot_id,
                "id": payload["id"],
                "command": payload["command"],
                "issued_at": payload["issued_at"],
                "issued_by": payload["issued_by"],
                "audit_id": entry["id"],
                "status_topic": protocol.topic(robot_id, protocol.STATUS),
            },
        )

    def _audit(self, params: dict):
        try:
            self._operator()
        except Unauthorized as exc:
            self._error(401, str(exc))
            return
        try:
            limit = int((params.get("limit") or ["100"])[0])
        except ValueError:
            self._error(400, "limit must be an integer")
            return
        robot_id = (params.get("robot_id") or [""])[0]
        self._send(
            200,
            {
                "schema": protocol.SCHEMA,
                "audit": self.server.registry.audit(limit=limit, robot_id=robot_id),
            },
        )

    # -- basemaps ---------------------------------------------------------

    def _map(self, rest: str):
        parts = rest.split("/")
        if len(parts) != 3 or parts[2] not in ("map.json", "map.png"):
            self._error(404, "expected /v1/maps/<site>/<floor>/map.json|map.png")
            return
        site, floor, leaf = parts
        if not (NAME_RE.match(site) and NAME_RE.match(floor)):
            self._error(400, "invalid site or floor name")
            return
        try:
            meta = self.server.read_map(site, floor)
        except FileNotFoundError:
            self._error(404, f"no map for {site}/{floor}")
            return
        except ValueError as exc:
            self._error(500, str(exc))
            return
        if leaf == "map.json":
            self._send(
                200,
                {
                    "schema": protocol.SCHEMA,
                    **{k: v for k, v in meta.items() if not k.startswith("_")},
                },
            )
            return
        image = Path(meta["_image_path"])
        self._send_bytes(
            200,
            mimetypes.guess_type(image.name)[0] or "application/octet-stream",
            image.read_bytes(),
            Cache_Control="no-cache",
        )

    # -- the UI -----------------------------------------------------------

    def _static(self, path: str):
        root = self.server.ui_dir
        if root is None:
            self._error(404, f"no route {path}")
            return
        target = (root / ("index.html" if path == "/" else path.lstrip("/"))).resolve()
        if not target.is_relative_to(root) or not target.is_file():
            self._error(404, f"no route {path}")
            return
        content_type = mimetypes.guess_type(target.name)[0] or "text/plain"
        if content_type.startswith("text/") or content_type.endswith("javascript"):
            content_type += "; charset=utf-8"
        self._send_bytes(
            200, content_type, target.read_bytes(), Cache_Control="no-cache"
        )


class FleetServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address,
        registry,
        *,
        broker_host,
        broker_port,
        id_prefix,
        publisher,
        maps_dir,
        ui_dir,
        broker_ws_host,
        broker_ws_port,
        foxglove_url,
    ):
        super().__init__(address, FleetHandler)
        self.registry = registry
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.id_prefix = id_prefix
        self.publisher = publisher
        self.maps_dir = Path(maps_dir).expanduser() if maps_dir else None
        self.ui_dir = Path(ui_dir).resolve() if ui_dir else None
        self.broker_ws_host = broker_ws_host
        self.broker_ws_port = broker_ws_port
        self.foxglove_url = foxglove_url

    # -- what the browser needs to bootstrap ------------------------------

    def ui_config(self) -> dict:
        """Everything the UI cannot work out from its own URL.

        ``ws_host`` is null by default and the page falls back to the host it
        was loaded from, so reaching the fleet box by MagicDNS, by tailnet
        address or over localhost all work with no per-deployment build step.
        """
        return {
            "schema": protocol.SCHEMA,
            "contract": f"{protocol.ROOT}/{protocol.VERSION}",
            "topics": {
                "root": f"{protocol.ROOT}/{protocol.VERSION}",
                "presence": protocol.PRESENCE,
                "health": protocol.HEALTH,
                "pose": protocol.POSE,
                "status": protocol.STATUS,
            },
            "broker": {
                "ws_host": self.broker_ws_host,
                "ws_port": self.broker_ws_port,
                "host": self.broker_host,
                "port": self.broker_port,
            },
            "foxglove_url": self.foxglove_url,
            "maps": bool(self.maps_dir and self.maps_dir.is_dir()),
        }

    # -- basemaps ---------------------------------------------------------

    def list_maps(self) -> list[dict]:
        """The site/floor pairs this server can draw a robot on.

        What it walks is a **site bundle** exactly as ``sites.py`` writes one,
        published symlink and all — which is why M4 can replace the source of
        these bytes with the canonical registry without this route, or the UI,
        changing shape.
        """
        if not self.maps_dir or not self.maps_dir.is_dir():
            return []
        found = []
        for site_dir in sorted(self.maps_dir.iterdir()):
            floors = site_dir / "floors"
            if not floors.is_dir():
                continue
            for floor_dir in sorted(floors.iterdir()):
                if (floor_dir / "map" / "map.yaml").is_file():
                    found.append({"site": site_dir.name, "floor": floor_dir.name})
        return found

    def read_map(self, site: str, floor: str) -> dict:
        """A floor's ``map.yaml`` as JSON, for the Q5 world→pixel transform.

        A hand-rolled reader for the flat scalar file ``map_saver`` writes,
        rather than a YAML dependency on a server whose whole dependency list is
        "python". M4 extracts the shared, ROS-free bundle validator the design
        asks for (fleet.md Q4) and this becomes a call into it.
        """
        if not self.maps_dir:
            raise FileNotFoundError(site)
        path = self.maps_dir / site / "floors" / floor / "map" / "map.yaml"
        if not path.is_file():
            raise FileNotFoundError(str(path))
        meta = {"site": site, "floor": floor}
        for raw in path.read_text().splitlines():
            line = raw.split("#", 1)[0].strip()
            key, sep, value = line.partition(":")
            if not sep:
                continue
            key, value = key.strip(), value.strip()
            try:
                if key == "origin":
                    meta["origin"] = [float(n) for n in value.strip("[]").split(",")]
                elif key in MAP_KEYS:
                    meta[key] = MAP_KEYS[key](value)
            except ValueError as exc:
                raise ValueError(f"{path}: bad {key}: {exc}") from exc
        missing = [k for k in ("resolution", "origin", "image") if k not in meta]
        if missing:
            raise ValueError(f"{path} is missing {', '.join(missing)}")

        image = (path.parent / meta["image"]).resolve()
        if not image.is_file():
            raise ValueError(f"{path} names an image that is not there: {image}")
        meta["_image_path"] = str(image)
        meta["image_url"] = f"/v1/maps/{site}/{floor}/map.png"
        size = png_size(image)
        if size is None:
            raise ValueError(f"{image} is not a PNG this server can measure")
        meta["width"], meta["height"] = size
        return meta


def png_size(path) -> tuple[int, int] | None:
    """Pixel dimensions from a PNG header — the other half of the transform.

    Reading 24 bytes beats making the browser wait for the image to decode
    before it can place a robot, and beats a Pillow dependency for a number that
    lives at a fixed header offset.
    """
    try:
        with open(path, "rb") as handle:
            header = handle.read(24)
    except OSError:
        return None
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    return int.from_bytes(header[16:20], "big"), int.from_bytes(header[20:24], "big")


def default_maps_dir() -> str:
    """Site bundles on the fleet box: ``$MOTE_FLEET_HOME/sites``.

    The same layout ``sites.py`` writes on a robot, so seeding it is an rsync of
    a floor directory until M4 makes the registry the source of truth.
    """
    return str(fleet_home() / "sites")


def serve(
    *,
    db,
    host="0.0.0.0",
    port=8080,
    broker_host=None,
    broker_port=1883,
    id_prefix=ID_PREFIX,
    publisher=None,
    maps_dir=None,
    ui_dir=UI_DIR,
    broker_ws_host=None,
    broker_ws_port=9001,
    foxglove_url=FOXGLOVE_URL,
) -> FleetServer:
    """Build a listening server. The caller runs it (or its ``serve_forever``)."""
    broker_host = broker_host or socket.gethostname()
    return FleetServer(
        (host, port),
        Registry(db),
        broker_host=broker_host,
        broker_port=broker_port,
        id_prefix=id_prefix,
        publisher=publisher or BrokerLink(broker_host, broker_port),
        maps_dir=maps_dir,
        ui_dir=ui_dir,
        broker_ws_host=broker_ws_host,
        broker_ws_port=broker_ws_port,
        foxglove_url=foxglove_url,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="fleet-server", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument(
        "--db",
        default=default_db(),
        help="registry SQLite file (default: $MOTE_FLEET_HOME/registry.db)",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--broker-host",
        help="MQTT host handed to enrolling robots, and the one this server "
        "publishes dispatches to (default: this hostname). On a tailnet this "
        "should be the fleet box's MagicDNS name.",
    )
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument(
        "--broker-ws-host",
        help="MQTT-over-WebSocket host for the browser (default: whichever host "
        "the UI was loaded from, which is right for MagicDNS and for localhost)",
    )
    parser.add_argument(
        "--broker-ws-port",
        type=int,
        default=9001,
        help="MQTT-over-WebSocket port the UI subscribes on (default: 9001)",
    )
    parser.add_argument(
        "--maps-dir",
        default=default_maps_dir(),
        help="site bundles to serve basemaps from (default: $MOTE_FLEET_HOME/sites)",
    )
    parser.add_argument(
        "--foxglove-url",
        default=FOXGLOVE_URL,
        help="deep-link template for the single-robot view; {robot_id} is "
        "substituted. Empty disables the button.",
    )
    parser.add_argument(
        "--no-ui", action="store_true", help="serve the API only, no static files"
    )
    parser.add_argument(
        "--id-prefix", default=ID_PREFIX, help="allocated ids look like <prefix>-01"
    )
    args = parser.parse_args(argv)

    server = serve(
        db=args.db,
        host=args.host,
        port=args.port,
        broker_host=args.broker_host,
        broker_port=args.broker_port,
        broker_ws_host=args.broker_ws_host,
        broker_ws_port=args.broker_ws_port,
        maps_dir=args.maps_dir,
        ui_dir=None if args.no_ui else UI_DIR,
        foxglove_url=args.foxglove_url,
        id_prefix=args.id_prefix,
    )
    print(
        f"mote-fleet on http://{args.host}:{args.port}  db={args.db}  "
        f"broker={server.broker_host}:{server.broker_port}  "
        f"ws={server.broker_ws_host or '<page host>'}:{server.broker_ws_port}  "
        f"maps={len(server.list_maps())}",
        file=sys.stderr,
    )
    if not server.registry.operators():
        print(
            "no operator tokens exist yet — dispatch will refuse every request. "
            "Mint one with: fleetctl operator new --name <you>",
            file=sys.stderr,
        )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        thread.join()
    except KeyboardInterrupt:
        server.shutdown()
        server.publisher.close()


if __name__ == "__main__":
    main()

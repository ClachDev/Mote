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
    GET  /v1/maps/<site>/<floor>/zones.json  the floor's zone *binding*
    GET  /v1/zones                           every floor's zone *vocabulary*
    GET  /v1/zones/<site>/<floor>            one floor's, as a zone/v0 document
    GET  /v1/sites                           the registry: floors + canonical rev
    GET  /v1/sites/<site>/floors/<floor>     revisions, validated, with provenance
    POST     .../revisions/<rev>             upload a candidate revision (robot)
    GET      .../revisions/<rev>/bundle.tar.gz   pull one (robot)
    GET      .../revisions/<rev>/map.json    that revision's own Q5 transform
    GET      .../revisions/<rev>/map.png     that revision's own image
    GET      .../revisions/<rev>/zones.json  that revision's own zone binding
    POST     .../revisions/<rev>/promote     make it canonical (operator)
    POST /v1/sites/<site>/floors/<floor>/zones  edited zones of a revision ->
                                             a new candidate (operator)
    GET  /                                   the operator UI (static files)

    pixi run fleet-server -- --db ~/fleet/registry.db --broker-host fleet-box

**Dispatch is mediated here, and only here.** M1's ``fleetctl`` published
straight to the broker; from M3 every write to ``task/command`` goes through
``POST /v1/robots/<id>/dispatch``, which authorizes an operator token, writes an
audit row, and only then publishes (fleet.md Q5/Q7). The topic tree does not
change — only who may publish to it. The browser therefore never holds a broker
credential that can publish, and the UI's MQTT client cannot publish at all
(``ui/mqtt.mjs`` implements no PUBLISH packet).

**The registry is the source of truth for maps (M4).** A robot uploads a saved
revision as a *candidate*, which changes nothing; an operator promotes one, which
flips the floor's ``map`` symlink and publishes the retained
``registry/site/<site>/floor/<floor>/current`` every agent pulls from. Two robots
mapping one floor therefore end with two candidates and no merge — a map frame's
origin is an accident of where SLAM started, so merging frames would break every
taught zone (fleet.md Q4). The bytes live in :mod:`bundle_store`, and both ends
validate with the *same* ROS-free module the robot writes with
(``mote_bringup.bundle``).

**Names are served; coordinates are not (zone/v0).** The same site bundles hold
the answer to the question a dispatcher actually asks — *what places can I
name?* — and until now the only ways to get it were an out-of-band document or
scraping the list a robot prints when it refuses an unknown zone. ``/v1/zones``
answers it directly, and answers it with a **vocabulary**: names, kinds and
aliases, no coordinates, no frame. That restraint is what makes publishing it
safe. A zone's pose is a coordinate in one robot's map frame, whose origin is an
accident of where its SLAM session started, so it is true for that robot and
false for the one beside it; the name is true for both. The binding stays where
it was, under ``/v1/maps``, served to the client that also has the basemap.

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
import os
import re
import socket
import sys
import traceback
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bundle_store import BundleStore, MAX_UPLOAD, StoreError  # noqa: E402
from mote_bringup import bundle  # noqa: E402  (put on the path by bundle_store)
from mote_fleet import protocol  # noqa: E402
from registry import (  # noqa: E402
    ID_PREFIX,
    Registry,
    RegistryError,
    default_db,
    fleet_home,
)

MAX_BODY = 64 * 1024

#: Which build is answering. Baked into the container image at build time
#: (deploy/Dockerfile), exactly as the inference server's is, so a deploy can
#: gate on *the new version* being the one that came back healthy rather than on
#: something being up. There is no git in the image, so this is the only source
#: of that answer; running from a checkout it is simply unset.
VERSION = os.environ.get("MOTE_VERSION", "unknown")

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

#: Route shape for the registry's per-revision paths. The three review leaves
#: mirror ``/v1/maps/<site>/<floor>/…`` exactly, because they answer the same
#: three questions about a revision that is *not* the floor's canonical one —
#: which is what an operator has to see before promoting it.
REVISION_RE = re.compile(
    r"^(?P<site>[^/]+)/floors/(?P<floor>[^/]+)/revisions/(?P<revision>[^/]+)"
    r"(?P<leaf>/promote|/bundle\.tar\.gz|/map\.json|/map\.png|/zones\.json)?$"
)


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

    def publish(
        self, topic: str, payload: bytes, retain: bool = False
    ) -> tuple[bool, str]:
        """``(published, detail)``. A broker that is down is reported, never
        swallowed: an operator told "dispatched" must know the command left.

        ``retain`` is false for a command (a retained one re-fires on every
        reconnect) and true for the registry's ``current`` announcement, whose
        whole job is to be waiting for a robot that was switched off."""
        with self._lock:
            if self._client is None:
                try:
                    self._client = self._connect()
                except (OSError, ValueError, ImportError) as exc:
                    return False, f"broker {self.host}:{self.port} unreachable: {exc}"
            client = self._client
        try:
            info = client.publish(topic, payload, qos=protocol.QOS, retain=retain)
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
        # `default` is a backstop, not the contract: payloads carrying a value
        # read off disk are typed where they are parsed (bundle._Loader), so
        # what reaches here is already serialisable. Whatever is not, a route
        # answers badly rather than raising through the handler — an
        # unserialisable field otherwise closes the connection with no status
        # line at all, and a client cannot tell that from the server being
        # down.
        body = json.dumps(payload, default=str).encode()
        self._send_bytes(code, "application/json", body)

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
                    "version": VERSION,
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
        elif path == "/v1/zones":
            self._store(lambda store: {"vocabularies": store.vocabularies()})
        elif path.startswith("/v1/zones/"):
            self._vocabulary(path[len("/v1/zones/") :])
        elif path == "/v1/sites":
            self._store(lambda store: {"sites": store.sites()})
        elif path.startswith("/v1/sites/"):
            self._registry_get(path[len("/v1/sites/") :])
        elif path.startswith("/v1/"):
            self._error(404, f"no route {path}")
        else:
            self._static(path)

    def do_POST(self):
        raw_path, _, query = self.path.partition("?")
        path = raw_path.rstrip("/") or "/"
        params = urllib.parse.parse_qs(query)
        # The upload route carries a packed bundle, so it reads its own body:
        # everything else here is JSON and small.
        if path.startswith("/v1/sites/") and not (
            path.endswith("/promote") or path.endswith("/zones")
        ):
            self._upload(path[len("/v1/sites/") :], params)
            return
        try:
            body = self._body()
        except ValueError as exc:
            self._error(400, str(exc))
            return
        if path == "/v1/enroll":
            self._enroll(body)
        elif path.startswith("/v1/robots/") and path.endswith("/dispatch"):
            self._dispatch(path[len("/v1/robots/") : -len("/dispatch")], body)
        elif path.startswith("/v1/sites/") and path.endswith("/promote"):
            self._promote(path[len("/v1/sites/") :], body)
        elif path.startswith("/v1/sites/") and path.endswith("/zones"):
            self._edit_zones(path[len("/v1/sites/") : -len("/zones")], body)
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
        leaves = ("map.json", "map.png", "zones.json")
        if len(parts) != 3 or parts[2] not in leaves:
            self._error(404, f"expected /v1/maps/<site>/<floor>/{'|'.join(leaves)}")
            return
        site, floor, leaf = parts
        if not self._names(site, floor):
            return
        if leaf == "zones.json":
            self._store(lambda store: store.read_zones(site, floor))
            return
        self._send_map(leaf, lambda store: store.read_map(site, floor))

    def _send_map(self, leaf: str, load):
        """``map.json`` or ``map.png`` from one ``read_map`` call.

        Shared by the canonical basemap and by a revision under review, so the
        transform and the pixels a client is handed can never come from
        different reads of the store.
        """
        try:
            meta = load(self.server.store)
        except StoreError as exc:
            self._error(exc.code, str(exc))
            return
        except bundle.BundleError as exc:
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

    # -- the zone vocabulary ----------------------------------------------

    def _vocabulary(self, rest: str):
        """``/v1/zones/<site>/<floor>`` — what places can be named here.

        The split zone/v0 asks for is expressed by the route, which is why this
        is not another leaf under ``/v1/maps``. Everything under that prefix is
        bound to a basemap and is only true for the robot that taught it; this
        is bound to nothing, and is true for every robot at the site. A caller
        that must never be handed a map — an MCP front door turning "take it to
        the kitchen" into ``goto kitchen`` — can be given this and only this.
        """
        parts = rest.split("/")
        if len(parts) != 2:
            self._error(404, "expected /v1/zones/<site>/<floor>")
            return
        site, floor = parts
        if not self._names(site, floor):
            return
        self._store(lambda store: store.read_vocabulary(site, floor))

    # -- the map registry -------------------------------------------------

    def _names(self, *names) -> bool:
        """Site/floor/revision names are directory names in a site bundle; a
        name outside NAME_RE cannot be a path component, which is the whole
        path-traversal story."""
        if all(NAME_RE.match(name or "") for name in names):
            return True
        self._error(400, "invalid site, floor or revision name")
        return False

    def _store(self, call):
        """Run a registry read and answer it, turning refusals into statuses."""
        try:
            payload = call(self.server.store)
        except StoreError as exc:
            self._error(exc.code, str(exc))
            return
        except bundle.BundleError as exc:
            self._error(500, str(exc))
            return
        self._send(200, {"schema": protocol.SCHEMA, **payload})

    def _registry_get(self, rest: str):
        parts = rest.split("/")
        if len(parts) == 3 and parts[1] == "floors":
            site, _, floor = parts
            if self._names(site, floor):
                self._store(lambda store: store.detail(site, floor))
            return
        match = REVISION_RE.match(rest)
        # `/promote` is a POST, and a bare revision path has nothing to answer:
        # both are 404 here rather than falling through to the bundle.
        if not match or match.group("leaf") in (None, "/promote"):
            self._error(404, f"no route /v1/sites/{rest}")
            return
        site, floor = match.group("site"), match.group("floor")
        revision = match.group("revision")
        if not self._names(site, floor, revision):
            return
        leaf = match.group("leaf").lstrip("/")
        if leaf == "zones.json":
            self._store(lambda store: store.read_revision_zones(site, floor, revision))
            return
        if leaf in ("map.json", "map.png"):
            self._send_map(leaf, lambda store: store.read_map(site, floor, revision))
            return
        try:
            blob = self.server.store.pack(site, floor, revision)
        except StoreError as exc:
            self._error(exc.code, str(exc))
            return
        except bundle.BundleError as exc:
            self._error(500, str(exc))
            return
        self._send_bytes(
            200,
            "application/gzip",
            blob,
            Cache_Control="no-cache",
            # The digest the retained announcement carries, so a puller can
            # check the bytes it got without a second request.
            X_Bundle_Sha256=bundle.digest(blob),
        )

    def _upload(self, rest: str, params: dict):
        """Take one candidate revision from a robot.

        Deliberately **not** operator-authenticated, and deliberately inert: a
        candidate changes nothing about any floor until it is promoted, and the
        route that does change something is the operator's. What is required is
        that the uploader name an enrolled robot, so the artifact has a subject
        in the audit log. M7 replaces that with a per-robot credential.
        """
        match = REVISION_RE.match(rest)
        if not match or match.group("leaf"):
            self._error(
                404, "expected POST /v1/sites/<site>/floors/<floor>/revisions/<rev>"
            )
            return
        site, floor = match.group("site"), match.group("floor")
        revision = match.group("revision")
        if not self._names(site, floor, revision):
            return
        robot_id = (params.get("robot_id") or [""])[0]
        if not protocol.valid_id(robot_id):
            self._error(400, "a robot_id query parameter is required")
            return
        if self.server.registry.robot(robot_id) is None:
            self._error(404, f"no robot {robot_id} in this fleet")
            return
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            self._error(400, "the bundle body is empty")
            return
        if length > MAX_UPLOAD:
            self._error(413, f"bundle is larger than {MAX_UPLOAD} bytes")
            return
        blob = self.rfile.read(length)

        registry = self.server.registry
        entry = registry.record(
            actor=robot_id,
            action="map.upload",
            robot_id=robot_id,
            command=f"{site}/{floor}/{revision}",
            result="receiving",
            remote=self.address_string(),
        )
        try:
            stored, report = self.server.store.accept(
                site, floor, revision, blob, robot_id=robot_id
            )
        except StoreError as exc:
            registry.finish(entry["id"], "rejected", str(exc))
            self._send(
                exc.code, {"schema": protocol.SCHEMA, "error": str(exc), **exc.detail}
            )
            return
        except Exception as exc:
            # An audit row opened as 'receiving' has to be closed on every
            # path, or an upload that crashes the handler leaves a row that
            # says the transfer is still in progress for ever.
            registry.finish(entry["id"], "failed", f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            self._error(500, "the bundle could not be stored")
            return
        registry.finish(entry["id"], "stored", "; ".join(report.warnings))
        print(
            f"map upload {site}/{floor}/{stored} from {robot_id} "
            f"({len(blob)} bytes, candidate)",
            file=sys.stderr,
        )
        self._send(
            201,
            {
                "schema": protocol.SCHEMA,
                "site": site,
                "floor": floor,
                "revision": stored,
                "canonical": self.server.store.canonical(site, floor),
                "promoted": False,
                "warnings": report.warnings,
                "url": self.server.store.bundle_url(site, floor, stored),
            },
        )

    def _promote(self, rest: str, body: dict):
        """Make a candidate canonical: authorize, flip, announce, record.

        The flip is the fact and the announcement is best effort — a broker
        that is down must not leave a floor half-promoted, so the response says
        plainly whether the fleet was told, and the server re-announces every
        floor at startup so a missed announcement heals itself.
        """
        match = REVISION_RE.match(rest)
        if not match or match.group("leaf") != "/promote":
            self._error(404, "expected POST .../revisions/<rev>/promote")
            return
        site, floor = match.group("site"), match.group("floor")
        revision = match.group("revision")
        registry = self.server.registry
        target = f"{site}/{floor}/{revision}"
        try:
            operator = self._operator()
        except Unauthorized as exc:
            registry.record(
                actor="anonymous",
                action="map.promote",
                command=target,
                result="unauthorized",
                detail=str(exc),
                remote=self.address_string(),
            )
            self._error(401, str(exc))
            return
        if not self._names(site, floor, revision):
            return
        actor = operator["name"]
        entry = registry.record(
            actor=actor,
            action="map.promote",
            command=target,
            result="promoting",
            remote=self.address_string(),
        )
        try:
            promoted = self.server.store.promote(site, floor, revision, by=actor)
        except StoreError as exc:
            registry.finish(entry["id"], "rejected", str(exc))
            self._send(
                exc.code, {"schema": protocol.SCHEMA, "error": str(exc), **exc.detail}
            )
            return
        except Exception as exc:
            # An audit row opened as 'promoting' has to be closed on every
            # path, or a promote that crashes the handler leaves a row that
            # says the promotion is still in progress for ever.
            registry.finish(entry["id"], "failed", f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            self._error(500, "the revision could not be promoted")
            return
        announced, detail = self.server.announce(promoted)
        registry.finish(
            entry["id"], "promoted" if announced else "announce-failed", detail
        )
        print(
            f"promoted {target} by {actor} ({'announced' if announced else detail})",
            file=sys.stderr,
        )
        self._send(
            200,
            {
                "schema": protocol.SCHEMA,
                **promoted,
                "announced": announced,
                "detail": detail,
                "topic": protocol.registry_topic(site, floor),
                "audit_id": entry["id"],
            },
        )

    def _edit_zones(self, rest: str, body: dict):
        """An operator's zone edit: derive a candidate from a revision.

        The edit writes nothing the fleet can see — the result is an ordinary
        candidate (same map bytes, new zones), validated like any upload and
        inert until the operator promotes it through the existing route. That
        keeps promoted revisions immutable, which the announced digests rely
        on, and keeps promotion the only write that changes a floor.

        The body's optional ``revision`` is the revision being edited; without
        one the canonical map is edited, as before. It is a body field rather
        than a path segment because the edited revision is an *input* to the
        derivation and never the thing written — the route's own resource is
        the floor's zones, and the result is a revision id neither end chose.
        """
        parts = rest.split("/")
        if len(parts) != 3 or parts[1] != "floors":
            self._error(404, "expected POST /v1/sites/<site>/floors/<floor>/zones")
            return
        site, _, floor = parts
        registry = self.server.registry
        target = f"{site}/{floor}"
        try:
            operator = self._operator()
        except Unauthorized as exc:
            registry.record(
                actor="anonymous",
                action="map.zones",
                command=target,
                result="unauthorized",
                detail=str(exc),
                remote=self.address_string(),
            )
            self._error(401, str(exc))
            return
        source = str(body.get("revision") or "")
        if not self._names(site, floor, *([source] if source else [])):
            return
        zones = body.get("zones")
        if not isinstance(zones, dict):
            self._error(400, "a zones mapping is required: {zones: {name: {...}}}")
            return
        actor = operator["name"]
        # The audit row names what was edited, not only which floor: two
        # candidates of one floor are two different maps, and "who renamed the
        # rooms on this map" is unanswerable from the floor alone.
        entry = registry.record(
            actor=actor,
            action="map.zones",
            command=f"{target}/{source}" if source else target,
            result="editing",
            remote=self.address_string(),
        )
        try:
            stored, report, derived_from = self.server.store.derive_zones(
                site, floor, zones, by=actor, source=source
            )
        except StoreError as exc:
            registry.finish(entry["id"], "rejected", str(exc))
            self._send(
                exc.code, {"schema": protocol.SCHEMA, "error": str(exc), **exc.detail}
            )
            return
        except Exception as exc:
            # An audit row opened as 'editing' has to be closed on every path,
            # or an edit that crashes the handler leaves a row that says the
            # edit is still in progress for ever.
            registry.finish(entry["id"], "failed", f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            self._error(500, "the zone edit could not be stored")
            return
        registry.finish(
            entry["id"], "stored", f"candidate {stored} from {derived_from}"
        )
        print(
            f"zone edit {target} by {actor}: candidate {stored} (from {derived_from})",
            file=sys.stderr,
        )
        self._send(
            201,
            {
                "schema": protocol.SCHEMA,
                "site": site,
                "floor": floor,
                "revision": stored,
                "derived_from": derived_from,
                "promoted": False,
                "warnings": report.warnings,
                "audit_id": entry["id"],
            },
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
        self.store = BundleStore(self.maps_dir)
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
            "registry": self.store.enabled,
        }

    # -- basemaps ---------------------------------------------------------

    def list_maps(self) -> list[dict]:
        """The site/floor pairs this server can draw a robot on.

        Same route and same shape as M3, now answered from the registry rather
        than from whatever an operator last rsynced onto the box: each entry
        carries the canonical revision it is showing, which is the one thing a
        viewer could not work out for itself.
        """
        return [
            {
                "site": floor["site"],
                "floor": floor["floor"],
                "revision": floor["canonical"],
                "candidates": len(floor["candidates"]),
            }
            for floor in self.store.sites()
            if floor["canonical"]
        ]

    # -- announcing the canonical revision --------------------------------

    def announce(self, promoted: dict) -> tuple[bool, str]:
        """Publish a floor's canonical revision, retained.

        Retained is the mechanism, not a detail: an agent that was offline
        through the whole mapping session is handed this the moment it
        reconnects, so map distribution has no polling and no missed-update
        case (fleet.md Q4).
        """
        payload = protocol.current(
            promoted["site"],
            promoted["floor"],
            promoted["revision"],
            url=promoted["url"],
            sha256=promoted.get("sha256", ""),
            bytes_=promoted.get("bytes", 0),
            promoted_by=promoted.get("promoted_by", ""),
        )
        return self.publisher.publish(
            protocol.registry_topic(promoted["site"], promoted["floor"]),
            protocol.encode(payload),
            retain=True,
        )

    def announce_all(self) -> tuple[int, bool]:
        """Re-announce every floor's canonical revision from what is on disk.

        Returns ``(announced, complete)`` — ``complete`` is False when a publish
        failed, which means the broker is unreachable rather than that a
        particular floor is bad, so the caller should retry the whole sweep.

        Run at startup: the symlink is the truth about what is canonical, so a
        promotion that could not reach the broker at the time — or a broker
        whose retained state was lost with its volume — is repaired by the next
        restart rather than leaving robots pinned to an old map forever.
        """
        announced = 0
        for floor in self.store.sites():
            if not floor["canonical"]:
                continue
            try:
                blob = self.store.pack(
                    floor["site"], floor["floor"], floor["canonical"]
                )
            except (StoreError, bundle.BundleError, OSError) as exc:
                print(
                    f"cannot announce {floor['site']}/{floor['floor']}: {exc}",
                    file=sys.stderr,
                )
                continue
            try:
                ok, detail = self.announce(
                    {
                        "site": floor["site"],
                        "floor": floor["floor"],
                        "revision": floor["canonical"],
                        "url": self.store.bundle_url(
                            floor["site"], floor["floor"], floor["canonical"]
                        ),
                        "sha256": bundle.digest(blob),
                        "bytes": len(blob),
                        "promoted_by": "",
                    }
                )
            except Exception as exc:
                # This runs on a daemon thread at startup, where an exception
                # is a silent death and every floor stays stale. A publisher
                # that raises means the same thing as one that returns False.
                ok, detail = False, f"{type(exc).__name__}: {exc}"
            if ok:
                announced += 1
            else:
                print(
                    f"cannot announce {floor['site']}/{floor['floor']}: {detail}",
                    file=sys.stderr,
                )
                return announced, False
        return announced, True

    def announce_all_until_delivered(
        self, attempts: int = 8, first_delay: float = 1.0
    ) -> int:
        """:meth:`announce_all`, retried while the broker is unreachable.

        The normal compose start races: this server and mosquitto come up
        together, so the first publish usually fails. One attempt would leave
        the retained topics stale until somebody restarted the server, which is
        the opposite of the self-repair this exists to provide. Backs off to
        roughly two minutes total and then gives up loudly — by then the broker
        is not merely slow.
        """
        delay = first_delay
        for attempt in range(1, attempts + 1):
            announced, complete = self.announce_all()
            if complete:
                if announced:
                    print(f"re-announced {announced} floor(s)", file=sys.stderr)
                return announced
            if attempt < attempts:
                print(
                    f"announce incomplete (attempt {attempt}/{attempts}), "
                    f"retrying in {delay:.0f}s",
                    file=sys.stderr,
                )
                time.sleep(delay)
                delay = min(delay * 2, 30.0)
        print(
            "gave up re-announcing canonical maps — robots may pull a stale "
            "revision until this server restarts",
            file=sys.stderr,
        )
        return 0


def default_maps_dir() -> str:
    """The registry's bytes: ``$MOTE_FLEET_HOME/sites``.

    The same layout ``sites.py`` writes on a robot — which is what lets a
    revision be distributed by copying a directory and flipping a link, and
    what lets a floor an operator rsynced onto the box before M4 keep working
    unchanged.
    """
    return str(fleet_home() / "sites")


def serve(
    *,
    db,
    host="0.0.0.0",
    port=8080,
    broker_host=None,
    broker_port=1883,
    publish_host=None,
    publish_port=None,
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
        publisher=publisher
        or BrokerLink(publish_host or broker_host, publish_port or broker_port),
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
        "--publish-host",
        help="where THIS SERVER reaches the broker, when that is not the "
        "address robots use (default: --broker-host). The deployed stack sets "
        "it: inside a compose network the broker is a service name on 1883, "
        "while robots dial the box's MagicDNS name on the published port.",
    )
    parser.add_argument("--publish-port", type=int, help="port for --publish-host")
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
        help="the map registry's site bundles (default: $MOTE_FLEET_HOME/sites)",
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
        publish_host=args.publish_host,
        publish_port=args.publish_port,
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
    # Reconcile the retained registry topics with what is actually published on
    # disk. In its own thread because the broker may be starting alongside us
    # and the API must not wait for it.
    threading.Thread(target=server.announce_all_until_delivered, daemon=True).start()
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

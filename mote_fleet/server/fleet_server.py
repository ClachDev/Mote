"""The fleet server's enrollment + registry API (no ROS, stdlib only).

This is the process that owns the ``robot_id`` space. A clean robot boots with
a tailnet key and an enrollment token, calls ``POST /v1/enroll``, and is told
who it is and where the control-plane broker lives — which is what supersedes
M0's operator-typed id (fleet.md Q3).

It runs off the robot, on a box that need not have ROS, a checkout, or a GPU,
so it imports nothing but the standard library and the shared wire contract
(``mote_fleet/protocol.py``) — the same arrangement as the inference server and
``depth_wire.py``. ``http.server`` rather than a web framework is a deliberate
floor, not an aspiration: five routes, no templating, no auth story yet, and one
fewer dependency to solve on whatever the fleet box turns out to be. M3 puts the
dispatch API and the operator UI on top, and that is the point at which a
framework earns its keep.

Routes (all JSON; ``schema`` on every payload)::

    GET  /healthz            liveness + how many robots are enrolled
    GET  /v1/robots          the roster
    GET  /v1/robots/<id>     one row
    POST /v1/enroll          allocate (or return) a robot id

    pixi run fleet-server -- --db ~/fleet/registry.db --broker-host fleet-box

**Security posture for M1:** no authentication on the read routes, and the
enrollment token is the only credential anywhere. That is proportionate exactly
as long as the server is reachable only over the tailnet, which is the M0
substrate this milestone assumes. M7 is where operator auth, per-robot broker
credentials, and Tailscale ACLs land; until then, do not expose this port to a
network the robots are not already trusted on.
"""

import argparse
import json
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_fleet import protocol  # noqa: E402
from registry import ID_PREFIX, Registry, RegistryError, default_db  # noqa: E402

MAX_BODY = 64 * 1024


class FleetHandler(BaseHTTPRequestHandler):
    server_version = "mote-fleet/1"

    # -- plumbing ---------------------------------------------------------

    def _send(self, code: int, payload: dict):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
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

    # -- routes -----------------------------------------------------------

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
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
        else:
            self._error(404, f"no route {path}")

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path != "/v1/enroll":
            self._error(404, f"no route {path}")
            return
        try:
            body = self._body()
        except ValueError as exc:
            self._error(400, str(exc))
            return
        self._enroll(body)

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


class FleetServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, registry, *, broker_host, broker_port, id_prefix):
        super().__init__(address, FleetHandler)
        self.registry = registry
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.id_prefix = id_prefix


def serve(
    *,
    db,
    host="0.0.0.0",
    port=8080,
    broker_host=None,
    broker_port=1883,
    id_prefix=ID_PREFIX,
) -> FleetServer:
    """Build a listening server. The caller runs it (or its ``serve_forever``)."""
    return FleetServer(
        (host, port),
        Registry(db),
        broker_host=broker_host or socket.gethostname(),
        broker_port=broker_port,
        id_prefix=id_prefix,
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
        help="MQTT host handed to enrolling robots (default: this hostname). "
        "On a tailnet this should be the fleet box's MagicDNS name.",
    )
    parser.add_argument("--broker-port", type=int, default=1883)
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
        id_prefix=args.id_prefix,
    )
    print(
        f"mote-fleet on http://{args.host}:{args.port}  db={args.db}  "
        f"broker={server.broker_host}:{server.broker_port}",
        file=sys.stderr,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        thread.join()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    main()

"""Bring up a private fleet, drive the dashboard in a real browser, tear it down.

    pixi run fleet-ui-check              # assert; exit 0/1
    pixi run fleet-ui-check -- --keep    # leave it up and print the URL

M3 verified the dashboard against a real browser (``browser_check.mjs``), but
the stack it ran against was assembled by hand and the robots behind it were a
throwaway script — so the checks were repeatable only by whoever had the
scratch directory. This is that setup, committed: a broker, a fleet server, a
map, enrolled robots and an operator token, all on ports nobody else is using,
around the same browser assertions.

**Nothing here touches the workstation's own fleet.** The broker is a container
on ports picked from the ephemeral range (the workstation usually already has
one on 1883 serving a real robot), the registry and the basemaps live in a
temporary directory rather than ``~/.mote-fleet``, and every process is started
in its own session so teardown reaps this stack and nothing else — the same
scoping rule the sim smoke test settled on.

**Why it is not a pytest.** It needs a docker (conda-forge's mosquitto is built
without websockets, measured again at 2.0.20, and the browser's read path is
MQTT-over-WebSockets) and a chrome. See ``docs/fleet/m3-verification.md`` §2 for
where that leaves CI.

The robots are ``fake_robots.py`` — the control-plane contract and nothing else,
which is exactly what the UI consumes. The *real* agent and behaviour tree are
covered by ``test_e2e_fleet.py``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SERVER_DIR = REPO / "mote_fleet" / "server"

# The registry and the wire contract, from the source tree — the same import
# the fleet server itself does, and the reason this needs no ROS.
sys.path.insert(0, str(SERVER_DIR))
sys.path.insert(0, str(HERE.parent))

#: A committed site bundle to draw the robots on. The sim's, because it is the
#: only real map (map.yaml + PNG + zones, saved by ``sites.py``) in the tree —
#: a synthetic one would exercise neither the origin/resolution transform at a
#: real scale nor the zone overlay.
DEFAULT_SITE = "office_world"
SIM_SITES = REPO / "mote_simulation" / "sim_home" / "sites"

#: What the fake fleet looks like: two robots reporting, one that dropped its
#: link so the broker published its will.
ROBOTS = ("ok", "degraded", "offline")

#: Where the broker image tag is pinned — once, in the compose file, for every
#: broker the fleet runs (``broker.sh`` defers to it the same way). A tag of its
#: own here would be a third broker that could drift onto a mosquitto whose
#: websockets support differs, which is precisely the failure this check exists
#: to catch. ``test_deploy_config.py`` fails if one reappears.
COMPOSE = REPO / "mote_fleet" / "deploy" / "docker-compose.yml"
IMAGE_PIN = re.compile(r"^ *image: *\$\{MOTE_BROKER_IMAGE:-([^}]*)\}", re.M)


def broker_image() -> str:
    override = os.environ.get("MOTE_BROKER_IMAGE")
    if override:
        return override
    found = IMAGE_PIN.findall(COMPOSE.read_text())
    if not found:
        raise RuntimeError(
            f"cannot read the broker image pin from {COMPOSE} — set "
            "MOTE_BROKER_IMAGE, or repair that file"
        )
    return found[0]


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def wait_for_port(port: int, what: str, timeout: float = 30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.2)
    raise RuntimeError(f"{what} never came up on port {port}")


def wait_for_http(url: str, what: str, timeout: float = 30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=2) as response:
                if response.status == 200:
                    return json.loads(response.read())
        except (urllib.error.URLError, OSError, ValueError):
            time.sleep(0.2)
    raise RuntimeError(f"{what} never answered {url}")


def post(url: str, payload: dict, token: str = "") -> dict:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.loads(response.read())


class Stack:
    """The processes this run owns, and the one way to stop all of them."""

    def __init__(self, root: Path):
        self.root = root
        self.processes: list[tuple[str, subprocess.Popen]] = []
        self.container = ""

    def spawn(self, name: str, argv: list[str], **kwargs) -> subprocess.Popen:
        # start_new_session: the process group is the exact teardown scope, so
        # stopping this stack can never reach another run's broker or server.
        process = subprocess.Popen(argv, start_new_session=True, **kwargs)
        self.processes.append((name, process))
        return process

    def broker(self, port: int, ws_port: int, image: str) -> str:
        """A container mosquitto on ports of our own, from the shipped config.

        The config is the deployed one with its two listener lines rewritten,
        rather than a second config that could drift from what a fleet box runs.
        """
        conf = (SERVER_DIR / "mosquitto.conf").read_text()
        conf = conf.replace("\nlistener 1883\n", f"\nlistener {port}\n")
        conf = conf.replace("\nlistener 9001\n", f"\nlistener {ws_port}\n")
        # Retained state is this run's alone: a fresh temp directory every time
        # means yesterday's robots never appear in today's roster.
        conf = conf.replace("persistence true", "persistence false")
        path = self.root / "mosquitto.conf"
        path.write_text(conf)

        self.container = f"mote-ui-check-{os.getpid()}"
        self.spawn(
            "broker",
            [
                "docker",
                "run",
                "--rm",
                "--name",
                self.container,
                "--network",
                "host",
                "-v",
                f"{path}:/mosquitto/config/mosquitto.conf:ro",
                image,
                # Named explicitly, as broker.sh does, rather than trusting the
                # image's default command to keep reading that path.
                "mosquitto",
                "-c",
                "/mosquitto/config/mosquitto.conf",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        wait_for_port(port, "the broker")
        wait_for_port(ws_port, "the broker's websocket listener")
        return self.container

    def stop(self):
        if self.container:
            subprocess.run(
                ["docker", "rm", "-f", self.container],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        for _name, process in reversed(self.processes):
            if process.poll() is not None:
                continue
            try:
                os.killpg(process.pid, 15)
            except (ProcessLookupError, PermissionError):
                process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, 9)


def preflight(args) -> list[str]:
    """What is missing, in the words of what to do about it."""
    missing = []
    if not shutil.which("docker"):
        missing.append(
            "docker — the browser's read path is MQTT-over-WebSockets and "
            "conda-forge's mosquitto is built without them"
        )
    if not shutil.which("node"):
        missing.append("node — browser_check.mjs speaks the DevTools protocol")
    if not (
        os.environ.get("CHROME") and shutil.which(os.environ["CHROME"])
    ) and not any(
        shutil.which(name) for name in ("google-chrome", "chromium", "chromium-browser")
    ):
        missing.append("a chrome/chromium on PATH (or $CHROME pointing at one)")
    if not (SIM_SITES / args.site).is_dir():
        missing.append(f"a site bundle at {SIM_SITES / args.site}")
    return missing


def seed_maps(root: Path, site: str) -> Path:
    """Copy the basemap in, rather than serving it out of the checkout.

    The registry writes to its maps directory (it re-announces floors, and an
    operator can promote from the UI), and a verification run must not leave
    anything in the git tree.
    """
    maps = root / "sites"
    maps.mkdir()
    shutil.copytree(SIM_SITES / site, maps / site, symlinks=True)
    return maps


def published_revision(maps: Path, site: str, floor: str) -> str:
    """What the floor's ``map`` symlink points at — the revision the registry
    will call canonical, and so the one the robots should report running."""
    link = maps / site / "floors" / floor / "map"
    return os.path.basename(os.readlink(link)) if link.is_symlink() else ""


def enroll_fleet(url: str, registry, count: int) -> list[str]:
    """Enrol through the real route, so the roster is the registry's own.

    The ids come back allocated (``mote-01``, ``mote-02``, …) rather than being
    asserted here: dispatch 404s on a robot the registry has never seen, so a
    fake fleet that skipped this would fail the one check that writes.
    """
    ids = []
    for index in range(count):
        answer = post(
            f"{url}/v1/enroll",
            {
                "schema": 1,
                "token": registry.new_token(note="fleet-ui-check"),
                "fingerprint": f"ui-check-{index}",
                "name": f"fake {index + 1}",
                "facts": {"model": "wire-only", "harness": "fleet-ui-check"},
            },
        )
        ids.append(answer["robot_id"])
    return ids


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="fleet-ui-check", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument(
        "--site",
        default=DEFAULT_SITE,
        help=f"site bundle to serve (default: {DEFAULT_SITE})",
    )
    parser.add_argument("--floor", default="ground")
    parser.add_argument(
        "--keep",
        action="store_true",
        help="skip the browser checks; leave the stack up and print how to reach it",
    )
    parser.add_argument(
        "--screenshot",
        default="fleet-ui.png",
        help="where browser_check.mjs writes its screenshot",
    )
    parser.add_argument(
        "--image",
        default=None,
        help="broker container image (default: the compose file's pin, or "
        "$MOTE_BROKER_IMAGE)",
    )
    args = parser.parse_args(argv)

    args.image = args.image or broker_image()

    missing = preflight(args)
    if missing:
        print("fleet-ui-check needs:", file=sys.stderr)
        for item in missing:
            print(f"  - {item}", file=sys.stderr)
        return 2

    from registry import Registry  # noqa: E402  (server dir is on sys.path)

    mqtt_port, ws_port, http_port = free_port(), free_port(), free_port()
    root = Path(tempfile.mkdtemp(prefix="mote-ui-check-"))
    stack = Stack(root)
    url = f"http://127.0.0.1:{http_port}"
    try:
        print(f"broker:  {args.image} on {mqtt_port} (mqtt) / {ws_port} (ws)")
        stack.broker(mqtt_port, ws_port, args.image)

        db = root / "registry.db"
        maps = seed_maps(root, args.site)
        stack.spawn(
            "fleet-server",
            [
                sys.executable,
                "-u",
                str(SERVER_DIR / "fleet_server.py"),
                "--db",
                str(db),
                "--host",
                "127.0.0.1",
                "--port",
                str(http_port),
                "--broker-host",
                "127.0.0.1",
                "--broker-port",
                str(mqtt_port),
                "--broker-ws-port",
                str(ws_port),
                "--maps-dir",
                str(maps),
            ],
        )
        wait_for_http(f"{url}/healthz", "the fleet server")
        print(f"server:  {url}  (state in {root})")

        registry = Registry(str(db))
        ids = enroll_fleet(url, registry, len(ROBOTS))
        token = registry.new_operator(name="fleet-ui-check")
        print(f"robots:  {', '.join(ids)}  on {args.site}/{args.floor}")

        stack.spawn(
            "fake-robots",
            [
                sys.executable,
                "-u",
                str(HERE / "fake_robots.py"),
                "--host",
                "127.0.0.1",
                "--port",
                str(mqtt_port),
                "--site",
                args.site,
                "--floor",
                args.floor,
                "--revision",
                published_revision(maps, args.site, args.floor),
                *sum(
                    (
                        ["--robot", f"{name}:{profile}"]
                        for name, profile in zip(ids, ROBOTS)
                    ),
                    [],
                ),
            ],
        )
        # The dashboard reads retained state, so the robots must have published
        # before the browser connects — otherwise the roster check is a race.
        # Waiting on the broker rather than on a sleep also settles the will:
        # the dropped robot's retained presence is the broker's own doing, and
        # if it never arrives the fixture is not modelling what it claims to.
        wait_for_retained(mqtt_port, ids, dropped=ids[ROBOTS.index("offline")])

        if args.keep:
            print(f"\n  open     {url}")
            print(f"  token    {token}")
            print(f"  broker   ws://127.0.0.1:{ws_port}")
            print("\nCtrl-C to tear it all down.")
            try:
                while True:
                    time.sleep(3600)
            except KeyboardInterrupt:
                print()
            return 0

        print()
        result = subprocess.run(
            [
                "node",
                str(HERE / "browser_check.mjs"),
                url,
                token,
                str(Path(args.screenshot).resolve()),
            ]
        )
        return result.returncode
    except (RuntimeError, urllib.error.URLError, KeyboardInterrupt) as exc:
        print(f"fleet-ui-check: {exc}", file=sys.stderr)
        return 2
    finally:
        stack.stop()
        shutil.rmtree(root, ignore_errors=True)


def wait_for_retained(
    port: int, ids: list[str], *, dropped: str, timeout: float = 30.0
):
    """Wait for the state the page will be handed on connect.

    Subscribing here is the same thing the browser does a moment later, so what
    this returns on is exactly what the roster, the health roll-up and the map
    are about to be drawn from — including the *offline* presence the broker
    publishes on the dropped robot's behalf.
    """
    import paho.mqtt.client as mqtt

    from mote_fleet import protocol

    seen: dict[tuple[str, str], dict] = {}

    def collect(_client, _userdata, message):
        parsed = protocol.parse_topic(message.topic)
        if parsed:
            seen[parsed] = json.loads(message.payload)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ui-check-wait")
    client.on_message = collect
    client.connect("127.0.0.1", port, keepalive=30)
    client.subscribe(f"{protocol.ROOT}/{protocol.VERSION}/#", qos=protocol.QOS)
    client.loop_start()
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            healthy = all(
                (robot, "health") in seen and (robot, "pose") in seen for robot in ids
            )
            will = seen.get((dropped, "presence"), {}).get("online") is False
            if healthy and will:
                print(f"         {dropped} is offline — the broker published its will")
                return
            time.sleep(0.25)
        missing = [robot for robot in ids if (robot, "health") not in seen]
        raise RuntimeError(
            f"retained state never arrived: no health from {missing or 'nobody'}"
            + ("" if will else f"; no will for {dropped}")
        )
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

"""Hold the fleet box to one broker: one image pin, one config, one WS listener.

There were two containerised brokers here once — a workstation `docker run` and
the compose service — with their own image tags and their own idea of the
config. Everything below is a way for that to come back, so each check is a
seam where the two could drift apart again rather than a property of either one
alone.

The websockets checks are the ones that matter most, because that failure is
silent: mosquitto opens the MQTT listener and keeps running whether or not the
WebSocket one came up, so robots and `fleetctl` look perfectly healthy while the
dashboard is dead. Nothing here needs docker — these are the text-level
invariants, and `docs/fleet/ms-verification.md` records the live run.
"""

import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
COMPOSE = REPO / "mote_fleet" / "deploy" / "docker-compose.yml"
BROKER_SH = REPO / "mote_fleet" / "server" / "broker.sh"
MOSQUITTO_CONF = REPO / "mote_fleet" / "server" / "mosquitto.conf"
#: The third broker that could drift: the one `fleet-ui-check` runs to check the
#: dashboard. It reads the compose file's pin for the same reason broker.sh does.
UI_CHECK = REPO / "mote_fleet" / "test" / "ui_check.py"

PIN = re.compile(r"^ *image: *\$\{MOTE_BROKER_IMAGE:-([^}]*)\}", re.M)


@pytest.fixture
def compose():
    return yaml.safe_load(COMPOSE.read_text())


def test_the_broker_image_is_pinned_exactly_once():
    """The compose file is the pin; nothing else may carry a default."""
    pins = PIN.findall(COMPOSE.read_text())
    assert len(pins) == 1, f"expected one broker image pin, found {pins}"

    others = [
        path
        for path in (BROKER_SH, MOSQUITTO_CONF, UI_CHECK)
        if "eclipse-mosquitto:" in path.read_text()
    ]
    assert others == [], (
        "a second broker image tag has appeared in "
        f"{[p.name for p in others]} — the pin belongs only in {COMPOSE.name}"
    )


def test_the_broker_image_is_not_a_floating_major_tag():
    """`:2` moved from mosquitto 2.0 to 2.1, which changed how WS is built in.

    2.0 links libwebsockets, 2.1 implements it natively, and whatever follows
    may do neither — so the tag names a minor series.
    """
    (pin,) = PIN.findall(COMPOSE.read_text())
    assert re.match(r"^eclipse-mosquitto:\d+\.\d+", pin), (
        f"{pin!r} does not pin a minor series"
    )


def test_broker_sh_runs_the_image_the_compose_file_pins(tmp_path):
    """Run the script against a stub docker and see which image it reaches for.

    This is the deferral itself: `broker.sh` keeps no tag of its own, so the
    only way it can name one is by reading the compose file.
    """
    (pin,) = PIN.findall(COMPOSE.read_text())

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    stub = bin_dir / "docker"
    stub.write_text('#!/usr/bin/env bash\necho "DOCKER_ARGS: $*"\n')
    stub.chmod(0o755)

    result = subprocess.run(
        ["bash", str(BROKER_SH)],
        capture_output=True,
        text=True,
        timeout=60,
        env={
            "PATH": f"{bin_dir}:/usr/bin:/bin",
            "HOME": str(tmp_path),
            "MOTE_FLEET_HOME": str(tmp_path / "state"),
        },
    )

    assert result.returncode == 0, result.stderr
    assert "DOCKER_ARGS: run " in result.stdout
    assert pin in result.stdout, (
        f"broker.sh ran something other than the pinned {pin}:\n{result.stdout}"
    )


def test_the_deployed_broker_mounts_the_repos_own_mosquitto_conf(compose):
    """One config file, mounted — not a copy that can fall behind."""
    mounts = compose["services"]["broker"]["volumes"]
    conf = [m for m in mounts if m.endswith("/mosquitto/config/mosquitto.conf:ro")]
    assert len(conf) == 1, f"expected one config mount, got {mounts}"

    source = COMPOSE.parent / conf[0].split(":")[0]
    assert source.resolve() == MOSQUITTO_CONF.resolve()


def test_the_websockets_listener_survives_into_the_shipped_config():
    """The dashboard's whole read path is this listener."""
    text = MOSQUITTO_CONF.read_text()
    block = re.search(r"^# >>> websockets$(.*?)^# <<< websockets$", text, re.M | re.S)
    assert block, "the websockets stanza and its strip markers are gone"
    assert "listener 9001" in block.group(1)
    assert "protocol websockets" in block.group(1)


def test_mosquitto_does_not_narrow_its_own_log_types():
    """`log_type` replaces mosquitto's defaults rather than adding to them.

    Naming any subset drops `error`, and the errors it drops are the two that
    make the broker exit: a port already in use, and a websockets listener the
    build cannot honour (m3-verification.md §1).
    """
    lines = [
        ln
        for ln in MOSQUITTO_CONF.read_text().splitlines()
        if ln.strip().startswith("log_type")
    ]
    assert lines == [], f"log_type is set, which silences errors: {lines}"


def test_the_healthcheck_covers_the_websocket_listener(compose):
    """An MQTT-only probe reports healthy on a broker with no dashboard."""
    test = " ".join(compose["services"]["broker"]["healthcheck"]["test"])
    assert "1883" in test, "the healthcheck no longer probes the MQTT listener"
    assert "9001" in test, (
        "the healthcheck does not probe the WebSocket listener, so a broker "
        "that never opened it would report healthy"
    )


def test_the_browser_is_told_the_published_websocket_port(compose):
    """The server hands the browser a port on the host, not the container's."""
    command = compose["services"]["server"]["command"]
    assert "--broker-ws-port" in command
    value = command[command.index("--broker-ws-port") + 1]
    assert "BROKER_WS_PORT" in value, f"--broker-ws-port is hardcoded to {value!r}"

    published = compose["services"]["broker"]["ports"]
    assert any("BROKER_WS_PORT" in p and p.endswith(":9001") for p in published), (
        f"the WebSocket listener is not published: {published}"
    )

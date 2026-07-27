"""M7's acceptance test: the topic tree is enforced, not merely documented.

The milestone's criterion is *"a robot cannot read another robot's command
topic"*, and the only thing that can answer it is a real broker reading the real
generated files. So this starts mosquitto with the ``password_file`` and
``acl_file`` the fleet server would have written, from a real registry with two
enrolled robots and one operator, and then tries the things that must not work.

Denial is asserted on **delivery**, not on a return code, because that is how
mosquitto denies: a publish to a forbidden topic is accepted at the socket and
silently dropped, and a subscribe to a forbidden filter is granted at SUBACK and
never delivers. A test that asserted an error code would pass against a broker
with no ACL at all. (Measured, and recorded in ``docs/fleet/m7-verification.md``
§2, because it is the sort of thing that reads like a bug until you know.)

The half this cannot prove is "off the tailnet reaches nothing", which is a
property of the Tailscale policy rather than of this code —
``mote_bringup/tailscale/policy.hujson`` and the m7 ledger cover that.
"""

import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

import credentials
import pytest
from registry import Registry

from mote_fleet import protocol

mqtt = pytest.importorskip("paho.mqtt.client")


def mosquitto_bin() -> str | None:
    """conda-forge installs the broker into ``$PREFIX/sbin``, and pixi only puts
    ``bin`` on PATH — so ``which mosquitto`` says no in an environment that
    has it (the same lookup ``test_e2e_fleet.py`` and ``broker.sh`` do)."""
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        candidate = Path(prefix) / "sbin" / "mosquitto"
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return shutil.which("mosquitto")


BROKER_BIN = mosquitto_bin()

pytestmark = pytest.mark.skipif(
    BROKER_BIN is None,
    reason="needs a mosquitto broker (pixi run -e dev / -e fleet)",
)

#: How long to wait before concluding a message is not coming. Everything here
#: is loopback and retained, so real deliveries land in single-digit
#: milliseconds; this is generous by two orders of magnitude.
SETTLE_S = 0.6


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Fleet:
    """A registry, its generated credential files, and a broker reading them."""

    def __init__(self, tmp_path):
        self.registry = Registry(tmp_path / "registry.db")
        self.passwords = {}
        for fingerprint in ("serial:aaa", "serial:bbb"):
            token = self.registry.new_token()
            robot, _ = self.registry.enroll(token=token, fingerprint=fingerprint)
            self.passwords[robot["robot_id"]] = robot["broker_password"]
        self.operator_token = self.registry.new_operator(name="michael")
        operator = self.registry.operator(self.operator_token)
        self.operator_user = operator["broker_user"]
        self.passwords[self.operator_user] = operator["broker_password"]
        self.passwords[credentials.SERVER_USER] = self.registry.server_broker_password()

        auth = credentials.BrokerAuth(tmp_path / "broker")
        auth.write(**self.registry.broker_principals())
        self.acl_path = auth.acl_path

        self.port = free_port()
        conf = tmp_path / "mosquitto.conf"
        conf.write_text(
            f"listener {self.port} 127.0.0.1\n"
            "allow_anonymous false\n"
            f"password_file {auth.passwd_path}\n"
            f"acl_file {auth.acl_path}\n"
            "persistence false\n"
        )
        self.process = subprocess.Popen(
            [BROKER_BIN, "-c", str(conf)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.5):
                    break
            except OSError:
                time.sleep(0.05)
        else:
            self.process.kill()
            pytest.fail(f"mosquitto did not start: {self.process.stderr.read()}")
        self.clients = []

    def client(self, username=None, *, password=None):
        """A connected (or refused) client. ``.connack`` is the verdict."""
        handle = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"{username or 'anon'}-{len(self.clients)}",
        )
        handle.received = []
        handle.connack = None
        if username:
            handle.username_pw_set(
                username,
                self.passwords[username] if password is None else password,
            )
        handle.on_connect = lambda c, u, f, rc, *a: setattr(handle, "connack", rc)
        handle.on_message = lambda c, u, m: handle.received.append(
            (m.topic, m.payload.decode())
        )
        handle.connect("127.0.0.1", self.port, keepalive=20)
        handle.loop_start()
        deadline = time.monotonic() + 5
        while handle.connack is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.clients.append(handle)
        return handle

    def close(self):
        for handle in self.clients:
            handle.loop_stop()
        self.process.terminate()
        self.process.wait(timeout=10)


@pytest.fixture
def fleet(tmp_path):
    made = Fleet(tmp_path)
    yield made
    made.close()


def connected(handle) -> bool:
    return str(handle.connack) in ("Success", "0")


def payloads(handle) -> list[str]:
    return [payload for _, payload in handle.received]


# ---- authentication -----------------------------------------------------


def test_an_anonymous_client_cannot_connect(fleet):
    """The M1 posture, gone: reaching the port is no longer enough."""
    assert not connected(fleet.client())


def test_a_wrong_password_cannot_connect(fleet):
    assert not connected(fleet.client("mote-01", password="not-the-password"))


def test_an_enrolled_robot_connects_with_its_issued_credential(fleet):
    assert connected(fleet.client("mote-01"))


def test_a_revoked_operator_is_gone_from_the_regenerated_files(fleet, tmp_path):
    """Revocation closes the MQTT half too, because the file is a projection of
    the rows — the operator is absent from the query, so absent from the file."""
    fleet.registry.revoke_operator(fleet.operator_token)
    principals = fleet.registry.broker_principals()
    assert fleet.operator_user not in principals["users"]
    assert fleet.operator_user not in principals["operators"]


# ---- the acceptance criterion -------------------------------------------


def test_a_robot_cannot_read_another_robots_command_topic(fleet):
    """*The* M7 criterion, end to end."""
    eavesdropper = fleet.client("mote-01")
    target = fleet.client("mote-02")
    server = fleet.client(credentials.SERVER_USER)
    eavesdropper.subscribe(protocol.topic("mote-02", protocol.COMMAND), qos=1)
    target.subscribe(protocol.topic("mote-02", protocol.COMMAND), qos=1)
    time.sleep(SETTLE_S)

    server.publish(protocol.topic("mote-02", protocol.COMMAND), "goto kitchen", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(target) == ["goto kitchen"]
    assert payloads(eavesdropper) == []


def test_a_robot_cannot_publish_as_another_robot(fleet):
    """The other direction: no robot can forge another's health or pose, so a
    roster reading `ok` is a claim that robot actually made."""
    liar = fleet.client("mote-01")
    watcher = fleet.client(fleet.operator_user)
    watcher.subscribe(protocol.any_robot(protocol.HEALTH), qos=1)
    time.sleep(SETTLE_S)

    liar.publish(protocol.topic("mote-02", protocol.HEALTH), "forged", qos=1)
    liar.publish(protocol.topic("mote-01", protocol.HEALTH), "genuine", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(watcher) == ["genuine"]


def test_a_robot_cannot_read_another_robots_health(fleet):
    """Robots have no business reading each other at all — there is no
    robot-to-robot coordination in v1, and this is what keeps it that way."""
    nosy = fleet.client("mote-01")
    other = fleet.client("mote-02")
    nosy.subscribe(protocol.any_robot(protocol.HEALTH), qos=1)
    time.sleep(SETTLE_S)

    other.publish(protocol.topic("mote-02", protocol.HEALTH), "mine", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(nosy) == []


# ---- the operator is read-only ------------------------------------------


def test_an_operator_sees_the_whole_fleets_telemetry(fleet):
    watcher = fleet.client(fleet.operator_user)
    for leaf in (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS):
        watcher.subscribe(protocol.any_robot(leaf), qos=1)
    time.sleep(SETTLE_S)

    fleet.client("mote-01").publish(
        protocol.topic("mote-01", protocol.PRESENCE), "one", qos=1
    )
    fleet.client("mote-02").publish(
        protocol.topic("mote-02", protocol.POSE), "two", qos=1
    )
    time.sleep(SETTLE_S)

    assert sorted(payloads(watcher)) == ["one", "two"]


def test_an_operator_cannot_publish_a_command(fleet):
    """The browser's "no PUBLISH packet" is a property of our client. This is
    the same property as a rule of the broker's, which is the point of M7."""
    operator = fleet.client(fleet.operator_user)
    robot = fleet.client("mote-01")
    robot.subscribe(protocol.topic("mote-01", protocol.COMMAND), qos=1)
    time.sleep(SETTLE_S)

    operator.publish(protocol.topic("mote-01", protocol.COMMAND), "goto nowhere", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(robot) == []


def test_an_operator_cannot_read_a_command_topic(fleet):
    """Who dispatched what is answered by the audit log, which attributes it —
    not by the broker, which cannot."""
    operator = fleet.client(fleet.operator_user)
    server = fleet.client(credentials.SERVER_USER)
    operator.subscribe(protocol.any_robot(protocol.COMMAND), qos=1)
    time.sleep(SETTLE_S)

    server.publish(protocol.topic("mote-01", protocol.COMMAND), "secret", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(operator) == []


def test_an_operator_cannot_forge_a_robots_health(fleet):
    operator = fleet.client(fleet.operator_user)
    watcher = fleet.client(credentials.SERVER_USER)
    watcher.subscribe(protocol.any_robot(protocol.HEALTH), qos=1)
    time.sleep(SETTLE_S)

    operator.publish(protocol.topic("mote-01", protocol.HEALTH), "forged", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(watcher) == []


# ---- the server is the only writer --------------------------------------


def test_the_fleet_server_can_dispatch_to_any_robot(fleet):
    server = fleet.client(credentials.SERVER_USER)
    one = fleet.client("mote-01")
    two = fleet.client("mote-02")
    one.subscribe(protocol.topic("mote-01", protocol.COMMAND), qos=1)
    two.subscribe(protocol.topic("mote-02", protocol.COMMAND), qos=1)
    time.sleep(SETTLE_S)

    server.publish(protocol.topic("mote-01", protocol.COMMAND), "for-one", qos=1)
    server.publish(protocol.topic("mote-02", protocol.COMMAND), "for-two", qos=1)
    time.sleep(SETTLE_S)

    assert payloads(one) == ["for-one"]
    assert payloads(two) == ["for-two"]


def test_the_denial_is_silent_which_is_why_delivery_is_what_is_asserted(fleet):
    """Documents the measurement the rest of this file depends on.

    mosquitto grants the subscription at SUBACK and simply never delivers, so
    "did the client get an error" is not a usable signal — and a suite that
    checked for one would pass against a broker with no ACL loaded.
    """
    granted = []
    eavesdropper = fleet.client("mote-01")
    eavesdropper.on_subscribe = lambda c, u, mid, codes, *a: granted.extend(
        getattr(code, "value", code) for code in codes
    )
    eavesdropper.subscribe(protocol.topic("mote-02", protocol.COMMAND), qos=1)
    time.sleep(SETTLE_S)

    assert granted == [1], "mosquitto grants a forbidden subscription"
    assert payloads(eavesdropper) == []

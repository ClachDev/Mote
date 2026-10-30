"""The ``fleet`` CLI dispatches; ``join`` runs the right steps in order."""

import json
from types import SimpleNamespace

import pytest
from mote_bringup import identity

from mote_fleet import cli


@pytest.fixture
def mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    return tmp_path


def test_enroll_and_publish_forward_their_arguments(monkeypatch):
    seen = {}
    monkeypatch.setattr(
        cli.enroll, "main", lambda argv: seen.setdefault("enroll", argv)
    )
    monkeypatch.setattr(
        cli.publish, "main", lambda argv: seen.setdefault("publish", argv)
    )

    cli.main(["enroll", "--", "--server", "http://fleet:8080", "--token", "t"])
    cli.main(["publish-map", "--revision", "20260727T101500"])
    assert seen == {
        "enroll": ["--server", "http://fleet:8080", "--token", "t"],
        "publish": ["--revision", "20260727T101500"],
    }


class Runner:
    """Records every command; answers ``tailscale status`` with a chosen state."""

    def __init__(self, state):
        self.state = state
        self.commands = []

    def __call__(self, cmd, **kwargs):
        self.commands.append(cmd)
        if cmd[:2] == ["tailscale", "status"]:
            if self.state is None:
                return SimpleNamespace(returncode=1, stdout="")
            return SimpleNamespace(
                returncode=0, stdout=json.dumps({"BackendState": self.state})
            )
        return SimpleNamespace(returncode=0, stdout="")


def join_args(**over):
    base = dict(
        server="http://fleet:8080",
        token="tok",
        name="",
        site="",
        auth_key="",
        auth_key_file="",
        no_enable=False,
    )
    base.update(over)
    return SimpleNamespace(**base)


def test_join_from_nothing_joins_enrolls_renames_and_enables(mote_home, monkeypatch):
    run = Runner(state="NeedsLogin")
    steps = []

    def fake_enroll(argv):
        steps.append(("enroll", argv))
        identity.set_identity(id="mote-07", name="Scout")

    monkeypatch.setattr(cli.socket, "gethostname", lambda: "raspberrypi.local")
    monkeypatch.setattr(cli, "tailscale_script", lambda: mote_home / "install.sh")
    (mote_home / "install.sh").write_text("#!/bin/sh\n")

    assert (
        cli.join(
            join_args(auth_key="tskey-auth-x"),
            run=run,
            which=lambda _: "/usr/bin/tailscale",
            enroll_main=fake_enroll,
        )
        == 0
    )

    script = str(mote_home / "install.sh")
    assert run.commands[0] == ["tailscale", "status", "--json"]
    assert run.commands[1] == [
        "bash",
        script,
        "--role",
        "robot",
        "--hostname",
        "raspberrypi",
        "--auth-key",
        "tskey-auth-x",
    ]
    assert steps == [("enroll", ["--server", "http://fleet:8080", "--token", "tok"])]
    assert run.commands[2] == ["bash", script, "--role", "robot"], (
        "renamed to the id once enrolled"
    )
    assert run.commands[3] == [
        "sudo",
        "systemctl",
        "enable",
        "--now",
        "mote-agent",
        "mote-foxglove",
    ]
    assert len(run.commands) == 4


def test_join_on_a_joined_robot_only_enrolls(mote_home, monkeypatch):
    identity.set_identity(id="mote-03", name="Rover")
    run = Runner(state="Running")
    steps = []
    monkeypatch.setattr(cli, "tailscale_script", lambda: mote_home / "absent.sh")

    cli.join(
        join_args(name="Rover", site="home", no_enable=True),
        run=run,
        which=lambda _: "/usr/bin/tailscale",
        enroll_main=lambda argv: steps.append(argv),
    )

    assert run.commands == [["tailscale", "status", "--json"]], (
        "no install, no rename, no enable"
    )
    assert steps == [
        [
            "--server",
            "http://fleet:8080",
            "--token",
            "tok",
            "--name",
            "Rover",
            "--site",
            "home",
        ]
    ]


def test_join_without_tailscale_installed_runs_the_installer(mote_home, monkeypatch):
    identity.set_identity(id="mote-03", name="Rover")
    run = Runner(state=None)
    monkeypatch.setattr(cli, "tailscale_script", lambda: mote_home / "install.sh")
    (mote_home / "install.sh").write_text("#!/bin/sh\n")

    cli.join(
        join_args(no_enable=True),
        run=run,
        which=lambda _: None,
        enroll_main=lambda argv: None,
    )

    # No `tailscale status` when the binary is absent; the installer gets the id's hostname itself.
    assert run.commands == [["bash", str(mote_home / "install.sh"), "--role", "robot"]]

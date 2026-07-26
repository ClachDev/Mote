"""Role → tag resolution in the tailnet joiner.

A machine is one tailnet node and `tailscale up` replaces the whole tag set, so
getting roles wrong doesn't error — it silently drops a tag and leaves the ACLs
that M7 will write against them wrong. The script's `--dry-run` resolves roles,
tags and hostname without touching the network, which is what these pin down.
"""

import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "tailscale" / "install.sh"


@pytest.fixture
def home(tmp_path):
    return tmp_path


def run(home, *args):
    return subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", *args],
        capture_output=True,
        text=True,
        env={"PATH": "/usr/bin:/bin", "HOME": str(home), "MOTE_HOME": str(home)},
    )


def identity(home, robot_id="mote-01"):
    (home / "robot.yaml").write_text(f"schema: 1\nid: {robot_id}\nname: Scout\n")


def test_several_roles_become_one_tag_set(home):
    result = run(home, "--role", "fleet,inference")
    assert result.returncode == 0
    assert "tags:     tag:fleet,tag:inference" in result.stdout


def test_repeating_the_flag_is_the_same_as_a_comma_list(home):
    comma = run(home, "--role", "fleet,inference").stdout
    repeated = run(home, "--role", "fleet", "--role", "inference").stdout
    assert comma == repeated


def test_a_workstation_is_an_untagged_user_device(home):
    result = run(home, "--role", "workstation")
    assert result.returncode == 0
    assert "user device" in result.stdout
    assert "tag:" not in result.stdout


def test_workstation_cannot_also_be_infrastructure(home):
    """Advertising a tag transfers the node from the user to the tailnet, so the
    two are mutually exclusive rather than additive."""
    result = run(home, "--role", "workstation,fleet")
    assert result.returncode == 1
    assert "user device" in result.stderr
    assert "--role fleet,inference" in result.stderr


def test_a_robot_is_addressed_by_its_robot_id(home):
    identity(home, "mote-07")
    result = run(home, "--role", "robot")
    assert result.returncode == 0
    assert "hostname: mote-07" in result.stdout
    assert "tags:     tag:robot" in result.stdout


def test_the_role_defaults_to_robot_when_an_identity_exists(home):
    identity(home)
    assert "roles:    robot" in run(home).stdout


def test_the_role_defaults_to_workstation_without_one(home):
    assert "roles:    workstation" in run(home).stdout


def test_a_robot_without_an_identity_is_refused(home):
    result = run(home, "--role", "robot")
    assert result.returncode == 1
    assert "identity set --id" in result.stderr


def test_unknown_roles_are_refused(home):
    result = run(home, "--role", "gpu")
    assert result.returncode == 1
    assert "unknown role" in result.stderr


def test_the_auth_key_is_not_echoed(home):
    result = run(home, "--role", "fleet", "--auth-key", "tskey-auth-SECRET")
    assert "tskey-auth-SECRET" not in result.stdout
    assert "--auth-key ***" in result.stdout

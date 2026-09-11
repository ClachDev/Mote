"""Role → tag resolution in the tailnet joiner.

A machine is one tailnet node and `tailscale up` replaces the whole tag set, so
getting roles wrong doesn't error — it silently drops a tag and leaves the ACLs
in `policy.hujson` wrong. The script's `--dry-run` resolves roles,
tags and hostname without touching the network, which is what these pin down.
"""

import json
import re
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


# ---- the access policy ------------------------------------------------------


POLICY = Path(__file__).resolve().parents[1] / "tailscale" / "policy.hujson"


def policy():
    """``policy.hujson`` as data.

    Tailscale's dialect is JSON with `//` comments and trailing commas, which
    the stdlib parser refuses, so both are stripped here. This is a syntax
    check, not a semantic one: the rules are enforced by Tailscale, and the
    `tests` block below is run by Tailscale on every save — it refuses a policy
    that fails one. What is checked here is what a paste into the console cannot
    tell us: that the file still parses, and that it still names the tags this
    repo advertises.
    """
    lines = []
    for line in POLICY.read_text().splitlines():
        code = line.split("//", 1)[0] if not line.lstrip().startswith("//") else ""
        lines.append(code)
    text = "\n".join(lines)
    return json.loads(re.sub(r",(\s*[}\]])", r"\1", text))


def test_the_policy_is_parseable():
    assert set(policy()) == {"tagOwners", "hosts", "acls", "ssh", "tests"}


def test_every_tag_the_joiner_advertises_is_owned():
    """An `--advertise-tags` on a tag the policy does not declare fails with
    "requested tags are invalid or not permitted", which is a joining robot
    stopped by a file it never reads."""
    advertised = set(re.findall(r"tag:[a-z]+", SCRIPT.read_text()))
    assert advertised
    assert advertised <= set(policy()["tagOwners"])


def test_no_rule_lets_one_robot_reach_another():
    """The milestone's criterion, asserted against the rules rather than the
    comment beside them: in v1 there is no robot-to-robot anything."""
    for rule in policy()["acls"]:
        if "tag:robot" in rule["src"]:
            assert not any(dst.startswith("tag:robot") for dst in rule["dst"])


def test_the_policy_asks_tailscale_to_check_that_too():
    """Tailscale evaluates `tests` on every save and refuses a policy failing
    one, so the criterion is checked by the thing enforcing it."""
    robot = next(case for case in policy()["tests"] if case["src"] == "tag:robot")
    assert {dst for dst in robot["deny"] if dst.startswith("tag:robot:")}

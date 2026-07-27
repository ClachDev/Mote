"""Rehearse the robot update flow: stage, cut over, roll back.

The expensive half of an update is `pixi install`, and it is also the half that
is not interesting to test -- pixi's job, not ours. What *is* ours is the state
machine around it: slots that only become visible once complete, a `current`
symlink that flips atomically, a recorded previous version, a rollback that
returns to it, and the promise that none of it touches the robot's own state.

So these tests run the real mote-update script against a stub pixi, which makes
a full stage/cutover/rollback cycle a fraction of a second instead of several
gigabytes. The on-robot rehearsal (docs/releasing.md) covers the parts a stub
cannot: the real install, systemd, and the hardware coming back up.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

RELEASE_DIR = Path(__file__).resolve().parent.parent
MOTE_UPDATE = RELEASE_DIR / "deploy" / "mote-update"
DEPLOY_MANIFEST = RELEASE_DIR / "deploy" / "pixi.toml"

# A stand-in for pixi that does what mote-update depends on and nothing else:
# `install` materialises the .pixi directory that marks a slot as ready, and
# `run install-systemd` records the MOTE_REPO it was handed so a test can assert
# the units would be pointed at the `current` symlink rather than a slot.
STUB_PIXI = """#!/usr/bin/env bash
set -euo pipefail
command="$1"; shift
manifest=""
while [ $# -gt 0 ]; do
    case "$1" in
        --manifest-path) manifest="$2"; shift 2 ;;
        *) shift ;;
    esac
done
case "$command" in
    install) mkdir -p "$(dirname "$manifest")/.pixi/envs/default" ;;
    run) echo "$MOTE_REPO" >> "$STUB_LOG" ;;
esac
"""


@pytest.fixture
def robot(tmp_path):
    """A fake robot: a deploy root, a stub pixi, and some precious state."""
    pixi = tmp_path / "bin" / "pixi"
    pixi.parent.mkdir()
    pixi.write_text(STUB_PIXI)
    pixi.chmod(0o755)

    # $MOTE_HOME -- identity, maps, calibration. Deliberately a sibling of the
    # deploy root, which is the whole point: updates must not be able to reach it.
    mote_home = tmp_path / ".mote"
    (mote_home / "sites").mkdir(parents=True)
    (mote_home / "robot.yaml").write_text("robot_id: mote-01\n")
    (mote_home / "sites" / "map.png").write_bytes(b"\x89PNG-not-really")

    env = {
        **os.environ,
        "MOTE_DEPLOY_ROOT": str(tmp_path / "mote-deploy"),
        "MOTE_HOME": str(mote_home),
        "PIXI": str(pixi),
        "STUB_LOG": str(tmp_path / "stub.log"),
    }
    return {"env": env, "root": tmp_path, "mote_home": mote_home}


def run(robot, *args, check=True):
    return subprocess.run(
        [str(MOTE_UPDATE), *args, "--manifest", str(DEPLOY_MANIFEST)],
        env=robot["env"],
        capture_output=True,
        text=True,
        check=check,
    )


def deploy_root(robot) -> Path:
    return Path(robot["env"]["MOTE_DEPLOY_ROOT"])


def state_fingerprint(mote_home: Path) -> dict[str, bytes]:
    return {
        str(p.relative_to(mote_home)): p.read_bytes()
        for p in sorted(mote_home.rglob("*"))
        if p.is_file()
    }


def test_stage_does_not_disturb_the_running_version(robot):
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    assert (deploy_root(robot) / "current").resolve().name == "0.1.0"

    # Staging a second version must leave `current` exactly where it was: the
    # robot keeps running the old stack while the new one downloads.
    run(robot, "stage", "0.2.0")
    assert (deploy_root(robot) / "current").resolve().name == "0.1.0"
    assert (deploy_root(robot) / "versions" / "0.2.0" / ".pixi").is_dir()


def test_cutover_records_the_previous_version(robot):
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    run(robot, "stage", "0.2.0")
    run(robot, "cutover", "0.2.0")

    assert (deploy_root(robot) / "current").resolve().name == "0.2.0"
    assert (deploy_root(robot) / "previous").read_text().strip() == "0.1.0"


def test_rollback_returns_to_the_previous_version(robot):
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    run(robot, "stage", "0.2.0")
    run(robot, "cutover", "0.2.0")

    run(robot, "rollback")
    assert (deploy_root(robot) / "current").resolve().name == "0.1.0"
    # And rolling back again returns to where we were -- previous is swapped,
    # so an operator who rolls back by mistake is not stuck.
    assert (deploy_root(robot) / "previous").read_text().strip() == "0.2.0"
    run(robot, "rollback")
    assert (deploy_root(robot) / "current").resolve().name == "0.2.0"


def test_rollback_without_a_previous_version_fails_loudly(robot):
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    result = run(robot, "rollback", check=False)
    assert result.returncode != 0
    assert "no previous version" in result.stderr


def test_cutover_refuses_an_unstaged_version(robot):
    """The failure has to happen before anything is stopped, not after."""
    result = run(robot, "cutover", "9.9.9", check=False)
    assert result.returncode != 0
    assert "not staged" in result.stderr
    assert not (deploy_root(robot) / "current").exists()


def test_a_failed_stage_leaves_no_usable_slot(robot):
    """A half-installed slot that a later cutover would accept is the dangerous case."""
    broken = robot["root"] / "bin" / "pixi"
    broken.write_text("#!/usr/bin/env bash\nexit 1\n")
    broken.chmod(0o755)

    result = run(robot, "stage", "0.3.0", check=False)
    assert result.returncode != 0
    assert not (deploy_root(robot) / "versions" / "0.3.0").exists()


def test_units_are_pointed_at_the_current_symlink(robot):
    """Not at the slot: otherwise every cutover would need the units reinstalled."""
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    seen = Path(robot["env"]["STUB_LOG"]).read_text().split()
    assert seen, "install-systemd was never invoked"
    assert all(s == str(deploy_root(robot) / "current") for s in seen)


def test_robot_state_survives_a_full_update_cycle(robot):
    """~/.mote must come out of stage -> cutover -> rollback byte-identical.

    Identity, site maps and calibration are what make a robot *that* robot; an
    update that could damage them would have to be treated as a risky operation
    rather than a routine one.
    """
    before = state_fingerprint(robot["mote_home"])
    assert before, "the fixture wrote no state to check"

    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    run(robot, "stage", "0.2.0")
    run(robot, "cutover", "0.2.0")
    run(robot, "rollback")
    run(robot, "prune")

    assert state_fingerprint(robot["mote_home"]) == before


def test_prune_keeps_current_and_previous(robot):
    for version in ("0.1.0", "0.2.0", "0.3.0"):
        run(robot, "stage", version)
        run(robot, "cutover", version)
    run(robot, "prune")

    remaining = sorted(p.name for p in (deploy_root(robot) / "versions").iterdir())
    assert remaining == ["0.2.0", "0.3.0"]


def test_status_reports_the_slots(robot):
    run(robot, "stage", "0.1.0")
    run(robot, "cutover", "0.1.0")
    out = run(robot, "status").stdout
    assert "current:     0.1.0" in out

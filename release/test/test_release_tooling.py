"""Tests for the release tooling.

These guard the things that are easy to get silently wrong and expensive to
discover late: a new package that nobody added to the release set, a build
manifest that stopped pinning the backend, a deploy manifest that drifted from
the workspace, and the promise that an update cannot touch a robot's state.

They are deliberately cheap -- no builds, no network, no ROS -- so they can run
in CI next to the unit tests rather than only when someone cuts a release.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

RELEASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = RELEASE_DIR.parent
sys.path.insert(0, str(RELEASE_DIR))

from build import RELEASE_SET  # noqa: E402
from version import RELEASED_PACKAGES, version_files  # noqa: E402


def first_party() -> list[str]:
    return [name for name, path in RELEASE_SET if not path.startswith("release/")]


# --- the release set ------------------------------------------------------


def test_every_first_party_package_is_released_or_deliberately_excluded():
    """A new mote_* package must be added to the release set or excluded on purpose.

    Without this, adding a package and forgetting the release set produces a
    robot deploy that is quietly missing it -- and the failure appears at
    launch time on the robot, not at release time.
    """
    on_disk = {path.parent.name for path in REPO_ROOT.glob("mote_*/package.xml")}
    # mote_simulation is workstation-only: excluded by policy (docs/releasing.md).
    expected = on_disk - {"mote_simulation"}
    assert {
        p.removeprefix("ros-jazzy-").replace("-", "_") for p in first_party()
    } == expected


def test_release_set_matches_version_tool():
    """version.py and build.py must agree on which packages carry the release version."""
    assert sorted(RELEASED_PACKAGES) == sorted(
        p.removeprefix("ros-jazzy-").replace("-", "_") for p in first_party()
    )


@pytest.mark.parametrize("name,manifest_path", RELEASE_SET)
def test_every_released_package_pins_the_build_backend(name, manifest_path):
    """pixi-build is a preview feature, so an unpinned backend is a moving target.

    A release built with whatever backend happened to be newest is not
    reproducible, and the backend has changed its config schema between minor
    versions (docs/releasing.md).
    """
    manifest = REPO_ROOT / manifest_path / "pixi.toml"
    assert manifest.exists(), f"{name} has no build manifest at {manifest}"
    build = tomllib.loads(manifest.read_text())["package"]["build"]
    assert build["backend"]["name"] == "pixi-build-ros"
    assert re.match(r"^\d+\.\d+\.\d+", build["backend"]["version"]), (
        f"{name} does not pin a backend version"
    )
    assert build["config"]["distro"] == "jazzy"


def test_build_manifests_do_not_duplicate_the_version():
    """package.xml is the single source of truth the version tool bumps.

    A `version` in the build manifest would be a second place to bump and a
    silent way to publish a package whose version disagrees with its manifest.
    """
    for name, manifest_path in RELEASE_SET:
        manifest = tomllib.loads((REPO_ROOT / manifest_path / "pixi.toml").read_text())
        assert "version" not in manifest.get("package", {}), (
            f"{name}'s build manifest pins a version; it must come from package.xml"
        )


# --- versions -------------------------------------------------------------


def test_version_is_consistent():
    result = subprocess.run(
        [sys.executable, str(RELEASE_DIR / "version.py"), "check"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_version_set_round_trip(tmp_path):
    """`set` must reach every file `check` looks at, or a release ships a mixed set."""
    work = tmp_path / "repo"
    work.mkdir()
    shutil.copy(REPO_ROOT / "pixi.toml", work / "pixi.toml")
    for package in RELEASED_PACKAGES:
        (work / package).mkdir()
        for filename in ("package.xml", "setup.py"):
            source = REPO_ROOT / package / filename
            if source.exists():
                shutil.copy(source, work / package / filename)

    # version.py locates the repo root relative to itself, so the copy has to
    # sit at the same depth it does in the real tree.
    (work / "release").mkdir()
    script = work / "release" / "version.py"
    shutil.copy(RELEASE_DIR / "version.py", script)
    subprocess.run([sys.executable, str(script), "set", "9.8.7"], check=True, cwd=work)
    check = subprocess.run(
        [sys.executable, str(script), "check"], capture_output=True, text=True, cwd=work
    )
    assert check.returncode == 0, check.stderr
    assert "9.8.7" in (work / "pixi.toml").read_text()
    for file in version_files():
        copied = work / file.path.relative_to(REPO_ROOT)
        assert "9.8.7" in copied.read_text(), f"{copied} was not bumped"


# --- the deploy manifest --------------------------------------------------


@pytest.fixture(scope="module")
def deploy_manifest(tmp_path_factory) -> dict:
    output = tmp_path_factory.mktemp("deploy") / "pixi.toml"
    subprocess.run(
        [
            sys.executable,
            str(RELEASE_DIR / "deploy_manifest.py"),
            "--output",
            str(output),
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    return tomllib.loads(output.read_text())


def test_deploy_manifest_pins_every_first_party_package(deploy_manifest):
    version = deploy_manifest["workspace"]["version"]
    dependencies = deploy_manifest["dependencies"]
    for name in first_party():
        assert dependencies[name] == f"=={version}", (
            f"{name} is not pinned to the release version"
        )


def test_deploy_manifest_includes_the_submodule_packages(deploy_manifest):
    """A robot with no lidar driver and no odometry is not a deployed robot."""
    assert "ros-jazzy-sllidar-ros2" in deploy_manifest["dependencies"]
    assert "ros-jazzy-kinematic-icp" in deploy_manifest["dependencies"]


def test_deploy_manifest_excludes_build_tooling(deploy_manifest):
    """A deployed robot compiles nothing, so it should not carry a toolchain."""
    for tool in ("cmake", "compilers", "ninja", "colcon-common-extensions"):
        assert tool not in deploy_manifest["dependencies"]


def test_deploy_task_names_exist_in_the_workspace(deploy_manifest):
    """Same task names on a checkout and a deploy.

    A runbook, a systemd unit or an operator's habit should not have to know
    which of the two it is talking to.
    """
    workspace = tomllib.loads((REPO_ROOT / "pixi.toml").read_text())
    workspace_tasks = set(workspace["tasks"])
    for feature in workspace.get("feature", {}).values():
        workspace_tasks |= set(feature.get("tasks", {}))
    unknown = set(deploy_manifest["tasks"]) - workspace_tasks
    assert not unknown, (
        f"deploy tasks that no workspace task matches: {sorted(unknown)}"
    )


def test_systemd_units_are_runnable_from_a_deploy(deploy_manifest):
    """Every `pixi run X` a unit invokes must exist in the deployed task set."""
    units = sorted((REPO_ROOT / "mote_bringup" / "systemd").glob("*.service"))
    assert units, "no systemd units found"
    invoked = set()
    for unit in units:
        for line in unit.read_text().splitlines():
            match = re.search(r"pixi run ([\w-]+)", line)
            if match:
                invoked.add(match.group(1))
    missing = invoked - set(deploy_manifest["tasks"])
    assert not missing, f"units invoke tasks a deploy does not have: {sorted(missing)}"


def test_committed_deploy_manifest_is_up_to_date(deploy_manifest):
    """release/deploy/pixi.toml is what a robot fetches by tag, so it must not drift."""
    committed = tomllib.loads((RELEASE_DIR / "deploy" / "pixi.toml").read_text())
    assert committed == deploy_manifest, (
        "release/deploy/pixi.toml is stale -- run: pixi run release-manifest"
    )


# --- runtime state safety -------------------------------------------------


def test_updates_cannot_touch_robot_state():
    """~/.mote must be outside everything an update writes.

    Identity, site maps, calibration and bags live in $MOTE_HOME. The update
    mechanism keeps its slots under $MOTE_DEPLOY_ROOT and must never read or
    write the other, which is what makes an update repeatable and a rollback
    safe.
    """
    script = (RELEASE_DIR / "deploy" / "mote-update").read_text()
    body = "\n".join(
        line for line in script.splitlines() if not line.lstrip().startswith("#")
    )
    assert "MOTE_HOME" not in body, "mote-update refers to MOTE_HOME"
    assert ".mote/" not in body and '"$HOME/.mote"' not in body
    # And the two roots must be different directories.
    assert "mote-deploy" in script

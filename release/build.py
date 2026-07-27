#!/usr/bin/env python3
"""Build every released package and put it in a channel.

One command for both destinations, because they must build *identically*: a
dry-run that exercises a different code path than the real publish is not a
dry-run. The only difference is where the artifacts land --

    --to dist/channel                  a local, indexed file:// channel (default)
    --to https://prefix.dev/mote       the real thing (interactive confirmation)

A local channel rather than a plain output directory is the point of the
dry-run: an indexed channel can actually be *resolved against*, so
``release/verify.py`` proves the packages install as a set before any of them
is uploaded. A directory of .conda files proves only that rattler-build exited
zero.

Packages are built for the host platform. Cross-compiling the C++ packages is
not attempted: the release matrix builds each architecture natively instead
(see docs/releasing.md), which is what CI's linux-64 + linux-aarch64 runners are
for.

Usage:
    release/build.py                            # -> dist/channel
    release/build.py --to dist/channel
    release/build.py --to https://prefix.dev/mote --yes
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from version import REPO_ROOT, current_version  # noqa: E402

# Every package published to the `mote` channel, in dependency order (which is
# cosmetic -- each build resolves its dependencies from the channels, not from
# its siblings -- but makes the log read sensibly).
#
# The value is the manifest pixi builds from. First-party packages carry their
# own pixi.toml next to package.xml; the two submodules cannot (a manifest added
# inside a submodule is an untracked file in someone else's repo), so they get
# an out-of-tree definition under release/third_party that points back at the
# submodule with `source.path`.
#
# mote_simulation is deliberately absent: it is workstation-only, is already
# excluded from `pixi run sync`, and is developed and run from a checkout. It
# builds fine if that ever changes -- the exclusion is policy, not capability.
RELEASE_SET = [
    ("ros-jazzy-mote-description", "mote_description"),
    ("ros-jazzy-mote-hardware", "mote_hardware"),
    ("ros-jazzy-mote-nav", "mote_nav"),
    ("ros-jazzy-mote-bringup", "mote_bringup"),
    ("ros-jazzy-mote-perception", "mote_perception"),
    ("ros-jazzy-mote-tasks", "mote_tasks"),
    ("ros-jazzy-mote-arm", "mote_arm"),
    ("ros-jazzy-mote-fleet", "mote_fleet"),
    # Third-party ROS packages carried as submodules. They are built and
    # published too, because a robot installing from the channel has no source
    # tree to colcon-build them from -- without these, a released robot has no
    # lidar driver and no odometry.
    ("ros-jazzy-sllidar-ros2", "release/third_party/sllidar_ros2"),
    ("ros-jazzy-kinematic-icp", "release/third_party/kinematic_icp"),
]

DEFAULT_CHANNEL = "dist/channel"
MOTE_CHANNEL = "https://prefix.dev/mote"


def is_local(channel: str) -> bool:
    return "://" not in channel or channel.startswith("file://")


def confirm_remote_publish(channel: str, version: str) -> None:
    """Gate an outward-facing upload behind a typed confirmation.

    Publishing to prefix.dev is irreversible in the way that matters: a version
    that has been downloaded cannot be un-published from someone's lockfile. The
    prompt is deliberately not a y/n.
    """
    print(f"\nAbout to publish Mote {version} to {channel}")
    print(f"  {len(RELEASE_SET)} packages, host platform only.")
    print("  This is public and cannot be taken back once anyone has resolved it.")
    reply = input(f"\nType the version ({version}) to continue: ").strip()
    if reply != version:
        raise SystemExit("aborted -- nothing was published")


def build_one(manifest: Path, channel: str, platform: str | None) -> None:
    # --clean is not optional here. The backend's incremental-build hash covers
    # package.xml and the sources but NOT [package.build.config], so editing a
    # dependency mapping and rebuilding silently reuses the cached recipe and
    # produces an artifact that does not match the manifest. A release that can
    # do that is not reproducible, so every release build starts from scratch.
    command = ["pixi", "publish", "--clean", "--path", str(manifest), "--to", channel]
    if platform:
        command += ["--target-platform", platform]
    subprocess.run(command, cwd=REPO_ROOT, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--to",
        default=DEFAULT_CHANNEL,
        help=f"destination channel (default: {DEFAULT_CHANNEL}, a local file:// channel)",
    )
    parser.add_argument(
        "--target-platform",
        default=None,
        help="platform to build for (default: the host; see docs/releasing.md on arch builds)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the confirmation prompt for a remote channel (for CI)",
    )
    args = parser.parse_args()

    version = current_version()
    if subprocess.run(
        [sys.executable, str(Path(__file__).parent / "version.py"), "check"],
        cwd=REPO_ROOT,
    ).returncode:
        return 1

    channel = args.to
    if is_local(channel):
        # Resolve a relative local channel against the repo root so the command
        # behaves the same from any working directory.
        path = Path(channel.removeprefix("file://"))
        if not path.is_absolute():
            path = REPO_ROOT / path
        path.mkdir(parents=True, exist_ok=True)
        channel = f"file://{path}"
    elif not args.yes:
        confirm_remote_publish(channel, version)

    print(f"\nBuilding Mote {version} -> {channel}\n")
    for name, manifest in RELEASE_SET:
        print(f"--- {name}  ({manifest})")
        build_one(REPO_ROOT / manifest, channel, args.target_platform)

    # The submodule packages keep their own upstream versions, so only the
    # first-party set carries the release version. Re-publishing an unchanged
    # submodule build is a no-op: pixi skips artifacts already in the channel.
    print(f"\n{len(RELEASE_SET)} packages -> {channel}  (first-party at {version})")
    if is_local(channel):
        print("Next: pixi run release-verify   (resolve a robot env against it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

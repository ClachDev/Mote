#!/usr/bin/env python3
"""Resolve a robot environment against the freshly built packages.

This is the dry-run with teeth. `release/build.py` proves each package builds;
this proves the release is *installable as a set*: the robot's deploy manifest,
pinned to the new version, must solve against the local dist/channel plus
robostack/conda-forge. It is the check that catches a package that built but
declares a dependency nothing provides, a version pin that no longer resolves,
or a first-party package accidentally left out of the release set.

It resolves (writes a lockfile) rather than installs -- solving is where the
answer is, and a full install would pull gigabytes of ROS to learn nothing more.

Only the host platform is verified, because a local channel holds only the
architecture that was just built. The other architecture is verified by the same
command on a runner of that architecture (docs/releasing.md).

Usage:
    release/verify.py                      # against dist/channel
    release/verify.py --channel dist/channel --evidence release/evidence
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build import RELEASE_SET  # noqa: E402
from version import REPO_ROOT, current_version  # noqa: E402

HERE = Path(__file__).resolve().parent


def host_platform() -> str:
    machine = platform.machine()
    return "linux-aarch64" if machine in ("aarch64", "arm64") else "linux-64"


def resolved_mote_packages(lock_path: Path, subdir: str) -> dict[str, dict[str, str]]:
    """Pull the mote package versions out of the lockfile.

    Read as text rather than parsed as YAML: pixi.lock is large, the field we
    want is in the artifact URL, and this keeps the release tooling free of a
    YAML dependency it would otherwise need only here.
    """
    wanted = {name for name, _ in RELEASE_SET}
    found: dict[str, dict[str, str]] = {}
    for raw in lock_path.read_text().splitlines():
        line = raw.strip()
        if not line.startswith("- conda:") or f"/{subdir}/" not in line:
            continue
        filename = line.rsplit("/", 1)[-1]
        if not filename.endswith((".conda", ".tar.bz2")):
            continue
        stem = filename.removesuffix(".conda").removesuffix(".tar.bz2")
        # Artifact names are <name>-<version>-<build>, and neither version nor
        # build may contain a dash, so splitting from the right is exact.
        parts = stem.rsplit("-", 2)
        if len(parts) == 3 and parts[0] in wanted:
            # Keep the URL, not just the version: it records which channel the
            # solver actually took the package from, which is the part of the
            # dry-run worth being able to check later.
            url = line.removeprefix("- conda:").strip()
            # Make a local channel hit repo-relative: the evidence is committed,
            # and whose machine ran the build is not part of it.
            found[parts[0]] = {
                "version": parts[1],
                "url": url.replace(f"{REPO_ROOT}/", ""),
            }
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--channel", default="dist/channel")
    parser.add_argument(
        "--evidence",
        default=str(HERE / "evidence"),
        help="directory to write the dry-run evidence into",
    )
    args = parser.parse_args()

    version = current_version()
    subdir = host_platform()
    channel = Path(args.channel)
    if not channel.is_absolute():
        channel = REPO_ROOT / channel
    if not (channel / subdir / "repodata.json").exists():
        raise SystemExit(
            f"error: no indexed channel at {channel}/{subdir}. Run: pixi run release-build"
        )

    with tempfile.TemporaryDirectory() as tmp:
        slot = Path(tmp) / "deploy"
        slot.mkdir()
        manifest = slot / "pixi.toml"
        subprocess.run(
            [
                sys.executable,
                str(HERE / "deploy_manifest.py"),
                "--output",
                str(manifest),
                "--channel",
                f"file://{channel}",
                "--platforms",
                subdir,
            ],
            check=True,
            cwd=REPO_ROOT,
        )

        print(f"\nResolving the robot deploy environment for {version} on {subdir} ...")
        result = subprocess.run(
            ["pixi", "lock", "--manifest-path", str(manifest)],
            capture_output=True,
            text=True,
        )
        if result.returncode:
            print(result.stdout)
            print(result.stderr, file=sys.stderr)
            raise SystemExit(
                "error: the robot environment does not resolve against this build"
            )

        lock = slot / "pixi.lock"
        resolved = resolved_mote_packages(lock, subdir)

        evidence = Path(args.evidence)
        evidence.mkdir(parents=True, exist_ok=True)
        # The manifest itself is not copied here: it is release/deploy/pixi.toml
        # with the channel swapped for the local one and the platform narrowed,
        # so a copy would only add an absolute path from whichever machine ran
        # the build to a file that gets committed.
        summary = {
            "version": version,
            "platform": subdir,
            "manifest": "release/deploy/pixi.toml (channel overridden to dist/channel)",
            "channel": "dist/channel (local, indexed)",
            "packages_built": [name for name, _ in RELEASE_SET],
            "resolved": resolved,
        }
        (evidence / f"dry-run-{subdir}.json").write_text(
            json.dumps(summary, indent=2) + "\n"
        )

        print(f"\nResolved {len(resolved)} mote packages:")
        for name in sorted(resolved):
            print(f"  {name} {resolved[name]['version']}")
        missing = {name for name, _ in RELEASE_SET} - set(resolved)
        if missing:
            print(f"\nNOT resolved: {', '.join(sorted(missing))}", file=sys.stderr)
            return 1
        print(f"\nEvidence written to {evidence.relative_to(REPO_ROOT)}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())

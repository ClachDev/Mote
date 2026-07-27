#!/usr/bin/env python3
"""Render the robot deploy manifest for a release.

The deploy manifest is the pixi.toml a robot actually runs: released conda
packages from the `mote` channel and nothing else -- no checkout, no colcon.

Its *dependencies* are generated rather than hand-maintained. The workspace
manifest already lists the ROS packages the robot needs, and a deployed robot
that resolved a different set than the release was tested against would be a
different robot. Its *tasks* are hand-written in the template, because the
deployed task surface is a curated subset (no build, no sync, no sim).

The rendered file is committed to the repo under release/deploy/pixi.toml, which
is how a robot gets it without a checkout: mote-update fetches it by tag.

Usage:
    release/deploy_manifest.py                 # -> release/deploy/pixi.toml
    release/deploy_manifest.py --output -      # to stdout
    release/deploy_manifest.py --channel file:///path/to/dist/channel
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build import RELEASE_SET  # noqa: E402
from version import REPO_ROOT, current_version  # noqa: E402

TEMPLATE = Path(__file__).resolve().parent / "deploy" / "pixi.toml.in"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "deploy" / "pixi.toml"

# Where a robot's deploy slots live, and the stable symlink the systemd units
# point at. Kept in step with mote-update.
DEPLOY_CURRENT = "$HOME/mote-deploy/current"

# Workspace dependencies that must not be carried into a robot deploy: the build
# toolchain, because a deployed robot compiles nothing. Everything else is
# carried across, including runtime libraries a released package already depends
# on (scservo-linux) -- the duplication is harmless and pins the version the
# release was actually tested against.
SKIP_DEPENDENCIES = {
    "cmake",
    "compilers",
    "ninja",
    "pkg-config",
    "colcon-common-extensions",
    "ros-jazzy-ament-cmake-auto",
    "ros-jazzy-ament-lint-auto",
    "ros-jazzy-ament-cmake-gtest",
}


def toml_value(value: object) -> str:
    """Render a dependency spec back to TOML (a string pin, or a table)."""
    if isinstance(value, str):
        return f'"{value}"'
    if isinstance(value, dict):
        inner = ", ".join(f"{k} = {toml_value(v)}" for k, v in value.items())
        return f"{{ {inner} }}"
    raise TypeError(f"unsupported dependency spec: {value!r}")


def render_dependencies(dependencies: dict[str, object]) -> str:
    return "\n".join(
        f"{name} = {toml_value(spec)}"
        for name, spec in sorted(dependencies.items())
        if name not in SKIP_DEPENDENCIES
    )


def render_mote_packages(version: str) -> str:
    """Pin every released first-party package to the exact release version.

    `==` and not a range: the packages are co-developed and released together,
    so a robot mixing versions is a configuration nobody has tested. The two
    submodule packages are pinned loosely by comparison -- they carry upstream
    versions that this repo does not bump.
    """
    lines = []
    for name, manifest in RELEASE_SET:
        if manifest.startswith("release/third_party/"):
            lines.append(f'{name} = "*"')
        else:
            lines.append(f'{name} = "=={version}"')
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="where to write the manifest ('-' for stdout)",
    )
    parser.add_argument(
        "--channel",
        default=None,
        help=(
            "override the mote channel -- point it at a local dist/channel to "
            "resolve a release that has not been published yet"
        ),
    )
    parser.add_argument(
        "--platforms",
        default=None,
        help=(
            "comma-separated platforms (default: linux-64,linux-aarch64). "
            "release/verify.py narrows this to the host, because a local "
            "dist/channel only holds the architecture that was just built"
        ),
    )
    args = parser.parse_args()

    workspace = tomllib.loads((REPO_ROOT / "pixi.toml").read_text())
    version = current_version()

    channels = list(workspace["workspace"]["channels"])
    if args.channel:
        # Prepend rather than replace. During a dry-run the freshly built
        # packages must win, but the `mote` channel has to stay reachable: the
        # release depends on things already published there that this build does
        # not produce (scservo-linux, which ros-jazzy-mote-hardware links).
        channels = [args.channel] + channels

    platforms = (
        args.platforms.split(",") if args.platforms else ["linux-64", "linux-aarch64"]
    )

    rendered = (
        TEMPLATE.read_text()
        .replace("@VERSION@", version)
        .replace("@PLATFORMS@", "[" + ", ".join(f'"{p}"' for p in platforms) + "]")
        .replace("@CHANNELS@", "[" + ", ".join(f'"{c}"' for c in channels) + "]")
        .replace("@MOTE_PACKAGES@", render_mote_packages(version))
        .replace("@DEPENDENCIES@", render_dependencies(workspace["dependencies"]))
        .replace(
            "@PYPI_DEPENDENCIES@", render_dependencies(workspace["pypi-dependencies"])
        )
        .replace("@DEPLOY_CURRENT@", DEPLOY_CURRENT)
    )

    if args.output == "-":
        print(rendered, end="")
    else:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered)
        # release/verify.py renders into a temp directory, so the path is not
        # always under the repo.
        shown = (
            output.relative_to(REPO_ROOT)
            if output.is_relative_to(REPO_ROOT)
            else output
        )
        print(f"wrote {shown} for version {version}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

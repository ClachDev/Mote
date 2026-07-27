#!/usr/bin/env python3
"""Read, check and bump the repo-wide release version.

Mote's first-party packages are co-developed and deployed as a set, so they
carry **one** version rather than eight independent ones: a robot runs "Mote
0.4.2", not a matrix of package versions that have to be reasoned about
together. The deploy manifest pins them all to the same string
(``release/deploy_manifest.py``), which is only meaningful if they are in fact
released in lockstep.

The version lives in three kinds of file, and this module keeps them equal:

* ``pixi.toml``    ``[workspace] version`` -- the human-facing source of truth
* ``<pkg>/package.xml``  ``<version>``     -- what pixi-build-ros stamps into
  the conda package (the per-package ``pixi.toml`` build manifests deliberately
  carry no ``version``, so this is the only place the built artifact reads)
* ``<pkg>/setup.py``  ``version=``         -- the Python dist metadata inside
  the ament_python packages

Third-party submodules keep their upstream versions and are not touched.

Usage:
    release/version.py show           # print the current version
    release/version.py check          # exit non-zero if the files disagree
    release/version.py set 0.2.0      # write it everywhere
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# The first-party packages released to the `mote` channel. mote_simulation is
# deliberately absent -- it is workstation-only and is never deployed (see
# docs/releasing.md), so it keeps no release version.
RELEASED_PACKAGES = [
    "mote_description",
    "mote_hardware",
    "mote_nav",
    "mote_bringup",
    "mote_perception",
    "mote_tasks",
    "mote_arm",
    "mote_fleet",
]

SEMVER = re.compile(r"^\d+\.\d+\.\d+$")

# `<version>0.1.0</version>` on its own line, and only the first one: a
# package.xml has exactly one, but a naive global sub would also rewrite any
# <version> nested in an <export> block.
_PACKAGE_XML_VERSION = re.compile(r"(?m)^(\s*<version>)([^<]*)(</version>\s*)$")
_SETUP_PY_VERSION = re.compile(r"(?m)^(\s*version=\")([^\"]*)(\",\s*)$")
_WORKSPACE_VERSION = re.compile(r"(?m)^(version = \")([^\"]*)(\")$")


class VersionFile:
    """One file holding the version, and the pattern that finds it."""

    def __init__(self, path: Path, pattern: re.Pattern[str]):
        self.path = path
        self.pattern = pattern

    @property
    def rel(self) -> str:
        return str(self.path.relative_to(REPO_ROOT))

    def read(self) -> str | None:
        if not self.path.exists():
            return None
        match = self.pattern.search(self.path.read_text())
        return match.group(2) if match else None

    def write(self, version: str) -> bool:
        """Set the version. Returns True if the file changed."""
        text = self.path.read_text()
        new_text, count = self.pattern.subn(
            lambda m: f"{m.group(1)}{version}{m.group(3)}", text, count=1
        )
        if not count:
            raise SystemExit(f"error: no version found in {self.rel}")
        if new_text == text:
            return False
        self.path.write_text(new_text)
        return True


def version_files() -> list[VersionFile]:
    """Every file the release version has to agree in, workspace first."""
    files = [VersionFile(REPO_ROOT / "pixi.toml", _WORKSPACE_VERSION)]
    for package in RELEASED_PACKAGES:
        files.append(
            VersionFile(REPO_ROOT / package / "package.xml", _PACKAGE_XML_VERSION)
        )
        setup_py = REPO_ROOT / package / "setup.py"
        if setup_py.exists():  # ament_cmake packages have none
            files.append(VersionFile(setup_py, _SETUP_PY_VERSION))
    return files


def current_version() -> str:
    """The workspace version -- the source of truth the others are checked against."""
    version = version_files()[0].read()
    if version is None:
        raise SystemExit("error: no [workspace] version in pixi.toml")
    return version


def cmd_show(_args: argparse.Namespace) -> int:
    print(current_version())
    return 0


def cmd_check(_args: argparse.Namespace) -> int:
    expected = current_version()
    mismatched = [
        (f.rel, f.read()) for f in version_files()[1:] if f.read() != expected
    ]
    if mismatched:
        print(f"version mismatch (pixi.toml says {expected}):", file=sys.stderr)
        for rel, found in mismatched:
            print(f"  {rel}: {found}", file=sys.stderr)
        print("\nrun: pixi run release-version set " + expected, file=sys.stderr)
        return 1
    print(f"version {expected} consistent across {len(version_files())} files")
    return 0


def cmd_set(args: argparse.Namespace) -> int:
    if not SEMVER.match(args.version):
        raise SystemExit(
            f"error: {args.version!r} is not MAJOR.MINOR.PATCH. Mote releases the "
            "first-party packages as one semver set (docs/releasing.md)."
        )
    changed = [f.rel for f in version_files() if f.write(args.version)]
    if changed:
        print(f"set version {args.version} in {len(changed)} file(s):")
        for rel in changed:
            print(f"  {rel}")
    else:
        print(f"version already {args.version} everywhere")
    print(f"\nnext: pixi run release, then tag v{args.version}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("show", help="print the current version").set_defaults(func=cmd_show)
    sub.add_parser(
        "check", help="verify every file agrees with pixi.toml"
    ).set_defaults(func=cmd_check)
    set_parser = sub.add_parser("set", help="write a new version everywhere")
    set_parser.add_argument("version", help="MAJOR.MINOR.PATCH")
    set_parser.set_defaults(func=cmd_set)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

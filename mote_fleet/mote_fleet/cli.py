"""``fleet`` — the robot's side of the fleet, in one command.

    pixi run fleet enroll --server URL --token TOKEN [--name N] [--site S]
    pixi run fleet publish-map [--revision REV] [--site S] [--floor F]
    pixi run fleet join --server URL --token TOKEN [--name N] [--site S]
                        [--auth-key KEY | --auth-key-file PATH] [--no-enable]

``enroll`` and ``publish-map`` are :mod:`mote_fleet.enroll` and
:mod:`mote_fleet.publish` with their own flags (``fleet enroll --help``). The
operator's commands — tokens, promote, dispatch — are ``fleetctl``, off-board.

``join`` is the once-per-robot sequence from "on its wifi" to "reporting to the
fleet", for a robot that was not provisioned by ``pixi run provision`` (which
does the first step itself at first boot):

1. **tailnet.** Skipped when tailscaled already reports ``Running``. Otherwise
   ``mote_bringup/tailscale/install.sh --role robot``. The node's name is the
   robot id, which a robot that has never enrolled does not have yet, so such a
   robot joins under its hostname and step 3 renames it.
2. **enroll.** The server answers with an id, a name and the broker address.
3. **rename.** Only when step 1 joined a robot that had no id: the install
   script runs again, now with the id to hand. ``tailscale up`` is declarative,
   so this is a rename and nothing else. A robot that was already on the tailnet
   keeps whatever name it had.
4. **enable** ``mote-agent`` and ``mote-foxglove``, the two units that draw
   nothing and drive nothing (``systemd/install.sh``). ``--no-enable`` skips
   it; both need ``pixi run setup`` to have installed them.

Each step is a subprocess or a call into a sibling module, so the sequence is
testable without a tailnet, a server or systemd.
"""

import argparse
import json
import shutil
import socket
import subprocess
import sys
from pathlib import Path

from mote_bringup import identity

from mote_fleet import enroll, publish

ENABLE_UNITS = ("mote-agent", "mote-foxglove")


def tailscale_script() -> Path:
    """``mote_bringup/tailscale/install.sh`` in the checkout this runs from.

    Not an installed data file: with ``--symlink-install`` this module resolves
    back into the source tree, and the script is only ever run on a robot that
    has one.
    """
    return (
        Path(__file__).resolve().parents[2]
        / "mote_bringup"
        / "tailscale"
        / "install.sh"
    )


def tailnet_state(run=subprocess.run, which=shutil.which) -> str:
    """tailscaled's ``BackendState`` (``Running`` when joined), or ``""``."""
    if which("tailscale") is None:
        return ""
    try:
        done = run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return ""
    if done.returncode != 0:
        return ""
    try:
        return str(json.loads(done.stdout).get("BackendState", ""))
    except (ValueError, AttributeError):
        return ""


def _forwarded(rest):
    """The subcommand's own arguments; pixi passes a ``--`` through, argparse
    would read it as the end of options."""
    return rest[1:] if rest[:1] == ["--"] else rest


def join(
    args, *, run=subprocess.run, which=shutil.which, enroll_main=enroll.main
) -> int:
    script = tailscale_script()
    had_id = bool(identity.robot_id())
    joined_here = False

    state = tailnet_state(run, which)
    if state == "Running":
        print("==> tailnet: already joined")
    else:
        if not script.is_file():
            sys.exit(f"tailscale install script not found at {script}")
        cmd = ["bash", str(script), "--role", "robot"]
        if not had_id:
            cmd += ["--hostname", socket.gethostname().split(".")[0]]
        if args.auth_key:
            cmd += ["--auth-key", args.auth_key]
        if args.auth_key_file:
            cmd += ["--auth-key-file", args.auth_key_file]
        print("==> tailnet: joining")
        run(cmd, check=True)
        joined_here = True

    print("==> enroll")
    enroll_argv = ["--server", args.server, "--token", args.token]
    if args.name:
        enroll_argv += ["--name", args.name]
    if args.site:
        enroll_argv += ["--site", args.site]
    enroll_main(enroll_argv)

    if joined_here and not had_id:
        print(f"==> tailnet: renaming the node to {identity.robot_id()}")
        run(["bash", str(script), "--role", "robot"], check=True)

    if args.no_enable:
        print(f"==> not enabling {' '.join(ENABLE_UNITS)} (--no-enable)")
    else:
        print(f"==> enabling {' '.join(ENABLE_UNITS)}")
        run(["sudo", "systemctl", "enable", "--now", *ENABLE_UNITS], check=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="fleet", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name, help_ in (
        ("enroll", "ask the fleet server who this robot is"),
        ("publish-map", "offer the active floor's saved map to the registry"),
    ):
        # Listed for --help; dispatched on argv[0] below, before argparse, because
        # REMAINDER would not capture a leading `--revision R` or `--help`.
        sub.add_parser(name, help=help_, add_help=False)
    p_join = sub.add_parser(
        "join", help="tailnet + enroll + enable the reporting units"
    )
    p_join.add_argument("--server", required=True, help="fleet server base URL")
    p_join.add_argument(
        "--token", required=True, help="enrollment token (fleetctl token)"
    )
    p_join.add_argument("--name", default="", help="human label, e.g. 'Scout'")
    p_join.add_argument("--site", default="", help="site whose bundles this robot uses")
    p_join.add_argument(
        "--auth-key", default="", help="pre-authorised tag:robot Tailscale key"
    )
    p_join.add_argument(
        "--auth-key-file", default="", help="read the auth key from a file"
    )
    p_join.add_argument(
        "--no-enable",
        action="store_true",
        help="leave mote-agent and mote-foxglove disabled",
    )

    argv = sys.argv[1:] if argv is None else list(argv)
    forwarded = {"enroll": enroll.main, "publish-map": publish.main}
    if argv and argv[0] in forwarded:
        return forwarded[argv[0]](_forwarded(argv[1:]))

    args = parser.parse_args(argv)
    try:
        return join(args)
    except subprocess.CalledProcessError as exc:
        sys.exit(f"join stopped: {' '.join(exc.cmd)} exited {exc.returncode}")


if __name__ == "__main__":
    sys.exit(main())

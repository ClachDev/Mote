"""Render the cloud-init user-data that provisions a clean Pi into the fleet.

A new robot is a stock Raspberry Pi OS image plus one file: ``user-data`` on the
boot partition. Raspberry Pi Imager 2.0+ writes cloud-init user-data for its own
OS-customisation, so this replaces that dialog rather than sitting beside it —
one template, versioned in the repo, instead of a bespoke golden image.

First boot then runs, with no interactive steps
(``mote_bringup/provisioning/user-data.template``):

1. write ``$MOTE_HOME/robot.yaml`` — the identity everything downstream is keyed
   by (:mod:`mote_bringup.identity`);
2. ``tailscale up`` with a **single-use, pre-authorised, tagged auth key baked
   into the image** — which is what resolves the ordering trap: the robot is on
   the tailnet before it has to reach anything, so no secret is ever fetched
   over a network it cannot yet use;
3. install pixi, fetch the workspace, build it;
4. run the host-level setup the software package cannot carry (udev, wifi power
   save, systemd units).

The key and the wifi passphrase are secrets: the rendered file is written 0600
and must never be committed. Mint the key short-lived and single-use
(https://tailscale.com/kb/1085/auth-keys) so a lost card cannot enrol anything.

Console script ``provision`` (pixi task: provision)::

    pixi run provision --id mote-02 --ssh-key ~/.ssh/id_ed25519.pub \\
        --ts-authkey tskey-auth-... --name "Front desk" \\
        --wifi-ssid HomeNet --wifi-psk secret --boot /media/$USER/bootfs
"""

import argparse
import json
import os
import re
import stat
import sys
from pathlib import Path

import yaml

from mote_bringup.identity import ID_RE

WIFI_OPEN = "# >>> wifi"
WIFI_CLOSE = "# <<< wifi"
PLACEHOLDER_RE = re.compile(r"@[A-Z_]+@")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")
CONTROL_RE = re.compile(r"[\x00-\x1f\x7f]")

# Placeholders that sit inside a double-quoted YAML scalar in the template. An
# auth key or a passphrase is arbitrary bytes, and a stray " or \ in one would
# otherwise produce a user-data that half-provisions a Pi nobody is watching.
QUOTED = {"ROBOT_NAME", "SITE", "SSH_KEY", "TS_AUTHKEY"}


def template_path() -> Path:
    """The user-data template, whether installed or run from the source tree."""
    override = os.environ.get("MOTE_PROVISION_TEMPLATE")
    if override:
        return Path(override)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("mote_bringup"))
        candidate = share / "provisioning" / "user-data.template"
        if candidate.exists():
            return candidate
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "provisioning" / "user-data.template"


def strip_wifi(text: str) -> str:
    """Drop the wifi blocks — the robot is wired, or the card already has a network."""
    out, skipping = [], False
    for line in text.splitlines():
        if line.strip() == WIFI_OPEN:
            skipping = True
            continue
        if line.strip() == WIFI_CLOSE:
            skipping = False
            continue
        if not skipping:
            out.append(line)
    return "\n".join(out) + "\n"


def keep_wifi(text: str) -> str:
    """Keep the wifi blocks, dropping just their markers."""
    return (
        "\n".join(
            line
            for line in text.splitlines()
            if line.strip() not in (WIFI_OPEN, WIFI_CLOSE)
        )
        + "\n"
    )


def quote(value: str) -> str:
    """A value's body as a YAML double-quoted scalar (JSON escaping is a subset)."""
    return json.dumps(value)[1:-1]


def render(template: str, values: dict) -> str:
    """Substitute @PLACEHOLDERS@ by plain string replacement — a passphrase full
    of backslashes, ampersands or pipes has to survive verbatim, so no
    regex/sed replacement — escaping the ones that land in a quoted scalar."""
    text = keep_wifi(template) if values.get("WIFI_SSID") else strip_wifi(template)
    for key, value in values.items():
        text = text.replace(f"@{key}@", quote(value) if key in QUOTED else value)
    # A site is optional; drop the key rather than emit an empty one.
    text = "\n".join(
        line for line in text.splitlines() if line.rstrip() != '      site: ""'
    )
    return text + "\n"


def check(text: str):
    """Fail loudly rather than write a user-data that would half-provision a Pi."""
    left = sorted(set(PLACEHOLDER_RE.findall(text)))
    if left:
        raise ValueError(f"unsubstituted placeholders remain: {' '.join(left)}")
    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f"rendered user-data is not valid YAML: {exc}") from exc


def build(args) -> str:
    if not ID_RE.match(args.id):
        raise ValueError(
            f"invalid --id {args.id!r}: lowercase letters, digits and hyphens, max 32"
        )
    if not USER_RE.match(args.user or ""):
        raise ValueError(f"invalid --user {args.user!r}: not a unix login name")
    key_file = Path(args.ssh_key).expanduser()
    if not key_file.is_file():
        raise ValueError(f"no such ssh public key: {key_file}")
    if args.wifi_ssid and not args.wifi_psk:
        raise ValueError("--wifi-ssid given without --wifi-psk")

    authkey = args.ts_authkey
    if args.ts_authkey_file:
        authkey = Path(args.ts_authkey_file).expanduser().read_text().strip()
    if not authkey:
        raise ValueError("--ts-authkey is required (single-use, tagged tag:robot)")

    values = {
        "ROBOT_ID": args.id,
        "ROBOT_NAME": args.name or args.id,
        "SITE": args.site or "",
        "USER": args.user,
        "SSH_KEY": key_file.read_text().splitlines()[0].strip(),
        "TS_AUTHKEY": authkey,
        "WIFI_SSID": args.wifi_ssid or "",
        "WIFI_PSK": args.wifi_psk or "",
        "REPO": args.repo,
        "TIMEZONE": args.timezone,
    }
    for key, value in values.items():
        # Values that land in a YAML block scalar (the wifi keyfile, the staged
        # robot.yaml) can't carry a newline without breaking the indentation.
        if CONTROL_RE.search(value):
            raise ValueError(f"{key.lower()} contains a control character")
    text = render(template_path().read_text(), values)
    check(text)
    return text


def main():
    parser = argparse.ArgumentParser(
        prog="provision", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument("--id", required=True, help="robot id, e.g. mote-02")
    parser.add_argument("--name", help="human label (default: the id)")
    parser.add_argument("--site", help="site whose bundles this robot uses")
    parser.add_argument(
        "--ssh-key", required=True, help="public key that gets you into the Pi"
    )
    parser.add_argument(
        "--ts-authkey", default="", help="single-use Tailscale auth key"
    )
    parser.add_argument("--ts-authkey-file", help="read the auth key from a file")
    parser.add_argument("--wifi-ssid")
    parser.add_argument("--wifi-psk")
    parser.add_argument(
        "--user", default=os.environ.get("USER", "mote"), help="login to create"
    )
    parser.add_argument("--repo", default="https://github.com/ClachDev/Mote.git")
    parser.add_argument("--timezone", default="Etc/UTC")
    parser.add_argument("--out", help="write here (default: stdout)")
    parser.add_argument(
        "--boot", help="boot partition mount point; writes <boot>/user-data"
    )
    args = parser.parse_args()

    try:
        text = build(args)
    except ValueError as exc:
        sys.exit(f"error: {exc}")

    out = args.out or (str(Path(args.boot) / "user-data") if args.boot else None)
    if not out:
        sys.stdout.write(text)
        return

    path = Path(out)
    path.write_text(text)
    path.chmod(stat.S_IRUSR | stat.S_IWUSR)
    print(f"wrote {path} (contains secrets: 0600, do not commit)")
    print(
        f"robot id: {args.id}   user: {args.user}   wifi: {args.wifi_ssid or '<none>'}"
    )
    print(
        f"next: boot the card, then from any tailnet device: tailscale ping {args.id}"
    )


if __name__ == "__main__":
    main()

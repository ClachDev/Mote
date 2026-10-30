"""Render the cloud-init user-data that provisions a clean Pi into the fleet.

A new robot is a stock Raspberry Pi OS image plus one file: ``user-data`` on the
boot partition. Raspberry Pi Imager 2.0+ writes cloud-init user-data for its own
OS-customisation, so this replaces that dialog rather than sitting beside it —
one template, versioned in the repo, instead of a bespoke golden image.

First boot then runs, with no interactive steps
(``mote_bringup/provisioning/user-data.template``):

1. write ``$MOTE_HOME/robot.yaml`` — the identity everything downstream is keyed
   by (:mod:`mote_bringup.identity`) — and beside it, when one is given, the
   camera calibration from bench prep (``pixi run camera-calibrate``);
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
        --ts-authkey tskey-auth-... --name "Scout" \\
        --wifi-ssid HomeNet --wifi-psk secret --boot /media/$USER/bootfs \\
        [--camera-calibration camera_calibration.yaml]
"""

import argparse
import base64
import json
import os
import re
import stat
import sys
from pathlib import Path

import yaml

from mote_bringup.identity import ID_RE

# Optional template blocks: `# >>> name` ... `# <<< name`, kept or dropped whole.
BLOCK_OPEN_RE = re.compile(r"^# >>> (\w+)$")
BLOCK_CLOSE_RE = re.compile(r"^# <<< (\w+)$")
PLACEHOLDER_RE = re.compile(r"@[A-Z_]+@")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")
CONTROL_RE = re.compile(r"[\x00-\x1f\x7f]")
COUNTRY_RE = re.compile(r"^[A-Z]{2}$")

# Placeholders that sit inside a double-quoted YAML scalar in the template. An
# auth key or a passphrase is arbitrary bytes, and a stray " or \ in one would
# otherwise produce a user-data that half-provisions a Pi nobody is watching.
QUOTED = {"ROBOT_NAME", "SITE", "SSH_KEY", "TS_AUTHKEY", "CAMERA_CALIBRATION_B64"}

# What a camera_info_manager file carries; anything else is the wrong file.
CAMERA_INFO_KEYS = {
    "image_width",
    "image_height",
    "camera_matrix",
    "distortion_coefficients",
}


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


def select_blocks(text: str, keep: dict) -> str:
    """Keep or drop the template's optional blocks whole, markers gone either way:
    wifi when the robot has a network to join, camera when a bench calibration
    was given."""
    out, skipping = [], False
    for line in text.splitlines():
        opened = BLOCK_OPEN_RE.match(line.strip())
        if opened:
            skipping = not keep.get(opened.group(1), False)
            continue
        if BLOCK_CLOSE_RE.match(line.strip()):
            skipping = False
            continue
        if not skipping:
            out.append(line)
    return "\n".join(out) + "\n"


def camera_calibration_b64(path: Path) -> str:
    """The bench calibration as cloud-init ``encoding: b64`` content: one line,
    so neither YAML quoting nor block indentation can touch the bytes the robot
    will load. Checked to be a camera_info file, not some other yaml."""
    if not path.is_file():
        raise ValueError(f"no such camera calibration: {path}")
    raw = path.read_bytes()
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise ValueError(f"{path} is not valid YAML: {exc}") from exc
    missing = CAMERA_INFO_KEYS - set(data or {})
    if missing:
        raise ValueError(
            f"{path} is not a camera_info calibration: missing "
            + ", ".join(sorted(missing))
        )
    return base64.b64encode(raw).decode("ascii")


def quote(value: str) -> str:
    """A value's body as a YAML double-quoted scalar (JSON escaping is a subset)."""
    return json.dumps(value)[1:-1]


def render(template: str, values: dict) -> str:
    """Substitute @PLACEHOLDERS@ by plain string replacement — a passphrase full
    of backslashes, ampersands or pipes has to survive verbatim, so no
    regex/sed replacement — escaping the ones that land in a quoted scalar."""
    text = select_blocks(
        template,
        {
            "wifi": bool(values.get("WIFI_SSID")),
            "camera": bool(values.get("CAMERA_CALIBRATION_B64")),
        },
    )
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
    if args.wifi_ssid and not COUNTRY_RE.match(args.wifi_country or ""):
        # Not defaulted: the Pi's WLAN stays rfkill-blocked until the regulatory
        # domain is set, and guessing the wrong country is a radio-law problem,
        # not a config preference.
        raise ValueError(
            "--wifi-country is required with --wifi-ssid: a two-letter ISO 3166 "
            "code (GB, US, DE, ...) — the Pi's wifi will not come up without it"
        )

    authkey = args.ts_authkey
    if args.ts_authkey_file:
        authkey = Path(args.ts_authkey_file).expanduser().read_text().strip()
    if not authkey:
        raise ValueError("--ts-authkey is required (single-use, tagged tag:robot)")

    camera_b64 = ""
    camera_file = getattr(args, "camera_calibration", None)
    if camera_file:
        camera_b64 = camera_calibration_b64(Path(camera_file).expanduser())

    values = {
        "ROBOT_ID": args.id,
        "ROBOT_NAME": args.name or args.id,
        "SITE": args.site or "",
        "USER": args.user,
        "SSH_KEY": key_file.read_text().splitlines()[0].strip(),
        "TS_AUTHKEY": authkey,
        "WIFI_SSID": args.wifi_ssid or "",
        "WIFI_PSK": args.wifi_psk or "",
        "WIFI_COUNTRY": args.wifi_country or "",
        "REPO": args.repo,
        "TIMEZONE": args.timezone,
        "CAMERA_CALIBRATION_B64": camera_b64,
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
        "--wifi-country", help="ISO 3166 code for the WLAN regulatory domain, e.g. GB"
    )
    parser.add_argument(
        "--user", default=os.environ.get("USER", "mote"), help="login to create"
    )
    parser.add_argument("--repo", default="https://github.com/ClachDev/Mote.git")
    parser.add_argument("--timezone", default="Etc/UTC")
    parser.add_argument(
        "--camera-calibration",
        help="camera_info yaml from `pixi run camera-calibrate`, installed as "
        "$MOTE_HOME/camera_calibration.yaml at first boot",
    )
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
        f"   camera calibration: {'yes' if args.camera_calibration else '<packaged default>'}"
    )
    print(
        f"next: boot the card, then from any tailnet device: tailscale ping {args.id}"
    )


if __name__ == "__main__":
    main()

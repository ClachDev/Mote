"""Probe the off-board inference servers from the robot side (torch-free).

Answers "is the inference machine up, and what is it running?" over the same
socket the depth/detect nodes use — no ROS, no torch, so it runs anywhere the
robot env does. It sends the health sentinel (see depth_wire) to each service
and prints the JSON status the server reports (model, device, GPU, versions).

    pixi run inference-health                 # probe the configured inference_host
    pixi run inference-health --host mote-gpu  # or a specific host
    pixi run inference-health --json          # machine-readable

Exit status is 0 only if every probed service answered, so it doubles as a
health gate in scripts.
"""

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception.depth_wire import DepthClient, repo_revision  # noqa: E402
from mote_perception.detect_wire import DetectClient  # noqa: E402


def _default_host():
    """The inference_host from perception.yaml (user override, then packaged)."""
    user = os.path.expanduser("~/.mote/perception.yaml")
    packaged = Path(__file__).resolve().parents[1] / "config" / "perception.yaml"
    path = user if os.path.exists(user) else packaged
    try:
        import yaml

        with open(path) as f:
            return yaml.safe_load(f).get("inference_host", "127.0.0.1")
    except Exception:
        return "127.0.0.1"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--host", default=None, help="inference host (default: perception.yaml)"
    )
    ap.add_argument("--depth-port", type=int, default=5601)
    ap.add_argument("--detect-port", type=int, default=5602)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--json", action="store_true", help="emit raw JSON, no table")
    args = ap.parse_args()

    host = args.host or _default_host()
    services = [
        (
            "depth",
            DepthClient(host, args.depth_port, args.timeout, warn=lambda m: None),
        ),
        (
            "detect",
            DetectClient(host, args.detect_port, args.timeout, warn=lambda m: None),
        ),
    ]

    results = {}
    for name, client in services:
        results[name] = client.health()
        client.close()

    local = repo_revision()
    if args.json:
        print(
            json.dumps(
                {"host": host, "local_version": local, "services": results}, indent=2
            )
        )
    else:
        print(f"inference host: {host}   (this machine: {local or 'unknown'})")
        for name, info in results.items():
            if info is None:
                print(f"  {name:7} DOWN (no response)")
            else:
                dev = info.get("device", "?")
                gpu = info.get("gpu")
                where = f"{dev} ({gpu})" if gpu else dev
                print(
                    f"  {name:7} UP   {info.get('model', '?')}  on {where}"
                    f"  torch {info.get('torch', '?')}  @ {info.get('version') or '?'}"
                )
        # Robot and server share the wire protocol, so a revision mismatch is
        # worth flagging before it shows up as a confusing protocol error.
        stale = {
            n: i.get("version")
            for n, i in results.items()
            if i and local and i.get("version") and i["version"] != local
        }
        if stale:
            print(
                f"\nWARNING: version skew — this machine is at {local}, but "
                + ", ".join(f"{n} is at {v}" for n, v in stale.items())
                + ".\n         Update the inference machine "
                "(deploy/windows/update.ps1) so both ends match."
            )

    return 0 if all(v is not None for v in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())

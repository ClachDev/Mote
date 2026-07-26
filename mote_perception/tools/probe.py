"""Prove an inference server actually serves — health *and* one real frame.

This is the gate the blue/green update runs against a candidate container
before it is allowed to take the served ports (deploy/inference-deploy.sh).
A health blob only proves the process is listening and can answer a sentinel;
it is answered before the model has ever been loaded, so it cannot see a
broken model download, a CUDA/driver mismatch, or a torch build that faults on
the first forward pass. Sending a synthetic frame does, because it forces the
on-demand load (tools/model_host.py) and a full inference.

It lives *inside the image* on purpose: the inference machine has no checkout,
no pixi, and no repo, so the only place a probe can come from is the artifact
being probed. `pixi run inference-health` is the robot-side sibling — same
sentinel, but it resolves `inference_host` from perception.yaml and needs yaml
and mote_bringup, neither of which the image carries.

    python /app/tools/probe.py                         # in-container, both services
    python /app/tools/probe.py --host 127.0.0.1 --depth-port 5611
    python /app/tools/probe.py --json                  # machine-readable

Exit status is 0 only if every probed service answered its health check *and*
served the synthetic frame, so it composes into a deploy script.
"""

import argparse
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception.depth_wire import DEFAULT_PORT as DEPTH_PORT  # noqa: E402
from mote_perception.depth_wire import DepthClient  # noqa: E402
from mote_perception.detect_wire import DEFAULT_PORT as DETECT_PORT  # noqa: E402
from mote_perception.detect_wire import DetectClient  # noqa: E402

#: Enough structure that a model has something to do, without shipping an asset:
#: a horizon-ish gradient with a block on the floor.
FRAME_W, FRAME_H = 320, 240


def synthetic_frame() -> bytes:
    """A JPEG the servers can be asked to process, generated not stored."""
    y = np.linspace(0, 1, FRAME_H, dtype=np.float32)[:, None]
    x = np.linspace(0, 1, FRAME_W, dtype=np.float32)[None, :]
    img = np.stack(
        [
            (60 + 160 * y) * np.ones_like(x),
            (80 + 120 * y) * (0.6 + 0.4 * x),
            (200 - 120 * y) * np.ones_like(x),
        ],
        axis=-1,
    )
    img[150:200, 120:200] = (200, 40, 40)  # a box on the floor
    buf = io.BytesIO()
    Image.fromarray(img.astype(np.uint8)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def probe_depth(host, port, blob, timeout, infer):
    client = DepthClient(host, port, timeout=timeout, warn=lambda msg: None)
    try:
        info = client.health()
        if info is None:
            return False, {"error": "no answer to the health request"}
        if not infer:
            return True, {"health": info}
        depth = client.infer(blob)
        if depth is None:
            return False, {"health": info, "error": "frame rejected or not served"}
        if not np.isfinite(depth).any():
            return False, {"health": info, "error": "depth map is entirely non-finite"}
        return True, {
            "health": info,
            "shape": list(depth.shape),
            "median_m": round(float(np.median(depth[np.isfinite(depth)])), 3),
        }
    finally:
        client.close()


def probe_detect(host, port, blob, timeout, infer):
    client = DetectClient(host, port, timeout=timeout, warn=lambda msg: None)
    try:
        info = client.health()
        if info is None:
            return False, {"error": "no answer to the health request"}
        if not infer:
            return True, {"health": info}
        # An empty result is a pass: the synthetic frame is not required to
        # contain a box, only to be *served* without erroring.
        found = client.infer(blob, ["box"])
        if found is None:
            return False, {"health": info, "error": "frame rejected or not served"}
        return True, {"health": info, "detections": len(found)}
    finally:
        client.close()


SERVICES = {
    "depth": (probe_depth, DEPTH_PORT),
    "detect": (probe_detect, DETECT_PORT),
}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--depth-port", type=int, default=DEPTH_PORT)
    ap.add_argument("--detect-port", type=int, default=DETECT_PORT)
    ap.add_argument(
        "--services",
        default="depth,detect",
        help="comma-separated subset to probe (default: all)",
    )
    # A cold container has to load its model on the first frame, which on a
    # loaded GPU is tens of seconds -- far longer than the wire client's normal
    # per-frame timeout, and not a failure.
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument(
        "--wait",
        type=float,
        default=0.0,
        help="seconds to keep retrying while the server is still starting up",
    )
    ap.add_argument(
        "--no-infer",
        action="store_true",
        help="health check only; does not load the model",
    )
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    wanted = [s.strip() for s in args.services.split(",") if s.strip()]
    unknown = [s for s in wanted if s not in SERVICES]
    if unknown:
        ap.error(f"unknown service(s): {', '.join(unknown)}")

    blob = synthetic_frame() if not args.no_infer else b""
    ports = {"depth": args.depth_port, "detect": args.detect_port}

    results, ok_all = {}, True
    for name in wanted:
        probe, _ = SERVICES[name]
        deadline = time.monotonic() + args.wait
        while True:
            started = time.monotonic()
            ok, detail = probe(
                args.host, ports[name], blob, args.timeout, not args.no_infer
            )
            if ok or time.monotonic() >= deadline:
                break
            time.sleep(1.0)
        detail["ok"] = ok
        detail["seconds"] = round(time.monotonic() - started, 2)
        results[name] = detail
        ok_all &= ok

    if args.json:
        print(json.dumps({"ok": ok_all, "services": results}, indent=2))
    else:
        for name, detail in results.items():
            health = detail.get("health") or {}
            where = f"{args.host}:{ports[name]}"
            if detail["ok"]:
                extra = (
                    f"detections={detail['detections']}"
                    if "detections" in detail
                    else f"depth={detail.get('shape')} median={detail.get('median_m')}m"
                )
                print(
                    f"{name:7} OK   {where}  {health.get('device', '?')} "
                    f"@ {health.get('version', '?')}  {extra}  "
                    f"({detail['seconds']}s)"
                )
            else:
                print(f"{name:7} FAIL {where}  {detail['error']}", file=sys.stderr)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())

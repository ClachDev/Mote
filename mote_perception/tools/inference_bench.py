"""Measure end-to-end inference latency/throughput over the socket (torch-free).

Runs on the robot side (or any machine on the LAN) and times the full round
trip the depth/detect nodes actually pay: compress -> send -> server infer ->
receive. That captures GPU time *and* the network hop, so the numbers are
comparable to the on-robot pipeline, not just raw model speed.

    # depth, 200 frames of a real image, over the LAN, results to a file
    pixi run inference-bench --host mote-gpu --image frame.jpg --frames 200 \
        --out mote_perception/benchmarks/depth_cuda_lan.json

    # synthetic frames if you have no sample image handy
    pixi run inference-bench --host mote-gpu --frames 100

    # the detect service
    pixi run inference-bench --service detect --host mote-gpu --labels "red box,shoe"

Prints a percentile table and, with --out, writes the raw samples + summary as
JSON so results can be committed and compared across machines.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception.depth_wire import DEFAULT_PORT as DEPTH_PORT  # noqa: E402
from mote_perception.depth_wire import DepthClient  # noqa: E402
from mote_perception.detect_wire import DEFAULT_PORT as DETECT_PORT  # noqa: E402
from mote_perception.detect_wire import DetectClient  # noqa: E402


def _frame(image, size):
    """A JPEG blob: the given image (resized to `size`), or synthetic noise."""
    w, h = size
    if image:
        img = cv2.imread(image, cv2.IMREAD_COLOR)
        if img is None:
            raise SystemExit(f"cannot read image: {image}")
        img = cv2.resize(img, (w, h))
    else:
        rng = np.random.default_rng(0)
        img = rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    if not ok:
        raise SystemExit("jpeg encode failed")
    return buf.tobytes()


def _pct(xs, q):
    return statistics.quantiles(xs, n=100, method="inclusive")[q - 1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--service", choices=["depth", "detect"], default="depth")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None, help="default: service port")
    ap.add_argument(
        "--image", default=None, help="sample JPEG to send (else synthetic)"
    )
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--warmup", type=int, default=5, help="untimed frames first")
    ap.add_argument("--labels", default="box", help="detect only, comma-separated")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--out", default=None, help="write JSON results here")
    args = ap.parse_args()

    blob = _frame(args.image, (args.width, args.height))
    if args.service == "depth":
        port = args.port or DEPTH_PORT
        client = DepthClient(args.host, port, args.timeout, warn=print)
        call = lambda: client.infer(blob)  # noqa: E731
    else:
        port = args.port or DETECT_PORT
        labels = [s.strip() for s in args.labels.split(",") if s.strip()]
        client = DetectClient(args.host, port, args.timeout, warn=print)
        call = lambda: client.infer(blob, labels)  # noqa: E731

    health = client.health()
    print(f"{args.service} server @ {args.host}:{port} -> {health}")
    if health is None:
        raise SystemExit("server did not answer health check; aborting")

    for _ in range(args.warmup):
        call()

    samples = []
    for i in range(args.frames):
        t0 = time.perf_counter()
        r = call()
        dt = (time.perf_counter() - t0) * 1000.0
        if r is None:
            print(f"frame {i}: no result (server rejected or dropped)")
            continue
        samples.append(dt)
    client.close()

    if not samples:
        raise SystemExit("no successful frames")

    summary = {
        "service": args.service,
        "host": args.host,
        "port": port,
        "server": health,
        "image": args.image or "synthetic",
        "resolution": [args.width, args.height],
        "payload_bytes": len(blob),
        "frames": len(samples),
        "latency_ms": {
            "min": round(min(samples), 1),
            "mean": round(statistics.fmean(samples), 1),
            "p50": round(_pct(samples, 50), 1),
            "p90": round(_pct(samples, 90), 1),
            "p99": round(_pct(samples, 99), 1),
            "max": round(max(samples), 1),
        },
        "fps": round(1000.0 / statistics.fmean(samples), 2),
    }

    lm = summary["latency_ms"]
    print(
        f"\n{args.service} over {len(samples)} frames  ({len(blob) / 1024:.0f} KiB/frame)"
    )
    print(
        f"  latency ms  min {lm['min']}  p50 {lm['p50']}  mean {lm['mean']}  "
        f"p90 {lm['p90']}  p99 {lm['p99']}  max {lm['max']}"
    )
    print(f"  throughput  {summary['fps']} fps")

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(
                {**summary, "samples_ms": [round(s, 2) for s in samples]}, f, indent=2
            )
        print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()

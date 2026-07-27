"""A torch-free stand-in for the inference servers, for testing the deployment.

The deploy pipeline (`deploy/inference-deploy.sh`) is about *containers* — does
a candidate get probed before it takes the served ports, does a bad build get
rejected, does rollback restore a serving version. None of that needs a GPU or
a 6 GB image, and requiring one would mean the pipeline is only ever exercised
on the one machine that has an NVIDIA card.

So this speaks the real wire protocol (mote_perception/depth_wire.py,
detect_wire.py) with constant answers, and the image built around it carries the
real `tools/probe.py`. What is under test is the deployment machinery; what is
faked is only the model.

    --mode serve    answer health and inference normally
    --mode reject   answer health, then reject every frame — the failure a
                    health-only gate cannot see, and the reason the probe
                    sends a real frame
    --mode crash    exit immediately, as a broken build does
"""

import argparse
import struct
import socket
import sys
import threading
from pathlib import Path

import numpy as np

# Resolved for the image layout (/app/tools/stub_server.py beside
# /app/mote_perception/), which is the only place this ever runs.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception import depth_wire, detect_wire  # noqa: E402


def serve_depth(port, mode):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[stub] depth on {port} ({mode})", flush=True)
    while True:
        conn, _ = srv.accept()
        with conn:
            while True:
                header = depth_wire.recvall(conn, 4)
                if header is None:
                    break
                (n,) = struct.unpack(">I", header)
                if n == depth_wire.HEALTH_MAGIC:
                    depth_wire.send_health(
                        conn,
                        {
                            "service": "depth",
                            "model": "stub",
                            "device": "cpu",
                            "version": depth_wire.repo_revision(),
                            "loaded": True,
                        },
                    )
                    continue
                if depth_wire.recvall(conn, n) is None:
                    break
                if mode == "reject":
                    depth_wire.send_rejection(conn)
                else:
                    depth_wire.send_depth(conn, np.full((48, 64), 1.5, np.float32))


def serve_detect(port, mode):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[stub] detect on {port} ({mode})", flush=True)
    while True:
        conn, _ = srv.accept()
        with conn:
            while True:
                header = depth_wire.recvall(conn, 4)
                if header is None:
                    break
                (n,) = struct.unpack(">I", header)
                if n == detect_wire.HEALTH_MAGIC:
                    detect_wire.send_health(
                        conn,
                        {
                            "service": "detect",
                            "model": "stub",
                            "device": "cpu",
                            "version": depth_wire.repo_revision(),
                            "loaded": True,
                        },
                    )
                    continue
                blob = depth_wire.recvall(conn, n)
                if blob is None:
                    break
                length = depth_wire.recvall(conn, 4)
                if length is None:
                    break
                if depth_wire.recvall(conn, struct.unpack(">I", length)[0]) is None:
                    break
                if mode == "reject":
                    detect_wire.send_rejection(conn)
                else:
                    detect_wire.send_detections(conn, [(0, 0.9, (10, 10, 40, 40))])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="serve", choices=["serve", "reject", "crash"])
    ap.add_argument("--depth-port", type=int, default=depth_wire.DEFAULT_PORT)
    ap.add_argument("--detect-port", type=int, default=detect_wire.DEFAULT_PORT)
    # The real servers take these; accept and ignore them so the deploy script's
    # SERVER_ARGS pass through unchanged.
    ap.add_argument("--idle-timeout", type=float, default=0.0)
    ap.add_argument("--device", default="cpu")
    args, _ = ap.parse_known_args()

    if args.mode == "crash":
        raise SystemExit("stub: this build cannot serve")

    threading.Thread(
        target=serve_depth, args=(args.depth_port, args.mode), daemon=True
    ).start()
    serve_detect(args.detect_port, args.mode)


if __name__ == "__main__":
    main()

"""Supervise every inference service as one process (cross-platform).

Runs each server (depth, detect, ...) as a child bound to 0.0.0.0 so the robot
reaches them over the LAN at `inference_host` (see perception.yaml). Replaces the
old bash launcher so the same command works on the Linux dev box *and* the
Windows gaming PC — pixi provides `python` on every platform, but not `bash`.

If any child exits, the rest are torn down so a partial failure is visible rather
than half-served (a supervisor that keeps a dead tenant's siblings alive just
hides the outage). SIGINT/SIGTERM (or the Windows equivalent) stops everything.

This is the seam for the multi-service pattern: a new inference tenant is one row
in SERVICES — it inherits binding, supervision, teardown, and (via the shared
wire) health and reconnect for free. Per-service flags go in the row's args; the
robot-side node already carries its own port. See docs/inference-server.md.

    pixi run inference        # CPU env
    pixi run inference-rocm   # AMD ROCm env
    pixi run inference-cuda   # Windows/NVIDIA env
    # extra args pass through to every server, e.g. a shared device override:
    pixi run inference-cuda -- --device cuda
"""

import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

# (name, script, per-service args). Add a tenant here — nothing else changes.
SERVICES = [
    ("depth", "depth_server.py", []),
    ("detect", "detect_server.py", []),
]


def main():
    passthrough = sys.argv[1:]
    procs = []
    for name, script, extra in SERVICES:
        cmd = [
            sys.executable,
            "-u",
            str(HERE / script),
            "--host",
            "0.0.0.0",
            *extra,
            *passthrough,
        ]
        print(f"[supervisor] starting {name}: {' '.join(cmd)}", flush=True)
        procs.append((name, subprocess.Popen(cmd)))

    def shutdown(*_):
        for name, p in procs:
            if p.poll() is None:
                p.terminate()
        for _, p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while True:
        for name, p in procs:
            code = p.poll()
            if code is not None:
                print(f"[supervisor] {name} exited ({code}); stopping all", flush=True)
                shutdown()
        time.sleep(0.5)


if __name__ == "__main__":
    main()

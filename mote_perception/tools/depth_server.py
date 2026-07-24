"""Off-board monocular-depth inference server (runs in the pixi inference environment).

Keeps Depth Anything V2 resident and serves depth over a local socket so the ROS
node (which has no torch) can stay light and run anywhere — on the workstation
next to this server, or on the robot talking to it over the network. This is the
deliberate two-process split that keeps torch out of the ROS/robot environment.

Default is the *relative* (SSI) V2-Small model: since the node refits a full affine
in disparity against lidar every frame, the absolute scale of a metric model is
discarded anyway, and the relative model measured both more accurate and faster (see
tools/depth_bag_eval.py). Relative models output disparity, so it's inverted to depth
here; pass --metric for a metric model (depth already in metres, no inversion).

The wire protocol lives in mote_perception/depth_wire.py (shared with the node and
the offline tools). This file runs uninstalled in the torch env, so the package is
imported straight from the source tree.
"""

import argparse
import io
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception.depth_wire import (  # noqa: E402
    DEFAULT_PORT,
    HEALTH_MAGIC,
    recvall,
    send_depth,
    send_health,
    send_rejection,
)

MODEL = (
    "depth-anything/Depth-Anything-V2-Small-hf"  # relative (SSI); see module docstring
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--model", default=MODEL)
    # Default model is relative (outputs disparity, near=large) -> invert to depth so
    # the node (which expects depth, then refits scale) works. --metric: the model
    # already outputs metric depth, so pass it through unchanged.
    ap.add_argument("--metric", action="store_true")
    ap.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    ap.add_argument("--fp16", action="store_true")
    args = ap.parse_args()

    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    use_fp16 = args.fp16 and device != "cpu"
    if device != "cpu":
        print(
            "using GPU:", torch.cuda.get_device_name(0), "fp16" if use_fp16 else "fp32"
        )
    else:
        print("using CPU")

    print("loading", args.model, "(metric)" if args.metric else "(relative)")
    proc = AutoImageProcessor.from_pretrained(args.model)
    model = AutoModelForDepthEstimation.from_pretrained(args.model).eval().to(device)
    if use_fp16:
        model = model.half()
    # Leave torch's default thread count (physical cores). Setting it to
    # os.cpu_count() counts SMT siblings, and oversubscribing them thrashes the
    # CPU (~460 ms vs ~330 ms per frame here — measured with depth_bag_eval.py).

    health_info = {
        "service": "depth",
        "model": args.model,
        "device": device,
        "gpu": torch.cuda.get_device_name(0) if device != "cpu" else None,
        "fp16": use_fp16,
        "metric": args.metric,
        "torch": torch.__version__,
        "cuda_available": torch.cuda.is_available(),
    }

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"depth server listening on {args.host}:{args.port}")

    while True:
        conn, addr = srv.accept()
        print("client", addr)
        try:
            while True:
                hdr = recvall(conn, 4)
                if hdr is None:
                    break
                (n,) = struct.unpack(">I", hdr)
                if n == HEALTH_MAGIC:
                    send_health(conn, health_info)
                    print("health check")
                    continue
                blob = recvall(conn, n)
                if blob is None:
                    break
                depth = None
                try:
                    img = Image.open(io.BytesIO(blob)).convert("RGB")
                    W, H = img.size
                    t0 = time.perf_counter()
                    inputs = proc(images=img, return_tensors="pt").to(device)
                    if use_fp16:
                        inputs = inputs.to(torch.float16)
                    with torch.no_grad():
                        pred = model(**inputs).predicted_depth
                    out = (
                        F.interpolate(
                            pred[None].float(),
                            size=(H, W),
                            mode="bicubic",
                            align_corners=False,
                        )[0, 0]
                        .cpu()
                        .numpy()
                        .astype(np.float32)
                    )
                    if not args.metric:  # disparity -> depth; clamp far (disp~0)
                        out = (1.0 / np.maximum(out, 1e-3)).astype(np.float32)
                    dt = (time.perf_counter() - t0) * 1000
                    depth, log = out, f"served {W}x{H} in {dt:.0f} ms"
                except OSError as e:
                    log = f"bad frame ({e}); skipping"
                except Exception as e:
                    log = f"inference failed ({e}); skipping"
                if depth is None:
                    send_rejection(conn)
                else:
                    send_depth(conn, depth)
                print(log)
        except OSError:
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()

"""Off-board monocular-depth inference server (runs in the pixi depth environment).

Keeps Depth Anything V2 resident and serves depth over a local socket so the ROS
node (which has no torch) can stay light and run anywhere — on the workstation
next to this server, or on the robot talking to it over the network. This is the
deliberate two-process split that keeps torch out of the ROS/robot environment.

Default is the *relative* (SSI) V2-Small model: since the node refits a full affine
in disparity against lidar every frame, the absolute scale of a metric model is
discarded anyway, and the relative model measured both more accurate and faster (see
tools/depth_bag_eval.py). Relative models output disparity, so it's inverted to depth
here; pass --metric for a metric model (depth already in metres, no inversion).

Protocol (length-prefixed, big-endian):
  request : uint32 nbytes, then `nbytes` of JPEG/PNG-compressed image
  reply   : uint32 H, uint32 W, then H*W float32 depth (row-major, metres)
"""

import argparse
import io
import os
import socket
import struct
import time

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

MODEL = (
    "depth-anything/Depth-Anything-V2-Small-hf"  # relative (SSI); see module docstring
)


def recvall(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--model", default=MODEL)
    # Default model is relative (outputs disparity, near=large) -> invert to depth so
    # the node (which expects depth, then refits scale) works. --metric: the model
    # already outputs metric depth, so pass it through unchanged.
    ap.add_argument("--metric", action="store_true")
    args = ap.parse_args()

    print("loading", args.model, "(metric)" if args.metric else "(relative)")
    proc = AutoImageProcessor.from_pretrained(args.model)
    model = AutoModelForDepthEstimation.from_pretrained(args.model).eval()
    torch.set_num_threads(os.cpu_count())

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
                blob = recvall(conn, n)
                if blob is None:
                    break
                img = Image.open(io.BytesIO(blob)).convert("RGB")
                W, H = img.size
                t0 = time.perf_counter()
                inputs = proc(images=img, return_tensors="pt")
                with torch.no_grad():
                    pred = model(**inputs).predicted_depth
                depth = (
                    F.interpolate(
                        pred[None], size=(H, W), mode="bicubic", align_corners=False
                    )[0, 0]
                    .numpy()
                    .astype(np.float32)
                )
                if not args.metric:  # disparity -> depth; clamp far (disp~0) to ~1 km
                    depth = (1.0 / np.maximum(depth, 1e-3)).astype(np.float32)
                dt = (time.perf_counter() - t0) * 1000
                conn.sendall(struct.pack(">II", H, W) + depth.tobytes())
                print(f"served {W}x{H} in {dt:.0f} ms")
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()

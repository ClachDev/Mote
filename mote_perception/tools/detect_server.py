"""Off-board open-vocabulary detection server (runs in the pixi inference environment).

Keeps OWLv2 resident and serves object detections over a local socket so the
ROS node (which has no torch) can stay light and run anywhere — the same
two-process split as tools/depth_server.py. OWLv2 detects arbitrary text
queries ("a red box", "shoe"), so the label set rides in each request and new
fetch targets need no retraining or restart.

The wire protocol lives in mote_perception/detect_wire.py (shared with the node
and the offline tools). This file runs uninstalled in the torch env, so the
package is imported straight from the source tree.
"""

import argparse
import io
import os
import socket
import struct
import sys
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import Owlv2ForObjectDetection, Owlv2Processor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_perception.detect_wire import (  # noqa: E402
    DEFAULT_PORT,
    HEALTH_MAGIC,
    recvall,
    repo_revision,
    send_detections,
    send_health,
    send_rejection,
)

MODEL = "google/owlv2-base-patch16-ensemble"


def detect(proc, model, img, labels, threshold, device):
    """Run OWLv2 on one image: [(label_index, score, (x0, y0, x1, y1)), ...].

    The processor pads the image bottom-right to a square before resizing, and
    the model's normalised boxes span that padded square — so post-processing
    with the square's side as the target size yields pixel coordinates in the
    original image directly, independent of how the installed transformers
    version handles the padding. Corners are clamped to the true image bounds.
    """
    W, H = img.size
    side = max(W, H)
    inputs = proc(text=[labels], images=img, return_tensors="pt").to(device)
    with torch.no_grad():
        outputs = model(**inputs)
    res = proc.post_process_object_detection(
        outputs,
        threshold=threshold,
        target_sizes=torch.tensor([(side, side)], device=device),
    )[0]
    out = []
    for idx, score, box in zip(res["labels"], res["scores"], res["boxes"]):
        x0, y0, x1, y1 = box.tolist()
        out.append(
            (
                int(idx),
                float(score),
                (
                    min(max(x0, 0.0), W),
                    min(max(y0, 0.0), H),
                    min(max(x1, 0.0), W),
                    min(max(y1, 0.0), H),
                ),
            )
        )
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--model", default=MODEL)
    # Low floor so score policy stays client-side (the node's min_score param);
    # this only trims the wire traffic of clear noise.
    ap.add_argument("--threshold", type=float, default=0.1)
    ap.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    args = ap.parse_args()

    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device

    print("loading", args.model)
    proc = Owlv2Processor.from_pretrained(args.model)
    model = Owlv2ForObjectDetection.from_pretrained(args.model).eval().to(device)
    if device == "cpu":
        # OWLv2 is CPU-bound here; use all cores. On GPU, leave torch's threading
        # alone — the heavy work runs on the device.
        torch.set_num_threads(os.cpu_count())
        print("using CPU")
    else:
        print("using GPU:", torch.cuda.get_device_name(0))

    health_info = {
        "service": "detect",
        "model": args.model,
        "device": device,
        "gpu": torch.cuda.get_device_name(0) if device != "cpu" else None,
        "threshold": args.threshold,
        "torch": torch.__version__,
        "cuda_available": torch.cuda.is_available(),
        "version": repo_revision(),
    }

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"detect server listening on {args.host}:{args.port}")

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
                hdr = recvall(conn, 4)
                if hdr is None:
                    break
                text = recvall(conn, struct.unpack(">I", hdr)[0])
                if text is None:
                    break
                labels = text.decode("utf-8").splitlines()
                dets = None
                try:
                    img = Image.open(io.BytesIO(blob)).convert("RGB")
                    t0 = time.perf_counter()
                    dets = detect(proc, model, img, labels, args.threshold, device)
                    dt = (time.perf_counter() - t0) * 1000
                    log = f"served {labels} -> {len(dets)} in {dt:.0f} ms"
                except OSError as e:
                    log = f"bad frame ({e}); skipping"
                except Exception as e:
                    log = f"inference failed ({e}); skipping"
                if dets is None:
                    send_rejection(conn)
                else:
                    send_detections(conn, dets)
                print(log)
        except OSError:
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()

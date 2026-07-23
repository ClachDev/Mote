"""Run the open-vocabulary detector over a recorded bag and write overlays.

Re-runs the stages the live node runs (server detection -> floor grounding)
on sampled frames and renders each with its boxes, scores, and the grounded
base-frame floor position of every detection that lands inside the trusted
range. The committed sanity harness for detector quality on real footage.

Needs a detect server listening (pixi run detect-server, or point --host at one):
    pixi run python mote_perception/tools/detect_bag.py <bag> "shoe, box" [--out DIR]
"""

import argparse
import os
import sys
import tempfile

import cv2
import numpy as np

import bag_utils
from mote_perception.detect_wire import DEFAULT_PORT, DetectClient
from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
)

MIN_SCORE, RANGE_MAX = 0.3, 3.0  # match the node's defaults


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("labels", help="comma-separated open-vocabulary queries")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--out", default=os.path.join(tempfile.gettempdir(), "detect_bag"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--min-score", type=float, default=MIN_SCORE)
    # OWLv2's first (cold) inference on CPU can take >10 s, especially with many
    # labels; the default socket timeout is generous so that frame doesn't drop.
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()
    labels = [w.strip() for w in args.labels.split(",") if w.strip()]

    imgs, _, tf_static, caminfo = bag_utils.load_perception_bag(args.bag)
    T_bo = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    proj = GroundProjector.from_camera_info(caminfo, T_bo)
    client = DetectClient(args.host, args.port, timeout=args.timeout)

    os.makedirs(args.out, exist_ok=True)
    step = max(1, len(imgs) // args.frames)
    for i, (stamp, blob) in enumerate(imgs[::step][: args.frames]):
        dets = client.infer(blob, labels)
        if dets is None:
            sys.exit("no detect server; start it with: pixi run detect-server")
        dets = [d for d in dets if d[1] >= args.min_score]
        img = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
        lines = []
        for label, score, (x0, y0, x1, y1) in dets:
            pt = proj.pixels_to_ground([[(x0 + x1) / 2.0, y1]])[0]
            ranged = np.isfinite(pt).all() and np.hypot(pt[0], pt[1]) <= RANGE_MAX
            where = f"({pt[0]:.2f}, {pt[1]:.2f})" if ranged else "out of range"
            color = (0, 255, 0) if ranged else (0, 165, 255)
            cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), color, 2)
            cv2.putText(
                img,
                f"{label} {score:.0%} {where}",
                (int(x0), max(int(y0) - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
            lines.append(f"{label} {score:.0%} {where}")
        path = os.path.join(args.out, f"frame_{i:03d}.jpg")
        cv2.imwrite(path, img)
        print(f"{path}: " + ("; ".join(lines) if lines else "nothing"))
    client.close()
    print(f"overlays in {args.out}")


if __name__ == "__main__":
    main()

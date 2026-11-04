#!/usr/bin/env python3
"""Measure the step `icp_odom_gate` puts into odom->base at each scan.

The gate publishes odom->base at the wheel rate: between scans it carries the
last gated pose forward on the wheel increments, and each scan re-anchors it on
the gated ICP increment. Every publication is therefore the previous one moved
by the wheels, except where a scan has re-anchored the track in between; there
the difference is the step a TF reader sees.

So the step is measured from the outside, with no model of the gate: for each
consecutive pair of published poses, the residual between the later pose and
the earlier one composed with the wheel increment over the same interval. It is
zero to rounding between scans and the step at a re-anchor.

    pixi run -- python mote_bringup/tools/odom_steps.py <bag>   # /tf + wheel odom

The bag needs the `odom -> base_footprint` broadcasts and the wheel odometry
(`/diff_drive_controller/odom`, or the inverted `odom_wheel` leaf).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bag_odometry import read_samples  # noqa: E402

from mote_bringup.odom_residual import rel_motion, yaw_of_quat  # noqa: E402

GATED_EDGE = ("odom", "base_footprint")
# Below this a residual is rounding, not a re-anchor.
EPS = 1e-7


def read_edge(bag: Path, pair=GATED_EDGE):
    """``(t, x, y, yaw)`` for every broadcast of one TF edge, sorted by stamp."""
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            if (tr.header.frame_id, tr.child_frame_id) == pair:
                p, q = tr.transform.translation, tr.transform.rotation
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                out.append((t, p.x, p.y, yaw_of_quat(q.x, q.y, q.z, q.w)))
    out.sort(key=lambda s: s[0])
    return out


def _compose(x, y, yaw, dx, dy, dyaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, yaw + dyaw


def residuals(gated, wheel):
    """``(t, dt, translation, yaw)`` per consecutive pair of published poses.

    The wheel pose is interpolated at each published stamp, linearly in
    translation and in unwrapped yaw -- what the gate itself does between wheel
    samples. Pairs outside the wheel stream's span are skipped.
    """
    w = np.asarray(wheel, dtype=float)
    wt, wx, wy = w[:, 0], w[:, 1], w[:, 2]
    wyaw = np.unwrap(w[:, 3])

    def wheel_at(t):
        return (
            float(np.interp(t, wt, wx)),
            float(np.interp(t, wt, wy)),
            float(np.interp(t, wt, wyaw)),
        )

    out = []
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(gated, gated[1:]):
        if t0 < wt[0] or t1 > wt[-1] or t1 <= t0:
            continue
        inc = rel_motion(*wheel_at(t0), *wheel_at(t1))
        ex, ey, ea = _compose(x0, y0, a0, *inc)
        dx, dy, dyaw = rel_motion(ex, ey, ea, x1, y1, a1)
        out.append((t1, t1 - t0, math.hypot(dx, dy), dyaw))
    return out


def summarise(rows, label=""):
    """Print the step distribution; returns the re-anchoring rows."""
    steps = [r for r in rows if r[2] > EPS or abs(r[3]) > EPS]
    span = rows[-1][0] - rows[0][0] if rows else 0.0
    rate = len(rows) / span if span > 0 else 0.0
    print(
        f"{label}{len(rows)} publications ({rate:.1f} Hz), "
        f"{len(steps)} re-anchoring steps"
    )
    if not steps:
        return steps
    trans = np.array([r[2] for r in steps]) * 1000.0
    yaw = np.degrees(np.abs([r[3] for r in steps]))
    q = (50, 90, 95, 99, 100)
    print(
        "  translation mm  "
        + "  ".join(f"p{p}={v:.2f}" for p, v in zip(q, np.percentile(trans, q)))
    )
    print(
        "  yaw deg         "
        + "  ".join(f"p{p}={v:.3f}" for p, v in zip(q, np.percentile(yaw, q)))
    )
    return steps


def write_csv(path: Path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "dt", "translation_m", "yaw_rad"])
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("--csv", type=Path, help="write every residual here")
    args = ap.parse_args()

    gated = read_edge(args.bag)
    wheel, _, _ = read_samples(args.bag)
    if len(gated) < 2 or len(wheel) < 2:
        raise SystemExit("bag lacks odom->base_footprint or the wheel odometry")
    rows = residuals(gated, wheel)
    summarise(rows, f"{args.bag.name}: ")
    if args.csv:
        write_csv(args.csv, rows)


if __name__ == "__main__":
    main()

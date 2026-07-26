#!/usr/bin/env python3
"""Score kinematic_icp against wheel odometry on a recorded mapping bag.

Mote carries two independent motion sources: wheel odometry, and kinematic_icp's
scan-matched pose. kinematic_icp *takes* the wheel odom as its prior and corrects
it, so the disagreement between them is a direct measure of how wrong the prior
was — a slip signal, and a lidar-odometry health check, needing no extra sensor.

Both signals are already in a mapping bag's ``/tf``, so nothing is replayed:

    odom -> base_footprint        kinematic_icp's scan-matched pose (lidar rate)
    base_footprint -> odom_wheel  the inverted wheel-odom leaf (odom_tf_relay)

Inverting the second gives the wheel-odom pose of base. Both frames start
coincident, so the trajectories are directly comparable.

Two things are reported:

* **Per-interval residual** — over each lidar-rate interval, the relative motion
  each source reports, expressed in the body frame. Accumulated drift is ordinary
  integration error; it is the *incremental* disagreement that flags an event.
* **Impossible-velocity frames** — intervals where the ICP pose implies a body
  speed above ``robot.yaml``'s measured ``max_wheel_speed``. Wheel slip cannot
  cause this (slip makes the *wheels* over-read, never the lidar), so these are
  scan-match excursions. Isolated single-frame runs indicate momentary jumps
  rather than sustained misregistration.

Caveat: wheel odom (~100 Hz) is resampled onto ICP stamps (~10 Hz), so during
fast in-place turns a small stamp misalignment inflates the *yaw* residual (at
90 deg/s, 10 ms of skew alone looks like ~9 deg/s). Treat the yaw residual as
indicative until the two streams are properly time-synced.

    pixi run -- python mote_bringup/tools/odom_health.py ~/.mote/bags/mapping/<run>
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

ICP_EDGE = ("odom", "base_footprint")
WHEEL_LEAF = ("base_footprint", "odom_wheel")


def max_wheel_speed() -> float:
    with open(
        Path(get_package_share_directory("mote_description")) / "config" / "robot.yaml"
    ) as f:
        return float(yaml.safe_load(f)["max_wheel_speed"])


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_tf(bag: Path):
    """Extract the ICP pose and the (inverted) wheel-odom pose from /tf."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))
    icp, wheel = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            p, q = tr.transform.translation, tr.transform.rotation
            pair = (tr.header.frame_id, tr.child_frame_id)
            if pair == ICP_EDGE:
                icp.append([t, p.x, p.y, yaw_of(q)])
            elif pair == WHEEL_LEAF:
                a = yaw_of(q)
                c, s = np.cos(-a), np.sin(-a)
                wheel.append([t, -(c * p.x - s * p.y), -(s * p.x + c * p.y), -a])
    return np.array(icp), np.array(wheel)


def rel_motion(x0, y0, a0, x1, y1, a1):
    """Motion from pose0 to pose1, expressed in pose0's body frame."""
    dx, dy = x1 - x0, y1 - y0
    c, s = np.cos(-a0), np.sin(-a0)
    return (
        c * dx - s * dy,
        s * dx + c * dy,
        np.arctan2(np.sin(a1 - a0), np.cos(a1 - a0)),
    )


def runs_of(mask):
    """Lengths of consecutive-True runs."""
    out, n = [], 0
    for v in mask:
        if v:
            n += 1
        elif n:
            out.append(n)
            n = 0
    if n:
        out.append(n)
    return out


def analyse(bag: Path, vmax: float, tol: float):
    icp, wheel = read_tf(bag)
    if icp.shape[0] < 10 or wheel.shape[0] < 10:
        print(
            f"{bag.name}: insufficient /tf data "
            f"(icp={icp.shape[0]}, wheel={wheel.shape[0]})"
        )
        return

    lo, hi = max(icp[0, 0], wheel[0, 0]), min(icp[-1, 0], wheel[-1, 0])
    m = (icp[:, 0] >= lo) & (icp[:, 0] <= hi)
    t = icp[m, 0]
    ix, iy, ia = icp[m, 1], icp[m, 2], np.unwrap(icp[m, 3])
    wx = np.interp(t, wheel[:, 0], wheel[:, 1])
    wy = np.interp(t, wheel[:, 0], wheel[:, 2])
    wa = np.interp(t, wheel[:, 0], np.unwrap(wheel[:, 3]))

    dt = np.diff(t)
    ok = dt > 1e-3
    idx, idy, ida = rel_motion(ix[:-1], iy[:-1], ia[:-1], ix[1:], iy[1:], ia[1:])
    wdx, wdy, wda = rel_motion(wx[:-1], wy[:-1], wa[:-1], wx[1:], wy[1:], wa[1:])

    i_d, w_d = np.hypot(idx, idy)[ok], np.hypot(wdx, wdy)[ok]
    i_a, w_a, dtv = ida[ok], wda[ok], dt[ok]
    iv, wv = i_d / dtv, w_d / dtv

    moving = wv > 0.02
    res_v = (w_d - i_d) / dtv
    res_a = np.degrees((w_a - i_a) / dtv)

    limit = vmax * tol
    imposs = iv > limit
    r = runs_of(imposs)

    print(f"\n=== {bag.name}  ({t[-1] - t[0]:.0f}s, {iv.size} intervals) ===")
    print(
        f"  path length   icp {np.sum(i_d):7.2f} m   wheel {np.sum(w_d):7.2f} m"
        f"   ({100 * (np.sum(w_d) - np.sum(i_d)) / max(np.sum(i_d), 1e-9):+.1f}% wheel)"
    )
    print(
        f"  yaw travelled icp {np.degrees(np.sum(np.abs(i_a))):7.1f} deg wheel "
        f"{np.degrees(np.sum(np.abs(w_a))):7.1f} deg"
        f"   ({100 * (np.sum(np.abs(w_a)) - np.sum(np.abs(i_a))) / max(np.sum(np.abs(i_a)), 1e-9):+.1f}% wheel)"
    )
    if moving.any():
        print("  residual |wheel-icp| while moving:")
        print(
            f"      translation  p50 {np.percentile(np.abs(res_v[moving]), 50):.4f}"
            f"  p99 {np.percentile(np.abs(res_v[moving]), 99):.4f}"
            f"  max {np.max(np.abs(res_v[moving])):.4f} m/s"
        )
        print(
            f"      yaw          p50 {np.percentile(np.abs(res_a[moving]), 50):.2f}"
            f"  p99 {np.percentile(np.abs(res_a[moving]), 99):.2f}"
            f"  max {np.max(np.abs(res_a[moving])):.2f} deg/s  (see time-sync caveat)"
        )
    print(
        f"  ICP speed above {limit:.3f} m/s (drive cannot produce it): "
        f"{np.count_nonzero(imposs)} intervals ({100 * imposs.mean():.2f}%)"
    )
    print(
        f"  wheel speed above the same:                        "
        f"{np.count_nonzero(wv > limit)} intervals ({100 * (wv > limit).mean():.2f}%)"
    )
    if r:
        print(
            f"      runs: n={len(r)}, longest={max(r)} "
            f"(~{max(r) * np.median(dtv):.2f}s), isolated={sum(1 for x in r if x == 1)}"
        )
    print(
        f"  speed p50/p99/max   icp {np.percentile(iv, 50):.3f}/"
        f"{np.percentile(iv, 99):.3f}/{iv.max():.3f}   wheel "
        f"{np.percentile(wv, 50):.3f}/{np.percentile(wv, 99):.3f}/{wv.max():.3f} m/s"
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bags", nargs="+", type=Path, help="mapping bag directories")
    ap.add_argument(
        "--tol",
        type=float,
        default=1.15,
        help="fraction of max_wheel_speed treated as impossible",
    )
    args = ap.parse_args()
    vmax = max_wheel_speed()
    print(f"max_wheel_speed = {vmax:.3f} m/s (robot.yaml), tolerance x{args.tol}")
    for bag in args.bags:
        analyse(bag, vmax, args.tol)


if __name__ == "__main__":
    main()

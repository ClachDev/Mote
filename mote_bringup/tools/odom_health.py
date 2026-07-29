#!/usr/bin/env python3
"""Score kinematic_icp against wheel odometry over a whole recorded session.

Mote carries two independent motion sources: wheel odometry, and kinematic_icp's
scan-matched pose. kinematic_icp *takes* the wheel odom as its prior and corrects
it, so the disagreement between them is a direct measure of how wrong the prior
was — a slip signal, and a lidar-odometry health check, needing no extra sensor.

This is the *survey* view of a session: totals and distributions over every
interval. For the verdicts the robot itself would have raised, use
``slip_replay.py``, which drives the live detector over the same bag. Both read
the bag through ``bag_odometry.read_samples`` and share ``rel_motion`` with the
node, so neither tool can drift from what runs on the robot.

Two things are reported:

* **Per-interval residual** — over each lidar-rate interval, the relative motion
  each source reports, expressed in the body frame. Accumulated drift is ordinary
  integration error; it is the *incremental* disagreement that flags an event.
* **Impossible-velocity frames** — intervals where the ICP pose implies a body
  speed above ``robot.yaml``'s measured ``max_wheel_speed``. Wheel slip cannot
  cause this (slip makes the *wheels* over-read, never the lidar), so these are
  scan-match excursions. Isolated single-frame runs indicate momentary jumps
  rather than sustained misregistration.

On the yaw residual: it is reported, but at this interval length it is dominated
by scan-match jitter rather than by any real disagreement. A lag sweep over the
recorded bags puts the two streams within +/-10 ms of each other, so stamp skew
does not explain it; it simply averages down as the comparison window grows
(p50 ~3.0 deg/s at 0.1 s, ~1.1 at 1.0 s, ~0.5 at 2.0 s). That is why the live
detector thresholds translation only — see
``docs/tuning/2026-07-28-slip-detection.md``.

    pixi run -- python mote_bringup/tools/odom_health.py ~/.mote/bags/mapping/<run>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bag_odometry import read_samples  # noqa: E402

from mote_bringup.odom_residual import rel_motion  # noqa: E402


def max_wheel_speed() -> float:
    with open(
        Path(get_package_share_directory("mote_description")) / "config" / "robot.yaml"
    ) as f:
        return float(yaml.safe_load(f)["max_wheel_speed"])


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


def rel_motion_series(x, y, a):
    """rel_motion applied pairwise down a trajectory.

    The scalar rel_motion is the one the node runs; looping it here rather than
    keeping a vectorised twin is what stops the two from drifting apart. A
    session is a few tens of thousands of intervals, so the loop costs nothing.
    """
    steps = [
        rel_motion(x[i], y[i], a[i], x[i + 1], y[i + 1], a[i + 1])
        for i in range(len(x) - 1)
    ]
    dx, dy, da = zip(*steps) if steps else ((), (), ())
    return np.array(dx), np.array(dy), np.array(da)


def analyse(bag: Path, vmax: float, tol: float):
    wheel_samples, icp_samples, _ = read_samples(bag)
    icp, wheel = np.array(icp_samples), np.array(wheel_samples)
    if icp.shape[0] < 10 or wheel.shape[0] < 10:
        print(
            f"{bag.name}: insufficient odometry "
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
    idx, idy, ida = rel_motion_series(ix, iy, ia)
    wdx, wdy, wda = rel_motion_series(wx, wy, wa)

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
            f"  max {np.max(np.abs(res_a[moving])):.2f} deg/s  (jitter-dominated)"
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

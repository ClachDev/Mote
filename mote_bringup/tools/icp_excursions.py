#!/usr/bin/env python3
"""Characterise kinematic_icp velocity excursions recorded in a mapping bag.

``odom_health.py`` counts the intervals where the ICP pose implies a body speed
the drive cannot produce. It does not say whether they matter. This does: for
each excursion it asks the one question that decides it — is the jump a *spike*
the next scan takes back, or a *step* that leaves the ``odom->base`` estimate
permanently displaced?

The two look identical in a per-interval speed histogram and are worlds apart
downstream: a spike briefly disturbs the costmap, a step corrupts the map frame
for the rest of the session and every zone bound in it.

Wheel odometry is the local reference. It is not truth over a session, but over
the couple of seconds either side of one scan it drifts far less than the metres
per second an excursion implies, so the *change* in (icp - wheel) displacement
across the excursion separates the two cases:

    recovered   the along-track gap returns to its pre-jump value      -> spike
    retained    the gap keeps the jump's displacement                  -> step

Also reported per excursion: the body-frame components of the jump, the yaw rate
at the time (scan deskew degrades with rotation, the obvious suspect), and the
wheel speed, which establishes what the robot was actually doing.

    pixi run -- python mote_bringup/tools/icp_excursions.py ~/.mote/bags/mapping/<run>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bag_odometry import read_samples  # noqa: E402
from odom_health import max_wheel_speed, rel_motion_series  # noqa: E402

from mote_bringup.odom_residual import rel_motion  # noqa: E402


def resample(bag: Path):
    """ICP and wheel poses on the common ICP stamps."""
    wheel_samples, icp_samples, _ = read_samples(bag)
    icp, wheel = np.array(icp_samples), np.array(wheel_samples)
    lo, hi = max(icp[0, 0], wheel[0, 0]), min(icp[-1, 0], wheel[-1, 0])
    m = (icp[:, 0] >= lo) & (icp[:, 0] <= hi)
    t = icp[m, 0]
    return (
        t,
        np.column_stack([icp[m, 1], icp[m, 2], np.unwrap(icp[m, 3])]),
        np.column_stack(
            [
                np.interp(t, wheel[:, 0], wheel[:, 1]),
                np.interp(t, wheel[:, 0], wheel[:, 2]),
                np.interp(t, wheel[:, 0], np.unwrap(wheel[:, 3])),
            ]
        ),
    )


def gap(i, w, a, b):
    """Along-track disagreement in metres between poses a and b of both sources.

    Both sources are integrated over the same span and the difference of the
    travelled distances is taken, so a constant offset in either frame cancels.
    """
    di = np.hypot(*rel_motion(*i[a], *i[b])[:2])
    dw = np.hypot(*rel_motion(*w[a], *w[b])[:2])
    return di - dw


def analyse(bag: Path, vmax: float, tol: float, window: float):
    t, i, w = resample(bag)
    dt = np.diff(t)
    ok = dt > 1e-3
    idx, idy, ida = rel_motion_series(*i.T)
    wdx, wdy, wda = rel_motion_series(*w.T)
    iv = np.hypot(idx, idy) / dt
    wv = np.hypot(wdx, wdy) / dt

    limit = vmax * tol
    hits = np.flatnonzero(ok & (iv > limit))
    span = max(1, int(round(window / np.median(dt))))

    print(f"\n=== {bag.name} ({t[-1] - t[0]:.0f}s, {iv.size} intervals) ===")
    print(f"    limit {limit:.3f} m/s, recovery window +-{span} scans (~{window:.1f}s)")
    if hits.size == 0:
        print("    no excursions")
        return []

    rows = []
    for k in hits:
        a, b = max(0, k - span), min(len(t) - 1, k + 1 + span)
        before = gap(i, w, a, k)
        jump = gap(i, w, k, k + 1)
        after = gap(i, w, k + 1, b)
        rows.append(
            dict(
                t=t[k] - t[0],
                dt=dt[k],
                icp_v=iv[k],
                wheel_v=wv[k],
                fwd=idx[k],
                lat=idy[k],
                yaw_rate=np.degrees(ida[k] / dt[k]),
                jump=jump,
                before=before,
                after=after,
                retained=jump + after,
            )
        )

    print(
        "      t(s)  icp_v  whl_v    fwd     lat  yaw/s |"
        "   jump  before   after  retained"
    )
    for r in rows:
        print(
            f"    {r['t']:6.1f} {r['icp_v']:6.3f} {r['wheel_v']:6.3f} "
            f"{r['fwd']:+7.4f} {r['lat']:+7.4f} {r['yaw_rate']:+6.1f} |"
            f" {r['jump']:+6.3f}  {r['before']:+6.3f}  {r['after']:+6.3f}"
            f"   {r['retained']:+7.3f}"
        )

    jmp = np.array([r["jump"] for r in rows])
    before = np.array([r["before"] for r in rows])
    after = np.array([r["after"] for r in rows])

    # Null: what one ordinary interval, and one ordinary `window`, cost in gap.
    per_interval = np.array(
        [gap(i, w, k, k + 1) for k in range(len(t) - 1) if k not in set(hits)]
    )
    print(
        f"\n    single-interval gap, excursions excluded: "
        f"|p50| {np.percentile(np.abs(per_interval), 50):.4f}  "
        f"|p99| {np.percentile(np.abs(per_interval), 99):.4f}  "
        f"|max| {np.abs(per_interval).max():.4f} m"
    )
    print(
        f"    excursion jumps:                          "
        f"|p50| {np.percentile(np.abs(jmp), 50):.4f}  "
        f"|max| {np.abs(jmp).max():.4f} m   "
        f"sum {jmp.sum():+.3f} m over {len(rows)} frames"
    )
    print(
        f"    gap rate around them: before {before.mean() / window:+.4f} m/s   "
        f"after {after.mean() / window:+.4f} m/s   "
        f"(a spike would show a large negative 'after')"
    )
    print(
        f"    -> the {len(rows)} jumps add {jmp.sum():+.3f} m to a "
        f"{np.hypot(*rel_motion(*i[0], *i[-1])[:2]):.1f} m displaced, "
        f"{np.sum(np.hypot(wdx, wdy)):.1f} m driven session"
    )
    # Where can a threshold sit? Express each interval as the surface speed the
    # faster wheel would need — the same quantity the Nav2 critic bounds — so
    # the gate and the critic describe one envelope rather than two.
    sep = 0.22
    icp_wheel = np.hypot(idx, idy)[ok] / dt[ok] + 0.5 * sep * np.abs(ida[ok] / dt[ok])
    whl_wheel = np.hypot(wdx, wdy)[ok] / dt[ok] + 0.5 * sep * np.abs(wda[ok] / dt[ok])
    legit = np.ones(iv.size, bool)
    legit[hits] = False
    legit = legit[ok]
    print(
        f"\n    implied max-wheel speed (|v| + S/2|w|), limit {vmax:.3f} m/s:\n"
        f"      wheels           p50 {np.percentile(whl_wheel, 50):.3f}  "
        f"p99 {np.percentile(whl_wheel, 99):.3f}  max {whl_wheel.max():.3f} m/s"
        f"   ({100 * (whl_wheel > vmax).mean():.2f}% over)\n"
        f"      icp, excursions excluded  p99 {np.percentile(icp_wheel[legit], 99):.3f}  "
        f"max {icp_wheel[legit].max():.3f} m/s "
        f"(x{icp_wheel[legit].max() / vmax:.2f} of limit)\n"
        f"      icp, excursions           min {icp_wheel[~legit].min():.3f} m/s "
        f"(x{icp_wheel[~legit].min() / vmax:.2f} of limit)"
    )
    yr = np.abs(np.degrees(ida[ok] / dt[ok]))
    print(
        f"    yaw rate  at excursions |p50| "
        f"{np.percentile(np.abs([r['yaw_rate'] for r in rows]), 50):.1f} deg/s"
        f"   session |p50| {np.percentile(yr, 50):.1f}  |p99| {np.percentile(yr, 99):.1f} deg/s"
    )
    print(
        f"    wheel speed at excursions |p50| "
        f"{np.percentile([r['wheel_v'] for r in rows], 50):.3f} m/s"
        f"   session |p50| {np.percentile(wv[ok], 50):.3f}  "
        f"|p99| {np.percentile(wv[ok], 99):.3f} m/s"
    )
    return rows


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("--tol", type=float, default=1.15)
    ap.add_argument(
        "--window",
        type=float,
        default=2.0,
        help="seconds either side to score recovery",
    )
    args = ap.parse_args()
    vmax = max_wheel_speed()
    print(f"max_wheel_speed = {vmax:.3f} m/s (robot.yaml), tolerance x{args.tol}")
    for bag in args.bags:
        analyse(bag, vmax, args.tol, args.window)


if __name__ == "__main__":
    main()

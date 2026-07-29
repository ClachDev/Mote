#!/usr/bin/env python3
"""Replay a recorded bag through the live slip detector.

This is how ``config/slip.yaml``'s thresholds were set, and how a change to them
is checked: it drives the *same* ``ResidualEstimator`` and ``classify`` the
``slip_monitor`` node runs, from a bag's recorded odometry, and reports both the
distribution of the residual and every verdict the filter would have raised.

Both motion sources are already in a mapping bag's ``/tf``:

    odom -> base_footprint        kinematic_icp's scan-matched pose (~10 Hz)
    base_footprint -> odom_wheel  the inverted wheel-odom leaf (~100 Hz)

``odom_tf_relay`` writes that leaf straight from ``/diff_drive_controller/odom``,
stamp and pose unchanged, so replaying the leaf and subscribing the topic feed
the estimator identical numbers. ``/diff_drive_controller/odom`` is used instead
when the bag happens to carry it.

    pixi run -- python mote_bringup/tools/slip_replay.py ~/.mote/bags/mapping/*/
    pixi run -- python mote_bringup/tools/slip_replay.py --verdicts-only <bag>...
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bag_odometry import read_samples  # noqa: E402

from mote_bringup.odom_residual import (  # noqa: E402
    OK,
    UNKNOWN,
    ResidualEstimator,
    Thresholds,
    VerdictFilter,
    classify,
)


def _share(package, *parts):
    return Path(get_package_share_directory(package)).joinpath(*parts)


def load_thresholds(path=None):
    cfg_path = path or _share("mote_bringup", "config", "slip.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f) or {}
    with open(_share("mote_description", "config", "robot.yaml")) as f:
        max_wheel_speed = float(yaml.safe_load(f)["max_wheel_speed"])
    fields = {k: v for k, v in cfg.items() if k in Thresholds.__dataclass_fields__}
    tolerance = float(cfg.get("max_body_speed_tolerance", 1.15))
    return Thresholds(**fields).with_max_wheel_speed(max_wheel_speed, tolerance)


def replay(bag: Path, thresholds, rate=10.0):
    """Drive the live estimator over a bag; returns (residuals, verdict events)."""
    wheel, icp, command = read_samples(bag)
    if len(wheel) < 10 or len(icp) < 10:
        return None, None, (len(wheel), len(icp))

    estimator = ResidualEstimator(thresholds)
    vfilter = VerdictFilter(thresholds)
    residuals, events = [], []
    reported = OK

    # One merged pass in stamp order, evaluating on a fixed grid — the node
    # evaluates on a timer, not on message arrival.
    merged = [(s[0], "w", s[1:]) for s in wheel]
    merged += [(s[0], "i", s[1:]) for s in icp]
    merged.sort(key=lambda m: m[0])

    ci = 0
    last_command = None
    t0 = merged[0][0]
    next_eval = t0 + thresholds.window
    for t, kind, pose in merged:
        if kind == "w":
            estimator.add_wheel(t, *pose)
        else:
            estimator.add_icp(t, *pose)
        while ci < len(command) and command[ci][0] <= t:
            last_command = (command[ci][1], command[ci][2])
            ci += 1
        if t < next_eval:
            continue
        next_eval = t + 1.0 / rate
        residual = estimator.residual(now=t)
        verdict = classify(residual, thresholds, last_command, estimator.reason)
        if residual is not None:
            residuals.append((t - t0, residual, verdict))
        current = vfilter.update(t, verdict)
        # OK and UNKNOWN are both quiet: the robot standing still is not an event.
        quiet = {OK, UNKNOWN}
        if (current.state in quiet) != (reported in quiet) or (
            current.state not in quiet and current.state != reported
        ):
            events.append((t - t0, reported, current))
        reported = current.state
    return residuals, events, (len(wheel), len(icp))


def _pct(values, p):
    if not values:
        return float("nan")
    ordered = sorted(values)
    k = min(len(ordered) - 1, max(0, int(round(p / 100 * (len(ordered) - 1)))))
    return ordered[k]


def report(bag, residuals, events, counts, verdicts_only):
    if residuals is None:
        print(
            f"\n### {bag.name}: insufficient odometry (wheel={counts[0]}, icp={counts[1]})"
        )
        return 0
    moving = [r for _, r, v in residuals if v.state != UNKNOWN and r.scale > 0.03]
    duration = residuals[-1][0] if residuals else 0.0
    print(
        f"\n### {bag.name}  {duration:.0f}s  {len(residuals)} windows, {len(moving)} moving"
    )

    if not verdicts_only and moving:
        resid = [r.speed_residual for r in moving]
        rel = [r.relative for r in moving]
        yaw = [r.yaw_rate_residual for r in moving]
        print(
            f"    speed residual   p1 {_pct(resid, 1):+.4f}  p50 {_pct(resid, 50):+.4f}"
            f"  p99 {_pct(resid, 99):+.4f}  max {max(resid, key=abs):+.4f} m/s"
        )
        print(
            f"    relative         p1 {_pct(rel, 1):+.3f}  p50 {_pct(rel, 50):+.3f}"
            f"  p99 {_pct(rel, 99):+.3f}  max {max(rel, key=abs):+.3f}"
        )
        print(
            f"    yaw residual     p1 {_pct(yaw, 1):+.3f}  p50 {_pct(yaw, 50):+.3f}"
            f"  p99 {_pct(yaw, 99):+.3f} rad/s   (logged, never thresholded)"
        )
        print(f"    stdev of speed residual  {statistics.pstdev(resid):.4f} m/s")

    raised = [e for e in events if e[2].state not in (OK, UNKNOWN)]
    if not raised:
        print("    verdicts: none — clean throughout")
    for t, _, current in events:
        if current.state in (OK, UNKNOWN):
            print(f"    t={t:7.1f}s  cleared")
        else:
            print(f"    t={t:7.1f}s  {current.state.upper()}: {current.detail}")
    return len(raised)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--config", type=Path, help="slip.yaml to use instead of the packaged one"
    )
    ap.add_argument(
        "--verdicts-only", action="store_true", help="skip the distributions"
    )
    ap.add_argument("--rate", type=float, default=10.0, help="evaluation rate, Hz")
    args = ap.parse_args()

    thresholds = load_thresholds(args.config)
    print(
        f"window {thresholds.window}s  slip > {thresholds.slip_speed} m/s and "
        f"{100 * thresholds.slip_fraction:.0f}%  icp_fault > {thresholds.icp_speed} m/s "
        f"and {100 * thresholds.icp_fraction:.0f}%  or > {thresholds.max_body_speed:.3f} m/s"
        f"  hold {thresholds.hold}s / release {thresholds.release}s"
    )
    total = 0
    for bag in args.bags:
        residuals, events, counts = replay(bag, thresholds, args.rate)
        total += report(bag, residuals, events, counts, args.verdicts_only)
    print(f"\n{total} verdict(s) raised across {len(args.bags)} bag(s)")


if __name__ == "__main__":
    main()

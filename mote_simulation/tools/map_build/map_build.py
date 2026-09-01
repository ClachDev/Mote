#!/usr/bin/env python3
"""``map-build`` — a mapping bag in, a reviewable map candidate out.

Stage 2 of the mapping pipeline (``docs/design/mapping-pipeline.md``). Capture
produces a bag; this produces a map revision; a human promotes it. The bag is
the source, the build parameters are the toolchain, and the revision is a build
artifact — so it is cheap to rebuild, and every rebuild names its exact inputs.

    pixi run map-build -- --bag ~/.mote/bags/mapping/20260802_142539 \\
        --site home --floor ground \\
        --baseline ~/.mote/sites/home/floors/ground/map

The chain, which is what the 2026-08-02 flat session ran by hand:

1. **solve** — lockstep replay of the whole bag under the committed *build*
   parameters (``slam_toolbox_build_params.yaml``), through the harness that
   already exists (``tools/bag_replay``). Minutes, not the bag's own duration.
2. **assemble** — the finished grid and the serialized posegraph, written out
   in the layout ``save-map`` writes, because there is exactly one shape of
   revision and the registry knows it.
3. **declutter** — ``sites.promote_cleaned``: the robot's own FFT structure
   pass, not a copy of it. The raw map_saver-shaped image is kept.
4. **segment** — one polygon zone per room of the cleaned map.
5. **validate + score** — ``bundle.validate`` (hard: a revision that fails is
   not emitted), then truth-free metrics diffed against a baseline revision
   (soft: a regression is printed for the reviewer).
6. **package** — the revision directory plus the gzipped bundle the registry
   accepts, and a build report.

**Two steps of the design are not here, deliberately.**

*Alignment* — measure the wall rotation, re-solve with it injected, keep the
better map — needs an estimator that can tell which map is better. The one in
the tree cannot: it called four of the 2026-08-02 solves square when they were
3.5–5.6° off (``docs/tuning/2026-09-01-alignment-residual.md``, task 615). A
re-solve is not a rigid rotation either, so the step is *undecidable* rather
than merely ungated, and building it on a measurement that cannot see would be
worse than not building it — the design says so in as many words. Until 615
lands, birth-alignment is an operator's judgment: ``--frame X Y YAW``, recorded
in the revision's meta so the map stays reproducible.

*Vocabulary carry-forward* is task 345. The build emits the segmenter's
placeholder room names and **reports** what the baseline floor was called, so
the gap is visible rather than silent.

*Upload* needs a build identity (task 344): today's registry accepts candidate
uploads only from enrolled robots. The build therefore emits the packed bundle
locally and prints what will send it.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
BAG_REPLAY = REPO / "mote_simulation" / "tools" / "bag_replay"
BUILD_PARAMS = REPO / "mote_bringup" / "config" / "slam_toolbox_build_params.yaml"

sys.path.insert(0, str(REPO / "mote_bringup"))
sys.path.insert(0, str(REPO / "mote_simulation" / "tools" / "benchmark"))
sys.path.insert(0, str(BAG_REPLAY))
sys.path.insert(0, str(HERE))

import build_report  # noqa: E402
import revision as rev  # noqa: E402

LEG = "build"


def log(msg):
    print(f"[map-build] {msg}", flush=True)


class BuildFailed(Exception):
    """A gate the build must not walk past. Nothing is emitted."""


def solve(bag: Path, params: Path, run_dir: Path, args) -> dict:
    """Replay the whole bag through slam_toolbox, in lockstep.

    The harness is called in-process rather than re-implemented: it owns the
    DDS isolation, the stack launch, the acceptance-chain feed and the
    teardown, and a second copy of any of that would be a second thing to keep
    in step with slam_toolbox.
    """
    import replay

    replay_args = argparse.Namespace(
        rate=1.0,
        settle=args.settle,
        max_scans=args.max_scans,
        skip_secs=args.skip_secs,
        stop_secs=args.stop_secs,
        frame=args.frame,
        lockstep=not args.paced,
        boot_timeout=args.boot_timeout,
        replay_timeout=args.replay_timeout,
    )
    leg = replay.run_one(bag, params, LEG, "slam", run_dir, replay_args)
    if leg is None:
        raise BuildFailed(
            f"the solve did not complete — see {run_dir / LEG}/stack.log and replay.log"
        )
    if not leg.get("map_npz"):
        raise BuildFailed("the solve produced no occupancy grid")
    return leg


def assemble(leg: dict, rev_dir: Path) -> dict:
    """The map pair and the posegraph, in the layout a revision has."""
    frame = rev.write_map_pair(leg["map_npz"], rev_dir)
    copied = rev.copy_posegraph(Path(leg["map_npz"]).parent, rev_dir)
    if len(copied) != 2:
        raise BuildFailed(
            "the solve serialized no posegraph — a revision without one cannot "
            "be extended later, and the frame is unrecoverable (extend, don't remap)"
        )
    return frame


def declutter(rev_dir: Path, enabled: bool) -> dict:
    """The robot's own cleaning pass, so a built map and a saved one compare."""
    if not enabled:
        return {"skipped": True}
    from mote_bringup import sites

    return sites.promote_cleaned(rev_dir)


def segment(rev_dir: Path, site: str, floor: str, out_dir: Path) -> dict:
    """One polygon zone per room of the cleaned map, plus an overlay to look at.

    Geometry only: the names are ``room_NN`` placeholders for the reviewer to
    replace in the dashboard's zone editor. What a room is *called* is a fact
    about the building that no map holds.
    """
    import cv2

    from mote_bringup.map_cleanup.room_segmentation import RoomParams, segment_rooms
    from mote_bringup.map_cleanup.rooms_cli import (
        load_map,
        make_overlay,
        merge_into_zones,
    )

    occ, geometry = load_map(rev_dir / "map.yaml")
    result = segment_rooms(occ, geometry, RoomParams())
    added, skipped = merge_into_zones(
        rev_dir,
        result.rooms,
        site=site,
        floor=floor,
        # The coordinates are in the *build's* map frame, which is no robot's.
        # zone/v0 wants the platform that holds the frame named; naming the
        # builder is the true answer and keeps a robot from being blamed for
        # poses it never drove to.
        platform_id="map-build",
    )
    overlay = out_dir / "rooms.png"
    cv2.imwrite(str(overlay), make_overlay(occ, geometry, result))
    return {
        "added": added,
        "skipped": skipped,
        "n_rooms": len(result.rooms),
        "overlay": overlay.name,
    }


def carry_forward(baseline_dir: Path | None) -> str:
    """What the previous revision's places were called — reported, not rebound.

    Re-binding a floor's names onto new geometry is task 345. Doing it badly is
    worse than not doing it: a name bound to the wrong room sends the robot to
    the wrong room, and nothing downstream can tell. So the build says what was
    lost and leaves the reviewer to rename in the editor.
    """
    if baseline_dir is None:
        return (
            "No baseline floor, so there were no names to carry forward. "
            "(Carrying a floor's vocabulary across a rebuild is task 345.)"
        )
    from mote_bringup import bundle

    try:
        previous = bundle.read_floor(baseline_dir)
    except bundle.BundleError:
        return (
            f"`{baseline_dir}` has no zone documents, so there were no names to "
            "carry forward. (Task 345.)"
        )
    names = sorted(previous.get("zones", {}))
    if not names:
        return f"`{baseline_dir}` names no places. (Task 345.)"
    return (
        f"**Not carried forward**: the baseline floor names {len(names)} place(s) — "
        + ", ".join(f"`{name}`" for name in names)
        + ". Re-binding them onto this map's rooms is task 345; until it lands "
        "the reviewer renames the placeholders above in the dashboard's zone "
        "editor, which is where a name is edited on a candidate anyway."
    )


def revision_metrics(revision_dir: Path) -> dict:
    """Truth-free map metrics for the map a revision *serves*.

    Both sides of the build's diff go through here, and that is the point. The
    replay leg carries map metrics too, but they describe the raw solve — the
    image before the declutter pass — while a stored revision only ever keeps
    the cleaned one. Scoring the candidate from the leg and the baseline from
    disk compares two different artifacts: measured on the 2026-08-02 bag, that
    reported the candidate's speckle as five times the baseline's when the two
    *served* maps agree to a thousandth, and every number it printed invited a
    reviewer to reject a good map.

    So the candidate is read back from its own pixels exactly as the baseline
    is, each at the thresholds its own ``map.yaml`` declares.
    """
    import cv2
    import numpy as np

    import metrics
    from mote_bringup import bundle

    map_yaml = revision_dir / "map.yaml"
    if not map_yaml.is_file():
        raise BuildFailed(f"{revision_dir} has no map.yaml")
    meta = bundle.read_map(map_yaml)
    image = cv2.imread(str(revision_dir / meta["image"]), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise BuildFailed(f"could not read {revision_dir / meta['image']}")
    grid = rev.png_to_grid(
        np.asarray(image),
        int(meta.get("negate", 0)),
        float(meta.get("free_thresh", rev.YAML_FREE_THRESH)),
        float(meta.get("occupied_thresh", rev.YAML_OCC_THRESH)),
    )
    return metrics.map_quality(grid, float(meta["resolution"]))


def resolve_baseline(argument: str | None) -> Path | None:
    if not argument:
        return None
    path = Path(argument).expanduser().resolve()
    if path.name == "map.yaml":
        path = path.parent
    if not path.is_dir():
        raise BuildFailed(f"baseline not found: {argument}")
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="map-build",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--bag", required=True, help="a recorded mapping bag directory")
    parser.add_argument(
        "--params",
        default=str(BUILD_PARAMS),
        help="slam_toolbox parameters (default: the committed build params)",
    )
    parser.add_argument("--site", default="local", help="site the revision is for")
    parser.add_argument("--floor", default="default", help="floor the revision is for")
    parser.add_argument(
        "--baseline",
        default="",
        help="a revision directory (or its map.yaml) to diff metrics and zone "
        "names against — normally the floor's current map",
    )
    parser.add_argument(
        "--frame",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW_DEG"),
        help="birth-align the map frame by this SE2. An operator's judgment "
        "until task 615 lands an estimator the build can gate on; it is "
        "recorded in the revision's meta either way.",
    )
    parser.add_argument(
        "--out",
        default=str(REPO / "map_build_results"),
        help="where the build lands (a UTC-stamped directory under this)",
    )
    parser.add_argument(
        "--no-clean",
        action="store_true",
        help="serve the raw solve rather than the declutter pass. For maps "
        "built from ground-truth geometry, where the pass would strip thin "
        "true walls — the same reason sim maps save with clean=False.",
    )
    parser.add_argument(
        "--no-segment", action="store_true", help="do not propose room zones"
    )
    parser.add_argument(
        "--paced",
        action="store_true",
        help="feed against the wall clock instead of in lockstep: a whole bag "
        "costs what it cost to record. For a parameter set whose gates the "
        "acceptance chain has not been validated against.",
    )
    parser.add_argument("--settle", type=float, default=8.0)
    parser.add_argument("--max-scans", type=int, default=0, help="0 = whole bag")
    parser.add_argument("--skip-secs", type=float, default=0.0)
    parser.add_argument("--stop-secs", type=float, default=0.0)
    parser.add_argument("--boot-timeout", type=float, default=120.0)
    parser.add_argument("--replay-timeout", type=float, default=7200.0)
    args = parser.parse_args(argv)

    bag = Path(args.bag).expanduser().resolve()
    params = Path(args.params).expanduser().resolve()
    if not bag.is_dir():
        sys.exit(f"bag not found: {bag}")
    if not params.is_file():
        sys.exit(f"param file not found: {params}")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path(args.out).expanduser() / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    revision_id = rev.new_revision_id()
    rev_dir = out_dir / "revision" / revision_id
    rev_dir.mkdir(parents=True)
    log(f"results -> {out_dir}")

    import replay

    build = {
        "revision": revision_id,
        "built": stamp,
        "out_dir": str(out_dir),
        "revision_dir": str(rev_dir),
        "site": args.site,
        "floor": args.floor,
        "stages": [],
        "diff": [],
        "images": [],
        # Shaped before the first step that can fail, so a build that dies
        # early still writes a report saying what it was asked to build.
        "inputs": {
            "bag": {"path": str(bag), "name": bag.name, "sha256": "", "files": []},
            "params": {"path": str(params), "sha256": ""},
            "frame": list(args.frame) if args.frame else None,
            "feed": "paced" if args.paced else "lockstep",
            "harness_commit": replay.git_commit(),
        },
    }
    stages = build["stages"]

    def stage(name, outcome, detail=""):
        stages.append({"name": name, "outcome": outcome, "detail": str(detail)})
        log(f"{name}: {outcome}{f' — {detail}' if detail else ''}")

    try:
        baseline_dir = resolve_baseline(args.baseline)

        log(f"digesting {bag.name}")
        build["inputs"]["bag"] = dict(rev.digest_bag(bag), path=str(bag))
        build["inputs"]["params"]["sha256"] = rev.digest_file(params)

        leg = solve(bag, params, out_dir, args)
        stage(
            "solve",
            "ok",
            f"{leg['n_inserted']} pose-graph nodes from {leg['metrics']['n_scans']} "
            f"scans in {leg.get('wall_s') or 0:.0f} s",
        )

        frame = assemble(leg, rev_dir)
        origin = ", ".join(f"{value:.3f}" for value in frame["origin"])
        stage(
            "assemble",
            "ok",
            f"{frame['width']}x{frame['height']} @ {frame['resolution']:.3f} m/px, "
            f"origin ({origin})"
            + ("" if frame["origin_yaw_recorded"] else " (origin yaw assumed 0)"),
        )

        clean = declutter(rev_dir, not args.no_clean)
        if clean.get("skipped"):
            stage("declutter", "skipped", "--no-clean: serving the raw solve")
        elif clean.get("ok"):
            stage(
                "declutter",
                "ok",
                f"-{clean['removed']} cells, +{clean['added']}, wall directions "
                f"{clean['directions_deg']}",
            )
        else:
            stage("declutter", "failed", f"{clean.get('error')}; serving raw")

        if args.no_segment:
            zones = {"added": [], "skipped": [], "n_rooms": 0}
            stage("segment", "skipped", "--no-segment")
        else:
            zones = segment(rev_dir, args.site, args.floor, out_dir)
            stage("segment", "ok", f"{len(zones['added'])} room zone(s) proposed")
        zones["carry_forward"] = carry_forward(baseline_dir)
        stage("carry forward", "stub", "task 345 — names are reported, not rebound")
        build["zones"] = zones

        meta = {
            "schema": 1,
            "saved": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S"),
            "built_by": "map-build",
            "site": args.site,
            "floor": args.floor,
            "bag": build["inputs"]["bag"]["name"],
            "bag_sha256": build["inputs"]["bag"]["sha256"],
            "slam_params": params.name,
            "slam_params_sha256": build["inputs"]["params"]["sha256"],
            "frame": build["inputs"]["frame"],
            "feed": build["inputs"]["feed"],
            "harness_commit": build["inputs"]["harness_commit"],
            "clean": clean,
        }
        rev.write_meta(rev_dir, meta)

        from mote_bringup import bundle

        report = bundle.validate(rev_dir)
        build["validation"] = {
            "summary": report.summary(),
            "errors": list(report.errors),
            "warnings": list(report.warnings),
            "occupancy": report.occupancy,
        }
        if not report.ok:
            stage("validate", "FAILED", report.summary())
            raise BuildFailed(f"the revision is not usable: {report.summary()}")
        stage("validate", "ok", report.summary())

        # The leg's own map metrics describe the raw solve; the served map is
        # what a promotion publishes, so that is what is scored and diffed.
        # ``loop`` comes from the leg either way — it is a property of the
        # trajectory, and no image holds it.
        candidate_metrics = dict(leg["metrics"], map=revision_metrics(rev_dir))
        candidate_metrics["map_raw"] = leg["metrics"].get("map", {})
        base_metrics = None
        if baseline_dir is not None:
            base_metrics = {"map": revision_metrics(baseline_dir)}
            build["baseline"] = {"path": str(baseline_dir), "metrics": base_metrics}
        build["metrics"] = candidate_metrics
        build["angular"] = candidate_metrics["map"]
        build["diff"] = build_report.compare(candidate_metrics, base_metrics)
        worse = [
            row["metric"] for row in build["diff"] if row.get("verdict") == "worse"
        ]
        stage(
            "score",
            "ok",
            f"{len(worse)} metric(s) worse than the baseline: {', '.join(worse)}"
            if worse
            else "no metric regressed against the baseline"
            if base_metrics
            else "no baseline to diff against",
        )

        blob = bundle.pack(rev_dir)
        bundle_path = out_dir / f"{revision_id}.tar.gz"
        bundle_path.write_bytes(blob)
        stage(
            "package",
            "ok",
            f"{bundle_path.name}, {len(blob)} bytes, {bundle.digest(blob)[:23]}…",
        )

        for name, caption in (
            ("map.png", "Built map (served)"),
            ("map_raw.png", "Raw solve"),
            ("diagnostics.png", "Declutter diagnostics"),
        ):
            source = rev_dir / name
            if source.is_file():
                shutil.copyfile(source, out_dir / name)
                build["images"].append((caption, name))
        if zones.get("overlay"):
            build["images"].append(("Proposed rooms", zones["overlay"]))

        build["verdict"] = "candidate emitted"
        build["verdict_detail"] = (
            f"`{rev_dir}` — validated, packed as `{bundle_path.name}`"
        )
        build["next"] = (
            f"Review the map above, then upload `{bundle_path.name}` to the "
            f"registry as a candidate for `{args.site}/{args.floor}`. The upload "
            "route accepts enrolled robots only, so a builder needs a credential "
            "of its own — that is task 344; until it lands, a robot at the site "
            "can side-load the revision directory into its floor and "
            "`pixi run publish-map --revision "
            f"{revision_id}`. Promotion is unchanged: an operator's audited "
            "call, in the dashboard or `fleetctl promote`."
        )
        status = 0
    except BuildFailed as failure:
        build.setdefault(
            "validation", {"summary": "not reached", "errors": [], "warnings": []}
        )
        build["verdict"] = "build failed"
        build["verdict_detail"] = str(failure)
        build["next"] = "Nothing was emitted. Fix the above and re-run."
        log(f"FAILED: {failure}")
        status = 1

    path = build_report.write(out_dir, build)
    log(f"wrote {path}")
    if status == 0:
        log(f"candidate {revision_id} -> {rev_dir}")
    return status


if __name__ == "__main__":
    sys.exit(main())

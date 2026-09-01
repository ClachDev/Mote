"""Site bundles: everything that is only meaningful relative to one mapped
place, managed as a single versioned unit.

    ~/.mote/active.yaml            -> {site: home, floor: ground}   (robot state)
    ~/.mote/sites/<site>/
        site.yaml                  -> {schema: 1, name, default_floor}
        floors/<floor>/
            vocabulary.yaml        what the places here are CALLED (zone/v0).
                                   No coordinates: safe to share with every
                                   robot at the site.
            binding.yaml           where THIS robot believes they are, in this
                                   floor's map frame. Never shared; travels
                                   inside a map revision, since a coordinate
                                   means nothing without the map beside it.
            zones.yaml             the combined file both used to be. Still
                                   read; migrated to the pair on first write.
            map -> maps/<rev>/     symlink to the current map revision
            maps/<rev>/            immutable once published:
                map.yaml + map.png     nav2 map_server pair — the cleaned map,
                                       the one that is served and distributed
                map_raw.png            the untouched map_saver output, kept for
                                       provenance/audit (same frame as map.png)
                diagnostics.png        before/after + detected structure panel
                map.posegraph + .data  slam_toolbox graph (lets mapping
                                       continue later in the same frame; belongs
                                       to the raw map, never the cleaned one)
                meta.yaml              provenance: when it was saved, which
                                       mapping bag (~/.mote/bags/mapping/<ts>,
                                       recorded by mapping_launch.py) the
                                       session was captured in, and the cleaning
                                       pass parameters + stats

A map revision is fully staged in its maps/<rev>/ directory before the
``map`` symlink is flipped to it (one atomic rename), so a half-written
save, crash, or interrupted transfer is never visible to readers and
rolling back is flipping to an older revision (``site use-map``). The
newest revisions are kept, older ones pruned.

Zone poses are coordinates in a map frame whose origin is an accident of
where SLAM started, so zones/map/posegraph must live and travel together.
A floor is one SLAM session (one frame); a site groups floors that share a
location. The whole bundle is plain files + YAML so it can be zipped,
synced, or served by a web API without translation.

Site bundles are per-robot state, so they live under ``MOTE_HOME`` (``~/.mote``
by default) with the rest of it — see :mod:`mote_bringup.mote_home`.

A revision's *contents* — what files it must hold, whether the map inside is
usable, and how it packs for a wire — are :mod:`mote_bringup.bundle`, which is
stdlib-only so the fleet server can validate an uploaded revision with the same
code that wrote it (fleet.md Q4). This module keeps the layout; that one keeps
the content.

Console script ``site`` (pixi tasks: site, save-map):
    site create <name> [--floor ground]   new site (+ becomes active if none)
    site add-floor <name>                 add a floor to the active site
    site use <site> [floor]               select the active site/floor
    site list | site info                 inspect
    site save-map                         save map + posegraph from a running
                                          mapping session into the active floor
    site use-map <rev>                    roll the active floor to a revision
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

from mote_bringup import bundle
from mote_bringup.mote_home import mote_dir

SCHEMA = 1
KEEP_REVISIONS = 3


def sites_dir() -> Path:
    return mote_dir() / "sites"


def site_dir(site: str) -> Path:
    return sites_dir() / site


def bags_dir(kind: str) -> Path:
    return mote_dir() / "bags" / kind


def floor_dir(site: str, floor: str) -> Path:
    return site_dir(site) / "floors" / floor


def floors(site: str) -> list[str]:
    root = site_dir(site) / "floors"
    return sorted(p.name for p in root.iterdir() if p.is_dir()) if root.is_dir() else []


def active() -> tuple[str, str] | None:
    """The active (site, floor), or None if unset or dangling."""
    path = mote_dir() / "active.yaml"
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text()) or {}
    site, floor = data.get("site"), data.get("floor")
    if not site or not floor or not floor_dir(site, floor).is_dir():
        return None
    return site, floor


def set_active(site: str, floor: str):
    mote_dir().mkdir(parents=True, exist_ok=True)
    (mote_dir() / "active.yaml").write_text(
        yaml.safe_dump({"site": site, "floor": floor}, sort_keys=False)
    )


def resolve_map() -> str:
    """The active floor's current map yaml, or ''.

    Safe to call at launch time with no sites configured.
    """
    act = active()
    if act:
        candidate = floor_dir(*act) / "map" / "map.yaml"
        if candidate.exists():
            return str(candidate)
    return ""


def resolve_zones() -> str:
    """The active floor's zones, or ''.

    A *directory* now, not a file: the floor's zones are two documents, and
    which of them a reader wants is the reader's business. What comes back is
    what ``bundle.read_floor`` takes, so a legacy combined file still works —
    it is inside the same directory.
    """
    act = active()
    if act:
        fdir = floor_dir(*act)
        if has_zones(fdir):
            return str(fdir)
    return ""


def has_zones(fdir: Path) -> bool:
    return (fdir / bundle.VOCABULARY_YAML).exists() or (
        fdir / bundle.ZONES_YAML
    ).exists()


def zones_for_write() -> Path:
    """The floor newly taught zones should be written into."""
    act = active()
    if not act:
        sys.exit("no active site (run: site create <name>)")
    return floor_dir(*act)


def _seed_floor(site: str, floor: str):
    """The directory, and nothing in it.

    A new floor deliberately has **no** zone documents. Seeding empty ones
    would make a hand-written ``zones.yaml`` dropped in beside them ambiguous —
    two layouts on one floor, with a precedence rule to remember — and the
    empty pair would win silently. A floor with no zones is a floor with no
    zone files; the first ``save-zone`` writes the pair.
    """
    floor_dir(site, floor).mkdir(parents=True, exist_ok=True)


def create(site: str, floor: str = "ground"):
    if site_dir(site).exists():
        sys.exit(f"site '{site}' already exists")
    _seed_floor(site, floor)
    (site_dir(site) / "site.yaml").write_text(
        yaml.safe_dump(
            {"schema": SCHEMA, "name": site, "default_floor": floor},
            sort_keys=False,
        )
    )
    if active() is None:
        set_active(site, floor)
    print(f"created site '{site}' with floor '{floor}' at {site_dir(site)}")
    print(f"active: {_active_str()}")


def add_floor(floor: str):
    act = active()
    if not act:
        sys.exit("no active site (run: site create <name> / site use <site>)")
    site = act[0]
    if floor in floors(site):
        sys.exit(f"floor '{floor}' already exists in site '{site}'")
    _seed_floor(site, floor)
    set_active(site, floor)
    print(f"added floor '{floor}' to site '{site}' (now active)")


def use(site: str, floor: str | None = None):
    if not site_dir(site).is_dir():
        sys.exit(f"no such site '{site}' (have: {', '.join(list_sites()) or 'none'})")
    if floor is None:
        meta = yaml.safe_load((site_dir(site) / "site.yaml").read_text()) or {}
        floor = meta.get("default_floor") or (floors(site) or [None])[0]
    if not floor or floor not in floors(site):
        sys.exit(
            f"no such floor '{floor}' in '{site}' (have: {', '.join(floors(site))})"
        )
    set_active(site, floor)
    print(f"active: {site}/{floor}")


def list_sites() -> list[str]:
    root = sites_dir()
    return sorted(p.name for p in root.iterdir() if p.is_dir()) if root.is_dir() else []


def _active_str() -> str:
    act = active()
    return f"{act[0]}/{act[1]}" if act else "none"


def cmd_list():
    act = active()
    for site in list_sites():
        for floor in floors(site):
            marker = " *" if act == (site, floor) else ""
            print(f"{site}/{floor}{marker}")
    if not list_sites():
        print("no sites (run: site create <name>)")


def cmd_info():
    act = active()
    if not act:
        print("active: none")
        return
    fdir = floor_dir(*act)
    print(f"active: {act[0]}/{act[1]}  ({fdir})")
    if has_zones(fdir):
        try:
            zones = bundle.read_floor(fdir, *act)["zones"]
        except bundle.BundleError as exc:
            print(f"  zones        UNREADABLE ({exc})")
        else:
            with_fp = sum(1 for z in zones.values() if "radius" in z or "polygon" in z)
            unbound = sum(1 for z in zones.values() if not z.get("bound"))
            notes = [f"{len(zones)} zones"]
            if with_fp:
                notes.append(f"{with_fp} with a footprint")
            # A name this robot has never been taught is worth saying: it is
            # the difference between a floor it can work on and one it has only
            # been told about.
            if unbound:
                notes.append(f"{unbound} not taught here")
            legacy = " (combined zones.yaml — migrates on next write)"
            split = (fdir / bundle.VOCABULARY_YAML).exists()
            print(f"  zones        ok ({', '.join(notes)}){'' if split else legacy}")
    else:
        print("  zones        missing")
    current = current_revision(fdir)
    if not current:
        print("  map          none (run: pixi run save-map during mapping)")
        return
    for rev in revisions(fdir):
        marker = " *" if rev == current else ""
        meta = revision_meta(fdir, rev)
        clean = meta.get("clean", {})
        if not clean:
            clean_note = "raw only"
        elif clean.get("skipped"):
            clean_note = "raw (clean skipped)"
        elif clean.get("ok"):
            clean_note = f"cleaned -{clean.get('removed', '?')}"
        else:
            clean_note = "clean FAILED, serving raw"
        bag = meta.get("bag")
        bag_note = f", bag: {bag}" if bag else ""
        print(f"  maps/{rev}{marker}  ({clean_note}{bag_note})")


def revisions(fdir: Path) -> list[str]:
    root = fdir / "maps"
    return sorted(p.name for p in root.iterdir() if p.is_dir()) if root.is_dir() else []


def current_revision(fdir: Path) -> str | None:
    link = fdir / "map"
    return Path(os.readlink(link)).name if link.is_symlink() else None


def _publish_revision(fdir: Path, rev: str):
    """Atomically point the floor's ``map`` symlink at maps/<rev>."""
    tmp = fdir / f".map-{os.getpid()}"
    os.symlink(os.path.join("maps", rev), tmp)
    os.replace(tmp, fdir / "map")


def _prune_revisions(fdir: Path):
    keep = set(revisions(fdir)[-KEEP_REVISIONS:])
    keep.add(current_revision(fdir))
    for rev in revisions(fdir):
        if rev not in keep:
            shutil.rmtree(fdir / "maps" / rev)


def _new_revision_dir(fdir: Path) -> Path:
    rev_dir = fdir / "maps" / time.strftime("%Y%m%dT%H%M%S")
    rev_dir.mkdir(parents=True, exist_ok=True)
    return rev_dir


def latest_mapping_bag(max_age_s: float = 900.0) -> Path | None:
    """The mapping bag the current session is writing, or None.

    The newest bags/mapping/<ts> directory counts only if it has been written
    recently — a stale directory belongs to some earlier session, not the
    mapping run being saved.
    """
    root = bags_dir("mapping")
    if not root.is_dir():
        return None
    dirs = sorted(p for p in root.iterdir() if p.is_dir())
    if not dirs:
        return None
    newest = dirs[-1]
    mtimes = [f.stat().st_mtime for f in newest.iterdir()]
    if time.time() - max(mtimes, default=newest.stat().st_mtime) > max_age_s:
        return None
    return newest


def revision_meta(fdir: Path, rev: str) -> dict:
    meta_file = fdir / "maps" / rev / "meta.yaml"
    if not meta_file.exists():
        return {}
    return yaml.safe_load(meta_file.read_text()) or {}


def _clean_map_png(raw_png: Path, out_png: Path, diag_png: Path) -> dict:
    """Declutter a saved occupancy PNG: read raw_png, write the cleaned map to
    out_png and a diagnostics panel to diag_png. Returns cleaning stats for
    meta.yaml. Kept file-only (no ROS) so it is testable off the robot.
    """
    import cv2

    from mote_bringup.map_cleanup import Params, extract_structure
    from mote_bringup.map_cleanup.cli import make_diagnostics

    occ = cv2.imread(str(raw_png), cv2.IMREAD_GRAYSCALE)
    if occ is None:
        raise RuntimeError(f"could not read {raw_png}")
    params = Params()
    res = extract_structure(occ, params)
    cv2.imwrite(str(out_png), res.cleaned_map)
    cv2.imwrite(str(diag_png), make_diagnostics(occ, res))
    before, after = int(res.wall.sum()), int(res.clean_wall.sum())
    return {
        "ok": True,
        "wedge_halfwidth_deg": params.wedge_halfwidth_deg,
        "peak_rel_threshold": params.peak_rel_threshold,
        "directions_deg": [round(d, 1) for d in res.directions_deg],
        "occupied_before": before,
        "occupied_after": after,
        "removed": before - after,
        "added": int((res.clean_wall & ~res.wall).sum()),
    }


def _promote_cleaned(rev_dir: Path) -> dict:
    """Turn a freshly-saved raw revision into a served, cleaned one.

    The untouched map_saver output (map.png) is kept as map_raw.png and the
    decluttered image is promoted to the served map.png. map.yaml's frame is
    identical for both (only pixels change), so zones and localization are
    unaffected. A cleaning failure never discards the map — the raw is served
    instead. Returns the clean stats block for meta.yaml.
    """
    raw_png = rev_dir / "map_raw.png"
    (rev_dir / "map.png").rename(raw_png)
    (rev_dir / "map_raw.yaml").write_text(
        (rev_dir / "map.yaml").read_text().replace("map.png", "map_raw.png")
    )
    try:
        return _clean_map_png(raw_png, rev_dir / "map.png", rev_dir / "diagnostics.png")
    except Exception as exc:  # noqa: BLE001 — a bad clean must not lose the map
        shutil.copyfile(raw_png, rev_dir / "map.png")
        print(f"WARNING: map cleaning failed ({exc}); serving raw map", file=sys.stderr)
        return {"ok": False, "error": str(exc)}


def save_map(clean: bool = True):
    """Save the running mapping session into a new revision of the active floor.

    ``clean`` runs the FFT declutter pass and serves the cleaned map (the robot
    default). Pass ``clean=False`` for already-clean maps — e.g. sim maps built
    from ground-truth geometry, where the declutter pass, tuned for real-sensor
    noise, would strip the thin true walls — to serve the raw map_saver output.
    """
    act = active()
    if not act:
        sys.exit("no active site (run: site create <name>)")
    fdir = floor_dir(*act)
    rev_dir = _new_revision_dir(fdir)
    stem = rev_dir / "map"
    try:
        saver = subprocess.run(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                str(stem),
                "--fmt",
                "png",
            ],
            timeout=60,
        )
        subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/slam_toolbox/serialize_map",
                "slam_toolbox/srv/SerializePoseGraph",
                f"{{filename: '{stem}'}}",
            ],
            timeout=30,
        )
    except subprocess.TimeoutExpired:
        shutil.rmtree(rev_dir)
        sys.exit("timed out talking to the map/slam services — is mapping running?")
    missing = [
        s
        for s in (".yaml", ".png", ".posegraph", ".data")
        if not stem.with_suffix(s).exists()
    ]
    if saver.returncode != 0 or missing:
        shutil.rmtree(rev_dir)
        sys.exit(
            f"incomplete map revision (missing map{'/map'.join(missing)}) — "
            "discarded; are mapping + slam_toolbox running?"
        )
    clean_stats = _promote_cleaned(rev_dir) if clean else {"skipped": True}

    meta = {"schema": SCHEMA, "saved": time.strftime("%Y-%m-%dT%H:%M:%S")}
    bag = latest_mapping_bag()
    if bag:
        meta["bag"] = str(bag.relative_to(mote_dir()))
    meta["clean"] = clean_stats
    (rev_dir / "meta.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))

    # The same check the fleet server will run on this revision if it is ever
    # published (mote_bringup.bundle), so a map that would be refused there is
    # refused here, on the robot, while the mapping session is still up.
    report = bundle.validate(rev_dir)
    if not report.ok:
        shutil.rmtree(rev_dir)
        sys.exit(f"unusable map revision, discarded: {report.summary()}")
    for warning in report.warnings:
        print(f"note: {warning}", file=sys.stderr)

    _publish_revision(fdir, rev_dir.name)
    _prune_revisions(fdir)
    if clean_stats.get("skipped"):
        served = "raw (clean skipped)"
    elif clean_stats.get("ok"):
        served = "cleaned"
    else:
        served = "raw (cleaning failed)"
    stats = (
        f", declutter -{clean_stats['removed']} cells" if clean_stats.get("ok") else ""
    )
    print(
        f"saved map + posegraph revision {rev_dir.name}  ({_active_str()}); "
        f"serving {served} map{stats}"
    )


def install_revision(site: str, floor: str, revision: str, blob: bytes) -> str:
    """Install a packed revision from the fleet registry and publish it locally.

    The distribution half of the design (fleet.md Q4): stage the whole revision
    in a temporary directory, check it, rename it into ``maps/<rev>/``, then
    flip the ``map`` symlink — so a half-transferred revision is never visible
    and nothing has to be undone if the transfer dies. Returns what it did:
    ``current`` (nothing to do), ``flipped`` (already had it) or ``installed``.

    **Coordinates travel with the map; names do not.** A revision from a
    different mapping session is a different map frame, so the poses taught in
    the old one are wrong the instant the new map is published — the bundle's
    ``binding.yaml`` therefore replaces the floor's, and the one it replaces is
    kept beside it as ``binding.<old-rev>.yaml``, because losing a map is
    recoverable and losing every taught place silently is not. The
    ``vocabulary.yaml`` is left alone: the rooms did not change their names
    when the robot re-mapped them, which is the practical dividend of the
    zone/v0 split.
    """
    fdir = floor_dir(site, floor)
    if not fdir.is_dir():
        _seed_floor(site, floor)
        if not (site_dir(site) / "site.yaml").exists():
            (site_dir(site) / "site.yaml").write_text(
                yaml.safe_dump(
                    {"schema": SCHEMA, "name": site, "default_floor": floor},
                    sort_keys=False,
                )
            )
    rev_dir = fdir / "maps" / revision
    if rev_dir.is_dir():
        if current_revision(fdir) == revision:
            return "current"
        _publish_revision(fdir, revision)
        _adopt_zones(fdir, rev_dir, revision)
        return "flipped"

    (fdir / "maps").mkdir(parents=True, exist_ok=True)
    staging = fdir / "maps" / f".staging-{os.getpid()}"
    shutil.rmtree(staging, ignore_errors=True)
    try:
        bundle.unpack(blob, staging)
        report = bundle.validate(staging, require_posegraph=False)
        if not report.ok:
            raise bundle.BundleError(
                f"the fleet served an unusable revision {revision}: {report.summary()}"
            )
        os.replace(staging, rev_dir)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    previous = current_revision(fdir)
    _publish_revision(fdir, revision)
    _adopt_zones(fdir, rev_dir, previous)
    _prune_revisions(fdir)
    return "installed"


def _adopt_zones(fdir: Path, rev_dir: Path, previous: str | None):
    """Install a revision's **binding** as the floor's.

    Only the binding: it is the half that is bound to this revision's map
    frame, and it is the half a different SLAM session makes wrong. The
    vocabulary stays where it is, because the names of the rooms did not change
    when the robot re-mapped the floor — which is the practical dividend of the
    split, and the reason re-mapping no longer costs an operator the names
    they typed.
    """
    for name in (bundle.BINDING_YAML, bundle.ZONES_YAML):
        source = rev_dir / name
        if not source.is_file():
            continue
        target = fdir / name
        if target.is_file():
            if target.read_bytes() == source.read_bytes():
                return
            target.rename(fdir / f"{Path(name).stem}.{previous or 'previous'}.yaml")
        shutil.copyfile(source, target)
        return


def use_map(rev: str):
    act = active()
    if not act:
        sys.exit("no active site")
    fdir = floor_dir(*act)
    if rev not in revisions(fdir):
        sys.exit(f"no such revision '{rev}' (have: {', '.join(revisions(fdir))})")
    _publish_revision(fdir, rev)
    print(f"map -> maps/{rev}  ({_active_str()}); restart nav to load it")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="site", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p_create = sub.add_parser("create", help="create a new site")
    p_create.add_argument("name")
    p_create.add_argument("--floor", default="ground")
    p_add = sub.add_parser("add-floor", help="add a floor to the active site")
    p_add.add_argument("name")
    p_use = sub.add_parser("use", help="select the active site/floor")
    p_use.add_argument("site")
    p_use.add_argument("floor", nargs="?")
    sub.add_parser("list", help="list sites and floors")
    sub.add_parser("info", help="show the active floor's artifacts")
    sub.add_parser("save-map", help="save map + posegraph into the active floor")
    p_use_map = sub.add_parser(
        "use-map", help="roll the active floor to a map revision"
    )
    p_use_map.add_argument("revision")

    args = parser.parse_args(argv)
    {
        "create": lambda: create(args.name, args.floor),
        "add-floor": lambda: add_floor(args.name),
        "use": lambda: use(args.site, args.floor),
        "list": cmd_list,
        "info": cmd_info,
        "save-map": save_map,
        "use-map": lambda: use_map(args.revision),
    }[args.command]()


if __name__ == "__main__":
    main()

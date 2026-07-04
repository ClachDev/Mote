"""Site bundles: everything that is only meaningful relative to one mapped
place, managed as a single versioned unit.

    ~/.mote/active.yaml            -> {site: home, floor: ground}   (robot state)
    ~/.mote/sites/<site>/
        site.yaml                  -> {schema: 1, name, default_floor}
        floors/<floor>/
            zones.yaml             named poses in this floor's map frame
            map -> maps/<rev>/     symlink to the current map revision
            maps/<rev>/            immutable once published:
                map.yaml + map.png     nav2 map_server pair
                map.posegraph + .data  slam_toolbox graph (lets mapping
                                       continue later in the same frame)

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

``MOTE_HOME`` overrides ``~/.mote`` (used by tests).

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

SCHEMA = 1
KEEP_REVISIONS = 3


def mote_dir() -> Path:
    return Path(os.environ.get("MOTE_HOME", "~/.mote")).expanduser()


def sites_dir() -> Path:
    return mote_dir() / "sites"


def site_dir(site: str) -> Path:
    return sites_dir() / site


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
    """The active floor's zones yaml, or ''."""
    act = active()
    if act:
        candidate = floor_dir(*act) / "zones.yaml"
        if candidate.exists():
            return str(candidate)
    return ""


def zones_for_write() -> Path:
    """Where newly taught zones should be written."""
    act = active()
    if not act:
        sys.exit("no active site (run: site create <name>)")
    return floor_dir(*act) / "zones.yaml"


def _seed_floor(site: str, floor: str):
    fdir = floor_dir(site, floor)
    fdir.mkdir(parents=True, exist_ok=True)
    zones = fdir / "zones.yaml"
    if not zones.exists():
        zones.write_text("frame_id: map\nzones: {}\n")


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
    zones_state = "ok" if (fdir / "zones.yaml").exists() else "missing"
    print(f"  zones.yaml   {zones_state}")
    current = current_revision(fdir)
    if not current:
        print("  map          none (run: pixi run save-map during mapping)")
        return
    for rev in revisions(fdir):
        marker = " *" if rev == current else ""
        print(f"  maps/{rev}{marker}")


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


def save_map():
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
    _publish_revision(fdir, rev_dir.name)
    _prune_revisions(fdir)
    print(f"saved map + posegraph revision {rev_dir.name}  ({_active_str()})")


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

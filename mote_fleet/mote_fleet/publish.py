"""``publish-map`` — offer this robot's saved map to the fleet registry.

The other half of ``pixi run save-map``: that writes a map revision into this
robot's site bundle, this hands it to the fleet server, which validates it and
keeps it as a **candidate**. Publishing changes nothing for anybody until an
operator promotes it (``fleetctl promote``), which is what makes it safe to run
after every mapping session and what keeps two robots mapping one floor from
overwriting each other (fleet.md Q4).

Separate from ``save-map`` on purpose. Saving is a local, offline act that must
work on a robot that has never seen a fleet server; publishing needs a network
and an enrollment. Chaining them would make the first fail when the second
cannot happen.

    pixi run save-map                     # save locally, as always
    pixi run publish-map                  # offer it to the fleet
    pixi run publish-map -- --revision 20260727T101500
"""

import argparse
import sys

from mote_bringup import identity, sites

from mote_fleet import fleet_config, mapsync


def main(argv=None):
    parser = argparse.ArgumentParser(prog="publish-map", description=__doc__)
    parser.add_argument(
        "--revision",
        default="",
        help="which revision to publish (default: the floor's current one)",
    )
    parser.add_argument("--site", default="", help="default: the active site")
    parser.add_argument("--floor", default="", help="default: the active floor")
    parser.add_argument(
        "--server",
        default="",
        help="fleet API base URL (default: the one enrollment recorded)",
    )
    parser.add_argument(
        "--robot-id",
        default="",
        dest="robot_id",
        help="default: this robot's enrolled id",
    )
    args = parser.parse_args(argv)

    site, floor = args.site, args.floor
    if not (site and floor):
        active = sites.active()
        if not active:
            sys.exit("no active site (run: pixi run site use <site> [floor])")
        site, floor = site or active[0], floor or active[1]

    fdir = sites.floor_dir(site, floor)
    revision = args.revision or sites.current_revision(fdir)
    if not revision:
        sys.exit(
            f"{site}/{floor} has no saved map to publish "
            "(run: pixi run save-map during a mapping session)"
        )

    config = fleet_config.load() or {}
    server = args.server or config.get("server") or ""
    if not server:
        sys.exit(
            f"no fleet server configured at {fleet_config.config_path()} — "
            "enrol this robot, or pass --server http://fleet-box:8080"
        )
    robot_id = args.robot_id or identity.robot_id()
    if not robot_id:
        sys.exit(f"no identity at {identity.identity_path()} — enrol this robot first")

    try:
        answer = mapsync.publish(server, site, floor, revision, robot_id)
    except mapsync.SyncError as exc:
        sys.exit(f"publish failed: {exc}")

    stored = answer.get("revision", revision)
    print(
        f"published {site}/{floor}/{stored} to {server} "
        f"({answer.get('bytes', 0)} bytes)"
    )
    if stored != revision:
        print(
            f"  (stored as {stored}: {revision} was already taken on this floor "
            "by another robot)"
        )
    for warning in answer.get("warnings") or []:
        print(f"  note: {warning}")
    canonical = answer.get("canonical")
    print(
        f"  it is a candidate; {site}/{floor} is still on "
        f"{canonical or 'no published map'}.\n"
        f"  an operator promotes it with: fleetctl promote {site} {floor} {stored}"
    )


if __name__ == "__main__":
    main()

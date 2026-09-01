# Sites, maps & zones

Everything that is only meaningful relative to one mapped place — the Nav2 map
pair, the `slam_toolbox` posegraph, and named zones — lives together as a
**site bundle**. A zone's pose is a coordinate in a map frame whose origin is
an accident of where SLAM happened to start, so those three artefacts must live
and travel as one unit or they quietly stop describing the same building.

- A **floor** is one SLAM session, i.e. one map frame.
- A **site** groups floors that share a location.
- A **revision** is one immutable set of map artefacts for a floor.

The layout is managed by `mote_bringup/sites.py` (`pixi run site`); what a
revision must *contain* is `mote_bringup/bundle.py`, which is ROS-free and
shared with the fleet server so both ends validate a revision with the same
code.

## The layout

```text
~/.mote/active.yaml              -> {site: home, floor: ground}
~/.mote/sites/<site>/
    site.yaml                    -> {schema: 1, name, default_floor}
    floors/<floor>/
        zones.yaml               named poses in this floor's map frame
        map -> maps/<rev>/       symlink to the current revision
        maps/<rev>/              immutable once published:
            map.yaml + map.png       the nav2 map_server pair — the cleaned
                                     map, the one served and distributed
            map_raw.png              the untouched map_saver output, kept for
                                     provenance (same frame as map.png)
            diagnostics.png          before/after + detected structure
            map.posegraph + .data    the slam_toolbox graph, so mapping can
                                     continue later in the same frame
            meta.yaml                provenance: when, which mapping bag, and
                                     the cleaning pass parameters and stats
```

Site bundles are per-robot state, so they live under `MOTE_HOME` (`~/.mote` by
default) with the rest of it — never in the package, which is shared config an
update may replace. Setting `MOTE_HOME` moves the lot, which is how tests and
[the sim](../simulation.md#sim-maps-are-real-site-bundles) get their own.

The whole bundle is plain files and YAML, so it can be zipped, synced, or
served by a web API without translation. That is exactly what the
[map registry](../fleet/README.md#11-maps-publishing-what-a-robot-mapped-promoting-what-the-fleet-uses)
does.

## Revisions, and why the symlink

A revision is staged completely inside `maps/<rev>/` before the `map` symlink
is flipped to it — one atomic rename. A half-written save, a crash, or an
interrupted transfer is therefore never visible to a reader, and a rollback is
just flipping to an older revision:

```bash
pixi run site list          # every site and floor
pixi run site info          # the active floor: revisions, zones, footprints
pixi run site use-map <rev> # roll back
```

The newest revisions are kept and older ones pruned.

## Saving a map

```bash
pixi run mapping   # bringup + SLAM
pixi run save-map  # into the active site's floor
```

`save-map` stores the posegraph alongside the map, which is what lets a later
session *continue* in the same frame. Extend a map; do not remap it — remapping
starts a new frame and every zone bound in the old one becomes wrong.

Mapping runs record a rosbag by default (`mapping_launch.py record:=true`; the
sim passes false) and `save-map` stamps that session's bag into the revision's
`meta.yaml`, so `site info` can say which drive produced which map.

`save-map` also runs the same validation the fleet server runs on an upload, so
a map the server would refuse is refused while the mapping session is still up
and can be extended.

### The cleaning pass

`save-map` automatically runs an FFT structure-extraction pass over the map. It
keeps the untouched `map_saver` output as `map_raw.png` and promotes the
decluttered image to the served `map.png`, alongside a `diagnostics.png`. So
navigation always consumes the cleaned map while the raw one is retained for
provenance. Both share a frame, so localisation and zones are unaffected, and a
cleaning failure falls back to serving the raw map. The posegraph belongs to
the *raw* map — mapping continuation extends from raw, never from the cleaned
image. Details: [map cleaning](map-cleanup.md).

## Zones

A **zone** is the one named-place concept: a pose in the floor's map frame
that the robot can navigate to. `goto <zone>` drives to any of them, and
`fetch` uses them as its pickup and drop waypoints. A zone may *optionally*
carry an area **footprint** — a circle or a polygon — which turns a bare
waypoint into something that also answers "am I inside it?". That is optional
metadata on one concept, not a second kind of thing: one YAML section, one
loader.

Geometry reaches a floor three ways: `save-zone` on the robot, `segment-map`
over a saved map, and the dashboard's zone editor. Only the first needs a robot,
and it is the only one that measures an approach heading — so a zone a mission
has to arrive at facing something is still worth driving to.

Drive there and capture the pose:

```bash
pixi run save-zone "the kitchen" --radius 1.5
```

Re-teaching a zone's pose keeps whatever footprint it already had; passing
`--radius` is the deliberate way to replace one.

Rooms do not have to be bound one at a time, either:

```bash
pixi run segment-map          # propose one polygon zone per room
pixi run segment-map --write  # merge the proposal into zones.yaml to rename
```

`segment-map` carves a saved map's free space into rooms on one physical
assumption — a doorway is narrow. It is additive over zones already bound (a
candidate covering an already-footprinted zone is dropped), so re-running is a
no-op, and it writes beside `zones.yaml`, never into the immutable map
revision. A proposed room is anchored `derived`, not `taught`: an algorithm read
it off a map, which is what tells an operator later that a re-map invalidates
it. Two consequences worth knowing: a corridor network is not proposed at
all, and the geometry is Manhattan after rotation. See
[map cleaning & room segmentation](map-cleanup.md) and the
[validation run](../tuning/2026-07-27-room-segmentation.md).

The shape of the file, circles versus polygons, and how membership is answered
are covered in [Missions](missions.md#zones-and-go-to-the-kitchen).

### Names travel, coordinates do not

**A zone is a place-name**: a human name bound to geometry. Beside its
coordinates it carries a **vocabulary** — the `name` it is called, a free-text
`note` for what the name cannot say ("stationery lives here, not in the
office"), and `navigable`. `save-zone` and the dashboard's zone editor both
write the first two; a zone that says nothing but its name is a place a robot
may drive to.

That split is the whole reason the vocabulary exists separately. `(2.0, 3.5)`
is a different physical point for the robot standing beside this one, and no
fleet-level transform fixes that; the *name* is true for both. So the fleet
publishes the vocabulary and not the binding, over
[`GET /v1/zones`](../fleet/fleet-api.md) — expressed by the route rather than
by a rule someone has to remember: everything under `/v1/maps` is bound to a
basemap, everything under `/v1/zones` is bound to nothing.

Locally, `load_zones` **refuses** a vocabulary in which two zones answer one
query, because loading it would resolve `goto` by dictionary order — silently,
once per boot, and differently after an edit.

## Publishing a map to the fleet

Saving and publishing are deliberately separate: saving has to work on a robot
that has never seen a fleet server.

```bash
pixi run publish-map   # offer the saved revision as a *candidate*
```

An upload changes nothing. The revision sits inert until an operator promotes
it, at which point the floor's canonical `map` flips and a retained MQTT
message hands it to every agent — including one that was switched off for the
whole mapping session. That inertness is also the conflict answer: two robots
that map one floor leave two candidates, never a merge.

A pulled revision replaces the floor's `zones.yaml` (keeping the old one as
`zones.<old-rev>.yaml`), because a different session's map makes the floor's
existing bindings wrong, and takes effect on the **next bringup**, since
`map_server` reads its map at startup.

The operator flow is in the
[fleet runbook](../fleet/README.md#11-maps-publishing-what-a-robot-mapped-promoting-what-the-fleet-uses);
the routes are in the [fleet API contract](../fleet/fleet-api.md).

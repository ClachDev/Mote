# mote_tasks

The task layer: behaviour trees ([py_trees](https://py-trees.readthedocs.io))
that sit on top of Nav2 and sequence missions. Two missions today — *fetch*,
the skeleton of "pick things up off the floor and take them somewhere", and
*goto*, place-based navigation ("go to the kitchen"):

```
fetch (Sequence)
├── wait_for_task      idle until a command arrives
├── acquire_object     zone target: pass through; label target: ask the detector
├── drive_to_object    Nav2 NavigateToPose to the object pose
├── pick               stub — the SO-101 arm slots in here
├── drive_to_drop      Nav2 NavigateToPose to the drop pose
└── place              stub

goto (Sequence)
├── wait_for_task      idle until a command arrives
└── drive_to_zone      Nav2 NavigateToPose to the zone's pose
```

`task_server.py` hosts both trees and dispatches on the command's first word
(`fetch` / `goto`); an unknown word is rejected. `WaitForTask` and the shared
`task` blackboard key live in `trees/common.py`.

py_trees is pure Python (a pixi PyPI dependency); the ROS glue is ours and
deliberately small: `task_server.py` ticks the tree on a timer, and
`behaviours/nav.py` wraps the Nav2 action client as a behaviour. There is no
py_trees_ros dependency (it isn't packaged for robostack, and this glue is
~100 lines).

## Running

```bash
# Terminal 1: sim with SLAM + Nav2 (no saved map needed)
pixi run sim-mapping

# Terminal 2: the task server (zones default to mote_world coordinates)
pixi run tasks use_sim_time:=true

# Terminal 3: give it a job, watch the status
pixi run -- ros2 run mote_tasks mission --list      # what it will accept
pixi run -- ros2 run mote_tasks mission fetch target=pickup destination=dropoff
```

On the real robot, run `pixi run tasks` alongside `pixi run robot`. Zones
belong to the active site (see the Sites section in CLAUDE.md). Geometry gets
there three ways: `pixi run save-zone <name>` captures the pose the robot is
standing at, `pixi run segment-map` reads room outlines off a saved map, and
the fleet dashboard's zone editor places them on a candidate revision, which a
promotion then hands to every robot at the site. Only `save-zone` proves the
pose is reachable, by having driven to it; the other two put a coordinate on
the map and leave that to the planner. `tasks_launch.py` resolves the active
site's zones automatically, falling back to the committed
`config/zones.default.yaml` (which also documents the format).

## Zones, and "go to the kitchen"

A **zone** is the one named-place concept: a pose in the map frame that the
robot can navigate to. `fetch` uses zones as its `pickup`/`dropoff`
waypoints, and `goto <zone>` drives to any of them — `goto kitchen`,
`goto home`, whatever is in the table. A zone can *optionally* carry an **area
footprint**, which turns it from a bare waypoint into something that also
answers "am I inside it?". That footprint is just optional metadata on the
single zone concept — not a second kind of thing — so there's one loader
whatever wrote the file.

A floor stores this in one `zones.yaml`, and the floor owns it: a zone is a
coordinate in the floor's frame — a fact about the building — where a map
revision is an estimate registered into that frame. A floor still holding
zone/v0's `vocabulary.yaml` + `binding.yaml` pair is read through
`bundle._read_split_pair` and rewritten as one file the first time anything
writes.

```yaml
frame_id: map
zones:
  pickup:  {x: 1.8, y: -1.5, yaw: 0.0}      # bare waypoint
  kitchen: {x: 2.0, y: 2.0, radius: 1.5}    # room: pose + circular footprint
  ward:    {x: 6.0, y: 1.0,                 # room: pose + outline
            polygon: [[4, 0], [9, 0], [9, 3], [4, 3]]}
```

`goto kitchen` navigates to the pose; success is exactly Nav2 reaching it —
the footprint isn't needed for `goto`. `zones.load_zones(path)` returns
`{name: Zone(name, pose, footprint)}`, and `zones.containing(zones, x, y)`
answers "which zone am I in?" (nearest-pose first) using the footprints.

### Circles and polygons

A `radius` is the simple default — one number, and `pixi run save-zone <name>
--radius R` writes it along with the pose. It only describes a roughly round
room, though. A real ward is a rectangle, a ward with an ensuite is an L, and a
corridor stretch is a long thin box; sizing a circle to fit inside one of those
leaves most of the room outside the zone, and sizing it to cover the room
spills into the neighbours. Concretely, the hospital world's wards are 4.7 x
5.6 m — the `radius: 1.5` circle they used to carry claimed 7.1 m² of a 26.5 m²
room, so standing 2 m inside the kitchen answered "you are in no zone".

A `polygon` is a list of `[x, y]` vertices in the file's `frame_id`, closed
implicitly, in either winding order, and may be concave — membership is a ray
cast, not a convex-hull test. A zone carrying both keys uses the polygon.

Polygons do not come from driving. `pixi run segment-map` reads them off a
saved map, and the fleet dashboard's zone editor draws and drags them on a
candidate revision — which is also why a polygon zone may omit `x`/`y`: the
loader then derives a pose guaranteed to lie inside the outline (the centroid,
or, when the shape is concave enough that its centroid falls outside, the
middle of the widest span through it). Where a pose *is* given it always wins,
which matters: in the hospital wards the room centre is occupied by a bed, so
the pose is the doorway approach.

Because polygons arrive from a different direction than poses do, re-capturing
a room's pose with `pixi run save-zone <name>` keeps whatever footprint the
zone already had; passing `--radius R` is the deliberate way to replace it.
Zones are written into the active site's floor (`site info` shows the zone
count, how many have a footprint, and how many names have no geometry here at
all), or the legacy `~/.mote/zones.yaml` when no site is active.

## Interface

All three topics are `std_msgs/String` carrying JSON. A custom message would
have bought type-checking inside the ROS graph and cost the property that
matters more: the fleet agent forwards these payloads to MQTT byte for byte, so
there is one definition of the wire and the bridge cannot reinterpret it.

- `task/capabilities` — a **capability/v0 capability set**, published latched:
  what this robot can be asked to do, with a JSON Schema per input.
  `mote_tasks/capabilities.py` is the authority. Two keys, both from the
  specification's standard registry:
  - `goto` with `{"target": "<zone>"}` — drive to a named zone's pose.
  - `fetch` with `{"target": "<zone or label>", "destination": "<zone>"}` — a
    `target` matching a zone name drives straight there; anything else is an
    **open-vocabulary object label** (underscores become spaces, so
    `red_box` looks for "red box") resolved by the perception stack's detector
    — run `pixi run perception` (the node) and `pixi run inference` (its
    server) alongside the mission (see mote_perception's README, L2).
- `task/command` — a **mission/v0 mission command**: a capability key, a typed
  input, and a correlation id the dispatcher chose.
- `task/status` — a **mission/v0 mission status** per transition:
  `dispatched` (published by the agent) / `accepted` / `succeeded` / `failed` /
  `rejected`, the last two carrying a typed `failure`.

The failure is the point of the typing. `rejected` says `unknown_capability`,
`invalid_input` (naming the property), `busy` (naming the mission holding the
lane), `precondition` or `unresolved_zone` (carrying zone/v0's own reason);
`failed` says `obstructed`, `unreachable`, `timeout` or `internal`, and the
behaviour that failed is what decides which — see `trees/common.py`'s
`report_failure`. Each carries `recoverable`, so a dispatcher retries or does
not without reading prose.

**This node owns the lane.** One mission at a time per lane; a second is
rejected with `busy`. It also evaluates the capability's blocking preconditions
before accepting — `localized` wants a `map`→`base_link` transform, `zone_known`
wants the zone to resolve — and enforces `max_duration_s`, failing a mission that
overran with `timeout`. An unmet *non-blocking* precondition is reported in
`warnings` on the `accepted` status rather than refusing, so a mission that
started degraded is visible.

Label missions go through `behaviours/perception.py`'s `AcquireObject`: it
publishes the label to `detect/labels`, waits for a matching detection on
`detected_objects` (vision_msgs/Detection3DArray, map frame), and writes a
**standoff goal** — 0.4 m short of the object, facing it — to the same
`object_pose` blackboard key the zone path fills in directly. The rest of the
tree cannot tell the difference. The label is cleared on the way out so the
detector idles between missions.

## Where this goes

- **Search**: `acquire_object` currently needs the object visible from where
  the robot stands; a search behaviour (spin in place, tour zones) slots in
  ahead of it.
- **Arm**: `pick`/`place` stubs become real behaviours talking to the SO-101
  (same STS3215 bus as the wheels).
- New missions get their own tree under `trees/`, composed from the shared
  behaviours.

## Testing

`pixi run test` runs the tree tests against a mock `navigate_to_pose` action
server — `test/test_fetch_tree.py` (a full fetch tick), `test/test_fetch_object.py`
(fetch-by-label against a mock detector), and `test/test_goto_tree.py` (a goto
tick, plus command dispatch) — and the pure parser/loader tests
(`test_parse_command.py`, `test_goto_command.py`, `test_zones.py`, which covers
zone footprints and `containing`). No Gazebo, Nav2, or detection server needed.

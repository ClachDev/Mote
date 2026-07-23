# mote_tasks

The task layer: behaviour trees ([py_trees](https://py-trees.readthedocs.io))
that sit on top of Nav2 and sequence missions. The first mission is *fetch* —
the skeleton of "pick things up off the floor and take them somewhere":

```
fetch (Sequence)
├── wait_for_task      idle until a command arrives
├── acquire_object     zone target: pass through; label target: ask the detector
├── drive_to_object    Nav2 NavigateToPose to the object pose
├── pick               stub — the SO-101 arm slots in here
├── drive_to_drop      Nav2 NavigateToPose to the drop pose
└── place              stub
```

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
pixi run -- ros2 topic pub --once /task/command std_msgs/msg/String "{data: fetch pickup dropoff}"
pixi run -- ros2 topic echo /task/status
```

On the real robot, run `pixi run tasks` alongside `pixi run robot`. Zones
belong to the active site (see the Sites section in CLAUDE.md) and are taught
by driving the robot to the spot and running `pixi run save-zone <name>` —
poses are reachable by construction. `tasks_launch.py` resolves the active
site's zones automatically, falling back to the committed
`config/zones.default.yaml` (which also documents the format).

## Interface

- `task/command` (std_msgs/String): `fetch <target> <drop_zone>`. A `target`
  matching a zone name drives straight there; anything else is an
  **open-vocabulary object label** (underscores become spaces, so
  `fetch red_box dropoff` looks for "red box") resolved by the perception
  stack's detector — run `pixi run perception` (the node) and `pixi run inference`
  (its server) alongside the mission (see mote_perception's README, L2).
- `task/status` (std_msgs/String): `accepted:` / `rejected:` / `succeeded:` /
  `failed:` plus the task text
- A task in progress rejects new commands; a failure anywhere (Nav2 abort,
  rejection, missing server, no detection within the timeout) clears the task
  and returns the tree to idle.

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

`pixi run test` runs `test/test_fetch_tree.py` — a full tick of the tree
against a mock `navigate_to_pose` action server — and `test/test_fetch_object.py`,
the fetch-by-label round trip against a mock detector as well. No Gazebo,
Nav2, or detection server needed.

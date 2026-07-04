# mote_tasks

The task layer: behaviour trees ([py_trees](https://py-trees.readthedocs.io))
that sit on top of Nav2 and sequence missions. The first mission is *fetch* —
the skeleton of "pick things up off the floor and take them somewhere":

```
fetch (Sequence)
├── wait_for_task      idle until a command arrives
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

On the real robot, run `pixi run tasks` alongside `pixi run robot` and put
map-specific zones in `~/.mote/zones.yaml` (preferred automatically by
`tasks_launch.py`, same pattern as the camera calibration); the committed
`config/zones.default.yaml` documents the format.

## Interface

- `task/command` (std_msgs/String): `fetch <object_zone> <drop_zone>`
- `task/status` (std_msgs/String): `accepted:` / `rejected:` / `succeeded:` /
  `failed:` plus the task text
- A task in progress rejects new commands; a failure anywhere (Nav2 abort,
  rejection, missing server) clears the task and returns the tree to idle.

## Where this goes

- **Semantics (L2+)**: an object detector replaces `<object_zone>` — the
  perception stack writes a detected object pose to the same blackboard key
  (`object_pose`) and the tree is unchanged.
- **Arm**: `pick`/`place` stubs become real behaviours talking to the SO-101
  (same STS3215 bus as the wheels).
- New missions get their own tree under `trees/`, composed from the shared
  behaviours.

## Testing

`pixi run test` runs `test/test_fetch_tree.py`: a full tick of the tree
against a mock `navigate_to_pose` action server — no Gazebo or Nav2 needed.

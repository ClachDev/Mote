# Camera obstacle layer — why the marks were permanent, 2026-07-29

**Verdict: `camera_layer` is now a `spatio_temporal_voxel_layer` with
`voxel_decay: 5.0`.** A transient obstacle used to leave a mark on the local
costmap that nothing could remove; it now goes on its own in ~5 s with the robot
stationary. This doc is the measurement behind that change.

## The defect

`depth_obstacle_node` publishes **only above-floor points** — `bz > z_obstacle`
inside the 0.25–1.2 m annulus — because the floor is most of the cloud and the
link is Wi-Fi. Nav2's `nav2_costmap_2d::VoxelLayer` clears a voxel only by
raytracing towards a point in a *later* observation. When someone walks through
the camera band and leaves open floor behind them, later clouds contain no
points in that direction at all, so no clearing ray is ever cast through the
voxels they marked. `clearing: True` on the camera source was inert over open
floor, and the marks were permanent for as long as the 3 m rolling window kept
them.

Publishing floor points would only half-fix it: clearing rays leave the camera
at 0.10 m and descend, so voxels in the 0.10–0.18 m band still get no crossing
ray over open floor.

## The measurement

A standalone Nav2 costmap carrying **only** the camera layer (no lidar, no
inflation), with the robot held still at the origin, fed a 0.1 m patch of
points at (0.6, 0.0, 0.10) m for 6 s and then left alone. The robot never
moves, so a mark that disappears can only have decayed. Counting lethal cells
within 0.1 m of the obstacle:

| layer configuration | while present | after the obstacle left |
| --- | --- | --- |
| `nav2_costmap_2d::VoxelLayer` (before) | 4 lethal cells | **still 4 after 20 s — permanent** |
| `spatio_temporal_voxel_layer` (after) | 4 lethal cells | **cleared at 5.4 s** |
| `spatio_temporal_voxel_layer`, `clear_after_reading: False` | 4 lethal cells | **still 4 after 20 s — permanent** |

The clear time is `voxel_decay` (5.0 s) plus the harness's ~0.4 s sample
interval. Re-running the shipped config through the committed tool gives 5.2 s.

The third row is the reason `clear_after_reading: True` carries a shouting
comment in `nav2_params.yaml`. STVL's measurement buffer holds its newest cloud
until something empties it, and every costmap update re-marks whatever it reads
— marking stamps each voxel with the *current* time, which restarts its decay.
Left at the default, STVL is bit-for-bit as permanent as the layer it replaced,
with no error and no warning. It is not a tidiness setting.

## Choosing `voxel_decay`

5.0 s, and it is the only real trade-off in the layer.

- **Too short** and the robot forgets a low obstacle while steering around it.
  The camera loses an obstacle below `range_min` 0.25 m, roughly a second
  before the wheels reach it, and the lidar plane never saw it — so the decay
  window is the robot's entire memory of it during the manoeuvre.
- **Too long** and a transient lingers, which is the complaint being fixed.
  Inflation is 0.35 m, so a stale mark blocks considerably more than itself.

5 s is ~1.1 m of travel at the measured 0.218 m/s `max_wheel_speed`. Revisit it
from a real go-around on the robot, not from this bench.

## What was deliberately not changed

- **Frustum clearing is off** (`clearing: False`). STVL can additionally clear
  everything inside the camera's frustum, accelerated by `decay_acceleration`.
  Decay already answers staleness, and a mis-stated FOV would clear real
  obstacles that the lidar plane cannot see — a worse failure than the one being
  fixed. The parameters to turn it on are named in the config comment.
- **`combination_method: 1`** (Maximum) is unchanged, so the camera still cannot
  erase a lidar mark.
- **The lidar `obstacle_layer` is untouched.**

## Repeating it

```bash
pixi run camera-decay-check     # ~40 s, no hardware, reads the shipped nav2_params.yaml
```

`mote_bringup/test/test_costmap_layers.py` is the static half, cheap enough for
every build: it resolves every configured layer class against the real pluginlib
index (a class no installed package exports is one buried log line while nav2
comes up around it) and asserts the four settings the decay depends on. Each of
those assertions was mutation-checked — typo the plugin string, set
`decay_model: -1`, zero `voxel_decay`, unset `clear_after_reading`, set
`filter: "none"`, or flip `combination_method` to 0, and the suite fails.

The on-robot acceptance check is step 4 of the L1 bring-up list in
`mote_perception/README.md`: stand in front of the stationary robot until you
mark, step out of shot, and the marks must go within ~5 s *without the robot
moving*.

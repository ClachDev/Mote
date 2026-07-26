# Foxglove layout

`mote.json` is the operator's layout: where the robot is, what it sees, and the
controls to drive it. Import it once per Foxglove install — **Layouts → Import
from file…** — then connect to `ws://<robot-id>:8765` over the tailnet. The
operator runbook is [`docs/fleet/README.md` §10](../../docs/fleet/README.md).

JSON has no comments, so the choices are recorded here.

## Panels

| Panel | Shows |
|---|---|
| **3D** | `/map` with the robot on it, `/scan_filtered`, `/plan`, TF, and the URDF model. Follows `base_footprint`, orthographic — a floor map read from above, not a 3D scene |
| **Image** | `/image_raw/compressed`. The compressed stream, never `/image_raw`: raw frames are ~100x the bytes over a WAN link |
| **Teleop** | drives the robot — see below |
| **Diagnostic summary** | `/diagnostics_agg` from the health monitor, per subsystem |

Costmaps, `/camera_obstacles`, `/detected_objects` and `/particle_cloud` ship
**present but hidden**: they are the topics you want one click away when
something looks wrong, and each is expensive enough that leaving it on would tax
every connection for the rare occasion it is needed. The bridge only serialises
what a panel has actually subscribed to, so a hidden topic costs nothing.

## Teleop

The panel publishes **`/cmd_vel_teleop`, not the controller's own topic**. It can
only emit `geometry_msgs/Twist`, while `DiffDriveController` consumes
`TwistStamped`, so `twist_relay` (started by `foxglove_launch.py`) adds the
header. The stamp is applied *on the robot*, which keeps the operator's clock out
of the safety path entirely.

Two numbers are load-bearing:

- **`publishRate: 10`.** The controller's `cmd_vel_timeout` is 0.5 s, so anything
  below 2 Hz makes the robot stutter — it halts between commands. Foxglove's
  panel default is 1 Hz, which is exactly that failure, so the rate is set here
  rather than left alone.
- **0.15 m/s and 0.6 rad/s**, roughly half of what `controllers.yaml` permits.
  Remote driving pays the link's latency before you see the consequence of a
  command, so the layout is deliberately slower than the robot can go. The
  controller clamps to its own maxima regardless, so editing these in the UI
  cannot exceed the robot's configured limits.

Releasing a button stops the robot, and so does losing the link: commands stop
arriving and `cmd_vel_timeout` halts the wheels. There is no remote e-stop — all
safety behaviour is local, as [`docs/design/fleet.md`](../../docs/design/fleet.md)
scopes it.

**Teleop does not pre-empt Nav2.** Both publish to the controller, so driving by
hand during an active goal means two writers fighting. Cancel the task first.
Arbitrating properly needs a `twist_mux` in the drive path, which is a change to
how the robot drives rather than to how it is watched, so it is filed as
follow-up work instead of being smuggled into this milestone.

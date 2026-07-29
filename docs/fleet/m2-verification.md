# M2 verification ledger

What was measured for [M2](README.md#10-watching-and-driving-one-robot-foxglove),
how, and what is still unverified. M2 is the milestone that lets an operator
*watch and drive* one robot remotely, and the milestone that finally pins the
robot's DDS graph to the robot.

## 1. A browser teleop panel reaches the wheels — **confirmed end to end**

The acceptance criterion's "drives" half, run against the real
`foxglove_bridge` 3.3.0 with a WebSocket client speaking the protocol the
Teleop panel speaks. Everything is real except the operator and the motors: the
bridge, the wire, the `twist_relay` node, and a `TwistStamped` subscriber
standing in for `DiffDriveController`.

```console
$ pixi run -e dev test-foxglove
2 passed in 12.98s
```

The two facts it pins down (`mote_bringup/test/test_foxglove_teleop.py`):

- a client advertise turns into a real ROS publisher, and the bytes arrive with
  the velocities intact;
- `twist_relay` converts them to `TwistStamped` with a **non-zero stamp taken on
  the robot** — which is what `cmd_vel_timeout` measures, so the operator's clock
  never enters the safety path.

### The type-name trap, checked rather than assumed

Foxglove's Teleop panel advertises the **ROS 1 spelling** `geometry_msgs/Twist`
([`TeleopPanel.tsx`](https://github.com/foxglove/studio) hardcodes it), while the
ROS 2 type is `geometry_msgs/msg/Twist`. Had the bridge not normalised that, the
panel would have looked connected and the robot would simply never have moved —
a silent failure, and the sort that is expensive to diagnose from the far end of
a WAN link. So it was measured, with a separate topic per variant so a publisher
created by one could not make the next look like it worked:

| client advertises | schema sent | ROS messages received |
|---|---|---|
| `geometry_msgs/msg/Twist` | yes | 20 / 20 |
| `geometry_msgs/Twist` | yes | 20 / 20 |
| `geometry_msgs/msg/Twist` | no | 20 / 20 |
| `geometry_msgs/Twist` | no | 20 / 20 |

All four work — the bridge normalises the name and does not require the schema
at all. The e2e test advertises the **ROS 1** spelling deliberately, so that if a
future bridge stops normalising, the test fails here rather than an operator
discovering it on a robot.

### The subprotocol changed under this version — worth knowing

`ros-jazzy-foxglove-bridge` **3.3.0 speaks `foxglove.sdk.v1`**, not the
`foxglove.websocket.v1` that the older bridge and most third-party examples use.
A client offering only the old subprotocol is refused at the handshake:

```console
['foxglove.websocket.v1'] -> FAIL InvalidStatus: server rejected WebSocket connection: HTTP 400
['foxglove.sdk.v1']       -> OK   {"op":"serverInfo", "capabilities":["clientPublish", ...]}
```

Current Foxglove offers both and negotiates `foxglove.sdk.v1`, so this does not
affect the shipped path. It matters for anything *else* pointed at the bridge —
an old Studio build, a scripted client, a third-party tool — which will fail with
a bare HTTP 400 and no explanation.

## 2. Teleop still works with DDS pinned to the robot — **confirmed**

This is the bet M2 exists to make: that `foxglove_bridge` is a real replacement
for joining the robot's ROS graph from a workstation, and therefore that
discovery can stop leaving the machine. The same e2e run, with the pin on:

```console
$ ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST pixi run -e dev test-foxglove
2 passed in 12.98s
```

The WebSocket is TCP and unaffected by DDS discovery scope, and the bridge shares
a host with the graph it serves, so the pin costs the operator nothing.

**Where the pin is applied:** the systemd units only — every `mote-*.service`
now carries `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, alongside the
`CYCLONEDDS_URI` they already had. An interactive `pixi run` keeps stock
discovery, exactly as `cyclonedds.xml` already worked, so bench workflows that
depend on a LAN graph — RViz, camera calibration, `pixi run teleop` from a
workstation — are untouched. Consistency is the reason it went on *all* the
units rather than the new one: a localhost-range participant can discover a
same-host default-range one but not the reverse, so a mixed set would be
asymmetric and confusing rather than half-safe.

## 3. DDS participant cost — **2 slots**, and the budget still holds

M0 asked that `dds-check` be re-run whenever a milestone adds processes, having
measured 17/33 for the sim nav mission and projected ~24 with M1 and M2 added.

```console
$ pixi run dds-check                      # nothing running
0/33 participant slots used, 33 free (MaxAutoParticipantIndex=32)

$ pixi run dds-check                      # with `pixi run foxglove` up
  index   0  port 30160  pid 3701921  foxglove_bridge __node:=foxglove_bridge
  index   1  port 30162  pid 3701923  python3.12 twist_relay
2/33 participant slots used, 31 free (MaxAutoParticipantIndex=32)
```

Two, not one: the projection counted `foxglove_bridge` and not the teleop relay,
which did not exist as a concept until the panel's message type forced it. The
robot's nav mission plus perception plus M1's agent plus these two lands around
**25 of 33** — still headroom, and still the number to re-check at M3.

## 4. The launch brings up what it claims — **confirmed**

```console
$ pixi run foxglove
[foxglove_bridge-1] Starting foxglove_bridge (jazzy, 3.3.0@7c27d75d-dirty)
[foxglove_bridge-1] Server listening on port 8765
[foxglove_bridge-1] Advertising new channel 4 for topic "/diff_drive_controller/cmd_vel"
[foxglove_bridge-1] Advertising new channel 5 for topic "/cmd_vel_teleop"

$ pixi run -- ros2 topic info /diff_drive_controller/cmd_vel
Type: geometry_msgs/msg/TwistStamped
Publisher count: 1
```

## 5. Not verified here — needs Foxglove, or the hardware

- **The layout has never been opened in Foxglove.** No Foxglove client is
  installable in this environment, so `mote_bringup/foxglove/mote.json` is
  verified only as far as `test_foxglove_layout.py` goes: valid JSON, every
  placed panel configured, the teleop rate above the controller's deadman, the
  teleop velocities inside the controller's limits, the camera panel on the
  compressed stream. Those are the things that drift silently. What they do
  **not** prove is that Foxglove renders it. The panel *configs* are the least
  certain part — they were written against the panel schemas rather than
  exported from the app, and the **URDF layer in the 3D panel is the single most
  likely thing to need a click to fix**, since its key names could not be
  checked against a running client. A layout key Foxglove does not recognise is
  ignored rather than fatal, so the expected worst case is a panel that opens
  with a default setting, not a layout that fails to import.
- **The acceptance criterion, off-LAN, on the robot.** Everything above ran on
  one workstation. What makes it remote is the M0 tailnet rather than this code —
  the same WebSocket over a WireGuard interface is the same WebSocket — but that
  hop is unrun, as it is for
  [M1](m1-verification.md#5-not-verified-here--needs-the-hardware). To close it:
  `systemctl enable --now mote-foxglove` on the Pi, connect Foxglove to
  `ws://<robot-id>:8765` from a tethered laptop, and drive.
- **Teleop against a moving robot.** The relay was exercised against a
  subscriber, not against `DiffDriveController` on real wheels, so
  `cmd_vel_timeout` halting the robot when the link drops is reasoned from the
  controller's documented behaviour ("Timeout in seconds, after which input
  command on cmd_vel topic is considered staled") and not observed. It is the
  one safety claim in M2 that has not been watched happening.
- **`mote-foxglove.service` under systemd.** The unit is installed by
  `pixi run setup` and modelled on `mote-health.service`, but has only been run
  via `pixi run foxglove`.
- **The dashboard's deep link, clicked.** M3 ships an *open in Foxglove* button
  per robot and its own ledger records the far end as unobserved, because
  "neither exists until M2"
  ([`m3-verification.md`](m3-verification.md)). The two agree on paper — the
  server's template is `ws://<robot_id>:8765`, which is this bridge's default
  port on the MagicDNS name M0 assigns — but nobody has clicked it and watched
  Foxglove connect. **M3's open item and this milestone's are the same test**:
  one session with the desktop app, a Pi, and the layout closes both. The
  mixed-content reasoning about the hosted web app (§10 of the runbook) is part
  of what that session should confirm rather than trust.
- **Bandwidth over a real link.** The camera is streamed compressed and only
  while a panel is subscribed, but nothing here measured what a DERP-relayed
  connection does to it.

## 6. Teleop pre-empting Nav2 — **closed, after M2**

M2 shipped with this as a known limitation: both the relay and Nav2's controller
published `TwistStamped` to `/diff_drive_controller/cmd_vel`, so driving by hand
during an active goal meant two writers competing, and the documented remedy was
to cancel the task first. That was deliberate rather than overlooked —
arbitrating properly is a change to *how the robot drives*, affecting every
mission including fully autonomous ones, inside a milestone whose subject is *how
the robot is watched* — so it was filed as follow-up work.

It has since been done, on its own: `twist_mux` now sits in the drive path with
teleop above navigation, so the first arrow an operator presses takes the wheels.
The design, the numbers and what they were measured against are in
[`mote_bringup/README.md` "Drive path"](../../mote_bringup/README.md#drive-path--who-gets-the-wheels).
Three consequences that matter to this ledger:

- the relay's output is `/cmd_vel_teleop_stamped` now, not the controller's
  topic. The panel's own topic is unchanged, so §1's measurements and the shipped
  layout still hold;
- §3's participant count goes up by one, to ~26 of 33;
- the deadman claim in §5 is unchanged and still the thing to watch on hardware.
  `twist_mux` publishes only from an input callback and stores no last command,
  which `test_twist_mux_arbitration.py` asserts by watching the drive topic stay
  silent after every source stops — but "`cmd_vel_timeout` halts real wheels"
  remains reasoned from the controller's documentation, exactly as it was.

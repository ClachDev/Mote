# M0 verification ledger

What was measured for [M0](README.md), how, and what is still unverified. The
fleet design doc flagged two of these **(verify)**; both are resolved here.

## 1. `MaxAutoParticipantIndex` under localhost discovery — **confirmed, 32**

The design doc's caveat was "~32, verify". It is exactly 32, and it comes from
rmw_cyclonedds itself, not from CycloneDDS defaults — the config fragment is
compiled into the RMW:

```console
$ strings .pixi/envs/default/lib/librmw_cyclonedds_cpp.so | grep -A2 ParticipantIndex
<Discovery><ParticipantIndex>none</ParticipantIndex>
<Discovery><ParticipantIndex>auto</ParticipantIndex>
<MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
</Discovery></Domain></CycloneDDS>
```

(`ros-jazzy-rmw-cyclonedds-cpp` from robostack-jazzy. `LOCALHOST` selects the
`auto` + cap variant and adds `<Peer address="localhost"/>`; the wider ranges use
`none`.) So indices 0–32 — **33 concurrent participants per host, per domain** —
and past that participant creation, and therefore node creation, fails.

## 2. How many participants the stack actually uses — **17 of 33**

Measured with `pixi run dds-check` (`mote_bringup/dds_participants.py`, which
reads the RTPS port map out of `/proc/net/udp`) against `pixi run sim-nav` on the
workstation, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, sampled at t = 10, 20,
30, 45, 60, 90, 120 s. Identical at every sample:

```
domain 0  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST  multicast discovery port bound: no
  index   0  port 7410  robot_state_publisher      index   9  port 7428  controller_server
  index   1  port 7412  scan_to_scan_filter_chain  index  11  port 7432  map_server
  index   2  port 7414  bt_navigator               index  12  port 7434  parameter_bridge (gz)
  index   3  port 7416  behavior_server            index  13  port 7436  lifecycle_manager (nav)
  index   4  port 7418  kinematic_icp_online_node  index  14  port 7438  smoother_server
  index   5  port 7420  amcl                       index  15  port 7440  odom_tf_relay
  index   6  port 7422  waypoint_follower          index  16  port 7442  task_server
  index   7  port 7424  planner_server             index  17  port 7444  gz sim (mote_world.sdf)
  index   8  port 7426  lifecycle_manager (loc)
17/33 participant slots used, 16 free (MaxAutoParticipantIndex=32)
```

Projecting to the robot: swap the two Gazebo processes (gz sim, `parameter_bridge`)
for the four the hardware base adds (`ros2_control_node`, `sllidar`,
`v4l2_camera`, `system_monitor`) → ~19; `pixi run perception` adds three
(`camera_monitor`, `depth_obstacle_node`, `object_detector_node`) → ~22; M1's
`mote_agent` and `foxglove_bridge` → **~24 of 33**.

Two things this measurement teaches:

- **One participant per process**, not per node — a composed/multi-node process
  claims one slot.
- **Indices are released on exit**, so the peak is what matters. An earlier run of
  the same mission, sampled while a previous stack was still shutting down, showed
  the survivors sitting at indices 19–31: transient processes (controller
  spawners, `ros2` CLI calls, the ROS daemon) each hold a slot while they run, and
  a stack starting on top of a dying one can climb much closer to the cap than its
  steady-state count suggests.

Verdict: headroom is real but finite. `pixi run dds-check` exists so every
milestone that adds processes can check rather than assume.

## 3. Localhost-pinned nodes still interoperate on one host — **confirmed**

The worry: pinning the robot to `LOCALHOST` breaks the workstation workflows that
share a graph. Matrix run on one machine (`ROS_DOMAIN_ID=47`, one publisher, one
subscriber, 12 s timeout, `rmw_cyclonedds_cpp`):

| publisher | subscriber | result |
|---|---|---|
| LOCALHOST | LOCALHOST | received |
| LOCALHOST | SUBNET | received |
| SUBNET | LOCALHOST | received |
| SUBNET | SUBNET | received |

So a `dev`-environment RViz (`SUBNET`) and a localhost-pinned sim or robot stack
on the *same* machine still see each other. Note the LOCALHOST→SUBNET case took
noticeably longer to discover (~8 s vs <2 s) — the subnet node has to wait for the
localhost node's periodic announcement rather than meeting it on multicast. Only
*cross-machine* discovery is actually cut off, which is the intent.

The mission-level check is the same sim-nav run as §2: under `LOCALHOST` the Nav2
stack configured, activated and bonded normally (`lifecycle_manager_navigation:
Managed nodes are active`), so pinning discovery does not disturb a stack whose
processes all live on one host — which is every mission the robot runs.

## 4. Not verified here — needs the hardware

- **Clean-Pi → reachable by MagicDNS off-LAN.** The cloud-init template is
  rendered, schema-checked and unit-tested (`mote_bringup/test/test_provision.py`
  parses the output as cloud-config and asserts the ordering invariants: identity
  before tailnet, tailnet before key shred), but it has not been booted on a
  card. Running it requires a spare SD card, a Tailscale auth key and a Pi.
- **`tailscale up` on the robot**, and therefore the tag/ACL behaviour. The
  install script is syntax-checked only.
- **Identity stable across reboots** is unit-tested at the file level
  (`test_identity.py`); the reboot itself is untested.

The acceptance checklist for those three is step 5 of
[README.md §5](README.md#5-provisioning-a-clean-pi).

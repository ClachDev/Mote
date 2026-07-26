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

**The same count under stock discovery.** Re-run with
`ROS_AUTOMATIC_DISCOVERY_RANGE` unset — what the robot actually runs today, since
M0 does not pin it (§4) — the same 17 processes claim the same indexed ports
(7410–7445), plus the multicast discovery ports 7400/7401 that `LOCALHOST`
suppresses and that `dds-check` reports as `multicast discovery port bound: yes`.
So the tool reads correctly on a LAN-discoverable robot; what changes with the
pin is that the 32 cap starts binding. (Stock mode's ceiling is CycloneDDS's own
default rather than the RMW's 32, and was not measured — it is demonstrably above
17, and `dds-check --max-index` takes another value if that ever matters.)

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

So a default-range RViz and a localhost-pinned sim on the *same* machine still see
each other. Note the LOCALHOST-publisher → SUBNET-subscriber case took noticeably
longer to discover (~8 s vs <2 s) — the subnet node has to wait for the localhost
node's periodic announcement rather than meeting it on multicast. Only
*cross-machine* discovery is actually cut off, which is the intent.

This is worth flagging because PR #54 documents the visibility as *one-way* ("a
`LOCALHOST` participant still finds same-host default-range ones, not the
reverse"), which is the stated reason `pixi run rviz-sim` exists. Measured here it
works in both directions, just slowly one way — so `rviz-sim` may be papering over
slow discovery rather than absent discovery. One pub/sub pair is weaker evidence
than RViz's full topic set, so nothing was changed on the strength of it; it is
recorded as a cheap thing to re-check.

The mission-level check is the same sim-nav run as §2: under `LOCALHOST` the Nav2
stack configured, activated and bonded normally (`lifecycle_manager_navigation:
Managed nodes are active`), so pinning discovery does not disturb a stack whose
processes all live on one host — which is every mission the robot runs.

## 4. Why the localhost pin is not in M0 after all

M0 was written against a milestone that said "pin DDS to localhost". PR #54
landed first and amended that milestone: the robot stays LAN-discoverable and is
narrowed by `mote_bringup/config/cyclonedds.xml` (one interface, SPDP-only
multicast) instead, "because nothing on-robot replaces an operator's RViz yet —
M2 is what makes the localhost pin safe there".

That is the right call and this branch follows it, so the pin was dropped from
this work. The reasoning matters more than the setting: a localhost pin on the
robot cannot be worked around from the operator's side. `dev` set to `SUBNET`
does nothing, because it is the *robot's* participants that stop announcing —
the only escape is remembering an environment variable on the Pi. So the pin is
only safe once Foxglove gives the operator a path that isn't DDS.

What survives from the original plan is everything that made the pin decidable:
the cap is confirmed (§1), the budget is measured (§2), and the interop
behaviour is known (§3). M2 flips one line with the numbers already in hand.

## 5. Not verified here — needs the hardware

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

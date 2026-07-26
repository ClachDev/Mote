# Fleet M0 — overlay + identity

The foundation every other fleet milestone stands on: a network where the
LAN/internet distinction has disappeared, a stable name for each robot, and a
sharp line between what ships in the package and what belongs to one machine.
The architecture and the milestones after this one are in
[`docs/design/fleet.md`](../design/fleet.md); the measurements behind the
choices here are in [`m0-verification.md`](m0-verification.md).

M0 deliberately has **no fleet server**: identity is operator-set, not
server-allocated. M1 replaces the operator with an enrollment endpoint without
changing the file's shape.

| | |
|---|---|
| `pixi run identity` | this robot's `id` / `name` / `site` |
| `pixi run tailnet` | join this machine to the Tailscale overlay |
| `pixi run provision` | render cloud-init user-data for a clean Pi |
| `pixi run dds-check` | DDS participant-slot headroom on this host |

---

## 1. The overlay: one tailnet, no port forwarding

Every robot, the workstation, the fleet box and the GPU inference box join a
single [Tailscale](https://tailscale.com/) tailnet. WireGuard gives each of them
an encrypted, NAT-traversing link and a stable MagicDNS name, so "same LAN, same
`ROS_DOMAIN_ID`" becomes "same tailnet" and nothing is ever exposed to the public
internet.

```bash
# on the robot (identity must exist first — its id becomes the MagicDNS name)
pixi run tailnet --role robot --auth-key tskey-auth-...

# on the workstation (a user device, untagged)
pixi run tailnet --role workstation

# on the fleet box / the GPU box
pixi run tailnet --role fleet
pixi run tailnet --role inference
```

`tailscale up` is declarative, so re-running is a no-op — the script is safe to
run from provisioning and by hand.

**Roles are tags.** Robots and servers join as *tagged* devices
(`tag:robot`, `tag:fleet`, `tag:inference`): a tagged device is owned by the
tailnet rather than by a person, so it outlives the operator's account and can be
ACL'd as a class. That is what M7's ACLs (operators reach robots; robots reach the
broker and their local inference box; robots cannot reach each other) will be
written against. The workstation stays a user device. Tags must exist in the
tailnet policy and the auth key that mints a robot must be
[pre-authorised and tagged](https://tailscale.com/kb/1085/auth-keys).

**Verify off-LAN** (the M0 acceptance test) — from a device on a *different*
network, e.g. a laptop tethered to a phone:

```bash
tailscale ping mote-01          # direct WireGuard path, or via a DERP relay
ssh michael@mote-01             # MagicDNS name == robot id
```

Once a robot is on the tailnet, its MagicDNS name works anywhere a hostname
does — including `pixi run sync`, whose target is still the legacy hardcoded
`michael@auldbot` (`pixi.toml`). Retargeting it at the robot id is a
one-line change to make when the current Pi is re-provisioned.

Cost shape and the escape hatch (Headscale) are in the design doc; at homelab
scale the free tier covers this.

---

## 2. Identity: `robot_id` is the fleet's primary key

```bash
pixi run identity set --id mote-01 --name "Front desk" --site home
pixi run identity show
pixi run identity id            # just the id, for scripts
```

writes `$MOTE_HOME/robot.yaml` (`~/.mote/robot.yaml` by default):

```yaml
schema: 1
id: mote-01
name: Front desk
site: home
```

- The **id** keys everything downstream: the MQTT topic tree
  (`mote/<robot_id>/…`), the registry row, and the robot's MagicDNS name. It is
  constrained to a lowercase DNS label (letters, digits, hyphens, ≤32) because it
  has to be simultaneously a hostname, an MQTT topic level and a directory name.
- It is **not the hostname**. The hostname is already ambiguous in this repo
  (`auldbot` in `pixi.toml` vs `mote` in the docs), and re-imaging a Pi must not
  silently mint a new fleet member. Provisioning does set the OS hostname to the
  id for convenience, but nothing depends on that.
- It is **stable across reboots and across updates**: the file lives in
  `MOTE_HOME`, outside the package, so an update physically cannot touch it.
- `name` is a free-text label, `site` says which site's map bundles this robot is
  entitled to. Both are optional to the software.

---

## 3. Per-robot state vs shared config

The rule, now enforced in one place
([`mote_bringup/mote_home.py`](../../mote_bringup/mote_bringup/mote_home.py)):
**shared config ships in the package; per-robot state lives under `MOTE_HOME`.**

| | Shared (package) | Per-robot (`$MOTE_HOME`, default `~/.mote`) |
|---|---|---|
| what | identical on every robot of a version | belongs to this machine |
| examples | `mote_description/config/robot.yaml` (wheel geometry, servo bus, device paths), `nav2_params.yaml`, `slam_toolbox_params.yaml`, `zones.default.yaml`, `camera_info.default.yaml`, `perception.yaml` | `robot.yaml` (identity), `active.yaml` (site + floor), `sites/…` (maps, zones, posegraphs), `bags/…`, `camera_calibration.yaml`, `perception.yaml` override |
| lifecycle | replaced wholesale by an update | survives every update |
| under version control | yes | no |

Two files are called `robot.yaml` and they are **not** the same thing:
`mote_description/config/robot.yaml` is the shared *hardware description*;
`$MOTE_HOME/robot.yaml` is this robot's *identity*.

Where a config has both halves, the per-robot file wins:
`mote_home.override("perception.yaml", packaged_default)`. Launch files use that
helper, so `MOTE_HOME` is honoured everywhere — which is what lets the sim point
it at an in-repo bundle and tests point it at a tmpdir.

The consequence the fleet layer needs: **an update can never clobber identity,
site selection, calibration, maps or bags.**

---

## 4. DDS: measured now, pinned to the robot at M2

The end state is that DDS never leaves a robot
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`): nothing off-box joins its ROS graph
because the fleet layer bridges over MQTT and Foxglove instead, so two robots
parked on the same LAN cannot see each other's nodes whatever their
`ROS_DOMAIN_ID` and there is no domain-id allocation problem at any fleet size.

**M0 does not flip that switch on the robot, deliberately.** Nothing on-robot
replaces an operator's RViz yet, so pinning the robot today would break the one
remote workflow that exists. The pin lands with **M2**, when `foxglove_bridge`
gives the off-box path — this is what the milestone in
[`fleet.md`](../design/fleet.md) now says. Until then the scoping in place is the
one from PR #54:

- sims and benchmarks pin *themselves* (`[feature.sim.activation.env]`), so a sim
  is invisible to the LAN and to other machines;
- the robot stays LAN-discoverable but is narrowed by
  [`mote_bringup/config/cyclonedds.xml`](../../mote_bringup/config/cyclonedds.xml)
  under systemd — one interface, SPDP-only multicast.

What M0 contributes here is the **measurement**, because the pin has a ceiling
worth knowing before we walk into it.

### The participant cap — check it before adding processes

Under `LOCALHOST`, rmw_cyclonedds hands CycloneDDS
`<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>`:
each participant (one per ROS *process*, in practice) claims an index 0–32 and
binds UDP ports derived from it. **Past 32, participant creation fails, which
means node creation fails.**

```bash
pixi run dds-check          # or: --json for a machine-readable report
```

Measured on the sim nav mission: **17 of 33 slots**, stable for the whole run —
and the same 17 under stock discovery, so the tool reads correctly on the robot
as it runs today. The projected robot stack — nav mission with real drivers, plus
perception, plus M1's agent and `foxglove_bridge` — lands around **24**. That is
headroom, but not an unlimited amount, and it is spent *before* M2 arrives to
claim it. Re-run `dds-check` whenever a milestone adds processes; if it runs out,
raise `MaxAutoParticipantIndex` in the robot's existing `cyclonedds.xml`.

Indices are released when a process exits, so what matters is the *concurrent*
peak; transient helpers (controller spawners, `ros2` CLI calls, the ROS daemon)
each take a slot while they run.

---

## 5. Provisioning a clean Pi

The image is stock Raspberry Pi OS plus **one file**: `user-data` on the boot
partition. Raspberry Pi Imager 2.0+ writes cloud-init user-data for its own
OS-customisation, so this replaces that dialog rather than sitting beside it.

The ordering trap — "the robot needs the tailnet to reach the server, but the key
comes from the server" — is resolved by making the Tailscale key a
**provisioning-time secret baked into the image**, minted out-of-band by the
operator. Nothing is fetched over a network the robot cannot yet use.

1. **Mint a single-use, pre-authorised, tagged key** in the Tailscale admin
   console (tag `tag:robot`, short expiry, ephemeral off).
2. **Image the card** with Raspberry Pi Imager — *skip* the OS-customisation
   dialog.
3. **Render and install `user-data`:**

   ```bash
   pixi run provision --id mote-02 --name "Back office" \
       --ssh-key ~/.ssh/id_ed25519.pub \
       --ts-authkey tskey-auth-... \
       --wifi-ssid HomeNet --wifi-psk '…' \
       --boot /media/$USER/bootfs
   ```

   The output carries a live auth key and a wifi passphrase: it is written `0600`
   and must never be committed.
4. **Boot the Pi.** With no interactive steps it: writes
   `~/.mote/robot.yaml`; joins the tailnet as `mote-02` and shreds the key;
   installs pixi, clones the workspace and builds it; runs `pixi run setup` (udev,
   wifi power save, systemd units).
5. **Verify** from any tailnet device, off-LAN:
   `tailscale ping mote-02 && ssh <user>@mote-02 'cd ~/Mote && pixi run identity id'`.
   First boot takes a while — the build is the long pole. Progress lands in
   `/var/log/mote-provision.log` and `/var/log/cloud-init-output.log`.

**Raspberry Pi Connect stays as the break-glass path**, independent of our
tailnet and fleet server, so a Pi with a botched key or a broken agent is still
recoverable without a keyboard.

Note that step 4 builds from source because the prefix.dev `mote` channel does
not ship a robot package yet; when it does (M5), that step becomes a pinned
`pixi install` and first boot gets much shorter. The template is the only thing
that changes.

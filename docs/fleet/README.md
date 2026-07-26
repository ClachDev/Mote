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

**One machine, several roles.** A machine is one tailnet node, and `tailscale up`
replaces the whole tag set — so roles are passed *together*, never in two runs
(the second would silently drop the first's tag). A home box that is both the
fleet server and the GPU inference node is one call:

```bash
pixi run tailnet --role fleet,inference
pixi run tailnet --role fleet,inference --dry-run   # resolve roles/tags/hostname only
```

**Should your dev machine take those roles?** Only if you want it to stop being
*yours*. Advertising a tag transfers the node from your user account to the
tailnet, which needs a re-auth and drops key expiry — so `--role workstation` and
any tagged role are mutually exclusive and the script refuses the combination.
For one operator at one site, leave the dev machine an untagged workstation that
happens to run Mosquitto and the inference servers: nothing functional depends on
the tag (robots reach it by MagicDNS either way, and `inference_host` is just a
name). What you defer is M7's ACLs — a rule keyed on your user's device rather
than `tag:fleet`/`tag:inference`. Tag it when a second person or a second machine
appears and the roles want to outlive your account.

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
pixi run identity set --id mote-01 --name "Scout" --site home
pixi run identity show
pixi run identity id            # just the id, for scripts
```

writes `$MOTE_HOME/robot.yaml` (`~/.mote/robot.yaml` by default):

```yaml
schema: 1
id: mote-01
name: Scout
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
- `name` is a free-text label for the *robot*, so name it like an individual
  ("Scout") rather than a place ("Front desk") — places are already a concept
  here, and a robot label that reads like a zone name is a trap when both appear
  in the same dispatch UI. `site` says which site's map bundles this robot is
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

**Everything up to "boot the Pi" happens on the workstation, against the SD card
mounted there.** The Pi runs nothing until it boots; it has no pixi, no network
config and no repo. `pixi run provision` is a workstation command that writes a
file onto the card — it never runs on the robot. The wifi *is* configured, by the
template, which is precisely why the Imager dialog is skipped: Imager would write
its own `user-data` to the same path and one would clobber the other.

The ordering trap — "the robot needs the tailnet to reach the server, but the key
comes from the server" — is resolved by making the Tailscale key a
**provisioning-time secret baked into the image**, minted out-of-band by the
operator. Nothing is fetched over a network the robot cannot yet use.

**Prerequisite:** the image must be one whose first boot runs cloud-init.
Raspberry Pi OS gained that with the Imager 2.0-era images; if `user-data` is
ignored on first boot, that is the thing to check first — none of this has been
run on real hardware yet (see [m0-verification.md §5](m0-verification.md)).

1. **[workstation] Mint a single-use, pre-authorised, tagged key** in the
   Tailscale admin console (tag `tag:robot`, short expiry, ephemeral off). The
   tag must already exist in the tailnet policy with an owner, or redeeming the
   key fails.
2. **[workstation] Image the card** with Raspberry Pi Imager — choose the OS and
   the card, and *skip* the OS-customisation dialog entirely (answer "no" when it
   offers to apply settings).
3. **[workstation] Re-insert the card.** Imager ejects it when it finishes, so
   the boot partition is not mounted any more. Pull it out, put it back, and
   check where it landed — usually `/media/$USER/bootfs`:

   ```bash
   lsblk -o NAME,LABEL,MOUNTPOINT | grep -i boot
   ```

4. **[workstation] Render `user-data` onto the card:**

   ```bash
   pixi run provision --id mote-02 --name "Rover" \
       --ssh-key ~/.ssh/id_ed25519.pub \
       --ts-authkey tskey-auth-... \
       --wifi-ssid HomeNet --wifi-psk '…' --wifi-country GB \
       --boot /media/$USER/bootfs
   ```

   `--wifi-country` is required with wifi and is not guessed: the Pi's WLAN stays
   rfkill-blocked until the regulatory domain is set. Omit the three wifi flags
   entirely if the robot is on ethernet.

   The output carries a live auth key and a wifi passphrase: it is written `0600`
   and must never be committed. Print it to stdout first (drop `--boot`) if you
   want to read it before it goes near a card.

5. **[workstation] Eject the card**, so the write is actually on it:

   ```bash
   sync && udisksctl unmount -b /dev/sdX1     # or just: umount /media/$USER/bootfs
   ```

6. **[Pi] Boot it.** With no interactive steps it: brings up wifi; writes
   `~/.mote/robot.yaml`; joins the tailnet as `mote-02` and shreds the key;
   installs pixi, clones the workspace and builds it; runs `pixi run setup` (udev,
   wifi power save, systemd units).
7. **[anywhere on the tailnet] Verify**, ideally off-LAN:

   ```bash
   tailscale ping mote-02
   ssh <user>@mote-02 'cd ~/Mote && pixi run identity id'
   ```

   First boot takes a while — the build is the long pole, tens of minutes on a Pi.
   The robot appears on the tailnet long before it finishes, so `tailscale ping`
   succeeding while ssh has no repo yet is expected. Progress lands in
   `/var/log/mote-provision.log` and `/var/log/cloud-init-output.log`.

**Raspberry Pi Connect stays as the break-glass path**, independent of our
tailnet and fleet server, so a Pi with a botched key or a broken agent is still
recoverable without a keyboard.

Note that step 6 builds from source because the prefix.dev `mote` channel does
not ship a robot package yet; when it does (M5), that step becomes a pinned
`pixi install` and first boot gets much shorter. The template is the only thing
that changes.

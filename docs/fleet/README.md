# Fleet — overlay, identity, control plane, and the operator view

The operator runbook for the fleet layer: a network where the LAN/internet
distinction has disappeared (**M0**), a stable name for each robot (**M0**), a
server that hands out those names and carries tasks and telemetry to and from
every robot (**M1**), and a browser you watch and drive the fleet from
(**M3**). The architecture and the milestones after these are in
[`docs/design/fleet.md`](../design/fleet.md); the measurements are in
[`m0-verification.md`](m0-verification.md),
[`m1-verification.md`](m1-verification.md) and
[`m3-verification.md`](m3-verification.md); the two wires are specified in
[`control-plane.md`](control-plane.md) (MQTT) and [`fleet-api.md`](fleet-api.md)
(HTTP).

| | |
|---|---|
| `pixi run identity` | this robot's `id` / `name` / `site` |
| `pixi run tailnet` | join this machine to the Tailscale overlay |
| `pixi run provision` | render cloud-init user-data for a clean Pi |
| `pixi run dds-check` | DDS participant-slot headroom on this host |
| `pixi run fleet-broker-ws` | the MQTT control plane, with WebSockets (fleet box) |
| `pixi run fleet-broker` | the same, from conda — no WebSockets, no dashboard |
| `pixi run fleet-server` | fleet API + operator dashboard (fleet box) |
| `pixi run fleetctl` | operator CLI: tokens, roster, dispatch, audit, watch |
| `pixi run enroll` | ask the server for this robot's identity |
| `pixi run agent` | the robot's bridge to the fleet |

**First time through**, in order: §1a (create the tailnet — browser, once, for
the whole fleet) → §1b (join your workstation and the fleet box) → §6 (stand up
the fleet server) → §7 (enroll a robot and start its agent) → §9 (open the
dashboard) → §5 (every robot after this one, unattended from a card).

§2 (typing an id by hand) is the M0 path, kept because it is what a robot with
no fleet server does; §7 supersedes it and adopts an already-set id rather than
renumbering it.

---

## 1. The overlay: one tailnet, no port forwarding

Every robot, the workstation, the fleet box and the GPU inference box join a
single [Tailscale](https://tailscale.com/) tailnet. WireGuard gives each of them
an encrypted, NAT-traversing link and a stable MagicDNS name, so "same LAN, same
`ROS_DOMAIN_ID`" becomes "same tailnet" and nothing is ever exposed to the public
internet.

### 1a. Creating the tailnet — the one-time account setup

Four things, all in the browser at
[login.tailscale.com](https://login.tailscale.com/admin), before any machine runs
`pixi run tailnet`. Ten minutes, once, for the whole fleet.

**1. Make the tailnet.** Sign up with whichever identity provider you already
use (Google / GitHub / Microsoft / Apple / email). Signing in *creates* the
tailnet — there is nothing to name or configure — and you get a tailnet domain
like `tail1a2b3.ts.net`. Everything below lives under that one account. The free
Personal plan covers a homelab fleet comfortably; check the current device and
user limits on the [pricing page](https://tailscale.com/pricing) at signup rather
than trusting a number written here, and note that the plan's *tagged*-device
allowance is the one that matters as robots multiply (see Cost & tradeoffs in
[`fleet.md`](../design/fleet.md)).

**2. Confirm MagicDNS is on.** Admin console → **DNS**. It is enabled by default
on new tailnets, and it is the thing that makes `ssh michael@mote-01` work — the
device's tailnet hostname becomes its name. Without it you are typing
`100.x.y.z` addresses everywhere.

**3. Declare the tags.** Admin console → **Access controls**, in the policy file.
Tags do not exist until an owner is declared for them, and `--advertise-tags` on
a machine fails with "requested tags are invalid or not permitted" if you skip
this:

```jsonc
{
  "tagOwners": {
    "tag:robot":     ["autogroup:admin"],
    "tag:fleet":     ["autogroup:admin"],
    "tag:inference": ["autogroup:admin"],
  },
  // The default policy already allows every device to reach every other, which
  // is what M0 wants. M7 replaces this with per-tag rules (operators reach
  // robots; robots reach the broker and their own inference box; robots cannot
  // reach each other).
}
```

**4. Mint an auth key per robot.** Admin console → **Settings → Keys → Generate
auth key**. For a robot:

| Option | Set it to | Why |
|---|---|---|
| Reusable | **off** | one key, one robot — a leaked key can enrol exactly nothing twice |
| Ephemeral | **off** | ephemeral nodes vanish when they go offline; a robot must persist |
| Expiration | **short** (a day is plenty) | it only has to survive from rendering the card to first boot |
| Tags | **`tag:robot`** | this is what makes the key mint a *tagged* device |

The key looks like `tskey-auth-…`; it goes into `pixi run provision --ts-authkey`
(or `pixi run tailnet --auth-key` on a machine you are sitting at). Keep the key's
tags and the `--role` you pass in agreement — a mismatch is rejected rather than
merged. The workstation needs no key at all: `pixi run tailnet --role workstation`
opens a browser to authenticate you.

**Why bother tagging the robots**, beyond ACLs: a *user* device's key expires
(180 days by default) and needs a human to re-authenticate it, which for a robot
means it silently drops off the tailnet one day months from now. Tagged devices
do not expire. That failure mode is the practical argument for `tag:robot`,
ahead of anything M7 does with it.

### 1b. Joining machines

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

The per-device cost curve at fleet scale, and the self-hosted escape hatch
(Headscale), are in the design doc.

---

## 2. Identity: `robot_id` is the fleet's primary key

> Since M1 the **server allocates the id** — see [§7](#7-enrolling-a-robot).
> `identity set` remains the way to give a robot an id with no fleet server in
> the picture, and enrollment adopts whatever it finds rather than renumbering.

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

M1's agent has since been measured at **exactly one slot**, as projected
([`m1-verification.md` §3](m1-verification.md)), so the budget is unchanged.

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

The template still writes an **operator-set** identity, as it did at M0. Now
that there is an enrollment endpoint, first boot should call `pixi run enroll`
with a token instead — a template change that is deliberately left until the
next real Pi provisioning, so the change and its verification land together
([`m1-verification.md` §5](m1-verification.md)). Nothing breaks meanwhile:
enrolling a robot that already has an id adopts that id.

---

## 6. The fleet server

Two processes on one always-on box — a VPS, a home server, or a spare Pi. Both
are ROS-free, so the box needs no robot software; the `fleet` pixi environment
carries nothing but a broker and Python.

```bash
pixi run -e fleet fleet-broker-ws                     # MQTT: 1883, WebSockets: 9001
pixi run -e fleet fleet-server -- --broker-host fleet-box   # API + UI, port 8080
```

**Why the broker runs in a container.** The dashboard (§9) subscribes to the
control plane straight from the browser, and a browser cannot speak raw MQTT —
it needs the broker's WebSocket listener. conda-forge's mosquitto is built
without one, so `fleet-broker-ws` runs `eclipse-mosquitto` under Docker with the
same `mosquitto.conf` this repo ships. `pixi run -e fleet fleet-broker` is still
there for a box with no Docker: robots and `fleetctl` work exactly as before,
and it tells you on startup that the dashboard will not. The reasoning and the
measurement are in [`m3-verification.md`](m3-verification.md) §1.

State lives in **`$MOTE_FLEET_HOME`** (default `~/.mote-fleet`) — the registry
database, the broker's retained messages, and the site bundles the dashboard
draws robots on. That is the server-side analogue of the robot's `MOTE_HOME`:
redeploying the server software replaces code around it and never the fleet's
memory of who is in it.

`--broker-host` is the address **robots** should dial, so on a tailnet it is the
fleet box's MagicDNS name (`fleet-box`), not `localhost` — it is handed out
verbatim in every enrollment answer. It defaults to the box's hostname.

**Security, plainly:** the broker is anonymous and the API's *read* routes are
unauthenticated. Dispatch is not — it needs an operator token (§8) — but that is
one credential on one path, not an auth story. It is proportionate only because
the tailnet is the boundary: WireGuard authenticates, and nothing here is
exposed to the internet. Do not put either on a network the robots are not
already trusted on. Per-robot broker credentials and operator auth everywhere
are M7; the shape of what changes is in
[`control-plane.md`](control-plane.md#security-posture-and-what-m7-changes).

To run it unattended, wrap the two commands in systemd units on that box. They
are deliberately *not* part of `pixi run setup`, which provisions robots.

---

## 7. Enrolling a robot

The server owns the id space. A robot presents a token and a hardware
fingerprint and is told who it is and where its broker lives.

```bash
# [fleet box] mint a token — single-use by default, one token per robot
pixi run fleetctl -- token new

# [robot] exchange it for an identity
pixi run enroll -- --server http://fleet-box:8080 --token tskey… --name Scout --site home
```

That writes both files the agent needs, and nothing else:

```yaml
# $MOTE_HOME/robot.yaml                # $MOTE_HOME/fleet.yaml
schema: 1                              schema: 1
id: mote-01                            server: http://fleet-box:8080
name: Scout                            broker:
site: home                               host: fleet-box
                                         port: 1883
```

Three properties make this safe to run unattended, or twice, or after a mistake:

- **Idempotent.** The registry keys on a stable hardware id (the Pi's SoC
  serial, else `/etc/machine-id`, else the MAC), so re-enrolling — after wiping
  `~/.mote`, after a failed attempt — returns the *same* robot, not a second
  one.
- **It adopts an existing id.** A robot with an M0 operator-set id offers it and
  the server records it. Upgrading a fleet to M1 registers it; it does not
  renumber it.
- **It refuses to re-key silently.** If the server answers with a different id
  from the one on disk, `enroll` writes nothing and says so. `--force` is the
  deliberate override.

Then start the bridge:

```bash
pixi run agent                                    # by hand
sudo systemctl enable --now mote-agent            # or as a service
```

The agent is *not* part of `pixi run robot` / `mapping`. It reports on the
mission and carries commands to it; folding it into bringup would mean a robot
that cannot reach the fleet server takes its own bringup down with it. It runs
alongside, like the health monitor. If it starts before the robot has enrolled,
it logs and retries — enrolling later brings it up without a restart.

---

## 8. Operating the fleet

```bash
# [fleet box] mint yourself an operator credential, once
pixi run -e fleet fleetctl -- operator new --name michael
export MOTE_FLEET_TOKEN=<that token>

pixi run fleetctl -- robots                             # the registry roster
pixi run fleetctl -- watch                              # live: presence, health, pose, status
pixi run fleetctl -- dispatch mote-01 fetch lab kitchen # send a task, follow it
pixi run fleetctl -- dispatch mote-01 goto kitchen
pixi run fleetctl -- audit                              # who dispatched what
```

**Dispatch goes through the fleet API, not to the broker.** Since M3 the API is
the single write path: it authorizes the operator token, writes an audit row,
and only then publishes to `task/command`. The topic tree did not change — only
who may publish to it — so `watch`, and the status half of `dispatch`, still
read straight from the broker with nothing in the middle. A token is minted
against the registry file while you are sitting on the fleet box, never over the
network; the **name on it is what the audit log records**, which is why an
unnamed one is refused. `fleetctl operator list|revoke` are the other two verbs,
and the route contract is [`fleet-api.md`](fleet-api.md).

`dispatch` exits 0 only if the task **succeeded**, so it composes into scripts.
The command grammar is the task layer's own, unchanged (`fetch <target>
<drop_zone>`, `goto <zone>` — see `mote_tasks`); the fleet adds no second
grammar.

What comes back is the task's transitions, tagged with the correlation id the
command went out with:

```console
-> mote-01: fetch lab kitchen  (id 3e99cf44d1294ab5)
2026-07-26T16:15:35.961Z  dispatched
2026-07-26T16:15:35.963Z  accepted
2026-07-26T16:15:38.305Z  succeeded
```

**One command at a time, per robot.** A second command sent while one is in
flight is rejected by the agent with the running command named — the robot never
sees two. Re-sending the *same* command id is safe and re-states its current
status rather than running it again. The full state machine, and why the agent
rather than the task layer enforces it, is in
[`control-plane.md`](control-plane.md#task-state-machine).

A task started **on the robot** (a `ros2 topic pub`, a bench script) appears in
`watch` too, tagged `source: local` with a null id — the fleet should see a
robot that is busy, whoever asked it to be.

Everything except commands is **retained**, so `watch` shows you the current
state of the fleet the instant it connects, with no polling and nothing replayed
on request. A robot that loses power is marked offline by the broker itself,
within the keepalive, via its Last Will — not after somebody notices the
heartbeats stopped.

---

## 9. The dashboard

`fleet-server` serves the operator view at `http://<fleet-box>:8080/`. It is the
fleet-wide picture — who is out there, where they are, what they are doing, and
sending one of them somewhere — and nothing else: the deep single-robot view
(3D, sensors, teleop) is Foxglove's job (M2), which each robot row deep-links to.

![The fleet dashboard](../images/fleet-ui.webp)

**Nothing on the page is polled.** The browser subscribes to
`mote/v1/+/{presence,health,pose,task/status}` over MQTT-over-WebSockets, and
because every one of those is retained, the whole fleet's current state is on
screen within a second of the page loading — no "wait for the next heartbeat",
no request/response loop, and no service between the broker and the browser.
That is the read path in [`fleet.md`](../design/fleet.md) Q5, and it is why the
broker needs the WebSocket listener from §6.

**Paste an operator token to dispatch.** Without one the page is read-only,
which is a perfectly good wall display. The token is kept in the browser's local
storage and sent to the fleet API as a bearer credential; the page holds **no
broker credential that can publish**, and its MQTT client implements no PUBLISH
packet at all.

**The map.** A floor's PNG basemap with live robot markers on it: pan by
dragging, zoom with the wheel, click a robot to select it, `follow` to keep the
selected one centred, `fit` to see the whole floor. The scale bar is metres.
Only robots on the *same* site and floor as the selected one are drawn — a pose
from another floor is a different map frame, and drawing it here would place a
robot somewhere it is not.

The server reads basemaps from `--maps-dir` (default `$MOTE_FLEET_HOME/sites`),
which is the **site bundle layout `sites.py` already writes**. Until M4 makes
the fleet server the canonical registry, seed it from a robot:

```bash
rsync -aL --delete michael@mote-01:~/.mote/sites/home ~/.mote-fleet/sites/
```

(`-L` because the published revision is reached through a symlink.) A robot with
no basemap on the server still appears in the roster with its health and its
task; the map pane says so rather than drawing an empty grid.

**What it does not do**, deliberately: no marker clustering, no basemap tiling,
no 3D, no camera, no teleop. The first two are what `fleet.md` Q5 describes for
large sites and would be unmeasured complexity at this fleet size; the last
three are Foxglove's half of the split.

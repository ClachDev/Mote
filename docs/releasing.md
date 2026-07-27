# Releasing Mote

Mote's first-party packages are published as versioned conda packages to the
[`prefix.dev/mote`](https://prefix.dev/channels/mote) channel. A robot runs an
environment resolved from that channel — released packages, no source checkout,
no `colcon`. This document is how a release is cut, and how a robot is moved
onto one.

## Two paths, and which to use

| | Dev loop | Release deploy |
| --- | --- | --- |
| Command | `pixi run sync` / `sync-watch` | `mote-update update <version>` |
| Moves | your working tree | a pinned, versioned package set |
| Robot needs | a checkout + build toolchain | pixi, and nothing else |
| Good for | iterating on one robot you are sitting next to | every robot you are not |
| Reversible | `git checkout` and re-sync | `mote-update rollback` |

`rsync` is still the right tool for the twenty-second edit-test loop, and
`--symlink-install` means launch/config/Python edits go live without a rebuild.
It is the wrong tool for *shipping*: it copies whatever your tree happens to
contain, to one host, with no version, no record, and no way back. Releases
replace it for deployment only.

## The version

**One semver version for all first-party packages**, bumped together. They are
co-developed and deployed as a set, so a robot runs "Mote 0.4.2" rather than a
combination of eight package versions that has to be reasoned about.

`[workspace] version` in `pixi.toml` is the source of truth. It is propagated to
each package's `package.xml` (which is what the built artifact carries) and
`setup.py`:

```bash
pixi run release-version show          # what are we on
pixi run release-version set 0.2.0     # bump everywhere
pixi run release-version check         # CI-friendly: do they agree
```

`check` runs as part of every release and in the release tests, because the one
failure mode worth automating away is publishing a package whose version does
not match the manifest that pins it.

Tag the commit `v<version>`. That tag matters at runtime: it is how a robot
fetches the deploy manifest for a version (below).

## What is released

Ten packages. Eight first-party, carrying the release version:

`ros-jazzy-mote-description`, `-hardware`, `-nav`, `-bringup`, `-perception`,
`-tasks`, `-arm`, `-fleet`

…and the two submodules under `third_party/`, at their own upstream versions:

`ros-jazzy-sllidar-ros2`, `ros-jazzy-kinematic-icp`

The submodules are not optional extras. A robot installing from the channel has
no source tree to build them from, and without them it has no lidar driver and
no odometry. They are built from **out-of-tree manifests** under
`release/third_party/`, because a `pixi.toml` added inside a submodule would be
an untracked file in someone else's repository, lost on the next clone.

**`mote_simulation` is deliberately excluded.** It is workstation-only, is
already excluded from `pixi run sync`, and is developed and run from a checkout —
there is no robot that wants it. It builds correctly if that ever changes; the
exclusion is policy, not a limitation.

## Cutting a release

```bash
pixi run release-version set 0.2.0
pixi run release                       # test -> manifest -> build -> verify
```

`release` is the whole dry run and publishes nothing:

1. **`test`** — the existing colcon suite.
2. **`release-manifest`** — regenerates `release/deploy/pixi.toml`.
3. **`release-build`** — builds all ten packages into `dist/channel`, a local
   *indexed* conda channel.
4. **`release-verify`** — renders the robot deploy manifest against
   `dist/channel` and **resolves it**. This is the step with teeth: it proves
   the release installs *as a set*, not merely that each package compiled.

Commit the regenerated `release/deploy/pixi.toml` and `release/evidence/`.

> `release-verify` is not ceremony. It is what caught `ros-jazzy-mote-fleet`
> declaring a dependency on `ros-jazzy-python3-paho-mqtt` — a package that does
> not exist, invented by the backend's fallback naming for a rosdep key
> RoboStack has no entry for. Every package built fine; the release was
> uninstallable.

### Architectures

Packages are **platform-specific** — nothing here is `noarch`, including the
Python ones — so each architecture is built natively:

| Platform | Where |
| --- | --- |
| `linux-aarch64` | the Pi's architecture; CI's `ubuntu-24.04-arm` runner |
| `linux-64` | workstation and sim; CI's `ubuntu-latest` runner |

Cross-compilation is not attempted. Native builds on both runners are cheaper
than making the C++ packages cross-compile, and CI already runs that matrix.
`release-verify` only checks the host architecture, because a local channel
holds only what was just built — the other half is verified by the same command
on the other runner.

## Publishing

The only outward-facing step, deliberately separate from `release` and never
part of it:

```bash
pixi run release-publish
```

It asks you to type the version to confirm, then uploads to
`https://prefix.dev/mote`. Publishing is irreversible in the way that matters: a
version someone has resolved cannot be taken out of their lockfile. Needs
`pixi auth login prefix.dev` once.

CI builds and verifies on both architectures but **does not publish** — the
upload is a human action.

## Updating a robot

A robot keeps versions in *slots* and points a `current` symlink at one:

```
~/mote-deploy/
  versions/0.1.0/     pixi.toml + pixi.lock + .pixi env
  versions/0.2.0/
  current -> versions/0.2.0
  previous            a file naming the rollback target
```

Two slots, not N: conda hard-links identical packages between environments, so a
point release costs disk only for what actually changed.

`$MOTE_HOME` (default `~/.mote`) — identity, sites and maps, calibration, bags —
is **outside the deploy root entirely** and is never read or written by an
update. That is what makes updates routine and rollback safe, and it is asserted
by `release/test/test_mote_update.py`, which runs a full
stage → cutover → rollback → prune cycle and checks `~/.mote` comes out
byte-identical.

### Bootstrapping a robot onto releases (once)

```bash
# pixi, if it isn't there already
curl -fsSL https://pixi.sh/install.sh | bash

mkdir -p ~/bin && curl -fsSL \
  https://raw.githubusercontent.com/ClachDev/Mote/v0.2.0/release/deploy/mote-update \
  -o ~/bin/mote-update && chmod +x ~/bin/mote-update

mote-update update 0.2.0

# one-time host setup (udev rules, wifi power save, systemd units)
pixi run --manifest-path ~/mote-deploy/current/pixi.toml setup
```

### Routine updates

```bash
mote-update status                 # what is installed, what is running
mote-update stage 0.3.0            # download and install; the robot keeps running
mote-update cutover 0.3.0          # stop, flip, restart, health-gate
mote-update rollback               # back to the previous slot
mote-update prune                  # drop everything but current + previous
```

`update` is `stage` then `cutover`. They are separate verbs on purpose: staging
is safe at any time and is the slow part, cutover is the brief outage.

**Cut over when the robot is idle, never mid-mission.** The Pi has neither the
CPU for two ROS stacks nor a second set of serial ports, so this is
stop-then-start, not a hot swap. "Install alongside" means two environments on
*disk*; only one ever runs.

A cutover:

1. Stops whichever of `mote-bringup`, `mote-health`, `mote-agent` were running —
   and only those, so a robot whose units are installed-but-not-enabled (the
   default) is not silently switched to autostart.
2. Flips `current` in a single `rename`, so it is never missing or dangling.
3. Reinstalls the systemd units from the new slot, pointed at the `current`
   symlink rather than at a slot — so the *next* cutover redirects them without
   reinstalling.
4. Waits for the units to come back (`--health-timeout`, default 90s) and
   **rolls back automatically** if they do not.

Rollback itself is not health-gated: it is the recovery path, and failing it
would leave nowhere to go. It reports and lets you look.

### Where the manifest comes from

`mote-update` needs the deploy manifest for the version. It resolves, in order:

1. `--manifest <path>` (offline, or testing an unreleased build)
2. `$MOTE_DEPLOY_MANIFEST`
3. `https://raw.githubusercontent.com/ClachDev/Mote/v<version>/release/deploy/pixi.toml`

The third is why the tag matters, and why the flow is headless: a fleet
orchestrator needs nothing but a version string. This is the OTA foundation the
fleet design assumes (`docs/design/fleet.md` §6).

The manifest's **dependencies are generated** from the workspace `pixi.toml` at
release time — a robot resolving a different set than the release was tested
against is not running the release. Its **tasks are hand-written** in
`release/deploy/pixi.toml.in`, because the deployed task surface is a curated
subset: no `build`, no `sync`, no sim. Task *names* match the workspace's, so a
runbook or a systemd unit works the same on a checkout and on a deploy — there
is a test for that.

## Notes on the build backend

- **`pixi-build` is a preview feature**, so every released package pins its
  backend in a small `pixi.toml` next to its `package.xml`. An unpinned backend
  is a moving target, and the backend has changed its config schema between
  minor versions.
- **The pin is `pixi-build-ros 0.5.0.*`**, because pixi 0.70.2 provides build
  API v4 and the 0.6.x backends require v5. Moving to 0.6.x means requiring a
  newer pixi; the payoff is a nicer `extra-package-mappings` syntax (an inline
  map rather than the list-of-tables the 0.5.x backend wants).
- **Builds are always `--clean`.** The backend's incremental-build hash covers
  `package.xml` and the sources but *not* `[package.build.config]`, so editing a
  dependency mapping and rebuilding silently reuses the cached recipe. That is a
  reproducibility hole, so release builds never reuse a build directory. (This
  cost an hour of confusion; it is why the note is here.)
- **Dependency names come from `package.xml` through RoboStack's table**, and
  anything unknown falls back to `ros-jazzy-<name>` — which is wrong for every
  non-ROS conda dependency. Those get an `extra-package-mappings` entry in the
  package's build manifest: `scservo_linux` → `scservo-linux` for
  `mote_hardware`, `python3-paho-mqtt` → `paho-mqtt` for `mote_fleet`. The
  mapping key must match the `package.xml` dependency string exactly.
- **`kinematic_icp` needs `EXTRA_CMAKE_ARGS=-DCMAKE_POLICY_VERSION_MINIMUM=3.5`**
  (its vendored tessil declares a pre-3.5 `cmake_minimum_required`) and pulls
  sources via CMake `FetchContent` at build time, so its build is not hermetic.
- **Tests do not run during a package build** (`BUILD_TESTING=OFF`). They run in
  `pixi run test` before anything is built, which is why `release` depends on
  `test`.

## Testing the release tooling

```bash
pixi run test-release
```

A ROS-free env, so it is fast. It checks that no first-party package is missing
from the release set, that backends stay pinned, that no build manifest
duplicates the version, that the committed deploy manifest is not stale, that
every task the systemd units invoke exists in a deploy, and that an update
cannot touch `~/.mote`.

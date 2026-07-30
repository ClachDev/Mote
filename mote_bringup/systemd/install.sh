#!/usr/bin/env bash
# Install the mote systemd services for the invoking user.
# Run via: pixi run install-systemd (uses sudo; @USER@/@HOME@/@REPO@ are
# filled in here).
set -euo pipefail

MOTE_USER="${SUDO_USER:-$USER}"
MOTE_HOME="$(getent passwd "$MOTE_USER" | cut -d: -f6)"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
# The checkout the units should run from: this script's own repo root, not a
# hardcoded ~/Mote. Installing from a second checkout (a git worktree, a staging
# clone) otherwise produces units pointing at a tree that need not contain the
# tasks they invoke, which fails ExecStartPre with status=127 and leaves the
# service restarting forever.
MOTE_REPO="$(cd "$SRC_DIR/../.." && pwd)"
echo "Repo: $MOTE_REPO"

for unit in "$SRC_DIR"/*.service; do
    sed "s|@USER@|$MOTE_USER|g; s|@HOME@|$MOTE_HOME|g; s|@REPO@|$MOTE_REPO|g" \
        "$unit" \
        | sudo tee "/etc/systemd/system/$(basename "$unit")" > /dev/null
done

# Bound the journal so the always-restarting services can never fill the disk.
sudo mkdir -p /etc/systemd/journald.conf.d
sudo cp "$SRC_DIR/journald-mote.conf" /etc/systemd/journald.conf.d/journald-mote.conf
sudo systemctl restart systemd-journald

sudo systemctl daemon-reload

# Installed but deliberately NOT enabled: starting the drive stack (and the
# recorder) on every boot drains the battery whenever the robot is just sitting
# on a desk, and the recorder's pruner trims older bags while it runs. Start a
# session by hand instead -- `pixi run robot` / `pixi run mapping`, which now
# include the health monitor -- and enable the units only for a robot meant to
# come up unattended:
#
#   sudo systemctl enable --now mote-bringup mote-health   # autostart at boot
#   sudo systemctl disable mote-bringup mote-health        # back to manual
#
# mote-agent and mote-foxglove are the exceptions worth enabling on their own:
# they draw nothing and drive nothing, and a robot that is not reporting to the
# fleet, or that an operator cannot look at, is a robot nobody can help. The
# agent needs `pixi run enroll` first; the bridge needs nothing.
# (docs/fleet/README.md)
#
#   sudo systemctl enable --now mote-agent mote-foxglove
#
# The robot's DDS graph is loopback-only (config/cyclonedds.xml, loaded by the
# units and by pixi activation alike), so no machine on the LAN can join it —
# mote-foxglove is the off-box window.
#
echo
echo "Units installed (not enabled). Start a session with 'pixi run robot'."
echo "For unattended boot:  sudo systemctl enable --now mote-bringup mote-health"
echo "To watch it remotely: sudo systemctl enable --now mote-foxglove"
echo "To join the fleet:    pixi run enroll ... && sudo systemctl enable --now mote-agent"

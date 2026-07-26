#!/usr/bin/env bash
# Install the mote systemd services for the invoking user.
# Run via: pixi run install-systemd (uses sudo; @USER@/@HOME@/@REPO@/@DDS_IFACE@
# are filled in here). Override the DDS interface with:
#   MOTE_DDS_INTERFACE=eth0 pixi run install-systemd
set -euo pipefail

MOTE_USER="${SUDO_USER:-$USER}"
MOTE_HOME="$(getent passwd "$MOTE_USER" | cut -d: -f6)"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
# The checkout the units should run from: this script's own repo root, NOT a
# hardcoded ~/Mote. Installing from a second checkout (a git worktree, a staging
# clone) otherwise produces units pointing at a tree that may not even contain
# the tasks they invoke — on the robot that gave a self-check ExecStartPre
# failing with status=127 ("task not found") and a permanent restart loop.
MOTE_REPO="$(cd "$SRC_DIR/../.." && pwd)"
echo "Repo: $MOTE_REPO"

# The interface the ROS graph should live on: the one carrying the default
# route. cyclonedds.xml treats it as optional, so a wrong guess degrades to a
# loopback-only graph rather than stopping the robot from booting.
DDS_IFACE="${MOTE_DDS_INTERFACE:-$(ip -o route show default | awk '{print $5; exit}')}"
DDS_IFACE="${DDS_IFACE:-wlan0}"
echo "DDS interface: $DDS_IFACE"

for unit in "$SRC_DIR"/*.service; do
    sed "s|@USER@|$MOTE_USER|g; s|@HOME@|$MOTE_HOME|g; s|@REPO@|$MOTE_REPO|g; \
         s|@DDS_IFACE@|$DDS_IFACE|g" "$unit" \
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
echo
echo "Units installed (not enabled). Start a session with 'pixi run robot'."
echo "For unattended boot: sudo systemctl enable --now mote-bringup mote-health"

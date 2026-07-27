#!/usr/bin/env bash
# Install the mote systemd services for the invoking user.
# Run via: pixi run install-systemd (uses sudo; @USER@/@HOME@/@REPO@/@DDS_CONFIG@
# /@DDS_IFACE@ are filled in here). Override the DDS interface with:
#   MOTE_DDS_INTERFACE=eth0 pixi run install-systemd
# Set MOTE_REPO to install units for a released deploy slot rather than for the
# checkout this script lives in (see below, and docs/releasing.md).
set -euo pipefail

MOTE_USER="${SUDO_USER:-$USER}"
MOTE_HOME="$(getent passwd "$MOTE_USER" | cut -d: -f6)"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
# The directory the units run from: the one holding the pixi manifest whose
# tasks they invoke. Two layouts:
#
#   source checkout -- this script's own repo root, not a hardcoded ~/Mote.
#     Installing from a second checkout (a git worktree, a staging clone)
#     otherwise produces units pointing at a tree that need not contain the
#     tasks they invoke, which fails ExecStartPre with status=127 and leaves
#     the service restarting forever.
#
#   released deploy -- the slot directory, which cannot be derived from SRC_DIR
#     because this script then lives inside the *environment*, under
#     share/mote_bringup. mote-update passes it in as MOTE_REPO, pointing at
#     the stable `current` symlink.
if [ -n "${MOTE_REPO:-}" ]; then
    MOTE_DEPLOYED=1
    MOTE_REPO="$(cd "$MOTE_REPO" && pwd)"
else
    MOTE_DEPLOYED=0
    MOTE_REPO="$(cd "$SRC_DIR/../.." && pwd)"
fi
echo "Repo: $MOTE_REPO"

# Where the units read cyclonedds.xml from. Deliberately expressed through
# $MOTE_REPO rather than as "$SRC_DIR/../config": in a deploy that keeps the
# path routed through the `current` symlink, so a cutover redirects every unit
# to the new slot's config instead of pinning them to the slot that happened to
# install them.
if [ "$MOTE_DEPLOYED" = "1" ]; then
    DDS_CONFIG="$MOTE_REPO/.pixi/envs/default/share/mote_bringup/config/cyclonedds.xml"
else
    DDS_CONFIG="$MOTE_REPO/mote_bringup/config/cyclonedds.xml"
fi
echo "DDS config: $DDS_CONFIG"

# The interface the ROS graph should live on: the one carrying the default
# route. cyclonedds.xml treats it as optional, so a wrong guess degrades to a
# loopback-only graph rather than stopping the robot from booting.
DDS_IFACE="${MOTE_DDS_INTERFACE:-$(ip -o route show default | awk '{print $5; exit}')}"
DDS_IFACE="${DDS_IFACE:-wlan0}"
echo "DDS interface: $DDS_IFACE"

for unit in "$SRC_DIR"/*.service; do
    sed "s|@USER@|$MOTE_USER|g; s|@HOME@|$MOTE_HOME|g; s|@REPO@|$MOTE_REPO|g; \
         s|@DDS_CONFIG@|$DDS_CONFIG|g; s|@DDS_IFACE@|$DDS_IFACE|g" "$unit" \
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
# mote-agent is the exception worth enabling on its own: it draws nothing and
# drives nothing, and a robot that is not reporting to the fleet is a robot the
# operator cannot see (it needs `pixi run enroll` first — docs/fleet/README.md).
#
#   sudo systemctl enable --now mote-agent
#
echo
echo "Units installed (not enabled). Start a session with 'pixi run robot'."
echo "For unattended boot: sudo systemctl enable --now mote-bringup mote-health"
echo "To join the fleet:   pixi run enroll ... && sudo systemctl enable --now mote-agent"

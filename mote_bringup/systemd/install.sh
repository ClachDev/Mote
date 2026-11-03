#!/usr/bin/env bash
# Install the mote systemd services for the invoking user.
# Run via: pixi run setup systemd (uses sudo; @USER@/@HOME@/@REPO@ are
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

# Units that once carried WantedBy=multi-user.target keep their boot-time
# symlinks after the [Install] section is gone, so a robot upgraded from that
# layout would still start the drive stack at boot. Drop those links (and the
# pre-template mote-nav.service) before installing.
for unit in mote-bringup mote-health mote-slam mote-record; do
    sudo systemctl disable "$unit.service" 2>/dev/null || true
done
if [ -e /etc/systemd/system/mote-nav.service ]; then
    sudo systemctl disable --now mote-nav.service 2>/dev/null || true
    sudo rm -f /etc/systemd/system/mote-nav.service
fi

render() {
    sed "s|@USER@|$MOTE_USER|g; s|@HOME@|$MOTE_HOME|g; s|@REPO@|$MOTE_REPO|g" "$1"
}

# mote-nav@.service reads nav-<instance>.env from the checkout ($MOTE_REPO), so
# the .env files are not copied.
for unit in "$SRC_DIR"/*.service "$SRC_DIR"/*.target; do
    render "$unit" | sudo tee "/etc/systemd/system/$(basename "$unit")" > /dev/null
done

# Lets the robot user start/stop/restart mote-* units without sudo (pixi run
# wake / sleep / map-mode). polkitd reloads its rules directory on its own.
sudo mkdir -p /etc/polkit-1/rules.d
render "$SRC_DIR/50-mote.rules" \
    | sudo install -m 0644 /dev/stdin /etc/polkit-1/rules.d/50-mote.rules

# Bound the journal so the always-restarting services can never fill the disk.
sudo mkdir -p /etc/systemd/journald.conf.d
sudo cp "$SRC_DIR/journald-mote.conf" /etc/systemd/journald.conf.d/journald-mote.conf
sudo systemctl restart systemd-journald

sudo systemctl daemon-reload

# Boot to standby: the fleet agent and the Foxglove bridge only. They draw
# nothing and drive nothing, and a robot that is not reporting to the fleet, or
# that an operator cannot look at, is a robot nobody can help.
#
# The mission stack is two targets, installed but NOT enabled: starting the
# drive stack on every boot drains the battery of a robot sitting on a desk.
#   mote-robot.target    pixi run wake      bringup, health, nav (AMCL), tasks
#   mote-mapping.target  pixi run map-mode  bringup, health, slam, nav, record
#   (both stopped)       pixi run sleep
# The two conflict, so starting one stops the other. A robot meant to drive
# unattended from boot: sudo systemctl enable mote-robot.target
#
# The agent needs an identity (pixi run fleet join, or provisioning) and exits
# without one, so it is enabled here only when one exists; `fleet join` enables
# it otherwise. The robot's DDS graph is loopback-only (config/cyclonedds.xml),
# so mote-foxglove is the off-box window.
sudo systemctl enable --now mote-foxglove.service
if [ -f "$MOTE_HOME/.mote/robot.yaml" ]; then
    sudo systemctl enable --now mote-agent.service
    AGENT_STATE="enabled"
else
    AGENT_STATE="not enabled: no identity at $MOTE_HOME/.mote/robot.yaml"
fi

echo
echo "Standby: mote-foxglove enabled; mote-agent $AGENT_STATE."
echo "Mission targets installed, not enabled:"
echo "  pixi run wake          mote-robot.target: drive the active map, take missions"
echo "  pixi run map-mode      mote-mapping.target: SLAM + mapping bag"
echo "  pixi run sleep         stop both, back to standby"
echo "  pixi run robot-status  every mote-* unit"
echo "Boot straight into the robot target:  sudo systemctl enable mote-robot.target"
echo "To join the fleet:  pixi run fleet join ...  (enables mote-agent)"

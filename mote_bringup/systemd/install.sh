#!/usr/bin/env bash
# Install the mote systemd services for the invoking user.
# Run via: pixi run install-systemd (uses sudo; @USER@/@HOME@ are filled in here).
set -euo pipefail

MOTE_USER="${SUDO_USER:-$USER}"
MOTE_HOME="$(getent passwd "$MOTE_USER" | cut -d: -f6)"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

for unit in "$SRC_DIR"/*.service; do
    sed "s|@USER@|$MOTE_USER|g; s|@HOME@|$MOTE_HOME|g" "$unit" \
        | sudo tee "/etc/systemd/system/$(basename "$unit")" > /dev/null
done

# Bound the journal so the always-restarting services can never fill the disk.
sudo mkdir -p /etc/systemd/journald.conf.d
sudo cp "$SRC_DIR/journald-mote.conf" /etc/systemd/journald.conf.d/journald-mote.conf
sudo systemctl restart systemd-journald

sudo systemctl daemon-reload
sudo systemctl enable mote-bringup mote-slam mote-nav mote-record mote-health

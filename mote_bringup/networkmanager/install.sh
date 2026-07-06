#!/usr/bin/env bash
# Install NetworkManager drop-in to disable WiFi power save.
# Run via: pixi run wifi-powersave
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
DEST="/etc/NetworkManager/conf.d/wifi-powersave.conf"

sudo mkdir -p /etc/NetworkManager/conf.d
sudo cp "$SRC_DIR/wifi-powersave.conf" "$DEST"

if systemctl is-active --quiet NetworkManager 2>/dev/null; then
    sudo systemctl reload NetworkManager
    echo "Installed $DEST and reloaded NetworkManager."
else
    echo "Installed $DEST (NetworkManager not active; reload skipped)."
fi

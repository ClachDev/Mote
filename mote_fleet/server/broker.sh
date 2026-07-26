#!/usr/bin/env bash
# Run the fleet's MQTT broker (pixi run fleet-broker).
#
# Mosquitto resolves persistence_file against its working directory, so this
# runs it from the fleet box's state root rather than from the checkout: the
# broker's retained state belongs with the registry database, not in a git tree
# that a redeploy replaces. Same split as the robot's MOTE_HOME.
set -euo pipefail

FLEET_HOME="${MOTE_FLEET_HOME:-$HOME/.mote-fleet}"
CONF="$(cd "$(dirname "$0")" && pwd)/mosquitto.conf"

# conda-forge's mosquitto package puts the *broker* in $PREFIX/sbin and only the
# clients (mosquitto_pub/_sub) in bin, and pixi only adds bin to PATH -- so a
# plain `mosquitto` is "command not found" in an environment that definitely has
# it. Look in sbin first, then fall back to PATH for a system install.
BROKER="${CONDA_PREFIX:-}/sbin/mosquitto"
if [ ! -x "$BROKER" ]; then
    BROKER="$(command -v mosquitto || true)"
fi
if [ -z "$BROKER" ]; then
    echo "no mosquitto broker found (looked in \$CONDA_PREFIX/sbin and PATH)" >&2
    exit 1
fi

mkdir -p "$FLEET_HOME"
echo "broker: $BROKER   state: $FLEET_HOME   config: $CONF"
cd "$FLEET_HOME"
exec "$BROKER" -c "$CONF" "$@"

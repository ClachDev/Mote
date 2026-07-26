#!/usr/bin/env bash
# Run the fleet's MQTT broker (pixi run fleet-broker).
#
# Mosquitto resolves persistence_file against its working directory, so this
# runs it from the fleet box's state root rather than from the checkout: the
# broker's retained state belongs with the registry database, not in a git tree
# that a redeploy replaces. Same split as the robot's MOTE_HOME.
#
# WEBSOCKETS. M3's dashboard subscribes to the control plane straight from the
# browser, which means the broker needs a `protocol websockets` listener -- and
# conda-forge's mosquitto is built without libwebsockets (measured: it refuses
# to start, docs/fleet/m1-verification.md S4). So there are two ways to run it:
#
#   pixi run fleet-broker       conda's mosquitto; MQTT only when that build has
#                               no websockets, and it says so on startup
#   pixi run fleet-broker-ws    mosquitto in a container, websockets and all --
#                               what the fleet box runs, and it needs no pixi
#                               environment at all
#
# The config is one file either way. This script strips the websockets stanza
# out of a copy when the local binary cannot honour it, rather than shipping two
# configs that drift.
set -euo pipefail

FLEET_HOME="${MOTE_FLEET_HOME:-$HOME/.mote-fleet}"
HERE="$(cd "$(dirname "$0")" && pwd)"
CONF="$HERE/mosquitto.conf"
IMAGE="${MOTE_BROKER_IMAGE:-eclipse-mosquitto:2}"
MODE=local

if [ "${1:-}" = "--docker" ]; then
    MODE=docker
    shift
fi

mkdir -p "$FLEET_HOME"

if [ "$MODE" = docker ]; then
    if ! command -v docker >/dev/null; then
        echo "no docker on PATH — install it, or run 'pixi run fleet-broker' for" >&2
        echo "an MQTT-only broker (no dashboard; see this script's header)" >&2
        exit 1
    fi
    # --network host: the broker must be reachable at the fleet box's own
    # address (its MagicDNS name is what enrollment hands to every robot), and
    # published ports would put a NAT between the two for no benefit.
    # --user: the image's mosquitto user cannot write a state directory owned by
    # whoever runs this, and the fleet's retained state should stay theirs.
    echo "broker: $IMAGE (docker)   state: $FLEET_HOME   config: $CONF"
    exec docker run --rm --name mote-broker --network host \
        --user "$(id -u):$(id -g)" \
        -v "$CONF:/mosquitto/config/mosquitto.conf:ro" \
        -v "$FLEET_HOME:/mosquitto/data" \
        -w /mosquitto/data \
        "$IMAGE" mosquitto -c /mosquitto/config/mosquitto.conf "$@"
fi

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

# Does this build have websockets? One built against libwebsockets links it; one
# that is not fails at config-parse time with "Websockets support not
# available", which is a worse way to find out.
RUNTIME_CONF="$CONF"
if ldd "$BROKER" 2>/dev/null | grep -q websockets; then
    echo "broker: $BROKER (websockets)   state: $FLEET_HOME   config: $CONF"
else
    RUNTIME_CONF="$FLEET_HOME/mosquitto.no-ws.conf"
    sed '/^# >>> websockets/,/^# <<< websockets/d' "$CONF" >"$RUNTIME_CONF"
    echo "broker: $BROKER (NO websockets)   state: $FLEET_HOME   config: $RUNTIME_CONF"
    echo "  the fleet dashboard needs MQTT-over-WebSockets and this build has" >&2
    echo "  none — run 'pixi run fleet-broker-ws' instead for the container" >&2
    echo "  broker. Robots, fleetctl and the fleet API are unaffected." >&2
fi

cd "$FLEET_HOME"
exec "$BROKER" -c "$RUNTIME_CONF" "$@"

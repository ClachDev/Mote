#!/usr/bin/env bash
# Join this machine to the Mote tailnet — the fleet overlay every other link
# rides on (docs/fleet/README.md).
#
# Run via: pixi run tailnet [--role robot|workstation|fleet|inference]
#                           [--auth-key tskey-... | --auth-key-file PATH]
#                           [--hostname NAME] [--ssh]
#
# Idempotent: `tailscale up` is declarative, so re-running with the same flags is
# a no-op. The robot's tailnet hostname is its robot_id (pixi run identity), so
# MagicDNS names and fleet identity can never drift apart.
set -euo pipefail

ROLE=""
AUTH_KEY=""
AUTH_KEY_FILE=""
HOSTNAME_ARG=""
SSH_FLAG=""

usage() {
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --role) ROLE="$2"; shift 2 ;;
        --auth-key) AUTH_KEY="$2"; shift 2 ;;
        --auth-key-file) AUTH_KEY_FILE="$2"; shift 2 ;;
        --hostname) HOSTNAME_ARG="$2"; shift 2 ;;
        --ssh) SSH_FLAG="--ssh"; shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown argument: $1" >&2; usage 1 ;;
    esac
done

MOTE_STATE="${MOTE_HOME:-$HOME/.mote}"
ROBOT_YAML="$MOTE_STATE/robot.yaml"

# Default the role from whether this machine has a robot identity.
if [ -z "$ROLE" ]; then
    if [ -f "$ROBOT_YAML" ]; then ROLE="robot"; else ROLE="workstation"; fi
fi

case "$ROLE" in
    # Tagged devices are owned by the tailnet, not by a user, so they survive the
    # operator's account and can be ACL'd as a class (M7).
    robot)       TAG="tag:robot" ;;
    fleet)       TAG="tag:fleet" ;;
    inference)   TAG="tag:inference" ;;
    workstation) TAG="" ;;
    *) echo "unknown role: $ROLE (robot|workstation|fleet|inference)" >&2; exit 1 ;;
esac

# A robot is addressed by its robot_id, so identity must exist first.
if [ -z "$HOSTNAME_ARG" ]; then
    if [ "$ROLE" = "robot" ]; then
        # Parsed rather than read through `identity`, because this runs at
        # provisioning time, before the workspace is necessarily built.
        HOSTNAME_ARG="$(sed -n 's/^id:[[:space:]]*//p' "$ROBOT_YAML" 2>/dev/null | tr -d '"'"'"' ' | head -1)"
        if [ -z "$HOSTNAME_ARG" ]; then
            echo "no robot id in $ROBOT_YAML — set one first:" >&2
            echo "    pixi run identity set --id mote-01 --name 'Front desk'" >&2
            exit 1
        fi
    else
        HOSTNAME_ARG="$(hostname -s)"
    fi
fi

if [ -n "$AUTH_KEY_FILE" ]; then
    AUTH_KEY="$(tr -d '[:space:]' < "$AUTH_KEY_FILE")"
fi

if ! command -v tailscale > /dev/null 2>&1; then
    echo "==> installing tailscale"
    curl -fsSL https://tailscale.com/install.sh | sh
fi

sudo systemctl enable --now tailscaled

UP_ARGS=(--hostname "$HOSTNAME_ARG" --accept-dns=true)
if [ -n "$TAG" ]; then UP_ARGS+=(--advertise-tags="$TAG"); fi
if [ -n "$SSH_FLAG" ]; then UP_ARGS+=("$SSH_FLAG"); fi
if [ -n "$AUTH_KEY" ]; then UP_ARGS+=(--auth-key "$AUTH_KEY"); fi

echo "==> tailscale up --hostname $HOSTNAME_ARG ${TAG:+--advertise-tags=$TAG} $SSH_FLAG"
sudo tailscale up "${UP_ARGS[@]}"

echo
echo "==> tailnet status"
sudo tailscale status || true
echo
echo "MagicDNS name: $HOSTNAME_ARG (reachable from any device on the tailnet)"
echo "Verify from another tailnet device:  tailscale ping $HOSTNAME_ARG"

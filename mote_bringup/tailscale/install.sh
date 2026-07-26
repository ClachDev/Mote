#!/usr/bin/env bash
# Join this machine to the Mote tailnet — the fleet overlay every other link
# rides on (docs/fleet/README.md).
#
# Run via: pixi run tailnet [--role ROLE[,ROLE...]] [--auth-key tskey-...]
#                           [--auth-key-file PATH] [--hostname NAME] [--ssh]
#                           [--dry-run]
#
# Roles are robot | workstation | fleet | inference. One machine may hold several
# (a home box that is both the fleet server and the GPU inference node is
# `--role fleet,inference`) — pass them together, because a machine is ONE tailnet
# node and `tailscale up` replaces the whole tag set, so running the script twice
# would silently drop the first role's tag.
#
# Idempotent: `tailscale up` is declarative, so re-running with the same flags is
# a no-op. The robot's tailnet hostname is its robot_id (pixi run identity), so
# MagicDNS names and fleet identity can never drift apart.
set -euo pipefail

ROLES=""
AUTH_KEY=""
AUTH_KEY_FILE=""
HOSTNAME_ARG=""
SSH_FLAG=""
DRY_RUN=""

usage() {
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --role) ROLES="${ROLES:+$ROLES,}$2"; shift 2 ;;
        --auth-key) AUTH_KEY="$2"; shift 2 ;;
        --auth-key-file) AUTH_KEY_FILE="$2"; shift 2 ;;
        --hostname) HOSTNAME_ARG="$2"; shift 2 ;;
        --ssh) SSH_FLAG="--ssh"; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown argument: $1" >&2; usage 1 ;;
    esac
done

MOTE_STATE="${MOTE_HOME:-$HOME/.mote}"
ROBOT_YAML="$MOTE_STATE/robot.yaml"

# Default the role from whether this machine has a robot identity.
if [ -z "$ROLES" ]; then
    if [ -f "$ROBOT_YAML" ]; then ROLES="robot"; else ROLES="workstation"; fi
fi

IS_ROBOT=""
IS_WORKSTATION=""
TAGS=""
for role in ${ROLES//,/ }; do
    case "$role" in
        # Tagged devices are owned by the tailnet, not by a user, so they survive
        # the operator's account and can be ACL'd as a class (M7).
        robot)     IS_ROBOT=1;      TAGS="${TAGS:+$TAGS,}tag:robot" ;;
        fleet)     TAGS="${TAGS:+$TAGS,}tag:fleet" ;;
        inference) TAGS="${TAGS:+$TAGS,}tag:inference" ;;
        # A user device: owned by your account, no tag.
        workstation) IS_WORKSTATION=1 ;;
        *) echo "unknown role: $role (robot|workstation|fleet|inference)" >&2; exit 1 ;;
    esac
done

# Tagging transfers the node from your account to the tailnet, so a machine
# cannot be both. Co-locating roles on your own desktop is fine — leave it a
# plain workstation and let M7's ACLs name the user rather than a tag.
if [ -n "$IS_WORKSTATION" ] && [ -n "$TAGS" ]; then
    echo "roles '$ROLES': 'workstation' is a user device and the others are" >&2
    echo "tailnet-owned tagged devices — a machine can only be one or the other." >&2
    echo "Pick the tagged roles (e.g. --role fleet,inference) to make it" >&2
    echo "infrastructure, or --role workstation to keep it yours." >&2
    exit 1
fi

TAG="$TAGS"

# A robot is addressed by its robot_id, so identity must exist first.
if [ -z "$HOSTNAME_ARG" ]; then
    if [ -n "$IS_ROBOT" ]; then
        # Parsed rather than read through `identity`, because this runs at
        # provisioning time, before the workspace is necessarily built. Guarded
        # rather than piped from a maybe-missing file: under `set -o pipefail`
        # that exits the script before the message below can explain itself.
        if [ -f "$ROBOT_YAML" ]; then
            HOSTNAME_ARG="$(sed -n 's/^id:[[:space:]]*//p' "$ROBOT_YAML" | tr -d '"'"'"' ' | head -1)"
        fi
        if [ -z "$HOSTNAME_ARG" ]; then
            echo "no robot id in $ROBOT_YAML — set one first:" >&2
            echo "    pixi run identity set --id mote-01 --name 'Scout'" >&2
            exit 1
        fi
    else
        HOSTNAME_ARG="$(hostname -s)"
    fi
fi

if [ -n "$AUTH_KEY_FILE" ]; then
    AUTH_KEY="$(tr -d '[:space:]' < "$AUTH_KEY_FILE")"
fi

# --dry-run resolves roles, tags and hostname and stops: what this machine would
# become, without installing anything or touching the tailnet.
if [ -n "$DRY_RUN" ]; then
    echo "roles:    $ROLES"
    echo "hostname: $HOSTNAME_ARG"
    echo "tags:     ${TAG:-<none> (user device)}"
    echo "would run: tailscale up --hostname $HOSTNAME_ARG --accept-dns=true" \
         "${TAG:+--advertise-tags=$TAG}" "$SSH_FLAG" "${AUTH_KEY:+--auth-key ***}"
    exit 0
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

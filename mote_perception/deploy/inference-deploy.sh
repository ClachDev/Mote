#!/usr/bin/env bash
# Deploy and update the Mote inference server, probe-gated, with rollback.
#
# The inference machine is not a development environment: it has an NVIDIA
# driver and a container runtime and nothing else (docs/inference-server.md).
# So this script is the *whole* pipeline that lives on that host -- one file,
# fetched once, no repo, no pixi, no python. It uses bash and docker and
# nothing else; every probe runs *inside the image being deployed*, which is
# the only place a probe can come from on a host that installs nothing.
#
#   curl -fsSLO https://raw.githubusercontent.com/ClachDev/Mote/main/mote_perception/deploy/inference-deploy.sh
#   chmod +x inference-deploy.sh
#   ./inference-deploy.sh up          # first deploy
#   ./inference-deploy.sh update      # pull, verify on a shadow port, cut over
#   ./inference-deploy.sh rollback    # back to the previous image
#   ./inference-deploy.sh status
#
# Why blue/green matters here even though the flip is not instantaneous: the
# failure this guards against is a *bad build* -- a broken model download, a
# CUDA/driver mismatch, a torch that faults on the first forward pass. The
# candidate is started on a shadow port and made to serve a real synthetic
# frame (tools/probe.py forces the on-demand model load) while the current
# version keeps serving the robots. A build that cannot infer never touches
# the served ports, and the update aborts with the old container still up.
#
# The cutover itself is a stop-then-start, not a load-balancer flip, and costs
# a few seconds of downtime plus the new container's first model load. The
# alternative -- keeping both alive and moving the robots to the new port --
# means editing perception.yaml on every robot and relaunching perception,
# which is a worse outage than the one it avoids. The robot's own fallback
# makes the gap a non-event: the depth/detect nodes treat "no server" as "skip
# this frame" and navigation runs on lidar (see the fallback matrix in
# docs/inference-server.md).
#
# Configuration is environment variables, so a host with a non-default setup
# writes them once into a wrapper or a systemd drop-in:
#
#   IMAGE       image repository            (ghcr.io/clachdev/mote-inference)
#   TAG         tag to deploy               (latest)
#   NAME        container name              (mote-inference)
#   GPUS        --gpus value, or "none"     (all)
#   BIND        publish address, e.g. a     (empty = every interface)
#               tailnet IP 100.x.y.z
#   DEPTH_PORT / DETECT_PORT                (5601 / 5602)
#   SHADOW_DEPTH_PORT / SHADOW_DETECT_PORT  (5611 / 5612)
#   SERVER_ARGS extra args for the servers  (empty; e.g. "--idle-timeout 0")
#   STATE_DIR   where the rollback pointer  (~/.mote-inference)
#               is kept
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/clachdev/mote-inference}"
TAG="${TAG:-latest}"
NAME="${NAME:-mote-inference}"
GPUS="${GPUS:-all}"
BIND="${BIND:-}"
DEPTH_PORT="${DEPTH_PORT:-5601}"
DETECT_PORT="${DETECT_PORT:-5602}"
SHADOW_DEPTH_PORT="${SHADOW_DEPTH_PORT:-5611}"
SHADOW_DETECT_PORT="${SHADOW_DETECT_PORT:-5612}"
SERVER_ARGS="${SERVER_ARGS:-}"
STATE_DIR="${STATE_DIR:-$HOME/.mote-inference}"
PROBE_WAIT="${PROBE_WAIT:-300}"

GREEN="${NAME}-green"
PREVIOUS_FILE="$STATE_DIR/previous"

say() { printf '\n== %s\n' "$*"; }
die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

command -v docker >/dev/null || die "docker is not installed"

gpu_args() {
    [ "$GPUS" = "none" ] || printf -- '--gpus %s' "$GPUS"
}

# Published-port arguments; BIND pins them to one interface (a tailnet address)
# rather than every interface on the host.
publish() {
    local depth="$1" detect="$2"
    if [ -n "$BIND" ]; then
        printf -- '-p %s:%s:5601 -p %s:%s:5602' "$BIND" "$depth" "$BIND" "$detect"
    else
        printf -- '-p %s:5601 -p %s:5602' "$depth" "$detect"
    fi
}

exists() { docker container inspect "$1" >/dev/null 2>&1; }

running_image() {
    # The image *id* the container is actually running, which is the artifact
    # to roll back to. A tag is not: `latest` has moved by the time an update
    # goes wrong, so rolling back to a tag can redeploy the broken build.
    docker container inspect --format '{{.Image}}' "$1" 2>/dev/null || true
}

image_version() {
    docker image inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null |
        sed -n 's/^MOTE_VERSION=//p' | head -1
}

start() {
    local name="$1" image="$2" depth="$3" detect="$4" restart="$5"
    # shellcheck disable=SC2046,SC2086  # word splitting is the point
    docker run -d --name "$name" --restart "$restart" $(gpu_args) \
        $(publish "$depth" "$detect") "$image" $SERVER_ARGS >/dev/null
}

# The gate: health *and* one real inference, from inside the container, so it
# needs nothing on the host and works identically on a gaming PC, a Linux box
# and a cloud instance.
probe() {
    local name="$1"
    shift
    docker exec "$name" python /app/tools/probe.py --wait "$PROBE_WAIT" "$@"
}

cmd_up() {
    if exists "$NAME"; then
        die "$NAME already exists -- use 'update' to move it to a new image"
    fi
    say "pulling $IMAGE:$TAG"
    # NO_PULL=1 deploys an image already on the box — a locally built one, or a
    # host with no registry access.
    [ "${NO_PULL:-0}" = "1" ] || docker pull "$IMAGE:$TAG"
    say "starting $NAME on ${DEPTH_PORT}/${DETECT_PORT}"
    start "$NAME" "$IMAGE:$TAG" "$DEPTH_PORT" "$DETECT_PORT" unless-stopped
    say "probing"
    if ! probe "$NAME"; then
        docker logs --tail 40 "$NAME" >&2 || true
        die "the new deployment did not serve a frame; it is left running for inspection"
    fi
    # No rollback pointer yet: there is nothing to go back to, and `rollback`
    # saying so is better than it restarting the version already running.
    say "up: $(image_version "$IMAGE:$TAG")"
}

cmd_update() {
    local ref="${1:-$IMAGE:$TAG}"
    exists "$NAME" || die "$NAME is not deployed -- run 'up' first"

    say "pulling $ref"
    [ "${NO_PULL:-0}" = "1" ] || docker pull "$ref"

    local previous
    previous="$(running_image "$NAME")"

    # ---- green: the candidate, on shadow ports, while blue keeps serving ----
    if exists "$GREEN"; then docker rm -f "$GREEN" >/dev/null; fi
    say "verifying candidate on shadow ports ${SHADOW_DEPTH_PORT}/${SHADOW_DETECT_PORT}"
    start "$GREEN" "$ref" "$SHADOW_DEPTH_PORT" "$SHADOW_DETECT_PORT" no
    if ! probe "$GREEN"; then
        docker logs --tail 40 "$GREEN" >&2 || true
        docker rm -f "$GREEN" >/dev/null
        die "candidate failed its probe; $NAME is untouched and still serving"
    fi
    # It has proven it can serve. Free its VRAM before the live one starts.
    docker rm -f "$GREEN" >/dev/null

    # The rollback pointer moves here and nowhere earlier: an update that is
    # abandoned on the shadow port has changed nothing, and must not overwrite
    # the target that a real rollback needs.
    mkdir -p "$STATE_DIR"
    printf '%s\n' "$previous" >"$PREVIOUS_FILE"

    # ---- flip: the verified image takes the served ports ----
    say "cutting over"
    docker rm -f "$NAME" >/dev/null
    start "$NAME" "$ref" "$DEPTH_PORT" "$DETECT_PORT" unless-stopped
    if probe "$NAME"; then
        say "updated: $(image_version "$ref")  (rollback target: ${previous:0:19})"
        return 0
    fi

    docker logs --tail 40 "$NAME" >&2 || true
    say "post-cutover probe failed -- rolling back"
    [ -n "$previous" ] || die "no previous image recorded; $NAME is left as-is"
    docker rm -f "$NAME" >/dev/null
    start "$NAME" "$previous" "$DEPTH_PORT" "$DETECT_PORT" unless-stopped
    probe "$NAME" || die "rollback also failed to serve -- the host needs a human"
    die "rolled back to $(image_version "$previous")"
}

cmd_rollback() {
    [ -s "$PREVIOUS_FILE" ] || die "no previous image recorded in $PREVIOUS_FILE"
    local previous
    previous="$(cat "$PREVIOUS_FILE")"
    say "rolling back to $(image_version "$previous") (${previous:0:19})"
    local current
    current="$(running_image "$NAME")"
    if exists "$NAME"; then docker rm -f "$NAME" >/dev/null; fi
    start "$NAME" "$previous" "$DEPTH_PORT" "$DETECT_PORT" unless-stopped
    probe "$NAME" || die "the rolled-back image did not serve; the host needs a human"
    # Rollback is itself reversible: the version we just left becomes the target.
    if [ -n "$current" ]; then printf '%s\n' "$current" >"$PREVIOUS_FILE"; fi
    say "rolled back"
}

cmd_status() {
    docker ps --filter "name=^/${NAME}$" \
        --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}'
    exists "$NAME" || die "$NAME is not deployed"
    printf '\nimage version: %s\n' "$(image_version "$(running_image "$NAME")")"
    if [ -s "$PREVIOUS_FILE" ]; then
        printf 'rollback target: %s\n' "$(image_version "$(cat "$PREVIOUS_FILE")")"
    fi
    # --no-infer: status must not wake a released model and pin VRAM on a box
    # somebody is using; `update` is where the real frame is served.
    probe "$NAME" --no-infer
}

usage() {
    cat <<'EOF'
inference-deploy.sh -- deploy and update the Mote inference server.

  up                deploy for the first time (pull, run, probe)
  update [image]    pull, verify the candidate on a shadow port, cut over,
                    and roll back automatically if the new one cannot serve
  rollback          go back to the previously deployed image
  status            what is running, its version, and a health probe
  logs [n]          follow the container's log

Configuration is environment variables (defaults in brackets):
  IMAGE [ghcr.io/clachdev/mote-inference]  TAG [latest]  NAME [mote-inference]
  GPUS [all] (or "none")   BIND [] (publish address, e.g. a tailnet IP)
  DEPTH_PORT [5601]  DETECT_PORT [5602]
  SHADOW_DEPTH_PORT [5611]  SHADOW_DETECT_PORT [5612]
  SERVER_ARGS []  (e.g. "--idle-timeout 0")   STATE_DIR [~/.mote-inference]
  PROBE_WAIT [300]  NO_PULL [0]

Full documentation: docs/inference-server.md
EOF
    exit "${1:-0}"
}

case "${1:-}" in
    up) cmd_up ;;
    update) cmd_update "${2:-}" ;;
    rollback) cmd_rollback ;;
    status) cmd_status ;;
    logs) docker logs -f --tail "${2:-100}" "$NAME" ;;
    -h | --help | help) usage ;;
    *) usage 2 ;;
esac

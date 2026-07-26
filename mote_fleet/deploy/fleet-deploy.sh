#!/usr/bin/env bash
# Deploy, update, back up and restore the fleet server (broker + registry API).
#
# The fleet box is ordinary server infrastructure, so this is an ordinary
# container deploy: docker compose is the mechanism and docker-compose.yml +
# .env are the declared state. What this script adds on top is the three things
# a bare `docker compose up -d` does not do — a health gate that actually
# checks the API answered, an automatic rollback to the image that was running
# when it does not, and a consistent backup of the two pieces of state that a
# rebuilt box needs (the registry database and the broker's retained messages).
#
#   ./fleet-deploy.sh up [--build]     first deploy (or converge to .env)
#   ./fleet-deploy.sh update [ref]     pull, restart, health-gate, auto-rollback
#   ./fleet-deploy.sh rollback         back to the previous image
#   ./fleet-deploy.sh status
#   ./fleet-deploy.sh backup [dir]     consistent snapshot -> a .tgz
#   ./fleet-deploy.sh restore <file>   restore one into the volumes
#   ./fleet-deploy.sh fleetctl ...     run the operator CLI in the container
#
# Why not blue/green like the inference server: this box holds state. Two
# servers on one SQLite file is a correctness problem, not a capacity win, and
# the broker's value *is* its connections and retained messages. The update is
# therefore a recreate — seconds of downtime, gated and reversible — which is
# affordable precisely because a fleet server outage does not touch robot
# autonomy: a robot keeps executing its mission and navigating locally, and its
# agent reconnects with backoff (mote_fleet/test/test_fleet_outage.py measures
# exactly this).
#
# Run it from the directory that holds docker-compose.yml and .env.
set -euo pipefail

cd "$(dirname "$0")"

usage() {
    cat <<'EOF'
fleet-deploy.sh -- deploy and update the Mote fleet server (broker + registry API).

  up [--build]      first deploy, or converge to what .env declares
  update [ref]      pull, recreate, health-gate, and roll back if it fails
  rollback          back to the previous image
  status            what is running, plus /healthz
  backup [dir]      consistent snapshot of the registry + broker state
  restore <file>    restore one into the volumes (stops the stack first)
  fleetctl ...      run the operator CLI where the registry lives
  logs [service]    follow the logs

Configuration is .env beside this script (start from env.example); BROKER_HOST
is required. Overridable at the command line: HEALTH_TIMEOUT [90], NO_PULL [0],
YES [0] (skip the restore confirmation), IMAGE_REPO.

Full documentation: docs/fleet/server-pipelines.md
EOF
    exit "${1:-0}"
}

case "${1:-}" in -h | --help | help) usage ;; esac

[ -f .env ] || {
    echo "no .env here -- copy env.example to .env and set BROKER_HOST" >&2
    exit 1
}
# shellcheck disable=SC1091
set -a && . ./.env && set +a

IMAGE_REPO="${IMAGE_REPO:-ghcr.io/clachdev/mote-fleet}"
FLEET_PORT="${FLEET_PORT:-8080}"
BROKER_PORT="${BROKER_PORT:-1883}"
HEALTH_TIMEOUT="${HEALTH_TIMEOUT:-90}"
COMPOSE=(docker compose)

say() { printf '\n== %s\n' "$*"; }
die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

command -v docker >/dev/null || die "docker is not installed"
docker compose version >/dev/null 2>&1 || die "docker compose v2 is required"

# ---------------------------------------------------------------- helpers ---

# An HTTP GET with no curl, wget or python on the host: the fleet box is
# allowed to have nothing but docker.
http_get() {
    local host="$1" port="$2" path="$3"
    # The braces matter: a failed /dev/tcp redirection reports through the
    # shell, not the command, so without them bash prints "connection refused"
    # on every poll of a server that is still starting.
    { exec 3<>"/dev/tcp/$host/$port"; } 2>/dev/null || return 1
    printf 'GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n' "$path" "$host" >&3
    # The server closes as soon as it has answered (Connection: close), which
    # makes cat report a reset on some kernels; the body is already complete.
    cat <&3 2>/dev/null
    exec 3<&-
}

tcp_open() { { (exec 3<>"/dev/tcp/$1/$2"); } 2>/dev/null; }

# What to write into .env when rolling back. The digest is preferred because it
# is immutable and pullable — a box rebuilt from this .env alone gets exactly
# the image that was rolled back to, not whatever the tag points at by then.
# A locally built image has no digest, so the local :previous tag stands in.
previous_ref() {
    local digest
    digest="$(docker image inspect --format '{{if .RepoDigests}}{{index .RepoDigests 0}}{{end}}' \
        "$IMAGE_REPO:previous" 2>/dev/null || true)"
    printf '%s\n' "${digest:-$IMAGE_REPO:previous}"
}

# The gate. Container health only says the process answered *itself*; this
# checks the published port an enrolling robot actually reaches, and that the
# broker's listener is up too.
health_gate() {
    local deadline=$((SECONDS + HEALTH_TIMEOUT)) body=""
    while [ "$SECONDS" -lt "$deadline" ]; do
        body="$(http_get 127.0.0.1 "$FLEET_PORT" /healthz || true)"
        if [[ "$body" == *'"ok": true'* ]] && tcp_open 127.0.0.1 "$BROKER_PORT"; then
            printf '%s\n' "${body##*$'\r\n\r\n'}"
            return 0
        fi
        sleep 2
    done
    printf 'health gate timed out after %ss\n' "$HEALTH_TIMEOUT" >&2
    [ -n "$body" ] && printf '%s\n' "$body" >&2
    return 1
}

running_image() { docker compose ps -q server | xargs -r docker inspect --format '{{.Image}}'; }

container_of() {
    local id
    id="$("${COMPOSE[@]}" ps -q "$1")"
    [ -n "$id" ] || die "the $1 container is not running"
    printf '%s\n' "$id"
}

# .env is the declared state, so an update or a rollback rewrites it. Otherwise
# the next `docker compose up` on that box would silently undo what this script
# just decided.
pin_ref() {
    local ref="$1"
    if grep -q '^MOTE_FLEET_REF=' .env; then
        sed -i "s|^MOTE_FLEET_REF=.*|MOTE_FLEET_REF=$ref|" .env
    else
        printf 'MOTE_FLEET_REF=%s\n' "$ref" >>.env
    fi
    export MOTE_FLEET_REF="$ref"
}

# ---------------------------------------------------------------- commands ---

cmd_up() {
    local build=()
    [ "${1:-}" = "--build" ] && build=(--build)
    say "starting the fleet server (broker: ${BROKER_HOST:-unset})"
    "${COMPOSE[@]}" up -d "${build[@]}"
    say "health gate"
    health_gate || die "the stack did not come up healthy -- see 'docker compose logs'"
    say "up"
}

cmd_update() {
    local ref="${1:-${MOTE_FLEET_REF:-$IMAGE_REPO:latest}}" previous
    previous="$(running_image)" || true
    [ -n "$previous" ] || die "nothing is running -- use 'up' first"

    # Tag the running image locally before pulling: `latest` will have moved by
    # the time an update goes wrong, so a tag is not a rollback target.
    docker tag "$previous" "$IMAGE_REPO:previous"

    say "pulling $ref"
    [ "${NO_PULL:-0}" = "1" ] || docker pull "$ref"
    pin_ref "$ref"

    say "recreating"
    "${COMPOSE[@]}" up -d
    if health_gate; then
        say "updated to $ref  (rollback: ./fleet-deploy.sh rollback)"
        return 0
    fi

    "${COMPOSE[@]}" logs --tail 40 server >&2 || true
    say "health gate failed -- rolling back to the previous image"
    pin_ref "$(previous_ref)"
    "${COMPOSE[@]}" up -d
    health_gate >/dev/null || die "rollback did not come up healthy either -- the box needs a human"
    die "rolled back; the new image was not deployed"
}

cmd_rollback() {
    docker image inspect "$IMAGE_REPO:previous" >/dev/null 2>&1 ||
        die "no previous image recorded (nothing to roll back to)"
    local current
    current="$(running_image)" || true
    say "rolling back"
    pin_ref "$(previous_ref)"
    "${COMPOSE[@]}" up -d
    health_gate || die "the rolled-back stack is not healthy -- the box needs a human"
    # Rollback is reversible: what we just left becomes the new target.
    [ -n "$current" ] && docker tag "$current" "$IMAGE_REPO:previous"
    say "rolled back"
}

cmd_status() {
    "${COMPOSE[@]}" ps
    say "healthz"
    health_gate || die "the API is not answering on port $FLEET_PORT"
}

# Both volumes in one archive: the registry rows, the site bundles the UI serves
# basemaps from, and the broker's retained state are what a rebuilt box needs to
# be the same fleet rather than a new one.
cmd_backup() {
    local dir="${1:-.}" stamp name
    stamp="$(date -u +%Y%m%dT%H%M%SZ)"
    name="mote-fleet-$stamp.tgz"
    mkdir -p "$dir"
    dir="$(cd "$dir" && pwd)"

    docker run --rm --user 0:0 \
        --volumes-from "$(container_of server)" \
        --volumes-from "$(container_of broker)" \
        -v "$dir:/backup" --entrypoint sh "$(running_image)" -c "
        set -e
        mkdir -p /tmp/b/fleet-state /tmp/b/broker-data
        # Everything under the state root -- notably sites/, where the basemaps
        # the dashboard draws live until M4 makes the registry their source.
        cp -a /var/lib/mote-fleet/. /tmp/b/fleet-state/ 2>/dev/null || true
        # ...then overwrite the database copy with a consistent one. sqlite3's
        # online backup API, not cp: the server is still writing to this file,
        # and a half-written page is not a registry.
        if [ -f /var/lib/mote-fleet/registry.db ]; then
          python - <<'PY'
import sqlite3
src = sqlite3.connect('/var/lib/mote-fleet/registry.db')
dst = sqlite3.connect('/tmp/b/fleet-state/registry.db')
src.backup(dst)
dst.close(); src.close()
PY
        fi
        cp -a /mosquitto/data/. /tmp/b/broker-data/ 2>/dev/null || true
        tar czf /backup/$name -C /tmp/b .
        # The tar had to run as root to read both volumes; the archive should
        # belong to whoever asked for the backup, not to root.
        chown $(id -u):$(id -g) /backup/$name
    "
    say "wrote $dir/$name"
    ls -lh "$dir/$name"
}

cmd_restore() {
    local archive="${1:-}"
    [ -n "$archive" ] && [ -f "$archive" ] || die "usage: fleet-deploy.sh restore <archive.tgz>"
    if [ "${YES:-0}" != "1" ]; then
        printf 'This replaces the registry and the broker state. Type yes to continue: '
        read -r reply
        [ "$reply" = "yes" ] || die "aborted"
    fi
    local dir base image
    dir="$(cd "$(dirname "$archive")" && pwd)"
    base="$(basename "$archive")"
    image="$(running_image)"
    [ -n "$image" ] || image="${MOTE_FLEET_REF:-$IMAGE_REPO:latest}"

    say "stopping the stack"
    # The broker writes its persistence file on shutdown, so restoring under a
    # running broker would be overwritten the moment it stops.
    local server_c broker_c
    server_c="$(container_of server)"
    broker_c="$(container_of broker)"
    "${COMPOSE[@]}" stop

    docker run --rm --user 0:0 \
        --volumes-from "$server_c" --volumes-from "$broker_c" \
        -v "$dir:/backup:ro" --entrypoint sh "$image" -c "
        set -e
        mkdir -p /tmp/r && tar xzf /backup/$base -C /tmp/r
        if [ -d /tmp/r/fleet-state ]; then
          cp -a /tmp/r/fleet-state/. /var/lib/mote-fleet/
          chown -R 10001:10001 /var/lib/mote-fleet
        fi
        rm -rf /mosquitto/data/* 2>/dev/null || true
        cp -a /tmp/r/broker-data/. /mosquitto/data/ 2>/dev/null || true
        chown -R 1883:1883 /mosquitto/data
    "
    say "starting the stack"
    "${COMPOSE[@]}" start
    health_gate || die "restored, but the stack is not healthy"
    say "restored from $base"
}

case "${1:-}" in
    up) cmd_up "${2:-}" ;;
    update) cmd_update "${2:-}" ;;
    rollback) cmd_rollback ;;
    status) cmd_status ;;
    backup) cmd_backup "${2:-}" ;;
    restore) cmd_restore "${2:-}" ;;
    # The operator CLI needs the registry file, which lives in the container's
    # volume: `fleet-deploy.sh fleetctl token new` is how the fleet box mints
    # an enrollment token.
    fleetctl)
        shift
        "${COMPOSE[@]}" exec -T server python /app/server/fleetctl.py "$@"
        ;;
    logs)
        shift
        "${COMPOSE[@]}" logs -f --tail 100 "$@"
        ;;
    -h | --help | help) usage ;;
    *) usage 2 ;;
esac

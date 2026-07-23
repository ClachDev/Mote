#!/usr/bin/env bash
# Chaos test: kill critical ROS nodes on the running robot and verify each is
# relaunched within a bounded time. Run ON THE ROBOT (auldbot) while the stack
# is up (systemd services active, or `pixi run mapping`/`robot` running):
#
#   pixi run chaos            # kills nodes, logs recovery to chaos_log.txt
#
# Recovery is expected because the launch files mark the driver / nav2 nodes
# respawn=True (see mote_launch.py / nav2_launch.py); this proves it end to end.
#
# Safety: nodes are matched by their executable name (e.g. ros2_control_node),
# which never matches this bash script, and this script's own PID is excluded —
# so the `pkill -f from an agent shell matches itself` foot-gun cannot fire.
set -uo pipefail

BOUND_S=30          # max seconds allowed for a node to reappear
POLL_S=0.5
SELF=$$
LOG="$(cd "$(dirname "$0")" && pwd)/chaos_log.txt"

# Executable names to knock over. These are the process names, not the launch
# nodes' remapped names, so they are stable to match on.
TARGETS=(
    "ros2_control_node"    # controller_manager — drive control
    "sllidar_node"         # lidar driver — /scan
    "controller_server"    # nav2 local controller
)

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$LOG"; }

pids_for() { pgrep -f "$1" | grep -vw "$SELF" || true; }

kill_target() {
    local pat="$1" pid
    for pid in $(pids_for "$pat"); do
        kill -9 "$pid" 2>/dev/null
    done
}

wait_recovery() {
    local pat="$1" waited=0
    while (( $(echo "$waited < $BOUND_S" | bc -l) )); do
        if [ -n "$(pids_for "$pat")" ]; then
            echo "$waited"
            return 0
        fi
        sleep "$POLL_S"
        waited=$(echo "$waited + $POLL_S" | bc -l)
    done
    echo "$waited"
    return 1
}

: > "$LOG"
log "=== mote chaos restart test on $(hostname) ==="
log "bound=${BOUND_S}s, targets: ${TARGETS[*]}"

# Precondition: the stack must be up, or there is nothing to knock over.
missing=0
for t in "${TARGETS[@]}"; do
    [ -z "$(pids_for "$t")" ] && { log "PRECONDITION: '$t' not running"; missing=1; }
done
if [ "$missing" -ne 0 ]; then
    log "ABORT: run this on the robot with the stack up (services active or"
    log "       'pixi run robot' / 'pixi run mapping' running). Nothing killed."
    exit 2
fi

fails=0
for t in "${TARGETS[@]}"; do
    before=$(pids_for "$t" | tr '\n' ' ')
    log "killing '$t' (pids: $before)"
    kill_target "$t"
    sleep 1
    if elapsed=$(wait_recovery "$t"); then
        after=$(pids_for "$t" | tr '\n' ' ')
        log "PASS '$t' recovered in ~${elapsed}s (pids: $after)"
    else
        log "FAIL '$t' did NOT recover within ${BOUND_S}s"
        fails=$((fails + 1))
    fi
    sleep 3   # let it settle before the next scenario
done

log "=== done: $((${#TARGETS[@]} - fails))/${#TARGETS[@]} recovered ==="
exit "$fails"

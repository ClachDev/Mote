#!/usr/bin/env bash
# Hardware-free proof that the hardened restart policy recovers a killed
# service within a bounded time. Uses a transient --user unit that mirrors the
# mote services' Restart settings (Restart=always + RestartSec backoff), so it
# runs on any workstation with a user systemd — no ROS, no robot.
#
# It is the local, committable half of the chaos validation: chaos_restart.sh
# proves per-node respawn on the real robot; this proves the systemd layer.
#
#   bash mote_bringup/test/chaos/chaos_policy_demo.sh
set -uo pipefail

UNIT="mote-chaos-demo"
BOUND_S=15
LOG="${CHAOS_LOG:-/tmp/mote_chaos_policy_log.txt}"

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$LOG"; }

if ! systemctl --user show-environment >/dev/null 2>&1; then
    log "SKIP: no user systemd manager available"
    exit 0
fi

: > "$LOG"
log "=== systemd restart-policy demo (mirrors mote-*.service) ==="

systemctl --user reset-failed "$UNIT" 2>/dev/null || true

# A trivial long-running payload that writes its start epoch to a marker file so
# we can prove the process identity changed (a real restart, not a survivor).
MARK="$(mktemp)"
systemd-run --user --unit="$UNIT" \
    --property=Restart=always \
    --property=RestartSec=2 \
    --property=RestartSteps=5 \
    --property=RestartMaxDelaySec=30 \
    --property=StartLimitIntervalSec=0 \
    /usr/bin/env bash -c "echo \$\$ > '$MARK'; exec sleep 3600" >/dev/null 2>&1

sleep 2
pid1=$(systemctl --user show -p MainPID --value "$UNIT")
log "started $UNIT, MainPID=$pid1"

log "killing MainPID $pid1 (SIGKILL)"
kill -9 "$pid1" 2>/dev/null

waited_ms=0
recovered=0
while [ "$waited_ms" -lt $((BOUND_S * 1000)) ]; do
    sleep 0.5
    waited_ms=$((waited_ms + 500))
    waited=$(printf '%d.%d' $((waited_ms / 1000)) $((waited_ms % 1000 / 100)))
    state=$(systemctl --user show -p ActiveState --value "$UNIT")
    pid2=$(systemctl --user show -p MainPID --value "$UNIT")
    if [ "$state" = "active" ] && [ -n "$pid2" ] && [ "$pid2" != "0" ] \
        && [ "$pid2" != "$pid1" ]; then
        log "PASS: $UNIT restarted in ~${waited}s, new MainPID=$pid2"
        recovered=1
        break
    fi
done

if [ "$recovered" -ne 1 ]; then
    log "FAIL: $UNIT did not restart within ${BOUND_S}s"
fi

systemctl --user stop "$UNIT" 2>/dev/null || true
systemctl --user reset-failed "$UNIT" 2>/dev/null || true
rm -f "$MARK"

log "=== done ==="
[ "$recovered" -eq 1 ]

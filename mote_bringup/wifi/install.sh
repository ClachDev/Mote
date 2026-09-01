#!/usr/bin/env bash
# Install the robot's wifi configuration.
#
#   pixi run wifi-powersave   # NetworkManager power-save drop-in only
#   pixi run wifi-roaming     # let the firmware roam (see README.md)
#   pixi run setup            # both, plus udev and systemd
#
# The roaming half writes one modprobe option and takes effect at the next
# reboot. It restarts NetworkManager only on a robot still carrying the iwd
# backend an earlier version of this branch installed, since that has to come
# back off. The robot's only link is its wifi, so that step runs detached with a
# guard: if the robot is not back on the network within --guard-timeout seconds,
# the guard puts iwd back. Read the verdict in /var/log/mote-wifi-install.log.
set -euo pipefail
# modprobe, ip and nmcli live in sbin, which a pixi shell does not carry.
export PATH="$PATH:/usr/sbin:/sbin"

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG=/var/log/mote-wifi-install.log
NM_DIR=/etc/NetworkManager/conf.d
POWERSAVE_DEST="$NM_DIR/wifi-powersave.conf"
BACKEND_DEST="$NM_DIR/wifi-backend-iwd.conf"
MODPROBE_DEST=/etc/modprobe.d/zz-mote-brcmfmac.conf
STALE_MODPROBE=/etc/modprobe.d/99-mote-brcmfmac.conf
IWD_CONF=/etc/iwd/main.conf

MODE=all
GUARD_TIMEOUT=120
GUARD=1

log() { echo "[$(date -Is)] $*"; }

install_powersave() {
    sudo mkdir -p "$NM_DIR"
    sudo cp "$SRC_DIR/wifi-powersave.conf" "$POWERSAVE_DEST"
    log "installed $POWERSAVE_DEST"
    if systemctl is-active --quiet NetworkManager 2>/dev/null; then
        sudo systemctl reload NetworkManager
        log "reloaded NetworkManager"
    else
        log "NetworkManager not active; reload skipped"
    fi
}

install_roam_option() {
    sudo install -D -m 0644 "$SRC_DIR/brcmfmac-roam.conf" "$MODPROBE_DEST"
    log "installed $MODPROBE_DEST (takes effect at next reboot)"

    # An earlier version of this file wrote roamoff=1 under a name that sorts
    # before the vendor's. Leaving it costs nothing today and misleads whoever
    # next reads `modprobe -c`.
    if [ -f "$STALE_MODPROBE" ]; then
        sudo rm -f "$STALE_MODPROBE"
        log "removed $STALE_MODPROBE (superseded)"
    fi

    local effective
    effective=$(modprobe -c 2>/dev/null | grep '^options brcmfmac' | tail -1)
    log "modprobe's last word on brcmfmac: ${effective:-none}"
    case "$effective" in
        *roamoff=0*) ;;
        *) log "WARNING: another modprobe.d file sorts after $(basename "$MODPROBE_DEST") and overrides roamoff" ;;
    esac
}

# True when this robot is still running the iwd backend. Nothing else in this
# script depends on iwd; a Pi that never had it skips the whole guarded step and
# so never restarts NetworkManager.
has_iwd_backend() {
    [ -f "$BACKEND_DEST" ] || systemctl is-enabled --quiet iwd 2>/dev/null
}

# Everything below runs detached and as root: it is the part that can take the
# network down, so it must outlive the ssh session it was started from.
guarded_revert() {
    local timeout="$1" deadline gw
    log "handing the wifi backend back to wpa_supplicant"

    [ -f "$BACKEND_DEST" ] && mv "$BACKEND_DEST" "$BACKEND_DEST.mote-bak"
    systemctl enable --now wpa_supplicant || true
    systemctl disable --now iwd || true
    systemctl restart NetworkManager

    deadline=$(( $(date +%s) + timeout ))
    while [ "$(date +%s)" -lt "$deadline" ]; do
        sleep 5
        [ "$(nmcli -t -f STATE general 2>/dev/null)" = "connected" ] || continue
        gw=$(ip route show default 2>/dev/null | awk '/default/ {print $3; exit}')
        [ -n "$gw" ] || continue
        if ping -c1 -W2 "$gw" >/dev/null 2>&1; then
            log "OK: connected via wpa_supplicant, gateway $gw reachable"
            # Only now is it safe to drop what a rollback would have needed: the
            # copy of the wifi key iwd was given, and its config.
            rm -f "$BACKEND_DEST.mote-bak"
            if [ -f "$IWD_CONF.mote-bak" ]; then
                mv "$IWD_CONF.mote-bak" "$IWD_CONF"
            else
                rm -f "$IWD_CONF"
            fi
            rm -f /var/lib/iwd/*.psk
            log "removed the iwd backend drop-in, its config and its copy of the wifi key"
            return 0
        fi
    done

    log "FAILED: no network ${timeout}s after the revert -- putting iwd back"
    [ -f "$BACKEND_DEST.mote-bak" ] && mv "$BACKEND_DEST.mote-bak" "$BACKEND_DEST"
    systemctl disable --now wpa_supplicant || true
    systemctl enable --now iwd || true
    systemctl restart NetworkManager
    sleep 15
    log "rollback done; NetworkManager state: $(nmcli -t -f STATE general 2>/dev/null || echo unknown)"
    return 1
}

usage() {
    sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --powersave-only) MODE=powersave ;;
        --roaming-only)   MODE=roaming ;;
        --no-guard)       GUARD=0 ;;
        --guard-timeout)  GUARD_TIMEOUT="$2"; shift ;;
        --_guarded-revert) exec >>"$LOG" 2>&1; guarded_revert "$2"; exit $? ;;
        -h|--help)        usage 0 ;;
        *) echo "unknown argument: $1" >&2; usage 1 ;;
    esac
    shift
done

[ "$MODE" = roaming ] || install_powersave
if [ "$MODE" = powersave ]; then
    exit 0
fi

install_roam_option

if ! has_iwd_backend; then
    log "wpa_supplicant is the wifi backend already; nothing to revert"
    echo
    echo "Reboot to load brcmfmac with roamoff=0, then: pixi run wifi-check"
    exit 0
fi

if [ "$GUARD" = 0 ]; then
    sudo bash "$SRC_DIR/install.sh" --_guarded-revert "$GUARD_TIMEOUT"
    exit $?
fi

sudo touch "$LOG"
# Follow only what this run appends: the log keeps previous runs, and their
# verdict lines would otherwise be read as this run's.
LOG_START=$(sudo cat "$LOG" 2>/dev/null | wc -l)
cat <<EOF

This robot is still on the iwd backend, which cannot roam on this card. Handing
it back to wpa_supplicant now. This restarts NetworkManager, so an ssh session
over wifi will drop for a few seconds. The revert runs detached and guards
itself: no network within ${GUARD_TIMEOUT}s and it puts iwd back.

Reconnect and read the verdict:  sudo tail -20 $LOG

EOF
# The child reopens $LOG as root itself (--_guarded-revert execs onto it). This
# shell must not redirect there: it is not root, and the redirect would be
# refused before sudo ever ran.
sudo setsid nohup bash "$SRC_DIR/install.sh" --_guarded-revert "$GUARD_TIMEOUT" \
    </dev/null >/dev/null 2>&1 &
disown || true

# Follow along for as long as this session survives the restart, and stop as
# soon as the guard has reached a verdict.
sleep 2
timeout "$((GUARD_TIMEOUT + 30))" sudo tail -f -n "+$((LOG_START + 1))" "$LOG" 2>/dev/null \
    | sed -E '/^\[[^]]+\] (OK|FAILED):/q' || true

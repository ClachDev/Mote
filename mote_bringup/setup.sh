#!/usr/bin/env bash
# One-time host setup for a robot: the things the software package cannot carry.
#
#   pixi run setup                    # all of the below, in this order
#   pixi run setup udev               # stable device names + dialout group
#   pixi run setup systemd            # the mote-*.service units (installed, not enabled)
#   pixi run setup wifi [ARGS...]     # power-save drop-in + firmware roaming
#
# Extra arguments go to the subcommand: `setup wifi --roaming-only`,
# `setup wifi --no-guard` (see wifi/install.sh). Idempotent; uses sudo. Joining
# the tailnet is not here: a robot is joined by provisioning (pixi run provision)
# or `pixi run fleet join`, any other machine by `pixi run tailnet`.
#
# wifi runs last because its roaming half may restart NetworkManager, and an ssh
# session over wifi does not survive that. Everything else is already done by then.
set -euo pipefail
# udevadm, usermod and modprobe live in sbin, which a pixi shell does not carry.
export PATH="$PATH:/usr/sbin:/sbin"

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
MOTE_USER="${SUDO_USER:-$USER}"
MOTE_STATE="${MOTE_HOME:-$HOME/.mote}"

usage() {
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

step() { echo; echo "==> $*"; }

do_udev() {
    step "udev: /dev/mote_servos, /dev/mote_lidar, /dev/mote_camera"
    sudo install -m 0644 "$SRC_DIR/udev/99-mote.rules" /etc/udev/rules.d/99-mote.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    # The servo bus and the lidar are serial devices owned by dialout. Takes
    # effect at the next login.
    sudo usermod -aG dialout "$MOTE_USER"
}

do_systemd() {
    step "systemd: mote-*.service"
    bash "$SRC_DIR/systemd/install.sh"
}

do_wifi() {
    step "wifi: power save off, firmware roaming on"
    bash "$SRC_DIR/wifi/install.sh" "$@"
}

identity_notice() {
    if [ ! -f "$MOTE_STATE/robot.yaml" ]; then
        echo
        echo "No robot identity yet ($MOTE_STATE/robot.yaml). The systemd units and the"
        echo "fleet agent need one:  pixi run identity set --id mote-01 --name 'Scout'"
    fi
}

cmd="${1:-all}"
[ $# -gt 0 ] && shift
case "$cmd" in
    udev)    do_udev ;;
    systemd) do_systemd ;;
    wifi)    do_wifi "$@" ;;
    all)
        [ $# -eq 0 ] || { echo "setup: 'all' takes no arguments" >&2; usage 1; }
        do_udev
        do_systemd
        identity_notice
        do_wifi
        ;;
    -h|--help) usage 0 ;;
    *) echo "unknown subcommand: $cmd (udev|systemd|wifi|all)" >&2; usage 1 ;;
esac

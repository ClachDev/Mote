#!/usr/bin/env bash
# Report the robot's wifi roaming state: whether anything at all is taking the
# roam decision, what is configured for the next boot, and what is in view.
#
#   pixi run wifi-check
#
# Read-only, and everything that decides the answer reads without root. The live
# module parameter needs it (/sys/module/brcmfmac/parameters is mode 0400), but
# it only confirms what `iw phy` already reports.
set -uo pipefail
export PATH="$PATH:/usr/sbin:/sbin"

IFACE="${1:-wlan0}"
PHY=$(iw dev "$IFACE" info 2>/dev/null | awk '/wiphy/ {print "phy"$2; exit}')
ok()   { printf '  \033[32m+\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m-\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }

echo "== who takes the roam decision =="
# The one fact that settles it, and it is not the module parameter: brcmfmac
# advertises NL80211_ATTR_ROAM_SUPPORT only when its firmware roaming engine is
# on, and no userspace backend roams on this card in its place. README.md has
# the two source paths.
if [ -z "$PHY" ]; then
    bad "$IFACE has no wiphy -- is the interface up?"
elif iw phy "$PHY" info 2>/dev/null | grep -q 'Device supports roaming.'; then
    ok "the Broadcom firmware does: $PHY advertises roaming support"
    info "trigger -75 dBm, delta 20 dB -- fixed in brcmfmac, not tunable from userspace"
else
    bad "nobody does: $PHY does not advertise roaming support, so the firmware engine is off"
    info "and neither iwd nor wpa_supplicant will roam on this card in its place"
    info "run: pixi run wifi-roaming, then reboot"
fi

echo
echo "== configured for the next boot =="
# `modprobe -c` is the concatenation modprobe will hand the module, in order,
# and duplicate parameters resolve last-wins -- so the last line is the answer
# and an earlier one saying otherwise is noise.
mapfile -t opts < <(modprobe -c 2>/dev/null | grep '^options brcmfmac')
if [ "${#opts[@]}" -eq 0 ]; then
    info "no modprobe.d file names brcmfmac; the driver default (roamoff=0) applies"
    configured=0
else
    for line in "${opts[@]}"; do info "$line"; done
    case "${opts[-1]}" in
        *roamoff=1*) configured=1 ;;
        *roamoff=0*) configured=0 ;;
        *)           configured=0 ;;
    esac
    [ "${#opts[@]}" -gt 1 ] && info "(the last line wins)"
fi
case "$configured" in
    0) ok "roamoff=0: the firmware will roam" ;;
    1) bad "roamoff=1: nothing will roam" ;;
esac

live=$(sudo -n cat /sys/module/brcmfmac/parameters/roamoff 2>/dev/null)
if [ -n "$live" ]; then
    if [ "$live" = "$configured" ]; then
        ok "the running module already has roamoff=$live"
    else
        bad "the running module has roamoff=$live -- reboot to pick up roamoff=$configured"
    fi
else
    info "live value needs root: sudo $0"
fi

echo
echo "== backend =="
if systemctl is-active --quiet iwd 2>/dev/null; then
    if [ "$configured" = 1 ]; then
        bad "iwd is running and firmware roaming is off: the combination that roams never"
    else
        info "iwd is running; with firmware roaming on it stands aside (station_cannot_roam)"
    fi
    info "iwd also hides real BSSIDs from nmcli, which makes a roam log harder to read"
    info "run: pixi run wifi-roaming"
elif systemctl is-active --quiet wpa_supplicant 2>/dev/null; then
    ok "wpa_supplicant is NetworkManager's wifi backend"
else
    info "neither iwd nor wpa_supplicant is running; NetworkManager will start one on demand"
fi

echo
echo "== link =="
iw dev "$IFACE" link 2>/dev/null | grep -E "Connected|SSID|freq|signal|bitrate" | sed 's/^/    /' \
    || info "not associated"

echo
echo "== same-SSID access points in view =="
ssid=$(iw dev "$IFACE" link 2>/dev/null | awk '/SSID:/ {$1=""; sub(/^ /,""); print; exit}')
if [ -n "$ssid" ]; then
    nmcli -f IN-USE,BSSID,CHAN,FREQ,SIGNAL,SSID dev wifi list --rescan yes 2>/dev/null \
        | awk -v s="$ssid" 'NR==1 || index($0, s)' | sed 's/^/    /'
else
    info "not associated; nothing to compare against"
fi

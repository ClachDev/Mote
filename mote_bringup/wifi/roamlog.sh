#!/usr/bin/env bash
# Log what the wifi link does while the robot moves, so an acceptance walk
# produces a file rather than an impression.
#
#   pixi run wifi-roamlog                    # log to ~/.mote/wifi/roam-<stamp>.csv
#   pixi run wifi-roamlog -- --ping 1.1.1.1  # measure loss against a host
#   pixi run wifi-roamlog -- --scan-every 0  # never scan (see below)
#
# One row a second: which AP the robot is on, how strong it is, and the best
# same-SSID AP it can see instead. Every change of BSSID is a roam and is called
# out on stderr as it happens, so the person carrying the robot hears it.
#
# The last column is what makes a walk that logs no roam worth anything. The
# firmware scans on its own and tells the host nothing, so with no scanning here
# a walk that stays on one AP cannot say whether a better one existed. This
# scans -- but only while the link is at or below --scan-below, so a robot on a
# strong link is measured without being disturbed, and the off-channel time only
# lands in the stretch where a roam was due anyway.
set -uo pipefail
export PATH="$PATH:/usr/sbin:/sbin"

IFACE=wlan0
PING_HOST=""
OUT=""
SCAN_BELOW=-70
SCAN_EVERY=15

while [ $# -gt 0 ]; do
    case "$1" in
        --iface)       IFACE="$2"; shift ;;
        --ping)        PING_HOST="$2"; shift ;;
        --out)         OUT="$2"; shift ;;
        --scan-below)  SCAN_BELOW="$2"; shift ;;
        --scan-every)  SCAN_EVERY="$2"; shift ;;
        -h|--help) sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
    shift
done

if [ -z "$OUT" ]; then
    dir="${MOTE_HOME:-$HOME/.mote}/wifi"
    mkdir -p "$dir"
    OUT="$dir/roam-$(date +%Y%m%dT%H%M%S).csv"
fi

# A default gateway is the cheapest honest reachability probe: it is one hop
# away, so loss is the wifi link's and not the internet's.
if [ -z "$PING_HOST" ]; then
    PING_HOST=$(ip route show default 2>/dev/null | awk '/default/ {print $3; exit}')
fi

echo "logging to $OUT (ctrl-c to stop); ping target ${PING_HOST:-none}" >&2
if [ "$SCAN_EVERY" -gt 0 ]; then
    echo "scanning every ${SCAN_EVERY}s while the link is at or below ${SCAN_BELOW} dBm" >&2
fi
echo "time,bssid,freq_mhz,signal_dbm,tx_bitrate_mbps,ipv4,visible_same_ssid,ping_ms,best_other_bssid,best_other_dbm,scanned" > "$OUT"

ssid=$(iw dev "$IFACE" link 2>/dev/null | awk '/SSID:/ {$1=""; sub(/^ /,""); print; exit}')
last_bssid=""
last_ipv4=""
roams=0
last_scan=0
start=$(date +%s)

cleanup() {
    local dur=$(( $(date +%s) - start ))
    echo >&2
    echo "stopped after ${dur}s: ${roams} roam(s) logged to $OUT" >&2
    exit 0
}
trap cleanup INT TERM

# Ask whatever owns the radio for a scan. `iw scan` needs CAP_NET_ADMIN and the
# walk is run by the login user, so it goes through a daemon either way -- and it
# has to be the right one: measured on mote-01, `nmcli dev wifi list --rescan
# yes` under the iwd backend leaves the kernel's BSS cache at one entry, because
# NetworkManager answers it from iwd's network list rather than triggering a
# scan. Fired and forgotten; the results are read from the cache on later ticks,
# so a scan taking seconds never stalls the log.
trigger_scan() {
    if systemctl is-active --quiet iwd 2>/dev/null; then
        iwctl station "$IFACE" scan >/dev/null 2>&1 &
    else
        nmcli dev wifi list --rescan yes >/dev/null 2>&1 &
    fi
}

# The kernel's scan cache, reduced to this network's access points. A BSS
# expires from it about 30 s after it was last seen, so what comes back is what
# is in view now rather than everything ever seen.
same_ssid_bss() {
    iw dev "$IFACE" scan dump 2>/dev/null | awk -v want="$ssid" '
        /^BSS / { bssid = $2; sub(/\(.*/, "", bssid); sig = "" }
        /^\tsignal: / { sig = $2 }
        /^\tSSID: / { name = substr($0, 8); if (name == want && sig != "") print bssid, sig }
    '
}

while true; do
    link=$(iw dev "$IFACE" link 2>/dev/null)
    bssid=$(echo "$link" | awk '/^Connected to/ {print $3; exit}')
    freq=$(echo "$link" | awk '/freq:/ {print $2; exit}')
    sig=$(echo "$link" | awk '/signal:/ {print $2; exit}')
    rate=$(echo "$link" | awk '/tx bitrate:/ {print $3; exit}')
    # The address, because a roam that changes it is a roam that breaks
    # everything above IP -- ssh, Foxglove, the fleet agent's broker connection
    # -- while the link itself reads perfectly. Measured on the 2026-09-01 walk:
    # 54 s of total loss at -35 dBm, explicable only from the DHCP journal until
    # this column existed.
    ipv4=$(ip -4 -o addr show "$IFACE" 2>/dev/null | awk '{print $4; exit}')

    # A scan costs off-channel time on the link being measured, and it sweeps
    # both bands, so the cost is not one tick: measured on mote-01, about 4 s
    # over which the round trip goes 3 ms -> 90-114 ms and the tx bitrate
    # 433 -> 24 Mbps. Narrowing it to the known channels would need root, which
    # a walk does not have. So the tick that fired one says so, and the rows
    # after it can be attributed rather than guessed at.
    scanned=0
    now=$(date +%s)
    if [ "$SCAN_EVERY" -gt 0 ] && [ -n "$sig" ] \
       && [ "${sig%.*}" -le "$SCAN_BELOW" ] \
       && [ $(( now - last_scan )) -ge "$SCAN_EVERY" ]; then
        last_scan=$now
        scanned=1
        trigger_scan
    fi

    seen=0
    best_bssid=""
    best_sig=""
    while read -r b s; do
        [ -n "$b" ] || continue
        seen=$((seen + 1))
        [ "$b" = "$bssid" ] && continue
        if [ -z "$best_sig" ] || [ "${s%.*}" -gt "${best_sig%.*}" ]; then
            best_bssid=$b
            best_sig=$s
        fi
    done < <(same_ssid_bss)

    rtt=""
    if [ -n "$PING_HOST" ]; then
        rtt=$(ping -c1 -W1 -I "$IFACE" "$PING_HOST" 2>/dev/null \
              | awk -F'time=' '/time=/ {print $2+0; exit}')
    fi

    if [ -z "$bssid" ]; then
        echo "$(date -Is),,,,,${ipv4},${seen},${rtt:-},${best_bssid},${best_sig},${scanned}" >> "$OUT"
        printf '\r%s  DISCONNECTED                        ' "$(date +%H:%M:%S)" >&2
    else
        echo "$(date -Is),$bssid,${freq:-},${sig:-},${rate:-},${ipv4},${seen},${rtt:-},${best_bssid},${best_sig},${scanned}" >> "$OUT"
        printf '\r%s  %s  %s MHz  %s dBm  %s Mbps  ping %sms  best other %s   ' \
            "$(date +%H:%M:%S)" "$bssid" "${freq:-?}" "${sig:-?}" "${rate:-?}" \
            "${rtt:-x}" "${best_sig:--}" >&2
        if [ -n "$last_bssid" ] && [ "$bssid" != "$last_bssid" ]; then
            roams=$((roams + 1))
            printf '\n  ROAM %s -> %s (%s MHz, %s dBm)\n' \
                "$last_bssid" "$bssid" "${freq:-?}" "${sig:-?}" >&2
        fi
        if [ -n "$last_ipv4" ] && [ "$ipv4" != "$last_ipv4" ]; then
            printf '\n  ADDRESS %s -> %s: this access point is a different network\n' \
                "$last_ipv4" "${ipv4:-none}" >&2
        fi
        last_bssid="$bssid"
        [ -n "$ipv4" ] && last_ipv4="$ipv4"
    fi
    sleep 1
done

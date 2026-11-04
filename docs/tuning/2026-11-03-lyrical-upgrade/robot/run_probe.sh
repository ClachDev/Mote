#!/usr/bin/env bash
# On the Pi: wake the robot target, wait for Nav2, run the probe, back to standby.
#   bash run_probe.sh <label>      (robot_probe.py in ~; results in ~/probe-<label>/)
set -u
L=$1
OUT=~/probe-$L
mkdir -p "$OUT"
START=$(date '+%Y-%m-%d %H:%M:%S')
systemctl start --no-block mote-robot.target
for _ in $(seq 1 150); do
  journalctl -u mote-nav@robot --since "$START" --no-pager | grep -q 'lifecycle_manager_navigation.*Managed nodes are active' && break
  sleep 2
done
journalctl -u mote-nav@robot --since "$START" --no-pager | grep -q 'Managed nodes are active' || echo "NAV NOT ACTIVE" > "$OUT/probe.txt"
sleep 10
cd ~/Mote || exit 1
timeout 200 ~/.pixi/bin/pixi run -- python ~/robot_probe.py >> "$OUT/probe.txt" 2>&1
systemctl list-units --all 'mote-*' --no-legend > "$OUT/units.txt"
for u in mote-bringup mote-health mote-nav@robot mote-tasks; do
  journalctl -u $u --since "$START" --no-pager > "$OUT/$u.log"
done
systemctl stop mote-robot.target
sleep 3
systemctl list-units 'mote-*' --no-legend > "$OUT/units-after.txt"
cat "$OUT/probe.txt"
echo "overrun lines: $(grep -c -i -E 'overrun|missed' "$OUT/mote-bringup.log")"
echo "nav missed-rate lines: $(grep -c 'Control loop missed' "$OUT/mote-nav@robot.log")"
cat "$OUT/units.txt"

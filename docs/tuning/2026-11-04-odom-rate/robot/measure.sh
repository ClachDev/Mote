#!/usr/bin/env bash
# On the Pi: wake the robot target, sample CPU for 60 s at idle, then run the
# Lyrical note's probe (teleop turn, Nav2 goal out and back) while recording
# /tf and the wheel odometry; back to standby.
#   bash measure.sh <label> [goal distance, m]    (results in ~/odom-rate-<label>/)
set -u
L=$1
DIST=${2:-1.0}
HERE=$(cd "$(dirname "$0")" && pwd)
PROBE=$HERE/../../2026-11-03-lyrical-upgrade/robot/robot_probe.py
OUT=~/odom-rate-$L
rm -rf "$OUT"; mkdir -p "$OUT"
START=$(date '+%Y-%m-%d %H:%M:%S')
systemctl start --no-block mote-robot.target
for _ in $(seq 1 150); do
  journalctl -u mote-nav@robot --since "$START" --no-pager | grep -q 'lifecycle_manager_navigation.*Managed nodes are active' && break
  sleep 2
done
journalctl -u mote-nav@robot --since "$START" --no-pager | grep -q 'Managed nodes are active' || echo "NAV NOT ACTIVE" > "$OUT/probe.txt"
sleep 20
cd ~/Mote || exit 1
~/.pixi/bin/pixi run node-cpu --duration 60 --tag idle --out "$OUT" > "$OUT/node-cpu.txt" 2>&1 &
python3 "$HERE/proc_cpu.py" 60 localization_container nav2_container > "$OUT/containers.txt" 2>&1
wait
setsid ~/.pixi/bin/pixi run -- ros2 bag record -s mcap -o "$OUT/bag" --topics /tf /diff_drive_controller/odom > "$OUT/rec.log" 2>&1 &
REC=$!
sleep 3
timeout 200 ~/.pixi/bin/pixi run -- python "$PROBE" "$DIST" >> "$OUT/probe.txt" 2>&1
# SIGTERM: a background job of a non-interactive shell has SIGINT ignored.
kill -TERM -- -$REC 2>/dev/null; wait $REC
for u in mote-bringup mote-health mote-nav@robot mote-tasks; do
  journalctl -u $u --since "$START" --no-pager > "$OUT/$u.log"
done
systemctl stop mote-robot.target
cat "$OUT/probe.txt"
echo "nav missed-rate lines: $(grep -c 'Control loop missed' "$OUT/mote-nav@robot.log")"
echo "gate rejections: $(grep -c 'rejected icp increment' "$OUT/mote-bringup.log")"
cat "$OUT/containers.txt"
tail -12 "$OUT/node-cpu.txt"

#!/usr/bin/env bash
# The Lyrical note's run.sh, plus a bag of /tf and the wheel odometry while the
# probe drives, for odom_steps.py.
#   pixi run -e dev -- bash docs/tuning/2026-11-04-odom-rate/run.sh <name> <nav2_params.yaml> <outdir>
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=$1
export MOTE_NAV2_PARAMS_FILE=$2
OUT=$3
mkdir -p "$OUT"
export ROS_DOMAIN_ID=$(( (RANDOM % 60) + 160 ))
export GZ_PARTITION=mote-rate-$ROS_DOMAIN_ID
LOG=$OUT/$NAME.sim.log
setsid ros2 launch mote_simulation sim_launch.py mode:=nav world:=office_world.sdf > "$LOG" 2>&1 &
PG=$!
for _ in $(seq 1 120); do
  grep -q 'lifecycle_manager_navigation.*Managed nodes are active' "$LOG" && break
  sleep 1
done
sleep 10
rm -rf "$OUT/$NAME.bag"
setsid ros2 bag record -s mcap -o "$OUT/$NAME.bag" --topics /tf /diff_drive_controller/odom > "$OUT/$NAME.rec.log" 2>&1 &
REC=$!
python "$HERE/../2026-11-03-lyrical-upgrade/probe.py" > "$OUT/$NAME.txt" 2>&1
# SIGTERM, not SIGINT: a background job of a non-interactive shell starts
# with SIGINT ignored, and the recorder then never finalises the bag.
kill -TERM -- -$REC 2>/dev/null; wait $REC
kill -TERM -- -$PG 2>/dev/null; sleep 5; kill -KILL -- -$PG 2>/dev/null
echo "$NAME: $(tr '\n' ' ' < "$OUT/$NAME.txt")"
echo "  missed-rate warnings: $(grep -c 'Control loop missed' "$LOG")"
python "$HERE/../../../mote_bringup/tools/odom_steps.py" "$OUT/$NAME.bag" --csv "$OUT/$NAME.steps.csv"

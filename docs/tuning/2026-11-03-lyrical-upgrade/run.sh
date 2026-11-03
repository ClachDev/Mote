#!/usr/bin/env bash
# One variant: sim mode:=nav on office_world with a given nav2 params file,
# one NavigateToPose goal (pickup), /cmd_vel_nav timed in wall clock by probe.py.
#   pixi run -e dev -- bash docs/tuning/2026-11-03-lyrical-upgrade/run.sh <name> <nav2_params.yaml> <outdir>
# variants.py writes the variant files used in the note.
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
python "$HERE/probe.py" > "$OUT/$NAME.txt" 2>&1
kill -TERM -- -$PG 2>/dev/null; sleep 5; kill -KILL -- -$PG 2>/dev/null
echo "$NAME: $(tr '\n' ' ' < "$OUT/$NAME.txt")"
echo "  missed-rate warnings: $(grep -c 'Control loop missed' "$LOG")"

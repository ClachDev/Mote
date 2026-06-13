#!/usr/bin/env bash
# Headless end-to-end smoke test for the Mote Gazebo sim (~25 s on a workstation
# with a GPU; longer under software rendering). Brings up sim_launch.py +
# slam_launch.py, runs verify_sim.py, and tears everything down.
#
# Must run inside the 'sim' pixi environment, where gz, ros2 and the sim deps
# are on PATH:  pixi run -e sim sim-test
#
# Exits 0 only if every stage passes; prints "FAIL: ..." and exits 1 otherwise.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERIFY="$SCRIPT_DIR/verify_sim.py"

SIM_LOG="$(mktemp -t mote_sim_smoke_sim.XXXXXX.log)"
SLAM_LOG="$(mktemp -t mote_sim_smoke_slam.XXXXXX.log)"
SIM_PID=""
SLAM_PID=""

cleanup() {
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    [ -n "$SLAM_PID" ] && kill -- -"$SLAM_PID" 2>/dev/null
    sleep 2
    # belt-and-braces: these match the sim's own processes, not this script
    pkill -9 -f 'mote_world' 2>/dev/null
    pkill -9 -f 'async_slam_toolbox_node' 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    true
}
trap cleanup EXIT

fail() { echo "FAIL: $1"; [ -n "${2:-}" ] && tail -25 "$2"; exit 1; }

# Start clean
ros2 daemon stop >/dev/null 2>&1
sleep 1

echo ">> launching sim..."
setsid ros2 launch mote_bringup sim_launch.py > "$SIM_LOG" 2>&1 &
SIM_PID=$!
for _ in $(seq 90); do
    grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" && break
    grep -q "Failed to load system plugin" "$SIM_LOG" && fail "gz_ros2_control plugin failed to load" "$SIM_LOG"
    kill -0 "$SIM_PID" 2>/dev/null || fail "sim process exited early" "$SIM_LOG"
    sleep 2
done
grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" \
    || fail "diff_drive_controller never activated" "$SIM_LOG"
echo "STEP1 OK: controllers active"

echo ">> launching slam..."
setsid ros2 launch mote_bringup slam_launch.py use_sim_time:=true > "$SLAM_LOG" 2>&1 &
SLAM_PID=$!
for _ in $(seq 45); do
    ros2 node list 2>/dev/null | grep -q slam_toolbox && break
    sleep 2
done
ros2 node list 2>/dev/null | grep -q slam_toolbox || fail "slam_toolbox never came up" "$SLAM_LOG"
echo "STEP2 OK: slam_toolbox up"

echo ">> driving + verifying..."
timeout 120 python3 "$VERIFY" || fail "verify_sim.py assertions failed"
echo "SMOKE TEST PASS"

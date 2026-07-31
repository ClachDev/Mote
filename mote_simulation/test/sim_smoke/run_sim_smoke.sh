#!/usr/bin/env bash
# Headless end-to-end smoke test for the Mote Gazebo sim (~25 s on a workstation
# with a GPU; longer under software rendering). Brings up the sim running the
# real mapping mission (sim_launch.py mode:=mapping, i.e. mapping_launch.py with
# base:=false), runs verify_sim.py, and tears everything down.
#
# Must run inside the 'sim' pixi environment, where gz, ros2 and the sim deps
# are on PATH:  pixi run sim-test
#
# Exits 0 only if every stage passes; prints "FAIL: ..." and exits 1 otherwise.
#
# Needs a real GPU render backend; llvmpipe is too slow. Local pre-PR gate,
# not hosted CI.
#
# Isolated from every other sim on the machine, in both directions: it claims a
# free ROS_DOMAIN_ID + GZ_PARTITION (tools/sim_domain.py, shared with bench.py)
# so it never sees a concurrent benchmark's /scan, /tf or /clock, and its
# teardown is scoped to its own process session and this worktree's path so it
# never reaps another worktree's gz server.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VERIFY="$SCRIPT_DIR/verify_sim.py"

SIM_LOG="$(mktemp -t mote_sim_smoke_sim.XXXXXX.log)"
SIM_PID=""
SIM_SID=""

cleanup() {
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    sleep 2
    # Everything the launch started lives in the session setsid gave it, so the
    # session id is an exact scope: it reaps stragglers whatever they are called
    # and can never touch another worktree's sim — which bare name matches
    # ('mote_world', 'async_slam_toolbox_node') did.
    [ -n "$SIM_SID" ] && pkill -9 -s "$SIM_SID" 2>/dev/null
    # Backstop for a gz server that escaped the session, scoped to THIS
    # worktree's world path (as bench.py's is).
    pkill -9 -f "gz sim.*$ROOT" 2>/dev/null
    # Daemons are per-domain, so this stops ours and leaves other runs' alone.
    ros2 daemon stop >/dev/null 2>&1
    true
}
trap cleanup EXIT

fail() { echo "FAIL: $1"; [ -n "${2:-}" ] && tail -25 "$2"; exit 1; }

# A graph of our own: no other sim, benchmark or robot can reach it, and nothing
# below can reach them. An inherited ROS_DOMAIN_ID/GZ_PARTITION is respected.
DOMAIN_ENV="$(python3 "$ROOT/mote_simulation/tools/sim_domain.py" --shell --prefix mote-smoke)" \
    || fail "could not claim a ROS domain"
eval "$DOMAIN_ENV"
echo ">> ROS_DOMAIN_ID=$ROS_DOMAIN_ID (${MOTE_DOMAIN_HOW:-unknown}), GZ_PARTITION=$GZ_PARTITION"

# Start clean
ros2 daemon stop >/dev/null 2>&1
sleep 1

echo ">> launching sim (mode:=mapping)..."
setsid ros2 launch mote_simulation sim_launch.py mode:=mapping > "$SIM_LOG" 2>&1 &
SIM_PID=$!
SIM_SID="$(ps -o sid= -p "$SIM_PID" 2>/dev/null | tr -d ' ')"
# If setsid did not detach it, the launch shares OUR session and killing that
# session would kill this script (and its caller) — drop the scope instead.
[ "$SIM_SID" = "$(ps -o sid= -p $$ | tr -d ' ')" ] && SIM_SID=""
for _ in $(seq 90); do
    grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" && break
    grep -q "Failed to load system plugin" "$SIM_LOG" && fail "gz_ros2_control plugin failed to load" "$SIM_LOG"
    kill -0 "$SIM_PID" 2>/dev/null || fail "sim process exited early" "$SIM_LOG"
    sleep 2
done
grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" \
    || fail "diff_drive_controller never activated" "$SIM_LOG"
echo "STEP1 OK: controllers active"

echo ">> waiting for the mapping mission (slam_toolbox)..."
for _ in $(seq 45); do
    ros2 node list 2>/dev/null | grep -q slam_toolbox && break
    sleep 2
done
ros2 node list 2>/dev/null | grep -q slam_toolbox || fail "slam_toolbox never came up" "$SIM_LOG"
echo "STEP2 OK: slam_toolbox up"

echo ">> driving + verifying..."
timeout 120 python3 "$VERIFY" || fail "verify_sim.py assertions failed"
echo "SMOKE TEST PASS"

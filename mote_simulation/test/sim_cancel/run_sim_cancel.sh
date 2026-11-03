#!/usr/bin/env bash
# Cancel a mission mid-drive in the Gazebo sim and time the stop (~2 min on a
# workstation with a GPU). Brings up `sim_launch.py mode:=nav` on mote_world,
# which is the real robot_launch.py and the real tasks_launch.py over a Gazebo
# base, runs verify_cancel.py, and tears everything down.
#
# Must run inside the 'sim' pixi environment:
#   pixi run -e sim bash mote_simulation/test/sim_cancel/run_sim_cancel.sh [--out FILE]
#
# Exits 0 only if every check passes; prints "FAIL: ..." and exits 1 otherwise.
#
# Isolated the way the smoke test is: a claimed ROS_DOMAIN_ID + GZ_PARTITION,
# and teardown scoped to the launch's own process session.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
LOG="$(mktemp -t mote_sim_cancel.XXXXXX.log)"
SIM_PID=""
SIM_SID=""

cleanup() {
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    sleep 2
    [ -n "$SIM_SID" ] && pkill -9 -s "$SIM_SID" 2>/dev/null
    pkill -9 -f "gz sim.*$ROOT" 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    true
}
trap cleanup EXIT

fail() { echo "FAIL: $1"; tail -25 "$LOG"; exit 1; }

# pixi activation sources the manifest's own install/. Run from a git worktree
# that has built its packages, this checkout's install/ goes over it, so what is
# under test is this checkout's code and not the main tree's.
if [ -f "$ROOT/install/local_setup.bash" ]; then
    # colcon's setup scripts read variables that may be unset.
    set +u
    # shellcheck disable=SC1091
    source "$ROOT/install/local_setup.bash"
    set -u
fi

eval "$(python3 "$ROOT/mote_simulation/tools/sim_domain.py" --shell --prefix mote-cancel)" \
    || fail "could not claim a ROS domain"
echo ">> ROS_DOMAIN_ID=$ROS_DOMAIN_ID (${MOTE_DOMAIN_HOW:-unknown}), GZ_PARTITION=$GZ_PARTITION"
ros2 daemon stop >/dev/null 2>&1

echo ">> launching sim (mode:=nav, mote_world)..."
setsid ros2 launch mote_simulation sim_launch.py mode:=nav > "$LOG" 2>&1 &
SIM_PID=$!
sleep 0.5
SIM_SID="$(ps -o sid= -p "$SIM_PID" 2>/dev/null | tr -d ' ')"
[ "$SIM_SID" = "$(ps -o sid= -p $$ | tr -d ' ')" ] && SIM_SID=""

for _ in $(seq 180); do
    grep -q "Managed nodes are active" "$LOG" && break
    kill -0 "$SIM_PID" 2>/dev/null || fail "sim exited early"
    sleep 1
done
grep -q "Managed nodes are active" "$LOG" || fail "Nav2 never activated"
echo ">> Nav2 active; log $LOG"

timeout 240 python3 "$SCRIPT_DIR/verify_cancel.py" "$@"

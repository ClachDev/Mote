#!/usr/bin/env bash
# The whole teleop loop against a follower that isn't there.
#
#   mock_arm (+ synthetic camera) -> arm_teleop --demo
#                                 -> episode_record -> episode_replay
#
# Nothing here needs the arm, the camera, or a terminal, so it is the gate to
# run before taking any of this to the bench (mote_arm/BENCH.md). It ends by
# planning the LeRobot export, which needs no LeRobot: the plan is computed from
# the capture alone.
#
#   pixi run arm-teleop-test [seconds]
set -euo pipefail

DEMO_SECONDS="${1:-12}"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/mote-teleop-loop.XXXXXX")"
CAPTURE="$WORK/episodes/loop"
LOGS="$WORK/logs"
mkdir -p "$LOGS"

# Off the LAN and off any sibling session: these nodes command arm_controller,
# which moves a real arm. Same rule as the rclpy unit tests.
export ROS_DOMAIN_ID=$((RANDOM % 40 + 60))
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# `ros2 run` is a wrapper that Popens the real executable and handles no SIGTERM,
# so killing its pid hands the node to init and leaks it — the exact class of
# straggler `pixi run sweep` exists to find (CLAUDE.md, "Stray ROS processes").
# Every job is therefore setsid-ed into a session of its own, and torn down by
# process group and then by session id: the shell equivalent of
# sweep_orphans.spawn_reapable/reap_group, and the same scoping the sim smoke
# test uses.
NAMES=()
declare -A PID_OF
declare -A SID_OF
OUR_SID="$(ps -o sid= -p $$ | tr -d ' ')"

reap() {
    local name="$1" pid="${PID_OF[$1]:-}" sid="${SID_OF[$1]:-}"
    [ -n "$pid" ] || return 0
    kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    # Backstop for anything that left the group but not the session.
    [ -n "$sid" ] && pkill -9 -s "$sid" 2>/dev/null
    unset "PID_OF[$name]"
    true
}

cleanup() {
    for name in "${NAMES[@]:-}"; do
        [ -n "$name" ] && reap "$name"
    done
}
trap cleanup EXIT

fail() {
    echo "FAIL: $*" >&2
    echo "--- logs in $LOGS ---" >&2
    tail -n 20 "$LOGS"/*.log >&2 || true
    exit 1
}

background() {
    local name="$1"
    shift
    setsid "$@" >"$LOGS/$name.log" 2>&1 &
    local pid=$!
    NAMES+=("$name")
    PID_OF[$name]=$pid
    local sid
    sid="$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')"
    # If setsid did not detach it, the job shares OUR session and killing that
    # session would take this script with it — drop the scope instead.
    [ "$sid" = "$OUR_SID" ] && sid=""
    SID_OF[$name]="$sid"
}

stop() {
    for name in "$@"; do
        reap "$name"
    done
}

echo "== 1/6  mock follower with a synthetic camera =="
# --droop: a real servo settles short of its goal (kp*error balances the load),
# so the mock does too. Without it the mock lands exactly on every setpoint and
# the recorded action would be indistinguishable from the observed state.
background mock_arm ros2 run mote_arm mock_arm --camera --rate 20 --speed 1.0 --droop 0.01
sleep 4

echo "== 2/6  teleop -> follower, ${DEMO_SECONDS}s =="
background teleop ros2 run mote_arm arm_teleop -- --demo "$DEMO_SECONDS" --speed 0.3

echo "== 3/6  record the session =="
ros2 run mote_arm episode_record -- \
    --task "sweep the first joint" \
    --root "$CAPTURE" \
    --duration "$((DEMO_SECONDS - 3))" \
    --episodes 1 \
    >"$LOGS/record.log" 2>&1 || fail "recording exited non-zero"
cat "$LOGS/record.log"

EPISODE="$CAPTURE/episode_000"
[ -f "$CAPTURE/dataset.json" ] || fail "no dataset.json in $CAPTURE"
[ -f "$EPISODE/episode.json" ] || fail "episode was not closed"

echo "== 4/6  check the capture actually holds a motion =="
python3 "$(dirname "$0")/check_capture.py" "$CAPTURE" || fail "capture check failed"

echo "== 5/6  replay it on the follower at half speed =="
# The leader and the mirror have to be out of the way first: replay publishes
# arm_controller itself, and two things commanding one arm fight. (The stall
# guard does catch it — that is how this was found — but a caught stall is not
# a passing replay.)
stop teleop
sleep 1
ros2 run mote_arm episode_replay -- "$CAPTURE" --episode 0 --yes --speed-scale 0.5 \
    >"$LOGS/replay.log" 2>&1 || fail "replay exited non-zero"
tail -n 12 "$LOGS/replay.log"
grep -q "STOPPED during" "$LOGS/replay.log" && fail "replay stalled"

echo "== 6/6  plan the LeRobot export (no LeRobot required) =="
python3 "$(dirname "$0")/../../tools/lerobot_export.py" --capture "$CAPTURE" --dry-run \
    || fail "export plan failed"

echo
echo "PASS — teleop, recording, replay and the export plan all ran headless."
echo "capture kept at: $CAPTURE"
trap - EXIT
cleanup

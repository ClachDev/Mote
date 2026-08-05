#!/usr/bin/env bash
# The whole teleop loop against a follower that isn't there.
#
#   mock_arm (+ synthetic camera) -> arm_mirror -> virtual_leader --demo
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

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT

fail() {
    echo "FAIL: $*" >&2
    echo "--- logs in $LOGS ---" >&2
    tail -n 20 "$LOGS"/*.log >&2 || true
    exit 1
}

declare -A PID_OF
background() {
    local name="$1"
    shift
    "$@" >"$LOGS/$name.log" 2>&1 &
    PID_OF[$name]=$!
    PIDS+=("$!")
}

stop() {
    for name in "$@"; do
        kill "${PID_OF[$name]}" 2>/dev/null || true
        wait "${PID_OF[$name]}" 2>/dev/null || true
    done
}

echo "== 1/6  mock follower with a synthetic camera =="
# --droop: a real servo settles short of its goal (kp*error balances the load),
# so the mock does too. Without it the mock lands exactly on every setpoint and
# the recorded action would be indistinguishable from the observed state.
background mock_arm ros2 run mote_arm mock_arm --camera --rate 20 --speed 1.0 --droop 0.01
background mirror ros2 run mote_arm arm_mirror
sleep 4

echo "== 2/6  teleop: virtual leader -> mirror -> follower, ${DEMO_SECONDS}s =="
background leader ros2 run mote_arm virtual_leader -- --demo "$DEMO_SECONDS" --speed 0.3

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
stop leader mirror
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

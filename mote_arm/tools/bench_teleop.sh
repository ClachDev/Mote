#!/usr/bin/env bash
# Guided bench session for keyboard teleop: teleop -> record -> inspect
# -> replay, with the safety behaviours demonstrated on the way.
#
# This is the hardware counterpart of `pixi run arm-teleop-test`, which runs the
# same loop headless against the mock follower. Run that first — this script
# assumes the software already works and is here to check the *arm* does.
#
# Three terminals. The first is the robot, the second is what you drive, and
# the third is this script:
#
#   1. pixi run launch                base + camera
#        (`pixi run arm` is the same thing without lidar/camera)
#   2. pixi run arm-teleop            YOU DRIVE THIS ONE
#   3. pixi run arm-bench-teleop      <- this script: asks, records, replays
#
# It writes a report you can paste into the task; nothing is recorded as passing
# that you did not say you saw.
set -euo pipefail

DATASET="${1:-bench}"
CAPTURE="${MOTE_HOME:-$HOME/.mote}/episodes/$DATASET"
REPORT="$CAPTURE/bench-report.txt"
HERE="$(cd "$(dirname "$0")" && pwd)"

note() { printf '%s\n' "$*" | tee -a "$REPORT"; }
rule() { printf '\n== %s ==\n' "$*" | tee -a "$REPORT"; }

# Every answer is typed in THIS terminal, never in the teleop one: there, 'y'
# drives joint 6 and 'z' clears the panic latch, so an answer aimed at the wrong
# window moves the arm instead of being read. Hence the marker on every prompt.
HERE_MARK="[answer HERE]"

ask() {
    # ask "<what you should have seen>" -> records observed / NOT OBSERVED
    local prompt="$1" reply
    read -r -p "  $HERE_MARK $prompt [y/N] " reply
    if [[ "$reply" =~ ^[Yy] ]]; then
        note "  PASS  $prompt"
    else
        note "  FAIL  $prompt"
        FAILURES=$((FAILURES + 1))
    fi
}

check() {
    # check "<do this in the teleop terminal>" "<what you should have seen>"
    echo
    echo "  -> in the TELEOP terminal: $1"
    ask "$2"
}

FAILURES=0
mkdir -p "$CAPTURE"
: >"$REPORT"
note "mote_arm keyboard teleop bench session"
note "date:    $(date -Is)"
note "capture: $CAPTURE"

rule "0. preconditions"
cat <<'EOF'
Before starting, confirm at the arm:
  * it is powered, physically supported, and free to move through its band
  * `pixi run arm-setup gains show` reports kp=32 (droop, not stall — see README)
  * the arm is up: `pixi run launch` (with the camera) or `pixi run arm`
  * the TELEOP terminal is running `pixi run arm-teleop` — the one you drive

This is the last terminal: it asks the questions and records the answers.
EOF
read -r -p "  $HERE_MARK ready? [y/N] " ready
[[ "$ready" =~ ^[Yy] ]] || { echo "aborted"; exit 1; }

rule "1. the arm is reporting"
if timeout 10 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
    note "  PASS  /joint_states is publishing"
else
    note "  FAIL  no /joint_states — is the ARM terminal running?"
    exit 1
fi
NODES="$(ros2 node list 2>/dev/null)"
if grep -q arm_teleop <<<"$NODES"; then
    note "  PASS  arm_teleop is up"
else
    note "  FAIL  no arm_teleop — every check below asks you to drive the arm"
    note "        from it. Open another terminal and run \`pixi run arm-teleop\`."
    exit 1
fi

rule "2. teleop, and the three safety behaviours"
cat <<'EOF'
One at a time: do the action in the TELEOP terminal (`pixi run arm-teleop`),
then come back to THIS terminal and answer. Keep a hand on SPACE throughout.

Do not answer in the teleop terminal — 'y' drives joint 6 there and 'z' clears
the panic latch, so an answer typed into the wrong window moves the arm.
EOF
check "hold one joint's key and watch the arm move" \
      "(a) the arm followed the leader smoothly"
check "keep holding that same key past the joint's soft limit" \
      "(b) it stopped at the limit and went no further"
check "drive again, then release the key mid-move" \
      "(c) releasing the key halted it within a fraction of a second"
check "press SPACE" \
      "(d) PANIC dropped torque and the arm went limp"
check "press z to clear the latch, then drive again" \
      "(e) it resumed following from where it is, with no jump"

rule "3. record an episode"
echo "Drive a simple motion in the TELEOP terminal while this records."
echo "The ENTER prompts below are read HERE, not there."
echo "Press ENTER to start the recording; 'q' ends the step, so pressing it"
echo "first leaves nothing to check, export or replay."
ros2 run mote_arm episode_record --task "${TASK:-move the arm through a simple motion}" \
    --dataset "$DATASET" --episodes 1 2>&1 | tee -a "$REPORT"

rule "4. check the capture"
if python3 "$HERE/../test/teleop_loop/check_capture.py" "$CAPTURE" 2>&1 | tee -a "$REPORT"; then
    note "  PASS  capture holds a real motion"
    RECORDED=1
else
    note "  FAIL  capture check"
    FAILURES=$((FAILURES + 1))
    RECORDED=0
fi

# Steps 5 and 6 export and replay the episode step 3 recorded. With no episode
# they can only ask about work nobody can do, and a FAIL for each would bury
# the one thing that went wrong.
if [ "$RECORDED" -eq 0 ]; then
    rule "5-6. export and replay"
    note "  SKIPPED  there is no episode to export or replay"
    rule "result"
    note "$FAILURES check(s) failed. Record an episode in step 3 and re-run."
    note "report: $REPORT"
    exit 1
fi

rule "5. export and inspect (off-board)"
cat <<EOF | tee -a "$REPORT"
  Copy the capture to the machine with the lerobot environment, then:

    pixi run -e lerobot arm-export -- --capture $CAPTURE --repo-id mote/$DATASET
    pixi run -e lerobot -- lerobot-dataset-viz --repo-id mote/$DATASET \\
        --root $CAPTURE/lerobot --episode-index 0

  The export loads the dataset back through LeRobot and prints what it holds;
  the viewer is LeRobot's own inspection tool.
EOF
ask "the export verified, and the viewer showed the episode"

rule "6. replay it on the arm"
echo "Stop the TELEOP terminal now (press x there): the mirror and the replay"
echo "would otherwise both command arm_controller and fight over the arm."
echo
# Watched rather than asked. An operator who says the leader is stopped when it
# is not gets a replay that loses to the mirror and reports a stall — which
# reads exactly like the arm failing, and is the one failure here that is not
# about the arm at all.
echo -n "  waiting for teleop to exit"
for _ in $(seq 60); do
    ros2 node list 2>/dev/null | grep -q arm_teleop || break
    echo -n "."
    sleep 2
done
echo
if ros2 node list 2>/dev/null | grep -q arm_teleop; then
    note "  SKIP  replay: teleop is still running after 2 minutes"
    FAILURES=$((FAILURES + 1))
else
    note "  teleop has exited; replaying"
    ros2 run mote_arm episode_replay "$CAPTURE" --episode 0 --speed-scale 0.25 2>&1 | tee -a "$REPORT"
    ask "the arm retraced the recorded motion"
fi

rule "result"
if [ "$FAILURES" -eq 0 ]; then
    note "  PASS — teleop, recording, inspection and replay all verified on hardware"
else
    note "  $FAILURES check(s) did not pass"
fi
note ""
note "report: $REPORT"
exit "$((FAILURES > 0))"

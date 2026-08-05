#!/usr/bin/env bash
# Guided bench session for virtual-leader teleop: teleop -> record -> inspect
# -> replay, with the safety behaviours demonstrated on the way.
#
# This is the hardware counterpart of `pixi run arm-teleop-test`, which runs the
# same loop headless against the mock follower. Run that first — this script
# assumes the software already works and is here to check the *arm* does.
#
# Three terminals:
#   A: pixi run arm mirror:=true       driver + mirror
#   B: pixi run arm-teleop             the virtual leader (you drive this)
#   C: bash mote_arm/tools/bench_teleop.sh    <- this script
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

ask() {
    # ask "<what you should have seen>" -> records observed / NOT OBSERVED
    local prompt="$1" reply
    read -r -p "  $prompt [y/N] " reply
    if [[ "$reply" =~ ^[Yy] ]]; then
        note "  PASS  $prompt"
    else
        note "  FAIL  $prompt"
        FAILURES=$((FAILURES + 1))
    fi
}

FAILURES=0
mkdir -p "$CAPTURE"
: >"$REPORT"
note "mote_arm virtual-leader bench session"
note "date:    $(date -Is)"
note "capture: $CAPTURE"

rule "0. preconditions"
cat <<'EOF'
Before starting, confirm at the arm:
  * it is powered, physically supported, and free to move through its band
  * `pixi run arm-gains show` reports kp=32 (droop, not stall — see README)
  * terminal A is running `pixi run arm mirror:=true`
  * terminal B is running `pixi run arm-teleop`
EOF
read -r -p "  ready? [y/N] " ready
[[ "$ready" =~ ^[Yy] ]] || { echo "aborted"; exit 1; }

rule "1. the arm is reporting"
if timeout 10 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
    note "  PASS  /joint_states is publishing"
else
    note "  FAIL  no /joint_states — is terminal A running?"
    exit 1
fi
if ros2 node list 2>/dev/null | grep -q arm_mirror; then
    note "  PASS  arm_mirror is up"
else
    note "  FAIL  arm_mirror is not running — start terminal A with mirror:=true"
    exit 1
fi

rule "2. teleop, and the three safety behaviours"
cat <<'EOF'
In terminal B, with a hand ready to hit SPACE:

  a) hold one joint's key and watch the arm follow smoothly
  b) keep holding past the joint's soft limit — it must stop at the limit
  c) release the key mid-move — it must stop within a fraction of a second
  d) press SPACE — the arm must go limp immediately (PANIC latches)
  e) press z to clear, then drive again — it must follow from where it is
EOF
ask "(a) the arm followed the leader smoothly"
ask "(b) it stopped at the soft limit and went no further"
ask "(c) releasing the key halted it"
ask "(d) SPACE dropped torque and the arm went limp"
ask "(e) clearing the panic resumed following without a jump"

rule "3. record an episode"
echo "Teleop a simple motion in terminal B while this records."
ros2 run mote_arm episode_record --task "${TASK:-move the arm through a simple motion}" \
    --dataset "$DATASET" --episodes 1 2>&1 | tee -a "$REPORT"

rule "4. check the capture"
if python3 "$HERE/../test/teleop_loop/check_capture.py" "$CAPTURE" 2>&1 | tee -a "$REPORT"; then
    note "  PASS  capture holds a real motion"
else
    note "  FAIL  capture check"
    FAILURES=$((FAILURES + 1))
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
echo "Stop terminal B (x) before replaying — two publishers would fight."
read -r -p "  virtual leader stopped? [y/N] " stopped
if [[ "$stopped" =~ ^[Yy] ]]; then
    ros2 run mote_arm episode_replay "$CAPTURE" --episode 0 --speed-scale 0.25 2>&1 | tee -a "$REPORT"
    ask "the arm retraced the recorded motion"
else
    note "  SKIP  replay (leader still running)"
    FAILURES=$((FAILURES + 1))
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

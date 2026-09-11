#!/usr/bin/env bash
# Paired CPU/RSS measurement of camera_monitor builds against one synthetic
# 30 Hz camera. Run inside the pixi default env from the repo root:
#
#   pixi run bash docs/tuning/2026-09-11-camera-monitor/run.sh [duration_s] [out_dir]
#
# Every variant runs at the same instant against the same stream, each under a
# script name of its own so overhead.py --cmdline-names can tell them apart:
#
#   cm_before    the raw-image build (git HEAD before this change), on /image_raw
#   cm_after     this build: /image_raw/compressed + a one-shot /camera_info read
#   cm_standing  this build with the /camera_info subscription left standing
#   cm_idle      the raw-image build on a topic nothing publishes
set -euo pipefail

DURATION=${1:-180}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
OUT=${2:-$HERE}
mkdir -p "$OUT"
WORK=$(mktemp -d)
trap 'kill $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true; rm -rf "$WORK"' EXIT

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

SRC=mote_perception/mote_perception/camera_monitor.py
BASE=${BASE:-f440e46}  # the commit before the compressed-stream change
git -C "$REPO" show "$BASE:$SRC" > "$WORK/cm_before.py"
cp "$REPO/$SRC" "$WORK/cm_after.py"
python - "$WORK/cm_after.py" "$WORK/cm_standing.py" <<'EOF'
import sys
src = open(sys.argv[1]).read()
cut = "        self.destroy_subscription(self._info_sub)\n        self._info_sub = None\n"
assert cut in src, "one-shot read not found"
open(sys.argv[2], "w").write(src.replace(cut, ""))
EOF
cp "$WORK/cm_before.py" "$WORK/cm_idle.py"

python "$HERE/camera_stream.py" > "$OUT/camera_stream.log" 2>&1 &
sleep 2

python "$WORK/cm_before.py" --ros-args -r __node:=cm_before \
    -r image:=/image_raw > "$OUT/cm_before.log" 2>&1 &
python "$WORK/cm_after.py" --ros-args -r __node:=cm_after \
    -r image/compressed:=/image_raw/compressed > "$OUT/cm_after.log" 2>&1 &
python "$WORK/cm_standing.py" --ros-args -r __node:=cm_standing \
    -r image/compressed:=/image_raw/compressed > "$OUT/cm_standing.log" 2>&1 &
python "$WORK/cm_idle.py" --ros-args -r __node:=cm_idle \
    -r image:=/no_such_image > "$OUT/cm_idle.log" 2>&1 &
sleep 3

samplers=()
for v in cm_before cm_after cm_standing cm_idle; do
    python "$REPO/mote_simulation/tools/benchmark/overhead.py" \
        --names none --min-procs 1 --cmdline-names "$v.py" \
        --duration "$DURATION" --interval 5 --out "$WORK/$v" > /dev/null &
    samplers+=($!)
done
uptime > "$OUT/load.txt"
wait "${samplers[@]}"
uptime >> "$OUT/load.txt"

for v in cm_before cm_after cm_standing cm_idle; do
    cp "$WORK/$v/overhead.csv" "$OUT/$v.csv"
    cp "$WORK/$v/overhead.json" "$OUT/$v.json"
done

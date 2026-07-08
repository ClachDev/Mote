#!/usr/bin/env bash
# Build a committed sim site by mapping a world with SLAM, the same way the
# robot does: launch the mapping mission headless, drive autonomous frontier
# coverage (explore.py), then save-map into the world's site under the
# in-repo sim MOTE_HOME. One site per world, floor "ground".
#
# Must run in the sim pixi env:
#   pixi run -e sim -- bash mote_simulation/tools/map_world.sh <world.sdf> [budget_s]
#
# Idempotent: re-running adds a new map revision to the same site.
set -u

WORLD="${1:?usage: map_world.sh <world.sdf> [budget_s]}"
BUDGET="${2:-900}"
STEM="${WORLD%.sdf}"

ROOT="${PIXI_PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
export MOTE_HOME="$ROOT/mote_simulation/sim_home"
SIM_DIR="$ROOT/mote_simulation"
LOG="$(mktemp -t mote_map_"$STEM".XXXXXX.log)"
SIM_PID=""

cleanup() {
    # Kill the launch's whole process group (gz, bridge, controllers, slam, nav).
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    sleep 2
    # Belt-and-suspenders for stragglers. Match the gz server and slam node by
    # name — NOT "$WORLD", which also appears in this script's own argv and
    # would make pkill -9 kill the orchestrator itself.
    pkill -9 -f 'gz sim' 2>/dev/null
    pkill -9 -f 'async_slam_toolbox_node' 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    return 0
}
# Only on interrupt/termination; the normal and fail paths call cleanup + exit
# explicitly so the script's exit code is never masked by cleanup's last command.
trap 'cleanup; exit 130' INT TERM

fail() { echo "FAIL: $1"; [ -n "${2:-}" ] && tail -30 "$2"; cleanup; exit 1; }

echo "=== mapping $WORLD -> site '$STEM' (budget ${BUDGET}s, MOTE_HOME=$MOTE_HOME) ==="

# Ensure the site + floor exist and seed zones from the world's zones file, so
# the bundle is self-contained and the fetch task has targets during mapping.
python3 - "$STEM" "$SIM_DIR" <<'PY' || fail "site setup failed"
import shutil, sys
from pathlib import Path
from mote_bringup import sites

stem, sim_dir = sys.argv[1], Path(sys.argv[2])
if stem not in sites.list_sites():
    sites.create(stem, "ground")
sites._seed_floor(stem, "ground")
sites.set_active(stem, "ground")
src = sim_dir / "worlds" / f"{stem}.zones.yaml"
if src.exists():
    shutil.copyfile(src, sites.floor_dir(stem, "ground") / "zones.yaml")
    print(f"seeded zones from {src.name}")
else:
    print(f"WARNING: no zones file {src}", file=sys.stderr)
PY

ros2 daemon stop >/dev/null 2>&1
sleep 1

echo ">> launching mapping sim..."
setsid ros2 launch mote_simulation sim_launch.py mode:=mapping world:="$WORLD" \
    > "$LOG" 2>&1 &
SIM_PID=$!
for _ in $(seq 90); do
    grep -q "Configured and activated diff_drive_controller" "$LOG" && break
    grep -q "Failed to load system plugin" "$LOG" && fail "gz_ros2_control plugin failed" "$LOG"
    kill -0 "$SIM_PID" 2>/dev/null || fail "sim exited early" "$LOG"
    sleep 2
done
grep -q "Configured and activated diff_drive_controller" "$LOG" \
    || fail "controllers never activated" "$LOG"
echo ">> controllers active; waiting for slam_toolbox..."
for _ in $(seq 45); do
    ros2 node list 2>/dev/null | grep -q slam_toolbox && break
    sleep 2
done
ros2 node list 2>/dev/null | grep -q slam_toolbox || fail "slam_toolbox never came up" "$LOG"

echo ">> exploring (budget ${BUDGET}s)..."
python3 -u "$SIM_DIR/tools/explore.py" --budget "$BUDGET" || fail "explore.py failed"

echo ">> saving map into site '$STEM'..."
python3 - <<'PY' || fail "save-map failed"
from mote_bringup import sites
# clean=False: sim maps are already clean ground truth; the FFT declutter pass
# (for real-sensor noise) would strip the thin true walls.
sites.save_map(clean=False)
PY

echo ">> done: $(python3 -c 'from mote_bringup import sites; sites.cmd_info()')"
echo "SITE BUILT: $STEM"
cleanup
exit 0

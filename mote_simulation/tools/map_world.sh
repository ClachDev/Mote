#!/usr/bin/env bash
# Build a committed sim site by mapping a world with SLAM, the same way the
# robot does: launch the mapping mission headless, drive autonomous frontier
# coverage (mote_bringup's explore, the same tool the robot runs), then
# save-map into the world's site under the in-repo sim MOTE_HOME. One site per
# world, floor "ground".
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
SIM_SID=""

cleanup() {
    # Kill the launch's whole process group (gz, bridge, controllers, slam, nav).
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    sleep 2
    # Belt-and-suspenders for stragglers, scoped to the session setsid gave the
    # launch: it reaps them whatever they are called and can never touch another
    # worktree's sim, which a bare 'gz sim' / 'async_slam_toolbox_node' match
    # did. (Never match "$WORLD" — it is in this script's own argv, so pkill -9
    # would kill the orchestrator itself.)
    [ -n "$SIM_SID" ] && pkill -9 -s "$SIM_SID" 2>/dev/null
    pkill -9 -f "gz sim.*$ROOT" 2>/dev/null
    # Daemons are per-domain, so this stops ours and leaves other runs' alone.
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
import sys
from pathlib import Path
from mote_bringup import bundle, sites

stem, sim_dir = sys.argv[1], Path(sys.argv[2])
if stem not in sites.list_sites():
    sites.create(stem, "ground")
sites._seed_floor(stem, "ground")
sites.set_active(stem, "ground")
src = sim_dir / "worlds" / f"{stem}.zones.yaml"
if src.exists():
    # The world file is a combined zones.yaml — one file is the right shape for
    # a fixture with exactly one robot in it. Read it through the migration and
    # write the split pair, so the sim site is the same shape as a real floor.
    floor = sites.floor_dir(stem, "ground")
    bundle.write_floor(
        floor, bundle.read_floor(src, stem, "ground"), site=stem, floor="ground"
    )
    print(f"seeded zones from {src.name}")
else:
    print(f"WARNING: no zones file {src}", file=sys.stderr)
PY

# A graph of our own, so a concurrent benchmark or smoke test can never feed
# this SLAM session foreign scans. An inherited domain/partition is respected.
DOMAIN_ENV="$(python3 "$SIM_DIR/tools/sim_domain.py" --shell --prefix mote-map)" \
    || fail "could not claim a ROS domain"
eval "$DOMAIN_ENV"
echo ">> ROS_DOMAIN_ID=$ROS_DOMAIN_ID (${MOTE_DOMAIN_HOW:-unknown}), GZ_PARTITION=$GZ_PARTITION"

ros2 daemon stop >/dev/null 2>&1
sleep 1

echo ">> launching mapping sim..."
setsid ros2 launch mote_simulation sim_launch.py mode:=mapping world:="$WORLD" \
    > "$LOG" 2>&1 &
SIM_PID=$!
SIM_SID="$(ps -o sid= -p "$SIM_PID" 2>/dev/null | tr -d ' ')"
# If setsid did not detach it, the launch shares OUR session and killing that
# session would kill this script (and its caller) — drop the scope instead.
[ "$SIM_SID" = "$(ps -o sid= -p $$ | tr -d ' ')" ] && SIM_SID=""
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
ros2 run mote_bringup explore --sim-time --budget "$BUDGET" || fail "explore failed"

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

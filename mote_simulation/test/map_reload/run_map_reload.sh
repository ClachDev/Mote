#!/usr/bin/env bash
# Sim acceptance for loading a promoted map into a running Nav2 (~2 min with a
# GPU). Copies the committed sim MOTE_HOME to a temp dir, adds a second
# mote_world revision (the first with a small obstacle drawn in, so /map
# visibly changes), brings up `sim_launch.py mode:=nav` on the first, and runs
# verify_map_reload.py, which installs the second the way the fleet agent does
# and asserts it is served, the pose survives, and a goto succeeds.
#
# Must run inside the 'sim' pixi environment:  pixi run sim-map-reload-test
# Isolated like the smoke test: its own ROS_DOMAIN_ID + GZ_PARTITION, and a
# teardown scoped to its own process session and this worktree's path.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VERIFY="$SCRIPT_DIR/verify_map_reload.py"
WORLD="${WORLD:-mote_world}"
NEW_REV="${NEW_REV:-20990101T000000}"

WORK="$(mktemp -d -t mote_map_reload.XXXXXX)"
SIM_LOG="$WORK/sim.log"
SIM_PID=""
SIM_SID=""

cleanup() {
    [ -n "$SIM_PID" ] && kill -- -"$SIM_PID" 2>/dev/null
    sleep 2
    [ -n "$SIM_SID" ] && pkill -9 -s "$SIM_SID" 2>/dev/null
    pkill -9 -f "gz sim.*$ROOT" 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    echo ">> logs kept in $WORK"
    true
}
trap cleanup EXIT

fail() { echo "FAIL: $1"; [ -n "${2:-}" ] && tail -40 "$2"; exit 1; }

cp -r "$ROOT/mote_simulation/sim_home" "$WORK/home"
rm -f "$WORK/home/active.yaml"
export MOTE_HOME="$WORK/home"
FLOOR="$MOTE_HOME/sites/$WORLD/floors/ground"
OLD_REV="$(basename "$(readlink "$FLOOR/map")")"
cp -r "$FLOOR/maps/$OLD_REV" "$FLOOR/maps/$NEW_REV"
python3 - "$FLOOR/maps/$NEW_REV" <<'EOF' || fail "could not draw the second revision"
import sys
from pathlib import Path

import yaml
from PIL import Image

rev = Path(sys.argv[1])
meta = yaml.safe_load((rev / "map.yaml").read_text())
image = Image.open(rev / meta["image"])
res, (ox, oy, _) = meta["resolution"], meta["origin"]
# A 0.2 m block at (-2.0, -0.5): free space in mote_world, clear of every zone.
col = int((-2.0 - ox) / res)
row = image.height - int((-0.5 - oy) / res)
free = [image.getpixel((c, r)) for c in range(col, col + 4) for r in range(row, row + 4)]
if not all(p > 250 for p in free):
    sys.exit(f"block at {col},{row} is not in free space: {free}")
for c in range(col, col + 4):
    for r in range(row, row + 4):
        image.putpixel((c, r), 0)
image.save(rev / meta["image"])
print(f"drew a 4x4 obstacle at pixel ({col}, {row}) in {rev.name}")
EOF
echo ">> $WORLD: serving $OLD_REV, will promote $NEW_REV (MOTE_HOME=$MOTE_HOME)"

DOMAIN_ENV="$(python3 "$ROOT/mote_simulation/tools/sim_domain.py" --shell --prefix mote-mapreload)" \
    || fail "could not claim a ROS domain"
eval "$DOMAIN_ENV"
echo ">> ROS_DOMAIN_ID=$ROS_DOMAIN_ID (${MOTE_DOMAIN_HOW:-unknown}), GZ_PARTITION=$GZ_PARTITION"
ros2 daemon stop >/dev/null 2>&1
sleep 1

echo ">> launching sim (mode:=nav world:=$WORLD.sdf, record:=false)..."
setsid ros2 launch mote_simulation sim_launch.py mode:=nav "world:=$WORLD.sdf" record:=false \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
SIM_SID="$(ps -o sid= -p "$SIM_PID" 2>/dev/null | tr -d ' ')"
[ "$SIM_SID" = "$(ps -o sid= -p $$ | tr -d ' ')" ] && SIM_SID=""
for _ in $(seq 90); do
    grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" && break
    kill -0 "$SIM_PID" 2>/dev/null || fail "sim process exited early" "$SIM_LOG"
    sleep 2
done
grep -q "Configured and activated diff_drive_controller" "$SIM_LOG" \
    || fail "diff_drive_controller never activated" "$SIM_LOG"
echo "STEP0 OK: controllers active"

timeout 600 python3 "$VERIFY" --site "$WORLD" --new "$NEW_REV" \
    | tee "$WORK/verify.log"
status=${PIPESTATUS[0]}
grep -E "map_reloader|task_server\]: Zones" "$SIM_LOG" > "$WORK/reloader.log"
echo ">> map_reloader's log:"
cat "$WORK/reloader.log"
[ "$status" -eq 0 ] || fail "verify_map_reload.py assertions failed" "$SIM_LOG"

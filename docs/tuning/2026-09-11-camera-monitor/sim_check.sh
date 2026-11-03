#!/usr/bin/env bash
# Sim acceptance: camera_monitor, started by perception_launch.py as
# `pixi run perception` starts it, reports the sim camera's real rate through
# sim_launch.py's compressed republisher. From the repo root:
#
#   pixi run -e sim bash docs/tuning/2026-09-11-camera-monitor/sim_check.sh [out_dir]
#
# Set OVERLAY to a colcon install/ to test a build other than the activated one.
HERE=$(cd "$(dirname "$0")" && pwd)
WT=$(cd "$HERE/../../.." && pwd)
OUT=${1:-$HERE/sim}
mkdir -p "$OUT"
if [ -n "${OVERLAY:-}" ]; then
    # shellcheck source=/dev/null
    source "$OVERLAY/local_setup.bash"
fi
set -uo pipefail
eval "$(python3 "$WT/mote_simulation/tools/sim_domain.py" --shell --prefix mote-cm)"
echo "domain=$ROS_DOMAIN_ID partition=$GZ_PARTITION ($MOTE_DOMAIN_HOW)" | tee "$OUT/domain.txt"
echo "sim_launch from: $(ros2 pkg prefix mote_simulation)" | tee -a "$OUT/domain.txt"
echo "camera_monitor from: $(ros2 pkg prefix mote_perception)" | tee -a "$OUT/domain.txt"

setsid ros2 launch mote_simulation sim_launch.py > "$OUT/sim.log" 2>&1 &
SIM=$!
cleanup() {
    kill -INT -- -"$SIM" 2>/dev/null; kill -INT -- -"${PERC:-0}" 2>/dev/null
    sleep 5
    kill -KILL -- -"$SIM" 2>/dev/null; kill -KILL -- -"${PERC:-0}" 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 60); do
    ros2 topic list 2>/dev/null | grep -qx /image_raw && break
    sleep 2
done
sleep 10

{
    echo "--- before perception: who publishes/subscribes /image_raw"
    ros2 topic info /image_raw
    ros2 topic info /image_raw/compressed
} > "$OUT/graph.txt" 2>&1

setsid ros2 launch mote_perception perception_launch.py use_sim_time:=true \
    > "$OUT/perception.log" 2>&1 &
PERC=$!

# Ground truth: the bridge's raw frames, rate taken from their (sim-time) stamps,
# against the /clock rate so the real-time factor is on record too.
python3 - "$OUT/truth.txt" <<'EOF'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

rclpy.init()
n = Node("rate_truth")
stamps, clocks = [], []
n.create_subscription(Image, "/image_raw", lambda m: stamps.append(
    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), qos_profile_sensor_data)
n.create_subscription(Clock, "/clock", lambda m: clocks.append(
    (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
end = time.monotonic() + 40
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
frames_per_sim_s = (len(stamps) - 1) / (stamps[-1] - stamps[0])
rtf = (clocks[-1][1] - clocks[0][1]) / (clocks[-1][0] - clocks[0][0])
line = (f"/image_raw: {len(stamps)} frames over {stamps[-1] - stamps[0]:.1f} sim s "
        f"= {frames_per_sim_s:.2f} frames per sim s; real-time factor {rtf:.2f}")
print(line)
open(sys.argv[1], "w").write(line + "\n")
EOF

{
    echo "--- with perception up"
    ros2 topic info /image_raw
    ros2 topic info /image_raw/compressed
} >> "$OUT/graph.txt" 2>&1
grep -h "camera_monitor" "$OUT/perception.log" | tail -n 8

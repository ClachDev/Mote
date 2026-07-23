#!/usr/bin/env bash
set -euo pipefail

# Start the off-board depth server plus the ROS obstacle node on a workstation
# sharing the robot's DDS graph. With no argument the server runs on CPU
# (`depth-server` in the `depth` env); pass `rocm` to run it on the GPU
# (`depth-server-rocm` in the `depth-rocm` env, which carries the AMD iGPU
# activation env). The server auto-falls back to CPU if no usable GPU is found.
server_task="depth-server"
if [[ "${1:-}" == "rocm" ]]; then
  server_task="depth-server-rocm"
fi

# This script runs in the default/ROS env, so PYTHONPATH points at the ROS
# Python 3.12 site-packages. Drop it for the depth-server child only, or its
# Python loads those incompatible numpy/torch C-extensions. The ROS node
# below keeps PYTHONPATH — it needs it.
env -u PYTHONPATH pixi run "$server_task" &
server_pid=$!

cleanup() {
  kill "$server_pid" 2>/dev/null || true
  wait "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run mote_perception depth_obstacle_node \
  --ros-args -r image/compressed:=/image_raw/compressed -p server_host:=127.0.0.1

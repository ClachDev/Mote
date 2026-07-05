#!/usr/bin/env bash
set -euo pipefail

# This script runs in the default/ROS env, so PYTHONPATH points at the ROS
# Python 3.12 site-packages. Drop it for the detect-server child only, or its
# Python 3.14 loads those incompatible numpy/torch C-extensions. The ROS node
# below keeps PYTHONPATH — it needs it.
env -u PYTHONPATH pixi run detect-server &
server_pid=$!

cleanup() {
  kill "$server_pid" 2>/dev/null || true
  wait "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run mote_perception object_detector_node \
  --ros-args -r image/compressed:=/image_raw/compressed -p server_host:=127.0.0.1

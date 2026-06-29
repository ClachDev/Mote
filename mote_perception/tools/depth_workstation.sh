#!/usr/bin/env bash
set -euo pipefail

pixi run depth-server &
server_pid=$!

cleanup() {
  kill "$server_pid" 2>/dev/null || true
  wait "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run mote_perception depth_obstacle_node \
  --ros-args -r image/compressed:=/image_raw/compressed -p server_host:=127.0.0.1

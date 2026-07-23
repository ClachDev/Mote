#!/usr/bin/env bash
set -euo pipefail

# Runs both inference servers together on the "inference" machine (the pixi
# inference env: torch, no ROS). The ROS depth/detect nodes run on the robot and
# reach these over TCP at inference_host (see perception.yaml). This runs directly
# in the torch env, so — unlike the old per-model launchers that shelled out from
# the ROS env — there is no PYTHONPATH to drop. If either server dies, tear both
# down so the failure is visible rather than half-served.
#
# --host 0.0.0.0: this launcher's whole purpose is a dedicated inference machine,
# so bind all interfaces — the robot connects over the LAN at inference_host. (The
# standalone depth-server/detect-server tasks keep the 127.0.0.1 default for the
# single-machine dev case.) The wire protocol is unauthenticated: run it on a
# trusted network.
python -u mote_perception/tools/depth_server.py --host 0.0.0.0 &
depth_pid=$!
python -u mote_perception/tools/detect_server.py --host 0.0.0.0 &
detect_pid=$!

cleanup() {
  kill "$depth_pid" "$detect_pid" 2>/dev/null || true
  wait "$depth_pid" "$detect_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n

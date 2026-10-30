#!/usr/bin/env bash
# Calibrate a camera's intrinsics with the camera plugged into this machine.
#
#   pixi run camera-calibrate [--device DEV] [--out FILE] [--size 6x9] [--square 0.025]
#
# Bench prep (docs/getting-started.md): the camera does not have to be on the
# robot for its intrinsics, so nothing here touches the robot or the LAN. A
# v4l2 camera node and the calibrator GUI run on this machine, on a ROS domain
# of their own and loopback only, and both are torn down on exit. In the GUI,
# move the printed target (mote_perception/config/README.md) until the X, Y,
# Size and Skew bars fill, press CALIBRATE, then SAVE, then close the window.
# The saved calibration is extracted to --out (default ./camera_calibration.yaml)
# in the camera_info format the robot loads, ready for
# `pixi run provision --camera-calibration FILE` or for ~/.mote on the robot.
#
# --device defaults to /dev/mote_camera when the udev rules are installed here
# (`pixi run setup udev`), else /dev/video0.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_INFO="$HERE/../config/camera_info.default.yaml"
TARBALL=/tmp/calibrationdata.tar.gz   # where cameracalibrator's SAVE writes

DEVICE=""
OUT="camera_calibration.yaml"
SIZE="6x9"
SQUARE="0.025"

usage() {
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --device) DEVICE="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --size) SIZE="$2"; shift 2 ;;
        --square) SQUARE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) echo "unknown argument: $1" >&2; usage 1 ;;
    esac
done

if [ -z "$DEVICE" ]; then
    if [ -e /dev/mote_camera ]; then DEVICE=/dev/mote_camera; else DEVICE=/dev/video0; fi
fi
if [ ! -e "$DEVICE" ]; then
    echo "no camera at $DEVICE: plug it in, or pass --device" >&2
    exit 1
fi

# The camera name camera_info_manager expects, taken from the packaged default
# so the robot loads this file without a name-mismatch warning.
CAMERA_NAME="$(sed -n 's/^camera_name:[[:space:]]*//p' "$DEFAULT_INFO" | tr -d "\"' " | head -1)"
CAMERA_NAME="${CAMERA_NAME:-mote_camera}"

# A domain of its own, loopback only: this must never join a robot's graph or a
# sim running beside it.
export ROS_DOMAIN_ID=$((RANDOM % 40 + 60))
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

LOG="$(mktemp "${TMPDIR:-/tmp}/mote-camera-calibrate.XXXXXX")"
CAM_PID=""
cleanup() {
    # `ros2 run` does not forward SIGTERM (CLAUDE.md, "Stray ROS processes"):
    # the camera node was started in a session of its own and is reaped by
    # process group. Bash runs no EXIT trap when a signal kills it, so INT and
    # TERM are trapped as well; otherwise Ctrl-C leaves the node running.
    if [ -n "$CAM_PID" ]; then
        kill -- -"$CAM_PID" 2>/dev/null || kill "$CAM_PID" 2>/dev/null || true
        wait "$CAM_PID" 2>/dev/null || true
        CAM_PID=""
    fi
    ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

echo "==> camera: $DEVICE -> /image_raw (log: $LOG)"
setsid ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="$DEVICE" >"$LOG" 2>&1 &
CAM_PID=$!

for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx /image_raw; then break; fi
    if ! kill -0 "$CAM_PID" 2>/dev/null; then
        echo "camera node exited:" >&2
        tail -n 20 "$LOG" >&2
        exit 1
    fi
    sleep 0.5
done
if ! ros2 topic list 2>/dev/null | grep -qx /image_raw; then
    echo "no /image_raw after 15 s:" >&2
    tail -n 20 "$LOG" >&2
    exit 1
fi

STARTED="$(date +%s)"
echo "==> calibrator: fill the X/Y/Size/Skew bars, press CALIBRATE, then SAVE, then close the window"
ros2 run camera_calibration cameracalibrator --size "$SIZE" --square "$SQUARE" \
    --camera_name "$CAMERA_NAME" --no-service-check --ros-args -r image:=/image_raw || true

if [ ! -f "$TARBALL" ] || [ "$(stat -c %Y "$TARBALL")" -lt "$STARTED" ]; then
    echo "nothing saved this run (press SAVE after CALIBRATE); $OUT is unchanged" >&2
    exit 1
fi
tar -xOf "$TARBALL" ost.yaml >"$OUT"
echo
echo "==> wrote $OUT"
echo "    new robot:      pixi run provision ... --camera-calibration $OUT"
echo "    running robot:  scp $OUT <robot>:~/.mote/camera_calibration.yaml"

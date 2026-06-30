"""Offline replay of the depth pipeline against a recorded bag.

Re-runs the exact off-board depth path on recorded footage: forwards each sampled
`/image_raw/compressed` frame to a running depth server, rescales the returned
depth against the time-nearest `/scan_filtered` via the body-fixed lidar->camera
transform (from the bag's `/tf_static`), and reports per-frame what the live node
would compute -- raw model depth stats, the lidar (pred,true) pair count and depth
spread, the fitted affine (a, b) and inlier %, and the rescaled depth range. It
saves colorized raw and corrected depth maps (TURBO) for eyeballing.

This is the rig used to find the RANSAC bistability in the lidar fit: a frame whose
fit collapses prints `DEGENERATE` (slope a < 0.5, i.e. the near-flat/inverted line),
and its corrected map will look inverted. Use it to inspect any future "depth goes
to noise" bag without needing the robot.

Needs a depth server already listening (separate terminal, in the depth env):
    pixi run depth                 # runs server + live node; or just the server:
    env -u PYTHONPATH pixi run depth-server

Then, in the dev/default env (needs cv2 + rosbag2_py):
    pixi run python mote_perception/tools/depth_bag_replay.py <bag_dir> [--frames N] [--out DIR]
"""

import argparse
import socket
import struct
import sys
import tempfile

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from tf2_msgs.msg import TFMessage

from mote_perception.ground_projection import chain_static_transforms
from mote_perception.depth_rescale import fit_affine_disparity
from mote_perception.lidar_rescale import LidarDepthRescaler

A_MIN = 0.5  # below this the lidar fit has collapsed to the degenerate line


def recvall(s, n):
    buf = bytearray()
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            return None
        buf.extend(c)
    return bytes(buf)


def infer(jpeg, host, port):
    """Send one JPEG to the depth server and return the float32 depth map."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((host, port))
    except OSError as e:
        sys.exit(
            f"could not reach depth server at {host}:{port} ({e}); "
            f"start it with `env -u PYTHONPATH pixi run depth-server`"
        )
    s.sendall(struct.pack(">I", len(jpeg)) + jpeg)
    hdr = recvall(s, 8)
    if hdr is None:
        sys.exit("depth server closed without replying (check its log for a traceback)")
    h, w = struct.unpack(">II", hdr)
    body = recvall(s, h * w * 4)
    s.close()
    return np.frombuffer(body, np.float32).reshape(h, w)


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    imgs, scans, tf_static, caminfo = [], [], None, None
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/image_raw/compressed":
            imgs.append((t, bytes(deserialize_message(data, CompressedImage).data)))
        elif topic == "/scan_filtered":
            scans.append((t, deserialize_message(data, LaserScan)))
        elif topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, TFMessage)
        elif topic == "/camera_info" and caminfo is None:
            caminfo = deserialize_message(data, CameraInfo)
    if not imgs or not scans or tf_static is None or caminfo is None:
        sys.exit(
            "bag is missing one of /image_raw/compressed /scan_filtered "
            "/tf_static /camera_info -- record the `perception` stream"
        )
    return imgs, scans, tf_static, caminfo


def colorize(d, vmax):
    d = np.clip(np.nan_to_num(d), 0, vmax)
    return cv2.applyColorMap((255 * d / vmax).astype(np.uint8), cv2.COLORMAP_TURBO)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="bag directory (mcap)")
    ap.add_argument("--frames", type=int, default=6, help="frames to sample, evenly")
    ap.add_argument(
        "--out", default=None, help="dir for colorized PNGs (default: temp)"
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="depth_replay_")

    imgs, scans, tf_static, caminfo = load(args.bag)
    print(f"{len(imgs)} imgs, {len(scans)} scans; writing PNGs to {out}")

    # T_optical_scan = inv(T_base_optical) @ T_base_scan (camera and lidar are
    # siblings under base_footprint), built from the bag's static transforms.
    T_bo = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    T_bs = chain_static_transforms(
        tf_static.transforms, "lidar_scan_link", "base_footprint"
    )
    resc = LidarDepthRescaler(caminfo.k, caminfo.d, np.linalg.inv(T_bo) @ T_bs)

    for k, i in enumerate(np.linspace(0, len(imgs) - 1, args.frames).astype(int)):
        ts, jpeg = imgs[i]
        depth = infer(jpeg, args.host, args.port)
        _, scan = min(scans, key=lambda s: abs(s[0] - ts))
        pred, true = resc.pairs(scan, depth)
        rd = depth[np.isfinite(depth)]
        line = (
            f"[f{k:>2}] raw {rd.min():.2f}-{rd.max():.2f}m finite "
            f"{100 * np.isfinite(depth).mean():.0f}%  pairs {len(pred)}"
        )
        corr = None
        if len(pred) >= 8:
            a, b, frac = fit_affine_disparity(pred, true, a_min=A_MIN)
            tag = "  DEGENERATE" if a < A_MIN else ""
            line += (
                f" spread {np.ptp(true):.2f}m  a={a:.3f} b={b:.3f} inl {frac:.0%}{tag}"
            )
            out_fit = resc.rescale(depth, scan)
            if out_fit is not None:
                corr = out_fit[0]
                cf = corr[np.isfinite(corr)]
                line += (
                    f"  -> corr {cf.min():.2f}-{cf.max():.2f}m "
                    f"med {np.median(cf):.2f} >20m {100 * (cf > 20).mean():.1f}%"
                )
        print(line)
        cv2.imwrite(f"{out}/f{k:02d}_raw.png", colorize(depth, 8.0))
        if corr is not None:
            cv2.imwrite(f"{out}/f{k:02d}_corr.png", colorize(corr, 5.0))


if __name__ == "__main__":
    main()

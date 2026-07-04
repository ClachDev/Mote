"""Shared bag-loading helpers for the offline perception tools.

Dev-env only (needs rosbag2_py). The tools import this as a sibling module — the
tools directory is sys.path[0] when a tool runs as a script.
"""

import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from tf2_msgs.msg import TFMessage

from mote_perception.ground_projection import chain_static_transforms


def load_perception_bag(bag):
    """All frames + scans and the static context from a `pixi run record` bag.

    Returns (imgs, scans, tf_static, caminfo): imgs as (recv_stamp_ns, jpeg bytes),
    scans as (recv_stamp_ns, LaserScan). Exits with a message if a topic is missing.
    """
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


def base_transforms(tf_static, scans):
    """(T_base_optical, T_base_scan) from a bag's static transforms.

    The scan frame is taken from the scans themselves — on this robot it is yawed
    90 degrees from base, so plotting raw scan coordinates without T_base_scan is
    not an approximation, it is wrong.
    """
    T_bo = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    T_bs = chain_static_transforms(
        tf_static.transforms, scans[0][1].header.frame_id, "base_footprint"
    )
    return T_bo, T_bs


def nearest_scan(scans, stamp):
    """The buffered (stamp, LaserScan) pair nearest a capture stamp."""
    return min(scans, key=lambda s: abs(s[0] - stamp))


def colorize(depth, vmax):
    """Colorized (TURBO) uint8 render of a depth map, clipped to [0, vmax] m."""
    d = np.clip(np.nan_to_num(depth), 0, vmax)
    return cv2.applyColorMap((255 * d / vmax).astype(np.uint8), cv2.COLORMAP_TURBO)

"""Named navigation targets loaded from a zones YAML file.

Format:

    frame_id: map
    zones:
      pickup: {x: 1.8, y: -1.5, yaw: 0.0}
      dropoff: {x: -1.8, y: 1.5}

Yaw is radians about +z and defaults to 0.
"""

import math
import os
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def append_zone(path, name: str, x: float, y: float, yaw: float) -> bool:
    """Write/replace one zone in the file, creating the file if needed.

    Returns True if an existing zone was replaced.
    """
    path = Path(path).expanduser()
    data = (
        yaml.safe_load(path.read_text()) or {} if path.exists() else {"frame_id": "map"}
    )
    if not isinstance(data.get("zones"), dict):
        data["zones"] = {}
    replaced = name in data["zones"]
    data["zones"][name] = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)}
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    os.replace(tmp, path)
    return replaced


def load_zones(path: str) -> dict[str, PoseStamped]:
    data = yaml.safe_load(Path(path).expanduser().read_text())
    frame_id = data.get("frame_id", "map")
    zones = {}
    for name, spec in data["zones"].items():
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(spec["x"])
        pose.pose.position.y = float(spec["y"])
        yaw = float(spec.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        zones[name] = pose
    return zones

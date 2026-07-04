"""Named navigation targets loaded from a zones YAML file.

Format:

    frame_id: map
    zones:
      pickup: {x: 1.8, y: -1.5, yaw: 0.0}
      dropoff: {x: -1.8, y: 1.5}

Yaw is radians about +z and defaults to 0.
"""

import math
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped


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

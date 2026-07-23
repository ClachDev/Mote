"""Arm configuration and unit conversions.

The single source of truth is the ``arm:`` section of
``mote_description/config/robot.yaml``. This module parses it into typed specs
and provides the encoder<->radian conversions and soft-limit clamping used by
the driver, the jog CLI, and the bench check tool.

It deliberately imports nothing from ROS or the servo SDK so the safety-critical
maths (clamping, conversions, validation) can be unit-tested without hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import yaml

# STS3215 encoders report a single 12-bit turn: 4096 counts = 2*pi.
COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV


@dataclass(frozen=True)
class JointSpec:
    """One arm servo: its bus ID, soft limits, zero offset, and direction."""

    name: str
    id: int
    min_rad: float
    max_rad: float
    # Raw encoder count that corresponds to 0 rad (the joint's mechanical zero).
    # Taught at the bench; defaults to the servo mid-point.
    home_counts: int = COUNTS_PER_REV // 2
    # True if the joint's positive direction is opposite the servo's.
    invert: bool = False

    @property
    def sign(self) -> int:
        return -1 if self.invert else 1

    def clamp_rad(self, rad: float) -> float:
        """Clamp a commanded angle to the joint's soft limits."""
        return max(self.min_rad, min(self.max_rad, rad))

    def counts_to_rad(self, counts: int) -> float:
        """Convert a raw encoder reading to radians about the joint zero."""
        return self.sign * (counts - self.home_counts) * RAD_PER_COUNT

    def rad_to_counts(self, rad: float) -> int:
        """Convert a joint angle to a raw encoder goal, clamped to [0, 4095].

        The angle is *not* soft-clamped here — callers clamp with ``clamp_rad``
        first so that a limit breach is a deliberate, visible decision rather
        than a silent saturation at the encoder edge.
        """
        counts = round(self.home_counts + self.sign * rad / RAD_PER_COUNT)
        return max(0, min(COUNTS_PER_REV - 1, counts))


@dataclass(frozen=True)
class ArmConfig:
    """The arm bus and its joints, in servo-command order."""

    port: str
    baud_rate: int
    joints: tuple[JointSpec, ...]
    # Gentle defaults for jog moves; steps/s and 0-254 accel units.
    moving_speed: int = 600
    moving_acc: int = 20

    @property
    def ids(self) -> list[int]:
        return [j.id for j in self.joints]

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]

    def joint(self, name: str) -> JointSpec:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"no arm joint named {name!r}")

    @staticmethod
    def from_dict(cfg: dict) -> "ArmConfig":
        arm = cfg.get("arm")
        if arm is None:
            raise KeyError("robot.yaml has no 'arm' section")

        joints: list[JointSpec] = []
        for entry in arm["joints"]:
            spec = JointSpec(
                name=str(entry["name"]),
                id=int(entry["id"]),
                min_rad=float(entry["min"]),
                max_rad=float(entry["max"]),
                home_counts=int(entry.get("home", COUNTS_PER_REV // 2)),
                invert=bool(entry.get("invert", False)),
            )
            if spec.min_rad > spec.max_rad:
                raise ValueError(
                    f"joint {spec.name!r}: min {spec.min_rad} > max {spec.max_rad}"
                )
            joints.append(spec)

        if not joints:
            raise ValueError("arm has no joints")

        dup_ids = {
            i for i in (s.id for s in joints) if [s.id for s in joints].count(i) > 1
        }
        if dup_ids:
            raise ValueError(f"duplicate servo IDs in arm config: {sorted(dup_ids)}")
        dup_names = {
            n for n in (s.name for s in joints) if [s.name for s in joints].count(n) > 1
        }
        if dup_names:
            raise ValueError(
                f"duplicate joint names in arm config: {sorted(dup_names)}"
            )

        return ArmConfig(
            port=str(arm["port"]),
            baud_rate=int(arm["baud_rate"]),
            joints=tuple(joints),
            moving_speed=int(arm.get("moving_speed", 600)),
            moving_acc=int(arm.get("moving_acc", 20)),
        )

    @staticmethod
    def from_yaml_file(path: str) -> "ArmConfig":
        with open(path) as f:
            return ArmConfig.from_dict(yaml.safe_load(f))


def default_robot_yaml() -> str:
    """Locate robot.yaml in the installed mote_description share."""
    from ament_index_python.packages import get_package_share_directory

    return f"{get_package_share_directory('mote_description')}/config/robot.yaml"


def load() -> ArmConfig:
    """Load the arm config from the installed robot.yaml."""
    return ArmConfig.from_yaml_file(default_robot_yaml())

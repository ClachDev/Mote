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
import os
from dataclasses import dataclass, replace
from pathlib import Path

import yaml

# STS3215 encoders report a single 12-bit turn: 4096 counts = 2*pi.
COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV


@dataclass(frozen=True)
class JointSpec:
    """One arm servo: its bus ID, soft limits, zero offset, and direction.

    ``zero_counts`` is deliberately not called "home". "home" is the name of a
    taught rest pose in ``arm_poses.yaml``; after calibration 0 rad is the
    middle of the joint's travel, a different place entirely. Keeping the two
    words apart is the whole point of the name.
    """

    name: str
    id: int
    min_rad: float
    max_rad: float
    # Raw encoder count that corresponds to 0 rad. Set by `arm-setup calibrate`;
    # defaults to the servo mid-point.
    zero_counts: int = COUNTS_PER_REV // 2
    # True if the joint's positive direction is opposite the servo's.
    invert: bool = False

    @property
    def sign(self) -> int:
        return -1 if self.invert else 1

    def clamp_rad(self, rad: float) -> float:
        """Clamp a commanded angle to the joint's soft limits."""
        return max(self.min_rad, min(self.max_rad, rad))

    @property
    def reachable_min(self) -> float:
        """Lowest angle the 0-4095 goal register can actually express."""
        edge = (0 - self.zero_counts) * RAD_PER_COUNT * self.sign
        other = (COUNTS_PER_REV - 1 - self.zero_counts) * RAD_PER_COUNT * self.sign
        return max(self.min_rad, min(edge, other))

    @property
    def reachable_max(self) -> float:
        """Highest angle the 0-4095 goal register can actually express."""
        edge = (0 - self.zero_counts) * RAD_PER_COUNT * self.sign
        other = (COUNTS_PER_REV - 1 - self.zero_counts) * RAD_PER_COUNT * self.sign
        return min(self.max_rad, max(edge, other))

    @property
    def unreachable(self) -> str | None:
        """Why part of this joint's soft band cannot be commanded, if any.

        A goal is written to a 12-bit register, so an angle outside
        [0, 4095] counts about ``zero`` saturates at the edge instead of being
        refused. The joint then stops at the same angle every time, in one
        direction only, whatever the load — which looks like a stall and is not
        one. A soft band wider than the register can address is therefore a
        configuration error worth naming, not a limit to discover by driving
        into it.
        """
        lo, hi = self.reachable_min, self.reachable_max
        if lo <= self.min_rad and hi >= self.max_rad:
            return None
        return (
            f"joint {self.name!r}: soft limits [{self.min_rad:+.3f}, "
            f"{self.max_rad:+.3f}] but zero={self.zero_counts} leaves only "
            f"[{lo:+.3f}, {hi:+.3f}] addressable in the 0-{COUNTS_PER_REV - 1} "
            "goal register — re-run `pixi run arm-setup calibrate` to re-centre it"
        )

    def counts_to_rad(self, counts: int) -> float:
        """Convert a raw encoder reading to radians about the joint zero."""
        return self.sign * (counts - self.zero_counts) * RAD_PER_COUNT

    def rad_to_counts(self, rad: float) -> int:
        """Convert a joint angle to a raw encoder goal, clamped to [0, 4095].

        The angle is *not* soft-clamped here — callers clamp with ``clamp_rad``
        first so that a limit breach is a deliberate, visible decision rather
        than a silent saturation at the encoder edge.
        """
        counts = round(self.zero_counts + self.sign * rad / RAD_PER_COUNT)
        return max(0, min(COUNTS_PER_REV - 1, counts))


@dataclass(frozen=True)
class ServoGains:
    """Position-loop gains held in servo EEPROM (registers 21/22/23).

    Recorded in robot.yaml so they survive a servo swap; `arm-setup gains apply`
    writes them to the hardware. kp too low leaves a permanent steady-state
    error under load, since ki=0 never integrates the droop away.
    """

    kp: int = 32
    kd: int = 32
    ki: int = 0

    def __post_init__(self) -> None:
        for name in ("kp", "kd", "ki"):
            value = getattr(self, name)
            if not 0 <= value <= 254:
                raise ValueError(f"gain {name}={value} outside servo range 0-254")


@dataclass(frozen=True)
class ArmConfig:
    """The arm bus and its joints, in servo-command order."""

    port: str
    baud_rate: int
    joints: tuple[JointSpec, ...]
    # Gentle defaults for jog moves; steps/s and 0-254 accel units.
    moving_speed: int = 600
    moving_acc: int = 20
    gains: ServoGains = ServoGains()

    @property
    def ids(self) -> list[int]:
        return [j.id for j in self.joints]

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]

    @property
    def problems(self) -> list[str]:
        """Configuration faults worth warning about, but not worth refusing over.

        Reported rather than raised: a robot with one over-wide band should
        still come up and drive the rest of its joints.
        """
        return [p for p in (j.unreachable for j in self.joints) if p]

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
                # `home:` is the pre-calibration spelling, still accepted so an
                # un-migrated robot.yaml keeps working.
                zero_counts=int(
                    entry.get("zero", entry.get("home", COUNTS_PER_REV // 2))
                ),
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

        # The arm shares the serial bus with the drive wheels, so an arm servo
        # ID colliding with a wheel ID would send arm commands to a wheel (and
        # vice versa). Reject it here rather than discover it by driving away.
        drive = cfg.get("servos") or {}
        wheel_ids = {int(drive[key]) for key in ("left_id", "right_id") if key in drive}
        if wheel_ids and str(arm["port"]) == str(drive.get("port")):
            collisions = sorted(wheel_ids.intersection(s.id for s in joints))
            if collisions:
                raise ValueError(
                    f"arm servo IDs {collisions} collide with the drive wheel "
                    f"IDs on the shared bus {arm['port']} — reassign the arm "
                    "servos (see mote_hardware setup_ids)"
                )

        raw_gains = arm.get("gains") or {}
        gains = ServoGains(
            kp=int(raw_gains.get("kp", 32)),
            kd=int(raw_gains.get("kd", 32)),
            ki=int(raw_gains.get("ki", 0)),
        )

        return ArmConfig(
            port=str(arm["port"]),
            baud_rate=int(arm["baud_rate"]),
            joints=tuple(joints),
            moving_speed=int(arm.get("moving_speed", 600)),
            moving_acc=int(arm.get("moving_acc", 20)),
            gains=gains,
        )

    @staticmethod
    def from_yaml_file(path: str) -> "ArmConfig":
        with open(path) as f:
            return ArmConfig.from_dict(yaml.safe_load(f))


def calibration_path() -> Path:
    """Where this robot's measured arm calibration lives.

    Per-robot state under ``MOTE_HOME``, not packaged config: the zeros and
    limits are measurements of one physical arm, every robot's differ, and the
    package directory is read-only once installed from a conda channel. Same
    rule as ``~/.mote/camera_calibration.yaml`` and the site bundles.
    """
    return Path(os.environ.get("MOTE_HOME", "~/.mote")).expanduser() / "arm.yaml"


def apply_calibration(cfg: "ArmConfig", data: dict | None) -> "ArmConfig":
    """Overlay measured per-joint zero/limits onto the packaged defaults.

    Only ``zero``/``min``/``max`` are overridden — the measurements. Bus ids,
    names, direction and gains stay with the package, because they describe the
    design rather than this particular arm, and a joint absent from the
    calibration simply keeps its packaged values. A calibration naming a joint
    the package does not have is ignored rather than fatal, so removing a joint
    upstream cannot brick a robot that still has an old file.
    """
    joints = ((data or {}).get("joints")) or {}
    if not joints:
        return cfg
    updated = []
    for spec in cfg.joints:
        entry = joints.get(spec.name)
        if not entry:
            updated.append(spec)
            continue
        merged = JointSpec(
            name=spec.name,
            id=spec.id,
            min_rad=float(entry.get("min", spec.min_rad)),
            max_rad=float(entry.get("max", spec.max_rad)),
            zero_counts=int(entry.get("zero", spec.zero_counts)),
            invert=spec.invert,
        )
        if merged.min_rad > merged.max_rad:
            raise ValueError(
                f"calibration for {spec.name!r}: min {merged.min_rad} > max "
                f"{merged.max_rad}"
            )
        updated.append(merged)
    return replace(cfg, joints=tuple(updated))


def load_calibration(path=None) -> dict:
    """This robot's arm calibration, or an empty dict if it has never run."""
    p = Path(path) if path is not None else calibration_path()
    if not p.exists():
        return {}
    return yaml.safe_load(p.read_text()) or {}


def default_robot_yaml() -> str:
    """Locate robot.yaml in the installed mote_description share."""
    from ament_index_python.packages import get_package_share_directory

    return f"{get_package_share_directory('mote_description')}/config/robot.yaml"


def load() -> ArmConfig:
    """The packaged arm config with this robot's calibration applied."""
    cfg = ArmConfig.from_yaml_file(default_robot_yaml())
    return apply_calibration(cfg, load_calibration())

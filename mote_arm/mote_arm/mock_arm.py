"""An arm control stack that isn't there: the interface, without the bus.

Teleoperation, recording, export and replay are four pieces that have to work
together, and none of them should need a physical arm — or a physical camera —
to be exercised. This node presents the same surface the real stack does, which
since arm control folded into `MoteHardware` is ros2_control's, not a driver's:

    publishes  joint_states                    (sensor_msgs/JointState)
    subscribes arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
    serves     controller_manager/switch_controller

so `mote_arm.control.ArmControl` — and therefore `arm-teleop`,
`arm-pose` and episode replay — cannot tell the difference. It also optionally
publishes a synthetic `image_raw/compressed` whose content tracks the first
joint, so a recorded episode has camera frames that actually change and an
exported dataset is worth looking at.

Like the real thing it starts **limp**: `arm_controller` is inactive until a
client activates it, which is what makes MoteHardware take hold. And like a real
position servo it can be told to settle `--droop` short of its goal, which is
what a proportional loop with `ki = 0` does under load — enough to exercise the
replayer's lag supervision honestly. It is a stand-in for the *control stack*,
not a simulation of the arm: a limp mock does not fall over.

Run it instead of `pixi run arm`:  `pixi run arm-mock`.
"""

from __future__ import annotations

import argparse
import math
import struct
import zlib

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory

from mote_arm import cli, config
from mote_arm.control import (
    ARM_CONTROLLER,
    LIST_SERVICE,
    SWITCH_SERVICE,
    TRAJECTORY_TOPIC,
)
from mote_arm.motion import advance


def _png(width: int, height: int, pixels: bytes) -> bytes:
    """Encode RGB bytes as a PNG, using nothing but zlib and struct.

    The mock has to produce a real, decodable image without dragging an imaging
    library onto the robot environment just so a fake camera can exist. PNG is
    the one format whose encoder is a handful of lines.
    """

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    stride = width * 3
    raw = b"".join(
        b"\x00" + pixels[row * stride : (row + 1) * stride] for row in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


class MockArm(Node):
    def __init__(self, args):
        super().__init__("mock_arm")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self.args = args
        self.position = {j.name: j.clamp_rad(0.0) for j in self.cfg.joints}
        self.goal = dict(self.position)
        # Per joint, rad/s towards the goal: a trajectory says when to arrive,
        # not how fast, so the speed is whatever that duration implies.
        self.rate = dict.fromkeys(self.position, 0.0)
        # Limp until a client activates arm_controller, exactly as the real
        # stack spawns it.
        self.holding = False

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(
            JointTrajectory, TRAJECTORY_TOPIC, self._on_trajectory, 10
        )
        self.create_service(SwitchController, SWITCH_SERVICE, self._on_switch)
        # A command client reads the controller's state rather than assuming
        # it, so the mock has to answer that too or every read waits out a
        # service timeout.
        self.create_service(ListControllers, LIST_SERVICE, self._on_list)
        self._period = 1.0 / args.rate
        self.create_timer(self._period, self._tick)

        self._camera = None
        if args.camera:
            self._camera = self.create_publisher(
                CompressedImage, "image_raw/compressed", 5
            )
            self.create_timer(1.0 / args.camera_rate, self._publish_image)

        self.get_logger().info(
            f"mock_arm up: {len(self.cfg.joints)} joints, no hardware"
            + (", synthetic camera" if args.camera else "")
            + f" ({ARM_CONTROLLER} inactive — the arm is limp)"
        )

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        point = msg.points[-1]
        seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        for name, value in zip(msg.joint_names, point.positions):
            try:
                joint = self.cfg.joint(name)
            except KeyError:
                self.get_logger().warn(f"ignoring goal for unknown joint '{name}'")
                continue
            target = joint.clamp_rad(value)
            self.goal[name] = target
            travel = abs(target - self.position[name])
            self.rate[name] = min(self.args.speed, travel / max(seconds, self._period))

    def _on_list(self, _request, response):
        state = ControllerState()
        state.name = ARM_CONTROLLER
        state.state = "active" if self.holding else "inactive"
        state.type = "joint_trajectory_controller/JointTrajectoryController"
        response.controller = [state]
        return response

    def _on_switch(self, request, response):
        if ARM_CONTROLLER in request.activate_controllers:
            self.holding = True
        if ARM_CONTROLLER in request.deactivate_controllers:
            self.holding = False
        response.ok = True
        return response

    def _tick(self) -> None:
        if self.holding:
            for name, target in self.goal.items():
                # Stop `droop` short of the goal, the way a proportional servo
                # with no integral term settles under a holding load.
                current = self.position[name]
                remaining = target - current
                if abs(remaining) <= self.args.droop:
                    continue
                short = target - math.copysign(self.args.droop, remaining)
                self.position[name] = advance(
                    current, short, self.rate[name] * self._period
                )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.position)
        msg.position = [self.position[n] for n in msg.name]
        self._pub.publish(msg)

    def _publish_image(self) -> None:
        width, height = self.args.camera_size
        first = self.cfg.joints[0]
        span = max(1e-6, first.max_rad - first.min_rad)
        fraction = (self.position[first.name] - first.min_rad) / span
        bar = min(width - 1, max(0, int(fraction * (width - 1))))

        rows = []
        for y in range(height):
            row = bytearray()
            for x in range(width):
                if abs(x - bar) <= 1:
                    row += b"\xff\xd0\x20"
                else:
                    row += bytes((x * 255 // width, y * 255 // height, 64))
            rows.append(bytes(row))

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "rgb8; png compressed rgb8"
        msg.data = _png(width, height, b"".join(rows))
        self._camera.publish(msg)


def _size(text: str) -> tuple[int, int]:
    width, _, height = text.partition("x")
    return int(width), int(height)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mock SO-101 control stack (no hardware)"
    )
    parser.add_argument("--rate", type=float, default=20.0, help="joint_states Hz")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="rad/s the mock will not exceed, whatever a trajectory asks (default 1.0)",
    )
    parser.add_argument(
        "--droop",
        type=float,
        default=0.0,
        help="radians of steady-state error to leave, as a real servo does (default 0)",
    )
    parser.add_argument(
        "--camera", action="store_true", help="publish a synthetic camera"
    )
    parser.add_argument("--camera-rate", type=float, default=10.0)
    parser.add_argument("--camera-size", type=_size, default=(96, 72))
    args = cli.parse(parser)

    rclpy.init()
    node = MockArm(args)
    # Spun on a worker thread and joined from here, rather than spun in the main
    # thread, so the teardown is the one in cli.py: shut the context down, join
    # the spinner, and only then destroy the node. The main thread has nothing
    # else to do — waiting on the spinner is what gives SIGINT somewhere to land.
    spinner = cli.spin_background(node)
    try:
        spinner.join()
    except KeyboardInterrupt:
        pass
    finally:
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()

"""The virtual leader: a leader arm that exists only in software.

Leader-follower teleoperation normally needs two arms — an operator moves the
leader and the follower mirrors it. We have one arm. So the leader is a pose
held in this process, moved by the keyboard, published on ``leader/joint_states``
for ``arm_mirror`` to stream to the follower (and for RViz to draw, if you want
to watch it).

Nothing here talks to the servo bus, or even to the driver: it publishes a pose
and an e-stop flag, and that is the whole interface. Any other frontend that can
publish ``leader/joint_states`` is a drop-in replacement — a slider GUI, a
gamepad, a script — which is why the leader and the mirror are separate nodes.

    hold  q/a w/s e/d r/f t/g y/h   move joint 1..6 up/down
    tap   0                          re-sync the leader to where the arm is
    tap   SPACE                      PANIC: torque off, latched
    tap   z                          clear the panic latch
    tap   [ ]                        slower / faster
    tap   ?                          help    x  quit

**The deadman is key repeat.** A held key auto-repeats; the leader moves only
while those repeats keep arriving and stops within ``--key-timeout`` of the last
one. Release the key and the leader stops publishing, which is what the mirror
reads as "the operator let go". A single tap therefore produces a short, bounded
move (``key_timeout * speed`` radians) rather than nothing — that is the terminal's
key-repeat behaviour showing through, not a debounce we could tune away without
losing the ability to run this over SSH.

Whenever it goes idle the leader re-syncs to the follower's measured pose, so it
can never bank up a lead the arm has to chase after the operator has stopped.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from mote_arm import cli, config, teleop
from mote_arm.mirror import latched

# Key pairs in joint order: the top row raises a joint, the home row lowers it.
KEY_PAIRS = [("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h")]

PANIC_KEY = " "
CLEAR_KEY = "z"
SYNC_KEY = "0"
QUIT_KEYS = ("x", "\x03", "\x04")
PUBLISH_RATE_HZ = 20.0


class VirtualLeader(Node):
    def __init__(self, speed: float, key_timeout: float):
        super().__init__("virtual_leader")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self.speed = speed
        self.key_timeout = key_timeout
        self._lock = threading.Lock()
        self._measured: dict[str, float] = {}
        self.pose: dict[str, float] = {}
        # Per joint: which way it is being driven, and when its key last repeated.
        self._direction: dict[str, float] = {}
        self._key_time: dict[str, float] = {}

        self._pub = self.create_publisher(JointState, "leader/joint_states", 10)
        self._estop_pub = self.create_publisher(Bool, "teleop/estop", latched())
        self.create_subscription(JointState, "joint_states", self._on_states, 10)

        self.keys: dict[str, tuple[str, float]] = {}
        for pair, joint in zip(KEY_PAIRS, self.cfg.joints):
            self.keys[pair[0]] = (joint.name, +1.0)
            self.keys[pair[1]] = (joint.name, -1.0)

    def _on_states(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._measured[name] = position

    def measured(self) -> dict[str, float]:
        with self._lock:
            return dict(self._measured)

    def wait_for_states(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if len(self.measured()) >= len(self.cfg.names):
                return True
            time.sleep(0.05)
        return False

    def sync(self) -> None:
        """Put the leader exactly where the arm is."""
        self.pose = teleop.sync_pose(self.measured(), self.cfg.joints)

    def press(self, name: str, direction: float, now: float) -> None:
        self._direction[name] = direction
        self._key_time[name] = now

    def step(self, now: float, dt: float) -> bool:
        """Advance the leader pose; True if it is live (an input is being held)."""
        live = False
        for name, last in list(self._key_time.items()):
            if now - last > self.key_timeout:
                continue
            live = True
            joint = self.cfg.joint(name)
            current = self.pose.get(name, self.measured().get(name, 0.0))
            self.pose[name] = joint.clamp_rad(
                current + self._direction[name] * self.speed * dt
            )
        return live

    def publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [j.name for j in self.cfg.joints if j.name in self.pose]
        msg.position = [self.pose[n] for n in msg.name]
        self._pub.publish(msg)

    def set_estop(self, engaged: bool) -> None:
        self._estop_pub.publish(Bool(data=engaged))


def _out(text: str = "") -> None:
    """Print in a raw terminal, where a bare newline would stair-step."""
    sys.stdout.write(text + "\r\n")
    sys.stdout.flush()


def _help(node: VirtualLeader) -> None:
    _out()
    _out(f"speed {node.speed:.2f} rad/s   deadman {node.key_timeout:.2f} s")
    for pair, joint in zip(KEY_PAIRS, node.cfg.joints):
        _out(
            f"  {pair[0]} / {pair[1]}   {joint.name:<14} "
            f"limits [{joint.min_rad:+.3f}, {joint.max_rad:+.3f}]"
        )
    _out("  SPACE panic (torque off)   z clear   0 re-sync   [ ] speed   x quit")


def _status(node: VirtualLeader, estopped: bool) -> None:
    measured = node.measured()
    state = "PANIC" if estopped else "ready"
    parts = " ".join(
        f"{j.name.split('_')[0]}={measured.get(j.name, float('nan')):+.3f}"
        for j in node.cfg.joints
    )
    _out(f"[{state}] {parts}")


def _drive(node: VirtualLeader) -> None:
    period = 1.0 / PUBLISH_RATE_HZ
    estopped = False
    idle_since = time.monotonic()

    _out("virtual leader — the arm mirrors this pose. '?' for keys, 'x' to quit.")
    if not node.wait_for_states():
        _out("warning: no /joint_states — is `pixi run arm` running?")
    node.sync()
    _help(node)

    while True:
        now = time.monotonic()
        # Drain every key waiting: a held key auto-repeats faster than we tick,
        # and the useful signal is *that* it repeated, not how many times.
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key in QUIT_KEYS:
                return
            if key in node.keys:
                name, direction = node.keys[key]
                node.press(name, direction, now)
            elif key == PANIC_KEY:
                estopped = True
                node.set_estop(True)
                node.sync()
                _out("PANIC — torque off and latched. 'z' to clear.")
            elif key == CLEAR_KEY:
                if estopped:
                    estopped = False
                    node.sync()
                    node.set_estop(False)
                    _out("panic cleared — the arm will follow again.")
            elif key == SYNC_KEY:
                node.sync()
                _out("leader re-synced to the arm's pose")
            elif key == "[":
                node.speed = max(0.05, node.speed - 0.05)
                _out(f"speed {node.speed:.2f} rad/s")
            elif key == "]":
                node.speed = min(1.0, node.speed + 0.05)
                _out(f"speed {node.speed:.2f} rad/s")
            elif key in ("?", "h"):
                _help(node)
            elif key == "p":
                _status(node, estopped)

        live = node.step(now, period) and not estopped
        if live:
            node.publish()
            idle_since = now
        elif now - idle_since > node.key_timeout:
            # Idle: the leader must not sit ahead of the arm, or resuming would
            # pay out the accumulated difference as an unrequested move.
            node.sync()
            idle_since = now

        time.sleep(period)


def _demo(node: VirtualLeader, seconds: float) -> None:
    """Drive a canned sweep with no terminal, for tests and unattended checks.

    It presses the same keys the operator would, through the same code path, so
    what it exercises is the real leader — including a deliberate pause in the
    middle, which is the deadman doing its job rather than a gap in the script.
    """
    period = 1.0 / PUBLISH_RATE_HZ
    joint = node.cfg.joints[0].name
    _out(f"demo: sweeping {joint} for {seconds:.0f}s (no terminal)")
    if not node.wait_for_states():
        raise SystemExit(
            "no /joint_states — is the arm (or `pixi run arm-mock`) running?"
        )
    node.sync()

    start = time.monotonic()
    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= seconds:
            break
        phase = (elapsed % (seconds / 2)) / (seconds / 2)
        # Middle fifth of each half: no key pressed, so the mirror's deadman
        # holds the arm. A demo that never lets go would not prove it stops.
        if not 0.4 <= phase < 0.6:
            node.press(joint, +1.0 if elapsed < seconds / 2 else -1.0, now)
        if node.step(now, period):
            node.publish()
        time.sleep(period)
    _out("demo finished")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Keyboard virtual leader for the SO-101"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.25,
        help="radians per second the leader moves while a key is held (default 0.25)",
    )
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=0.35,
        help="seconds after the last key repeat before the leader stops (default 0.35)",
    )
    parser.add_argument(
        "--demo",
        type=float,
        default=None,
        metavar="SECONDS",
        help="sweep the first joint for N seconds without a terminal (tests, checks)",
    )
    args = cli.parse(parser)

    rclpy.init()
    node = VirtualLeader(args.speed, args.key_timeout)

    spinner = cli.spin_background(node)

    try:
        if args.demo is not None:
            _demo(node, args.demo)
        else:
            _interactive(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nvirtual leader stopped; the arm holds where it is.")
        cli.shutdown(node, spinner)


def _interactive(node: VirtualLeader) -> None:
    """Run the keyboard loop with the terminal in cbreak mode, and restore it."""
    if not sys.stdin.isatty():
        raise SystemExit(
            "the virtual leader needs a terminal (it reads held keys) — run it "
            "with `pixi run arm-teleop`, not from a launch file. For an "
            "unattended sweep, use --demo SECONDS."
        )
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        _drive(node)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()

"""Keyboard teleoperation of the SO-101 arm.

One process, one node. The keyboard moves a commanded pose; every safety rule
in `mote_arm.teleop.LeaderMirror` is applied to it — clamping, rate limiting,
the deadman, the panic latch — and the result goes to `arm_controller` through
`mote_arm.control`. Nothing here opens the servo bus.

    hold  q/a w/s e/d r/f t/g y/h   move joint 1..6 up/down
    tap   0                          re-sync the commanded pose to the arm
    tap   SPACE                      PANIC: torque off, latched
    tap   z                          clear the panic latch
    tap   [ ]                        slower / faster
    tap   ?                          help    x  quit

**The deadman is key repeat.** A held key auto-repeats; the pose advances only
while those repeats keep arriving and stops within ``--key-timeout`` of the
last one. Release the key and it stops advancing, which is what the mirror
reads as "the operator let go". A single tap therefore produces a short,
bounded move (``key_timeout * speed`` radians) rather than nothing — that is
the terminal's key-repeat behaviour showing through, not a debounce we could
tune away without losing the ability to run this over SSH.

Whenever it goes idle the commanded pose re-syncs to the arm's measured one, so
it can never bank up a lead the arm has to chase after the operator has
stopped.

**Two loops, deliberately.** The keyboard reads on the main thread and the
mirror ticks on its own, because taking hold of the arm is a `switch_controller`
call and a service call made from inside an executor callback can never
complete — the future is resolved by the executor the callback is blocking.
`arm-jog` had the same shape for the same reason.

This was two processes and a `leader/joint_states` topic between them, on the
theory that a gamepad or a slider GUI would one day publish that topic instead.
Nothing ever did, DDS here is loopback-only so no remote frontend could, and the
seam that actually makes a different frontend possible is `teleop.py` being a
library with no ROS in it. What the split bought in practice was a second
terminal and a second thing to remember to start.
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

from mote_arm import cli, config, teleop
from mote_arm.control import ArmControl
from mote_arm.diagnostics import Diagnostics
from mote_arm.teleop import ESTOPPED, HOLDING, TRACKING, LeaderMirror, MirrorLimits

# Key pairs in joint order: the top row raises a joint, the home row lowers it.
KEY_PAIRS = [("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h")]

PANIC_KEY = " "
CLEAR_KEY = "z"
SYNC_KEY = "0"
MODE_KEY = "m"
DEFAULT_STEP_RAD = 0.05
QUIT_KEYS = ("x", "\x03", "\x04")
PUBLISH_RATE_HZ = 20.0
# Slow enough to read while a joint is moving, fast enough to look continuous.
LIVE_LINE_PERIOD = 0.15


class ArmTeleop(Node):
    """The keyboard, the safety rules and the arm, in one node."""

    def __init__(self, speed: float, key_timeout: float):
        super().__init__("arm_teleop")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("max_velocity", MirrorLimits.max_velocity)
        self.declare_parameter("deadman_timeout", MirrorLimits.deadman_timeout)
        # `pixi run arm-teleop --ros-args -p diagnose:=true`
        self.declare_parameter("diagnose", False)

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
        self._estop_requested = False

        self.mirror = LeaderMirror(
            self.cfg.joints,
            MirrorLimits(
                max_velocity=self.get_parameter("max_velocity").value,
                deadman_timeout=self.get_parameter("deadman_timeout").value,
            ),
        )
        self.arm = ArmControl(self)
        self.create_subscription(JointState, "joint_states", self._on_states, 10)

        self._reported = None
        self._stalled: list[str] = []
        self.diagnostics = (
            Diagnostics(self) if self.get_parameter("diagnose").value else None
        )
        self.period = 1.0 / max(1.0, self.get_parameter("rate").value)

        self.keys: dict[str, tuple[str, float]] = {}
        for pair, joint in zip(KEY_PAIRS, self.cfg.joints):
            self.keys[pair[0]] = (joint.name, +1.0)
            self.keys[pair[1]] = (joint.name, -1.0)

        for problem in self.cfg.problems:
            self.get_logger().warn(problem)

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
        """Put the commanded pose exactly where the arm is."""
        self.pose = teleop.sync_pose(self.measured(), self.cfg.joints)

    def press(self, name: str, direction: float, now: float) -> None:
        self._direction[name] = direction
        self._key_time[name] = now

    def driving(self, now: float) -> list[str]:
        """Joints whose key is still repeating, in config order."""
        return [
            j.name
            for j in self.cfg.joints
            if now - self._key_time.get(j.name, -1e9) <= self.key_timeout
        ]

    def nudge(self, name: str, direction: float, size: float, now: float) -> bool:
        """Advance one joint by exactly ``size`` radians. False if ignored.

        A held key auto-repeats and a terminal cannot tell a repeat from a fresh
        press, so a repeat inside ``key_timeout`` is ignored: holding the key
        steps once, and stepping again means releasing and pressing again. That
        is what `arm-jog` did with a typed step and an Enter, without a second
        keyboard path that has no rate limit, no deadman and no panic latch.
        """
        if now - self._key_time.get(name, -1e9) <= self.key_timeout:
            return False
        self._key_time[name] = now
        joint = self.cfg.joint(name)
        current = self.pose.get(name, self.measured().get(name, 0.0))
        self.pose[name] = joint.clamp_rad(current + direction * size)
        return True

    def settle_time(self, size: float) -> float:
        """How long the arm needs to walk one step, at the rate limit.

        A step is offered for this long rather than once, because the deadman
        would otherwise fire mid-travel and stop the arm short of the increment
        that was asked for.
        """
        return abs(size) / max(1e-6, self.mirror.limits.max_velocity)

    def step(self, now: float, dt: float) -> bool:
        """Advance the commanded pose; True if an input is being held."""
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

    def offer(self) -> None:
        """Hand the commanded pose to the safety rules.

        Named for what it does rather than for a topic: this used to be a
        publish, and the mirror on the other end was free to refuse it. It still
        is — `LeaderMirror` clamps, rate-limits and may be latched off.
        """
        if self.diagnostics is not None:
            self.diagnostics.on_leader(time.monotonic())
        self.mirror.on_leader(dict(self.pose), self._now())

    def set_estop(self, engaged: bool) -> None:
        """Latch or clear the panic. Acted on by the tick, never from here.

        Dropping torque deactivates `arm_controller`, which is a service call,
        and a service call cannot complete on the thread the keyboard loop runs
        on while the executor is elsewhere. So this records intent and `tick`
        performs it.
        """
        self._estop_requested = engaged

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _apply_estop(self) -> None:
        if self._estop_requested == self.mirror.estopped:
            return
        self.mirror.set_estop(self._estop_requested, self._now())
        if self._estop_requested:
            self.get_logger().warn("PANIC: dropping torque and refusing goals")
            if not self.arm.set_holding(False):
                self.get_logger().error(
                    "could not deactivate arm_controller — the arm may still be "
                    "holding; stop the control stack or cut power"
                )
        else:
            self.get_logger().info("panic cleared; following again")

    def tick(self) -> None:
        if self.diagnostics is not None:
            self.diagnostics.tick(time.monotonic())
        self._apply_estop()
        self.mirror.on_measured(self.measured())
        goal = self.mirror.update(self._now(), self.period)
        if goal:
            # One period to reach the point: the mirror has already rate-limited
            # the step to what that allows, and a trajectory the arm cannot
            # finish in time just runs ahead of the hardware.
            self.arm.send(goal, self.period)

        if self.mirror.stalled != self._stalled:
            self._stalled = list(self.mirror.stalled)
            if self._stalled:
                self.get_logger().warn(
                    f"not following: {', '.join(self._stalled)} is "
                    f"{self.mirror.limits.max_lag:.2f} rad behind and not moving — "
                    "holding the command there rather than driving further ahead"
                )
            else:
                self.get_logger().info("following again")

        if self.mirror.state != self._reported:
            self._reported = self.mirror.state
            if self.mirror.state == HOLDING:
                self.get_logger().info("deadman: no input, holding position")
            elif self.mirror.state == TRACKING:
                self.get_logger().info("following the keyboard")
            elif self.mirror.state == ESTOPPED:
                self.get_logger().warn("e-stopped")

    def run_mirror(self) -> None:
        """Tick until the context goes down; runs on its own thread."""
        while rclpy.ok():
            self.tick()
            time.sleep(self.period)


# True while a live status line is on screen, waiting to be overwritten in place.
# Any ordinary line must clear it first, or the two overlap.
_live_line = False


def _out(text: str = "") -> None:
    """Print in a raw terminal, where a bare newline would stair-step."""
    global _live_line
    if _live_line:
        sys.stdout.write("\r\033[K")
        _live_line = False
    sys.stdout.write(text + "\r\n")
    sys.stdout.flush()


def _live(text: str) -> None:
    """Rewrite one status line in place, rather than scrolling a new one."""
    global _live_line
    sys.stdout.write("\r\033[K" + text)
    sys.stdout.flush()
    _live_line = True


def _clear_live() -> None:
    global _live_line
    if _live_line:
        sys.stdout.write("\r\033[K")
        sys.stdout.flush()
        _live_line = False


def _driving_line(node: ArmTeleop, names: list[str]) -> str:
    """Where the driven joints are, and whether they are against a limit.

    "Hold the key past the soft limit and watch it stop" is not something an
    operator can judge from an arm that has simply stopped moving: it looks the
    same as a stall, a dropped link or a servo that gave up. So say which it is.
    """
    measured = node.measured()
    parts = []
    for name in names:
        joint = node.cfg.joint(name)
        now = measured.get(name, float("nan"))
        target = node.pose.get(name, now)
        if target in (joint.min_rad, joint.max_rad):
            edge = "min" if target == joint.min_rad else "max"
            note = "" if not joint.unreachable else " — but the register edge"
            note += "" if not joint.unreachable else " bites first, see '?'"
            parts.append(f"{name} {now:+.3f}  AT LIMIT ({edge} {target:+.3f}){note}")
        else:
            line = (
                f"{name} {now:+.3f} -> {target:+.3f} "
                f"[{joint.min_rad:+.3f}, {joint.max_rad:+.3f}]"
            )
            # The arm is being asked for something it is not doing. Saying so
            # here is the difference between "why is nothing happening" and
            # knowing the command is fine and the joint is not moving.
            if now == now and abs(target - now) > MirrorLimits.max_lag:
                line += "  NOT FOLLOWING"
            parts.append(line)
    return "  " + "   ".join(parts)


def _help(node: ArmTeleop) -> None:
    _out()
    _out(f"speed {node.speed:.2f} rad/s   deadman {node.key_timeout:.2f} s")
    for pair, joint in zip(KEY_PAIRS, node.cfg.joints):
        # The reachable band, not the configured one: an angle the 12-bit goal
        # register cannot express is not a limit you can drive to, it is one the
        # arm stops at without saying why.
        line = (
            f"  {pair[0]} / {pair[1]}   {joint.name:<14} "
            f"limits [{joint.reachable_min:+.3f}, {joint.reachable_max:+.3f}]"
        )
        if joint.unreachable:
            line += f"  (narrowed from [{joint.min_rad:+.3f}, {joint.max_rad:+.3f}])"
        _out(line)
    for problem in node.cfg.problems:
        _out(f"  WARNING {problem}")
    _out("  SPACE panic (torque off)   z clear   0 re-sync   [ ] speed")
    _out("  m hold/step mode   p all joint positions   ? this help   x quit")


def _status(node: ArmTeleop, estopped: bool) -> None:
    measured = node.measured()
    state = "PANIC" if estopped else "ready"
    parts = " ".join(
        f"{j.name.split('_')[0]}={measured.get(j.name, float('nan')):+.3f}"
        for j in node.cfg.joints
    )
    _out(f"[{state}] {parts}")


def _drive(node: ArmTeleop, step_size: float = DEFAULT_STEP_RAD) -> None:
    period = 1.0 / PUBLISH_RATE_HZ
    estopped = False
    idle_since = time.monotonic()
    last_line = 0.0
    # Hold mode moves while a key is held; step mode moves one increment per
    # press. Step mode is what `arm-jog` was for, on the one keyboard path that
    # has the rate limit, the deadman and the panic latch.
    stepping = False
    settle_until = 0.0

    _out("arm teleop — the arm follows this pose. '?' for keys, 'x' to quit.")
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
                _clear_live()
                return
            if key in node.keys:
                name, direction = node.keys[key]
                if stepping:
                    if node.nudge(name, direction, step_size, now):
                        settle_until = now + node.settle_time(step_size)
                        _clear_live()
                        _out(f"{name} {direction * step_size:+.3f} rad")
                else:
                    node.press(name, direction, now)
            elif key == MODE_KEY:
                stepping = not stepping
                node.sync()
                settle_until = 0.0
                _clear_live()
                _out(
                    f"step mode: one {step_size:.3f} rad increment per press"
                    if stepping
                    else "hold mode: moves while a key is held"
                )
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
                _out("re-synced to the arm's pose")
            elif key == "[":
                node.speed = max(0.05, node.speed - 0.05)
                _out(f"speed {node.speed:.2f} rad/s")
            elif key == "]":
                node.speed = min(1.0, node.speed + 0.05)
                _out(f"speed {node.speed:.2f} rad/s")
            elif key == "?":
                # Not "h" as well: `h` drives joint 6 down, and the joint keys
                # are matched first, so a help key there could never fire.
                _help(node)
            elif key == "p":
                _status(node, estopped)

        driving = node.driving(now)
        if stepping:
            # Offered until the arm has had time to walk the increment, or the
            # deadman stops it half a step short of what was asked for.
            live = now < settle_until and not estopped
        else:
            live = node.step(now, period) and not estopped
        if live:
            node.offer()
            idle_since = now
            if now - last_line >= LIVE_LINE_PERIOD:
                last_line = now
                _live(_driving_line(node, driving))
        elif now - idle_since > node.key_timeout:
            _clear_live()
            # Idle: the command must not sit ahead of the arm, or resuming would
            # pay out the accumulated difference as an unrequested move.
            node.sync()
            idle_since = now

        time.sleep(period)


def _demo(node: ArmTeleop, seconds: float) -> None:
    """Drive a canned sweep with no terminal, for tests and unattended checks.

    It presses the same keys the operator would, through the same code path, so
    what it exercises is the real teleop path — including a deliberate pause in
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
            node.offer()
        time.sleep(period)
    _out("demo finished")


def main() -> None:
    parser = argparse.ArgumentParser(description="Keyboard teleop for the SO-101")
    parser.add_argument(
        "--speed",
        type=float,
        default=0.25,
        help="radians per second the commanded pose moves while a key is held "
        "(default 0.25)",
    )
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=0.35,
        help="seconds after the last key repeat before it stops (default 0.35)",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=DEFAULT_STEP_RAD,
        help=f"radians per press in step mode, toggled with 'm' "
        f"(default {DEFAULT_STEP_RAD})",
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
    node = ArmTeleop(args.speed, args.key_timeout)
    spinner = cli.spin_background(node)
    # The mirror ticks on a thread of its own: it makes service calls, which
    # cannot complete on a thread the executor is blocking, and the keyboard
    # owns the main one.
    ticker = threading.Thread(target=node.run_mirror, daemon=True)
    ticker.start()

    try:
        if args.demo is not None:
            _demo(node, args.demo)
        else:
            _interactive(node, args.step)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the arm limp: this process took hold of it, so it gives it back
        # rather than leaving a torqued arm behind an exited process.
        node.arm.set_holding(False)
        print("\nteleop stopped; the arm is limp.")
        cli.shutdown(node, spinner)
        ticker.join(timeout=2.0)


def _interactive(node: ArmTeleop, step_size: float = DEFAULT_STEP_RAD) -> None:
    """Run the keyboard loop with the terminal in cbreak mode, and restore it."""
    if not sys.stdin.isatty():
        raise SystemExit(
            "arm teleop needs a terminal (it reads held keys) — run it with "
            "`pixi run arm-teleop`, not from a launch file. For an unattended "
            "sweep, use --demo SECONDS."
        )
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        _drive(node, step_size)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()

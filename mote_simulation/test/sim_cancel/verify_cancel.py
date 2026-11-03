#!/usr/bin/env python3
"""Cancel a `goto` mid-drive in the sim and time what the robot does about it.

Run by run_sim_cancel.sh once `sim_launch.py mode:=nav` is up. Dispatches
`goto dropoff`, waits until odometry shows the robot really driving, sends a
mission/v0 cancel, and records every command on `/diff_drive_controller/cmd_vel`
and every odometry sample around it. Then sends a second `goto` and cancels
that too.

Passes when the mission ends `cancelled`, the drive command is zero within
`--period-ms` of the cancel (Nav2's controller runs at 20 Hz), nothing non-zero
is commanded after that, and the second `goto` is `accepted` rather than
refused `busy`. The figures go to stdout and, with `--out`, to a JSON file.

Exits 0 on PASS, 1 on any failed check (printed as "FAIL: ...").
"""

import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mote_bringup.spec import mission

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: Below this a drive command or an odometry speed counts as stopped.
STILL_V = 1e-3
STILL_ODOM_V = 0.01
STILL_ODOM_W = 0.02


class Probe(Node):
    def __init__(self):
        super().__init__("sim_cancel_verifier")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.platform_id = None
        self.statuses = []
        self.cmds = []  # (wall, sim, v, w)
        self.odom = []  # (wall, sim, v, w)
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.cancel_pub = self.create_publisher(String, "task/cancel", 10)
        self.create_subscription(String, "task/status", self._status, 10)
        self.create_subscription(String, "task/capabilities", self._caps, LATCHED)
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self._cmd, 50
        )
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._odom, 50
        )

    def sim(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _status(self, msg):
        status = json.loads(msg.data)
        status["_wall"], status["_sim"] = time.monotonic(), self.sim()
        self.statuses.append(status)
        failure = (status.get("failure") or {}).get("class", "")
        print(f"  {status['id']} {status['state']} {failure}", flush=True)

    def _caps(self, msg):
        document = json.loads(msg.data)
        self.platform_id = document["platform_id"]
        self.cancellable = {
            c["key"]: c["execution"]["cancellable"] for c in document["capabilities"]
        }

    def _cmd(self, msg):
        self.cmds.append(
            (time.monotonic(), self.sim(), msg.twist.linear.x, msg.twist.angular.z)
        )

    def _odom(self, msg):
        twist = msg.twist.twist
        self.odom.append(
            (time.monotonic(), self.sim(), twist.linear.x, twist.angular.z)
        )

    def spin_for(self, seconds, until=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if until is not None and until():
                return True
        return until is None or bool(until())

    def of(self, mission_id):
        return [s for s in self.statuses if s["id"] == mission_id]

    def goto(self, target):
        payload = mission.command(self.platform_id, "goto", {"target": target})
        self.command_pub.publish(String(data=json.dumps(payload)))
        print(f"-> goto {target} ({payload['id']})", flush=True)
        return payload["id"]

    def cancel(self, mission_id):
        payload = mission.cancel(
            self.platform_id, mission_id, issued_by="sim-cancel", reason="acceptance"
        )
        self.cancel_pub.publish(String(data=json.dumps(payload)))
        print(f"-> cancel {mission_id}", flush=True)

    def driving(self, since):
        """Odometry above 0.05 m/s for a continuous second, and still now."""
        fast = [o for o in self.odom if o[0] >= since and abs(o[2]) > 0.05]
        return (
            bool(fast)
            and fast[-1][0] - fast[0][0] > 1.0
            and time.monotonic() - fast[-1][0] < 0.2
        )


def ms(seconds):
    return None if seconds is None else round(seconds * 1000, 1)


def measure(args, probe) -> dict:
    assert probe.spin_for(60, lambda: probe.platform_id), "no capability set"
    assert all(probe.cancellable.values()), f"not cancellable: {probe.cancellable}"
    assert probe.spin_for(
        30,
        lambda: (
            probe.command_pub.get_subscription_count()
            and probe.cancel_pub.get_subscription_count()
        ),
    ), "task_server is not subscribed to task/command and task/cancel"

    # AMCL settles after Nav2 activates; a `precondition` refusal until then is
    # the robot being right, so this retries rather than fails.
    first = None
    for _ in range(30):
        first = probe.goto(args.target)
        probe.spin_for(10, lambda: probe.of(first))
        if [s["state"] for s in probe.of(first)] == [mission.ACCEPTED]:
            break
        time.sleep(2)
    assert probe.of(first) and probe.of(first)[0]["state"] == mission.ACCEPTED, (
        f"the first goto was never accepted: {probe.of(first)}"
    )

    accepted_at = time.monotonic()
    assert probe.spin_for(60, lambda: probe.driving(accepted_at)), "never drove"
    probe.spin_for(1.0)

    wall0, sim0 = time.monotonic(), probe.sim()
    speed = next((o[2] for o in reversed(probe.odom) if o[0] < wall0), None)
    probe.cancel(first)
    assert probe.spin_for(15, lambda: any(s["terminal"] for s in probe.of(first)))
    final = probe.of(first)[-1]
    probe.spin_for(3.0)

    after = [c for c in probe.cmds if c[0] >= wall0]
    zero = next((c for c in after if abs(c[2]) < STILL_V and abs(c[3]) < STILL_V), None)
    moving_after_zero = [
        c
        for c in after
        if zero and c[0] > zero[0] and (abs(c[2]) >= STILL_V or abs(c[3]) >= STILL_V)
    ]
    still = next(
        (
            o
            for o in probe.odom
            if o[0] >= wall0 and abs(o[2]) < STILL_ODOM_V and abs(o[3]) < STILL_ODOM_W
        ),
        None,
    )

    second = probe.goto(args.second_target)
    assert probe.spin_for(10, lambda: probe.of(second)), "no answer to the second goto"
    second_state = probe.of(second)[0]["state"]
    probe.spin_for(2.0)
    probe.cancel(second)
    probe.spin_for(15, lambda: any(s["terminal"] for s in probe.of(second)))

    return {
        "speed_at_cancel_mps": None if speed is None else round(speed, 3),
        "final_state": final["state"],
        "final_detail": final.get("detail"),
        "status_after_cancel_sim_ms": ms(final["_sim"] - sim0),
        "zero_command_after_cancel_sim_ms": ms(zero[1] - sim0) if zero else None,
        "nonzero_commands_after_the_zero": len(moving_after_zero),
        "wheels_at_rest_after_cancel_sim_ms": ms(still[1] - sim0) if still else None,
        "second_goto_first_state": second_state,
        "second_goto_final_state": probe.of(second)[-1]["state"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--target", default="dropoff")
    parser.add_argument("--second-target", default="pickup")
    parser.add_argument("--period-ms", type=float, default=50.0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    try:
        result = measure(args, probe)
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        return 1
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()

    print(json.dumps(result, indent=2))
    if args.out:
        with open(args.out, "w") as handle:
            json.dump(result, handle, indent=2)
    checks = [
        (
            result["final_state"] == mission.CANCELLED,
            "the mission did not end cancelled",
        ),
        (
            result["zero_command_after_cancel_sim_ms"] is not None
            and result["zero_command_after_cancel_sim_ms"] <= args.period_ms,
            f"the drive command was not zero within {args.period_ms:.0f} ms",
        ),
        (
            result["nonzero_commands_after_the_zero"] == 0,
            "something drove the wheels again after the zero",
        ),
        (
            result["second_goto_first_state"] == mission.ACCEPTED,
            "the next goto was not accepted",
        ),
    ]
    failed = [message for ok, message in checks if not ok]
    for message in failed:
        print(f"FAIL: {message}")
    if not failed:
        print("SIM CANCEL PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

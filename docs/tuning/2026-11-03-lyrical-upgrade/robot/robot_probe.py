"""On-robot check: loop rates at idle, a teleop turn in place, a Nav2 goal out and back.

    python robot_probe.py [DIST]    (default 1.0 m)

Teleop turns in place and back, since that path has no obstacle check. The
goals are DIST m ahead along the AMCL heading and back to the start, so Nav2's
costmaps guard the motion.
"""

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
rclpy.init()
n = Node("robot_probe")
js, nav, odom = [], [], []
amcl = {}
cm_diag = {}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(
    JointState, "/joint_states", lambda m: js.append(time.monotonic()), 100
)
n.create_subscription(
    TwistStamped, "/cmd_vel_nav", lambda m: nav.append(time.monotonic()), 50
)
n.create_subscription(
    Odometry,
    "/diff_drive_controller/odom",
    lambda m: odom.append(
        (
            time.monotonic(),
            m.pose.pose.position.x,
            m.pose.pose.position.y,
            yaw(m.pose.pose.orientation),
        )
    ),
    50,
)
n.create_subscription(
    PoseWithCovarianceStamped,
    "/amcl_pose",
    lambda m: amcl.update(p=m.pose.pose),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)


def on_diag(m):
    for s in m.status:
        if "controller_manager" in s.name.lower() or "periodicity" in s.name.lower():
            cm_diag[s.name] = (s.message, {kv.key: kv.value for kv in s.values})


n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)
teleop = n.create_publisher(TwistStamped, "/cmd_vel_teleop_stamped", 10)


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.01)


def rate(stamps, t0, t1):
    s = [t for t in stamps if t0 <= t <= t1]
    if len(s) < 2:
        return f"{len(s)} msgs"
    d = sorted(b - a for a, b in zip(s, s[1:]))
    return (
        f"{len(s)} msgs {(len(s) - 1) / (s[-1] - s[0]):.1f} Hz median {d[len(d) // 2] * 1000:.0f} ms "
        f"p95 {d[int(len(d) * 0.95)] * 1000:.0f} ms max {d[-1] * 1000:.0f} ms"
    )


t0 = time.monotonic()
spin(15)
t1 = time.monotonic()
print("idle /joint_states:", rate(js, t0, t1))
print("idle /diff_drive_controller/odom:", rate([o[0] for o in odom], t0, t1))
for k, (msg, vals) in sorted(cm_diag.items()):
    print(
        "diag",
        k,
        "|",
        msg,
        "|",
        {a: b for a, b in vals.items() if "eriod" in a or "ate" in a or "ycle" in a},
    )


def turn(wz, sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        m = TwistStamped()
        m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.angular.z = wz
        teleop.publish(m)
        spin(0.1)
    spin(2.0)


y0 = odom[-1][3] if odom else None
turn(0.5, 2.0)
if y0 is not None and odom:
    dy = math.degrees(
        math.atan2(math.sin(odom[-1][3] - y0), math.cos(odom[-1][3] - y0))
    )
    print(
        f"teleop: 0.5 rad/s for 2.0 s, wheel odom turned {dy:.1f} deg (open-loop expectation ~57)"
    )
else:
    print("teleop: no wheel odometry")
turn(-0.5, 2.0)

spin(2.0)
if "p" not in amcl:
    print("nav: no /amcl_pose; goal skipped")
else:
    client = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=20):
        print("nav: no navigate_to_pose server")
        raise SystemExit(1)

    def go(x, y, q, label):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation = q
        fut = client.send_goal_async(g)
        while not fut.done():
            spin(0.05)
        gh = fut.result()
        if not gh.accepted:
            print(f"nav {label}: goal rejected")
            return
        res = gh.get_result_async()
        ts = time.monotonic()
        while not res.done() and time.monotonic() - ts < 90:
            spin(0.05)
        te = time.monotonic()
        st = res.result().status if res.done() else "timeout"
        print(f"nav {label}: status {st} (4=succeeded) after {te - ts:.1f} s")
        print(f"nav {label} /cmd_vel_nav:", rate(nav, ts, te))
        print(f"nav {label} /joint_states:", rate(js, ts, te))

    spin(2.0)
    p = amcl["p"]
    th = yaw(p.orientation)
    print(
        f"nav: start ({p.position.x:.2f},{p.position.y:.2f},{math.degrees(th):.0f}deg), {DIST} m out and back"
    )
    go(
        p.position.x + DIST * math.cos(th),
        p.position.y + DIST * math.sin(th),
        p.orientation,
        "out",
    )
    go(p.position.x, p.position.y, p.orientation, "back")

"""Send one NavigateToPose goal and time /cmd_vel_nav for 20 s of driving."""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

rclpy.init()
n = Node("rate_probe")
stamps = []
n.create_subscription(
    TwistStamped, "/cmd_vel_nav", lambda m: stamps.append(time.monotonic()), 50
)
client = ActionClient(n, NavigateToPose, "navigate_to_pose")
if not client.wait_for_server(timeout_sec=30):
    print("no navigate_to_pose server")
    raise SystemExit(1)
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = -8.8
goal.pose.pose.position.y = 3.5
goal.pose.pose.orientation.w = 1.0
client.send_goal_async(goal)
deadline = time.monotonic() + 60
while time.monotonic() < deadline and (not stamps or stamps[-1] - stamps[0] < 20):
    rclpy.spin_once(n, timeout_sec=0.05)
if len(stamps) < 2:
    print("no /cmd_vel_nav")
    raise SystemExit(1)
d = sorted(b - a for a, b in zip(stamps, stamps[1:]))
print(
    f"{len(stamps)} msgs {len(stamps) / (stamps[-1] - stamps[0]):.1f} Hz"
    f" median {d[len(d) // 2] * 1000:.0f} ms p95 {d[int(len(d) * 0.95)] * 1000:.0f} ms"
)

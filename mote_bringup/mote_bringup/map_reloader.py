"""``map_reloader`` — loads a newly installed map revision into a running Nav2.

``map_server`` reads its map once, at startup. The fleet agent installs a
promoted revision by flipping the floor's ``map`` symlink, which only the next
bringup would see. This node closes that gap: it hears ``map/installed`` from
the agent, and when the installed revision is on the floor Nav2 is serving it
calls ``map_server/load_map`` with that revision's yaml.

It lives beside Nav2 rather than in the agent because it acts on Nav2, and the
agent stays a bridge and a reporter: the agent states what it put on disk, and
this node decides when navigation takes it. ``nav2_launch.py`` starts it only
with ``localisation:=true``, since under SLAM there is no saved map to swap.

**It never swaps the map under a live mission.** Before loading it asks the
task server (``task/idle``), which owns the lane, and waits while a mission is
in flight. With no task server running there is no mission to protect. While
a load is under way ``map/serving`` says so, and the task server refuses a new
mission as ``busy`` until it finishes.

**The pose estimate is carried across.** AMCL re-initialises its filter when a
new map arrives. A promoted revision is registered into the same floor frame,
so the last ``amcl_pose`` is still valid on it; this node re-publishes it on
``/initialpose`` after the load, and again if AMCL's next estimate lands
elsewhere. ``amcl_pose`` rather than a TF lookup, because a TransformListener
takes the whole ``/tf`` stream for the lifetime of the node to answer one
question a few times a year.

**A failed load changes nothing.** The old map stays served, the reason is
logged and published in ``map/serving.error``, and the agent reports it in
health. The same revision is not retried until the agent installs again.

    ros2 run mote_bringup map_reloader --ros-args -p map:=<map.yaml>
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mote_bringup import map_reload

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

LOAD_RESULTS = {
    LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: "map does not exist",
    LoadMap.Response.RESULT_INVALID_MAP_DATA: "invalid map data",
    LoadMap.Response.RESULT_INVALID_MAP_METADATA: "invalid map metadata",
    LoadMap.Response.RESULT_UNDEFINED_FAILURE: "undefined failure",
}


def planar_distance(a, b) -> tuple[float, float]:
    """Translation (m) and absolute yaw (rad) between two ``Pose`` messages."""

    def yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

    d = math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)
    dyaw = abs(
        math.atan2(
            math.sin(yaw(a.orientation) - yaw(b.orientation)),
            math.cos(yaw(a.orientation) - yaw(b.orientation)),
        )
    )
    return d, dyaw


class MapReloader(Node):
    def __init__(self, **node_kwargs):
        super().__init__("map_reloader", **node_kwargs)
        map_yaml = self.declare_parameter("map", "").value
        self.poll_period = self.declare_parameter("poll_period", 1.0).value
        self.call_timeout = self.declare_parameter("call_timeout", 10.0).value
        self.settle = self.declare_parameter("settle", 0.5).value
        self.pose_attempts = self.declare_parameter("pose_attempts", 3).value
        self.pose_wait = self.declare_parameter("pose_wait", 3.0).value
        self.pose_tolerance = self.declare_parameter("pose_tolerance", 0.25).value
        self.yaw_tolerance = self.declare_parameter("yaw_tolerance", 0.2).value

        located = map_reload.locate(map_yaml)
        self.site, self.floor, self.revision = located or (None, None, None)
        self.error = None
        self.loading = None
        self._installed = None
        self._failed = None
        self._deferred_for = None
        self._pose = None
        self._pose_seen = threading.Condition()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stopping = False

        self.serving_pub = self.create_publisher(
            String, map_reload.SERVING_TOPIC, LATCHED
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self.create_subscription(
            String, map_reload.INSTALLED_TOPIC, self._on_installed, LATCHED
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, LATCHED
        )
        self.load_client = self.create_client(LoadMap, "map_server/load_map")
        self.idle_client = self.create_client(Trigger, map_reload.IDLE_SERVICE)
        self.create_timer(self.poll_period, self._wake.set)

        if located:
            self.get_logger().info(
                f"serving {self.site}/{self.floor} revision {self.revision}"
            )
        else:
            self.get_logger().warning(
                f"map {map_yaml!r} is not a site revision; promotions will not "
                "be loaded until nav is restarted on a site map"
            )
        self.publish_serving()

        self._worker = threading.Thread(
            target=self._run, name="map-reloader", daemon=True
        )
        self._worker.start()

    # ---- inputs ---------------------------------------------------------

    def _on_installed(self, msg: String):
        payload = map_reload.parse(msg.data)
        if payload is None:
            self.get_logger().warning(f"unreadable {map_reload.INSTALLED_TOPIC}")
            return
        with self._lock:
            self._installed = payload
            # A fresh installation is a fresh chance, even of a revision that
            # failed before: the agent may have re-fetched it.
            self._failed = None
        self._wake.set()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        with self._pose_seen:
            self._pose = msg
            self._pose_seen.notify_all()

    # ---- output ---------------------------------------------------------

    def publish_serving(self):
        self.serving_pub.publish(
            String(
                data=map_reload.serving(
                    self.site,
                    self.floor,
                    self.revision,
                    loading=self.loading,
                    error=self.error,
                )
            )
        )

    # ---- the worker -----------------------------------------------------

    def _run(self):
        while not self._stopping:
            self._wake.wait()
            self._wake.clear()
            if self._stopping:
                return
            try:
                self._step()
            except Exception as exc:  # the worker must outlive any one attempt
                self.get_logger().error(f"map reload step failed: {exc!r}")

    def _step(self):
        with self._lock:
            target = map_reload.wanted(
                (self.site, self.floor, self.revision), self._installed
            )
            if target is None or target == self._failed:
                return
        busy = self._lane_holder()
        if busy is not None:
            if self._deferred_for != target:
                self._deferred_for = target
                self.get_logger().info(
                    f"revision {target} installed; loading it when the mission "
                    f"ends ({busy})"
                )
            return
        self._deferred_for = None
        self._load(target)

    def _lane_holder(self) -> str | None:
        """Why the map may not change now, or None when it may."""
        if not self.idle_client.service_is_ready():
            return None
        response = self._call(self.idle_client, Trigger.Request())
        if response is None:
            return "the task server did not answer"
        return None if response.success else response.message

    def _call(self, client, request):
        future = client.call_async(request)
        deadline = time.monotonic() + self.call_timeout
        while not future.done():
            if time.monotonic() > deadline or self._stopping:
                client.remove_pending_request(future)
                return None
            time.sleep(0.02)
        return future.result()

    def _load(self, target: str):
        path = map_reload.revision_yaml(self.site, self.floor, target)
        carried = self._pose
        self.loading = target
        self.publish_serving()
        self.get_logger().info(f"loading revision {target} from {path}")
        try:
            failure = self._request_load(path)
        finally:
            self.loading = None
        if failure is not None:
            with self._lock:
                self._failed = target
            self.error = f"revision {target}: {failure}"
            self.get_logger().error(
                f"could not load {self.error}; still serving {self.revision}"
            )
            self.publish_serving()
            return
        previous = self.revision
        self.revision = target
        self.error = None
        self.publish_serving()
        self.get_logger().info(f"now serving revision {target} (was {previous})")
        self._carry_pose(carried)

    def _request_load(self, path) -> str | None:
        """None on success, else the reason the old map is still served."""
        if not self.load_client.wait_for_service(timeout_sec=2.0):
            return "map_server/load_map is not available"
        request = LoadMap.Request()
        request.map_url = str(path)
        response = self._call(self.load_client, request)
        if response is None:
            return f"map_server/load_map did not answer within {self.call_timeout}s"
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return LOAD_RESULTS.get(response.result, f"result {response.result}")
        return None

    def _carry_pose(self, carried: PoseWithCovarianceStamped | None):
        """Put AMCL back where it was, and check that it stayed there."""
        if carried is None:
            self.get_logger().warning(
                "no amcl_pose to carry across the load; AMCL will need an initial pose"
            )
            return
        time.sleep(self.settle)
        for attempt in range(1, self.pose_attempts + 1):
            sent = PoseWithCovarianceStamped()
            sent.header.frame_id = carried.header.frame_id or "map"
            sent.header.stamp = self.get_clock().now().to_msg()
            sent.pose = carried.pose
            with self._pose_seen:
                before = self._pose
                self.initial_pose_pub.publish(sent)
                arrived = self._pose_seen.wait_for(
                    lambda: self._pose is not before, timeout=self.pose_wait
                )
                estimate = self._pose
            if not arrived:
                self.get_logger().warning(
                    f"AMCL published no estimate within {self.pose_wait}s of the "
                    "carried initial pose"
                )
                return
            d, dyaw = planar_distance(estimate.pose.pose, carried.pose.pose)
            if d <= self.pose_tolerance and dyaw <= self.yaw_tolerance:
                self.get_logger().info(
                    f"pose carried across the load (off by {d:.3f} m, "
                    f"{math.degrees(dyaw):.1f} deg)"
                )
                return
            self.get_logger().warning(
                f"AMCL came back {d:.2f} m / {math.degrees(dyaw):.0f} deg from the "
                f"carried pose (attempt {attempt}); re-sending it"
            )

    def stop(self):
        self._stopping = True
        self._wake.set()
        self._worker.join(timeout=2.0)


def main():
    rclpy.init()
    node = MapReloader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

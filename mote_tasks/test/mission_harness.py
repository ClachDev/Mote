"""What all three tree tests need once a mission is a typed payload.

Two things changed for them at once. A command is now a JSON mission/v0 payload
rather than a sentence, so building one by hand in three files would be three
chances to build it wrongly; and ``goto``/``fetch`` declare a blocking
``localized`` precondition, so a task server with no ``map``->``base_link``
transform now *correctly* refuses every mission. A mock world therefore has to
say where the robot is, which it should have been doing anyway — Nav2 could
never have driven without it.

That transform is broadcast on a timer rather than as a static one, and the
difference is the point of the precondition. ``localized`` asks whether the
platform knows where it is **now**; a static transform's stamp never advances,
so a mock world that published one would look localised at second one and
unlocalised at second six, which is precisely the "AMCL died and nobody
noticed" case the check exists to catch. The real robot's ``map``->``base``
edge is composed of two live broadcasts, so a timer is what it actually looks
like.
"""

import json
import time

import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from mote_bringup.spec import mission

#: Passed to the task server as a parameter override rather than left to
#: identity: a developer's ``~/.mote`` has a real robot id in it, and a test
#: that addressed missions to whatever that happened to be would pass on one
#: machine and hang on the next.
PLATFORM = "mote-test"


class Localiser:
    """Broadcasts ``map``->``base_link`` at 10 Hz: the robot, at the origin."""

    def __init__(self, node, parent="map", children=("base_link",), period=0.1):
        self.node = node
        self.parent = parent
        self.children = tuple(children)
        self.broadcaster = tf2_ros.TransformBroadcaster(node)
        self.timer = node.create_timer(period, self.publish)
        self.publish()

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        for child in self.children:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(tf)


def localise(node, children=("base_link",)):
    return Localiser(node, children=children)


def command(capability, payload_input, **kwargs) -> dict:
    """A mission command as the fleet agent forwards one."""
    return mission.command(PLATFORM, capability, payload_input, **kwargs)


def send(publisher, capability, payload_input, **kwargs) -> dict:
    payload = command(capability, payload_input, **kwargs)
    publisher.publish(String(data=json.dumps(payload)))
    return payload


def states(statuses) -> list:
    return [status["state"] for status in statuses]


def failures(statuses) -> list:
    """``(state, failure class)`` for every status that carries one."""
    return [
        (status["state"], status["failure"]["class"])
        for status in statuses
        if status.get("failure")
    ]


def collect(node, statuses):
    """Subscribe to ``task/status`` and decode into ``statuses``."""
    return node.create_subscription(
        String, "task/status", lambda m: statuses.append(json.loads(m.data)), 10
    )


def spin_until(executor, condition, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)
    return condition()


def ready(executor, server, publisher, timeout=30.0):
    """Wait until a mission published here would actually be taken.

    Both halves matter and both are discovery races: the task server has to be
    subscribed, and it has to have seen enough TF to believe it is localised.
    Sending before the second one is a test that measures how fast DDS is
    today rather than what the executor does.
    """
    assert spin_until(
        executor, lambda: publisher.get_subscription_count() > 0, timeout
    ), "task_server never subscribed to task/command"
    assert spin_until(executor, server._localized, timeout), (
        "the mock world never localised the robot"
    )

"""twist_relay adds a header and nothing else."""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist

from mote_bringup.twist_relay import to_stamped


def _twist(lin_x=0.15, ang_z=-0.6):
    t = Twist()
    t.linear.x = lin_x
    t.angular.z = ang_z
    return t


def test_velocities_pass_through_untouched():
    out = to_stamped(_twist(), Time(sec=5, nanosec=250), "base_footprint")
    assert out.twist.linear.x == 0.15
    assert out.twist.angular.z == -0.6


def test_stamp_and_frame_are_applied():
    out = to_stamped(_twist(), Time(sec=5, nanosec=250), "base_footprint")
    assert out.header.stamp.sec == 5
    assert out.header.stamp.nanosec == 250
    assert out.header.frame_id == "base_footprint"


def test_a_zero_twist_stays_zero():
    """The stop command is the one that must never be embellished."""
    out = to_stamped(Twist(), Time(sec=1, nanosec=0), "base_footprint")
    assert out.twist.linear.x == 0.0
    assert out.twist.angular.z == 0.0

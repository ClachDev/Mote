"""FeetechBus register logic against a stub packet handler.

Two properties are safety-relevant on a bus shared with the drive wheels:
mode changes are EEPROM writes that must never happen blind (a servo whose
mode cannot be read is left untouched), and a servo is only reported as being
in position mode when a read-back confirms it — wheel mode obeys GOAL_SPEED,
so position goals to a misreported servo would spin it continuously.

The scservo_sdk import is lazy inside ``open()``, so these tests inject a stub
packet handler directly and never touch a serial port or the SDK.
"""

from mote_arm import bus as bus_mod
from mote_arm.bus import FeetechBus

COMM_OK = 0
COMM_FAIL = -1


class StubPacket:
    def __init__(self, modes=None, fail_mode_reads=0, fail_rate_writes=False):
        self.writes = []  # (servo_id, addr, value)
        self.modes = modes or {}
        self.fail_mode_reads = fail_mode_reads
        self.fail_rate_writes = fail_rate_writes
        self.mode_writes_take = True

    def read1ByteTxRx(self, _port, servo_id, addr):
        if addr == bus_mod._MODE:
            if self.fail_mode_reads > 0:
                self.fail_mode_reads -= 1
                return 0, COMM_FAIL, 0
            return self.modes.get(servo_id, 0), COMM_OK, 0
        return 0, COMM_OK, 0

    def write1ByteTxRx(self, _port, servo_id, addr, value):
        self.writes.append((servo_id, addr, value))
        if addr == bus_mod._MODE and self.mode_writes_take:
            self.modes[servo_id] = value
        if addr == bus_mod._ACC and self.fail_rate_writes:
            return COMM_FAIL, 0
        return COMM_OK, 0

    def write2ByteTxRx(self, _port, servo_id, addr, value):
        self.writes.append((servo_id, addr, value))
        return COMM_OK, 0


def make_bus(packet):
    bus = FeetechBus("/dev/fake", 1000000)
    bus._packet = packet
    bus._port = object()
    bus._comm_success = COMM_OK
    return bus


def test_position_mode_confirmed_without_writes():
    packet = StubPacket(modes={1: bus_mod._MODE_POSITION})
    assert make_bus(packet).ensure_position_mode(1) is True
    assert packet.writes == []


def test_unreadable_mode_writes_nothing():
    """A mode that cannot be read must not be blind-written to EEPROM."""
    packet = StubPacket(fail_mode_reads=99)
    assert make_bus(packet).ensure_position_mode(1) is False
    assert packet.writes == []


def test_wrong_mode_rewritten_and_verified():
    packet = StubPacket(modes={1: 1})  # wheel mode
    assert make_bus(packet).ensure_position_mode(1) is True
    assert (1, bus_mod._MODE, bus_mod._MODE_POSITION) in packet.writes
    locks = [w for w in packet.writes if w[1] == bus_mod._LOCK]
    assert [w[2] for w in locks] == [0, 1]


def test_mode_write_that_does_not_take_reports_failure():
    packet = StubPacket(modes={1: 1})
    packet.mode_writes_take = False
    assert make_bus(packet).ensure_position_mode(1) is False


def test_write_goal_rewrites_rates_only_on_change():
    packet = StubPacket()
    bus = make_bus(packet)
    bus.write_goal(1, 100, 600, 20)
    assert [w[1] for w in packet.writes] == [
        bus_mod._ACC,
        bus_mod._GOAL_SPEED,
        bus_mod._GOAL_POSITION,
    ]

    packet.writes.clear()
    bus.write_goal(1, 200, 600, 20)
    assert [w[1] for w in packet.writes] == [bus_mod._GOAL_POSITION]

    packet.writes.clear()
    bus.write_goal(1, 300, 700, 20)
    assert [w[1] for w in packet.writes] == [
        bus_mod._ACC,
        bus_mod._GOAL_SPEED,
        bus_mod._GOAL_POSITION,
    ]


def test_write_goal_retries_rates_until_they_land():
    packet = StubPacket(fail_rate_writes=True)
    bus = make_bus(packet)
    bus.write_goal(1, 100, 600, 20)
    bus.write_goal(1, 200, 600, 20)
    assert [w[1] for w in packet.writes].count(bus_mod._ACC) == 2

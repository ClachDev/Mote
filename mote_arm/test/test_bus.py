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


class RaisingPacket:
    """A packet handler that fails the way scservo_sdk really fails.

    ``read2ByteTxRx`` indexes the reply buffer before checking its length, so a
    dropped packet surfaces as IndexError rather than a failure code. On the
    real arm that crashed a calibration run *between* two EEPROM writes.
    """

    def __init__(self, raise_on=IndexError):
        self.raise_on = raise_on

    def read1ByteTxRx(self, _port, _servo_id, _addr):
        raise self.raise_on("list index out of range")

    def read2ByteTxRx(self, _port, _servo_id, _addr):
        raise self.raise_on("list index out of range")

    def ping(self, _port, _servo_id):
        raise self.raise_on("list index out of range")


def _raising_bus(exc=IndexError):
    b = FeetechBus("/dev/null", 1_000_000)
    b._packet = RaisingPacket(exc)
    b._comm_success = COMM_OK
    return b


def test_a_short_reply_reads_as_none_not_an_exception():
    b = _raising_bus()
    assert b.read_position(1) is None
    assert b.read_health(1) is None
    assert b.read_homing_offset(1) is None
    assert b.read_homing_offset_raw(1) is None
    assert b.read_gains(1) is None
    assert b.ensure_position_mode(1) is False
    assert b.ping(1) is False


def test_other_sdk_unpacking_failures_are_caught_too():
    for exc in (TypeError, IndexError):
        assert _raising_bus(exc).read_position(1) is None


class OffsetPacket:
    """Enough of the register file to exercise the offset read/write path."""

    def __init__(self, initial=0):
        self.words = {bus_mod._HOMING_OFFSET: initial}
        self.bytes = {}

    def read1ByteTxRx(self, _port, _servo_id, addr):
        return self.bytes.get(addr, 0), COMM_OK, 0

    def read2ByteTxRx(self, _port, _servo_id, addr):
        return self.words.get(addr, 0), COMM_OK, 0

    def write1ByteTxRx(self, _port, _servo_id, addr, value):
        self.bytes[addr] = value
        return COMM_OK, 0

    def write2ByteTxRx(self, _port, _servo_id, addr, value):
        self.words[addr] = value
        return COMM_OK, 0


def _offset_bus(initial=0):
    b = FeetechBus("/dev/null", 1_000_000)
    b._packet = OffsetPacket(initial)
    b._comm_success = COMM_OK
    return b


def test_offset_write_round_trips_through_sign_magnitude():
    for value in (0, 700, -700, 2047, -2047):
        b = _offset_bus()
        assert b.write_homing_offset(1, value) is True
        assert b.read_homing_offset(1) == value


def test_negative_offset_is_stored_with_the_sign_bit_set():
    b = _offset_bus()
    b.write_homing_offset(1, -1040)
    assert b._packet.words[bus_mod._HOMING_OFFSET] == (1040 | bus_mod.OFFSET_SIGN_BIT)


def test_offset_write_leaves_the_eeprom_lock_re_engaged():
    b = _offset_bus()
    b.write_homing_offset(1, 500)
    assert b._packet.bytes[bus_mod._LOCK] == 1


class StalePacket:
    """Returns the previous register's reply once, the way the real bus did.

    After an EEPROM write, a read of PRESENT_POSITION came back with the value
    of the register read just before it — 3902, which is -1854 in the offset
    encoding. A single read cannot tell that apart from a real position.
    """

    def __init__(self, stale, real):
        self.stale = stale
        self.real = real
        self.reads = 0

    def read2ByteTxRx(self, _port, _servo_id, _addr):
        self.reads += 1
        return (self.stale if self.reads == 1 else self.real), COMM_OK, 0


def _stale_bus(stale, real):
    b = FeetechBus("/dev/null", 1_000_000)
    b._packet = StalePacket(stale, real)
    b._comm_success = COMM_OK
    return b


def test_settled_read_rejects_a_stale_reply_and_retries():
    b = _stale_bus(stale=3902, real=2923)
    assert b.read_position_settled(1) == 2923


def test_settled_read_tolerates_a_limp_arm_drifting_a_count():
    b = _stale_bus(stale=2001, real=2000)  # 1 count apart: same position
    assert b.read_position_settled(1) == 2000


def test_settled_read_gives_up_rather_than_guessing():
    class Never:
        def __init__(self):
            self.n = 0

        def read2ByteTxRx(self, _p, _s, _a):
            self.n += 1
            return (self.n * 500) % 4096, COMM_OK, 0

    b = FeetechBus("/dev/null", 1_000_000)
    b._packet = Never()
    b._comm_success = COMM_OK
    assert b.read_position_settled(1, attempts=3) is None


def test_reads_clear_a_stale_input_buffer_first():
    class Port:
        def __init__(self):
            self.cleared = 0

        def clearPort(self):
            self.cleared += 1

    b = _stale_bus(1, 1)
    b._port = Port()
    b.read_position(1)
    assert b._port.cleared == 1

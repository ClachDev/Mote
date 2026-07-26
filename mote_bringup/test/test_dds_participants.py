"""The participant counter reads /proc, so the port arithmetic is the part worth
pinning down: it is what turns "a bound UDP port" into "a participant slot"."""

from mote_bringup import dds_participants as dds


def test_rtps_port_mapping():
    # Domain 0, indices 0.. -> 7410, 7412, ... (PB 7400 + d1 10 + PG 2*index)
    assert dds.discovery_port(0, 0) == 7410
    assert dds.discovery_port(0, 1) == 7412
    assert dds.discovery_port(0, 32) == 7474
    # Domain 47 shifts the whole block by DG*47.
    assert dds.discovery_port(47, 0) == 19160
    assert dds.discovery_port(47, 3) == 19166


def test_scan_reports_headroom_against_the_cap(monkeypatch):
    monkeypatch.setattr(dds, "bound_udp_ports", lambda: {7410: [1], 7414: [2]})
    monkeypatch.setattr(dds, "inode_owners", lambda: {1: (100, "ros2 launch")})
    result = dds.scan(domain=0, max_index=32)
    assert result["used"] == 2
    assert result["capacity"] == 33
    assert result["free"] == 31
    assert [p["index"] for p in result["participants"]] == [0, 2]
    assert result["participants"][0]["command"] == "ros2 launch"
    # A socket owned by another user still counts as a claimed slot.
    assert result["participants"][1]["pid"] is None


def test_ports_beyond_the_cap_are_not_counted(monkeypatch):
    monkeypatch.setattr(dds, "bound_udp_ports", lambda: {7410: [1], 7476: [2]})
    monkeypatch.setattr(dds, "inode_owners", lambda: {})
    assert dds.scan(domain=0, max_index=32)["used"] == 1


def test_reads_the_real_proc_table():
    """Smoke: /proc/net/udp parses on this host (it is the only data source)."""
    ports = dds.bound_udp_ports()
    assert isinstance(ports, dict)
    assert all(isinstance(p, int) and 0 <= p <= 65535 for p in ports)

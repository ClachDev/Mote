#!/usr/bin/env python3
"""Unit tests for the sim domain picker.

Runnable standalone (`python test_sim_domain.py`) or under pytest. The port
probe is faked, so this needs neither ROS, a sim, nor any bound socket — what is
pinned is the arithmetic that decides a domain is free and the shell contract
the smoke test and map_world.sh eval.
"""

import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_domain


def _with_busy(ports, env):
    """Run pick_domain_id() seeing exactly ``ports`` bound and ``env`` set."""
    real_probe = sim_domain.bound_udp_ports
    sim_domain.bound_udp_ports = lambda: set(ports)
    try:
        return sim_domain.pick_domain_id(env)
    finally:
        sim_domain.bound_udp_ports = real_probe


def test_inherited_domain_wins():
    """A caller who set ROS_DOMAIN_ID meant it — the sweep assigns one per run."""
    domain, how = _with_busy([], {"ROS_DOMAIN_ID": "42"})
    assert (domain, how) == (42, "inherited")


def test_claims_a_domain_with_no_bound_ports():
    domain, how = _with_busy([], {})
    assert how == "claimed"
    assert domain in sim_domain.DOMAIN_CANDIDATES


def test_avoids_a_domain_with_any_port_in_its_block():
    """One bound port anywhere in a domain's 250-port block rules it out — a
    LOCALHOST-range participant binds unicast ports only, never the base."""
    busy = set()
    for domain in sim_domain.DOMAIN_CANDIDATES:
        if domain != 37:
            # +11 is a unicast participant port, not the multicast base.
            busy.add(sim_domain.DDS_PORT_BASE + sim_domain.DDS_PORT_STEP * domain + 11)
    for _ in range(20):  # the candidate order is randomised
        assert _with_busy(busy, {}) == (37, "claimed")


def test_falls_back_to_zero_when_every_domain_is_busy():
    busy = {
        sim_domain.DDS_PORT_BASE + sim_domain.DDS_PORT_STEP * d
        for d in sim_domain.DOMAIN_CANDIDATES
    }
    domain, how = _with_busy(busy, {})
    assert domain == 0 and how.startswith("fallback")


def test_claim_writes_env_and_keeps_an_inherited_partition():
    env = {"ROS_DOMAIN_ID": "7"}
    domain, how, partition = sim_domain.claim("mote-smoke", env)
    assert (domain, how, partition) == (7, "inherited", "mote-smoke-7")
    assert env["ROS_DOMAIN_ID"] == "7" and env["GZ_PARTITION"] == "mote-smoke-7"

    env = {"ROS_DOMAIN_ID": "7", "GZ_PARTITION": "mine"}
    assert sim_domain.claim("mote-smoke", env)[2] == "mine"


def test_shell_output_is_evalable():
    """The exact contract run_sim_smoke.sh and map_world.sh depend on: eval the
    stdout, then read $ROS_DOMAIN_ID / $GZ_PARTITION / $MOTE_DOMAIN_HOW."""
    env = dict(os.environ, ROS_DOMAIN_ID="5")
    env.pop("GZ_PARTITION", None)
    out = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("sim_domain.py")),
            "--shell",
            "--prefix",
            "mote-map",
        ],
        capture_output=True,
        text=True,
        check=True,
        env=env,
    ).stdout
    shown = subprocess.run(
        ["bash", "-c", f'{out}\necho "$ROS_DOMAIN_ID $GZ_PARTITION $MOTE_DOMAIN_HOW"'],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    assert shown == "5 mote-map-5 inherited"


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                failures += 1
                print(f"FAIL {name}: {e}")
    print("ALL PASS" if not failures else f"{failures} FAILED")
    sys.exit(1 if failures else 0)

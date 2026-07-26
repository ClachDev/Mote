"""Unit tests for on-demand model loading (tools/model_host.py).

The inference machine may be someone's gaming PC, so the servers must not hold
VRAM while idle. These cover the load/release cycle without torch or a model.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from model_host import ModelHost  # noqa: E402


def _counting_loader():
    calls = []

    def load():
        calls.append(1)
        return f"model-{len(calls)}"

    return load, calls


def test_does_not_load_until_first_use():
    load, calls = _counting_loader()
    host = ModelHost(load, idle_timeout=60, log=lambda m: None)
    assert not host.loaded
    assert calls == []


def test_loads_once_then_reuses():
    load, calls = _counting_loader()
    host = ModelHost(load, idle_timeout=60, log=lambda m: None)
    assert host.get() == "model-1"
    assert host.get() == "model-1"
    assert calls == [1]
    assert host.loaded


def test_releases_when_idle_and_reloads_on_next_use():
    load, calls = _counting_loader()
    host = ModelHost(load, idle_timeout=0.05, log=lambda m: None)
    host.get()
    assert host.release_if_idle() is False  # just used
    time.sleep(0.1)
    assert host.release_if_idle() is True
    assert not host.loaded
    # The next request transparently reloads — the caller never sees the gap.
    assert host.get() == "model-2"
    assert calls == [1, 1]


def test_zero_timeout_never_releases():
    load, _ = _counting_loader()
    host = ModelHost(load, idle_timeout=0, log=lambda m: None)
    host.get()
    time.sleep(0.05)
    assert host.release_if_idle() is False
    assert host.loaded


def test_release_before_any_load_is_a_noop():
    load, calls = _counting_loader()
    host = ModelHost(load, idle_timeout=0.01, log=lambda m: None)
    assert host.release_if_idle() is False
    assert calls == []

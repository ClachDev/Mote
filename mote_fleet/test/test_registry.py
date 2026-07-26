"""Allocation and idempotency — the two properties the id space depends on."""

import sqlite3
import threading

import pytest
from registry import Registry, RegistryError


@pytest.fixture
def registry(tmp_path):
    return Registry(tmp_path / "registry.db")


def enroll(registry, fingerprint, **kwargs):
    token = kwargs.pop("token", None) or registry.new_token()
    return registry.enroll(token=token, fingerprint=fingerprint, **kwargs)


def test_ids_are_allocated_in_order(registry):
    first, _ = enroll(registry, "serial:aaa")
    second, _ = enroll(registry, "serial:bbb")
    assert (first["robot_id"], second["robot_id"]) == ("mote-01", "mote-02")


def test_re_enrolling_the_same_machine_returns_the_same_id(registry):
    first, created = enroll(registry, "serial:aaa", name="Scout")
    again, created_again = enroll(registry, "serial:aaa")
    assert first["robot_id"] == again["robot_id"]
    assert (created, created_again) == (True, False)
    assert len(registry.robots()) == 1


def test_re_enrolling_keeps_the_name_unless_a_new_one_is_given(registry):
    enroll(registry, "serial:aaa", name="Scout")
    again, _ = enroll(registry, "serial:aaa")
    assert again["name"] == "Scout"
    renamed, _ = enroll(registry, "serial:aaa", name="Rover")
    assert renamed["name"] == "Rover"


def test_an_m0_robot_can_bring_its_own_id(registry):
    """The upgrade path: an operator-set id is adopted, not renumbered."""
    robot, created = enroll(registry, "serial:aaa", requested_id="mote-07")
    assert (robot["robot_id"], created) == ("mote-07", True)
    # And the next allocation still starts from the low end.
    other, _ = enroll(registry, "serial:bbb")
    assert other["robot_id"] == "mote-01"


def test_a_requested_id_that_belongs_to_another_machine_is_refused(registry):
    enroll(registry, "serial:aaa", requested_id="mote-07")
    with pytest.raises(RegistryError, match="already taken"):
        enroll(registry, "serial:bbb", requested_id="mote-07")


def test_an_enrolled_machine_cannot_silently_re_key_itself(registry):
    enroll(registry, "serial:aaa")
    with pytest.raises(RegistryError, match="already enrolled"):
        enroll(registry, "serial:aaa", requested_id="mote-09")


def test_ids_fill_gaps_left_by_a_removed_robot(registry):
    enroll(registry, "serial:aaa")
    enroll(registry, "serial:bbb")
    with sqlite3.connect(registry.path) as conn:
        conn.execute("DELETE FROM robots WHERE robot_id = 'mote-01'")
    robot, _ = enroll(registry, "serial:ccc")
    assert robot["robot_id"] == "mote-01"


def test_facts_are_recorded_for_audit(registry):
    robot, _ = enroll(registry, "serial:aaa", facts={"model": "Pi 5", "mac": "aa:bb"})
    assert registry.robot("mote-01")["facts"]["model"] == "Pi 5"
    assert robot["fingerprint"] == "serial:aaa"


# ---- tokens -------------------------------------------------------------


def test_a_single_use_token_enrols_exactly_one_robot(registry):
    token = registry.new_token(single_use=True)
    enroll(registry, "serial:aaa", token=token)
    with pytest.raises(RegistryError, match="already used"):
        enroll(registry, "serial:bbb", token=token)


def test_a_single_use_token_still_works_for_its_own_robot(registry):
    """Re-running enroll on the same robot must not need a new token."""
    token = registry.new_token(single_use=True)
    enroll(registry, "serial:aaa", token=token)
    again, created = enroll(registry, "serial:aaa", token=token)
    assert (again["robot_id"], created) == ("mote-01", False)


def test_a_reusable_token_enrols_a_bench(registry):
    token = registry.new_token(single_use=False)
    enroll(registry, "serial:aaa", token=token)
    enroll(registry, "serial:bbb", token=token)
    assert len(registry.robots()) == 2


def test_an_unknown_token_is_refused(registry):
    with pytest.raises(RegistryError, match="unknown enrollment token"):
        registry.enroll(token="not-a-token", fingerprint="serial:aaa")
    assert registry.robots() == []


def test_a_refused_enrollment_leaves_no_row(registry):
    """The whole exchange is one transaction: no half-enrolled robots."""
    registry.enroll(token=registry.new_token(), fingerprint="serial:aaa")
    with pytest.raises(RegistryError):
        registry.enroll(token="bogus", fingerprint="serial:bbb")
    assert [r["robot_id"] for r in registry.robots()] == ["mote-01"]


def test_concurrent_enrollments_do_not_collide(registry):
    """Allocation reads the table to pick the next id, so it has to be
    serialised — two robots booting together must not both become mote-01."""
    token = registry.new_token(single_use=False)
    results, errors = [], []

    def worker(index):
        try:
            robot, _ = registry.enroll(token=token, fingerprint=f"serial:{index}")
            results.append(robot["robot_id"])
        except Exception as exc:  # surfaced by the assertion below
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert not errors, errors
    assert sorted(results) == [f"mote-{i:02d}" for i in range(1, 9)]

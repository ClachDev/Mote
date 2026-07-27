"""Broker credentials: the hash format, the ACL, and the namespace split.

These are the pure-function half of M7. The half that matters more — that a real
mosquitto *enforces* what ``render_acl`` writes — is ``test_broker_acl.py``,
which needs a broker; this file needs nothing and therefore runs everywhere.

The one test here that reaches outside is the hash comparison against the real
``mosquitto_passwd``, which is what stops our reimplementation of the ``$7$``
format from drifting away from the broker that has to read it.
"""

import os
import shutil
import stat
import subprocess

import credentials
import pytest

from mote_fleet import protocol


def mosquitto_passwd() -> str | None:
    """conda-forge puts the clients in ``bin`` (unlike the broker, which is in
    ``sbin``), so PATH is enough for this one."""
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        candidate = os.path.join(prefix, "bin", "mosquitto_passwd")
        if os.access(candidate, os.X_OK):
            return candidate
    return shutil.which("mosquitto_passwd")


# ---- passwords ----------------------------------------------------------


def test_a_hash_round_trips():
    password = credentials.new_password()
    assert credentials.verify_password(password, credentials.hash_password(password))


def test_a_wrong_password_does_not_verify():
    encoded = credentials.hash_password("hunter2")
    assert not credentials.verify_password("hunter3", encoded)


def test_the_same_password_hashes_differently_each_time():
    """A fresh salt per call, so two robots with the same password — or one
    robot's two rotations — are not visibly the same in the password file."""
    first = credentials.hash_password("same")
    second = credentials.hash_password("same")
    assert first != second
    assert credentials.verify_password("same", first)
    assert credentials.verify_password("same", second)


def test_the_encoded_shape_is_mosquittos():
    encoded = credentials.hash_password("pw", salt=b"0123456789ab")
    marker, tag, iterations, salt, key = encoded.split("$")
    assert (marker, tag, iterations) == ("", "7", "101")
    assert salt == "MDEyMzQ1Njc4OWFi"
    assert len(key) == 88  # 64 bytes, base64


def test_a_garbled_hash_is_refused_rather_than_raising():
    for bad in ("", "$7$", "notahash", "$6$101$aaaa$bbbb", "$7$x$y$z"):
        assert credentials.verify_password("pw", bad) is False


@pytest.mark.skipif(mosquitto_passwd() is None, reason="needs mosquitto_passwd")
def test_our_hasher_agrees_with_mosquitto_passwd(tmp_path):
    """The contract that matters: a hash mosquitto wrote, verified by us.

    Compared this way round rather than by generating one and asking mosquitto,
    because ``mosquitto_passwd`` has no verify mode — and this direction is the
    one that proves we can read what the broker reads.
    """
    path = tmp_path / "passwd"
    subprocess.run(
        [mosquitto_passwd(), "-c", "-b", str(path), "someone", "s3cret"], check=True
    )
    _, encoded = path.read_text().strip().split(":", 1)
    assert credentials.verify_password("s3cret", encoded)
    assert not credentials.verify_password("s3crer", encoded)


# ---- the namespace split ------------------------------------------------


def test_an_operator_username_can_never_be_a_robot_id():
    """The whole reason operator names carry an underscore.

    Robot ids are lowercase DNS labels, so the two namespaces cannot overlap and
    there is no precedence rule to get wrong — which is what lets the ACL grant
    robot rights by username without checking anything at runtime.
    """
    for name in ("michael", "Night Shift", "a", "!!!", "mote-01", "x" * 60):
        username = credentials.operator_username(name)
        assert username.startswith(credentials.OPERATOR_PREFIX)
        assert not protocol.valid_id(username), username


def test_the_server_username_can_never_be_a_robot_id():
    assert not protocol.valid_id(credentials.SERVER_USER)


def test_operator_usernames_are_unique_per_mint():
    names = {credentials.operator_username("michael") for _ in range(50)}
    assert len(names) > 1


def test_a_robot_id_is_its_own_broker_username():
    assert credentials.is_robot_username("mote-01")
    assert not credentials.is_robot_username(credentials.SERVER_USER)


# ---- the generated files ------------------------------------------------


def test_the_password_file_is_sorted_and_one_line_per_user():
    text = credentials.render_passwd({"b": "$7$b", "a": "$7$a"})
    assert text == "a:$7$a\nb:$7$b\n"


def test_an_empty_password_file_is_empty_not_blank_lines():
    assert credentials.render_passwd({}) == ""


def test_a_robot_gets_its_own_branch_and_nothing_else():
    acl = credentials.render_acl(robots=["mote-01", "mote-02"])
    block = _block(acl, "mote-01")
    assert "topic write mote/v1/mote-01/health" in block
    assert "topic read mote/v1/mote-01/task/command" in block
    # The acceptance criterion, as a property of the generated text.
    assert "mote-02" not in block
    assert "topic read mote/v1/mote-01/health" not in block


def test_an_operator_may_read_the_fleet_and_write_nothing():
    acl = credentials.render_acl(operators=["op_michael_1a2b"])
    block = _block(acl, "op_michael_1a2b")
    assert "topic read mote/v1/+/health" in block
    assert "topic read mote/v1/+/pose" in block
    assert "topic write" not in block
    # Not the command topic: an operator reads who dispatched what from the
    # audit log, which is attributable, rather than off the broker, which is not.
    assert "task/command" not in block


def test_only_the_server_may_write_a_command():
    acl = credentials.render_acl(
        robots=["mote-01"], operators=["op_a_1111", "op_b_2222"]
    )
    writers = [
        line for line in acl.splitlines() if line.startswith("topic write mote/v1/+/")
    ]
    assert writers == [f"topic write mote/v1/+/{protocol.COMMAND}"]
    assert _block(acl, credentials.SERVER_USER).count("topic write") == 1


def test_the_acl_has_no_rule_outside_a_user_block():
    """A ``topic`` line before any ``user`` line is a rule for *everyone*, which
    would quietly undo the whole file. Assert the file never opens with one."""
    acl = credentials.render_acl(robots=["mote-01"], operators=["op_a_1111"])
    lines = [
        line for line in acl.splitlines() if line.strip() and not line.startswith("#")
    ]
    assert lines[0].startswith("user ")
    assert not any(line.startswith("pattern") for line in lines)


def test_the_generated_files_are_private(tmp_path):
    auth = credentials.BrokerAuth(tmp_path / "broker")
    auth.write(users={"mote-01": "$7$x"}, robots=["mote-01"], operators=[])
    for path in (auth.passwd_path, auth.acl_path):
        assert stat.S_IMODE(path.stat().st_mode) == 0o600, path


def test_writing_twice_replaces_rather_than_appends(tmp_path):
    auth = credentials.BrokerAuth(tmp_path / "broker")
    auth.write(users={"mote-01": "$7$x"}, robots=["mote-01"], operators=[])
    auth.write(users={"mote-02": "$7$y"}, robots=["mote-02"], operators=[])
    # The point of regenerating from the registry: a robot that is gone from the
    # rows is gone from the file, without anything remembering to delete it.
    assert "mote-01" not in auth.passwd_path.read_text()
    assert "mote-01" not in auth.acl_path.read_text()


def test_no_reload_command_is_not_a_failure(tmp_path):
    auth = credentials.BrokerAuth(tmp_path / "broker")
    ok, _ = auth.reload()
    assert ok


def test_a_failing_reload_is_reported_not_raised(tmp_path):
    auth = credentials.BrokerAuth(tmp_path / "broker", reload_cmd=["false"])
    ok, _ = auth.reload()
    assert not ok
    assert auth.last_reload_error == ""  # `false` says nothing; the code is the news


def test_a_missing_reload_command_is_reported_not_raised(tmp_path):
    auth = credentials.BrokerAuth(
        tmp_path / "broker", reload_cmd=["/nonexistent/broker.sh", "reload"]
    )
    ok, detail = auth.reload()
    assert not ok
    assert detail


def _block(acl: str, user: str) -> str:
    """The lines belonging to one ``user`` stanza."""
    out, collecting = [], False
    for line in acl.splitlines():
        if line.startswith("user "):
            collecting = line == f"user {user}"
            continue
        if collecting:
            out.append(line)
    return "\n".join(out)

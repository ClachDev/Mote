"""Broker credentials: who may connect, and what they may say.

M1 shipped an anonymous broker and said so in three places. This module is what
replaces it (fleet.md Q7, milestone M7): every client authenticates, and every
client is confined by an ACL derived from the registry — so *the topic tree
stops being a convention and becomes an enforced boundary*. A robot that is
compromised, misconfigured, or simply running the wrong ``robot_id`` cannot read
another robot's commands or forge another robot's health.

Three principals, and their namespaces are **disjoint by construction** rather
than by a runtime check:

===============  ==========================  ====================================
principal        broker username             may
===============  ==========================  ====================================
robot            its ``robot_id``            publish its own presence/health/pose/
                                             task-status; subscribe its own
                                             task/command — nothing else
operator         ``op_<slug>_<4 hex>``       subscribe the fleet-wide read topics;
                                             **publish nothing, anywhere**
fleet server     ``fleet_server``            publish task/command for any robot;
                                             subscribe the whole tree
===============  ==========================  ====================================

A ``robot_id`` is a lowercase DNS label (``protocol.ID_RE``), so it can never
contain an underscore — which is exactly why the other two usernames carry one.
No enrolled robot can ever collide with an operator or with the server, and
there is no ordering or precedence question to get wrong. ``test_credentials``
asserts that property directly rather than trusting this paragraph.

**Why generate mosquitto's hash here rather than shell out to
``mosquitto_passwd``.** The fleet server's whole dependency list is "python"
(``fleet_server.py``), and it may well run in a container that has no broker
binary in it at all. The format is published and stable — ``$7$`` is PBKDF2-
HMAC-SHA512, 101 iterations, 12-byte salt, 64-byte key, both base64 — so
``hashlib`` produces it in six lines. ``test_credentials.py`` checks the output
against the real ``mosquitto_passwd`` wherever one exists, so this cannot drift
from the broker that has to read it.

**Why the ACL enumerates robots instead of using mosquitto's ``pattern`` /
``%u`` substitution.** One ``pattern read mote/v1/%u/task/command`` line would
cover every robot forever and never need regenerating. It is rejected because
the password file has to be regenerated on every enrollment regardless, so the
pattern buys no reload the fleet was not already doing — and it would silently
extend robot-shaped rights to *any* authenticated username, including
operators. The enumerated form is what an operator can read top to bottom and
confirm that ``mote-01`` appears in exactly its own five lines.
"""

import base64
import hashlib
import os
import re
import secrets
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mote_fleet import protocol  # noqa: E402

#: How a running broker is told to re-read the files this module writes.
#: ``broker.sh`` is the only thing that knows whether the broker it started is a
#: local process or a container, so the signal goes through it rather than
#: through a pid this module would have to guess at.
DEFAULT_RELOAD_CMD = [str(Path(__file__).resolve().parent / "broker.sh"), "reload"]

#: mosquitto's ``$7$`` password hash: PBKDF2-HMAC-SHA512. The iteration count is
#: part of the encoded hash, so raising it here is backwards compatible — old
#: entries keep verifying at their own count until they are next rotated.
HASH_TAG = "7"
ITERATIONS = 101
SALT_BYTES = 12
KEY_BYTES = 64

#: The server's own broker identity. Underscore, so it cannot be a ``robot_id``.
SERVER_USER = "fleet_server"

#: Operator usernames. Same reasoning; the slug is only there to make a broker
#: log line legible, and the suffix is what makes it unique.
OPERATOR_PREFIX = "op_"

#: What an operator's browser subscribes to: the read half of the control plane,
#: every robot. Deliberately *not* ``mote/v1/#`` — that would include
#: ``task/command``, and an operator has no business reading commands issued to
#: a robot by somebody else out of the broker rather than out of the audit log.
OPERATOR_READ = (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS)

#: What a robot publishes. The mirror of OPERATOR_READ, scoped to itself.
ROBOT_WRITE = (protocol.PRESENCE, protocol.HEALTH, protocol.POSE, protocol.STATUS)

#: ...and the one topic it reads.
ROBOT_READ = (protocol.COMMAND,)

_SLUG_RE = re.compile(r"[^a-z0-9]+")


# ---- passwords -----------------------------------------------------------


def new_password() -> str:
    """A broker password. URL-safe so it survives every config file and CLI
    quoting rule between here and the robot."""
    return secrets.token_urlsafe(24)


def hash_password(password: str, *, salt: bytes | None = None) -> str:
    """A password in mosquitto's ``password_file`` format.

    ``$7$<iterations>$<base64 salt>$<base64 key>`` — verified byte for byte
    against ``mosquitto_passwd`` in the tests.
    """
    salt = secrets.token_bytes(SALT_BYTES) if salt is None else salt
    key = hashlib.pbkdf2_hmac("sha512", password.encode(), salt, ITERATIONS, KEY_BYTES)
    return (
        f"${HASH_TAG}${ITERATIONS}$"
        f"{base64.b64encode(salt).decode()}${base64.b64encode(key).decode()}"
    )


def verify_password(password: str, encoded: str) -> bool:
    """Does ``password`` produce ``encoded``? Used by the tests, and by anyone
    debugging a broker that says "bad username or password"."""
    try:
        _, tag, iterations, salt_b64, key_b64 = encoded.split("$")
        if tag != HASH_TAG:
            return False
        salt = base64.b64decode(salt_b64)
        expected = base64.b64decode(key_b64)
    except (ValueError, TypeError):
        return False
    key = hashlib.pbkdf2_hmac(
        "sha512", password.encode(), salt, int(iterations), len(expected)
    )
    return secrets.compare_digest(key, expected)


def operator_username(name: str) -> str:
    """``op_michael_3f9a`` — legible in a broker log, unique, and impossible to
    confuse with a ``robot_id``."""
    slug = _SLUG_RE.sub("_", name.strip().lower()).strip("_")[:24] or "operator"
    return f"{OPERATOR_PREFIX}{slug}_{secrets.token_hex(2)}"


def is_robot_username(username: str) -> bool:
    return protocol.valid_id(username)


# ---- the two files mosquitto reads ---------------------------------------


def render_passwd(users: dict) -> str:
    """``username:hash`` per line, sorted so the file only changes when the
    credentials do (a diff of noise is a diff nobody reads)."""
    lines = [f"{user}:{hashed}" for user, hashed in sorted(users.items())]
    return "\n".join(lines) + ("\n" if lines else "")


def render_acl(*, robots=(), operators=(), server_user: str = SERVER_USER) -> str:
    """The ACL file, generated from the registry.

    Read it as the fleet's authorization policy in full: there is no default
    rule, no ``topic`` line outside a ``user`` block, and therefore nothing any
    principal may do that is not written here. ``allow_anonymous false`` in
    ``mosquitto.conf`` is the other half — a client with no username never gets
    as far as this file.
    """
    root = f"{protocol.ROOT}/{protocol.VERSION}"
    out = [
        "# Generated by the mote fleet server from the registry — do not edit.",
        "# Regenerated on every enrollment and every operator change; the broker",
        "# re-reads it on SIGHUP (mote_fleet/server/broker.sh reload).",
        "#",
        "# There is no rule outside a `user` block, so nothing is permitted that",
        "# is not written below, and mosquitto.conf refuses anonymous clients.",
        "",
        f"user {server_user}",
        "# The fleet API: the only principal that may issue a command, and it may",
        "# only do so after authorizing an operator and writing the audit row.",
        f"topic write {root}/+/{protocol.COMMAND}",
        f"topic read {root}/#",
        "",
    ]
    for user in sorted(operators):
        out.append(f"user {user}")
        out.append("# An operator: the read half of the control plane, fleet-wide.")
        out.append("# No write rule at all — dispatch is the fleet API's, not theirs.")
        out.extend(f"topic read {root}/+/{leaf}" for leaf in OPERATOR_READ)
        out.append("")
    for robot_id in sorted(robots):
        out.append(f"user {robot_id}")
        out.append(f"# {robot_id}: its own branch of the tree and nothing else.")
        out.extend(f"topic write {root}/{robot_id}/{leaf}" for leaf in ROBOT_WRITE)
        out.extend(f"topic read {root}/{robot_id}/{leaf}" for leaf in ROBOT_READ)
        out.append("")
    return "\n".join(out)


def write_private(path: Path, text: str) -> Path:
    """Write ``text`` to ``path`` atomically, readable only by this user.

    Atomic because mosquitto may re-read either file at any moment — a SIGHUP it
    was already handling, or a restart — and half a password file is a fleet
    that cannot connect. ``0600`` set on the temporary file *before* it is
    renamed, so the secret is never briefly world-readable.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(text)
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)
    return path


class BrokerAuth:
    """The generated ``password_file`` + ``acl_file`` pair, and the reload.

    Owned by the fleet server, which regenerates both from the registry whenever
    a credential changes: a robot enrolls, an operator is minted, an operator is
    revoked. Nothing else writes them, and hand-edits are lost on the next
    enrollment — which is why both files say so at the top.
    """

    def __init__(self, directory, *, reload_cmd=None):
        self.directory = Path(directory).expanduser()
        self.passwd_path = self.directory / "passwd"
        self.acl_path = self.directory / "acl"
        self.reload_cmd = reload_cmd
        self.last_reload_error = ""

    def write(self, *, users: dict, robots=(), operators=()) -> None:
        write_private(self.passwd_path, render_passwd(users))
        write_private(self.acl_path, render_acl(robots=robots, operators=operators))

    def reload(self) -> tuple[bool, str]:
        """Ask the broker to re-read both files. ``(ok, detail)``.

        A credential the broker has not read yet is a robot that cannot connect,
        so the failure is reported to the caller rather than swallowed — but it
        is never fatal: the files are correct on disk and the next broker start
        picks them up.
        """
        if not self.reload_cmd:
            return True, "no reload command configured"
        try:
            done = subprocess.run(
                self.reload_cmd,
                shell=isinstance(self.reload_cmd, str),
                capture_output=True,
                text=True,
                timeout=15,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.last_reload_error = str(exc)
            return False, str(exc)
        if done.returncode != 0:
            detail = (done.stderr or done.stdout or "").strip()
            self.last_reload_error = detail
            return False, detail
        self.last_reload_error = ""
        return True, (done.stdout or "").strip()


def sync(registry, directory, *, reload_cmd=DEFAULT_RELOAD_CMD) -> tuple[bool, str]:
    """Regenerate both files from ``registry`` and reload the broker.

    The one entry point for "make the broker agree with the registry", shared by
    the fleet server (which calls it on every enrollment) and ``fleetctl``
    (which calls it when an operator is minted or revoked). ``registry`` is
    duck-typed on ``broker_principals()`` rather than imported, because
    ``registry.py`` imports *this* module.
    """
    auth = BrokerAuth(directory, reload_cmd=reload_cmd)
    auth.write(**registry.broker_principals())
    return auth.reload()

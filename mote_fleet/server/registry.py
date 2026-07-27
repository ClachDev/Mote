"""The fleet registry: robots, enrollment tokens, operators, and the audit log.

This is the server side of "identity is server-allocated" (fleet.md Q3). M0 let
the operator type an id into ``~/.mote/robot.yaml``; from M1 the id space has an
owner, and this module is it. SQLite because the store is a handful of rows that
must survive a restart and be queried by exactly one process — a file the whole
registry fits in beats a database service at this size, and the schema is small
enough that the Regime-B/C move to a real DB is a rewrite of this file alone.

M3 adds the two tables the dispatch API needs: **operators**, whose tokens are
the credential the API authorizes a dispatch against, and **audit**, the record
it writes before publishing. Every table is created ``IF NOT EXISTS`` on open,
so an M1 registry file gains them by being opened by an M3 server.

M7 makes this file the source of truth for **broker credentials** as well
(:mod:`credentials`): a robot's is issued at enrollment, an operator's is minted
with their token, and the server's own lives in ``settings``. Nothing else knows
who may connect to the broker — the ``password_file`` and ``acl_file`` mosquitto
reads are *generated* from these rows, which is what makes revoking an operator
close their HTTP and their MQTT access in the same step. Columns added after the
table already exists are applied by :func:`_migrate`, since ``CREATE TABLE IF
NOT EXISTS`` cannot alter one.

**This file is a secret store**, and since M7 it is created ``0600``. Enrollment
tokens, operator tokens and the operators' broker passwords are all recoverable
from it. Robot broker passwords are not: only their hash is kept, so a lost one
is re-issued by re-enrolling rather than looked up.

Two invariants are worth naming because everything else leans on them:

**Allocation is transactional.** ``mote-03`` is derived from the ids already in
the table, so two robots enrolling at the same instant would otherwise both read
``mote-02`` and both claim ``mote-03``. Every enrollment runs inside a
``BEGIN IMMEDIATE`` write transaction, which serialises them.

**Enrollment is idempotent on the fingerprint.** The unique index on
``fingerprint`` is what makes re-enrolling a robot return its existing id rather
than allocate another. Wiping ``~/.mote`` and enrolling again gets the same
robot back; that is the whole point of keying on hardware rather than on disk.

Tokens are the enrollment credential. A single-use token is consumed by the
robot that redeems it, so a card that goes missing between imaging and first
boot can enroll nothing once its robot has booted. Reusable tokens exist for
bench work and are recorded the same way. This is *not* the fleet's
authorization story — see the security note in docs/fleet/control-plane.md.
"""

import json
import os
import secrets
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import credentials  # noqa: E402

#: The fleet box's state root — the server-side analogue of the robot's
#: ``MOTE_HOME``. The registry and the broker's persistence both live here, so a
#: redeploy of the server software (a container image, a git pull) replaces code
#: without touching the fleet's memory of who is in it.
FLEET_HOME_ENV = "MOTE_FLEET_HOME"
FLEET_HOME_DEFAULT = "~/.mote-fleet"

SCHEMA = """
CREATE TABLE IF NOT EXISTS robots (
    robot_id         TEXT PRIMARY KEY,
    name             TEXT NOT NULL DEFAULT '',
    site             TEXT NOT NULL DEFAULT '',
    fingerprint      TEXT NOT NULL UNIQUE,
    facts            TEXT NOT NULL DEFAULT '{}',
    enrolled_at      TEXT NOT NULL,
    last_enrolled_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS tokens (
    token      TEXT PRIMARY KEY,
    single_use INTEGER NOT NULL DEFAULT 1,
    note       TEXT NOT NULL DEFAULT '',
    created_at TEXT NOT NULL,
    used_at    TEXT,
    used_by    TEXT
);
CREATE TABLE IF NOT EXISTS operators (
    token       TEXT PRIMARY KEY,
    name        TEXT NOT NULL,
    note        TEXT NOT NULL DEFAULT '',
    created_at  TEXT NOT NULL,
    last_used_at TEXT,
    revoked_at  TEXT
);
CREATE TABLE IF NOT EXISTS audit (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    stamp      TEXT NOT NULL,
    actor      TEXT NOT NULL,
    action     TEXT NOT NULL,
    robot_id   TEXT NOT NULL DEFAULT '',
    command    TEXT NOT NULL DEFAULT '',
    command_id TEXT NOT NULL DEFAULT '',
    result     TEXT NOT NULL DEFAULT '',
    detail     TEXT NOT NULL DEFAULT '',
    remote     TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS audit_stamp ON audit (stamp DESC);
CREATE TABLE IF NOT EXISTS settings (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);
"""

#: Columns added to already-existing tables after M1. ``CREATE TABLE IF NOT
#: EXISTS`` is a no-op on a table that is already there, so an M1/M3 registry
#: file would silently keep its old shape without these.
MIGRATIONS = (
    ("robots", "broker_hash", "TEXT NOT NULL DEFAULT ''"),
    ("robots", "credential_issued_at", "TEXT"),
    ("operators", "broker_user", "TEXT NOT NULL DEFAULT ''"),
    ("operators", "broker_password", "TEXT NOT NULL DEFAULT ''"),
)

#: The server's own broker password, in ``settings``. Generated on first use and
#: kept, because regenerating it on every restart would lock the server out of
#: its own broker until the next reload.
SERVER_PASSWORD_KEY = "broker_server_password"

ID_PREFIX = "mote"


class RegistryError(Exception):
    """A request the registry refuses (bad token, id already taken)."""


def fleet_home() -> Path:
    return Path(os.environ.get(FLEET_HOME_ENV, FLEET_HOME_DEFAULT)).expanduser()


def default_db() -> str:
    return str(fleet_home() / "registry.db")


def now() -> str:
    return (
        datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
    )


class Registry:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as conn:
            conn.executescript(SCHEMA)
            _migrate(conn)
        # Tokens and operator broker passwords live in here in the clear, so the
        # file is the operator's to read and nobody else's. Applied on every
        # open rather than only at creation, so an M1 file tightens on upgrade.
        os.chmod(self.path, 0o600)

    def _connect(self):
        # isolation_level=None: transactions are opened explicitly with
        # BEGIN IMMEDIATE where allocation correctness needs them, rather than
        # implicitly by the driver where it does not.
        conn = sqlite3.connect(self.path, isolation_level=None, timeout=10.0)
        conn.row_factory = sqlite3.Row
        return conn

    # ---- tokens ---------------------------------------------------------

    def new_token(self, *, single_use: bool = True, note: str = "") -> str:
        token = secrets.token_urlsafe(24)
        with self._connect() as conn:
            conn.execute(
                "INSERT INTO tokens (token, single_use, note, created_at) "
                "VALUES (?, ?, ?, ?)",
                (token, int(single_use), note, now()),
            )
        return token

    def tokens(self) -> list[dict]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT * FROM tokens ORDER BY created_at DESC"
            ).fetchall()
        return [dict(row) for row in rows]

    def _redeem(self, conn, token: str, robot_id: str):
        row = conn.execute("SELECT * FROM tokens WHERE token = ?", (token,)).fetchone()
        if row is None:
            raise RegistryError("unknown enrollment token")
        if row["single_use"] and row["used_at"] and row["used_by"] != robot_id:
            raise RegistryError("enrollment token already used")
        conn.execute(
            "UPDATE tokens SET used_at = ?, used_by = ? WHERE token = ?",
            (now(), robot_id, token),
        )

    # ---- broker credentials ---------------------------------------------

    def server_broker_password(self) -> str:
        """The fleet API's own broker password, generated once and kept.

        Kept rather than regenerated per start because the broker only learns a
        new password when the generated files are reloaded — a server that
        re-minted its own on every restart would spend the window between
        starting and reloading unable to publish a dispatch.
        """
        with self._connect() as conn:
            row = conn.execute(
                "SELECT value FROM settings WHERE key = ?", (SERVER_PASSWORD_KEY,)
            ).fetchone()
            if row is not None:
                return row["value"]
            password = credentials.new_password()
            conn.execute(
                "INSERT OR REPLACE INTO settings (key, value) VALUES (?, ?)",
                (SERVER_PASSWORD_KEY, password),
            )
        return password

    def issue_robot_credential(self, robot_id: str) -> str:
        """Mint this robot's broker password, returning the *only* plaintext
        copy there will ever be. Re-enrolling rotates it, which is the fleet's
        whole robot-credential-rotation story: one command, on the robot."""
        password = credentials.new_password()
        with self._connect() as conn:
            changed = conn.execute(
                "UPDATE robots SET broker_hash = ?, credential_issued_at = ? "
                "WHERE robot_id = ?",
                (credentials.hash_password(password), now(), robot_id),
            ).rowcount
        if not changed:
            raise RegistryError(f"no such robot {robot_id}")
        return password

    def broker_principals(self) -> dict:
        """Everything the generated broker files are built from.

        ``{"users": {username: hash}, "robots": [...], "operators": [...]}`` —
        one query so the password file and the ACL can never describe different
        fleets.
        """
        server_user = credentials.SERVER_USER
        users = {
            server_user: credentials.hash_password(self.server_broker_password()),
        }
        robots, operators = [], []
        with self._connect() as conn:
            for row in conn.execute(
                "SELECT robot_id, broker_hash FROM robots WHERE broker_hash != ''"
            ):
                users[row["robot_id"]] = row["broker_hash"]
                robots.append(row["robot_id"])
            for row in conn.execute(
                "SELECT broker_user, broker_password FROM operators "
                "WHERE revoked_at IS NULL AND broker_user != ''"
            ):
                users[row["broker_user"]] = credentials.hash_password(
                    row["broker_password"]
                )
                operators.append(row["broker_user"])
        return {"users": users, "robots": robots, "operators": operators}

    # ---- operators ------------------------------------------------------

    def new_operator(self, *, name: str, note: str = "") -> str:
        """Mint an operator token, and the subscribe-only broker credential that
        goes with it.

        One act, two credentials, deliberately: an operator needs the HTTP
        bearer token to dispatch *and* a broker login to see the fleet, and
        minting them together is what lets ``revoke`` close both at once. The
        broker password is stored in the clear because the dashboard fetches it
        on every load; the registry file is ``0600`` and that is the boundary.
        """
        if not name.strip():
            raise RegistryError(
                "an operator needs a name — it is what the audit log records"
            )
        token = secrets.token_urlsafe(24)
        with self._connect() as conn:
            conn.execute(
                "INSERT INTO operators (token, name, note, created_at, "
                "broker_user, broker_password) VALUES (?, ?, ?, ?, ?, ?)",
                (
                    token,
                    name.strip(),
                    note,
                    now(),
                    credentials.operator_username(name),
                    credentials.new_password(),
                ),
            )
        return token

    def operators(self) -> list[dict]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT * FROM operators ORDER BY created_at DESC"
            ).fetchall()
        return [dict(row) for row in rows]

    def operator(self, token: str) -> dict | None:
        """The operator this token belongs to, or None if it is unknown or
        revoked. Records the use, so `operator list` shows a stale credential."""
        if not token:
            return None
        with self._connect() as conn:
            row = conn.execute(
                "SELECT * FROM operators WHERE token = ? AND revoked_at IS NULL",
                (token,),
            ).fetchone()
            if row is None:
                return None
            conn.execute(
                "UPDATE operators SET last_used_at = ? WHERE token = ?",
                (now(), token),
            )
        return dict(row)

    def revoke_operator(self, token: str) -> bool:
        with self._connect() as conn:
            changed = conn.execute(
                "UPDATE operators SET revoked_at = ? "
                "WHERE token = ? AND revoked_at IS NULL",
                (now(), token),
            ).rowcount
        return bool(changed)

    # ---- audit ----------------------------------------------------------

    def record(
        self,
        *,
        actor: str,
        action: str,
        robot_id: str = "",
        command: str = "",
        command_id: str = "",
        result: str = "",
        detail: str = "",
        remote: str = "",
    ) -> dict:
        """Append one audit line. Written for refused attempts as well as
        accepted ones: "who tried" is the half of an audit log that a dashboard
        never shows you."""
        row = dict(
            stamp=now(),
            actor=actor,
            action=action,
            robot_id=robot_id,
            command=command,
            command_id=command_id,
            result=result,
            detail=detail,
            remote=remote,
        )
        with self._connect() as conn:
            cursor = conn.execute(
                "INSERT INTO audit (stamp, actor, action, robot_id, command, "
                "command_id, result, detail, remote) "
                "VALUES (:stamp, :actor, :action, :robot_id, :command, "
                ":command_id, :result, :detail, :remote)",
                row,
            )
        return {"id": cursor.lastrowid, **row}

    def finish(self, entry_id: int, result: str, detail: str = "") -> None:
        """Close an audit line with what actually happened. Separate from
        ``record`` so the row exists *before* the side effect it describes."""
        with self._connect() as conn:
            conn.execute(
                "UPDATE audit SET result = ?, detail = ? WHERE id = ?",
                (result, detail, entry_id),
            )

    def audit(self, *, limit: int = 100, robot_id: str = "") -> list[dict]:
        """Most recent first. ``id`` orders it, not ``stamp``: two dispatches in
        the same second are still ordered."""
        query = "SELECT * FROM audit"
        params: list = []
        if robot_id:
            query += " WHERE robot_id = ?"
            params.append(robot_id)
        query += " ORDER BY id DESC LIMIT ?"
        params.append(max(1, min(int(limit), 1000)))
        with self._connect() as conn:
            rows = conn.execute(query, params).fetchall()
        return [dict(row) for row in rows]

    # ---- robots ---------------------------------------------------------

    def robots(self) -> list[dict]:
        with self._connect() as conn:
            rows = conn.execute("SELECT * FROM robots ORDER BY robot_id").fetchall()
        return [_row_to_robot(row) for row in rows]

    def robot(self, robot_id: str) -> dict | None:
        with self._connect() as conn:
            row = conn.execute(
                "SELECT * FROM robots WHERE robot_id = ?", (robot_id,)
            ).fetchone()
        return _row_to_robot(row) if row else None

    def enroll(
        self,
        *,
        token: str,
        fingerprint: str,
        facts: dict | None = None,
        name: str = "",
        site: str = "",
        requested_id: str = "",
        prefix: str = ID_PREFIX,
    ) -> tuple[dict, bool]:
        """Allocate (or return) this machine's row. ``(robot, created)``."""
        facts = facts or {}
        with self._connect() as conn:
            conn.execute("BEGIN IMMEDIATE")
            try:
                existing = conn.execute(
                    "SELECT * FROM robots WHERE fingerprint = ?", (fingerprint,)
                ).fetchone()
                if existing:
                    robot_id = existing["robot_id"]
                    if requested_id and requested_id != robot_id:
                        raise RegistryError(
                            f"this machine is already enrolled as {robot_id}; "
                            f"refusing to re-key it as {requested_id}"
                        )
                    self._redeem(conn, token, robot_id)
                    conn.execute(
                        "UPDATE robots SET name = ?, site = ?, facts = ?, "
                        "last_enrolled_at = ? WHERE robot_id = ?",
                        (
                            name or existing["name"],
                            site or existing["site"],
                            json.dumps(facts, sort_keys=True),
                            now(),
                            robot_id,
                        ),
                    )
                    created = False
                else:
                    robot_id = requested_id or _next_id(conn, prefix)
                    taken = conn.execute(
                        "SELECT 1 FROM robots WHERE robot_id = ?", (robot_id,)
                    ).fetchone()
                    if taken:
                        raise RegistryError(
                            f"robot id {robot_id} is already taken by another machine"
                        )
                    self._redeem(conn, token, robot_id)
                    stamp = now()
                    conn.execute(
                        "INSERT INTO robots (robot_id, name, site, fingerprint, "
                        "facts, enrolled_at, last_enrolled_at) "
                        "VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (
                            robot_id,
                            name or robot_id,
                            site,
                            fingerprint,
                            json.dumps(facts, sort_keys=True),
                            stamp,
                            stamp,
                        ),
                    )
                    created = True
                # The broker credential is issued *inside* the enrollment
                # transaction, so a robot is never a registry row without one —
                # and re-enrolling rotates it, which is the whole rotation
                # story: one idempotent command, run on the robot.
                password = credentials.new_password()
                conn.execute(
                    "UPDATE robots SET broker_hash = ?, credential_issued_at = ? "
                    "WHERE robot_id = ?",
                    (credentials.hash_password(password), now(), robot_id),
                )
                row = conn.execute(
                    "SELECT * FROM robots WHERE robot_id = ?", (robot_id,)
                ).fetchone()
                conn.execute("COMMIT")
            except Exception:
                conn.execute("ROLLBACK")
                raise
        robot = _row_to_robot(row)
        # The only moment this plaintext exists. It is not a column and never
        # appears on `robots()` or `/v1/robots`; the caller hands it to the
        # robot that just enrolled, and after that only its hash is anywhere.
        robot["broker_password"] = password
        return robot, created


def _next_id(conn, prefix: str) -> str:
    """The lowest free ``<prefix>-NN``. Reuses gaps left by a deleted robot."""
    taken = {
        row["robot_id"]
        for row in conn.execute("SELECT robot_id FROM robots").fetchall()
    }
    for number in range(1, 1000):
        candidate = f"{prefix}-{number:02d}"
        if candidate not in taken:
            return candidate
    raise RegistryError(f"no free id left under the prefix {prefix!r}")


def _migrate(conn) -> None:
    """Add any column MIGRATIONS names that this file does not have yet.

    SQLite has no ``ADD COLUMN IF NOT EXISTS``, so the existing columns are read
    back and compared. Every migration is a nullable/defaulted add, which is the
    only shape that can be applied to a live registry without a rewrite.
    """
    for table, column, decl in MIGRATIONS:
        present = {
            row["name"]
            for row in conn.execute(f"PRAGMA table_info({table})").fetchall()
        }
        if column not in present:
            conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {decl}")


def _row_to_robot(row) -> dict:
    robot = dict(row)
    robot["facts"] = json.loads(robot.get("facts") or "{}")
    # A password hash is not roster data. Dropped here rather than in the API so
    # that no future route can serve it by forgetting to.
    robot.pop("broker_hash", None)
    return robot

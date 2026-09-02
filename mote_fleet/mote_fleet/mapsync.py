"""The robot's side of the map registry: publish a revision, pull the canonical one.

Two directions, both deliberately small, because the interesting invariants
already live elsewhere. Uploading is "pack the revision ``save-map`` wrote and
POST it"; the fleet server decides whether it is any good and keeps it as a
*candidate* either way (:mod:`bundle_store`). Pulling is "fetch the announced
revision and hand the bytes to :func:`mote_bringup.sites.install_revision`",
which stages and flips atomically. Nothing here has to be careful about partial
state, because neither end lets partial state exist.

Kept ROS-free so the whole flow can be exercised without a graph: the agent
(:mod:`mote_fleet.agent`) calls into it from a worker thread, and
``pixi run publish-map`` calls the same functions from a terminal.

The one judgement call that lives here is :func:`wants` — *which* floors this
robot acts on. The registry announces every floor in the fleet, and a robot has
no business downloading maps of buildings it will never be in, so it takes the
floor it is on plus any floor it already holds a bundle for. A robot that is
moved to a new floor picks that floor up when it becomes active.
"""

import hashlib
import json
import urllib.error
import urllib.parse
import urllib.request

from mote_bringup import bundle, sites

TIMEOUT = 120.0

#: Refuse a download that is larger than a map revision could plausibly be,
#: before reading it into memory.
MAX_BUNDLE = 64 * 1024 * 1024


class SyncError(Exception):
    """A registry exchange that did not happen. The message is for a log."""


def _url(server: str, path: str) -> str:
    return server.rstrip("/") + path


def fetch(server: str, path: str, *, timeout: float = TIMEOUT) -> bytes:
    """Download a packed revision. Raises :class:`SyncError` with the reason."""
    url = _url(server, path)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            declared = int(response.headers.get("Content-Length") or 0)
            if declared > MAX_BUNDLE:
                raise SyncError(f"{url}: bundle is {declared} bytes, refusing")
            blob = response.read(MAX_BUNDLE + 1)
    except urllib.error.HTTPError as exc:
        raise SyncError(f"{url}: {exc.code} {exc.reason}") from exc
    except (urllib.error.URLError, OSError) as exc:
        raise SyncError(f"{url}: {exc}") from exc
    if len(blob) > MAX_BUNDLE:
        raise SyncError(f"{url}: bundle is larger than {MAX_BUNDLE} bytes")
    return blob


def check_digest(blob: bytes, expected: str):
    """Refuse bytes that are not the ones that were announced.

    Not a security boundary — the tailnet is that until M7 — but a transfer
    that silently truncated would otherwise become a map, and a wrong map is
    worse than no map.
    """
    if not expected:
        return
    actual = "sha256:" + hashlib.sha256(blob).hexdigest()
    if actual != expected:
        raise SyncError(f"downloaded bundle is {actual}, announced as {expected}")


def wants(announcement: dict, active: tuple | None) -> bool:
    """Should this robot act on a floor's announcement?

    Yes for the floor it is on — that is the map it navigates with — and yes
    for any floor it already has a bundle for, so a robot that moves between
    two floors keeps both current instead of re-downloading on arrival. No for
    everything else: the registry is fleet-wide, this robot is not.
    """
    site = announcement.get("site")
    floor = announcement.get("floor")
    if not site or not floor:
        return False
    if active and (site, floor) == tuple(active):
        return True
    return sites.floor_dir(site, floor).is_dir()


def pull(server: str, announcement: dict, *, timeout: float = TIMEOUT) -> dict:
    """Bring this robot's copy of a floor up to the announced revision.

    Returns ``{"action": ..., "revision": ...}`` where the action is one of
    ``current`` (already running it), ``flipped`` (had the revision, published
    it) or ``installed``.
    """
    site = announcement["site"]
    floor = announcement["floor"]
    revision = announcement["revision"]
    fdir = sites.floor_dir(site, floor)
    if fdir.is_dir() and sites.current_revision(fdir) == revision:
        return {"action": "current", "site": site, "floor": floor, "revision": revision}
    blob = b""
    if not (fdir / "maps" / revision).is_dir():
        blob = fetch(server, announcement["url"], timeout=timeout)
        check_digest(blob, announcement.get("sha256", ""))
    try:
        action = sites.install_revision(site, floor, revision, blob)
    except bundle.BundleError as exc:
        raise SyncError(str(exc)) from exc
    except OSError as exc:
        raise SyncError(f"could not install {site}/{floor}/{revision}: {exc}") from exc
    return {"action": action, "site": site, "floor": floor, "revision": revision}


def publish(
    server: str,
    site: str,
    floor: str,
    revision: str,
    robot_id: str,
    *,
    timeout: float = TIMEOUT,
) -> dict:
    """Upload one local revision as a candidate. Returns the server's answer.

    The floor's zones are packed *into* the revision. The floor owns them — a
    zone is a coordinate in the floor's frame and a map revision is an estimate
    registered into it — but a revision is the vehicle the fleet already has for
    getting a floor's places to a robot that has never driven there, so a
    revision carries a copy.
    """
    fdir = sites.floor_dir(site, floor)
    rev_dir = fdir / "maps" / revision
    if not rev_dir.is_dir():
        raise SyncError(f"no revision {revision} in {site}/{floor}")
    report = bundle.validate(rev_dir)
    if not report.ok:
        raise SyncError(f"refusing to publish {revision}: {report.summary()}")
    extra = {}
    if not (rev_dir / bundle.ZONES_YAML).is_file():
        if (fdir / bundle.ZONES_YAML).is_file():
            extra[bundle.ZONES_YAML] = (fdir / bundle.ZONES_YAML).read_bytes()
        else:
            # A floor still holding zone/v0's two documents: both travel, and
            # the receiver joins them the same way this robot does.
            for name in (bundle.VOCABULARY_YAML, bundle.BINDING_YAML):
                if (fdir / name).is_file():
                    extra[name] = (fdir / name).read_bytes()
    blob = bundle.pack(rev_dir, extra)

    path = (
        f"/v1/sites/{urllib.parse.quote(site)}/floors/{urllib.parse.quote(floor)}"
        f"/revisions/{urllib.parse.quote(revision)}"
        f"?robot_id={urllib.parse.quote(robot_id)}"
    )
    request = urllib.request.Request(
        _url(server, path),
        data=blob,
        headers={"Content-Type": "application/gzip"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            answer = json.loads(response.read())
    except urllib.error.HTTPError as exc:
        detail = ""
        try:
            body = json.loads(exc.read())
            detail = body.get("error", "")
            if body.get("errors"):
                detail += " — " + "; ".join(body["errors"])
        except (ValueError, OSError):
            pass
        raise SyncError(
            f"{exc.code} {exc.reason}{f': {detail}' if detail else ''}"
        ) from exc
    except (urllib.error.URLError, OSError) as exc:
        raise SyncError(f"{server}: {exc}") from exc
    answer["bytes"] = len(blob)
    return answer

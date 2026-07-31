"""The map registry's byte store: candidate revisions, and the canonical one.

The fleet server is the source of truth for sites, floors and map revisions
(fleet.md Q4), and this is where those bytes live. It is deliberately **the same
on-disk layout ``sites.py`` writes on a robot** — ``<site>/floors/<floor>/``
with immutable ``maps/<rev>/`` directories and a ``map`` symlink at the
published one — so distribution is "copy a revision directory, flip a link"
rather than a format conversion at each end, and so M3's basemap routes keep
reading exactly what they already read.

Three properties are the whole design:

**Uploading is not publishing.** A revision that arrives is a *candidate*: it is
stored, validated, recorded, and changes nothing. Only an operator's promote
flips the symlink and announces the floor's new canonical revision. That is what
makes "two robots mapped the same floor" a non-event — both candidates are kept,
neither is merged, and a human picks (fleet.md Q4: a map frame's origin is an
accident of where SLAM started, so silently merging two frames breaks every
taught zone coordinate).

**The filesystem is the truth about what is canonical.** The symlink *is* the
answer, not a row that describes one, and it is flipped by an atomic
``os.replace`` — so a reader never sees a half-published floor, and a rollback
is flipping it back. The registry database records who promoted what and when
(the audit log); it never gets to disagree about which map is live.

**Validation runs on the way in, on the server's own terms.** The robot already
refused to save an incomplete revision, but an upload can truncate where a local
save could not, so :mod:`mote_bringup.bundle` — the same module the robot uses —
re-checks it here. A candidate that fails is refused with the reasons, not
stored and quietly ignored.
"""

import json
import os
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

# The shared bundle module lives in the package that owns the bundle layout,
# and this server imports it two ways: from the sibling package directory in a
# checkout, and from beside ``mote_fleet`` in the deploy image, where only the
# two ROS-free files are copied. Neither needs ROS on the box — that is the
# whole point of ``bundle`` being ROS-free (fleet.md Q4). Its only third-party
# import is PyYAML, which the deploy image installs beside paho.
for _candidate in (
    Path(__file__).resolve().parents[1],
    Path(__file__).resolve().parents[2] / "mote_bringup",
):
    if (_candidate / "mote_bringup").is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from mote_bringup import bundle  # noqa: E402

#: Candidates kept per floor, on top of whatever is canonical. Enough to see a
#: mapping session's history and to roll back through it; not so many that a
#: robot that publishes on a loop fills the box. The canonical revision is
#: never pruned, however old it is.
KEEP_REVISIONS = 5

#: Provenance the registry keeps *about* a revision, beside it but never inside
#: the bundle: who uploaded it, when, and what validation said at the time.
UPLOAD_JSON = "upload.json"

#: Ceiling on one uploaded bundle. A full floor's revision is a few hundred KB
#: of PNG plus a posegraph that is usually a few MB.
MAX_UPLOAD = 64 * 1024 * 1024


class StoreError(Exception):
    """A registry request that is refused. ``code`` is the HTTP status."""

    def __init__(self, message: str, code: int = 400, detail=None):
        super().__init__(message)
        self.code = code
        self.detail = detail or {}


class BundleStore:
    """Site bundles on the fleet box, rooted at ``--maps-dir``."""

    def __init__(self, root):
        self.root = Path(root).expanduser() if root else None

    # -- reading ----------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self.root is not None

    def floor_dir(self, site: str, floor: str) -> Path:
        if not self.root:
            raise StoreError("this server stores no site bundles", 404)
        return self.root / site / "floors" / floor

    def sites(self) -> list[dict]:
        """Every floor this server knows, with its canonical revision.

        Walks the bundle layout rather than a table, because the layout is the
        record: a floor rsynced onto the box before M4 (fleet-api.md) shows up
        here with no upload history and works.
        """
        if not self.root or not self.root.is_dir():
            return []
        found = []
        for site_dir in sorted(self.root.iterdir()):
            floors_dir = site_dir / "floors"
            if not floors_dir.is_dir():
                continue
            for floor_dir in sorted(floors_dir.iterdir()):
                if not floor_dir.is_dir():
                    continue
                canonical = self.canonical(site_dir.name, floor_dir.name)
                revisions = self.revisions(site_dir.name, floor_dir.name)
                if not canonical and not revisions:
                    continue
                found.append(
                    {
                        "site": site_dir.name,
                        "floor": floor_dir.name,
                        "canonical": canonical,
                        "candidates": [r for r in revisions if r != canonical],
                        "revisions": revisions,
                    }
                )
        return found

    def revisions(self, site: str, floor: str) -> list[str]:
        """Every stored revision, newest last.

        Dot-directories are skipped because an upload in flight stages inside
        ``maps/`` (``accept``), and a staging directory is not a revision until
        it is renamed into place: listing one shows a reader half a bundle, and
        pruning one deletes a *concurrent* upload's work.
        """
        maps = self.floor_dir(site, floor) / "maps"
        if not maps.is_dir():
            return []
        return sorted(
            p.name for p in maps.iterdir() if p.is_dir() and not p.name.startswith(".")
        )

    def canonical(self, site: str, floor: str) -> str | None:
        """The published revision — read from the symlink, which *is* the fact."""
        link = self.floor_dir(site, floor) / "map"
        if link.is_symlink():
            return Path(os.readlink(link)).name
        # A floor seeded by rsync may have a real directory rather than a link.
        # It is still the published map; it simply cannot be rolled back until
        # a revision is promoted over it.
        return "map" if link.is_dir() else None

    def revision_dir(self, site: str, floor: str, revision: str) -> Path:
        floor_dir = self.floor_dir(site, floor)
        target = (
            floor_dir / "map" if revision == "map" else floor_dir / "maps" / revision
        )
        if not target.is_dir():
            raise StoreError(f"no revision {revision} on {site}/{floor}", 404)
        return target

    def detail(self, site: str, floor: str) -> dict:
        """One floor: its canonical revision, and every candidate with why it
        is or is not fit to be promoted."""
        if not self.floor_dir(site, floor).is_dir():
            raise StoreError(f"no floor {site}/{floor}", 404)
        canonical = self.canonical(site, floor)
        revisions = []
        for revision in self.revisions(site, floor):
            revisions.append(self.describe(site, floor, revision, canonical))
        return {
            "site": site,
            "floor": floor,
            "canonical": canonical,
            "revisions": revisions,
        }

    def describe(
        self, site: str, floor: str, revision: str, canonical: str | None = None
    ) -> dict:
        directory = self.revision_dir(site, floor, revision)
        upload = {}
        upload_file = directory / UPLOAD_JSON
        if upload_file.is_file():
            try:
                upload = json.loads(upload_file.read_text())
            except ValueError:
                upload = {}
        # The report recorded at upload is reused while the revision is
        # untouched, because this runs for every revision of a floor on every
        # dashboard floor-switch. `mtime` on the directory changes if anything
        # is added, removed or renamed inside it, which is what a restore or a
        # half-copied backup looks like — that re-validates. `promote` always
        # re-validates regardless, so the claim that gates publishing is never
        # a cached one.
        report = self._report(directory, upload)
        return {
            "revision": revision,
            "canonical": revision == (canonical or self.canonical(site, floor)),
            "uploaded_at": upload.get("uploaded_at"),
            "uploaded_by": upload.get("uploaded_by"),
            "robot_id": upload.get("robot_id"),
            "bytes": upload.get("bytes"),
            "sha256": upload.get("sha256"),
            "url": self.bundle_url(site, floor, revision),
            **report,
        }

    def _report(self, directory: Path, upload: dict) -> dict:
        """The stored validation payload if it still describes what is on disk,
        else a fresh one (which is then stored).

        A payload rather than a :class:`bundle.Report`, because ``as_dict`` is
        lossy — it reduces zones to their names — so a Report rebuilt from one
        would quietly differ from a validated one.
        """
        stored = upload.get("report")
        if stored and upload.get("validated_mtime") == _mtime(directory):
            return stored
        report = bundle.validate(directory, require_posegraph=False)
        upload_file = directory / UPLOAD_JSON
        if upload_file.is_file():
            upload["report"] = report.as_dict()
            upload["validated_mtime"] = _mtime(directory)
            try:
                upload_file.write_text(json.dumps(upload, indent=2))
            except OSError:
                pass  # a read-only store still answers, just without the cache
        return report.as_dict()

    def bundle_url(self, site: str, floor: str, revision: str) -> str:
        return f"/v1/sites/{site}/floors/{floor}/revisions/{revision}/bundle.tar.gz"

    def read_map(self, site: str, floor: str, revision: str | None = None) -> dict:
        """A floor's map metadata for the world→pixel transform (Q5).

        Reads the canonical revision unless a revision is named. The parsing is
        :mod:`mote_bringup.bundle`, i.e. the same code that validated the
        revision on upload and the same the robot writes against — M3 shipped a
        hand-rolled reader here with a note that M4 would replace it, and this
        is that.
        """
        directory = self.revision_dir(site, floor, revision or self._live(site, floor))
        meta = bundle.read_map(directory / bundle.MAP_YAML)
        image = directory / meta["image"]
        if not image.is_file():
            raise StoreError(f"{site}/{floor}: the map image is missing", 500)
        size = bundle.png_size(image)
        if size is None:
            raise StoreError(
                f"{site}/{floor}: the map image is not a readable PNG", 500
            )
        meta["width"], meta["height"] = size
        meta.update(
            site=site,
            floor=floor,
            revision=self.canonical(site, floor) if revision is None else revision,
            image_url=f"/v1/maps/{site}/{floor}/map.png",
        )
        meta["_image_path"] = str(image)
        return meta

    def read_zones(self, site: str, floor: str) -> dict:
        """The floor's taught zones, in the map frame the basemap is drawn in.

        This is the **binding**: coordinates, and therefore meaningful only
        against the map they were taught on. It is served beside the basemap,
        to the one client that also has the basemap, and it is not what a
        dispatcher gets — see :meth:`read_vocabulary`.

        Zones travel inside a published revision, because a zone is a
        coordinate in one SLAM session's frame. A floor seeded by rsync keeps
        them at floor level, as ``sites.py`` writes them, so that is the
        fallback rather than an error.
        """
        # A binding without the map it is bound to is the coordinate-shaped
        # thing this whole split exists to stop, so it stays gated on there
        # being a published revision.
        path = self._zones_file(site, floor, revision=self._live(site, floor))
        if path is None:
            raise StoreError(f"no zones for {site}/{floor}", 404)
        zones = bundle.read_zones(path)
        return {
            "site": site,
            "floor": floor,
            "frame_id": zones["frame_id"],
            "zones": list(zones["zones"].values()),
        }

    def read_vocabulary(self, site: str, floor: str) -> dict:
        """The floor's zone **vocabulary** — names, kinds and aliases, no
        coordinates and no frame.

        Deliberately *not* gated on a published map, where
        :meth:`read_zones` is. A vocabulary is a fact about the building, so a
        floor that has been named but never mapped still has one, and a robot
        arriving at a site can be told what the places are called before it has
        driven a metre. That is the portability the split buys, and gating this
        on a promoted revision would have quietly given it back.
        """
        path = self._zones_file(site, floor, revision=self.canonical(site, floor))
        if path is None:
            raise StoreError(f"no zones for {site}/{floor}", 404)
        return bundle.vocabulary(bundle.read_zones(path), site, floor)

    def vocabularies(self) -> list:
        """Every floor's vocabulary, for a dispatcher bootstrapping a whole
        fleet in one call. A floor with no zones yet is skipped, not an error.

        Walks the floors itself rather than reusing :meth:`sites`, which lists
        floors that have a *map*. The two sets are not the same one, and the
        difference is the interesting case: a floor someone has named but not
        yet mapped belongs here.
        """
        found = []
        for site, floor in self._floors():
            try:
                found.append(self.read_vocabulary(site, floor))
            except (StoreError, bundle.BundleError):
                continue
        return found

    def _floors(self):
        """Every ``(site, floor)`` in the layout, which is the record."""
        if not self.root or not self.root.is_dir():
            return
        for site_dir in sorted(self.root.iterdir()):
            floors_dir = site_dir / "floors"
            if not floors_dir.is_dir():
                continue
            for floor_dir in sorted(floors_dir.iterdir()):
                if floor_dir.is_dir():
                    yield site_dir.name, floor_dir.name

    def _zones_file(self, site: str, floor: str, revision: str = ""):
        """``zones.yaml`` from the given revision, else the floor-level one."""
        candidates = []
        if revision:
            candidates.append(self.revision_dir(site, floor, revision))
        candidates.append(self.floor_dir(site, floor))
        for directory in candidates:
            path = directory / bundle.ZONES_YAML
            if path.is_file():
                return path
        return None

    def _live(self, site: str, floor: str) -> str:
        canonical = self.canonical(site, floor)
        if not canonical:
            raise StoreError(f"no published map for {site}/{floor}", 404)
        return canonical

    def pack(self, site: str, floor: str, revision: str) -> bytes:
        """A revision as the bytes a robot pulls. Re-packed from the stored
        files, which is byte-identical to what was uploaded because
        :func:`bundle.pack` is deterministic — so the digest announced on the
        retained topic keeps matching without keeping the upload around."""
        return bundle.pack(self.revision_dir(site, floor, revision))

    # -- writing ----------------------------------------------------------

    def accept(
        self,
        site: str,
        floor: str,
        revision: str,
        blob: bytes,
        *,
        robot_id: str = "",
        uploaded_by: str = "",
    ) -> tuple[str, bundle.Report]:
        """Store an uploaded revision as a **candidate**. Returns ``(id, report)``.

        Nothing about the floor changes here — not the symlink, not what any
        robot is running. That separation is the milestone's conflict story: a
        second robot's map of the same floor is kept beside the first, and an
        operator promotes one.
        """
        if not self.root:
            raise StoreError("this server stores no site bundles", 404)
        if len(blob) > MAX_UPLOAD:
            raise StoreError(f"bundle is larger than {MAX_UPLOAD} bytes", 413)
        floor_dir = self.floor_dir(site, floor)
        maps = floor_dir / "maps"
        maps.mkdir(parents=True, exist_ok=True)

        staging = Path(tempfile.mkdtemp(prefix=f".{revision}-", dir=maps))
        try:
            try:
                bundle.unpack(blob, staging)
            except bundle.BundleError as exc:
                raise StoreError(str(exc), 400) from exc
            try:
                report = bundle.validate(staging)
            except bundle.BundleError as exc:
                # validate() documents that it reports rather than raises, and
                # it is tested that way. Belt and braces: a validator that
                # breaks that promise on some input nobody thought of should
                # still be a 422 about the bundle, not a dropped socket.
                raise StoreError(
                    f"the bundle could not be validated: {exc}", 422
                ) from exc
            if not report.ok:
                raise StoreError(
                    f"the bundle is not a usable map revision: {report.summary()}",
                    422,
                    {"errors": report.errors, "warnings": report.warnings},
                )
            stored = self._free_revision(maps, revision, blob)
            if stored is None:
                # Byte-identical to what is already here: the upload is a
                # retry, and a retry must not mint a second candidate.
                shutil.rmtree(staging, ignore_errors=True)
                return revision, report
            (staging / UPLOAD_JSON).write_text(
                json.dumps(
                    {
                        "revision": stored,
                        "proposed": revision,
                        "uploaded_at": _now(),
                        "uploaded_by": uploaded_by,
                        "robot_id": robot_id,
                        "bytes": len(blob),
                        "sha256": bundle.digest(blob),
                        "report": report.as_dict(),
                    },
                    indent=2,
                )
            )
            os.replace(staging, maps / stored)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        report.revision = stored
        self.prune(site, floor)
        return stored, report

    def _free_revision(self, maps: Path, revision: str, blob: bytes) -> str | None:
        """Where this upload should land: its own id if free, a qualified one if
        another robot already used it, or None if it is byte-for-byte what is
        already stored there.

        A revision directory is immutable once published, so an id that is
        taken is never overwritten. Ids are per-second timestamps, so a
        collision means two robots mapping the same floor in the same second —
        rare, and the qualified id keeps both rather than losing one.
        """
        candidate = maps / revision
        if not candidate.exists():
            return revision
        try:
            if bundle.pack(candidate) == blob:
                return None
        except (OSError, bundle.BundleError):
            pass
        for suffix in range(2, 100):
            qualified = f"{revision}-{suffix}"
            if not (maps / qualified).exists():
                return qualified
        raise StoreError(f"too many revisions named {revision}", 409)

    def promote(self, site: str, floor: str, revision: str, *, by: str = "") -> dict:
        """Make a candidate the floor's canonical map: validate, then flip.

        The flip is ``sites.py``'s: a symlink written under a temporary name and
        moved into place with ``os.replace``, so there is no instant at which
        the floor has no map. Rolling back is this same call with an older
        revision, which is ``site use-map`` with the fleet in the loop.
        """
        directory = self.revision_dir(site, floor, revision)
        report = bundle.validate(directory, require_posegraph=False)
        if not report.ok:
            raise StoreError(
                f"revision {revision} cannot be promoted: {report.summary()}",
                422,
                {"errors": report.errors, "warnings": report.warnings},
            )
        floor_dir = self.floor_dir(site, floor)
        link = floor_dir / "map"
        if link.exists() and not link.is_symlink():
            raise StoreError(
                f"{site}/{floor}/map is a directory, not a published revision — "
                "move it into maps/<rev>/ before promoting",
                409,
            )
        temporary = floor_dir / f".map-{os.getpid()}"
        if temporary.is_symlink() or temporary.exists():
            temporary.unlink()
        os.symlink(os.path.join("maps", revision), temporary)
        os.replace(temporary, link)
        blob = self.pack(site, floor, revision)
        self.prune(site, floor)
        return {
            "site": site,
            "floor": floor,
            "revision": revision,
            "url": self.bundle_url(site, floor, revision),
            "sha256": bundle.digest(blob),
            "bytes": len(blob),
            "promoted_by": by,
            "warnings": report.warnings,
        }

    def prune(self, site: str, floor: str):
        """Keep the canonical revision and the newest candidates."""
        canonical = self.canonical(site, floor)
        revisions = self.revisions(site, floor)
        keep = set(revisions[-KEEP_REVISIONS:]) | {canonical}
        for revision in revisions:
            if revision not in keep:
                shutil.rmtree(self.floor_dir(site, floor) / "maps" / revision, True)


def _mtime(directory: Path) -> int:
    """Whole nanoseconds, so the value round-trips through JSON exactly."""
    try:
        return directory.stat().st_mtime_ns
    except OSError:
        return 0


def _now() -> str:
    return (
        datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
    )

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
bound zone coordinate).

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

import yaml
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

    def revision_url(self, site: str, floor: str, revision: str, leaf: str) -> str:
        return f"/v1/sites/{site}/floors/{floor}/revisions/{revision}/{leaf}"

    def bundle_url(self, site: str, floor: str, revision: str) -> str:
        return self.revision_url(site, floor, revision, "bundle.tar.gz")

    def read_map(self, site: str, floor: str, revision: str | None = None) -> dict:
        """A floor's map metadata for the world→pixel transform (Q5).

        Reads the canonical revision unless a revision is named. The parsing is
        :mod:`mote_bringup.bundle`, i.e. the same code that validated the
        revision on upload and the same the robot writes against — M3 shipped a
        hand-rolled reader here with a note that M4 would replace it, and this
        is that.

        ``image_url`` names the route the caller must fetch to get *these*
        pixels, which is not the same route in both cases: the canonical map is
        served under ``/v1/maps`` and a named revision under its own path. A
        payload describing a candidate's transform while pointing at the
        canonical image is the one failure the review view exists to remove —
        the operator would be shown the map they already have, labelled as the
        one they are about to promote.
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
            image_url=(
                f"/v1/maps/{site}/{floor}/map.png"
                if revision is None
                else self.revision_url(site, floor, revision, "map.png")
            ),
        )
        meta["_image_path"] = str(image)
        return meta

    def read_zones(self, site: str, floor: str) -> dict:
        """The floor's zones with their coordinates, for a client drawing them.

        Served beside the basemap, to the one client that also has the basemap,
        and gated on there being a published map for the same reason: what this
        route is for is drawing zones *on* one, and a client with nothing to
        draw them on has asked the wrong question. ``/v1/zones`` is the one that
        answers without a map — see :meth:`read_vocabulary`.

        A revision carries a copy of the floor's zones and a floor seeded by
        rsync keeps them at floor level, as ``sites.py`` writes them, so that is
        the fallback rather than an error.
        """
        path = self._zones_file(site, floor, revision=self._live(site, floor))
        if path is None:
            raise StoreError(f"no zones for {site}/{floor}", 404)
        zones = self._read(path, site, floor)
        return {
            "site": site,
            "floor": floor,
            "frame_id": zones["frame_id"],
            "zones": list(zones["zones"].values()),
        }

    def read_revision_zones(self, site: str, floor: str, revision: str) -> dict:
        """The zones **one revision carries**, for the operator reviewing it.

        Served the same way :meth:`read_zones` is — under the revision's own
        path, beside the revision's own basemap, never over ``/v1/zones``. What
        it drops is that method's gate on there being a published map, which is
        the one thing that would make it useless here: the review that matters
        most is the *first* candidate on a floor with nothing published at all.
        """
        # An unknown revision is a 404 about the revision rather than about the
        # zones, and the floor-level fallback below must not answer for one.
        directory = self.revision_dir(site, floor, revision)
        path = self._zones_file(site, floor, revision=revision)
        if path is None:
            raise StoreError(f"no zones for {site}/{floor}/{revision}", 404)
        zones = self._read(path, site, floor)
        return {
            "site": site,
            "floor": floor,
            "revision": revision,
            # Which of ``_zones_file``'s two candidates answered. A revision
            # carrying no zones of its own falls back to the floor's, and an
            # operator reviewing a candidate is entitled to know that what is
            # drawn came from beside it rather than from inside it.
            "source": "revision" if path == directory else "floor",
            "frame_id": zones["frame_id"],
            "zones": list(zones["zones"].values()),
        }

    def read_vocabulary(self, site: str, floor: str) -> dict:
        """The names-only view of the same floor: what each place is called and
        a note about it, no coordinates and no frame.

        The same read as :meth:`read_zones` and a different *view* of it —
        ``bundle.vocabulary`` builds the payload from the fields a vocabulary
        may carry rather than stripping the ones it may not, which is what keeps
        a coordinate out of it when someone adds a key.

        Deliberately *not* gated on a published map, where :meth:`read_zones`
        is. What places a building has is a fact about the building, so a floor
        that has been named but never mapped still answers, and a robot arriving
        at a site can be told what the places are called before it has driven a
        metre.
        """
        # The canonical revision's copy where there is one, exactly as
        # `read_zones` reads it: promotion is what publishes an edit, so the
        # copy inside the revision an operator promoted is the fleet's current
        # answer and the floor's own file is the fallback.
        path = self._zones_file(site, floor, revision=self.canonical(site, floor))
        if path is None:
            raise StoreError(f"no zones for {site}/{floor}", 404)
        return bundle.vocabulary(self._read(path, site, floor), site, floor)

    def derive_zones(
        self, site: str, floor: str, zones: dict, *, by: str, source: str = ""
    ):
        """A new candidate revision: one revision's map bytes, edited zones.

        An operator's zone edit never touches a stored revision — a promoted
        revision's bytes back a digest the fleet has been told, and a candidate
        is immutable for the same reason one id is never reused. So an edit is
        a *derivation*: pack the source revision with the submitted zones in
        place of its own, and accept the result as an ordinary candidate —
        validated by the same code as any upload, inert until promoted.

        ``source`` is the revision being edited, defaulting to the canonical
        one. Naming a candidate is what makes an unpromoted map editable: the
        first build of a floor arrives with `zone_01`..`zone_07` from
        `segment-map`, and defaulting to the canonical would have meant
        promoting placeholder names in order to be allowed to fix them —
        publishing a map *because* it was wrong. It also derives from what the
        operator was looking at: the review pane draws the candidate's own map,
        and deriving from a different revision's zones would save an edit
        nobody made.
        Returns ``(stored_revision, report, derived_from)``.
        """
        source = source or self.canonical(site, floor)
        if not source:
            raise StoreError(
                f"{site}/{floor} has no published map to edit zones on", 409
            )
        rev_dir = self.revision_dir(site, floor, source)
        previous = {}
        # The zones the operator was shown, which is what the edit is a delta
        # of: `_zones_file` falls back to the floor's file for a revision
        # carrying none, and so does the review pane that fed the editor. The
        # `frame_id` and `revision` come from there for the same reason — an
        # edit that silently reset the revision would make a later
        # carry-forward unable to tell which of two copies is newer.
        zones_file = self._zones_file(site, floor, revision=source)
        if zones_file is not None:
            try:
                previous = self._read(zones_file, site, floor)
            except bundle.BundleError:
                previous = {}
        # The submitted entries may echo their key as a `name` field (the
        # zones.json shape); the file format keys by name and carries no copy.
        cleaned = {}
        for name, entry in zones.items():
            entry = dict(entry)
            entry.pop("name", None)
            cleaned[name] = entry
        payload = {
            "frame_id": previous.get("frame_id") or "map",
            "revision": int(previous.get("revision") or 0) + 1,
            "zones": cleaned,
        }
        blob = bundle.pack(
            rev_dir,
            {bundle.ZONES_YAML: yaml.safe_dump(payload, sort_keys=False).encode()},
        )
        stored, report = self.accept(
            site,
            floor,
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S"),
            blob,
            uploaded_by=by,
            require_posegraph=False,
        )
        return stored, report, source

    def vocabularies(self) -> list:
        """Every floor's names, for a dispatcher bootstrapping a whole fleet in
        one call. A floor with no zones yet is skipped, not an error.

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
        """The directory holding the zones of the given revision, else the
        floor's own. A directory rather than a file, because that is what
        ``bundle.read_floor`` takes."""
        candidates = []
        if revision:
            candidates.append(self.revision_dir(site, floor, revision))
        candidates.append(self.floor_dir(site, floor))
        for directory in candidates:
            if (directory / bundle.ZONES_YAML).is_file():
                return directory
        return None

    def _read(self, directory, site: str, floor: str) -> dict:
        """One floor directory's zones, whichever layout is on disk."""
        return bundle.read_floor(directory, site, floor)

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
        require_posegraph: bool = True,
    ) -> tuple[str, bundle.Report]:
        """Store an uploaded revision as a **candidate**. Returns ``(id, report)``.

        Nothing about the floor changes here — not the symlink, not what any
        robot is running. That separation is the milestone's conflict story: a
        second robot's map of the same floor is kept beside the first, and an
        operator promotes one.

        ``require_posegraph`` is what a *robot's upload* is held to and what a
        derivation is not: a mapping session that produced no posegraph produced
        a map nothing can extend, and that is worth refusing at the source. A
        revision already in the registry has been judged by the looser rule
        :meth:`promote` uses, so a derivation of one must not be held to a
        stricter bar than the revision it came from — the review pane calls such
        a revision promotable, and an `edit zones` button beside that verdict
        that could only ever fail is worse than no button (observed: the sim's
        own bundles carry no posegraph).
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
                report = bundle.validate(staging, require_posegraph=require_posegraph)
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

"""Mount the markdown that lives beside the code, and keep repo links working.

A package README is read in a checkout and on GitHub at least as often as on
this site, so it stays with its package and is copied in at build time. The
copy's relative links still point at the repo, which the site does not carry,
so every link is resolved against the file's real home and then rewritten:

* to another site page, where the target is one (a mounted file, or anything
  already under ``docs/``);
* to a GitHub blob/tree URL, where it is source the site does not serve.

The same rewriting runs over the pages that do live in ``docs/`` — they link
out to source files just as freely.

A link the rewriter cannot resolve is left exactly as written, so it surfaces
as a MkDocs warning (and fails ``--strict``) rather than being papered over.
"""

import posixpath
import re
from pathlib import Path

from mkdocs.structure.files import File
from pymdownx.slugs import slugify

REPO = "https://github.com/ClachDev/Mote"
BRANCH = "main"

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"

# site page -> the file in the repo it is a copy of.
MOUNTS = {
    "hardware/index.md": "design/README.md",
    "hardware/bom.md": "design/BOM.md",
    "hardware/assembly.md": "design/ASSEMBLY.md",
    "hardware/wiring.md": "design/WIRING.md",
    "hardware/servo-tools.md": "mote_hardware/tools/README.md",
    "hardware/research/caster.md": "design/research/caster.md",
    "hardware/research/imu-fusion.md": "design/research/imu_fusion_study.md",
    "hardware/research/lidar-odometry.md": "design/research/lidar_odometry.md",
    "hardware/research/sfm-depth.md": "design/research/sfm_multiview_depth.md",
    "hardware/research/sfm-stage0.md": "design/research/sfm_stage0_results.md",
    "robot/bringup.md": "mote_bringup/README.md",
    "robot/health.md": "mote_health/README.md",
    "robot/missions.md": "mote_tasks/README.md",
    "robot/map-cleanup.md": "mote_bringup/mote_bringup/map_cleanup/README.md",
    "robot/foxglove.md": "mote_bringup/foxglove/README.md",
    "robot/chaos.md": "mote_bringup/test/chaos/README.md",
    "perception/index.md": "mote_perception/README.md",
    "perception/camera-calibration.md": "mote_perception/config/README.md",
    "perception/benchmarks.md": "mote_perception/benchmarks/README.md",
    "arm/index.md": "mote_arm/README.md",
    "arm/teleop.md": "mote_arm/TELEOP.md",
    "arm/bench.md": "mote_arm/BENCH.md",
    "fleet/package.md": "mote_fleet/README.md",
    "simulation/benchmark.md": "mote_simulation/tools/benchmark/README.md",
    "simulation/sweep.md": "mote_simulation/tools/benchmark/sweep/README.md",
    "simulation/bag-replay.md": "mote_simulation/tools/bag_replay/README.md",
}

SOURCES = {source: page for page, source in MOUNTS.items()}

# A figure a page carries from outside docs/ is copied in under figures/, so
# the site never reaches back to GitHub to render its own pages.
FIGURES = {".webp", ".png", ".jpg", ".jpeg", ".gif", ".svg"}

# An inline link or image: the bracket part, the target, and an optional title.
# The bracket part may wrap across lines, so this is matched over whole
# fence-free spans of the file rather than line by line.
LINK = re.compile(
    r"(!?\[[^\]]*\])\((<[^>]*>|[^()\s]+)((?:\s+(?:\"[^\"]*\"|'[^']*'))?)\)"
)
FENCE = re.compile(r"^\s{0,3}(```+|~~~+)")
SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")

_figures: dict[str, Path] = {}


def _resolve(url: str, source: str, page: str) -> str | None:
    """Rewrite one link target, or return None to leave it as written.

    ``source`` is the repo-relative path of the file the link was written in;
    ``page`` is the docs-relative path of the site page it will be served on.
    """
    if not url or url.startswith(("#", "/")) or SCHEME.match(url):
        return None

    path, sep, anchor = url.partition("#")
    if not path:
        return None

    target = posixpath.normpath(posixpath.join(posixpath.dirname(source), path))
    if target.startswith(".."):
        return None
    absolute = ROOT / target
    if not absolute.exists():
        return None

    destination = SOURCES.get(target)
    if destination is None and absolute.is_relative_to(DOCS):
        destination = absolute.relative_to(DOCS).as_posix()
    if destination is None and absolute.suffix.lower() in FIGURES:
        destination = f"figures/{target}"
        _figures[destination] = absolute
    if destination is not None:
        return posixpath.relpath(destination, posixpath.dirname(page)) + sep + anchor

    kind = "tree" if absolute.is_dir() else "blob"
    return f"{REPO}/{kind}/{BRANCH}/{target}{sep}{anchor}"


def rewrite(markdown: str, source: str, page: str) -> str:
    """Rewrite the repo-relative links in one file's markdown."""

    def replace(match: re.Match) -> str:
        brackets, url, title = match.groups()
        bare = url[1:-1] if url.startswith("<") and url.endswith(">") else url
        resolved = _resolve(bare, source, page)
        return match.group(0) if resolved is None else f"{brackets}({resolved}{title})"

    out: list[str] = []
    span: list[str] = []
    fence: str | None = None
    for line in markdown.split("\n"):
        marker = FENCE.match(line)
        if fence is not None:
            out.append(line)
            if marker and marker.group(1).startswith(fence):
                fence = None
        elif marker:
            if span:
                out.append(LINK.sub(replace, "\n".join(span)))
                span = []
            fence = marker.group(1)[0] * 3
            out.append(line)
        else:
            span.append(line)
    if span:
        out.append(LINK.sub(replace, "\n".join(span)))
    return "\n".join(out)


def on_config(config):
    """Slug headings the way GitHub does.

    Every heading link already written in the repo was written against a file
    rendered on GitHub, and the two sluggers disagree — an em dash leaves
    GitHub a double hyphen where Python-Markdown leaves a single one. Matching
    GitHub is what lets those links keep working here unchanged.
    """
    config.mdx_configs.setdefault("toc", {})["slugify"] = slugify(case="lower")
    return config


def on_files(files, config):
    # Per build, not per process: mkdocs caches the hook module, so under
    # `mkdocs serve` this survives every rebuild, and a figure edited out or
    # renamed mid-session would be read from its old path and raise.
    _figures.clear()

    for page, source in MOUNTS.items():
        markdown = (ROOT / source).read_text(encoding="utf-8")
        files.append(
            File.generated(config, page, content=rewrite(markdown, source, page))
        )

    # A page under docs/ can reference a figure outside it too, and figures are
    # collected as links are resolved — so resolve those pages' links here as
    # well, discarding the result. The rewrite is deterministic, so the pass
    # that counts (on_page_markdown, below) agrees with this one.
    for file in list(files.documentation_pages()):
        if file.src_uri not in MOUNTS and file.abs_src_path:
            source = f"docs/{file.src_uri}"
            rewrite(
                Path(file.abs_src_path).read_text(encoding="utf-8"),
                source,
                file.src_uri,
            )

    for page, figure in _figures.items():
        files.append(File.generated(config, page, content=figure.read_bytes()))
    return files


def on_page_markdown(markdown, page, config, files):
    if page.file.src_uri in MOUNTS:
        return None  # already rewritten on the way in
    return rewrite(markdown, f"docs/{page.file.src_uri}", page.file.src_uri)


def on_page_context(context, page, config, nav):
    """Point a mounted page's edit link at the file it was copied from."""
    source = MOUNTS.get(page.file.src_uri)
    if source:
        page.edit_url = f"{REPO}/edit/{BRANCH}/{source}"
    return context

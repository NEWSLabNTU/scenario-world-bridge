"""The bridge's per-episode files, cut per run: gap traces and contact reports.

docs/design/projects-and-pilots.md, "Logs". The bridge writes one gap trace per SSv2
``Initialize`` (``<gap_trace_dir>/<UTC time>_<episode>.jsonl``, roadmap 024) and one contact
report per ego run (``<contact_report_dir>/<UTC time>_e<episode>.json``, roadmap 021), into
directories it reads once, at start: they cannot follow a run. So the stack points both at
its own directory (``stacks/<id>/episodes/gap_trace/``, ``.../contacts/``), and each run
takes the files that appeared there between its start and its end, moving them into
``runs/<id>/gap_trace/`` and ``runs/<id>/contacts/``. One run is one SSv2 process, and SSv2
despawns every entity when its scenario ends, which closes the trace and writes the report
before SSv2 exits; a TIER IV scenario's variants are several episodes, so several files.
"""
import json
import os
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path

GAP_TRACE, CONTACTS = "gap_trace", "contacts"
KINDS = (GAP_TRACE, CONTACTS)
SETTLE = 5.0      # s the writer threads get to finish a file after SSv2 exits
QUIET = 0.5       # s a file must not grow to count as finished


def stack_dirs(stack_dir: Path, gap_trace: bool, contacts: bool) -> dict:
    """``{kind: directory}`` the stack's bridge writes into (None: off or the user's)."""
    root = Path(stack_dir) / "episodes"
    return {GAP_TRACE: str(root / GAP_TRACE) if gap_trace else None,
            CONTACTS: str(root / CONTACTS) if contacts else None}


def _names(d) -> set:
    try:
        return {p.name for p in Path(d).iterdir() if p.is_file()}
    except OSError:
        return set()


@dataclass
class Mark:
    """The files each directory held when the run started."""
    dirs: dict
    before: dict = field(default_factory=dict)

    @classmethod
    def take(cls, dirs: dict) -> "Mark":
        dirs = {k: v for k, v in (dirs or {}).items() if v}
        return cls(dirs, {k: _names(v) for k, v in dirs.items()})

    def new(self) -> dict:
        """``{kind: [paths]}`` of the files that appeared since, oldest name first."""
        out = {}
        for k, d in self.dirs.items():
            names = sorted(_names(d) - self.before.get(k, set()))
            if names:
                out[k] = [Path(d) / n for n in names]
        return out


def _finished(path: Path, kind: str) -> bool:
    if kind != GAP_TRACE:
        return True
    try:
        with open(path, "rb") as f:
            f.seek(max(0, os.path.getsize(path) - 4096))
            tail = f.read().decode("utf-8", "replace").strip().splitlines()
        return bool(tail) and json.loads(tail[-1]).get("type") == "end"
    except (OSError, ValueError):
        return False


def settle(mark: Mark, timeout: float = SETTLE, quiet: float = QUIET) -> dict:
    """Wait (up to ``timeout``) until every new file is finished: a gap trace has its ``end``
    line, and no file grew for ``quiet`` s. Returns ``mark.new()`` then."""
    if not mark.dirs:
        return {}
    deadline = time.time() + timeout
    sizes, still = {}, 0.0
    while True:
        new = mark.new()
        now = {str(p): p.stat().st_size for ps in new.values() for p in ps if p.exists()}
        done = all(_finished(p, k) for k, ps in new.items() for p in ps)
        if now == sizes:
            still += 0.1
        else:
            sizes, still = now, 0.0
        if (done and still >= quiet) or time.time() >= deadline:
            return new
        time.sleep(0.1)


def collect(mark: Mark, run_dir: Path, timeout: float = SETTLE) -> dict:
    """Move the run's files into ``run_dir/<kind>/``; ``{kind: [moved paths]}``."""
    out = {}
    for kind, paths in settle(mark, timeout).items():
        dest = Path(run_dir) / kind
        dest.mkdir(parents=True, exist_ok=True)
        moved = []
        for p in paths:
            target = dest / p.name
            try:
                os.replace(p, target)   # the bridge's writer keeps its handle: same inode
            except OSError:
                try:
                    shutil.copyfile(p, target)
                except OSError:
                    continue
            moved.append(target)
        if moved:
            out[kind] = moved
    return out

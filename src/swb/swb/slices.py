"""A run's slices: the byte range of each stack process log between the run's start and end.

docs/design/projects-and-pilots.md, "Logs". The stack writes ``stacks/<id>/proc/<group>/
<process>.log``; the runner takes a ``LogMark`` of that directory before SSv2 starts and
another after it ends, and ``cut`` copies each file's range to ``runs/<id>/logs/<group>/``.

A file that shrank between the marks (truncated, or rotated and recreated) is taken from 0;
a file new at the end is taken whole, unless it is an old file renamed by rotation (the same
inode), which is taken from its start offset.
"""
import dataclasses
import json
import os
import time
from pathlib import Path

TOP_GROUP = "stack"  # files directly under proc/, outside any group


@dataclasses.dataclass
class LogMark:
    root: str
    t_wall: float
    files: dict  # relative path -> [size, inode]

    @classmethod
    def take(cls, proc_dir) -> "LogMark":
        root = Path(proc_dir)
        files = {}
        if root.is_dir():
            for dirpath, _dirs, names in os.walk(root):
                for n in names:
                    p = Path(dirpath) / n
                    try:
                        st = p.stat()
                    except OSError:
                        continue  # gone between listing and stat
                    if p.is_file():
                        files[p.relative_to(root).as_posix()] = [st.st_size, st.st_ino]
        return cls(root=str(root), t_wall=time.time(), files=files)

    def size(self, rel):
        f = self.files.get(rel)
        return f[0] if f else None

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d) -> "LogMark":
        return cls(root=d["root"], t_wall=d["t_wall"],
                   files={k: list(v) for k, v in d["files"].items()})

    def write(self, path) -> Path:
        Path(path).write_text(json.dumps(self.to_dict()))
        return Path(path)

    @classmethod
    def read(cls, path) -> "LogMark":
        return cls.from_dict(json.loads(Path(path).read_text()))


def group_of(rel: str) -> tuple:
    """``(group, path inside the group)`` for a path relative to proc/."""
    parts = rel.split("/")
    if len(parts) == 1:
        return TOP_GROUP, parts[0]
    return parts[0], "/".join(parts[1:])


def _start_offset(rel, start: LogMark, end_entry) -> int:
    size_end, ino_end = end_entry
    before = start.files.get(rel)
    if before is not None:
        size0, ino0 = before
        if ino0 == ino_end and size0 <= size_end:
            return size0
        if ino0 != ino_end:
            # replaced by a new file: was the new one somewhere else at the start?
            for other, (s, i) in start.files.items():
                if i == ino_end and other != rel and s <= size_end:
                    return s
        return 0  # shrank: truncated or rotated, the whole file is this run's
    for other, (s, i) in start.files.items():
        if i == ino_end and s <= size_end:
            return s  # renamed by rotation: what was there at the start is not ours
    return 0


def cut(mark_start: LogMark, mark_end: LogMark, dest) -> list:
    """Copy each file's slice into ``dest/<group>/<file>``; returns the written paths.

    Reads at most up to the end mark's size (the stack keeps writing). Files with an empty
    slice are not written."""
    root = Path(mark_end.root)
    dest = Path(dest)
    written = []
    for rel, entry in sorted(mark_end.files.items()):
        begin, stop = _start_offset(rel, mark_start, entry), entry[0]
        if stop <= begin:
            continue
        src = root / rel
        try:
            with open(src, "rb") as f:
                f.seek(begin)
                data = f.read(stop - begin)
        except OSError:
            continue  # removed since the mark
        if not data:
            continue
        group, inner = group_of(rel)
        out = dest / group / inner
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)
        written.append(out)
    return written

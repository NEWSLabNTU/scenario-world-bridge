"""The log budget: refuse to start runs whose logs cannot fit.

``logging.budget`` in ``testbench.yaml`` or a suite (docs/design/projects-and-pilots.md, "Logs")::

    budget: {per_run: 2GB, total: 200GB}

``per_run`` is what one run may take while it is captured (before ``keep`` prunes it);
``total`` caps the project's runs directory. Before the first run, scensim needs
``per_run x runs`` both free on the log disk and within ``total`` less what the runs directory
already holds; otherwise it refuses, naming the numbers and what to do. Sizes are bytes or a
number with B, KB, MB, GB or TB (powers of 1024).
"""
import os
import re
import shutil
from pathlib import Path

UNITS = {"": 1, "B": 1, "K": 1024, "KB": 1024, "M": 1024 ** 2, "MB": 1024 ** 2,
         "G": 1024 ** 3, "GB": 1024 ** 3, "T": 1024 ** 4, "TB": 1024 ** 4}
KEYS = ("per_run", "total")
_SIZE = re.compile(r"^\s*([0-9]+(?:\.[0-9]+)?)\s*([A-Za-z]*)\s*$")


def parse_size(v) -> int:
    """Bytes in ``v`` (an int, or ``"2GB"``); ValueError naming it otherwise."""
    if isinstance(v, bool):
        raise ValueError(f"{v!r} is not a size")
    if isinstance(v, (int, float)):
        if v < 0:
            raise ValueError(f"{v!r} is not a size")
        return int(v)
    m = _SIZE.match(str(v))
    if not m or m.group(2).upper() not in UNITS:
        raise ValueError(f"{v!r} is not a size (e.g. 500MB, 2GB)")
    return int(float(m.group(1)) * UNITS[m.group(2).upper()])


def fmt(n: int) -> str:
    for unit in ("TB", "GB", "MB", "KB"):
        if n >= UNITS[unit]:
            return f"{n / UNITS[unit]:.1f} {unit}"
    return f"{n} B"


def problems(b) -> list:
    """Every way a ``budget:`` mapping is wrong, as messages."""
    if b is None:
        return []
    if not isinstance(b, dict):
        return ["is not a mapping: {per_run: 2GB, total: 200GB}"]
    out = [f"unknown key {k} (use only {', '.join(KEYS)})" for k in b if k not in KEYS]
    for k in KEYS:
        if b.get(k) is not None:
            try:
                parse_size(b[k])
            except ValueError as e:
                out.append(f"{k}: {e}")
    return out


def dir_size(root: Path) -> int:
    """Bytes under ``root``, following the links a runs directory holds to run directories
    once each, never out of a cycle."""
    total, seen = 0, set()
    stack = [Path(root)]
    while stack:
        d = stack.pop()
        try:
            st = d.stat()
        except OSError:
            continue
        if (st.st_dev, st.st_ino) in seen:
            continue
        seen.add((st.st_dev, st.st_ino))
        try:
            entries = list(os.scandir(d))
        except OSError:
            continue
        for e in entries:
            try:
                if e.is_dir():
                    stack.append(Path(e.path))
                elif e.is_file(follow_symlinks=False):
                    total += e.stat(follow_symlinks=False).st_size
            except OSError:
                continue
    return total


def check(budget, n_runs: int, runs_root: Path, free=None, used=None) -> str:
    """Empty when ``n_runs`` fit ``budget``; else the reason to refuse. ``free``/``used``
    replace the disk's free space and the runs directory's size (tests)."""
    if not budget or not budget.get("per_run") or n_runs <= 0:
        return ""
    need = parse_size(budget["per_run"]) * n_runs
    root = Path(runs_root)
    if free is None:
        probe = root
        while not probe.exists() and probe != probe.parent:
            probe = probe.parent
        free = shutil.disk_usage(probe).free
    if need > free:
        return (f"{n_runs} run(s) at logging.budget.per_run {fmt(need // n_runs)} need "
                f"{fmt(need)}; {fmt(free)} is free under {root}: prune "
                "(`ros2 run scensim runs prune`), lower capture or per_run, or run fewer")
    if budget.get("total"):
        total = parse_size(budget["total"])
        if used is None:
            used = dir_size(root) if root.exists() else 0
        if used + need > total:
            return (f"{n_runs} run(s) at logging.budget.per_run {fmt(need // n_runs)} need "
                    f"{fmt(need)}; the runs hold {fmt(used)} of logging.budget.total "
                    f"{fmt(total)}: prune (`ros2 run scensim runs prune`), raise total, or run fewer")
    return ""

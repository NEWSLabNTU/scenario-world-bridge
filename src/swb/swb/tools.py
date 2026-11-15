"""The host tools the project workflow needs, and their floors (roadmap 077, G26); the
GPU's state when a stack starts (G27); a workspace checked out off its pins (G31); the
Python modules CARLA's agents import (G38).

``project check`` (so the stack before it starts) refuses a play_launch older than the
workflow needs: an older one parses the stack's launch differently and fails later, far from
the cause (a host with 0.18.3 needed another install before a run, 2026-11-15).
"""
import re
import shutil
import subprocess

from .findings import Finding

PLAY_LAUNCH_MIN = (0, 30, 0)
PLAY_LAUNCH_FIX = "python3 -m pip install --user 'play_launch>=0.30.0'"


def play_launch_version():
    """``(version tuple, path)`` of the play_launch on PATH; ``(None, path)`` when its
    version does not parse; ``(None, None)`` when there is none."""
    path = shutil.which("play_launch")
    if path is None:
        return None, None
    try:
        out = subprocess.run([path, "--version"], capture_output=True, text=True,
                             timeout=20).stdout
    except (OSError, subprocess.SubprocessError):
        return None, path
    return parse_version(out), path


def parse_version(out: str):
    """The version tuple in ``play_launch --version``'s output, else None."""
    m = re.search(r"play_launch\s+(\d+)\.(\d+)\.(\d+)", out)
    return tuple(int(g) for g in m.groups()) if m else None


def findings(version_fn=None) -> list:
    version, path = (version_fn or play_launch_version)()
    floor = ".".join(map(str, PLAY_LAUNCH_MIN))
    if path is None:
        return [Finding("host", "play_launch is not on PATH (the stack runs under it)",
                        PLAY_LAUNCH_FIX, refused=True)]
    if version is None:
        return [Finding("host", f"{path}: its version does not parse (`play_launch --version`)"
                        f"; the workflow needs {floor} or newer", PLAY_LAUNCH_FIX)]
    if version < PLAY_LAUNCH_MIN:
        have = ".".join(map(str, version))
        return [Finding("host", f"play_launch {have} ({path}) is older than {floor}, which the "
                        "stack needs", PLAY_LAUNCH_FIX, refused=True)]
    return []


# GPU memory a stack's simulator needs on its card, roughly, as measured on Town01 (CARLA 0.9.16
# at Epic quality, Isaac Sim 6.1 headless with the SimReady town), and a busy card's load
GPU_NEED_MB = {"carla": 2500, "isaac": 10000}
GPU_BUSY_PERCENT = 80


def gpu_state(index: int):
    """``(used MiB, total MiB, utilisation %)`` of GPU ``index`` (nvidia-smi), else None."""
    try:
        out = subprocess.run(["nvidia-smi", f"--id={index}", "--query-gpu=memory.used,"
                              "memory.total,utilization.gpu", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return None
    try:
        used, total, util = (int(float(x)) for x in out.stdout.strip().split(","))
    except ValueError:
        return None
    return used, total, util


def gpu_note(index: int, simulator: str, state) -> str:
    """A warning when GPU ``index`` has less memory free than ``simulator`` needs, or is busy;
    "" when it looks fine or is unknown. A warning, not a refusal: other users' jobs come and
    go, and the simulator may still fit."""
    if state is None:
        return ""
    used, total, util = state
    free, need = total - used, GPU_NEED_MB.get(simulator, 0)
    why = []
    if need and free < need:
        why.append(f"{free / 1024:.1f} GB free, {simulator} needs about {need / 1024:.1f}")
    if util >= GPU_BUSY_PERCENT:
        why.append(f"{util} % busy")
    if not why:
        return ""
    return (f"GPU {index} ({used / 1024:.1f} of {total / 1024:.1f} GB in use): "
            + ", ".join(why) + "; `nvidia-smi` names its processes, the host setting `gpu:` "
            "(or the seat table's gpus) picks another card")


def submodule_drift(workspace) -> list:
    """``[(path, checked out, pinned)]`` for every submodule of ``workspace``, nested ones
    (a component's ``deps/``) too, checked out at another commit than its parent pins, or not
    checked out at all (``checked out`` None). A build from such a tree runs code the pins do
    not name: on 2026-11-16 a non-recursive update left acb's ``deps/`` on an older vehicle
    spec, and the bridge refused the ego a minute into the run (roadmap 077, G31)."""
    try:
        out = subprocess.run(["git", "submodule", "status", "--recursive"], cwd=str(workspace),
                             capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError):
        return []
    if out.returncode != 0:
        return []
    drift = []
    for line in out.stdout.splitlines():
        if not line or line[0] not in "+-":
            continue
        parts = line[1:].split()
        if len(parts) < 2:
            continue
        sha, path = parts[0], parts[1]
        if line[0] == "-":
            drift.append((path, None, sha))
            continue
        pinned = _pinned(workspace, path)
        drift.append((path, sha, pinned))
    return drift


def _pinned(workspace, path):
    """The commit that ``path``'s parent repository (the innermost one holding it) pins."""
    import os
    full = os.path.join(str(workspace), path)
    try:
        top = subprocess.run(["git", "-C", os.path.dirname(full), "rev-parse", "--show-toplevel"],
                             capture_output=True, text=True, timeout=20).stdout.strip()
        if not top:
            return None
        rel = os.path.relpath(full, top)
        return subprocess.run(["git", "-C", top, "rev-parse", f"HEAD:{rel}"],
                              capture_output=True, text=True, timeout=20).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def drift_note(drift: list) -> str:
    """One warning line for ``submodule_drift``'s list, "" when it is empty."""
    if not drift:
        return ""
    items = [f"{p} at {c[:7]} (pinned {(pin or '?')[:7]})" if c else f"{p} not checked out"
             for p, c, pin in drift]
    shown = items[:4] + ([f"and {len(items) - 4} more"] if len(items) > 4 else [])
    return ("the workspace is off its pins: " + "; ".join(shown) + ". A build from it may "
            "run other code than the pins name: `git submodule update --init --recursive`, "
            "then rebuild (unless you moved them on purpose)")


# CARLA's agents (PythonAPI/carla/agents, the carla_basic pilot) import these at start; a host
# without them crash-loops the pilot (roadmap 077, G38)
AGENT_MODULES = ("shapely", "networkx")
AGENT_FIX = ("sudo apt install python3-shapely python3-networkx, or "
             "python3 -m pip install --user shapely networkx")


def missing_modules(modules, python="python3"):
    """The modules of ``modules`` that do not import under ``python`` (the interpreter the
    stack's pilots run with, first on PATH); [] when that python cannot be run."""
    missing = []
    for m in modules:
        try:
            r = subprocess.run([python, "-c", f"import {m}"], capture_output=True, timeout=30)
        except (OSError, subprocess.SubprocessError):
            return []
        if r.returncode != 0:
            missing.append(m)
    return missing


def agent_findings(settings: dict, missing_fn=None) -> list:
    """``project check``'s finding for a carla_basic pilot on a host without its modules."""
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    users = sorted(k for k, p in pilots.items()
                   if isinstance(p, dict) and p.get("kind") == "carla_basic")
    if not users:
        return []
    missing = (missing_fn or missing_modules)(AGENT_MODULES)
    if not missing:
        return []
    return [Finding("host", f"python3 cannot import {', '.join(missing)}, which CARLA's agents "
                    f"need (the carla_basic pilot{'s' if len(users) > 1 else ''} "
                    f"{', '.join(users)})", AGENT_FIX, refused=True)]

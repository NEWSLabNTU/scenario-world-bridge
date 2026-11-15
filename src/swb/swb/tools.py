"""The host tools the project workflow needs, and their floors (roadmap 077, G26).

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

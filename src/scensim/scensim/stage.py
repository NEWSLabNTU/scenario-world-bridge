"""An authored Isaac map's stage (roadmap 091 W15, W16; design roboracer-indoor-twin.md 3.4).

The map check: ``testbench check`` runs scensim_isaac_world's ``stage_record map`` on every map
whose ``isaac/world.yaml`` says ``authored: true``, in a Python that has pxr (Isaac's, else
this one or ``python3`` with usd-core), and refuses the testbench with each problem it names
(units, up axis, a PhysX vehicle prim, an unresolved dependency, an unknown class token, a
friction out of range). Without such a Python the check is skipped, and ``notes`` says so.

The run's record: the world server writes an authored map's ``provenance.stage`` to
``<project>/state/<map>/stage.json`` at each load; ``provenance`` puts it into the run's
``result.json``.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

from .findings import Finding

STAGE_JSON = "stage.json"
WORLD_FILE = Path("isaac") / "world.yaml"
CHECK_TIMEOUT_S = 600  # a scanned field's layers are read, not hashed


def _sources(ws):
    """PYTHONPATH for the check: scensim_isaac_world's sources and scensim_world_server's (its
    class table)."""
    if ws is None:
        return []
    ws = Path(ws)
    return [p for p in (ws / "src" / "scensim_isaac_world",
                        ws / "src" / "scensim_world_protocol" / "scensim_world_server")
            if p.is_dir()]


def _has_pxr(python, env) -> bool:
    try:
        return subprocess.run([str(python), "-c", "import pxr, yaml, numpy"], env=env,
                              capture_output=True, timeout=60).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def pxr_python(host: dict, env=None):
    """``(python, tried)``: the first Python that imports pxr (with PyYAML and numpy): Isaac's
    venv (``isaac.root``), this one, ``python3``; None and every one tried when none does."""
    from . import isaac as isaacmod
    env = os.environ if env is None else env
    tried = []
    r = isaacmod.root(host)
    for py in ([Path(r) / "venv" / "bin" / "python"] if r else []) + [Path(sys.executable),
                                                                        Path("python3")]:
        if py.is_absolute() and not py.is_file():
            continue
        tried.append(str(py))
        if _has_pxr(py, env):
            return str(py), tried
    return None, tried


def authored_maps(project, settings: dict) -> list:
    """``[(map, world.yaml)]`` of the project's maps whose world file says ``authored: true``
    (testbench.yaml's ``maps:``, else every map under ``assets/maps``)."""
    import yaml
    root = Path(project) / "assets" / "maps"
    names = settings.get("maps") if isinstance(settings.get("maps"), list) else \
        sorted(p.name for p in root.iterdir()) if root.is_dir() else []
    out = []
    for m in names:
        wy = root / str(m) / WORLD_FILE
        try:
            doc = yaml.safe_load(wy.read_text()) or {}
        except (OSError, yaml.YAMLError):
            continue  # a missing or broken world file is the world's own finding
        if isinstance(doc, dict) and doc.get("authored") is True:
            out.append((str(m), wy))
    return out


_SKIPPED = {}  # project -> why the map check did not run, for ``notes``
_NOTES = {}    # project -> the map check's notes (refusing nothing: an unsourced friction, W14)


def map_findings(project, settings: dict, host: dict, ws=None) -> list:
    """The map check of each authored map: one refusing finding per problem. Nothing when the
    project has no authored map; when no Python with pxr is found, nothing either, and
    ``notes`` says the check was skipped and why."""
    project = Path(project)
    _SKIPPED.pop(str(project), None)
    _NOTES.pop(str(project), None)
    maps = authored_maps(project, settings)
    if not maps:
        return []
    if ws is None:
        from . import stack as stackmod
        ws = stackmod.workspace_root()
    src = _sources(ws)
    if len(src) < 2:
        _SKIPPED[str(project)] = (f"the authored maps' check ({', '.join(m for m, _ in maps)}) "
                                  "was skipped: scensim_isaac_world's sources are not in this "
                                  "workspace")
        return []
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
    env["PYTHONPATH"] = os.pathsep.join(map(str, src))
    python, tried = pxr_python(host, env)
    if python is None:
        _SKIPPED[str(project)] = (f"the authored maps' check ({', '.join(m for m, _ in maps)}) "
                                  f"was skipped: no Python with pxr (tried {', '.join(tried)}); "
                                  "set isaac.root, or pip install usd-core")
        return []
    out = []
    state = project / "state"
    for m, wy in maps:
        where = str(Path("assets") / "maps" / m / WORLD_FILE)
        try:
            r = subprocess.run([python, "-m", "scensim_isaac_world.stage_record", "map",
                                str(wy), "--state-dir", str(state)], env=env,
                               capture_output=True, text=True, timeout=CHECK_TIMEOUT_S)
        except (OSError, subprocess.TimeoutExpired) as e:
            out.append(Finding(where, f"the map check did not run: {e}", refused=True))
            continue
        try:
            res = json.loads(r.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            tail = (r.stderr or r.stdout).strip().splitlines()[-1:] or [""]
            out.append(Finding(where, f"the map check failed: {tail[0][:300]}", refused=True))
            continue
        for p in res.get("problems") or []:
            out.append(Finding(where, p, "fix the stage in Isaac's editor (design "
                               "roboracer-indoor-twin.md 3)", refused=True))
        for n in res.get("notes") or []:
            _NOTES.setdefault(str(project), []).append(f"{where}: {n}")
    return out


def notes(project) -> list:
    """Why the map check was skipped, when it was, and its notes (after ``map_findings``):
    an authored friction with no ``scensim:source`` (roadmap 091 W14), and the like."""
    why = _SKIPPED.get(str(Path(project)))
    return ([why] if why else []) + list(_NOTES.get(str(Path(project)), []))


def provenance(project, rec: dict):
    """``provenance.stage`` of a run on an Isaac stack: the world server's record of the map's
    stage (``<project>/state/<map>/stage.json``) when the map is authored; None otherwise. A
    record missing or unreadable says so (``{"missing": why}``)."""
    if (rec or {}).get("simulator") != "isaac" or not rec.get("map") or project is None:
        return None
    m = str(rec["map"])
    if not any(name == m for name, _ in authored_maps(project, {"maps": [m]})):
        return None
    path = Path(project) / "state" / m / STAGE_JSON
    try:
        doc = json.loads(path.read_text())
    except FileNotFoundError:
        return {"missing": f"{path} not written (a world server before 091 W15, or its own)"}
    except (OSError, ValueError) as e:
        return {"missing": f"{path}: {e}"}
    return doc

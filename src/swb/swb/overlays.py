"""A pilot's extra ROS workspaces (roadmap 077, G60): ``overlays: [<setup.bash>, ...]``.

An ``autoware`` or ``launch`` pilot sources each entry, in order, after this workspace's
installs and before the TensorRT runtime (autoware_env.bash, overlay_env.bash), so a driver
can include a package built in a workspace of its own (autoware-drift's drift_controller).

An entry is a path relative to the project, or ``{workspace:<name>}/<path>``: the directory
the host settings name, ``workspaces: {<name>: <dir>}``, as G8's ``datasets:`` names a
dataset's copy, so the project file reads the same on every host. An absolute or ``~`` path
works but is a host path in a project file, and ``project check`` warns about it.
"""
import os
import re
from pathlib import Path

from swb import config
from swb.findings import Finding

KINDS = ("autoware", "launch")
WORKSPACE = re.compile(r"^\{workspace:([^}/]+)\}(?:/(.*))?$")
HOST_FILE = "~/.config/swb/host.yaml"


def host_workspaces(host: dict) -> dict:
    """``{workspace name: directory}`` from the host settings' ``workspaces:``."""
    w = (host or {}).get("workspaces")
    w = w if isinstance(w, dict) else {}
    return {str(k): Path(os.path.expanduser(str(v))) for k, v in w.items() if v}


def entries(spec: dict) -> list:
    """The pilot's ``overlays:`` when it is a list of strings, else []."""
    v = (spec or {}).get("overlays")
    return [e for e in v if isinstance(e, str)] if isinstance(v, list) else []


def is_host_path(entry: str) -> bool:
    return entry.startswith("~") or os.path.isabs(entry)


def resolve(project, entry: str, host: dict) -> dict:
    """``{entry, path, workspace, problem}`` for one entry; ``problem`` is ``(what, fix)``
    when it cannot be used (an unknown workspace, no such file), else None."""
    out = {"entry": entry, "path": None, "workspace": None, "problem": None}
    m = WORKSPACE.match(entry)
    if m:
        name, rest = m.group(1), m.group(2) or ""
        out["workspace"] = name
        root = host_workspaces(host).get(name)
        if root is None:
            out["problem"] = (f"{entry}: this host names no workspace {name}",
                              f"workspaces: {{{name}: <dir>}} in {HOST_FILE}")
            return out
        path = root / rest
    elif entry.startswith("{"):
        out["problem"] = (f"{entry}: not a project path or {{workspace:<name>}}/<path>",
                          "{workspace:<name>}/install/setup.bash")
        return out
    elif is_host_path(entry):
        path = Path(os.path.expanduser(entry))
    else:
        path = Path(project) / entry
    out["path"] = str(path)
    if not path.is_file():
        fix = (f"build the workspace, or set workspaces: {{{out['workspace']}: <dir>}} in"
               f" {HOST_FILE} to where it is built" if out["workspace"] else
               "build it, or name the setup.bash it has")
        out["problem"] = (f"{entry}: no such file ({path})", fix)
    return out


def schema_problems(name: str, spec: dict) -> list:
    """The shape of ``pilots.<name>.overlays`` (no host settings needed)."""
    v = spec.get("overlays")
    if v is None:
        return []
    where = f"pilots.{name}.overlays"
    if not isinstance(v, list) or not all(isinstance(e, str) and e for e in v):
        return [Finding(config.PROJECT_FILE, f"{where}: is not a list of setup files",
                        "overlays: [{workspace:<name>}/install/setup.bash]")]
    return []


SUBST = re.compile(r"\{workspace:([^}/]+)\}")


def _strings(v):
    if isinstance(v, str):
        yield v
    elif isinstance(v, dict):
        for x in v.values():
            yield from _strings(x)
    elif isinstance(v, list):
        for x in v:
            yield from _strings(x)


def problems(project, settings: dict, host: dict) -> list:
    """Every overlay that cannot be sourced on this host, and every ``{workspace:<name>}`` in
    a pilot's ``args`` or ``run`` this host does not name, refused (the pilot would start
    without its packages)."""
    out = []
    known = host_workspaces(host)
    for name, spec in (settings.get("pilots") or {}).items():
        if not isinstance(spec, dict):
            continue
        for key in ("args", "run"):
            for w in sorted({m for s in _strings(spec.get(key)) for m in SUBST.findall(s)}):
                if w not in known:
                    out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.{key}: "
                                       f"{{workspace:{w}}}: this host names no workspace {w}",
                                       f"workspaces: {{{w}: <dir>}} in {HOST_FILE}",
                                       refused=True))
        if spec.get("kind") not in KINDS:
            continue
        for e in entries(spec):
            r = resolve(project, e, host)
            if r["problem"]:
                what, fix = r["problem"]
                out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.overlays: {what}", fix,
                                   refused=True))
    return out


def notes(settings: dict) -> list:
    """Warnings: an absolute or ``~`` entry, a host path in a project file."""
    out = []
    for name, spec in (settings.get("pilots") or {}).items():
        if not isinstance(spec, dict):
            continue
        for e in entries(spec):
            if is_host_path(e):
                out.append(f"pilots.{name}.overlays: {e}: a host path in a project file; name"
                           f" it as a host workspace ({{workspace:<name>}}/... with"
                           f" workspaces: {{<name>: <dir>}} in {HOST_FILE})")
    return out


def _commit(directory):
    from swb import results
    try:
        return results.checkout_commit(directory)
    except OSError:
        return None


def resolved(project, spec: dict, host: dict) -> list:
    """What a stack records per overlay: ``{entry, path, commit}``; the commit is the
    workspace's (the setup file's directory's for a path entry) when it is a git checkout."""
    out = []
    for e in entries(spec):
        r = resolve(project, e, host)
        if r["path"] is None:
            continue
        where = (host_workspaces(host)[r["workspace"]] if r["workspace"]
                 else Path(r["path"]).parent)
        out.append({"entry": e, "path": r["path"],
                    "commit": _commit(where) if Path(where).is_dir() else None})
    return out


def paths(recorded: list) -> list:
    """The setup files of a stack record's ``overlays``, in order."""
    return [o["path"] for o in recorded or [] if isinstance(o, dict) and o.get("path")]


def flags(recorded: list) -> list:
    """``--overlay <file>`` per recorded overlay, for autoware_env.bash / overlay_env.bash."""
    out = []
    for p in paths(recorded):
        out += ["--overlay", p]
    return out

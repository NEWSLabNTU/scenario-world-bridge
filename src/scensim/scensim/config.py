"""Settings every scensim command reads: the project and the host, from files, never exports.

Precedence, highest first (docs/design/projects-and-pilots.md, "Host settings"):

1. arguments given to the command or launch (``overrides``);
2. the checkout's ``<project>/.scensim/local.yaml`` (git-ignored);
3. the user's ``~/.config/scensim/host.yaml``;
4. the host's ``/etc/scensim/host.yaml`` (an administrator's, e.g. a shared seat table);
5. the defaults below.

``SCENSIM_CONFIG_HOME`` and ``SCENSIM_SYSTEM_CONFIG`` move the user's and the host's directory
(tests, containers); nothing else is read from the environment.
"""
import copy
import os
from pathlib import Path

import yaml

PROJECT_FILE = "swb.yaml"
LOCK_FILE = "scensim.lock"

DEFAULTS = {
    "seat": "auto",
    "gpu": None,  # None: the seat table's
    "logs": "~/.local/state/scensim/logs",
    "cache": "~/.cache/scensim",
    # datasets this host keeps a copy of itself, {<name>: <directory>} (roadmap 077, G8)
    "datasets": {},
    # ROS workspaces a pilot's overlays: name, {<name>: <directory>} (roadmap 077, G60)
    "workspaces": {},
    # start: auto (start CARLA from root when nothing serves host:port), start, external;
    # ready_timeout: s a started CARLA has to serve RPC (scensim.simulator); map_packages: the
    # directories this host's CARLA towns are installed in (default: <root>/CarlaUE4/Content),
    # a town installed into the binary being a host fact (roadmap 077, G13)
    "carla": {"root": "", "host": "localhost", "port": 2000, "start": "auto",
              "ready_timeout": 300, "map_packages": []},
    "isaac": {"root": ""},
    # the launcher of the stack, its pilots and SSv2's scenario: auto (play_launch 0.30 or
    # later when on PATH, else ros2 launch), play_launch or ros2 (roadmap 079)
    "launcher": "auto",
    # dds.profile: a Cyclone DDS XML for every process of the stack and every tool that talks
    # to it (a host whose network needs another, e.g. unicast on loopback); "": scensim_launch's
    # localhost profile (roadmap 077, G15)
    "dds": {"profile": ""},
    # the autoware pilot's environment, as `just ego-av` builds it (roadmap 077, G1); empty
    # paths: the workspace's own third_party/ builds, when there
    "autoware": {"setup": "/opt/autoware/1.5.0/setup.bash", "overlay": "",
                 "use_overlay": True, "cuda_ndt": "", "tensorrt_lib": ""},
    "seats": {
        # seat n: scenario domain base+stride*n, pilot domains the next stride-1
        "domains": {"base": 1, "stride": 8},
        # SSv2 +0 (+1, +2 its sensor-release sockets), relay +5, world server +36
        "ports": {"base": 5555, "stride": 100},
        "web": {"base": 8080, "stride": 10},
        "count": 7,
        "gpus": None,  # list, seat -> GPU index; None: GPU 0 for every seat
    },
    "seat_locks": "/tmp/scensim-seats",
}


class ConfigError(RuntimeError):
    """A settings file that cannot be used; the message names the file."""


def user_dir() -> Path:
    return Path(os.environ.get("SCENSIM_CONFIG_HOME") or Path.home() / ".config" / "scensim")


def system_dir() -> Path:
    return Path(os.environ.get("SCENSIM_SYSTEM_CONFIG") or "/etc/scensim")


def merge(base: dict, over: dict) -> dict:
    """``over`` into a copy of ``base``, dicts recursively; None in ``over`` keeps ``base``."""
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if v is None:
            continue
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def read_yaml(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: not valid YAML: {e}") from None
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top")
    return data


def find_project(start: Path = None) -> Path:
    """The project directory: ``start`` (default the working directory) or the nearest parent
    holding swb.yaml. A ``pkg://<name>[/<subdir>]`` start is a ROS package's share directory,
or a directory in it (``pkg://scensim_examples/projects/first_drive``)."""
    start = str(start) if start is not None else os.getcwd()
    if start.startswith("pkg://"):
        from ament_index_python.packages import get_package_share_directory
        pkg, _, sub = start[len("pkg://"):].partition("/")
        root = Path(get_package_share_directory(pkg)) / sub
        if not (root / PROJECT_FILE).is_file():
            raise ConfigError(f"{start}: the package installs no {PROJECT_FILE}")
        return root
    p = Path(start).expanduser().resolve()
    for d in (p, *p.parents):
        if (d / PROJECT_FILE).is_file():
            return d
    raise ConfigError(f"no {PROJECT_FILE} in {p} or above: run `ros2 run scensim project new <dir>`"
                      " or give project:=<dir>")


def package_project(project) -> str:
    """The installed package whose share holds ``project`` (``pkg://`` or a path into an
    install's ``share/``), else "". Such a project is read-only: it is copied to run
    (``project new --from pkg://...``), so its lock, assets, state and runs are the copy's."""
    p = str(project)
    for prefix in (x for x in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if x):
        share = os.path.join(prefix, "share") + os.sep
        if p.startswith(share):
            return p[len(share):].split(os.sep)[0]
    return ""


def writable_project(project) -> Path:
    """``project``, or ConfigError when it is an installed package's (``package_project``)."""
    pkg = package_project(project)
    if pkg:
        sub = str(project).split(os.sep + "share" + os.sep + pkg, 1)[-1].strip(os.sep)
        src = f"pkg://{pkg}" + (f"/{sub}" if sub else "")
        raise ConfigError(f"{project} is installed with the package {pkg}, read-only: copy it "
                          f"and run the copy (`ros2 run scensim project new <dir> --from {src}`); "
                          "its lock, assets, state and runs are then the copy's")
    return Path(project)


# Roadmap 080: the project was swb. Its variables are refused by name, naming the successor
# (no shims), and its host directories are not read: the first command that finds one prints
# how to move it.
OLD_ENV_PREFIX, ENV_PREFIX = "SWB_", "SCENSIM_"
_hinted = set()


def renamed_env(environ=None) -> list:
    """One line per variable of the old name in ``environ``, naming the one to set instead."""
    environ = os.environ if environ is None else environ
    return [f"{k} was renamed with the project (roadmap 080); set "
            f"{ENV_PREFIX}{k[len(OLD_ENV_PREFIX):]} instead (unset {k} in this shell and in "
            ".env/.envrc)" for k in sorted(environ) if k.startswith(OLD_ENV_PREFIX)]


def old_dirs(project: Path = None, home: Path = None) -> list:
    """``(old, new)`` for each directory of the old name that exists while its successor does
    not: the user's config, cache and state, and the project's ``.swb/``."""
    home = Path(home) if home is not None else Path.home()
    # a moved config home (tests, containers) is not the user's: their directories are not checked
    pairs = [] if os.environ.get("SCENSIM_CONFIG_HOME") and home == Path.home() else [(home / ".config" / "swb", home / ".config" / "scensim"),
             (home / ".cache" / "swb", home / ".cache" / "scensim"),
             (home / ".local" / "state" / "swb", home / ".local" / "state" / "scensim")]
    if project is not None:
        pairs.append((Path(project) / ".swb", Path(project) / ".scensim"))
    return [(o, n) for o, n in pairs if o.is_dir() and not n.exists()]


def check_renamed(project: Path = None):
    """Refuse a variable of the old name (ConfigError, every one named); print, once per
    directory and process, the ``mv`` that moves an old directory. Old paths are never read."""
    lines = renamed_env()
    if lines:
        raise ConfigError("; ".join(lines))
    import sys
    for old, new in old_dirs(project):
        if old in _hinted:
            continue
        _hinted.add(old)
        print(f"scensim: {old} has the old name and is not read (roadmap 080); move it:\n"
              f"    mv {old} {new}", file=sys.stderr)


def host_settings(project: Path = None, overrides: dict = None) -> dict:
    """The merged host settings for a project (or none), paths expanded."""
    check_renamed(project)
    s = merge(DEFAULTS, read_yaml(system_dir() / "host.yaml"))
    s = merge(s, read_yaml(user_dir() / "host.yaml"))
    if project is not None:
        s = merge(s, read_yaml(Path(project) / ".scensim" / "local.yaml"))
    s = merge(s, overrides or {})
    for k in ("logs", "cache", "seat_locks"):
        s[k] = str(Path(s[k]).expanduser())
    return s


def project_settings(project: Path) -> dict:
    """The project file, as written (validation is scensim.project's)."""
    path = Path(project) / PROJECT_FILE
    if not path.is_file():
        raise ConfigError(f"{path}: no project file")
    return read_yaml(path)


def write_user_setting(key: str, value) -> Path:
    """Set one top-level key in the user's host.yaml, keeping the rest; returns the file."""
    path = user_dir() / "host.yaml"
    data = read_yaml(path)
    data[key] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path

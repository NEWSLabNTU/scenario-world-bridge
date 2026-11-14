"""Settings every swb command reads: the project and the host, from files, never exports.

Precedence, highest first (docs/design/projects-and-pilots.md, "Host settings"):

1. arguments given to the command or launch (``overrides``);
2. the checkout's ``<project>/.swb/local.yaml`` (git-ignored);
3. the user's ``~/.config/swb/host.yaml``;
4. the host's ``/etc/swb/host.yaml`` (an administrator's, e.g. a shared seat table);
5. the defaults below.

``SWB_CONFIG_HOME`` and ``SWB_SYSTEM_CONFIG`` move the user's and the host's directory
(tests, containers); nothing else is read from the environment.
"""
import copy
import os
from pathlib import Path

import yaml

PROJECT_FILE = "swb.yaml"
LOCK_FILE = "swb.lock"

DEFAULTS = {
    "seat": "auto",
    "gpu": None,  # None: the seat table's
    "logs": "~/.local/state/swb/logs",
    "cache": "~/.cache/swb",
    # start: auto (start CARLA from root when nothing serves host:port), start, external;
    # ready_timeout: s a started CARLA has to serve RPC (swb.simulator)
    "carla": {"root": "", "host": "localhost", "port": 2000, "start": "auto",
              "ready_timeout": 300},
    "isaac": {"root": ""},
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
    "seat_locks": "/tmp/swb-seats",
}


class ConfigError(RuntimeError):
    """A settings file that cannot be used; the message names the file."""


def user_dir() -> Path:
    return Path(os.environ.get("SWB_CONFIG_HOME") or Path.home() / ".config" / "swb")


def system_dir() -> Path:
    return Path(os.environ.get("SWB_SYSTEM_CONFIG") or "/etc/swb")


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
or a directory in it (``pkg://swb_examples/projects/first_drive``)."""
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
    raise ConfigError(f"no {PROJECT_FILE} in {p} or above: run `ros2 run swb project new <dir>`"
                      " or give project:=<dir>")


def host_settings(project: Path = None, overrides: dict = None) -> dict:
    """The merged host settings for a project (or none), paths expanded."""
    s = merge(DEFAULTS, read_yaml(system_dir() / "host.yaml"))
    s = merge(s, read_yaml(user_dir() / "host.yaml"))
    if project is not None:
        s = merge(s, read_yaml(Path(project) / ".swb" / "local.yaml"))
    s = merge(s, overrides or {})
    for k in ("logs", "cache", "seat_locks"):
        s[k] = str(Path(s[k]).expanduser())
    return s


def project_settings(project: Path) -> dict:
    """The project file, as written (validation is swb.project's)."""
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

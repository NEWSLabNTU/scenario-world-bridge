"""The host tools the project workflow needs, and their floors (roadmap 077, G26); the
GPU's state when a stack starts (G27); a workspace checked out off its pins (G31); the
Python modules CARLA's agents import (G38); the launcher (roadmap 079).

The launcher is a host setting, ``launcher: auto | play_launch | ros2`` (``auto`` by
default): play_launch when one at 0.30 or later is on PATH, else ``ros2 launch``. Only an
explicit ``play_launch`` that is missing or older than 0.30 is refused: an older one parses
the stack's launch differently and fails later, far from the cause (a host with 0.18.3
needed another install before a run, 2026-11-15).
"""
import os
import re
import shutil
import subprocess
from pathlib import Path

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


LAUNCHERS = ("auto", "play_launch", "ros2")


def _version_text(version):
    return ".".join(map(str, version)) if version else "?"


def launcher(setting="auto", version_fn=None):
    """``(launcher, why)``: the launcher a stack and a run use, ``play_launch`` or ``ros2``,
    for the host setting ``setting``, and the words that say why ("play_launch 0.35.1",
    "play_launch not on PATH"). An explicit ``play_launch`` is kept even when it is missing
    (``findings`` refuses it)."""
    setting = setting or "auto"
    version, path = (version_fn or play_launch_version)()
    if path is None:
        why = "play_launch not on PATH"
    elif version is None:
        why = f"{path}: play_launch's version does not parse"
    elif version < PLAY_LAUNCH_MIN:
        why = f"play_launch {_version_text(version)} is older than {_version_text(PLAY_LAUNCH_MIN)}"
    else:
        why = f"play_launch {_version_text(version)}"
    if setting == "ros2":
        return "ros2", "set"
    if setting == "play_launch":
        return "play_launch", why
    usable = path is not None and version is not None and version >= PLAY_LAUNCH_MIN
    return ("play_launch" if usable else "ros2"), why


def launcher_line(setting="auto", version_fn=None) -> str:
    """What `project check` and `seat show` print: "launcher: auto -> play_launch 0.35.1",
    "launcher: auto -> ros2 (play_launch not on PATH)", "launcher: ros2"."""
    setting = setting or "auto"
    name, why = launcher(setting, version_fn)
    if setting == "auto":
        return f"launcher: auto -> {why}" if name == "play_launch" else \
            f"launcher: auto -> ros2 ({why})"
    return f"launcher: {setting}" if setting == "ros2" else f"launcher: play_launch ({why})"


ROS2_CONTAINERS_NOTE = ("under ros2 launch, a crashed Autoware component ends its container "
                        "(play_launch restarts the component alone)")


def launcher_notes(launcher_name, pilot_kinds) -> list:
    """What the resolved launcher changes for these pilots, said once: under `ros2 launch`
    the pilots' isolated containers (play_launch's) are not there."""
    if launcher_name == "ros2" and any(k != "carla_basic" for k in pilot_kinds):
        return [ROS2_CONTAINERS_NOTE]
    return []


def rviz_display_note(pilots, env=None) -> list:
    """A pilot that opens RViz (`rviz: true`) on a host where DISPLAY is unset: RViz aborts at
    once and is respawned, and under play_launch the run ends `error` on its crashes ("its
    node /rviz2 crashed 17 times"); the cause is the terminal, not the pilot."""
    env = os.environ if env is None else env
    names = [n for n, p in (pilots or {}).items() if isinstance(p, dict) and p.get("rviz")]
    if names and not env.get("DISPLAY"):
        return [f"DISPLAY is not set, and {', '.join(sorted(names))} "
                f"{'opens' if len(names) == 1 else 'open'} RViz (rviz: true): it will crash "
                "at start (fix: export DISPLAY, or rviz: false)"]
    return []


def findings(setting="auto", version_fn=None) -> list:
    """Problems with the launcher: an unknown setting, or an explicit ``play_launch`` that is
    missing or older than the workflow needs (``auto`` falls back to ros2 instead)."""
    setting = setting or "auto"
    if setting not in LAUNCHERS:
        return [Finding("host settings", f"launcher: {setting!r} is not one of "
                        f"{', '.join(LAUNCHERS)}", "launcher: auto", refused=True)]
    if setting != "play_launch":
        return []
    version, path = (version_fn or play_launch_version)()
    floor = _version_text(PLAY_LAUNCH_MIN)
    if path is None:
        return [Finding("host settings", "launcher: play_launch, but play_launch is not on "
                        "PATH", f"{PLAY_LAUNCH_FIX}, or launcher: auto", refused=True)]
    if version is None:
        return [Finding("host settings", f"launcher: play_launch, but {path}: its version does "
                        f"not parse (`play_launch --version`); the workflow needs {floor} or "
                        "newer", PLAY_LAUNCH_FIX)]
    if version < PLAY_LAUNCH_MIN:
        return [Finding("host settings", f"launcher: play_launch, but play_launch "
                        f"{_version_text(version)} ({path}) is older than {floor}",
                        f"{PLAY_LAUNCH_FIX}, or launcher: auto", refused=True)]
    return []


RMW = "rmw_cyclonedds_cpp"


def rmw_fix(distro: str = None) -> str:
    """The install line for the sourced distro (ROS_DISTRO; humble when none is sourced)."""
    distro = distro or os.environ.get("ROS_DISTRO") or "humble"
    return (f"sudo apt install ros-{distro}-rmw-cyclonedds-cpp (or rosdep update, then rosdep "
            "install --from-paths src --ignore-src -y in the workspace)")


def rmw_installed() -> bool:
    """Whether Cyclone DDS's RMW, which every launch of the stack uses, is installed."""
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix(RMW)
        return True
    except Exception:  # noqa: BLE001 (not found, or no ROS environment)
        return False


def rmw_findings(installed_fn=None) -> list:
    """Cyclone DDS's RMW missing (roadmap 079, step 6): every process of the stack runs on it
    (scensim_launch's DDS profile), and a stock ROS 2 install may not have it."""
    if (installed_fn or rmw_installed)():
        return []
    return [Finding("host", f"{RMW} is not installed (the stack runs on Cyclone DDS)", rmw_fix(),
                    refused=True)]


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
        # --no-optional-locks: no index.lock left behind by a call killed at the timeout
        out = subprocess.run(["git", "--no-optional-locks", "submodule", "status", "--recursive"],
                             cwd=str(workspace),
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


# Autoware 1.5.0's TensorRT; another version rebuilds CenterPoint's engines at every Autoware
# start (minutes; CLAUDE.md, scripts/autoware_tensorrt.bash). A lifetime: run pilot starts
# Autoware for each run (roadmap 077, G54: the ego drove i1 blind while they built).
TENSORRT_FIX = ("just setup-tensorrt in the scensim checkout, then autoware: {tensorrt_lib: "
                "<its lib directory>} in ~/.config/scensim/host.yaml")


def centerpoint_pilots(project, settings: dict) -> list:
    """The autoware pilots that run CenterPoint (its ``args``, or a suite's ``pilot_args``)."""
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    aw = {k for k, p in pilots.items() if isinstance(p, dict) and p.get("kind") == "autoware"}

    def cp(args):
        return isinstance(args, dict) and \
            str(args.get("lidar_detection_model") or "").startswith("centerpoint")
    out = {k for k in aw if cp(pilots[k].get("args"))}
    import yaml
    for f in sorted((Path(project) / "suites").glob("*.yaml")) if project else []:
        try:
            pa = (yaml.safe_load(f.read_text()) or {}).get("pilot_args") or {}
        except (OSError, yaml.YAMLError, AttributeError):
            continue
        out |= {k for k, a in pa.items() if k in aw and cp(a)} if isinstance(pa, dict) else set()
    return sorted(out)


def tensorrt_ok(lib) -> bool:
    return bool(lib) and any(Path(os.path.expanduser(str(lib))).glob("libnvinfer.so.10*"))


def tensorrt_note(project, settings: dict, host: dict) -> str:
    """A warning when a CenterPoint pilot will run on a TensorRT other than Autoware's."""
    users = centerpoint_pilots(project, settings)
    lib = (host.get("autoware") or {}).get("tensorrt_lib")
    if not users or tensorrt_ok(lib):
        return ""
    why = f"autoware.tensorrt_lib {lib} has no libnvinfer.so.10" if lib else \
        "autoware.tensorrt_lib is not set"
    return (f"the pilot{'s' if len(users) > 1 else ''} {', '.join(users)} run CenterPoint and "
            f"{why}: its engines are rebuilt at every Autoware start (minutes; each run with "
            f"lifetime: run). Fix: {TENSORRT_FIX}")

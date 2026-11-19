"""``scensim setup`` and ``scensim autoware-side build`` (roadmap 089).

One command from a ``pip install`` to a host that runs Tutorial 1: the release installed, the
system packages and the DDS socket buffers behind one sudo prompt, ``~/scensim.bash`` and
``host.yaml`` written from what is detected, the car's Autoware packages built against the
host's Autoware, then a check that names each fix.

No ROS import and no YAML library needed: the front end runs before any ROS is sourced.
Every step says what it does before it does it, is skipped when already done, and never
overwrites a file of the user's without ``--force``.
"""

from __future__ import annotations

import difflib
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

from .release import Refused, Release, home, read_release_yaml, sim_prefix

# The CycloneDDS socket buffers (config/99-cyclonedds-max.conf; a test keeps the two equal)
SYSCTL_NAME = "99-cyclonedds-max.conf"
SYSCTL_TEXT = """\
# CycloneDDS kernel network buffers for single-host runs with several Autoware stacks.
# Values and reasons follow the Autoware DDS settings page and the golf-cart project's
# configure-cyclonedds-sysctl.sh: rmem_max is the ceiling Cyclone's receive
# buffers may reach; the 16 MB defaults are what a socket gets without asking;
# a 1.4 MB PointCloud2 fragments into ~22 datagrams and does not fit the 208 kB
# stock buffers, which shows up as UDP RcvbufErrors and stalled publishers.
# Numbered 99- so it wins over lower-numbered files (the ZED SDK ships a 60-).
net.core.rmem_max=2147483647
net.core.rmem_default=16777216
net.core.wmem_max=16777216
net.core.wmem_default=16777216
net.core.netdev_max_backlog=8192
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
"""
# what rosdep, the Autoware side's import and its build need
APT_TOOLS = ["python3-rosdep", "python3-vcstool", "python3-colcon-common-extensions",
             "python3-pip", "git"]
# the car's Autoware packages, and the profile (built always: seconds, CMake only)
AUTOWARE_SIDE_PACKAGES = ["tesla_model3_launch", "tesla_model3_sensor_kit_launch",
                          "individual_params", "autoware_sim_profile"]
AUTOWARE_SIDE_URL = "https://github.com/NEWSLabNTU/autoware_sim_vehicle.git"
AUTOWARE_SIDE_PIN = "src/autoware_sim_vehicle"
BUILT_MARK = ".scensim-built"

SETUP_USAGE = """\
usage: scensim setup [--no-sudo] [--force] [--autoware <setup.bash>|none] [--check]
       scensim autoware-side build [--autoware <setup.bash>] [--workspace <dir>] [--force]
"""


# --- where things are (each movable by the environment, for the tests) ------------------------


def ros_root() -> Path:
    return Path(os.environ.get("SCENSIM_ROS_ROOT", "/opt/ros"))


def autoware_root() -> Path:
    return Path(os.environ.get("SCENSIM_AUTOWARE_ROOT", "/opt/autoware"))


def etc() -> Path:
    return Path(os.environ.get("SCENSIM_ETC", "/etc"))


def proc_sys() -> Path:
    return Path(os.environ.get("SCENSIM_PROC_SYS", "/proc/sys"))


def config_dir() -> Path:
    return Path(os.environ.get("SCENSIM_CONFIG_HOME") or Path.home() / ".config" / "scensim")


def env_file() -> Path:
    return Path.home() / "scensim.bash"


def default_workspace() -> Path:
    return Path.home() / "autoware_side"


def say(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


# --- detection -------------------------------------------------------------------------------


def detect_autoware(arg: str | None) -> Path | None:
    """The Autoware's ``setup.bash``: ``--autoware``, else the one the terminal sourced (a
    prefix under the Autoware root on ``AMENT_PREFIX_PATH``), else the only one installed.
    ``none``, or nothing installed: None (a simulation-only host)."""
    if arg == "none":
        return None
    if arg:
        p = Path(arg).expanduser()
        if p.is_dir():
            p = p / "setup.bash"
        if not p.is_file():
            raise Refused(f"--autoware {arg}: no such setup.bash")
        return p.resolve()
    root = autoware_root()
    for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        try:
            rel = Path(entry).relative_to(root)
        except ValueError:
            continue
        if rel.parts and (root / rel.parts[0] / "setup.bash").is_file():
            return root / rel.parts[0] / "setup.bash"
    found = sorted(root.glob("*/setup.bash"))
    if len(found) > 1:
        raise Refused(
            "several Autowares are installed and none is sourced: "
            + ", ".join(str(f) for f in found)
            + "; name one with --autoware <setup.bash> (or --autoware none)")
    return found[0] if found else None


def carla_root(host_yaml: dict) -> Path | None:
    """CARLA's install: host.yaml's ``carla.root``, else a ``~/CARLA_*`` with CarlaUE4.sh."""
    root = (host_yaml.get("carla") or {}).get("root") if isinstance(host_yaml, dict) else None
    if root:
        return Path(str(root)).expanduser()
    for d in sorted(Path.home().glob("CARLA_*")):
        if (d / "CarlaUE4.sh").is_file():
            return d
    return None


def on_spinning_disk(path: Path) -> bool:
    """``path``'s block device is rotational (a partition reads its disk's flag)."""
    try:
        st = os.stat(path)
        dev = Path(f"/sys/dev/block/{os.major(st.st_dev)}:{os.minor(st.st_dev)}").resolve()
        for d in (dev, dev.parent):
            f = d / "queue" / "rotational"
            if f.is_file():
                return f.read_text().strip() == "1"
    except OSError:
        pass
    return False


def read_host_yaml(path: Path) -> dict | None:
    """The user's host.yaml as a dict; {} when absent; None when it cannot be read here
    (no YAML library, or not YAML): setup then leaves it alone."""
    if not path.is_file():
        return {}
    try:
        import yaml
    except ImportError:
        return None
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError:
        return None
    return data if isinstance(data, dict) else None


@dataclass
class Host:
    distro: str
    ros_setup: Path
    autoware: Path | None
    carla: Path | None
    host_yaml: dict | None
    spinning_home: bool = False

    @property
    def autoware_release(self) -> str | None:
        return self.autoware.parent.name if self.autoware else None


def detect(release: Release, autoware_arg: str | None) -> Host:
    ros = ros_root() / release.distro / "setup.bash"
    if not ros.is_file():
        raise Refused(
            f"no ROS 2 {release.distro} at {ros.parent}: install it first "
            f"(https://docs.ros.org/en/{release.distro}/Installation.html)")
    hy = read_host_yaml(config_dir() / "host.yaml")
    return Host(distro=release.distro, ros_setup=ros, autoware=detect_autoware(autoware_arg),
                carla=carla_root(hy or {}), host_yaml=hy,
                spinning_home=on_spinning_disk(Path.home()))


# --- the system step --------------------------------------------------------------------------


def apt_missing(pkgs: list[str]) -> list[str]:
    out = []
    for p in pkgs:
        r = subprocess.run(["dpkg-query", "-W", "-f=${Status}", p],
                           capture_output=True, text=True)
        if r.returncode != 0 or "install ok installed" not in r.stdout:
            out.append(p)
    return out


def rosdep_initialised() -> bool:
    d = etc() / "ros" / "rosdep" / "sources.list.d"
    return d.is_dir() and any(d.iterdir())


def rosdep_satisfied(share: Path, distro: str) -> bool:
    """Every rosdep key of the release is installed (``rosdep check``, after an update as the
    user). False when rosdep cannot say (not installed, not initialised)."""
    if not shutil.which("rosdep") or not rosdep_initialised():
        return False
    say("rosdep update (as you)")
    if subprocess.run(["rosdep", "update"], stdout=subprocess.DEVNULL).returncode != 0:
        return False
    # --rosdistro: setup runs before any ROS is sourced, and sudo drops ROS_DISTRO
    r = subprocess.run(["rosdep", "check", "--rosdistro", distro, "--from-paths", str(share),
                        "--ignore-src"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


def sysctl_wanted() -> dict[str, str]:
    return dict(line.split("=", 1) for line in SYSCTL_TEXT.splitlines()
                if line and not line.startswith("#"))


def sysctl_short() -> list[str]:
    """The keys whose running value falls short of the file's (ipfrag_time: differs)."""
    out = []
    for key, want in sysctl_wanted().items():
        f = proc_sys() / key.replace(".", "/")
        try:
            have = int(f.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            out.append(key)
            continue
        if (have != int(want)) if key.endswith("ipfrag_time") else (have < int(want)):
            out.append(key)
    return out


@dataclass
class SystemStep:
    lines: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def script(self) -> str:
        return "set -e\n" + "\n".join(self.lines) + "\n"


def system_step(release: Release, force: bool) -> SystemStep:
    """What needs root, each line only when not done yet."""
    step = SystemStep()
    share = home() / "current" / "sim" / "share"
    missing = apt_missing(APT_TOOLS)
    if missing:
        step.lines.append("apt-get update -qq")
        step.lines.append("DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "
                          + " ".join(missing))
    initialised = rosdep_initialised()
    if not initialised:
        step.lines.append("rosdep init")
    if missing or not initialised or not rosdep_satisfied(share, release.distro):
        step.lines.append('sudo -u "$SUDO_USER" -H rosdep update')
        # Python 3.11 and later is externally managed (PEP 668): see `scensim install`
        pep668 = ("PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1 "
                  if sys.version_info >= (3, 11) else "")
        # root's rosdep reads the cache the user's update wrote (sudo resets HOME)
        step.lines.append(f'ROS_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)/.ros '
                          f"{pep668}rosdep install --rosdistro {release.distro} "
                          f"--from-paths {shlex.quote(str(share))} "
                          f"--ignore-src -y")
    conf = etc() / "sysctl.d" / SYSCTL_NAME
    have = conf.read_text() if conf.is_file() else None
    if have is not None and have != SYSCTL_TEXT and not force:
        step.notes.append(f"{conf} differs from scensim's; kept (--force replaces it)")
    elif have != SYSCTL_TEXT:
        step.lines.append(f"mkdir -p {conf.parent}")
        step.lines.append(f"cat > {conf} <<'SCENSIM_SYSCTL'\n{SYSCTL_TEXT}SCENSIM_SYSCTL")
        step.lines.append(f"sysctl -q -p {conf}")
    elif sysctl_short():
        step.lines.append(f"sysctl -q -p {conf}")
    return step


def run_system_step(step: SystemStep, no_sudo: bool) -> bool:
    """Run the script under one sudo, or print it. True when nothing is left to do."""
    for n in step.notes:
        say(f"note: {n}")
    if not step.lines:
        say("system: nothing to do")
        return True
    text = step.script()
    if no_sudo:
        say("system: not run (--no-sudo); as an administrator, run:")
        say("  sudo bash -c " + shlex.quote(text))
        return False
    say("system: as root, one password prompt:")
    for line in step.lines:
        say("  " + line.splitlines()[0] + (" ..." if "\n" in line else ""))
    r = subprocess.run(["sudo", "bash", "-c", text])
    if r.returncode != 0:
        raise Refused(f"the system step failed (exit {r.returncode}); its output is above")
    return True


# --- brotli (issue 050) ----------------------------------------------------------------------


def _version(name: str) -> tuple:
    import importlib.metadata
    try:
        text = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return ()
    out = []
    for part in text.split("."):
        digits = "".join(c for c in part if c.isdigit()) if part[:1].isdigit() else ""
        if not digits:
            break
        out.append(int(digits))
    return tuple(out)


def brotli_short() -> bool:
    """huggingface_hub 2.x with a brotli older than 1.2 (Ubuntu's): the Hub's downloads fail
    to decode, and `testbench check` names it as a problem of the host (scensim.assets)."""
    hf = _version("huggingface_hub")
    br = _version("brotli") or _version("Brotli")
    return bool(hf) and hf[:1] >= (2,) and bool(br) and br < (1, 2)


def fix_brotli() -> None:
    """``pip install --user 'brotli>=1.2'`` for this Python: no root, ahead of apt's."""
    if not brotli_short():
        return
    say("brotli: Ubuntu's is older than huggingface_hub 2 needs; pip install --user 'brotli>=1.2'")
    pep668 = ["--break-system-packages"] if sys.version_info >= (3, 11) else []
    r = subprocess.run([sys.executable, "-m", "pip", "install", "-q", "--user", *pep668,
                        "brotli>=1.2"])
    if r.returncode != 0:
        say("brotli: pip failed; `scensim assets fetch` falls back to plain HTTPS")


# --- the user's files ------------------------------------------------------------------------


def env_text(host: Host, release: Release, autoware_side: Path | None) -> str:
    lines = [f"# {env_file()}: written by `scensim setup` (scensim {release.version}).",
             "# Source it in every terminal, before a workspace of your own.",
             f"source {shlex.quote(str(host.ros_setup))}"]
    if host.autoware:
        lines.append(f"source {shlex.quote(str(host.autoware))}")
    lines.append(f"source {shlex.quote(str(home() / 'current' / 'sim' / 'setup.bash'))}")
    if autoware_side:
        lines.append(f"source {shlex.quote(str(autoware_side / 'install' / 'setup.bash'))}")
    return "\n".join(lines) + "\n"


def write_kept(path: Path, text: str, force: bool) -> str:
    """Write ``path`` unless it holds something else and ``force`` is off; then print the
    difference. Returns what happened: written, unchanged, kept."""
    if path.is_file():
        old = path.read_text()
        if old == text:
            say(f"{path}: unchanged")
            return "unchanged"
        if not force:
            say(f"{path}: kept, it differs from what setup writes (--force replaces it):")
            for line in difflib.unified_diff(old.splitlines(), text.splitlines(),
                                             "yours", "setup's", lineterm="", n=0):
                say("  " + line)
            return "kept"
        backup = path.with_name(path.name + ".bak")
        shutil.copy2(path, backup)
        say(f"{path}: replaced (the old one is {backup})")
    else:
        say(f"{path}: written")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return "written"


def host_yaml_keys(host: Host) -> dict:
    keys: dict = {}
    if host.carla:
        keys.setdefault("carla", {})["root"] = str(host.carla)
    if host.autoware:
        keys.setdefault("autoware", {})["setup"] = str(host.autoware)
    return keys


def lacking(have: dict, want: dict, prefix: str = "") -> list[tuple[str, object]]:
    out = []
    for k, v in want.items():
        if isinstance(v, dict):
            sub = have.get(k) if isinstance(have.get(k), dict) else {}
            out += lacking(sub, v, f"{prefix}{k}.")
        elif have.get(k) != v:
            out.append((f"{prefix}{k}", v))
    return out


def write_host_yaml(host: Host, force: bool) -> None:
    path = config_dir() / "host.yaml"
    want = host_yaml_keys(host)
    if not want:
        say(f"{path}: nothing detected to write")
        return
    if host.host_yaml is None:
        say(f"{path}: not read (no YAML library, or not YAML); left as it is")
        return
    gaps = lacking(host.host_yaml, want)
    if not gaps:
        say(f"{path}: has what setup detected")
        return
    if path.is_file() and not force:
        say(f"{path}: kept; setup detected (add them, or --force merges them in):")
        for k, v in gaps:
            say(f"  {k}: {v}")
        return
    import yaml  # read_host_yaml returned a dict, so it is there
    merged = dict(host.host_yaml)
    for k, v in want.items():
        merged[k] = {**(merged.get(k) or {}), **v} if isinstance(v, dict) else v
    if path.is_file():
        shutil.copy2(path, path.with_name("host.yaml.bak"))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# written by `scensim setup`; every key: docs/reference/host-settings.md\n"
                    + yaml.safe_dump(merged, sort_keys=False))
    say(f"{path}: written ({', '.join(k for k, _ in gaps)})")


# --- the Autoware side -----------------------------------------------------------------------


def release_commits(prefix: Path) -> dict[str, str]:
    """``scensim-release.yaml``'s ``commits:`` block."""
    out: dict[str, str] = {}
    f = prefix / "scensim-release.yaml"
    if not f.is_file():
        return out
    inside = False
    for line in f.read_text().splitlines():
        if re.match(r"^commits:\s*$", line):
            inside = True
            continue
        if inside:
            m = re.match(r'^\s+"?([^":]+)"?:\s*"?([0-9a-f]{40})', line)
            if m:
                out[m.group(1)] = m.group(2)
            elif line and not line[0].isspace():
                break
    return out


def autoware_side_repos(release: Release) -> str:
    """The text ``scripts/release/autoware_side_repos.py`` writes, from the installed release's
    pin of autoware_sim_vehicle."""
    prefix = sim_prefix(release.version, release.distro)
    sha = release_commits(prefix).get(AUTOWARE_SIDE_PIN)
    if not sha:
        raise Refused(f"{prefix}/scensim-release.yaml names no {AUTOWARE_SIDE_PIN} commit; "
                      f"`scensim install --force`, or import autoware-side.repos by hand "
                      f"(docs/how-to/connect-autoware.md)")
    return ("# The car's Autoware packages for this scensim release: build them in your\n"
            "# Autoware workspace (docs/how-to/connect-autoware.md).\n"
            "repositories:\n"
            "  autoware_sim_vehicle:\n"
            "    type: git\n"
            f"    url: {AUTOWARE_SIDE_URL}\n"
            f"    version: {sha}\n")


def bash_in(sources: list[Path], command: str, cwd: Path | None = None) -> int:
    lines = [f"source {shlex.quote(str(s))}" for s in sources] + [command]
    return subprocess.run(["bash", "-c", " && ".join(lines)], cwd=cwd).returncode


def build_autoware_side(release: Release, ros: Path, autoware: Path, ws: Path,
                        force: bool) -> Path:
    """Import the release's Autoware side into ``ws`` and build it over ``autoware``; skipped
    when the same repos were built over the same Autoware."""
    repos = autoware_side_repos(release)
    mark_text = f"autoware: {autoware}\n{repos}"
    mark = ws / BUILT_MARK
    if not force and mark.is_file() and mark.read_text() == mark_text \
            and (ws / "install" / "setup.bash").is_file():
        say(f"autoware side: {ws} is built for this release over {autoware}")
        return ws
    src = ws / "src"
    if src.is_dir():
        others = sorted(p.name for p in src.iterdir() if p.name != "autoware_sim_vehicle")
        if others:
            raise Refused(f"{ws} holds other packages ({', '.join(others)}); name another "
                          f"workspace with --workspace")
    for tool in ("git", "vcs", "colcon"):
        if not shutil.which(tool):
            raise Refused(f"autoware side: {tool} is not installed (scensim setup installs "
                          f"it, or: sudo apt install git python3-vcstool "
                          f"python3-colcon-common-extensions)")
    src.mkdir(parents=True, exist_ok=True)
    (ws / "autoware-side.repos").write_text(repos)
    say(f"autoware side: importing into {ws} (autoware-side.repos written there)")
    if subprocess.run(["vcs", "import", "--force", "--input", "autoware-side.repos", "src"],
                      cwd=ws).returncode != 0:
        raise Refused(f"autoware side: vcs import failed in {ws}; its output is above")
    say(f"autoware side: building over {autoware}: {' '.join(AUTOWARE_SIDE_PACKAGES)}")
    mark.unlink(missing_ok=True)
    rc = bash_in([ros, autoware], "colcon build --packages-up-to "
                 + " ".join(AUTOWARE_SIDE_PACKAGES), cwd=ws)
    if rc != 0:
        raise Refused(f"autoware side: colcon build failed in {ws} (exit {rc}); its output "
                      f"is above")
    mark.write_text(mark_text)
    return ws


def autoware_data(env: Path, autoware: Path) -> None:
    """``setup_autoware_data`` from the Autoware's packaged ``data``, when it has one."""
    data = autoware.parent / "data"
    if not data.is_dir():
        say(f"autoware data: {data} does not exist; skipped (the profile's data_path: "
            f"docs/how-to/connect-autoware.md)")
        return
    say(f"autoware data: ~/autoware_data mirrors {data}")
    if bash_in([env], f"ros2 run autoware_sim_profile setup_autoware_data "
                      f"{shlex.quote(str(data))}") != 0:
        say("autoware data: setup_autoware_data failed; its output is above")


# --- the check -------------------------------------------------------------------------------


def check(host: Host, release: Release) -> bool:
    env = env_file()
    rows: list[tuple[bool, str]] = []
    exe = sim_prefix(release.version, release.distro) / "bin" / "scensim"
    rows.append((exe.is_file(), f"release {release.version} for {release.distro}"
                 + ("" if exe.is_file() else ": run `scensim install`")))
    rows.append((env.is_file(), f"{env}" + ("" if env.is_file() else ": run `scensim setup`")))
    share = home() / "current" / "sim" / "share"
    ok = rosdep_satisfied(share, release.distro)
    rows.append((ok, "system packages (rosdep)" + ("" if ok else
                 ": run `scensim setup`, or its system step as an administrator")))
    short = sysctl_short()
    rows.append((not brotli_short(), "brotli for huggingface_hub" + ("" if not brotli_short()
                 else ": pip install --user 'brotli>=1.2'")))
    rows.append((not short, "DDS socket buffers" + ("" if not short else
                 f": {', '.join(short)} below scensim's; run `scensim setup`")))
    if env.is_file():
        ok = bash_in([env], f"{shlex.quote(str(exe))} seat show >/dev/null") == 0
        rows.append((ok, "scensim answers in ~/scensim.bash's environment"
                     + ("" if ok else ": `source ~/scensim.bash; scensim seat show` says why")))
        if host.autoware:
            for pkg in AUTOWARE_SIDE_PACKAGES:
                ok = bash_in([env], f"ros2 pkg prefix {pkg} >/dev/null 2>&1") == 0
                rows.append((ok, f"the Autoware side's {pkg}" + ("" if ok else
                             ": run `scensim autoware-side build`")))
    if host.carla and (host.carla / "CarlaUE4.sh").is_file():
        rows.append((True, f"CARLA at {host.carla}"))
    else:
        say("note: no CARLA found; `scensim carla fetch 0.9.16`, or set carla.root, or "
            "serve one on carla.host:carla.port")
    if host.autoware_release == "1.5.0":
        say("note: Autoware 1.5.0 rebuilds CenterPoint's engines at every start without "
            "TensorRT 10.8.0: `scensim autoware tensorrt`")
    if host.spinning_home:
        say("note: your home is on a spinning disk; set `logs:` in host.yaml to a fast one")
    for ok, text in rows:
        say(f"  {'ok  ' if ok else 'FIX '} {text}")
    return all(ok for ok, _ in rows)


# --- the commands ----------------------------------------------------------------------------


def parse(args: list[str], allowed: set[str]) -> dict:
    out: dict = {"no_sudo": False, "force": False, "check": False, "autoware": None,
                 "workspace": None}
    it = iter(args)
    for a in it:
        key, _, val = a.partition("=")
        name = key.lstrip("-").replace("-", "_")
        if not a.startswith("--") or name not in allowed:
            raise Refused(f"unknown argument {a!r}\n{SETUP_USAGE}")
        if name in ("autoware", "workspace"):
            out[name] = val or next(it, None)
            if not out[name]:
                raise Refused(f"{key} needs a value\n{SETUP_USAGE}")
        else:
            out[name] = True
    return out


def setup(release: Release, args: list[str], install) -> int:
    o = parse(args, {"no_sudo", "force", "autoware", "check"})
    host = detect(release, o["autoware"])
    say(f"scensim setup: ROS {host.distro}, "
        + (f"Autoware {host.autoware_release} ({host.autoware})" if host.autoware
           else "no Autoware (simulation only)")
        + (f", CARLA at {host.carla}" if host.carla else ""))
    if o["check"]:
        return 0 if check(host, release) else 3
    install(release, force=False, first_use=False, quiet_next_steps=True)
    done = run_system_step(system_step(release, o["force"]), o["no_sudo"])
    fix_brotli()
    ws = None
    if host.autoware:
        ws = build_autoware_side(release, host.ros_setup, host.autoware, default_workspace(),
                                 o["force"])
    write_kept(env_file(), env_text(host, release, ws), o["force"])
    write_host_yaml(host, o["force"])
    if host.autoware and ws:
        autoware_data(env_file(), host.autoware)
    ok = check(host, release)
    if ok and done:
        say("ready: `source ~/scensim.bash` in every terminal, then Tutorial 1")
        return 0
    return 3


def autoware_side(release: Release, args: list[str]) -> int:
    if not args or args[0] != "build":
        raise Refused(f"scensim autoware-side: only `build`\n{SETUP_USAGE}")
    o = parse(args[1:], {"autoware", "workspace", "force"})
    autoware = detect_autoware(o["autoware"])
    if not autoware:
        raise Refused("scensim autoware-side build: no Autoware found; source it, or "
                      "--autoware <setup.bash>")
    ros = ros_root() / release.distro / "setup.bash"
    ws = Path(o["workspace"]).expanduser() if o["workspace"] else default_workspace()
    build_autoware_side(release, ros, autoware, ws, o["force"])
    say(f"built: source {ws / 'install' / 'setup.bash'} after scensim's (~/scensim.bash "
        f"does when `scensim setup` wrote it)")
    return 0

"""Isaac Sim on this host: the Autoware message bindings its Python needs (roadmap 077, G25).

The Isaac world server publishes Autoware's messages with Isaac's own rclpy (Python 3.12), so
each Autoware message package it uses is rebuilt for that Python into ``<isaac root>/msgs``
by aib's ``build_msgs.sh`` (installed as ``share/aib/scripts/build_msgs.sh``). A package
missing there, or built before the Autoware install changed, used to surface only mid-run, as
a failed frame. ``problems`` finds both before a stack starts; ``project check`` and the
stack report them, and ``ros2 run swb isaac msgs`` builds what is missing.

The packages are the build script's own default list: one source for both.
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

from . import config
from .findings import Finding

FIX = "ros2 run swb isaac msgs"
SCRIPT = Path("share") / "aib" / "scripts" / "build_msgs.sh"
_DEFAULT = re.compile(r"\|\|\s*PKGS=\(([^)]*)\)")
_TYPESUPPORT = ("generator_c", "typesupport_c", "typesupport_fastrtps_c",
                "typesupport_introspection_c")


def build_script(prefixes=None):
    """aib's ``build_msgs.sh`` in the first ament prefix that has it, else None."""
    if prefixes is None:
        prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
    for p in prefixes:
        f = Path(p) / SCRIPT
        if f.is_file():
            return f
    return None


def required_packages(script: Path) -> list:
    """The packages ``build_msgs.sh`` builds when given none."""
    m = _DEFAULT.search(Path(script).read_text())
    if not m:
        raise ValueError(f"{script}: no default package list (`|| PKGS=(...)`)")
    return m.group(1).split()


def root(host: dict) -> str:
    """Isaac's install: the host setting ``isaac.root``, else the ISAAC_ROOT the world
    server inherits; "" when neither is set (the world server's own default)."""
    r = str((host.get("isaac") or {}).get("root") or "") or os.environ.get("ISAAC_ROOT", "")
    return os.path.expanduser(r)


def autoware_prefix(host: dict) -> Path:
    """The Autoware install the bindings are built from: its setup.bash's directory."""
    setup = str((host.get("autoware") or {}).get("setup") or "")
    return Path(os.path.expanduser(setup)).parent


def _modules(d: Path) -> set:
    return {f"{k}/{p.name}" for k in ("msg", "srv", "action")
            for p in (d / k).glob("_*.py")} if d.is_dir() else set()


def problems(isaac_root, prefix, packages) -> list:
    """``[(package, what)]``: each package not built in ``<isaac_root>/msgs``, or built from
    another Autoware than ``prefix`` (a message or service it lacks, or a library older than
    the install's)."""
    out = []
    msgs = Path(isaac_root) / "msgs"
    src_root = Path(prefix) / "local" / "lib" / "python3.10" / "dist-packages"
    for pkg in packages:
        built = msgs / pkg
        if not (built / "__init__.py").is_file() or \
                not (built / f"lib{pkg}__rosidl_generator_py.so").is_file():
            out.append((pkg, "is not built"))
            continue
        lacking = sorted(_modules(src_root / pkg) - _modules(built))
        if lacking:
            names = ", ".join(m.split("/")[0] + "/" + m.split("/")[1][1:-3] for m in lacking)
            out.append((pkg, f"is stale: lacks {names}"))
            continue
        for ts in _TYPESUPPORT:
            lib = f"lib{pkg}__rosidl_{ts}.so"
            have, want = msgs / "lib" / lib, Path(prefix) / "lib" / lib
            if not have.is_file():
                out.append((pkg, f"is not built ({lib} missing)"))
                break
            if want.is_file() and want.stat().st_mtime > have.stat().st_mtime:
                out.append((pkg, f"is stale: {lib} is older than the Autoware install's"))
                break
    return out


def findings(host: dict, prefixes=None) -> list:
    """``project check``'s findings for an Isaac project on this host."""
    r = root(host)
    if not r:
        return []
    if not (Path(r) / "venv" / "bin" / "python").is_file():
        return [Finding("host settings", f"isaac.root {r} is not an Isaac Sim install "
                        "(no venv/bin/python)", "set isaac.root in ~/.config/swb/host.yaml",
                        refused=True)]
    script = build_script(prefixes)
    if script is None:
        return [Finding("host settings", "aib is not installed (no share/aib/scripts/"
                        "build_msgs.sh on AMENT_PREFIX_PATH), so the Isaac message bindings "
                        "cannot be checked", "source the workspace's install/setup.bash")]
    out = []
    for pkg, what in problems(r, autoware_prefix(host), required_packages(script)):
        out.append(Finding("host settings", f"Isaac message bindings: {pkg} {what} in "
                           f"{Path(r) / 'msgs'} (the Isaac world server needs it)", FIX,
                           refused=True))
    return out


def build(host: dict, packages=None, prefixes=None) -> int:
    """Run ``build_msgs.sh`` for ``packages`` (default: the ones ``problems`` names)."""
    r = root(host)
    if not r:
        print("swb isaac: isaac.root is not set (~/.config/swb/host.yaml)", file=sys.stderr)
        return 2
    script = build_script(prefixes)
    if script is None:
        print("swb isaac: no share/aib/scripts/build_msgs.sh on AMENT_PREFIX_PATH; source the "
              "workspace's install/setup.bash", file=sys.stderr)
        return 2
    prefix = autoware_prefix(host)
    if packages is None:
        packages = [p for p, _ in problems(r, prefix, required_packages(script))]
        if not packages:
            print(f"swb isaac: every message package is built in {Path(r) / 'msgs'}")
            return 0
    env = dict(os.environ, ISAAC_ROOT=r, AUTOWARE_PREFIX=str(prefix))
    return subprocess.call(["bash", str(script), *packages], env=env)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ros2 run swb isaac",
                                 description="Isaac Sim on this host.")
    ap.add_argument("--project", help="the project (default: the working directory's)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("msgs", help="build the Autoware message bindings Isaac's Python lacks")
    p.add_argument("packages", nargs="*", help="rebuild these (default: the missing or stale)")
    p.add_argument("--check", action="store_true", help="only list what is missing or stale")
    args = ap.parse_args(argv)
    try:
        project = config.find_project(Path(args.project) if args.project else None)
    except config.ConfigError:
        project = None
    host = config.host_settings(project)
    if args.check:
        found = findings(host)
        for f in found:
            print(f)
        return 1 if found else 0
    return build(host, args.packages or None)


if __name__ == "__main__":
    sys.exit(main())

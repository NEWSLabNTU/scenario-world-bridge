"""``scensim``: the pip front end.

``scensim --version``, ``install``, ``setup``, ``autoware-side`` and ``carla`` are its own; every other command is scensim's,
from the prebuilt release for this host's ROS 2, which the first such command downloads
(saying so, with the size) and then runs.

Exit codes follow scensim's: 0 success, 2 a download or unpack that failed (run again), 3
refused (the message names the fix).
"""

from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

from . import __version__
from .fetch import DownloadError, download, human, point_current, unpack
from .release import (
    Refused,
    Release,
    choose_distro,
    home,
    install_dir,
    load_release,
    read_release_yaml,
    same_version,
    CONNECT_AUTOWARE,
    sim_prefix,
)

ROS_ROOT = os.environ.get("SCENSIM_ROS_ROOT", "/opt/ros")

USAGE = """\
usage: scensim install [--distro <ros>] [--force]
       scensim setup [--no-sudo] [--force] [--autoware <setup.bash>|none] [--check]
       scensim autoware-side build [--autoware <setup.bash>] [--workspace <dir>] [--force]
       scensim carla fetch <version> [--dir <parent>]
       scensim --version
       scensim <command> ...     (scensim's own commands: run, runs, testbench, assets, ...)

The first command that needs scensim downloads the prebuilt release for your ROS 2.
"""


def installed(version: str, distro: str) -> bool:
    """The simulation side of ``version`` for ``distro`` is unpacked."""
    return same_version(read_release_yaml(sim_prefix(version, distro)).get("version"), version)


def install(release: Release, force: bool, first_use: bool,
            quiet_next_steps: bool = False) -> None:
    """Download and unpack the simulation side unless it is in place; point ``current`` at
    it; print the next steps. A sourced Autoware changes nothing here: the Autoware-side
    packages are built in the user's Autoware workspace, never downloaded."""
    v, d = release.version, release.distro
    art = release.sim_artefact()
    prefix = sim_prefix(v, d)
    if force or not same_version(read_release_yaml(prefix).get("version"), v):
        lead = "first use: downloading" if first_use else "downloading"
        print(f"{lead} scensim {v} for ROS 2 {d}, {human(int(art['size']))}", file=sys.stderr)
        archive = download(art, home() / ".downloads")
        print(f"  sha256 ok; unpacking to {prefix}", file=sys.stderr)
        unpack(archive, prefix, v, d)
        archive.unlink(missing_ok=True)
    point_current(install_dir(v, d))
    cur = home() / "current"
    print(f"installed in {install_dir(v, d)} (current)", file=sys.stderr)
    if quiet_next_steps:  # `scensim setup` does them
        return
    print("next: `scensim setup` does the rest (or, by hand:)", file=sys.stderr)
    # rosdep update first: an old cache misses keys (python3-yamale-pip) and install exits 1
    print("missing system packages: run once, the second as an administrator:", file=sys.stderr)
    print("  rosdep update", file=sys.stderr)
    # Python 3.11 and later is "externally managed" (PEP 668, Ubuntu 24.04): rosdep's pip
    # keys need pip allowed next to apt's packages, and installed over (not uninstalling)
    # the Debian-owned dependencies they want newer (typing_extensions, idna)
    pep668 = "PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1 " if sys.version_info >= (3, 11) else ""
    print(f"  sudo {pep668}rosdep install --from-paths {cur}/sim/share --ignore-src -y",
          file=sys.stderr)
    print("then, in every terminal (after ROS, and your Autoware if you use one):", file=sys.stderr)
    print(f"  source {cur}/sim/setup.bash", file=sys.stderr)
    print(f"an Autoware pilot needs the car's Autoware packages in your Autoware workspace: "
          f"{CONNECT_AUTOWARE}", file=sys.stderr)


def environment_lines(release: Release) -> list[str]:
    """What to source before the release's ``bin/scensim``, in order: ROS (when no ROS is
    sourced), then the simulation side unless it is already on ``AMENT_PREFIX_PATH``. A
    sourced Autoware and its workspace stay as the terminal has them."""
    v, d = release.version, release.distro
    sourced = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    lines: list[str] = []
    ros = Path(ROS_ROOT) / d / "setup.bash"
    if not os.environ.get("ROS_DISTRO") and ros.exists():
        lines.append(f"source {shlex.quote(str(ros))}")
    sim = sim_prefix(v, d)
    if str(sim) not in sourced:
        lines.append(f"source {shlex.quote(str(sim / 'setup.bash'))}")
    return lines


def exec_scensim(release: Release, argv: list[str]) -> None:
    """Run the release's own ``bin/scensim`` with ``argv`` unchanged, in its environment; its
    exit code is the command's (the process is replaced)."""
    exe = sim_prefix(release.version, release.distro) / "bin" / "scensim"
    if not exe.exists():
        raise DownloadError(f"{exe} is missing from the install; run `scensim install --force`")
    lines = environment_lines(release)
    if not lines:
        os.execv(str(exe), [str(exe), *argv])
    lines.append(f'exec {shlex.quote(str(exe))} "$@"')
    os.execv("/bin/bash", ["/bin/bash", "-c", " && ".join(lines), "scensim", *argv])


def version_text() -> str:
    """``--version``: the front end, and what is installed. Never downloads."""
    out = [f"scensim {__version__} (pip front end, {sys.executable})"]
    try:
        distro = choose_distro(None)
        release = load_release(distro)
    except Refused as e:
        out.append(str(e))
        return "\n".join(out)
    if installed(release.version, distro):
        out.append(f"installed for ROS {distro}: {install_dir(release.version, distro)}")
    else:
        out.append(f"not installed for ROS {distro}: run `scensim install`")
    return "\n".join(out)


def parse_install(args: list[str]) -> tuple[str | None, bool]:
    distro = None
    force = False
    it = iter(args)
    for a in it:
        if a == "--distro":
            distro = next(it, None)
        elif a.startswith("--distro="):
            distro = a.split("=", 1)[1]
        elif a == "--autoware" or a.startswith("--autoware="):
            raise Refused(
                "scensim install: --autoware was removed in 0.2.0: a release downloads the "
                "simulation side only, and the car's Autoware packages are built in your "
                f"Autoware workspace: {CONNECT_AUTOWARE}"
            )
        elif a == "--force":
            force = True
        else:
            raise Refused(f"scensim install: unknown argument {a!r}\n{USAGE}")
    return distro, force


def main(argv: list[str] | None = None) -> int:
    """The front end: the console script, and the callable the release's own ``scensim``
    forwards ``--version``, ``-V`` and ``install`` to once its setup.bash is sourced (it is
    first on PATH then). A stable interface: returns the exit code (0, 2 download, 3 refused,
    130), and for those three never execs the release, or the two would loop. Every other
    command execs the release; so does ``--help``, after the front end's usage."""
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        if not args or args[0] in ("-h", "--help"):
            print(USAGE, end="")
            # and scensim's own commands, from the installed release (never a download)
            try:
                release = load_release(choose_distro(None))
            except Refused:
                return 0
            if installed(release.version, release.distro):
                sys.stdout.flush()
                print()
                exec_scensim(release, ["--help"])
            return 0
        if args[0] in ("--version", "-V"):
            print(version_text())
            return 0
        if args[0] == "carla":
            from .carla import fetch
            return fetch(args[1:])
        if args[0] in ("setup", "autoware-side"):
            from . import setup as setup_mod
            release = load_release(choose_distro(None))
            if args[0] == "setup":
                return setup_mod.setup(release, args[1:], install)
            if not installed(release.version, release.distro):
                install(release, force=False, first_use=True)
            return setup_mod.autoware_side(release, args[1:])
        if args[0] == "install":
            distro, force = parse_install(args[1:])
            release = load_release(choose_distro(distro))
            install(release, force, first_use=False)
            return 0
        release = load_release(choose_distro(None))
        if not installed(release.version, release.distro):
            install(release, force=False, first_use=True)
        exec_scensim(release, args)
        return 0  # not reached: exec replaces the process
    except Refused as e:
        print(f"scensim: {e}", file=sys.stderr)
        return 3
    except DownloadError as e:
        print(f"scensim: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())

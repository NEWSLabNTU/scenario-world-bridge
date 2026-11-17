"""``scensim``: the pip front end.

``scensim --version`` and ``scensim install`` are its own; every other command is scensim's,
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
    sim_prefix,
    sourced_autoware,
    vehicle_prefix,
)

ROS_ROOT = os.environ.get("SCENSIM_ROS_ROOT", "/opt/ros")
AUTOWARE_ROOT = os.environ.get("SCENSIM_AUTOWARE_ROOT", "/opt/autoware")

USAGE = """\
usage: scensim install [--distro <ros>] [--autoware <release>] [--force]
       scensim --version
       scensim <command> ...     (scensim's own commands: run, runs, testbench, assets, ...)

The first command that needs scensim downloads the prebuilt release for your ROS 2.
"""


def installed(version: str, distro: str) -> bool:
    return same_version(read_release_yaml(sim_prefix(version, distro)).get("version"), version)


def install(release: Release, autoware: str | None, force: bool, first_use: bool) -> None:
    """Download and unpack what is missing; point ``current``; print the next steps."""
    v, d = release.version, release.distro
    sides: list[tuple[str, str | None]] = [("sim", None)]
    if autoware:
        sides.append(("vehicle", autoware))
    downloads = home() / ".downloads"
    for side, aw in sides:
        prefix = sim_prefix(v, d) if side == "sim" else vehicle_prefix(v, d, aw or "")
        if not force and same_version(read_release_yaml(prefix).get("version"), v):
            continue
        art = release.artefact(side, aw)
        what = "scensim" if side == "sim" else f"scensim's vehicle side for Autoware {aw}"
        lead = "first use: downloading" if first_use else "downloading"
        print(f"{lead} {what} {v} for ROS 2 {d}, {human(int(art['size']))}", file=sys.stderr)
        archive = download(art, downloads)
        print(f"  sha256 ok; unpacking to {prefix}", file=sys.stderr)
        unpack(archive, prefix, v, d)
        archive.unlink(missing_ok=True)
    point_current(install_dir(v, d))
    cur = home() / "current"
    print(f"installed in {install_dir(v, d)} (current)", file=sys.stderr)
    print("missing system packages: run once, as an administrator:", file=sys.stderr)
    paths = f"{cur}/sim/share" + (f" {cur}/vehicle-autoware{autoware}/share" if autoware else "")
    print(f"  sudo rosdep install --from-paths {paths} --ignore-src -y", file=sys.stderr)
    print("then, in every terminal (after ROS and your Autoware):", file=sys.stderr)
    print(f"  source {cur}/sim/setup.bash", file=sys.stderr)
    if autoware:
        print(f"  source {cur}/vehicle-autoware{autoware}/setup.bash", file=sys.stderr)


def autoware_setup(version: str, distro: str) -> tuple[str | None, Path | None]:
    """The Autoware to source before the vehicle side: ``(release, its setup.bash)``.

    A sourced Autoware is taken as it is (``setup`` None). Otherwise, an installed vehicle side
    for release R is paired with ``/opt/autoware/R/setup.bash`` when that exists."""
    sourced = sourced_autoware()
    if sourced:
        return sourced, None
    root = install_dir(version, distro)
    for veh in sorted(root.glob("vehicle-autoware*")):
        rel = veh.name.removeprefix("vehicle-autoware")
        setup = Path(AUTOWARE_ROOT) / rel / "setup.bash"
        if (veh / "setup.bash").exists() and setup.exists():
            return rel, setup
    return None, None


def environment_lines(release: Release) -> list[str]:
    """What to source before the release's ``bin/scensim``, in order: ROS (when no ROS is
    sourced), Autoware (when one is detected and not sourced), the simulation side, the
    vehicle side for that Autoware. A prefix already on ``AMENT_PREFIX_PATH`` is not sourced
    again."""
    v, d = release.version, release.distro
    sourced = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    lines: list[str] = []
    ros = Path(ROS_ROOT) / d / "setup.bash"
    if not os.environ.get("ROS_DISTRO") and ros.exists():
        lines.append(f"source {shlex.quote(str(ros))}")
    aw, aw_setup = autoware_setup(v, d)
    if aw_setup is not None:
        lines.append(f"source {shlex.quote(str(aw_setup))}")
    sim = sim_prefix(v, d)
    if str(sim) not in sourced:
        lines.append(f"source {shlex.quote(str(sim / 'setup.bash'))}")
    if aw:
        veh = vehicle_prefix(v, d, aw)
        if (veh / "setup.bash").exists() and str(veh) not in sourced:
            lines.append(f"source {shlex.quote(str(veh / 'setup.bash'))}")
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


def parse_install(args: list[str]) -> tuple[str | None, str | None, bool]:
    distro = autoware = None
    force = False
    it = iter(args)
    for a in it:
        if a == "--distro":
            distro = next(it, None)
        elif a.startswith("--distro="):
            distro = a.split("=", 1)[1]
        elif a == "--autoware":
            autoware = next(it, None)
        elif a.startswith("--autoware="):
            autoware = a.split("=", 1)[1]
        elif a == "--force":
            force = True
        else:
            raise Refused(f"scensim install: unknown argument {a!r}\n{USAGE}")
    return distro, autoware, force


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
        if args[0] == "install":
            distro, autoware, force = parse_install(args[1:])
            release = load_release(choose_distro(distro))
            install(release, autoware or sourced_autoware(), force, first_use=False)
            return 0
        release = load_release(choose_distro(None))
        if not installed(release.version, release.distro):
            install(release, sourced_autoware(), force=False, first_use=True)
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

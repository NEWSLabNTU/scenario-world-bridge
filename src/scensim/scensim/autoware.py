"""``scensim autoware overlay|tensorrt``: the two pieces an Autoware pilot runs better with.

``overlay`` builds the Autoware packages this workspace carries with fixes Autoware 1.5.0
lacks (roadmap 042: the blind_spot right turn, NDT's step reversal, pose_instability_detector's
negative time) into the directory an autoware pilot sources them from: the host settings'
``autoware.overlay``, else ``<workspace>/third_party/autoware_overlay``. Without it a stack
runs stock Autoware and says so.

``tensorrt`` fetches the TensorRT runtime Autoware 1.5.0 was compiled against (10.8.0) into
``autoware.tensorrt_lib``, else ``<workspace>/third_party/tensorrt/lib``, so CenterPoint's
cached engines load instead of being rebuilt at every start.

Both run this workspace's own scripts (``scripts/autoware_overlay.bash``,
``scripts/autoware_tensorrt.bash``), as the developer recipes ``just build-autoware-overlay``
and ``just setup-tensorrt`` do; neither needs ``just``.
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

from scensim import config

DEFAULT_SETUP = "/opt/autoware/1.5.0/setup.bash"
# autoware_sim_profile's autoware_release choices (its check_launch_args); an autoware pilot
# runs the release of the host's Autoware (autoware.setup), never another
PROFILE_RELEASES = ("1.5.0", "2.0.0", "2.1.0")
RELEASE = re.compile(r"^\d+\.\d+\.\d+$")


def release_of(setup) -> str:
    """The Autoware release a setup.bash belongs to: the nearest version-named directory above
    it (/opt/autoware/2.1.0/setup.bash -> 2.1.0), or None (an Autoware in the ROS prefix)."""
    for part in reversed(Path(os.path.expanduser(str(setup))).parent.parts):
        if RELEASE.match(part):
            return part
    return None


def host_setup(host: dict) -> str:
    return str((host.get("autoware") or {}).get("setup") or DEFAULT_SETUP)


# colcon's setup.bash names each underlay the build had sourced, in order
CHAINED = re.compile(r'^COLCON_CURRENT_PREFIX="([^"$`]+)"', re.M)


def chained_release(setup_bash) -> str:
    """The Autoware release a workspace was built against: the first underlay of its
    ``install/setup.bash`` with a version-named directory (/opt/autoware/1.5.0), or None."""
    try:
        text = Path(setup_bash).read_text(errors="replace")
    except OSError:
        return None
    for prefix in CHAINED.findall(text):
        release = release_of(Path(prefix) / "setup.bash")
        if release:
            return release
    return None


def install_problems(host: dict, workspace) -> list:
    """The workspaces an autoware pilot sources (acb's, then this one's) that were built
    against another Autoware than autoware.setup's. Their setup.bash sources that Autoware
    again, ahead of autoware.setup's, so the pilot would run a mix of the two releases."""
    setup = host_setup(host)
    want = release_of(setup)
    if workspace is None or want is None:
        return []
    ws = Path(workspace)
    out = []
    for f in (ws / "src" / "autoware_carla_bridge" / "install" / "setup.bash",
              ws / "install" / "setup.bash"):
        got = chained_release(f)
        if got and got != want:
            out.append(f"{f} was built against Autoware {got} and sources it, but autoware.setup "
                       f"is Autoware {want} ({setup}); the pilot would run {got}'s packages "
                       f"ahead of {want}'s: rebuild with AUTOWARE_SETUP={setup} (acb's `just "
                       f"build` first, then this workspace's), or point autoware.setup at "
                       f"Autoware {got}")
    return out


def pilot_release(host: dict, where: str, args: dict) -> dict:
    """An autoware pilot's arguments with ``autoware_release`` set from the host's Autoware
    (autoware.setup): a pilot naming another release is refused by name, as is a release the
    profile does not have. Unchanged when the setup's path names no release."""
    setup = host_setup(host)
    release = release_of(setup)
    given = args.get("autoware_release")
    if release is None:
        return args
    if release not in PROFILE_RELEASES:
        raise ValueError(f"the host's Autoware is {release} ({setup}); autoware_sim_profile "
                         f"runs {', '.join(PROFILE_RELEASES)}")
    if given is not None and str(given) != release:
        raise ValueError(f"{where}: autoware_release:={given}, but the host's Autoware is "
                         f"{release} ({setup}); leave autoware_release out (scensim sets it "
                         f"from autoware.setup) or point autoware.setup at Autoware {given}")
    return {**args, "autoware_release": release}


def workspace():
    from scensim import stack
    return stack.workspace_root()


def dirs(host: dict, ws: Path) -> dict:
    """Where the overlay and the TensorRT runtime go: the host settings', else the
    workspace's defaults (the ones an autoware pilot's environment reads)."""
    a = host.get("autoware") or {}
    exp = lambda v: Path(os.path.expanduser(str(v))) if v else None  # noqa: E731
    return {"setup": exp(a.get("setup")) or Path(DEFAULT_SETUP),
            "overlay": exp(a.get("overlay")) or ws / "third_party" / "autoware_overlay",
            "tensorrt": exp(a.get("tensorrt_lib")) or ws / "third_party" / "tensorrt" / "lib"}


def command(what: str, ws: Path, d: dict) -> list:
    """The bash command that builds the overlay or fetches the runtime."""
    if what == "overlay":
        body = (f'source "{ws}/scripts/autoware_overlay.bash" && '
                f'scensim_autoware_overlay_build "{d["overlay"]}" "{d["setup"]}"')
    else:
        body = (f'source "{ws}/scripts/autoware_tensorrt.bash" && '
                f'scensim_tensorrt_setup "{d["tensorrt"]}"')
    return ["bash", "-c", body]


def clean_env() -> dict:
    """The build's environment without a Python venv on PATH (as the recipe strips it)."""
    env = dict(os.environ)
    env["PATH"] = ":".join(p for p in env.get("PATH", "").split(":")
                           if not p.endswith("/.venv/bin"))
    return env


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="scensim autoware", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="what", required=True)
    sub.add_parser("overlay", help="build Autoware's carried fixes")
    sub.add_parser("tensorrt", help="fetch the TensorRT 10.8.0 runtime Autoware 1.5.0 needs")
    sub.add_parser("where", help="print where each goes on this host")
    a = ap.parse_args(argv)
    ws = workspace()
    if ws is None:
        print("scensim autoware: the scensim checkout is not known (an installed scensim outside its "
              "checkout); build from the checkout: just build-autoware-overlay", file=sys.stderr)
        return 3
    try:
        host = config.host_settings()
    except config.ConfigError as e:
        print(f"scensim autoware: {e}", file=sys.stderr)
        return 3
    d = dirs(host, ws)
    if a.what == "where":
        for k in ("setup", "overlay", "tensorrt"):
            print(f"{k:9} {d[k]}")
        return 0
    if not d["setup"].is_file():
        print(f"scensim autoware: no Autoware at {d['setup']} (the host setting autoware.setup)",
              file=sys.stderr)
        return 3
    print(f"[scensim autoware] {a.what} -> {d['overlay' if a.what == 'overlay' else 'tensorrt']}",
          flush=True)
    return subprocess.run(command(a.what, ws, d), env=clean_env()).returncode


if __name__ == "__main__":
    sys.exit(main())

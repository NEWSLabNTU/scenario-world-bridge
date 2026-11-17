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
import subprocess
import sys
from pathlib import Path

from scensim import config

DEFAULT_SETUP = "/opt/autoware/1.5.0/setup.bash"


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
    sub.add_parser("overlay", help="build Autoware's carried fixes (roadmap 042)")
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

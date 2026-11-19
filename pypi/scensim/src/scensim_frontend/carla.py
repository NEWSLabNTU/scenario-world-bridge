"""``scensim carla fetch <version>`` (roadmap 089): CARLA's own release, checked and unpacked.

CARLA publishes no checksums, so the front end pins each tarball's size and sha256 as scensim
measured them. Only the CARLA this release's binaries serve is offered.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from .fetch import DownloadError, download, human
from .release import Refused
from . import setup as setup_mod

# version -> CARLA's tarball (no top directory: CarlaUE4.sh at its root)
CARLA = {
    "0.9.16": {
        # the link of CARLA's GitHub release page (github.com/carla-simulator/carla/releases)
        "url": "https://downloads.carlasim.com/Linux/CARLA_0.9.16.tar.gz",
        "size": 8346095504,
        "sha256": "09e3ebb28df17962f0c997e66f4b914ad5ea6f1d6a6dbbf13c9f87eb38346d57",
    },
}
USAGE = "usage: scensim carla fetch <version> [--dir <parent>]   (versions: " + \
    ", ".join(CARLA) + ")\n"


def say(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def parse(args: list[str]) -> tuple[str, Path]:
    if not args or args[0] != "fetch":
        raise Refused(f"scensim carla: only `fetch`\n{USAGE}")
    version, parent = None, Path.home()
    it = iter(args[1:])
    for a in it:
        if a == "--dir" or a.startswith("--dir="):
            v = a.partition("=")[2] or next(it, None)
            if not v:
                raise Refused(f"--dir needs a directory\n{USAGE}")
            parent = Path(v).expanduser()
        elif not a.startswith("-") and version is None:
            version = a
        else:
            raise Refused(f"unknown argument {a!r}\n{USAGE}")
    if version not in CARLA:
        raise Refused(f"no CARLA {version!r} to fetch\n{USAGE}")
    return version, parent


def set_root(dest: Path) -> None:
    """Write host.yaml with ``carla.root`` when there is none; an existing host.yaml is the
    user's, so its ``carla.root`` line is printed instead."""
    path = setup_mod.config_dir() / "host.yaml"
    have = setup_mod.read_host_yaml(path)
    root = ((have or {}).get("carla") or {}).get("root") if isinstance(have, dict) else None
    if root and Path(str(root)).expanduser() == dest:
        say(f"{path}: carla.root is {dest} already")
    elif path.exists():
        say(f"{path}: kept; to use this CARLA set\n  carla: {{root: {dest}}}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# written by `scensim carla fetch`; every key: "
                        "docs/reference/host-settings.md\n"
                        f"carla:\n  root: {dest}\n")
        say(f"{path}: written (carla.root: {dest})")


def fetch(args: list[str]) -> int:
    version, parent = parse(args)
    art = CARLA[version]
    dest = parent / f"CARLA_{version}"
    if (dest / "CarlaUE4.sh").is_file():
        say(f"CARLA {version} is at {dest} already")
    else:
        if dest.exists() and any(dest.iterdir()):
            raise Refused(f"{dest} exists and holds no CarlaUE4.sh; move it away first")
        say(f"downloading CARLA {version}, {human(int(art['size']))}, to {parent}")
        try:
            archive = download(art, parent / ".scensim-downloads", resume=50)
        except DownloadError as e:
            raise DownloadError(
                f"{e}\nor download it yourself from {art['url']} (CARLA's release page: "
                f"https://github.com/carla-simulator/carla/releases/tag/{version}), unpack it "
                f"into {dest}, and run `scensim carla fetch {version}` again to set carla.root"
            ) from None
        say(f"  sha256 ok; unpacking to {dest}")
        dest.mkdir(parents=True, exist_ok=True)
        r = subprocess.run(["tar", "-xzf", str(archive), "-C", str(dest)])
        if r.returncode != 0:
            raise DownloadError(f"unpacking {archive} failed (exit {r.returncode}); "
                                f"remove {dest} and run again")
        archive.unlink(missing_ok=True)
        try:
            archive.parent.rmdir()
        except OSError:
            pass
    set_root(dest)
    return 0

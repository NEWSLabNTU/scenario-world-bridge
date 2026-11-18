#!/usr/bin/env python3
"""Write scensim-ros-<distro>'s manifest.json from a release's built artefacts.

    make_manifest.py --version 0.1.0 --distro humble --base-url <release download URL> \
        scensim-sim-0.1.0-humble-x86_64.tar.zst

The artefact name is the release build's (scripts/release):
scensim-sim-<ver>-<distro>-<arch>.tar.zst. A release has no other artefact: the car's Autoware
packages are source, built in the user's Autoware workspace. Size and sha256 come from the
file itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any


def pattern(version: str) -> re.Pattern[str]:
    """The artefact name for ``version`` (which may hold dashes, e.g. 0.0.0-dev)."""
    v = re.escape(version)
    return re.compile(rf"scensim-sim-v?{v}-(?P<distro>[a-z]+)-(?P<arch>[a-z0-9_]+)\.tar\.zst$")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True)
    ap.add_argument("--distro", required=True)
    ap.add_argument("--base-url", required=True)
    ap.add_argument("--out", default=None, help="default: the release package's manifest.json")
    ap.add_argument("files", nargs="+", type=Path)
    a = ap.parse_args()
    arts: dict[str, dict[str, Any]] = {}
    SIM = pattern(a.version.removeprefix("v"))
    for f in a.files:
        sha = hashlib.sha256()
        with open(f, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                sha.update(chunk)
        entry = {
            "url": f"{a.base_url.rstrip('/')}/{f.name}",
            "size": f.stat().st_size,
            "sha256": sha.hexdigest(),
        }
        if m := SIM.match(f.name):
            if m["distro"] != a.distro:
                raise SystemExit(f"{f.name} is for {m['distro']}, not {a.distro}")
            arts.setdefault(m["arch"], {})["sim"] = entry
        elif f.name.startswith("scensim-vehicle-"):
            raise SystemExit(
                f"{f.name}: a release has no vehicle tarball since 0.2.0; the car's Autoware "
                f"packages are source, built in the user's Autoware workspace"
            )
        else:
            raise SystemExit(
                f"{f.name}: not a scensim {a.version} release artefact name "
                f"(scensim-sim-<ver>-<distro>-<arch>.tar.zst)"
            )
    out = (
        Path(a.out)
        if a.out
        else Path(__file__).parent
        / f"scensim-ros-{a.distro}"
        / "src"
        / f"scensim_ros_{a.distro}"
        / "manifest.json"
    )
    manifest = {"version": a.version, "ros_distro": a.distro, "artefacts": arts}
    out.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

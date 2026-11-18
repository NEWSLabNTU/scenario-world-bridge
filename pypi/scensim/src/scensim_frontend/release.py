"""Which prebuilt scensim this host takes, and where it is installed.

No ROS import here: the front end runs before any ROS environment is sourced.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import os
import platform
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import __version__

# The Python each ROS 2 release runs on with apt (one Ubuntu LTS each).
PYTHON_TO_DISTRO = {(3, 10): "humble", (3, 12): "jazzy"}
SOURCE_BUILD = (
    "https://github.com/NEWSLabNTU/scensim/blob/main/docs/getting-started/install.md#from-source"
)


class Refused(Exception):
    """A refusal the user can fix: printed as given, exit code 3."""


@dataclass
class Release:
    distro: str
    version: str
    manifest: dict[str, Any]

    def artefact(self, side: str, autoware: str | None = None) -> dict[str, Any]:
        arch = machine()
        by_arch = self.manifest.get("artefacts", {}).get(arch)
        if not by_arch:
            raise Refused(
                f"no prebuilt scensim {self.version} for {arch}; build from source: {SOURCE_BUILD}"
            )
        if side == "sim":
            art = by_arch.get("sim")
        else:
            art = by_arch.get("vehicle", {}).get(f"autoware{autoware}")
            if art is None:
                have = ", ".join(sorted(by_arch.get("vehicle", {}))) or "none"
                raise Refused(
                    f"no prebuilt vehicle side for Autoware {autoware} in scensim {self.version} "
                    f"(this release has: {have}); build the vehicle packages from source in your "
                    f"workspace"
                )
        if (
            not art
            or not art.get("size")
            or not re.fullmatch(r"[0-9a-f]{64}", art.get("sha256", ""))
        ):
            raise Refused(
                f"the release package scensim-ros-{self.distro} {self.version} names no usable "
                f"{side} artefact (a development placeholder?); install a released version"
            )
        return art


def machine() -> str:
    return {"amd64": "x86_64", "arm64": "aarch64"}.get(platform.machine(), platform.machine())


def python_distro() -> str | None:
    return PYTHON_TO_DISTRO.get(sys.version_info[:2])


def installed_release_packages() -> list[str]:
    """The ROS releases whose ``scensim-ros-<distro>`` is installed for this Python."""
    out = []
    for dist in importlib.metadata.distributions():
        name = (dist.metadata["Name"] or "").lower().replace("_", "-")
        if name.startswith("scensim-ros-"):
            out.append(name.removeprefix("scensim-ros-"))
    return sorted(set(out))


def choose_distro(requested: str | None) -> str:
    """The ROS release to serve: ``--distro``, else ``$ROS_DISTRO``, else the one this Python's
    release package is for. Refuses a ``$ROS_DISTRO`` that disagrees with ``--distro``."""
    env = os.environ.get("ROS_DISTRO") or None
    if requested and env and requested != env:
        raise Refused(
            f"--distro {requested} but the terminal sourced ROS {env}: source ROS {requested}, "
            f"or drop --distro"
        )
    if requested or env:
        return requested or env  # type: ignore[return-value]
    have = installed_release_packages()
    if len(have) == 1:
        return have[0]
    guess = python_distro()
    if guess:
        return guess
    raise Refused(
        f"no prebuilt scensim for Python {sys.version_info[0]}.{sys.version_info[1]} "
        f"({sys.executable}); apt's ROS 2 runs on 3.10 (Humble) or 3.12 (Jazzy). Install with "
        f"that Python (/usr/bin/python3 -m pip install --user scensim), or build from source: "
        f"{SOURCE_BUILD}"
    )


def load_release(distro: str) -> Release:
    """The release package for ``distro``, with the guards that name the fix."""
    module = f"scensim_ros_{distro}"
    try:
        mod = importlib.import_module(module)
    except ImportError:
        have = installed_release_packages()
        if have:
            raise Refused(
                f"scensim is installed for {', '.join(have)}, but ROS_DISTRO is {distro} "
                f"(python {sys.executable}): source the ROS the install is for, or "
                f"`pip install scensim-ros-{distro}` when it exists"
            ) from None
        raise Refused(
            f"no prebuilt scensim for ROS {distro} yet; build from source: {SOURCE_BUILD}"
        ) from None
    manifest = mod.manifest()
    version = str(manifest.get("version", ""))
    if not same_version(version, __version__):
        raise Refused(
            f"version skew: the scensim front end is {__version__}, scensim-ros-{distro} is "
            f"{version}; `pip install --user --upgrade scensim scensim-ros-{distro}`"
        )
    return Release(distro=distro, version=version, manifest=manifest)


# --- where it lives ---------------------------------------------------------------------------


def home() -> Path:
    env = os.environ.get("SCENSIM_HOME")
    if env:
        return Path(env).expanduser()
    data = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local" / "share")
    return Path(data) / "scensim"


def install_dir(version: str, distro: str) -> Path:
    return home() / f"{version}-{distro}"


def sim_prefix(version: str, distro: str) -> Path:
    return install_dir(version, distro) / "sim"


def vehicle_prefix(version: str, distro: str, autoware: str) -> Path:
    return install_dir(version, distro) / f"vehicle-autoware{autoware}"


def sourced_autoware() -> str | None:
    """The release of an Autoware the terminal sourced (``/opt/autoware/<rel>`` on a prefix
    path), or None."""
    for var in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"):
        m = re.search(r"/autoware/(\d+\.\d+\.\d+)(?:/|$|:)", os.environ.get(var, ""))
        if m:
            return m.group(1)
    return None


# A version as PEP 440 reads it, in the spellings a release and a wheel use: the release's
# (its tag and manifest: 0.1.0-rc1, 0.0.0-dev) and the wheel's normalised one (0.1.0rc1,
# 0.0.0.dev0). No dependency on `packaging`: the front end keeps to the standard library.
_VERSION = re.compile(
    r"^v?(?P<release>\d+(?:\.\d+)*)"
    r"(?:[-_.]?(?P<pre>a|b|c|rc|alpha|beta|pre|preview)[-_.]?(?P<pre_n>\d*))?"
    r"(?:[-_.]?(?P<post>post|rev|r)[-_.]?(?P<post_n>\d*))?"
    r"(?:[-_.]?(?P<dev>dev)[-_.]?(?P<dev_n>\d*))?$",
    re.IGNORECASE,
)
_PRE = {
    "a": "a",
    "alpha": "a",
    "b": "b",
    "beta": "b",
    "c": "rc",
    "rc": "rc",
    "pre": "rc",
    "preview": "rc",
}


def normal_version(text: str) -> tuple | None:
    """``text`` as PEP 440 normalises it, comparable with ``==``; None when it is not one.
    ``0.1.0-rc1`` and ``0.1.0rc1`` are the same, and so are ``0.0.0-dev`` and ``0.0.0.dev0``
    and ``0.1`` and ``0.1.0``."""
    m = _VERSION.match(str(text).strip())
    if not m:
        return None
    release = [int(x) for x in m["release"].split(".")]
    while len(release) > 1 and release[-1] == 0:
        release.pop()
    pre = (_PRE[m["pre"].lower()], int(m["pre_n"] or 0)) if m["pre"] else None
    post = int(m["post_n"] or 0) if m["post"] else None
    dev = int(m["dev_n"] or 0) if m["dev"] else None
    return tuple(release), pre, post, dev


def same_version(a: str | None, b: str | None) -> bool:
    """The same version, by PEP 440: the release's spelling and the wheel's compare equal
    (``0.1.0-rc1`` is ``0.1.0rc1``, ``v0.1.0`` is ``0.1.0``). A string that is no version
    compares as written."""
    if not a or not b:
        return False
    na, nb = normal_version(a), normal_version(b)
    if na is None or nb is None:
        return str(a).removeprefix("v") == str(b).removeprefix("v")
    return na == nb


def read_release_yaml(prefix: Path) -> dict[str, str]:
    """``scensim-release.yaml``'s top-level scalar keys (no YAML library needed)."""
    out: dict[str, str] = {}
    f = prefix / "scensim-release.yaml"
    if not f.is_file():
        return out
    for line in f.read_text().splitlines():
        m = re.match(r"^([A-Za-z_][\w-]*):\s*(.*?)\s*$", line)
        if m and m.group(2) and not m.group(2).startswith(("{", "[", "|", ">")):
            out[m.group(1)] = m.group(2).strip("'\"")
    return out

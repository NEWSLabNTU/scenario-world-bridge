"""Download an artefact, check it, and unpack it into its prefix."""

from __future__ import annotations

import hashlib
import http.client
import os
import shutil
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path
from typing import Any

from .release import Refused, read_release_yaml, same_version

CHUNK = 1 << 20


class DownloadError(Exception):
    """A download or unpack that failed for a reason outside the user's setup: exit code 2."""


def human(n: int) -> str:
    return f"{n / 1e6:.1f} MB"


def download(art: dict[str, Any], dest_dir: Path, quiet: bool = False) -> Path:
    """Download ``art`` (url, size, sha256) to ``dest_dir``; a short or wrong file is removed."""
    url, size, sha = art["url"], int(art["size"]), art["sha256"]
    dest_dir.mkdir(parents=True, exist_ok=True)
    name = url.rsplit("/", 1)[-1]
    part = dest_dir / (name + ".part")
    digest = hashlib.sha256()
    got = 0
    next_report = size // 10
    try:
        with urllib.request.urlopen(url, timeout=60) as resp, open(part, "wb") as out:
            while True:
                chunk = resp.read(CHUNK)
                if not chunk:
                    break
                out.write(chunk)
                digest.update(chunk)
                got += len(chunk)
                if not quiet and size and got >= next_report:
                    print(f"  {human(got)} of {human(size)}", file=sys.stderr)
                    next_report += size // 10
    except (OSError, http.client.HTTPException) as e:
        part.unlink(missing_ok=True)
        raise DownloadError(
            f"download of {url} failed ({type(e).__name__}: {e}); the partial file is removed, "
            f"run the command again"
        ) from e
    if got != size:
        part.unlink(missing_ok=True)
        raise DownloadError(
            f"download cut: {got} of {size} bytes from {url}; the partial file is removed, "
            f"run the command again"
        )
    if digest.hexdigest() != sha:
        part.unlink(missing_ok=True)
        raise DownloadError(
            f"sha256 mismatch for {name}: expected {sha}, got {digest.hexdigest()}; the file is "
            f"removed, run the command again"
        )
    final = dest_dir / name
    os.replace(part, final)
    return final


def unpack(archive: Path, prefix: Path, version: str, distro: str) -> None:
    """Unpack a ``.tar.zst`` into ``prefix``, atomically, after checking its release file."""
    import zstandard

    prefix.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=f".{prefix.name}-", dir=prefix.parent))
    try:
        with open(archive, "rb") as fh:
            reader = zstandard.ZstdDecompressor().stream_reader(fh)
            with tarfile.open(fileobj=reader, mode="r|") as tar:
                if hasattr(tarfile, "data_filter"):
                    tar.extractall(tmp, filter="tar")
                else:  # pragma: no cover (Python without PEP 706)
                    tar.extractall(tmp)
        root = tmp
        entries = [p for p in tmp.iterdir()]
        if (
            not (tmp / "scensim-release.yaml").exists()
            and len(entries) == 1
            and entries[0].is_dir()
        ):
            root = entries[0]  # an archive with one top directory
        rel = read_release_yaml(root)
        if not rel:
            raise DownloadError(
                f"{archive.name} holds no scensim-release.yaml; not a scensim release"
            )
        if not same_version(rel.get("version"), version):
            raise Refused(
                f"version skew: {archive.name} is scensim {rel.get('version')}, the front end "
                f"expects {version}"
            )
        if rel.get("ros_distro") and rel["ros_distro"] != distro:
            raise Refused(f"{archive.name} is for ROS {rel['ros_distro']}, not {distro}")
        if prefix.exists():
            shutil.rmtree(prefix)
        os.replace(root, prefix)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def point_current(target: Path) -> None:
    """``<home>/current`` -> ``target``, replaced atomically."""
    link = target.parent / "current"
    tmp = target.parent / ".current.new"
    tmp.unlink(missing_ok=True)
    tmp.symlink_to(target.name)
    os.replace(tmp, link)

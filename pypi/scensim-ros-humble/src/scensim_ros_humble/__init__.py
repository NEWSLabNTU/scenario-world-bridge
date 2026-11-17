"""The manifest of scensim's prebuilt release for ROS 2 Humble.

It holds no binaries: only where each artefact is (a GitHub Release URL), its size and its
sha256, written by the release workflow from the artefacts it built
(``pypi/make_manifest.py``). The ``scensim`` front end reads it through ``manifest()``.
"""

from __future__ import annotations

import json
from importlib import resources
from typing import Any

ROS_DISTRO = "humble"


def manifest() -> dict[str, Any]:
    """The manifest: ``version``, ``ros_distro`` and ``artefacts[arch]`` (``sim``, and
    ``vehicle`` by Autoware release), each ``{url, size, sha256}``."""
    text = resources.files(__name__).joinpath("manifest.json").read_text()
    return json.loads(text)

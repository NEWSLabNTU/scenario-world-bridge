"""The manifest of scensim's prebuilt release for ROS 2 Jazzy.

It holds no binaries: only where each artefact is (a GitHub Release URL), its size and its
sha256, written by the release workflow from the artefacts it built
(``pypi/make_manifest.py``). The ``scensim`` front end reads it through ``manifest()``.
"""

from __future__ import annotations

import json
from importlib import resources
from typing import Any

ROS_DISTRO = "jazzy"


def manifest() -> dict[str, Any]:
    """The manifest: ``version``, ``ros_distro`` and ``artefacts[arch]`` (``sim`` only: the
    vehicle side carries Autoware 1.5.0's packages, which are built for ROS 2 Humble only),
    each ``{url, size, sha256}``."""
    text = resources.files(__name__).joinpath("manifest.json").read_text()
    return json.loads(text)

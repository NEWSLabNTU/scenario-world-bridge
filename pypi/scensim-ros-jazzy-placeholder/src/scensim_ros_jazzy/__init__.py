"""A placeholder for scensim 0.1.0 on ROS 2 Jazzy, which 0.1.0 has no release for.

scensim 0.1.0's front end requires this package on Python 3.12 (Ubuntu 24.04, Jazzy's
Python), so `pip install scensim` succeeds there; every command then stops here with the
reason. The front end reads the release through ``manifest()`` and reports a ``Refused`` as
"scensim: <message>", exit 3, so that is what this raises.
"""

from __future__ import annotations

from typing import Any

ROS_DISTRO = "jazzy"

MESSAGE = (
    "scensim 0.1.0 has no ROS 2 Jazzy release: Jazzy arrives in 0.2.0. "
    "0.1.0 needs Ubuntu 22.04 and ROS 2 Humble (Python 3.10): on such a host, "
    "`/usr/bin/python3 -m pip install --user scensim`. "
    "Or build from source: https://github.com/NEWSLabNTU/scensim/blob/main/docs/getting-started/install.md#from-source"
)


def manifest() -> dict[str, Any]:
    """Refuses: there is no Jazzy release to describe."""
    try:
        from scensim_frontend.release import Refused
    except ImportError:  # pragma: no cover (installed without the front end)
        raise RuntimeError(MESSAGE) from None
    raise Refused(MESSAGE)

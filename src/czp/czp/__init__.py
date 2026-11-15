"""czp: the conflict-zone protocol (roadmap 078, docs/design/conflict-zone-protocol.md).

Pure Python; no ROS and no CARLA at library level. Steps 2-10 of the per-vehicle workflow:
relation (``relation.relate``), intervals (``intervals.intervals``), band (``band.band``), role
(``role.role``), convention (``convention.decide``), probe (``probe.probe``), act's latches
(``act.latch``), contracts (``contracts.check``), record (``record.record``); ``cycle.Protocol`` runs
them in order. Step 1 is the offline tool ``czp-zones`` (``czp.zones_tool``, needs lanelet2), the
ROS adapter the ``interaction_strategy`` node (``czp.node``, needs rclpy).
"""

from .config import load_contracts, load_conventions, load_params  # noqa: F401
from .cycle import Protocol  # noqa: F401
from .model import *  # noqa: F401,F403

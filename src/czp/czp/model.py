"""The data model of the conflict-zone protocol (docs/design/conflict-zone-protocol.md, "Data model").

Plain dataclasses, no ROS and no CARLA. Distances along a path are metres, times seconds, speeds
m/s. An agent's position relative to a zone is given along its own path: ``s_entry`` is the
distance from its front to the zone's near edge (positive before the zone), ``s_exit`` the
distance its rear still has to go to leave the zone (the front's distance to the far edge plus
the agent's length). Everything a step needs about the geometry is in those two numbers, so the
library runs unchanged on the arena's one-dimensional cars and on Autoware's tracked objects.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

INF = math.inf

# relation kinds (Relation.kind)
LEAD, CUT_IN, CROSS, MERGE, OPPOSE, ONCOMING_LEFT, CIRCULATING = (
    "lead", "cut_in", "cross", "merge", "oppose", "oncoming_left", "circulating")
RELATIONS = (LEAD, CUT_IN, CROSS, MERGE, OPPOSE, ONCOMING_LEFT, CIRCULATING)

# zone kinds: the convention table's rows (conventions/<jurisdiction>.yaml)
ALL_WAY_STOP, CROSS_UNSIGNALLED, MERGE_LANE_DROP, NARROW_PASSAGE, LEFT_TURN, ROUNDABOUT_ENTRY = (
    "all_way_stop", "cross_unsignalled", "merge_lane_drop", "narrow_passage", "left_turn", "roundabout_entry")
ZONE_KINDS = (ALL_WAY_STOP, CROSS_UNSIGNALLED, MERGE_LANE_DROP, NARROW_PASSAGE, LEFT_TURN, ROUNDABOUT_ENTRY,
              "cut_in")

# the law's priority of an approach (Approach.priority); lower ranks go first
PRIORITY_RANK = {"major": 0, "circulating": 0, "free": 0, "all_way_stop": 1, "equal": 1, "minor": 2,
                 "yield_to_circulating": 2, "obstructed": 2}

# band (Band.kind)
CLEAR, BAND, CONTESTED, AMBIGUOUS = "clear", "band", "contested", "ambiguous"

# role facts (Role.fact) and readings (Role.reading)
MAP_ORDER, MAP_PRIORITY, STOPPED_FIRST, AHEAD_AT_END, OBSTRUCTION_SIDE = (
    "map_order", "map_priority", "stopped_first", "ahead_at_end", "obstruction_side")
FIRST, SECOND, TIE, AMBIG, PENDING = "first", "second", "tie", "ambiguous", "pending"

# decisions (Decision.kind)
GO, YIELD, WAIT, WAIT_SECOND_ROUND, CREEP, STOCK = "go", "yield", "wait", "wait_second_round", "creep", "stock"
DECISIONS = (GO, YIELD, WAIT, WAIT_SECOND_ROUND, CREEP, STOCK)
STOPPING = (YIELD, WAIT, WAIT_SECOND_ROUND)


@dataclass(frozen=True)
class Approach:
    """One approach (an incoming lanelet) of a zone."""
    id: str  # the approach's id within the zone (a lanelet id on a map; "A", "B" in the arena)
    lanelet: Optional[int] = None
    heading: float = 0.0  # rad, the approach's heading at the stop line, ROS frame (x east, y north)
    order: int = 0  # the map's fixed order (czp-zones: clockwise from north); unique within the zone
    priority: str = "equal"  # PRIORITY_RANK's keys
    stop_line: Optional[tuple] = None  # ((x, y), (x, y)) or None
    entry_s: float = 0.0  # arc length along the lanelet's centreline where the zone starts
    exit_s: float = 0.0  # ... and ends
    width: float = 3.5  # m, the lane's width at the zone
    obstructed: bool = False  # narrow passage: an obstruction on this approach's side
    centerline: tuple = ()  # ((x, y), ...): the path through the zone, from upstream; entry_s / exit_s along it
    exit_lanelet: Optional[int] = None


@dataclass(frozen=True)
class Zone:
    id: str
    kind: str  # ZONE_KINDS
    approaches: tuple  # of Approach
    polygon: tuple = ()  # ((x, y), ...)
    lanelets: tuple = ()  # the lanelets that cross it
    exit_box: tuple = ()  # polygon of the space after the zone that must be clear
    shared_width: Optional[float] = None  # narrow passage: the free width shared by both directions

    def approach(self, aid):
        for a in self.approaches:
            if a.id == aid:
                return a
        raise KeyError(aid)


@dataclass(frozen=True)
class Capability:
    """An agent's capability bounds (params.yaml's classes, or the vehicle spec for the own car)."""
    a_max: float = 3.0  # m/s^2, the largest drive acceleration
    a_launch: float = 3.0  # m/s^2, from rest
    launch_dead: float = 0.5  # s from the first drive command at rest to motion (074: LAUNCH_DEAD)
    b_max: float = 7.0  # m/s^2, the largest braking
    b_bound: float = 4.0  # m/s^2, the braking the protocol assumes of it (the latest-arrival bound)
    b_comfort: float = 3.5  # m/s^2: a stop that needs more is not a planned stop
    length: float = 4.79
    width: float = 2.163
    v_max: float = 14.0  # m/s, the speed limit it is bounded by


@dataclass
class Agent:
    """One car as one cycle sees it: tracked (or the own car), relative to one zone."""
    id: str
    cls: str = "car"
    approach: Optional[str] = None  # the zone approach it is on (relate() sets it from the lanelet)
    s_entry: float = INF  # m, front to the zone's near edge along its path (< 0: front inside)
    s_exit: float = INF  # m, rear to the zone's far edge (<= 0: has left)
    v: float = 0.0
    a: float = 0.0
    stamp: float = 0.0  # when the state was true (the observation's time)
    lanelet: Optional[int] = None
    pose: Optional[tuple] = None  # (x, y, yaw) for path matching, optional
    footprint: tuple = ()
    covariance: Optional[tuple] = None
    sources: frozenset = frozenset({"lidar"})
    cap: Capability = field(default_factory=Capability)
    track_age: float = INF  # s the track has existed
    phantom: bool = False

    @property
    def in_zone(self):
        return self.s_entry <= 0.0 < self.s_exit + 1e-9 and self.s_exit > 0.0

    @property
    def cleared(self):
        return self.s_exit <= 0.0


@dataclass(frozen=True)
class Relation:
    agent: str
    zone: str
    kind: str


@dataclass(frozen=True)
class Interval:
    agent: str
    zone: str
    t_in: tuple  # (earliest, latest), s from now
    t_out: tuple
    reason: str = "track"  # track, phantom, uwb, own


@dataclass(frozen=True)
class Band:
    kind: str  # CLEAR, BAND, CONTESTED, AMBIGUOUS
    who: Optional[str] = None  # CLEAR: the agent that goes first
    slack_own: float = 0.0  # s: other's earliest entry minus own latest exit
    slack_other: float = 0.0  # s: own earliest entry minus the other's latest exit


@dataclass(frozen=True)
class Role:
    fact: str
    reading: str  # FIRST, SECOND, TIE, AMBIG, PENDING
    value: dict = field(default_factory=dict)  # the fact's value per agent
    confidence: float = 1.0

    @property
    def ambiguous(self):
        return self.reading == AMBIG


@dataclass(frozen=True)
class Decision:
    kind: str  # DECISIONS
    zone: Optional[str] = None
    reason: str = ""
    row: str = ""  # the convention row that decided
    against: tuple = ()  # the agents it yields or waits for
    latched_until: float = 0.0
    committed: bool = False
    stock_equivalent: bool = False  # what stock Autoware would do here anyway (R8): the ROS adapter issues nothing


@dataclass
class Record:
    """One line per decision change (step 10)."""
    t: float
    zone: Optional[str]
    conflict_set: list
    agents: dict  # per agent: relation, interval, band
    band: Optional[str]
    role: Optional[dict]
    row: str
    probe: Optional[dict]
    contracts: dict
    decision: str
    reason: str
    learned_share: float = 0.0  # the learned parameters' share of the margin (none learned yet: 0)

    def as_dict(self):
        return {"t": round(self.t, 3), "zone": self.zone, "conflict_set": self.conflict_set, "agents": self.agents,
                "band": self.band, "role": self.role, "row": self.row, "probe": self.probe,
                "contracts": self.contracts, "decision": self.decision, "reason": self.reason,
                "learned_share": self.learned_share}

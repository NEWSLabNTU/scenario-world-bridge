"""The ROS adapter's logic without ROS: from Autoware's inputs (as plain values) to the actions on
Autoware's interfaces. ``czp.node`` wraps it in rclpy; the cycle-time bench (EV3) runs it as is.

Per cycle: the zones near the own car whose approach it is on (and its planned path crosses, when a
trajectory is given), nearest first; per zone the objects placed by ``relate`` and one
``Protocol.step``; the nearest zone with a decision that is not stock's decides the actions:

- go (not stock-equivalent): the driven RTC module to manual mode, ACTIVATE its statuses;
- yield / wait / wait_second_round: manual mode, DEACTIVATE; with no RTC status of a driven module
  near (the module does not cover the zone), a velocity limit that still stops before the zone;
- creep (the probe): ACTIVATE with the creep speed as the velocity limit (probe:creep), DEACTIVATE
  (probe:watch);
- a stop held over a slow cycle (``hold: contract``, issue 075): as a wait -- manual mode, DEACTIVATE -- with the
  velocity limit that stops before the zone whether a module covers it or not; never auto mode;
- stock, or every zone clear: the modules back to auto mode and the velocity limit cleared -- and when
  nothing was ever changed, nothing is sent (EV2: no RTC command on a clear route);
- the mode hysteresis (078 P2b): a module this node holds goes back to auto mode only once the decision
  has been stock-equivalent (or stock) for ``mode_hold`` s. Meanwhile it stays manual and follows the
  module's own reading -- ACTIVATE when its status is safe, DEACTIVATE when not --, which is what auto
  mode would do, one cycle later. Readings of clear or contested at the band's edges, which used to
  toggle the mode at every flip (078 P2: 216 switches in 26 runs), now change only the command. A failed
  contract, or no status of the module near, releases at once.
- a dropped partner (078 P2c ruling): a car of the deciding zone's conflict set that leaves it without being
  seen to exit the zone (it was inside) or to turn away (it is still tracked and moving, placed off the
  zone's approaches) keeps the last decision for ``mode_hold``; it is not read as clear. A track that drops
  for a cycle at rest (P2c: the ego crept 1 m under the module's ``safe`` while the other car stood) no
  longer hands over to the module. A failed contract still releases at once.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace

from . import contracts as C
from .act import HOLD
from .cycle import Protocol
from .geom import segment_hits_poly
from .model import CREEP, GO, STOCK, STOPPING, Agent
from .relation import conflict_set, match_approach, place, relate

LABEL_CLASS = {1: "car", 2: "truck", 3: "bus", 4: "truck", 5: "two_wheeler", 6: "two_wheeler", 7: "pedestrian"}


@dataclass
class Obj:
    """One tracked object (from PredictedObjects): id, label, pose, speed along its heading, accel, size."""
    id: str
    label: int
    x: float
    y: float
    yaw: float
    v: float
    a: float = 0.0
    length: float = 4.5
    width: float = 1.9


@dataclass
class Status:
    """One RTC cooperate status: module name, uuid (hex), start distance."""
    module: str
    uuid: str
    start_distance: float = 0.0
    safe: bool = True
    auto_mode: bool = False  # the status's own auto_mode


@dataclass
class Actions:
    rtc: list = field(default_factory=list)  # (module, uuid, activate: bool)
    auto_mode: list = field(default_factory=list)  # (module, enable: bool)
    velocity_limit: object = None  # float m/s, "clear", or None (no change)
    records: list = field(default_factory=list)
    decision: object = None
    zone: object = None
    held_dropped: tuple = ()  # partners that dropped out of the conflict set unseen, holding the decision
    cs_held: dict = field(default_factory=dict)  # issue 079: zone id -> ids kept in its conflict set this cycle
    cs_events: list = field(default_factory=list)  # issue 079: dicts for decisions.jsonl when a zone's kept set changes


class Strategy:
    def __init__(self, zones, protocol=None, modules=("intersection",), horizon_m=120.0, own_class="car",
                 own_cap=None):
        self.zones = list(zones)
        self.proto = protocol or Protocol()
        self.p = self.proto.params
        self.modules = tuple(modules)
        self.horizon = horizon_m
        self.own_cap = own_cap or self.p.capability(own_class)
        self.centroid = {}
        self.poly_bbox = {}  # zone id -> the polygon's (min x, min y, max x, max y): the trajectory test's prefilter
        for z in self.zones:
            pts = list(z.polygon) or [p for a in z.approaches for p in a.centerline[:1]]
            self.centroid[z.id] = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
            if z.polygon:
                xs, ys = [p[0] for p in z.polygon], [p[1] for p in z.polygon]
                self.poly_bbox[z.id] = (min(xs), min(ys), max(xs), max(ys))
        self.first_seen = {}
        self.manual = set()  # modules this node put in manual mode
        self.sent = {}  # uuid -> last command sent
        self.limited = False
        self.equiv_since = None  # since when every decision has been stock or stock-equivalent (the mode hysteresis)
        self.last_chosen = None  # (decision, zone) of the last cycle with a decision that is not stock's
        self.partners = {}  # that zone's conflict set as last seen: id -> placed Agent
        self.drop_since = None  # since when a dropped partner holds the last decision
        self.dropped = ()  # the partners holding it (for the record)
        self.cs_last = {}  # issue 079: zone id -> {agent id: (t last in the conflict set, the agent as placed then)}
        self.cs_kept = {}  # issue 079: zone id -> the ids kept last cycle (for the record of changes)
        self.counts = {"rtc": 0, "auto_mode": 0, "velocity_limit": 0}

    def candidates(self, x, y, yaw, traj=None, cache=None, limit=None):
        """The zones ahead whose approach the car is on (and its trajectory crosses, when given), nearest first;
        the first ``limit`` only when given. Issue 075: the trajectory test was 95 % of a conflict cycle's cost
        (every zone on the approach against every segment, 30 zones x 59 segments at a Town05 junction); it now
        runs nearest first until ``limit`` zones pass, and only on the segments whose bounding box meets the
        zone polygon's (a segment that meets a polygon meets its bounding box: the result is the same)."""
        out = []
        for z in self.zones:
            cx, cy = self.centroid[z.id]
            if abs(cx - x) > self.horizon or abs(cy - y) > self.horizon:
                continue
            ap, s, _ = match_approach(z, x, y, yaw, cache, "ego")
            if ap is None:
                continue
            s_entry = ap.entry_s - (s + self.own_cap.length / 2)
            s_exit = ap.exit_s - (s - self.own_cap.length / 2)
            if s_exit <= 0:
                continue
            out.append((s_entry, z, ap, s_exit))
        out.sort(key=lambda r: r[0])
        if not (traj and len(traj) >= 2):
            return out if limit is None else out[:limit]
        segs = [(traj[i], traj[i + 1], min(traj[i][0], traj[i + 1][0]), min(traj[i][1], traj[i + 1][1]),
                 max(traj[i][0], traj[i + 1][0]), max(traj[i][1], traj[i + 1][1])) for i in range(len(traj) - 1)]
        kept = []
        for r in out:
            z = r[1]
            if z.polygon:
                x0, y0, x1, y1 = self.poly_bbox[z.id]
                if not any(sx1 >= x0 and sx0 <= x1 and sy1 >= y0 and sy0 <= y1 and segment_hits_poly(p, q, z.polygon)
                           for p, q, sx0, sy0, sx1, sy1 in segs):
                    continue
            kept.append(r)
            if limit is not None and len(kept) >= limit:
                break
        return kept

    def _seen_leaving(self, last, tracked):
        """A partner out of the conflict set has left it: it was inside the zone (it exits), or it is tracked and
        moving, off the approaches (it turned away). The 078 P2c ruling."""
        if last.s_entry <= 0.0:
            return True
        now = tracked.get(last.id)
        return now is not None and now.v > self.p.moved_speed

    def _keep_dropped(self, t, z, confl, tracked):
        """Issue 079: the zone's conflict set with each partner that dropped out of it unseen leaving, for
        cs_hold_s after it was last in it (as placed then, its stamp now). Returns (set, the ids kept)."""
        last = self.cs_last.setdefault(z.id, {})
        for ag in confl:
            last[ag.id] = (t, ag)
        present = {ag.id for ag in confl}
        kept = []
        for i, (t_in, ag) in list(last.items()):
            if i in present:
                continue
            if t - t_in >= self.p.cs_hold_s - 1e-9 or self._seen_leaving(ag, tracked):
                del last[i]
                continue
            kept.append(replace(ag, stamp=t))
        return confl + kept, tuple(sorted(ag.id for ag in kept))

    def _dropped_hold(self, t, agents, confl_by_zone, s_entry_by_zone, contract):
        """The last decision, held while a partner of its zone that dropped out of the conflict set unseen is
        within ``mode_hold`` of its drop (the 078 P2c ruling); None once released (and the memory cleared)."""
        dec, z = self.last_chosen
        if contract or z.id not in confl_by_zone:  # a failed contract, or the zone is behind the car
            self.last_chosen, self.partners, self.drop_since, self.dropped = None, {}, None, ()
            return None
        present = {ag.id for ag in confl_by_zone[z.id]}
        tracked = {ag.id: ag for ag in agents}
        dropped = tuple(i for i, last in self.partners.items() if i not in present
                        and not self._seen_leaving(last, tracked))
        if dropped and self.drop_since is None:
            self.drop_since = t  # issue 079: after cs_hold_s kept in the conflict set (the two add up)
        if not dropped or t - self.drop_since >= self.p.mode_hold - 1e-9:
            self.last_chosen, self.partners, self.drop_since, self.dropped = None, {}, None, ()
            return None
        for ag in confl_by_zone[z.id]:
            self.partners[ag.id] = ag
        self.dropped = dropped
        return dec, z, s_entry_by_zone[z.id]

    def cycle(self, t, ego, objects, traj=None, statuses=(), obs_age=0.0, cycle_time=0.0, max_zones=3, self_ok=None,
              alive=True):
        """ego: (x, y, yaw, v, a). objects: [Obj]. statuses: [Status] of the driven modules. Returns Actions.
        cycle_time, self_ok, alive: the self contract's reading and state (contracts.SelfWindow; issue 066)."""
        t0, c0 = time.perf_counter(), time.thread_time()
        x, y, yaw, v, a = ego
        acts = Actions()
        agents = []
        for o in objects:
            self.first_seen.setdefault(o.id, t)
            cls = LABEL_CLASS.get(o.label, "car")
            cap = self.p.capability(cls, length=o.length, width=o.width)
            agents.append(Agent(o.id, cls=cls, pose=(o.x, o.y, o.yaw), v=max(o.v, 0.0), a=o.a, stamp=t - obs_age, cap=cap,
                                track_age=t - self.first_seen[o.id]))
        chosen = None
        contract = False
        cache = {}
        confl_by_zone = {}
        s_entry_by_zone = {}
        tracked = {ag.id: ag for ag in agents}
        cands = self.candidates(x, y, yaw, traj, cache, max_zones)
        for zid in set(self.cs_last) - {r[1].id for r in cands}:  # behind the car, or past max_zones
            del self.cs_last[zid]
        for s_entry, z, ap, s_exit in cands:
            own = Agent("ego", approach=ap.id, s_entry=s_entry, s_exit=s_exit, v=v, a=a, stamp=t, cap=self.own_cap,
                        pose=(x, y, yaw))
            cx, cy = self.centroid[z.id]
            near_agents = [ag for ag in agents if abs(ag.pose[0] - cx) < self.horizon and abs(ag.pose[1] - cy) < self.horizon]
            rel = conflict_set(relate(near_agents, own, [z], cache=cache).get(z.id, []))
            confl, kept = self._keep_dropped(t, z, [ag for _, ag in rel], tracked)
            if kept:
                acts.cs_held[z.id] = kept
            if kept != self.cs_kept.get(z.id, ()):
                acts.cs_events.append({"t": t, "zone": z.id, "kept": list(kept),
                                       "conflict_set": [ag.id for ag in confl]})
            confl_by_zone[z.id], s_entry_by_zone[z.id] = confl, s_entry
            inp = C.Inputs(obs_age=obs_age, cycle_time=cycle_time, self_ok=self_ok, alive=alive,
                           track_age={ag.id: ag.track_age for ag in confl},
                           accel={ag.id: (ag.a, ag.cap.a_max, ag.cap.b_max) for ag in confl})
            dec, rec = self.proto.step(t, own, confl, z, inp, obs_age=obs_age)
            if rec is not None:
                acts.records.append(rec)
            if chosen is None and dec.kind != STOCK and not dec.stock_equivalent:
                chosen = (dec, z, s_entry)
            contract = contract or (dec.kind == STOCK and dec.reason == "contract")
        for zid in set(self.cs_kept) - set(acts.cs_held):
            if zid not in confl_by_zone:  # the zone is no longer evaluated: its kept set ends unrecorded otherwise
                acts.cs_events.append({"t": t, "zone": zid, "kept": [], "conflict_set": []})
        self.cs_kept = dict(acts.cs_held)
        if chosen is not None:
            self.last_chosen = (chosen[0], chosen[1])
            self.partners = {ag.id: ag for ag in confl_by_zone.get(chosen[1].id, [])}
            self.drop_since, self.dropped = None, ()
        elif self.last_chosen is not None:
            chosen = self._dropped_hold(t, agents, confl_by_zone, s_entry_by_zone, contract)
            if chosen is not None:
                acts.held_dropped = self.dropped
        near = [s for s in statuses if s.module in self.modules]
        if chosen is not None:
            self.equiv_since = None
        elif self.equiv_since is None:
            self.equiv_since = t
        held = [s for s in near if s.module in self.manual]
        if chosen is None and held and not contract and t - self.equiv_since < self.p.mode_hold - 1e-9:
            # the mode hysteresis: still manual, acting as auto mode would (the module's own safe)
            for s in held:
                if s.auto_mode and s.uuid in self.sent:
                    del self.sent[s.uuid]
                if self.sent.get(s.uuid) != s.safe:
                    self.sent[s.uuid] = s.safe
                    acts.rtc.append((s.module, s.uuid, s.safe))
            if self.limited:
                acts.velocity_limit, self.limited = "clear", False
        elif chosen is None:
            for m in sorted(self.manual):
                acts.auto_mode.append((m, True))
            self.manual.clear()
            self.sent.clear()
            if self.limited:
                acts.velocity_limit, self.limited = "clear", False
        else:
            dec, z, s_entry = chosen
            acts.decision, acts.zone = dec, z.id
            activate = dec.kind == GO or (dec.kind == CREEP and dec.reason.endswith("creep"))
            for m in self.modules:
                if m not in self.manual and any(s.module == m for s in near):
                    self.manual.add(m)
                    acts.auto_mode.append((m, False))
            for s in near:
                if s.auto_mode and s.module in self.manual and s.uuid in self.sent:
                    # the module is back in auto mode while this node holds it (the watchdog restored it, or
                    # the status is a new registration): the command is sent again, which makes it manual again
                    del self.sent[s.uuid]
                if self.sent.get(s.uuid) != activate:
                    self.sent[s.uuid] = activate
                    acts.rtc.append((s.module, s.uuid, activate))
            limit = None
            if dec.kind == CREEP and dec.reason.endswith("creep"):
                limit = self.p.creep_speed
            elif dec.kind in STOPPING or dec.kind == CREEP:
                if not near or dec.reason == HOLD:
                    # no driven module covers the zone, or a stop held over a slow cycle (issue 075: a creep aborted
                    # to a wait, the creep's 1 m/s limit gone): stop before it by the velocity limit
                    limit = math.sqrt(2 * self.own_cap.b_comfort * max(s_entry - 2.0, 0.0))
            if limit is not None:
                acts.velocity_limit, self.limited = round(limit, 2), True
            elif self.limited:
                acts.velocity_limit, self.limited = "clear", False
        self.counts["rtc"] += len(acts.rtc)
        self.counts["auto_mode"] += len(acts.auto_mode)
        self.counts["velocity_limit"] += acts.velocity_limit is not None
        self.last_cycle = time.perf_counter() - t0
        self.last_cycle_cpu = time.thread_time() - c0
        return acts


def replace_agent(a, **kw):
    return replace(a, **kw)


__all__ = ["Strategy", "Obj", "Status", "Actions", "place"]

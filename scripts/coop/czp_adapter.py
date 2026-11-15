#!/usr/bin/env python3
"""Roadmap 078 I3: the arena adapter. The arena (and kinematic) car decided by the czp library in
place of a study's hand-written rules.

The adapter turns what a study's policy object perceives (the other car's (u, v, a) stamped, the
car's own u and v) into czp's data model -- a Zone with the study's approaches and map order, two
Agents with s_entry / s_exit along their own paths --, calls ``czp.Protocol.step`` every step, and
executes the decision as the study's cars execute theirs: go = drive to the cruise speed; creep =
the probe's creep (about 0.5 m at creep speed, then hold and watch); yield / wait = brake for the
stop line (an all-way stop) or hold out and brake for it only when needed (a crossing, a narrow
passage); stock = drive. The study's safety layer stays underneath, unchanged (crossing_game's).

Rule kinds registered (none exist until ``enable()`` runs, so every study default is unchanged):

  czp      the full protocol with 074 C8's three fixes (guard filter and go latch, own capability,
           the legible probe before a round-2 go)
  czp0     the same with the fixes off (no filter, no go latch, no probe, and V2b's fresh-rest guard and
           first-go probe off): the parity reference against C8's (e) (fcfse)

Studies (``zone_kind``):
  all_way_stop       U3 / C8: four_way_game, four_way_arena (rules czp_c1.5, czp0_c1.5)
  cross_unsignalled  S2 / C7: crossing_game via ``patched_crossing`` (as tiebreak.patched)
  narrow_passage     S4 / C7: narrow_game via ``patched_crossing`` (A's side obstructed)
  merge_lane_drop    S3: merge_game via ``MergeRules`` (czp decides A's merge against B)
"""

from __future__ import annotations

import contextlib
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "czp"))

import crossing_game as X  # noqa: E402
import czp  # noqa: E402
import czp.contracts  # noqa: E402
from czp import model as CM  # noqa: E402
from czp.model import Agent, Approach, Zone  # noqa: E402

CZP_KINDS = ("czp", "czp0")
# per car capability (the own car's, fix (b)); four_way_arena's --heavy sets one
CAPS = {}
# the role-time bias the observer adds (the arena's rest stamp lag, C8 measure-e); four_way_arena sets it
BIAS = {"value": 0.0}
# the margin m (C8's --fm-m), when a study varies it
MARGIN = {"value": None}


def zone_for(kind, angle=90.0, obstructed=None):
    """The study's zone: A along +x (heading 0, order 0), B from A's right (heading angle), order 1 --
    C8 (e)'s map order (A, then B). At the narrow passage B comes head-on (heading pi) and A's side is
    obstructed; at the lane drop B is on the through lane (priority major), A on the ending lane."""
    th = math.radians(angle)
    if kind == CM.NARROW_PASSAGE:
        aps = (Approach("A", heading=0.0, order=0, obstructed=obstructed != "B"),
               Approach("B", heading=math.pi, order=1, obstructed=obstructed == "B"))
    elif kind == CM.MERGE_LANE_DROP:
        aps = (Approach("A", heading=0.0, order=1, priority="minor"), Approach("B", heading=0.0, order=0, priority="major"))
    else:
        aps = (Approach("A", heading=0.0, order=0), Approach("B", heading=th, order=1))
    return Zone("z0", kind, aps)


def params_for(fixes=True, bias=0.0, margin=None):
    p = czp.load_params()
    kw = {"role_time_bias": bias}
    if margin is not None:
        kw["tie_margin_m"] = margin
    if not fixes:  # C8's three fixes off, and V2b's two guards (the reference closest to C8's (e))
        kw.update(guard_filter_n=1, go_latch_launch=False, fresh_rest_guard=False, probe_first_moving=False)
    return p.override(source="arena adapter", **kw)


def conventions_for(fixes=True):
    c = czp.load_conventions()
    if not fixes:
        c = dict(c)
        c["all_way_stop"] = dict(c["all_way_stop"], probe_round2=False)
    return c


class CZPCore:
    """The adapter's czp side: build the agents, step the protocol, keep the last decision."""

    def __init__(self, role, kind, zone_kind, v0, exit_, other_exit, angle=90.0):
        self.fixes = kind == "czp"
        self.params = params_for(self.fixes, BIAS["value"], MARGIN["value"])
        self.proto = czp.Protocol(conventions_for(self.fixes), self.params)
        self.zone = zone_for(zone_kind, angle)
        self.role, self.v0 = role, v0
        self.other = "B" if role == "A" else "A"
        p = self.params
        car = p.capability("car", v_max=v0, length=X.L)
        own_cap = CAPS.get(role, car) if self.fixes else car  # fix (b): the car's own capability
        self.own_cap = own_cap
        self.other_cap = CAPS.get(self.other, car)
        self.exit, self.other_exit = exit_, other_exit
        self.dec = None
        self.flips = 0
        self.records = []

    def decide(self, t, u, v, a, seen, exit_clear=True):
        """seen: (stamp, ou, ov, oa) of the other as this car sees it now (predicted to now in ou)."""
        own = Agent(self.role, approach=self.role, s_entry=-u, s_exit=self.exit - u, v=v, a=a, stamp=t, cap=self.own_cap)
        others = []
        age = 0.0
        if seen is not None:
            stamp, ou, ov, oa = seen
            age = t - stamp
            others.append(Agent(self.other, approach=self.other, s_entry=-ou, s_exit=self.other_exit - ou, v=ov, a=oa,
                                stamp=stamp, cap=self.other_cap))
        # the contracts see the perception's latency (OBS); the role reads the view's full age (OBS + DECIDE)
        inputs = czp.contracts.Inputs(obs_age=min(age, X.OBS))
        dec, rec = self.proto.step(t, own, others, self.zone, inputs=inputs, exit_clear=exit_clear, obs_age=age)
        if rec is not None:
            self.records.append(rec)
            if self.dec is not None:
                self.flips += 1
        self.dec = dec
        return dec

    def other_rest(self):
        m = self.proto.mem.get(self.zone.id)
        if m is None:
            return None
        am = m.agents.get(self.other)
        return None if am is None else am.rest_t


def creep_accel(u, v, params, vc=None):
    """The probe's creep: toward creep speed (the watch: the watch speed), never into the zone (u < 0)."""
    vc = params.creep_speed if vc is None else vc
    if u > -0.3:
        return -min(X.B_MAX, v / X.DT)
    return min(X.A_MAX, (vc - v) / X.DT) if v < vc else -min(X.B_MAX, (v - vc) / X.DT)


def hold_accel(v):
    return -min(X.B_MAX, v / X.DT) if v > 0 else 0.0


# --- U3 / C8: the four-way stop --------------------------------------------------------------------
def _fw():
    import four_way_game as F
    return F


def make_fw_class():
    F = _fw()

    class CZPFourWay(F.FWPolicy):
        """four_way_game.FWPolicy's interface (perceive / step / after, its bookkeeping and its safety
        layer) with the decision taken by czp (all_way_stop)."""

        def __init__(self, role, rule, v0, hold_stop=None, exit_=None, other_exit=None):
            super().__init__(role, rule, v0, hold_stop, exit_, other_exit)
            self.core = CZPCore(role, self.kind, CM.ALL_WAY_STOP, v0, self.exit, self.other_exit, ANGLE["value"])
            self.a_prev = 0.0

        def step(self, t, u, v):
            sd = self._seen(t, F.OBS + F.DECIDE)
            seen = None
            if sd is not None:
                seen = (sd[0], sd[1] + sd[2] * (t - sd[0]), sd[2], sd[3])
                if sd[1] >= self.other_exit:
                    self.other_cleared = True
            dec = self.core.decide(t, u, v, self.a_prev, seen, self.exit_clear)
            self.reading = dec.reason
            self.other_rest_t = self.core.other_rest()
            want = dec.kind == CM.GO or (dec.kind == CM.STOCK)
            if self.go and not want:
                self.restarts += 1
            self.go = want
            if want:
                a = X.drive(v, self.v0)
            elif dec.kind == CM.CREEP:
                a = creep_accel(u, v, self.core.params, None if dec.reason.endswith("creep")
                                else self.core.params.probe_watch_speed)
            else:
                a = F.plan_stop(u, v, self.v0)
            a = self._shield(t, u, v, a)
            self.a_prev = a
            return a

        def _shield(self, t, u, v, a):
            """FWPolicy's safety layer, as it is (crossing_game's, REACT s late)."""
            sr = self._seen(t, F.REACT, first=False)
            if sr is None or u >= 0:
                return a
            ou, ov = sr[1] + sr[2] * (t - sr[0]), sr[2]
            in_box = 0 <= ou < self.other_exit
            closing = (ou < 0 and ov > 0.5 and X.t_in(ou, ov) < 2.0 and X.t_in(ou, ov) <= X.t_in(u, max(v, 0.5)) + 0.3
                       and X.t_in(ou, ov) < X.t_cover(self.exit - u, v, self.v0))
            if closing and self.core.dec is not None and self.core.dec.reason not in ("turn", "contested") \
                    and self.other_rest_t is None and self._stopping(sr[1], sr[2], sr[3]):
                closing = False  # as fcfsm: a car braking for its line will stop (the zone contract)
            if in_box or closing:
                need = v * v / (2 * max(-u - 0.3, 0.05))
                if need > 0.3:
                    planned = a
                    a = min(a, -min(X.B_MAX, need))
                    self.shield = self.shield or (v > 0.5 and planned - a > 1.0 and need > X.BRAKE_LATE + 1.0)
                elif a > 0 and u > F.STOP - 1.0:
                    a = min(a, 0.0)
                    self.clip_n += 1
                    if self.hold_stop and v < X.HOLD_STOP_V:
                        a = -min(X.B_MAX, v / X.DT)
            return a

    return CZPFourWay


ANGLE = {"value": 90.0}
CZP_RULES = [(f"{k}_c1.5", k, 1.5) for k in CZP_KINDS]
CZP_NAMES = [r[0] for r in CZP_RULES]


def enable(angle=90.0, bias=0.0, margin=None):
    """Register czp's rules with four_way_game (RULES, POLICY_KINDS). Nothing changes until this runs."""
    F = _fw()
    ANGLE["value"], BIAS["value"], MARGIN["value"] = angle, bias, margin
    cls = make_fw_class()
    for k in CZP_KINDS:
        F.POLICY_KINDS[k] = cls
    for r in CZP_RULES:
        if r not in F.RULES:
            F.RULES.append(r)
    return CZP_NAMES


# --- S2 / C7 / S4: the crossing and the narrow passage (crossing_game.Policy's interface) -----------
class CZPPassage(X.Policy):
    """crossing_game.Policy's interface with czp deciding: cross_unsignalled (S2, C7) or
    narrow_passage (S4). go = drive; yield / wait = hold out and brake for the line when needed."""

    def __init__(self, role, rule, v0, b_cont="hold", exit_=X.EXIT, hold_stop=None, zone_kind=CM.CROSS_UNSIGNALLED,
                 kind="czp", ident=None):
        super().__init__(role, ("hold", None, None) if role == "B" else ("go", -math.inf, False, math.inf), v0, b_cont,
                         exit_, hold_stop)
        self.ident = ident or role
        self.core = CZPCore(self.ident, kind, zone_kind, v0, exit_, exit_)
        self.name = kind
        self.a_prev = 0.0
        self.committed = False
        self.restarts = 0

    def step(self, t, u, v):
        sd = self._seen(t, X.OBS + X.DECIDE)
        seen = None if sd is None else (sd[0], sd[1] + sd[2] * (t - sd[0]), sd[2], sd[3])
        dec = self.core.decide(t, u, v, self.a_prev, seen)
        want = dec.kind in (CM.GO, CM.STOCK)
        if self.committed and not want:
            self.restarts += 1
        self.committed = want
        if want:
            a = X.drive(v, self.v0)
        elif dec.kind == CM.CREEP:
            a = creep_accel(u, v, self.core.params, None if dec.reason.endswith("creep")
                            else self.core.params.probe_watch_speed)
        else:
            a = X.hold_out(u, v)
        a = self._shield(t, u, v, a)
        self.a_prev = a
        return a

    def _shield(self, t, u, v, a):
        sr = self._seen(t, X.REACT, first=False)
        if sr is None or u >= 0:
            return a
        ou, ov = sr[1] + sr[2] * (t - sr[0]), sr[2]
        in_box = 0 <= ou < self.exit
        closing = (ou < 0 and ov > 0.5 and X.t_in(ou, ov) < 2.0 and X.t_in(ou, ov) <= X.t_in(u, max(v, 0.5)) + 0.3
                   and X.t_in(ou, ov) < X.t_cover(self.exit - u, v, self.v0))
        if in_box or closing:
            need = v * v / (2 * max(-u - 0.3, 0.05))
            if need > 0.3:
                planned = a
                a = min(a, -min(X.B_MAX, need))
                self.shield = self.shield or (v > 0.5 and planned - a > 1.0 and need > X.BRAKE_LATE + 1.0)
            elif a > 0 and u > X.STOP - 1.0:
                a = min(a, 0.0)
                self.note_clip(v)
                if self.hold_stop and v < X.HOLD_STOP_V:
                    a = -min(X.B_MAX, v / X.DT)
        return a


@contextlib.contextmanager
def patched_crossing(czp_cars=("A", "B"), zone_kind=CM.CROSS_UNSIGNALLED, kind="czp"):
    """crossing_game.Policy replaced, for the cars named, by CZPPassage for the next simulate (A is built
    first, then B, in crossing_game, narrow_game and the arenas), as tiebreak.patched does."""
    orig = X.Policy
    idents = iter("AB")
    made = []

    def make(role, rule, v0, b_cont="hold", exit_=X.EXIT, hold_stop=None):
        ident = next(idents)
        if ident in czp_cars:
            p = CZPPassage(role, rule, v0, b_cont, exit_, hold_stop, zone_kind, kind, ident)
        else:
            p = orig(role, rule, v0, b_cont, exit_, hold_stop)
        made.append(p)
        return p

    X.Policy = make
    try:
        yield made
    finally:
        X.Policy = orig


# --- S3: the merge at a lane drop (merge_game.Rules' interface) -------------------------------------
def make_merge_rules():
    import merge_game as MG

    class CZPMergeRules(MG.Rules):
        """merge_game.Rules with A's merge decided by czp (merge_lane_drop: who is ahead at the lane's
        end, with the margin; the through lane first in the map's order). go = merge ahead of B when the
        study's gap fits (its physics, as a safety check); yield or wait = fall behind B and merge into
        its wake (the study's drop). B keeps its own rule. Positions are measured to the lane's end."""

        def __init__(self, ra, rb, x_end, va0, vb0, b_cont="hold", kind="czp"):
            # A's own gap acceptance off (tau inf): czp decides the merge ahead; the merge into B's wake
            # and the drop at the deadline stay the study's (zipper timing)
            ra = (ra[0], ra[1], math.inf, False, "drop")
            super().__init__(ra, rb, x_end, va0, vb0, b_cont)
            self.core = CZPCore("A", kind, CM.MERGE_LANE_DROP, va0, MG.L, MG.L)
            self.core.other_cap = self.core.params.capability("car", v_max=vb0, length=MG.L)
            self.core.own_cap = self.core.params.capability("car", v_max=va0, length=MG.L)
            self.a_prev = 0.0
            self.last_dec = None

        def step(self, t, xa, va, lat, xb, vb, seen_b, seen_a):
            if self.decide is None and not self.dropping:
                xb_s, vb_s, ab_s = seen_b
                age = MG.OBS + MG.DECIDE
                u_a = -(self.x_end - xa)  # the front's position past the lane's end
                u_b = -(self.x_end - xb_s)
                dec = self.core.decide(t, u_a, va, self.a_prev, (t - age, u_b, vb_s, ab_s))
                self.last_dec = dec
                gap_ahead = (xa - MG.L) - xb_s
                # go: A is ahead at the end (or clear): merge ahead once the study's gap fits; yield or
                # wait: keep the lane (the study's zipper merges into B's wake, or drops at the deadline)
                deadline = self.x_end - xa <= MG.lane_change_dist(va)
                fits = gap_ahead >= MG.BUFFER + vb_s * MG.TAUS[0] or (deadline and gap_ahead >= MG.BUFFER)
                if dec.kind in (CM.GO, CM.STOCK) and fits:
                    self.decide = t  # at the deadline A, ahead at the end, claims its slot (B's turn is behind)
            aa, ab = super().step(t, xa, va, lat, xb, vb, seen_b, seen_a)
            self.a_prev = aa
            return aa, ab

    return CZPMergeRules


@contextlib.contextmanager
def patched_merge(kind="czp"):
    """merge_game.Rules replaced by the czp merge for the next simulate (A is czp, B keeps its rule)."""
    import merge_game as MG
    orig = MG.Rules
    cls = make_merge_rules()
    MG.Rules = lambda ra, rb, x_end, va0, vb0, b_cont="hold": cls(ra, rb, x_end, va0, vb0, b_cont, kind)
    try:
        yield
    finally:
        MG.Rules = orig

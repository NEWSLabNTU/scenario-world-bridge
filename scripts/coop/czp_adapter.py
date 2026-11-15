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
END_STOP = 1.0  # m: the czp car's planned stop before the lane's end (as the study's drop aims for)
END_PLAN = 2.5  # m/s^2: it brakes for that stop once it needs this much (the study's BRAKE_LATE) ...
DROP_LEAD = 1.0  # s: an unresolved (ambiguous) order is settled this long before the deadline, B first
DROP_DV = 2.0  # m/s: A falls back this much below B's speed while beside it
LAT_SEEN = 0.08  # A's lateral progress at which B sees the lane change (merge_game's B rules: 0.08)
B_YIELD_DECEL = 3.0  # m/s^2: czp's B makes room at most this hard (S3's c40 courtesy)
REST_GAP = 0.5  # m: at rest at the end, A merges ahead of a B at rest behind it with this much room
END_LAG = 0.15  # s: ... over the distance left after the brake's dead time and lag (the study ignores them)


def make_merge_rules():
    import cutin_game as C1
    import merge_game as MG
    Study = getattr(MG, "_orig_rules", MG.Rules)  # the study's class, also while MG.Rules is patched

    class CZPMergeRules(Study):
        """merge_game.Rules with A decided and driven by czp (merge_lane_drop: who leads whom by a
        headway, ``band: gap``; then ahead_at_end with the margin; the through lane first in the map's
        order). B keeps its own rule and safety layer (the study's, unchanged). Positions are measured to
        the lane's end.

        A's act (078 S3 fix):
        - merge ahead only on czp's go and the study's full gap test (tau 0.6 s with the closing term),
          never on the bare buffer at the deadline: a car beside the other is not ahead;
        - merge into B's wake (B first) whenever the gap behind is there (the planned stop at the end keeps a
          late one from running out: it waits, half across or at the end, for B to pass);
        - merge ahead only before its commit point (half across, at its speed, before the planned stop at
          the end) and only when the gap also holds over the half lane change at both cars' current
          accelerations (A braking for the end in front of an accelerating B was a contact); past it A drops;
        - drop back behind B (the study's drop) as soon as czp yields, and when the order is still
          unresolved DROP_LEAD s before the deadline (the last point a lane change still ends before the lane
          does), so that there is room to merge behind B instead of stopping;
        - B's courtesy (seen braking <= -yield_decel for yield_hold s, the probe's yield test) with A not
          behind it: A stops dropping back and merges ahead once the gap holds for the half lane change
          (BUFFER plus B's closing over it at its seen braking; mutual courtesy, A dropping while B makes room, was S3's forced
          stop at the end); from rest at the end A merges ahead of a B that has stopped behind it;
        - brake for the end as a planned stop, END_STOP m before it, over the distance left after the brake's
          lag, also once it has begun a merge (the study brakes for the end itself, then without the 1 m and
          the lag, and a merge begun late over-ran the end: the 10 contacts).
        """

        def __init__(self, ra, rb, x_end, va0, vb0, b_cont="hold", kind="czp"):
            super().__init__(ra, rb, x_end, va0, vb0, b_cont)
            self.core = CZPCore("A", kind, CM.MERGE_LANE_DROP, va0, MG.L, MG.L)
            self.core.other_cap = self.core.params.capability("car", v_max=vb0, length=MG.L)
            self.core.own_cap = self.core.params.capability("car", v_max=va0, length=MG.L)
            self.a_prev = 0.0
            self.last_dec = None
            self.why = None  # how A merged: ahead, courtesy, wake, rest
            self.b_yield_since = None
            self.courtesy = False

        def committable(self, rem, va):
            """The merge ahead's commit point: half across (T_LC / 2 at its speed) before the planned stop at
            the end."""
            return rem - END_STOP >= max(va, 1.0) * MG.T_LC / 2 + 0.5

        def step(self, t, xa, va, lat, xb, vb, seen_b, seen_a):
            L, CAP = MG.L, MG.CAP
            xb_s, vb_s, ab_s = seen_b
            xa_s, va_s, lat_s = seen_a
            rem = self.x_end - xa
            # --- A: czp's decision and its act
            if self.decide is None:
                age = MG.OBS + MG.DECIDE
                dec = self.core.decide(t, -rem, va, self.a_prev, (t - age, -(self.x_end - xb_s), vb_s, ab_s))
                self.last_dec = dec
                gap_ahead = (xa - L) - xb_s
                gap_behind = (xb_s - L) - xa
                need = MG.BUFFER + vb_s * MG.TAUS[0] + max(0.0, vb_s ** 2 - va ** 2) / (2 * MG.A_REAR)
                ok = self.committable(rem, va)
                if gap_behind >= MG.S0 + va * MG.HEADWAY:
                    self.decide, self.why = t, ("rest" if self.stopped else "wake")
                # B's courtesy, read as czp's probe reads a yield: braking <= -yield_decel held yield_hold s
                p = self.core.params
                if ab_s <= -p.yield_decel:
                    self.b_yield_since = t if self.b_yield_since is None else self.b_yield_since
                else:
                    self.b_yield_since = None
                courtesy = self.b_yield_since is not None and t - self.b_yield_since >= p.yield_hold - 1e-9 \
                    and gap_ahead > -L  # A's front ahead of B's
                self.courtesy = courtesy
                # behind a yielding B the gap only has to hold while A is half across (B is opening it)
                h = MG.T_LC / 2
                # the gap over the half lane change at both cars' current accelerations (A's own command)
                # B may resume its speed (a stock car's cruise: 1.5 /s toward its limit, up to a_max) unless its
                # courtesy has been seen
                ab_eff = ab_s if self.courtesy else max(ab_s, min(MG.CAP.a_max, 1.5 * (self.core.other_cap.v_max - vb_s)))
                hold_h = gap_ahead - ((vb_s - va) * h + 0.5 * (ab_eff - min(self.a_prev, 0.0)) * h * h) >= MG.BUFFER
                need_c = MG.BUFFER + max(0.0, (vb_s - va) * h + 0.5 * min(ab_s, 0.0) * h * h)
                if self.decide is None and self.stopped and vb_s < 0.3 and gap_ahead >= REST_GAP:
                    self.decide, self.why = t, "rest_ahead"  # both at rest, A ahead: B is waiting for A
                if self.decide is None and (ok or self.stopped) and (
                        (dec.kind in (CM.GO, CM.STOCK) and not self.dropping and gap_ahead >= need and hold_h) or
                        (courtesy and gap_ahead >= min(need, need_c))):
                    self.decide, self.why = t, ("courtesy" if courtesy and dec.kind not in (CM.GO, CM.STOCK) else "ahead")
                if self.decide is None and not self.dropping and (
                        dec.kind in (CM.YIELD, CM.WAIT_SECOND_ROUND) or not ok or
                        (dec.kind != CM.GO and rem <= MG.lane_change_dist(va) + va * DROP_LEAD)):
                    self.dropping = True  # B first: drop back now, while there is room to merge behind it
            # --- A's longitudinal
            if self.decide is not None and lat > 0.0 and xa < xb:
                aa = C1.follow_cruise((xb - L) - xa, va, vb, CAP, self.va0)
            elif self.dropping and self.decide is None and self.courtesy:
                aa = 0.0  # B is making room ahead of it: stop dropping back, keep the speed
            elif self.dropping and self.decide is None and (xb_s - L) - xa < 0 and vb_s > 1.0:
                # beside B: fall back at DROP_DV below B's speed (the study brakes to rest beside it, which
                # against a courteous B stops both cars short of the end)
                aa = -C1.A_WAIT_DECEL if va > vb_s - DROP_DV else 0.0
            elif self.dropping and self.decide is None:
                aa = -C1.A_WAIT_DECEL if va > 0.5 * vb_s or (xb_s - L) - xa < 0 else 0.0
            else:
                aa = max(-1.0, min(CAP.a_max, 1.5 * (self.va0 - va)))
            if (self.decide is None or lat < 0.5) and rem > 0:  # not half across: a planned stop before the end
                d = rem - END_STOP - va * END_LAG
                need_e = va * va / (2 * max(d, 0.05))
                if d <= 0.05:
                    aa = -CAP.b_max if va > 0 else 0.0
                elif need_e >= END_PLAN:
                    aa = min(aa, -min(CAP.b_max, need_e * 1.15))
            # --- B: the study's rule and safety layer, unchanged (merge_game.Rules.step)
            if lat_s > 0.08 and self.lat_seen_at is None:
                self.lat_seen_at = t
            a_ahead = xa_s - L > xb - L
            gap_b = (xa_s - L) - xb
            yielding = False
            if self.bname == "react" and self.lat_seen_at is not None and t >= self.lat_seen_at + MG.REACT and a_ahead:
                yielding = self.b_cont == "yield"
            if self.bname.startswith("c"):
                if (not self.b_latched and t >= MG.REACT and a_ahead and gap_b < self.band and lat_s < 0.5
                        and (self.x_end - xa_s) <= MG.COURTESY_T * max(va_s, 1.0)):
                    self.b_latched = True
                yielding = self.b_latched and a_ahead
            b_dec = self.b_y if self.bname.startswith("c") else C1.B_YIELD_DECEL
            if yielding:
                ab = -b_dec if gap_b < MG.S0 + vb * MG.HEADWAY else C1.follow_cruise(gap_b, vb, va_s, CAP, self.vb0)
            else:
                ab = max(-1.0, min(CAP.a_max, 1.5 * (self.vb0 - vb)))
            if lat_s >= 0.5 and xa_s > xb:
                ab = min(ab, C1.follow_accel(gap_b, vb, va_s, CAP, self.vb0))
            aa = max(-CAP.b_max, min(CAP.a_max, aa))
            self.a_prev = aa
            return aa, max(-CAP.b_max, ab)

    class _BCZP:
        """B, the through lane's car, decided by czp (merge_lane_drop from B's side; B's approach is the major
        one). B's act: on czp's yield (A leads by the gap band, or is ahead at the end) once A's lane change is
        seen (lateral progress LAT_SEEN, the indicator and the drift) and while A is not across and not wholly
        behind B, B makes room: it follows A as its leader (IDM to S0 + v HEADWAY behind A's rear, at most
        B_YIELD_DECEL), the zipper's opening and C2's committed claimant; otherwise it keeps its lane and speed
        (stock's hold, whose safety layer stays underneath). Yielding before A moves made both cars yield
        against a dropping A (mutual courtesy) and, read back as a yield, both cars go."""

        def b_init(self, kind, vb0):
            self.core_b = CZPCore("B", kind, CM.MERGE_LANE_DROP, vb0, MG.L, MG.L)
            self.core_b.other_cap = self.core_b.params.capability("car", v_max=self.va0, length=MG.L)
            self.core_b.own_cap = self.core_b.params.capability("car", v_max=vb0, length=MG.L)
            self.b_prev = 0.0
            self.last_dec_b = None


        def b_step(self, t, ab, xa_s, va_s, lat_s, xb, vb):
            L = MG.L
            age = MG.OBS + MG.DECIDE
            if lat_s >= 1.0:
                self.b_prev = ab
                return ab  # A is across: B's safety layer follows it
            dec = self.core_b.decide(t, -(self.x_end - xb), vb, self.b_prev, (t - age, -(self.x_end - xa_s), va_s, 0.0))
            self.last_dec_b = dec
            if dec.kind not in (CM.GO, CM.STOCK) and xa_s > xb - L and lat_s >= LAT_SEEN:
                # make room at a comfortable deceleration (S3's courtesy: 2-3 m/s^2), not IDM's full braking
                ab = min(ab, max(-B_YIELD_DECEL, C1.follow_accel((xa_s - L) - xb, vb, va_s, MG.CAP, self.vb0)))
            ab = max(-MG.CAP.b_max, ab)
            self.b_prev = ab
            return ab

    Base = Study

    class CZPMergeRulesB(_BCZP, Base):
        """A keeps its study rule; B is czp (its study part is hold: the lane and speed, and the safety layer)."""

        def __init__(self, ra, rb, x_end, va0, vb0, b_cont="hold", kind="czp"):
            Base.__init__(self, ra, ("hold", None, None), x_end, va0, vb0, "hold")
            self.b_init(kind, vb0)

        def step(self, t, xa, va, lat, xb, vb, seen_b, seen_a):
            aa, ab = Base.step(self, t, xa, va, lat, xb, vb, seen_b, seen_a)
            return aa, self.b_step(t, ab, seen_a[0], seen_a[1], seen_a[2], xb, vb)

    class CZPMergeRulesAB(_BCZP, CZPMergeRules):
        """Both cars czp (self-play)."""

        def __init__(self, ra, rb, x_end, va0, vb0, b_cont="hold", kind="czp"):
            CZPMergeRules.__init__(self, ra, ("hold", None, None), x_end, va0, vb0, "hold", kind)
            self.b_init(kind, vb0)

        def step(self, t, xa, va, lat, xb, vb, seen_b, seen_a):
            aa, ab = CZPMergeRules.step(self, t, xa, va, lat, xb, vb, seen_b, seen_a)
            return aa, self.b_step(t, ab, seen_a[0], seen_a[1], seen_a[2], xb, vb)

    CZPMergeRules.B_ONLY, CZPMergeRules.BOTH = CZPMergeRulesB, CZPMergeRulesAB
    return CZPMergeRules


MERGE_CZP_A = ("czp", "zipper", 0.6, False, "drop")  # merge_game rule tuples naming czp in A's / B's place
MERGE_CZP_B = ("czp", None, None)


def merge_rules_factory(kind="czp", cars=None):
    """A merge_game.Rules factory: czp in the place whose rule is named "czp" (MERGE_CZP_A / MERGE_CZP_B), or
    in the places named by cars ("A", "B", "AB"); the study's Rules otherwise."""
    import merge_game as MG
    orig = MG.Rules if not hasattr(MG, "_orig_rules") else MG._orig_rules
    cls = make_merge_rules()

    def make(ra, rb, x_end, va0, vb0, b_cont="hold"):
        a = (cars is not None and "A" in cars) or ra[0] == "czp"
        b = (cars is not None and "B" in cars) or rb[0] == "czp"
        if a and b:
            return cls.BOTH(ra, rb, x_end, va0, vb0, b_cont, kind)
        if a:
            return cls(ra, rb, x_end, va0, vb0, b_cont, kind)
        if b:
            return cls.B_ONLY(ra, rb, x_end, va0, vb0, b_cont, kind)
        return orig(ra, rb, x_end, va0, vb0, b_cont)
    return make


@contextlib.contextmanager
def patched_merge(kind="czp", cars="A"):
    """merge_game.Rules replaced by the czp merge for the next simulate (default: A is czp, B keeps its rule)."""
    import merge_game as MG
    orig = MG.Rules
    MG._orig_rules = orig
    MG.Rules = merge_rules_factory(kind, cars)
    try:
        yield
    finally:
        MG.Rules = orig
        del MG._orig_rules

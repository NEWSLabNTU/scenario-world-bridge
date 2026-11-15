"""Step 4, the band: clear (and who goes), band, contested, or ambiguous at the band's edge.

clear when one interval's latest exit precedes the other's earliest entry by the margin M; the
reading is made robust to a perception error e on each side (as the role's tie margin, 074 C8):
clear needs the slack >= M + e, band needs both slacks <= M - e, and in between the reading is
ambiguous (both cars may read differently: wait). contested when the other shows no braking that
can stop it before the zone and would enter before this car could leave it: it keeps the turn
(C7). At an all-way stop the guard is C8's zone guard (any car not braking for its line, predicted
into the zone before this car leaves it plus a margin), on the filtered "stopping" test.
"""

from __future__ import annotations

from .intervals import ZoneInterval, stop_need
from .model import AMBIGUOUS, BAND, CLEAR, CONTESTED, INF, Agent, Band


def contested(own_iv: ZoneInterval, other: Agent, other_iv: ZoneInterval, stopping: bool, params, stop_law=False,
              rested=False):
    """The other cannot or will not stop before the zone and would be in it before this car is out."""
    if other.cleared or other.v < params.moved_speed or rested:
        return False
    if other.in_zone:
        return True
    if other.s_entry <= 0:
        return False
    if stopping:
        return False
    if other_iv.nominal_in > own_iv.t_out[1] + params.zone_guard_margin:
        return False
    if stop_law:
        return True  # the zone guard: not braking for its line, and it would be in before this car is out
    return stop_need(other.s_entry, other.v) > other.cap.b_comfort  # past its comfortable stop point


def gap_slacks(own: Agent, other: Agent, params):
    """The merge's (lane drop's) slacks, in seconds: both cars travel the same lane toward its end, and
    the zone is the stretch where the merge can happen, not a point. Each car's lead over the other is
    the distance from its rear to the other's front (s_exit is the rear's distance to the end, s_entry
    the front's), predicted over ``merge_horizon`` (the lane change's half: where the merging car is
    half across), less the tightest headway a merge accepts (``merge_headway``, S3's tau); the slack is
    that lead in time at the follower's speed. A merge ahead needs the lead now, not at the lane's end:
    a car that is ahead but slower is caught before the end, and a car beside but faster is not ahead
    until its rear has passed the other's front (078 S3 fix)."""
    h, hw = params.merge_horizon, params.merge_headway
    lead_own = (other.s_entry - own.s_exit) + (own.v - other.v) * h
    lead_other = (own.s_entry - other.s_exit) + (other.v - own.v) * h
    return lead_own / max(other.v, 1.0) - hw, lead_other / max(own.v, 1.0) - hw


def band(own_iv: ZoneInterval, other_iv: ZoneInterval, other: Agent, params, stopping=False, stop_law=False,
         rested=False, margin=None, e=None, mode="interval", own: Agent = None):
    """mode "interval" (the design's test: one's latest exit before the other's earliest entry, margin
    band_margin), "entry" (the arrival order: one's latest entry before the other's earliest entry,
    margin band_margin_entry; the later car yields, as the studies' nearer-car-first outside the
    tie band, 074 S2: |dt| <= 0.2 s) or "gap" (a merge: who leads whom by a headway, ``gap_slacks``;
    margin band_margin_entry; needs ``own``)."""
    e = params.role_time_error_e if e is None else e
    if mode == "gap":
        if other.cleared:
            return Band(CLEAR, own_iv.agent, INF, INF)
        m = params.band_margin_entry if margin is None else margin
        slack_own, slack_other = gap_slacks(own, other, params)
        if slack_own >= m + e:
            return Band(CLEAR, own_iv.agent, slack_own, slack_other)
        if slack_other >= m + e:
            return Band(CLEAR, other.id, slack_own, slack_other)
        if slack_own <= m - e and slack_other <= m - e:
            return Band(BAND, None, slack_own, slack_other)
        return Band(AMBIGUOUS, None, slack_own, slack_other)
    if mode == "entry":
        m = params.band_margin_entry if margin is None else margin
        slack_own = other_iv.t_in[0] - own_iv.t_in[1]  # I am in before it could be
        slack_other = own_iv.t_in[0] - other_iv.proceed_in  # it is in (going now) before I could be
    else:
        m = params.band_margin if margin is None else margin
        slack_own = other_iv.t_in[0] - own_iv.t_out[1]  # I am out before it could be in
        slack_other = own_iv.t_in[0] - other_iv.proceed_out  # it is out (if it keeps going) before I could be in
    if other.cleared:
        return Band(CLEAR, own_iv.agent, INF, INF)
    if contested(own_iv, other, other_iv, stopping, params, stop_law, rested) and slack_own < m + e:
        return Band(CONTESTED, other.id, slack_own, slack_other)
    if slack_own >= m + e:
        return Band(CLEAR, own_iv.agent, slack_own, slack_other)
    if slack_other >= m + e:
        return Band(CLEAR, other.id, slack_own, slack_other)
    if slack_own <= m - e and slack_other <= m - e:
        return Band(BAND, None, slack_own, slack_other)
    return Band(AMBIGUOUS, None, slack_own, slack_other)

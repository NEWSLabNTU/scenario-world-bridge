#!/usr/bin/env python3
"""Roadmap 074 N2: the arena with a third car, and the same scene with one car left out.

arena.py's cut-in (A in lane -2 wants into B's lane -1) with one more car on the same road:

  scene a  C follows B in lane -1 at a time headway hf (the gap S0 + v_B hf): A's cut-in lands
           ahead of B, or, if A drops back, in the B-C slot or behind C
  scene b  L leads A in A's own lane -2, `gl` m ahead of A's front, at B's speed
  scene a4 scene a with a fourth car: D follows C at the same headway hf (074 N3's four-car queue)
  scene p  A and B alone (074 N3's table of pair terms at shifted contexts)

A group runs any subset of the scene's cars ("ABC", "AB", "AC", "BC", "ABL", "AL", "BL", "ABCD", ...) from
the same start states, with the same rules, reaction draws and observer noise (common random
numbers: each car's draws come from its own stream, seeded by the context, the profile and the
seed, so they do not depend on which other cars are present). The two-car groups are N2's pair
terms; the three-car group is what the composition is tested against (n2_composition.py).

The rules, all seeing each other through arena.Observer (10 Hz, delayed, noisy):
  A  arena.Pair's parameterized rule (pa): the gap check against the first car of the target
     lane's queue (B, C, D: those present); dropping back, it takes the slot behind a queue car
     only if the gap ahead of the next one passes the same check, else it drops behind that one
     too, slot by slot (the minimal extension: with one car in the queue it is Pair's rule). In
     its own lane it follows L (IDM interaction term, cutin_game.follow_accel) and otherwise
     cruises as in Pair.
  B  arena.Pair's parameterized rule (pb) against A; cruise otherwise.
  C, D  followers: cruise at the start speed, capped by the IDM interaction term toward the
     nearest car ahead in lane -1 (the queue cars ahead; A once it is seen in the lane, after
     the follower's reaction time).
  L  cruise at its start speed.

A and B follow with arena.follow (cutin_game.follow_cruise since N3: the follower regains its
speed; Pair's rule as well).

Measures: one measures.Measures per present pair (push_poses every tick); per car the arc
length, peak braking (as Pair), peak lateral acceleration, contact (any pair's footprints
overlap) and rho_ij per pair. U_i = measures.utility(lost_i, max_j rho_ij, ...), with Pair's
rules: lost = 1 on contact, and for A if it never got into lane -1. Every group runs a fixed
T_RUN after the warm-up (or ends at the first contact); route progress is scored over the first
HORIZON (20 s) of it, as arena.Pair scores it (074 N3), the rest counts in risk and comfort.

Warm-up: every car tracks a kinematic ghost of B (accelerating from rest to v_B) at its own
offset, ramped so the context holds at the run's start; the reached context is recorded.

Not standalone: n2_composition.py and n3_model.py drive it.
"""

from __future__ import annotations

import math
import random
from collections import deque

import carla

import arena as AR
import cutin_game as C1
import measures

T_RUN = C1.T_MAX  # s after the warm-up (40 s: a four-car queue decides late); progress over its first HORIZON
HORIZON = C1.HORIZON
V_FREE = 1e3  # IDM's desired speed for the interaction term only (cruise caps the speed)
TRACE_DT = 0.1  # s between recorded poses (N1's conflict sets)
ROLES = "ABCLD"
QUEUE = "BCD"  # lane -1, front to back
LANES = {"A": -2, "B": -1, "C": -1, "L": -2, "D": -1}
AR.ROAD_LEN = 1500.0  # v_B 16 m/s: a warm-up and T_RUN reach ~900 m; absent cars wait past that
PARK_X = {"A": 1460.0, "B": 1430.0, "C": 1400.0, "L": 1370.0, "D": 1340.0}  # where an absent car waits
X_MAX = PARK_X["D"] - 20.0  # a present car past this could reach a parked one: the run is refused


def setup_cars(ar, k, roles=ROLES):
    """Cars per road (A, B, C, L and D), physics applied; absent roles are parked per group."""
    actors = []
    for r in range(k):
        actors.append({role: ar.spawn(r, LANES[role], PARK_X[role]) for role in roles})
    ar.put_physics([a for d in actors for a in d.values()])
    return [{role: AR.Car(ar, a) for role, a in d.items()} for d in actors]


def _rng(job, *tag):
    key = "|".join(str(x) for x in (job["scene"], job["d0"], job["dv"], job.get("hf"), job.get("gl"),
                                    job["pa"]["name"], job["pb"]["name"], job["seed"], *tag))
    return random.Random(key)


class Group:
    def __init__(self, ar, road, cars, job, args):
        self.ar, self.road, self.job, self.args = ar, road, job, args
        self.roles = job["roles"]
        self.cars = {r: cars[r] for r in self.roles}
        self.absent = {r: cars[r] for r in cars if r not in self.roles}
        self.y = {r: ar.lane_y(road, LANES[r])[0] for r in cars}
        self.ln = cars["B"].length
        jit = args.reaction_jitter
        cap = C1.TYPES["car"]
        self.react = {r: cap.reaction + _rng(job, "react", r).uniform(-jit, jit) for r in "ABCD"}
        self.obs = {}
        pairs = [("A", "B"), ("A", "C"), ("A", "L"), ("B", "A"), ("C", "B"), ("C", "A"),
                 ("A", "D"), ("C", "D"), ("D", "A"), ("D", "B"), ("D", "C")]
        for v, u in pairs:
            if v in self.roles and u in self.roles:
                self.obs[v + u] = AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, _rng(job, "obs", v, u))
        self.phase = "warmup"
        self.rec = None

    # --- geometry
    def offsets(self, t):
        """Each car's centre offset from the ghost (B's centre) and its rate, at warm-up time t."""
        j, W = self.job, AR.WARMUP
        la = lb = lc = ll = self.ln  # every car is the spec's Model 3
        rA = j["d0"] - j["dv"] * max(0.0, W - t)
        off = {"B": (0.0, 0.0), "A": (lb / 2 + rA + la / 2, j["dv"] if t < W else 0.0)}
        if j["scene"] in ("a", "a4"):
            off["C"] = (-(lb / 2 + self.gap_c0 + lc / 2), 0.0)
            off["D"] = (off["C"][0] - (lc / 2 + self.gap_c0 + self.ln / 2), 0.0)
        elif j["scene"] == "b":
            g = j["gl"] + j["dv"] * max(0.0, W - t)
            gr = -j["dv"] if t < W else 0.0
            if g < 8.0:
                g, gr = 8.0, 0.0
            off["L"] = (off["A"][0] + la / 2 + g + ll / 2, off["A"][1] + gr)
        return off

    def place(self):
        j = self.job
        self.gap_c0 = C1.S0 + j["vb"] * j.get("hf", 1.0)
        off = {r: o for r, o in self.offsets(0.0).items() if r in self.cars}
        x0 = 30.0 - min(o[0] for o in off.values()) + 5.0
        for r, c in self.cars.items():
            loc = c.a.get_transform().location
            c.park(carla.Transform(carla.Location(x=x0 + off[r][0], y=self.y[r], z=loc.z + 0.3), carla.Rotation(yaw=0)))
        for r, c in self.absent.items():
            loc = c.a.get_transform().location
            c.park(carla.Transform(carla.Location(x=PARK_X[r], y=self.y[r], z=loc.z + 0.3), carla.Rotation(yaw=0)))
        self.xg, self.vg = x0, 0.0
        self.tw = self.ar.t

    def hold_absent(self):
        for c in self.absent.values():
            self.ar.pending.append(carla.command.ApplyVehicleControl(c.a.id, carla.VehicleControl(brake=1.0, hand_brake=True)))

    def step_warmup(self):
        t = self.ar.t - self.tw
        vb = self.job["vb"]
        ag = max(-3.0, min(3.0, 1.5 * (vb - self.vg)))
        off = self.offsets(t)
        for r, c in self.cars.items():
            x, _, _, v = c.state()
            o, orate = off[r]
            a = ag + 0.6 * (self.xg + o - x) + 1.8 * (self.vg + orate - v)
            c.drive(max(-4.0, min(3.0, a)), self.y[r])
        self.hold_absent()
        self.xg += self.vg * AR.DT + 0.5 * ag * AR.DT ** 2
        self.vg += ag * AR.DT
        if t >= AR.WARMUP:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.ar.t
        st = {r: c.state() for r, c in self.cars.items()}
        self.v0 = {r: s[3] for r, s in st.items()}
        self.ctx_real = {}
        cs = self.cars
        if "A" in cs and "B" in cs:
            self.ctx_real["d0_real"] = round((st["A"][0] - cs["A"].length / 2) - (st["B"][0] + cs["B"].length / 2), 2)
            self.ctx_real["dv_real"] = round(st["A"][3] - st["B"][3], 2)
        if "B" in cs and "C" in cs:
            gap = (st["B"][0] - cs["B"].length / 2) - (st["C"][0] + cs["C"].length / 2)
            self.ctx_real["hf_real"] = round((gap - C1.S0) / max(st["C"][3], 0.1), 2)
        if "C" in cs and "D" in cs:
            gap = (st["C"][0] - cs["C"].length / 2) - (st["D"][0] + cs["D"].length / 2)
            self.ctx_real["hfd_real"] = round((gap - C1.S0) / max(st["D"][3], 0.1), 2)
        if "A" in cs and "L" in cs:
            self.ctx_real["gl_real"] = round((st["L"][0] - cs["L"].length / 2) - (st["A"][0] + cs["A"].length / 2), 2)
        for r in ROLES:
            if r in st:
                self.ctx_real[f"v{r}_real"] = round(st[r][3], 2)
        self.pa, self.pb = self.job["pa"], self.job["pb"]
        self.decide = None
        self.dropping = False
        self.queue = [r for r in QUEUE if r in cs]  # the target lane's cars, front to back
        self.slot = self.queue[0] if self.queue else None  # A's target-lane lead when dropping
        self.dec_state = None  # A's context when it decided (gap ahead of the queue's head, speeds)
        self.b_latched = False
        self.lat_seen = self.in_lane_at = self.b_brake_seen = None
        self.lead_a_at = {}  # follower -> when it first saw A in its lane
        self.contact = {r: False for r in cs}
        self.peak = {r: 0.0 for r in cs}
        self.vhist = {r: deque() for r in cs}
        self.meas = {a + b: measures.Measures(b_max=(C1.TYPES["car"].b_max,) * 2)
                     for i, a in enumerate(ROLES) for b in ROLES[i + 1:] if a in cs and b in cs}
        self.trace = []
        self.t_trace = -1.0
        self.yA0, self.yT = self.y["A"], self.y["B"]

    def lat(self):
        y = self.cars["A"].state()[1]
        return max(0.0, min(1.0, (self.yA0 - y) / (self.yA0 - self.yT)))

    def seen(self, v, u, t, x, lat, spd):
        return self.obs[v + u].push(t, x, lat, spd)

    def step_run(self):
        t = self.ar.t - self.t0
        cs = self.cars
        cap = C1.TYPES["car"]
        st = {r: c.state() for r, c in cs.items()}
        lat = self.lat() if "A" in cs else 0.0
        S = {}  # (viewer, target) -> (x, lat, v, acc) as seen
        for key, ob in self.obs.items():
            u = key[1]
            x, _, _, spd = st[u]
            s = ob.push(t, x, lat if u == "A" else 0.0, spd)
            S[key] = (s[1], s[2], s[3], s[4]) if s is not None else (x, lat if u == "A" else 0.0, spd, 0.0)
        acc = {}
        y_t = {r: self.y[r] for r in cs}
        # --- A
        if "A" in cs:
            A = cs["A"]
            xa, _, _, va = st["A"]
            la = A.length
            pa = self.pa
            tgt = "A" + self.queue[0] if self.queue else None
            if self.decide is None and not self.dropping:
                if tgt is None:
                    self.decide = t  # nobody in the target lane
                else:
                    xt, _, vt, at = S[tgt]
                    lt = cs[tgt[1]].length
                    gap_ahead = (xa - la / 2) - (xt + lt / 2)
                    need = pa["buffer"] + vt * pa["tau"] + max(0.0, vt ** 2 - va ** 2) / (2 * pa["a_rear"])
                    if at <= -C1.YIELD_SEEN and self.b_brake_seen is None:
                        self.b_brake_seen = t
                    if gap_ahead >= need:
                        self.decide = t
                    elif (pa["use_yield"] and self.b_brake_seen is not None and t >= self.b_brake_seen + self.react["A"]
                          and gap_ahead > -la / 2):
                        self.decide = t
                    elif t >= pa["patience"] + (C1.WATCH if pa["use_yield"] else 0.0):
                        self.dropping = True
            if self.dropping and self.decide is None:
                def slot_gaps(k):
                    """gap behind queue car k, the gap A wants there; whether the gap ahead of the
                    next queue car is short of A's check"""
                    q = self.queue[k]
                    xs, _, vs, _ = S["A" + q]
                    gb = (xs - cs[q].length / 2) - (xa + la / 2)
                    short = False
                    if k + 1 < len(self.queue):
                        n = self.queue[k + 1]
                        xn, _, vn, _ = S["A" + n]
                        gn = (xa - la / 2) - (xn + cs[n].length / 2)
                        short = gn < pa["buffer"] + vn * pa["tau"] + max(0.0, vn ** 2 - va ** 2) / (2 * pa["a_rear"])
                    return gb, C1.S0 + vs * C1.HEADWAY, short, vs
                k = self.queue.index(self.slot)
                gap_behind, want, rear_short, vs = slot_gaps(k)
                while gap_behind >= want and rear_short:
                    k += 1  # the slot behind this car is too short: drop behind the next one as well
                    self.slot = self.queue[k]
                    gap_behind, want, rear_short, vs = slot_gaps(k)
                if gap_behind >= want and not rear_short:
                    self.decide = t
                aa = -C1.A_WAIT_DECEL if (gap_behind < want or rear_short) and va > 0.5 * vs else 0.0
                if self.decide is not None:
                    aa = AR.follow(gap_behind, va, vs, cap, self.v0["A"])
            else:
                # behind a target-lane car while changing lanes: follow it (Pair's rule)
                ahead = [(S["A" + r][0], r) for r in self.queue if S["A" + r][0] > xa]
                if lat > 0.05 and ahead:
                    xs, r = min(ahead)
                    vs = S["A" + r][2]
                    aa = AR.follow((xs - cs[r].length / 2) - (xa + la / 2), va, vs, cap, self.v0["A"])
                else:
                    aa = max(-1.0, min(cap.a_max, 1.5 * (self.v0["A"] - va)))
            if "L" in cs and lat < C1.LANE_LINE:
                xl, _, vl, _ = S["AL"]
                gl = (xl - cs["L"].length / 2) - (xa + la / 2)
                if gl > 0:
                    aa = min(aa, C1.follow_accel(gl, va, vl, cap, V_FREE))
            t_prep = pa["t_prep"]
            if self.decide is not None and self.dec_state is None:
                h = self.queue[0] if self.queue else None
                self.dec_state = {"dec_t": round(t, 2), "dec_vA": round(va, 3), "dec_slot": self.slot if self.dropping else "ahead"}
                if h is not None:
                    self.dec_state["dec_d0"] = round((xa - la / 2) - (st[h][0] + cs[h].length / 2), 2)
                    self.dec_state["dec_dv"] = round(va - st[h][3], 3)
            if self.decide is not None and t >= self.decide + t_prep:
                s = min(1.0, (t - self.decide - t_prep) / cap.t_lc)
                y_t["A"] = self.yA0 + (self.yT - self.yA0) * 0.5 * (1 - math.cos(math.pi * s))
            acc["A"] = aa
        # --- B
        if "B" in cs:
            B = cs["B"]
            xb, _, _, vb = st["B"]
            lb = B.length
            ab = max(-1.0, min(cap.a_max, 1.5 * (self.v0["B"] - vb)))
            if "A" in cs:
                xa_s, lat_s, va_s, _ = S["BA"]
                la = cs["A"].length
                if lat_s > 0.08 and self.lat_seen is None:
                    self.lat_seen = t
                if lat_s >= C1.LANE_LINE and xa_s > xb and self.in_lane_at is None:
                    self.in_lane_at = t
                gap_b = (xa_s - la / 2) - (xb + lb / 2)
                a_ahead = xa_s > xb - lb
                pb = self.pb
                pre = t >= self.react["B"] and a_ahead and lat_s < C1.LANE_LINE and gap_b < pb["pre_band"]
                react = (self.lat_seen is not None and t >= self.lat_seen + self.react["B"] and a_ahead
                         and gap_b < pb["react_gap"])
                self.b_latched = self.b_latched or ((pre or react) and a_ahead)
                if self.b_latched and a_ahead:
                    ab = -pb["b_y"] if gap_b < C1.S0 + vb * C1.HEADWAY else AR.follow(gap_b, vb, va_s, cap, self.v0["B"])
                if self.in_lane_at is not None and t >= self.in_lane_at + self.react["B"] and xa_s > xb:
                    ab = min(ab, AR.follow(gap_b, vb, va_s, cap, self.v0["B"]))
            acc["B"] = ab
        # --- the followers, C and D: the nearest car ahead in lane -1
        for f in ("C", "D"):
            if f not in cs:
                continue
            xc, _, _, vc = st[f]
            lc = cs[f].length
            ac = max(-1.0, min(cap.a_max, 1.5 * (self.v0[f] - vc)))
            leads = []
            for q in self.queue[:self.queue.index(f)]:
                xq_s, _, vq_s, _ = S[f + q]
                leads.append(((xq_s - cs[q].length / 2) - (xc + lc / 2), vq_s))
            if "A" in cs:
                xa_s, lat_s, va_s, _ = S[f + "A"]
                if lat_s >= C1.LANE_LINE and xa_s > xc and f not in self.lead_a_at:
                    self.lead_a_at[f] = t
                if f in self.lead_a_at and t >= self.lead_a_at[f] + self.react[f] and xa_s > xc:
                    leads.append(((xa_s - cs["A"].length / 2) - (xc + lc / 2), va_s))
            leads = [g for g in leads if g[0] > -lc]
            if leads:
                g, vl = min(leads)
                ac = min(ac, C1.follow_accel(g, vc, vl, cap, V_FREE))
            acc[f] = ac
        # --- L
        if "L" in cs:
            acc["L"] = max(-1.0, min(cap.a_max, 1.5 * (self.v0["L"] - st["L"][3])))
        for r, c in cs.items():
            c.drive(acc[r], y_t[r])
        self.hold_absent()
        # --- truth
        for r in cs:
            h = self.vhist[r]
            h.append((t, st[r][3]))
            while h and t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[r] = max(self.peak[r], (h[0][1] - st[r][3]) / AR.PEAK_WINDOW)
        poses = {r: c.pose() for r, c in cs.items()}
        for p, m in self.meas.items():
            m.push_poses(t, poses[p[0]], poses[p[1]])
            if m.clear <= 0.0:
                self.contact[p[0]] = self.contact[p[1]] = True
        if t - self.t_trace >= TRACE_DT - 1e-6:
            self.t_trace = t
            self.trace.append([round(t, 2)] + [[r] + [round(x, 3) for x in poses[r][:5]] for r in cs])
        if max(st[r][0] for r in cs) > X_MAX:
            raise RuntimeError(f"road {self.road}: a car passed x {X_MAX} m, near the parked cars ({self.job})")
        if any(self.contact.values()) or t >= T_RUN - 1e-9:
            self.finish(t, st, lat)

    def finish(self, t, st, lat):
        cs = self.cars
        cap = C1.TYPES["car"]
        s, s_h, latp, rho = {}, {}, {r: 0.0 for r in cs}, {}
        for p, m in self.meas.items():
            m.pet = m._pet()
            rho[p] = measures.rho(m.ttc2d, m.thw, m.pet, m.clear)
            for k, r in enumerate(p):
                s[r], s_h[r] = m.s[k], m.s_h[k]
                latp[r] = max(latp[r], m.lat_peak[k])
        got_in = lat >= 0.9 if "A" in cs else None
        rec = {**{k: v for k, v in self.job.items() if k not in ("pa", "pb")}, "a": self.pa["name"], "b": self.pb["name"],
               "road": self.road, **self.ctx_real, "t_run": round(t, 2)}
        for r in cs:
            lost = measures.lost_progress(self.v0[r], s_h[r], HORIZON)
            if self.contact[r] or (r == "A" and not got_in):
                lost = 1.0
            risk = max([rho[p] for p in rho if r in p] or [0.0])
            u = measures.utility(lost, risk, self.peak[r], cap.b_max, latp[r], self.contact[r])
            rec.update({f"u{r}": round(u, 4), f"lost{r}": round(lost, 4), f"rho{r}": round(risk, 4),
                        f"peak{r}": round(self.peak[r], 3), f"lat{r}": round(latp[r], 3),
                        f"contact{r}": int(self.contact[r]), f"v0{r}": round(self.v0[r], 3), f"s{r}": round(s[r], 2)})
        for p, m in self.meas.items():
            rr = m.result()
            rec.update({f"rho_{p}": round(rho[p], 4), f"ttc2d_{p}": rr["ttc2d"], f"thw_{p}": rr["thw"],
                        f"pet_{p}": rr["pet"], f"clear_{p}": rr["clear"]})
        if "A" in cs:
            xa = st["A"][0]
            rec["got_in"] = int(got_in)
            rec["decide"] = None if self.decide is None else round(self.decide, 2)
            rec["dropped"] = int(self.dropping)
            rec.update(self.dec_state or {})
            # where A ended in lane -1: ahead of every target-lane car, between two, behind all;
            # a_slot: how many queue cars ended ahead of A
            pos, n_ahead = "none", None
            if got_in:
                ahead_of_a = [r for r in self.queue if st[r][0] > xa]
                n_ahead = len(ahead_of_a)
                pos = "ahead" if not n_ahead else ("behind" if n_ahead == len(self.queue) else "between")
            rec["a_pos"], rec["a_slot"] = pos, n_ahead
        if self.job.get("trace"):
            rec["trace"] = self.trace
        self.rec = rec
        self.phase = "done"


def run_groups(ar, cars, work, args, log=None):
    """Run jobs K at a time (one group per road); one row per job, in job order."""
    rows = []
    queue = deque(work)
    import time
    t_run = time.monotonic()
    while queue:
        groups = []
        for r in range(len(cars)):
            if not queue:
                break
            g = Group(ar, r, cars[r], queue.popleft(), args)
            g.place()
            groups.append(g)
        for _ in range(3):
            for g in groups:
                for c in list(g.cars.values()) + list(g.absent.values()):
                    c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            ar.tick()
        for g in groups:
            g.tw = ar.t
        while any(g.phase != "done" for g in groups):
            for g in groups:
                if g.phase == "warmup":
                    g.step_warmup()
                elif g.phase == "run":
                    g.step_run()
                else:
                    for r, c in g.cars.items():
                        c.drive(-2.0, c.state()[1] if r == "A" else g.y[r])
                    g.hold_absent()
            ar.tick()
        rows += [g.rec for g in groups]
        if log:
            log(f"  {len(rows)}/{len(work)} runs, {time.monotonic() - t_run:.0f} s")
    return rows
